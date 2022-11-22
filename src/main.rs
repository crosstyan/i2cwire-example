use core::panic;
use linux_embedded_hal::{Delay, I2cdev};
use log::{debug, error, info, warn};
use mpu6050::*;
use nalgebra::{Vector3, Vector4};
use parking_lot::Mutex;
use rumqttc::Event as MqEv;
use rumqttc::Packet as MqPkt;
use rumqttc::{AsyncClient, MqttOptions, QoS};
use serde::ser::SerializeStruct;
use serde_derive::{Deserialize, Serialize};
use serde_json::json;
use std::collections::HashMap;
use std::env;
use std::ops::Not;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use tokio::time::Duration;
use tokio::{task, time};
// awkwardly import the `wire`
// TODO: find a way to share code instead of copy-pasting
use wire::IoReader;
pub mod wire;

// TODO: switch to clap
// https://github.com/clap-rs/clap/issues/254
// https://lib.rs/crates/clap_conf
#[derive(Debug, Serialize, Deserialize, Clone)]
struct AppConfig {
    device_id: String,
    mqtt_host: String,
    mqtt_id: String,
    mqtt_port: u16,
    enable_i2c: bool,
    i2c_bus: String,
    enable_mpu6050: bool,
    /// the polling rate of mpu6050
    mpu6050_delay_ms: u64,
    enable_bmp: bool,
    /// if true using bmp280, otherwise bmp180.
    /// bme280 is not impl yet
    use_bme280: bool,
    enable_iowire: bool,
    i2c_delay_ms: u64,
    mqtt_poll_deadline: u64,
    max_error_count: usize,
}

impl Default for AppConfig {
    fn default() -> AppConfig {
        AppConfig {
            device_id: "c8bc0011".to_string(),
            mqtt_host: "localhost".into(),
            mqtt_id: "test".into(),
            mqtt_port: 1883,
            enable_i2c: false,
            i2c_bus: "/dev/i2c-1".into(),
            enable_bmp: false,
            enable_iowire: false,
            enable_mpu6050: false,
            mpu6050_delay_ms: 100,
            use_bme280: false,
            i2c_delay_ms: 500,
            mqtt_poll_deadline: 1000,
            max_error_count: 3,
        }
    }
}

// TODO: generic
#[derive(Debug, Clone)]
pub struct SerdeVec3(Vector3<f32>);
#[derive(Debug, Clone, Default, Serialize)]
pub struct MPUData {
    pub temperature: f32,
    pub gyro: SerdeVec3,
    pub acc: SerdeVec3,
}

impl Default for SerdeVec3 {
    fn default() -> Self {
        let v: Vector3<f32> = Vector3::new(0f32, 0f32, 0f32);
        SerdeVec3(v)
    }
}

impl From<Vector3<f32>> for SerdeVec3 {
    fn from(v: Vector3<f32>) -> Self {
        SerdeVec3(v)
    }
}

impl serde::Serialize for SerdeVec3 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let v = self.0;
        let mut state = serializer.serialize_struct("SerdeVec3", 3)?;
        state.serialize_field("x", &v.x)?;
        state.serialize_field("y", &v.y)?;
        state.serialize_field("z", &v.z)?;
        state.end()
    }
}

#[derive(Debug, Serialize, Deserialize, Clone, Default)]
pub struct BMPData {
    pub temperature: f32,
    pub pressure: f32,
}

pub type IOWireData = HashMap<u8, u8>;

pub enum I2CMessage {
    BMP(BMPData),
    MPU(MPUData),
    IOWire(IOWireData),
}

#[tokio::main]
async fn main() {
    /**** Ctrl+C *****/
    let run_flag = Arc::new(AtomicBool::new(true));
    let send_pic_flag = Arc::new(AtomicBool::new(true));
    let r = run_flag.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::Relaxed);
        print!("\n");
        info!("Ctrl+C received, exiting...");
    })
    .expect("Error setting Ctrl-C handler");
    /**** Ctrl+C Ends *****/

    /**** LOG Config *****/
    // Set the default log level to debug if RUST_LOG is not set
    if env::var("RUST_LOG").is_err() {
        env::set_var("RUST_LOG", "debug")
    }

    let mut log_builder = env_logger::Builder::from_default_env();
    log_builder.target(env_logger::Target::Stdout);
    log_builder.init();
    /**** LOG Config Ends *****/

    /**** Confy *****/
    let app_name = "i2ctest";
    let res: Result<AppConfig, _> = confy::load(app_name, None);
    let path = confy::get_configuration_file_path(&app_name, None).unwrap();
    info!("Reading config file from {:#?}", path);
    let cfg: AppConfig = match res {
        Ok(cfg) => cfg,
        Err(e) => {
            error!(
                "Parse Config Error: Please check your configuration file at {:#?}",
                path
            );
            panic!("{}", e)
        }
    };
    dbg!(&cfg);
    /**** Confy Ends*****/

    /**** MQTT init ****/
    let mut mqttoptions = MqttOptions::new(&cfg.mqtt_id, &cfg.mqtt_host, cfg.mqtt_port);
    // in bytes
    let max_packet_size = 40960;
    info!("Max packet size default {}", mqttoptions.max_packet_size());
    mqttoptions.set_max_packet_size(max_packet_size, max_packet_size);
    info!(
        "Max packet size is set to {}",
        mqttoptions.max_packet_size()
    );
    mqttoptions.set_keep_alive(Duration::from_secs(5));
    mqttoptions.set_clean_session(true);
    let (client, mut eventloop) = AsyncClient::new(mqttoptions, 10);
    // Don't afraid of panic. systemd will restart it.
    if let Err(e) = subscribe_device(&client, &cfg.device_id).await {
        error!("MQTT Subscribe Error: {}", e);
        panic!("{}", e)
    }
    /**** MQTT init end ****/

    /*********** I2C Stuff ***************/
    let mut wire_reader: Option<IoReader<I2cdev>> = None;
    let mut bmp: Option<bmp180::BMP180BarometerThermometer<I2cdev>> = None;
    // let mut bme: Option<bme280::i2c::BME280<I2cdev>> = None;
    let mut mpu: Option<Mpu6050<I2cdev>> = None;
    let (tx, mut rx) = tokio::sync::mpsc::channel::<I2CMessage>(10);
    if cfg.enable_i2c {
        if cfg.enable_bmp {
            if cfg.use_bme280.not() {
                let i2c_bmp = I2cdev::new(&cfg.i2c_bus).expect("Failed to open i2c device for bmp");
                // TODO: use tokio delay instead of std::thread::sleep
                // I don't think it's possible with synchronous API
                // tokio would create a new thread when possible so
                // I guess it won't be a problem
                let delay = Box::new(Delay);
                let bmp180 = bmp180::BMP180BarometerThermometer::new(
                    i2c_bmp,
                    delay,
                    bmp180::BMP180PressureMode::BMP180Standard,
                );
                bmp = Some(bmp180);
                info!("BMP180 initialized");
            } else {
                // TODO BME280
                panic!("BMP280 is not supported yet");
            }
        }
        if cfg.enable_mpu6050 {
            let i2c_mpu = I2cdev::new(&cfg.i2c_bus).expect("Failed to open i2c device for mpu");
            let mut mpu6050 = Mpu6050::new(i2c_mpu);
            let mut delay = Delay;
            mpu6050.init(&mut delay).unwrap();
            mpu = Some(mpu6050);
            info!("MPU6050 initialized");
        }
        if cfg.enable_iowire {
            let i2c = I2cdev::new(&cfg.i2c_bus).expect("Failed to open i2c device for io reader");
            let reader = IoReader::new(i2c);
            wire_reader = Some(reader);
            info!("IOWire initialized");
        }

        let delay_ms = cfg.i2c_delay_ms;
        let f = run_flag.clone();
        // https://stackoverflow.com/questions/69638710/when-should-you-use-tokiojoin-over-tokiospawn
        let f_mpu = run_flag.clone();
        let tx_mpu = tx.clone();
        let mpu_delay_ms = cfg.mpu6050_delay_ms;
        if cfg.enable_mpu6050 {
            tokio::spawn(async move {
                while f_mpu.load(Ordering::Relaxed) {
                    match mpu.as_mut() {
                        Some(mpu) => {
                            let mut data = MPUData::default();
                            // get sensor temp
                            if let Ok(temp) = mpu.get_temp() {
                                data.temperature = temp;
                            }

                            // get gyro data, scaled with sensitivity
                            if let Ok(gyro) = mpu.get_gyro() {
                                // perhaps two different versions of crate `nalgebra` are being used?
                                // use certain version of nalgebra!
                                // https://github.com/sebcrozet/kiss3d/issues/66
                                // The broader issue here is that nalgebra types are
                                // always incompatible between versions. They tend
                                // to make a lot of breaking changes between minor
                                // versions in general.
                                data.gyro = gyro.into();
                            }

                            // get accelerometer data, scaled with sensitivity
                            if let Ok(acc) = mpu.get_acc() {
                                data.acc = acc.into();
                            }
                            // not sure whether blocking send or spawn a tokio task
                            if let Err(e) = tx_mpu.send(I2CMessage::MPU(data)).await {
                                error!("mpu tx error: {}", e);
                            };
                        }
                        None => {
                            error!("mpu is not found");
                        }
                    }
                    tokio::time::sleep(Duration::from_millis(mpu_delay_ms)).await;
                }
            });
        }
        tokio::spawn(async move {
            while f.load(Ordering::Relaxed) {
                // query/polling instead of breaking down the whole pipeline
                match bmp.as_mut() {
                    Some(bmp180) => {
                        let mut data = BMPData::default();
                        let pressure_in_hpa: f32 = bmp180.pressure_hpa();
                        let pressure_temp_celsius: f32 = bmp180.temperature_celsius();
                        data.pressure = pressure_in_hpa;
                        data.temperature = pressure_temp_celsius;
                        let tx = tx.clone();
                        if let Err(e) = tx.send(I2CMessage::BMP(data)).await {
                            error!("bmp tx error: {}", e);
                        };
                    }
                    None => {
                        if cfg.enable_bmp {
                            error!("bmp is not found");
                        }
                    }
                }
                match wire_reader.as_mut() {
                    Some(reader) => {
                        let res = reader.update();
                        if let Err(e) = res {
                            debug!("Error: {}", e);
                        } else {
                            let data: IOWireData = reader.ios().clone();
                            if let Err(e) = tx.send(I2CMessage::IOWire(data)).await {
                                error!("iowire tx error: {}", e);
                            };
                        }
                    }
                    None => {
                        if cfg.enable_iowire {
                            error!("iowire is not found");
                        }
                    }
                }
                tokio::time::sleep(Duration::from_millis(delay_ms)).await;
            }
        });
    }
    /*********** I2C Stuff Ends ***************/

    let c = client.clone();
    let f = run_flag.clone();
    let device_id = cfg.device_id.clone();
    tokio::spawn(async move {
        // TODO: make a function to avoid code duplication
        while f.load(Ordering::Relaxed) {
            if let Some(msg) = rx.recv().await {
                match msg {
                    I2CMessage::MPU(data) => {
                        let topic = format!("/device/{}/mpu", device_id);
                        let payload = json!({
                            "device_id": device_id,
                            "type": "mpu",
                            "data": data,
                        })
                        .to_string();
                        let _ = c.publish(topic, QoS::AtLeastOnce, false, payload).await;
                    }
                    I2CMessage::BMP(data) => {
                        let topic = format!("/device/{}/bmp", device_id);
                        let payload = json!({
                            "device_id": device_id,
                            "type": "bmp",
                            "data": data,
                        })
                        .to_string();
                        let _ = c.publish(topic, QoS::AtLeastOnce, false, payload).await;
                    }
                    I2CMessage::IOWire(data) => {
                        let topic = format!("/device/{}/iowire", device_id);
                        let paylod = json!({
                            "device_id": device_id,
                            "type": "iowire",
                            "data": data,
                        })
                        .to_string();
                        let _ = c.publish(topic, QoS::AtLeastOnce, false, paylod).await;
                    }
                }
            }
        }
    });

    // TODO: BMP will lost data/weight after a while
    //       detect this and send a message to BMP to reset
    //       the BMP.
    let max_error_count = cfg.max_error_count;
    let mut error_counter = 0;
    while run_flag.load(Ordering::Relaxed) {
        // TODO: fix this when we have a connection error
        // The control will not resopnd to the signal after the connection is lost
        let poll = eventloop.poll();
        let timeout = tokio::time::timeout(Duration::from_millis(cfg.mqtt_poll_deadline), poll);
        match timeout.await {
            Ok(res) => {
                // once connection resumes, reset the counter
                match res {
                    Ok(ev) => {
                        if let MqEv::Incoming(pkt) = ev {
                            match pkt {
                                MqPkt::Publish(publish) => {
                                    let topic = publish.topic;
                                    let path_list = topic.split('/').collect::<Vec<&str>>();
                                    // we only subscribe our device id
                                    if path_list.contains(&"command") {
                                        let payload =
                                            String::from_utf8(publish.payload.to_vec()).unwrap();
                                        match payload.as_str() {
                                            "send" => {
                                                send_pic_flag.store(true, Ordering::Relaxed);
                                            }
                                            "stop" => {
                                                send_pic_flag.store(false, Ordering::Relaxed);
                                            }
                                            &_ => {
                                                error!("unknown command: {}", payload);
                                            }
                                        }
                                    }
                                }
                                MqPkt::ConnAck(_) => {
                                    // resubscribe the topic after reconnect
                                    // why the subscription is not persistent? no idea
                                    info!("try to resubscribe!");
                                    if let Err(e) = subscribe_device(&client, &cfg.device_id).await
                                    {
                                        // usually it's not possible to fail after the connection is established
                                        // I won't handle this shit
                                        error!("resubscribe failed: {}", e);
                                    }
                                }
                                MqPkt::PingResp => {
                                    // set the counter to 0 and continue
                                    error_counter = 0;
                                }
                                MqPkt::PingReq => {
                                    // Don't care
                                }
                                _ => {
                                    // Don't care
                                }
                            }
                        } else {
                            // Don't care outgoing event
                        }
                    }
                    Err(e) => {
                        error_counter += 1;
                        if error_counter > max_error_count {
                            panic!("error too many times, exit");
                        }
                        error!("connection error: {}", e);
                    }
                }
            }
            Err(e) => {
                error!("timeout error: {}", e);
                // use counter to avoid dead loop
                // use with systemd service
                // Type=simple
                // Restart=always
                // RestartSec=10
                // StartLimitIntervalSec=60
                // StartLimitBurst=3
                error_counter += 1;
                if error_counter > max_error_count {
                    // systemd should restart the program
                    panic!("error too many times, exit");
                }
            }
        }
    }

    std::process::exit(0);
}

/// subscribe all the actions of the device
async fn subscribe_device(
    client: &AsyncClient,
    device_id: &str,
) -> Result<(), rumqttc::ClientError> {
    let device_topic = format!("/device/{}/#", device_id);
    client.subscribe(&device_topic, QoS::AtMostOnce).await
}

use std::thread::sleep;
use i2cdev::linux::LinuxI2CError;
use linux_embedded_hal::{Delay, I2cdev};
use mpu6050::*;
mod io_reader;
// use std::sync::{Arc, Mutex};

fn main() -> Result<(), Mpu6050Error<LinuxI2CError>> {
    let i2c = I2cdev::new("/dev/i2c-1").expect("Failed to open i2c device for io reader");
    let mut reader = io_reader::IoReader::new(i2c);

    let i2c_bmp = I2cdev::new("/dev/i2c-1").expect("Failed to open i2c device for bmp");
    let i2c_mpu = I2cdev::new("/dev/i2c-1").expect("Failed to open i2c device for mpu");
    let delay = Box::new(Delay);
    let mut bmp180 = bmp180::BMP180BarometerThermometer::new(
        i2c_bmp,
        delay,
        bmp180::BMP180PressureMode::BMP180Standard,
    );

    let mut delay = Delay;
    let mut mpu = Mpu6050::new(i2c_mpu);

    mpu.init(&mut delay)?;

    loop {
        // get roll and pitch estimate
        let acc = mpu.get_acc_angles()?;
        println!("r/p: {:?}", acc);

        // get sensor temp
        let temp = mpu.get_temp()?;
        println!("temp from mpu: {:?}c", temp);

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro()?;
        println!("gyro: {:?}", gyro);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc()?;
        println!("acc: {:?}", acc);

        // bmp180
        let pressure_in_hpa: f32 = bmp180.pressure_hpa();
        let pressure_temp_celsius: f32 = bmp180.temperature_celsius();
        println!("pressure: {:?}hpa", pressure_in_hpa);
        println!("temp from bmp: {:?}c", pressure_temp_celsius);

        let res = reader.update();
        if let Err(e) = res {
            println!("Error: {}", e);
        }
        println!("{:?}", reader.ios());
    }
}

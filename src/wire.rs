use anyhow::{bail, Error};
use bytes::{BufMut, Bytes, BytesMut};
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};
use std::collections::HashMap;

pub const START: u8 = 0x02;
pub const END: u8 = 0x04;
pub const DEFAULT_SLAVE_ADDR: u8 = 0x20;

// a fixed format of request for WirePacket
// See https://github.com/gutierrezps/ESP32_I2C_Slave/blob/master/README_old.md
pub const REQUEST: [u8; 4] = [START, 0x04, 0x00, END];

// See https://github.com/gutierrezps/ESP32_I2C_Slave/blob/master/README_old.md
pub struct IoReader<I> {
    i2c: I,
    slave_addr: u8,
    // IO pin: count of reading
    ios: HashMap<u8, u8>,
    buf: [u8; 64],
}

impl<I, E> IoReader<I>
where
    I: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I) -> Self {
        IoReader {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            ios: HashMap::new(),
            buf: [0; 64],
        }
    }

    pub fn new_with_addr(i2c: I, slave_addr: u8) -> Self {
        IoReader {
            i2c,
            slave_addr,
            ios: HashMap::new(),
            buf: [0; 64],
        }
    }

    /// Reads series of bytes into `buf` from I2C.
    /// 
    /// Side effect: `buf` is updated
    fn read_bytes(&mut self) -> Result<(), E> {
        let res = self
            .i2c
            .write_read(self.slave_addr, &REQUEST, &mut self.buf);
        res
    }

    /// Updates the counts of IOs.
    /// The updated counts are stored in the `ios` field.
    /// equivalent to `read_bytes` then `data_to_hashmap`.
    /// 
    /// Side effect: the `ios` and `buf` field is updated.
    pub fn update(&mut self) -> Result<(), anyhow::Error> {
        let res = self.read_bytes();
        if res.is_err() {
            bail!("Error reading from I2C slave");
        }
        self.data_to_hashmap()
    }

    /// Read the `buf` field and update the `ios` field.
    /// 
    /// Side effect: the `ios` field is updated.
    fn data_to_hashmap(&mut self) -> Result<(), anyhow::Error> {
        let bytes = BytesMut::from(&self.buf[..]);
        let decoded = decode(&bytes)?;
        for i in 0..decoded.len() {
            // the decoded data should be in pairs
            if i % 2 == 0 {
                // original data is count, IO_PIN
                // no idea why IO_PIN is reserved
                // https://stackoverflow.com/questions/67376209/whats-the-best-practice-of-insert-or-update-operation-in-rusts-hashmap
                let count = decoded[i];
                let pin = decoded[i + 1];
                let entry = self.ios.entry(pin);
                entry.and_modify(|e| *e = count).or_insert(count);
            }
        }
        Ok(())
    }

    pub fn get_io_count(&mut self, io_pin: u8) -> Option<u8> {
        self.ios.get(&io_pin).cloned()
    }

    /// Returns a map of IO pin to count.
    /// `ios` mean input/output (s)
    pub fn ios(&self) -> &HashMap<u8, u8> {
        &self.ios
    }
}

/// pure function to get a reference of data from the raw bytes wrapped in
/// `WirePacker` format.
pub fn decode(data: &BytesMut) -> Result<&[u8], anyhow::Error> {
    // START LEN DATA CRC8 END
    let (s, rest) = data.split_first().unwrap();
    if (*s) != START {
        bail!("START not found, got {:X}, bytes {:?}", s, &data);
    }
    // l is the length of total data, including START, LEN, CRC8, END, i.e. payload
    let (l, rest) = rest.split_first().unwrap();
    let len = *l as usize - 4; // -4 for START, LEN, CRC8, END
    let (d, rest) = rest.split_at(len as usize);
    // TODO: actually check CRC8
    // Hasn't check CRC8
    let (_c, rest) = rest.split_first().unwrap();
    let (e, _) = rest.split_first().unwrap();
    if (*e) != END {
        bail!("END not found, got {:X}, bytes {:?}", e, &data);
    }
    Ok(d)
}
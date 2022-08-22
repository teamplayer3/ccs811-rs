//! This is a platform agnostic Rust driver for the CCS881 indoor air quality
//! sensor, based on the [`embedded-hal blocking i2c`] traits.
//!
//! [`embedded-hal blocking i2c`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Start App mode.
//! - Read all info available [eCO2, eTVOC, Status, ErrorId, RawData].
//! - Reset device.
//!
//! ## The device
//!
//! The CCS811 The CCS811 is an ultra-low power digital gas sensor solution
//! which integrates a metal oxide (MOX) gas sensor to detect a
//! wide range of Volatile Organic Compounds (VOCs) for indoor
//! air quality monitoring with a microcontroller unit (MCU), which
//! includes an Analog-to-Digital converter (ADC), and an I²C
//! interface.
//!
//! Datasheet:
//! - [ccs811](https://ams.com/documents/20143/36005/CCS811_DS000459_7-00.pdf/3cfdaea5-b602-fe28-1a14-18776b61a35a)
//!
//!
//! ## Usage examples (see also examples folder)
//!
//! To use this driver, import this crate and an `embedded-hal` implementation,
//! then instantiate the device.

#![deny(missing_docs, unsafe_code, warnings)]
#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::i2c::{Read, Write, WriteRead};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// IÂ²C bus error
    I2C(E),
    /// Invalid input data
    InvalidInputData,
}

/// Measurement Mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum DriveMode {
    /// Idle Mode
    DriveMode0Idle = 0b0u8,
    /// Mode 1 : 1 read per second    
    DriveMode1Sec = 0b1_0000u8,
    /// Mode 2 : read each 10 seconds
    DriveMode10Sec = 0b10_0000u8,
    /// Mode 3 : read each 60 seconds
    DriveMode60Sec = 0b11_0000u8,
    /// Mode 4 : read every 250 ms, expose only raw data
    DriveModeRawData = 0b100_0000u8,
}

/// Interrupt Mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum InterruptDataReady {
    /// Assert nINT when data is available in ALG_RESULT_DATA
    Enabled = 0b1000u8,
    /// Interrupt generation is disabled
    Disabled = 0b0u8,
}

/// Interrupt Mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum InterruptThreshold {
    /// Assert nINT if ALG_RESULT_DATA crosses thresholds
    Enabled = 0b100u8,
    /// Interrupt mode, if asserted, operates normally
    Disabled = 0x0,
}

/// ErrorId Codes
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ErrorIdCodes {
    /// WRITE_REG_INVALID
    ///
    /// "The CCS811 received an I2C write request addressed to this station but with
    /// invalid register address ID"
    WriteRegInvalid = 0b0u8,
    /// READ_REG_INVALID
    ///
    /// "The CCS811 received an I2C read request to a mailbox ID that is invalid"
    ReadRegInvalid = 0b10u8,
    /// MEASMODE_INVALID
    ///
    /// "The CCS811 received an I2C request to write an unsupported mode to
    /// MEAS_MODE"
    MeasModeInvalid = 0b100u8,
    /// MAX_RESISTANCE
    ///
    /// "The sensor resistance measurement has reached or exceeded the maximum
    /// range"
    MaxResistance = 0b1000u8,
    /// HEATER_FAULT
    ///
    /// "The Heater current in the CCS811 is not in range"
    HeaterFault = 0b1_0000u8,
    /// HEATER_SUPPLY
    ///
    /// "The Heater voltage is not being applied correctly"
    HeaterSupply = 0b10_0000u8,
}

/// Boot mode Register addresses
/// Taken from the CCS811 data sheet (Figure 25, p.26)
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum BootRegister {
    /// Status (RO)
    ///
    /// "Status register"
    Status = 0x00,
    /// HW_ID (RO)
    ///
    /// "Hardware ID. The value is 0x81"
    HwId = 0x20,
    /// HW Version (RO)
    ///
    /// "Hardware Version. The value is 0x1X"
    HwVersion = 0x21,
    /// FW_Boot_Version (RO)
    ///
    /// "Firmware Boot Version. The first 2 bytes contain the firmware version number for the boot code."
    FwBootVersion = 0x23,
    /// FW_App_Version (RO)
    ///
    /// "Firmware Application Version. The first 2 bytes contain the firmware version number for the application code"
    FwAppVersion = 0x24,
    /// Error Id
    ///
    /// "Error ID. When the status register reports an error its source is located in this register"
    ErrorId = 0xE0,
    /// APP_START (WO)
    ///
    /// "Status register"
    AppStart = 0xF4,
    /// SW_RESET
    ///
    /// "If the correct 4 bytes (0x11 0xE5 0x72 0x8A) are written to this register in a single
    /// sequence the device will reset and return to BOOT mode."
    SwReset = 0xFF,
}

/// Application mode Register addresses
/// Taken from the CCS811 data sheet (Figure 14, p.17)
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AppRegister {
    /// Status (RO)
    ///
    /// "Status register"
    Status = 0x00,
    /// Measurement Mode (RW)
    ///
    /// "Measurement mode and conditions register"
    MeasMode = 0x01,
    /// ALG_RESULT_DATA (RO)
    ///
    /// "Algorithm result. The most significant 2 bytes contain a
    /// ppm estimate of the equivalent CO 2 (eCO 2 ) level, and
    /// the next two bytes contain a ppb estimate of the total
    /// VOC level."
    AlgResultData = 0x02,
    /// RAW_DATA (RO)
    ///
    /// "Raw ADC data values for resistance and current source used."
    RawData = 0x03,
    /// ENV_DATA (WO)
    ///
    /// "Temperature and humidity data can be written to enable compensation"
    EnvData = 0x05,
    ///THRESHOLDS (WO)
    ///
    /// "Thresholds for operation when interrupts are only generated when eCO 2 ppm crosses a threshold"
    Thresholds = 0x10,
    /// BASELINE (R/W)
    ///
    /// "The encoded current baseline value can be read. A previously saved encoded baseline can be written."
    Baseline = 0x11,
    /// HW_ID (RO)
    ///
    /// "Hardware ID. The value is 0x81"
    HwId = 0x20,
    /// HW Version (RO)
    ///
    /// "Hardware Version. The value is 0x1X"
    HwVersion = 0x21,
    /// FW_Boot_Version (RO)
    ///
    /// "Firmware Boot Version. The first 2 bytes contain the firmware version number for the boot code."
    FwBootVersion = 0x23,
    /// FW_App_Version (RO)
    ///
    /// "Firmware Application Version. The first 2 bytes contain the firmware version number for the application code"
    FwAppVersion = 0x24,
    /// Internal_State (RO)
    ///
    /// "Internal Status register"
    InternalStatus = 0xA0,
    /// Error Id
    ///
    /// "Error ID. When the status register reports an error its source is located in this register"
    ErrorId = 0xE0,
    /// SW_RESET
    ///
    /// "If the correct 4 bytes (0x11 0xE5 0x72 0x8A) are written to this register in a single
    /// sequence the device will reset and return to BOOT mode."
    SwReset = 0xFF,
}

/// Result
#[derive(Debug, Default)]
pub struct SensorData {
    /// eCO
    pub e_co: u16,
    /// eTVoc
    pub e_tvoc: u16,
    /// Status
    pub status: u8,
    /// Error Id
    pub error_id: u8,
    /// Raw data
    pub raw: [u8; 2],
}

/// Ccs811 device driver.
#[derive(Debug, Default)]
pub struct Ccs811<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// i2c address
    address: u8,
}

impl<I2C, E> Ccs811<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E> + Read<Error = E>,
{
    /// Create new instance of the Ccs811 device.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Ccs811 { i2c, address }
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Enter App mode
    pub fn app_start(&mut self) -> Result<(), E> {
        self.i2c
            .write(self.address, &[BootRegister::AppStart as u8])
    }

    /// Set MEAS_MODE Register
    pub fn set_meas_mode(
        &mut self,
        drive_mode: DriveMode,
        interrupt: InterruptDataReady,
        threshold: InterruptThreshold,
    ) -> Result<(), E> {
        let mode: u8 = drive_mode as u8 | interrupt as u8 | threshold as u8;
        self.i2c
            .write(self.address, &[AppRegister::MeasMode as u8, mode])
    }

    /// Get MEAS_MODE Register
    pub fn get_meas_mode(&mut self, data: &mut [u8; 1]) -> Result<(), E> {
        self.i2c
            .write_read(self.address, &[AppRegister::MeasMode as u8], data)
    }

    /// Perform a SwReset, which brings firmware in Boot Mode.
    pub fn reset(&mut self) -> Result<(), E> {
        self.i2c.write(
            self.address,
            &[AppRegister::SwReset as u8, 0x11, 0xe5, 0x72, 0x8a],
        )
    }

    /// Returns [HwId;HwVersion]
    pub fn hw_info(&mut self) -> Result<[u8; 2], E> {
        let mut hw = [0u8; 2];
        self.i2c
            .write_read(self.address, &[AppRegister::HwId as u8], &mut hw[..1])?;
        self.i2c
            .write_read(self.address, &[AppRegister::HwVersion as u8], &mut hw[1..])?;
        Ok(hw)
    }

    /// Returns [FwBootVersion;FwAppVersion]
    pub fn fw_info(&mut self) -> Result<[u8; 4], E> {
        let mut fw = [0u8; 4];
        self.i2c.write_read(
            self.address,
            &[AppRegister::FwBootVersion as u8],
            &mut fw[0..2],
        )?;
        self.i2c.write_read(
            self.address,
            &[AppRegister::FwAppVersion as u8],
            &mut fw[2..],
        )?;
        Ok(fw)
    }

    /// Returns RAW_DATA
    pub fn raw_data(&mut self) -> Result<[u8; 2], E> {
        let mut data = [0u8; 2];
        self.i2c
            .write_read(self.address, &[AppRegister::RawData as u8], &mut data)?;
        Ok(data)
    }

    /// Set a previosuly retrieved baseline
    ///
    /// "A previously stored value may be written back to this two byte
    /// register and the Algorithms will use the new value in its
    /// calculations (until it adjusts it as part of its internal Automatic
    /// Baseline Correction). For more information, refer to ams
    /// application note AN000370: CCS811 Clean Air Baseline Save and
    /// Restore."
    pub fn set_baseline(&mut self, baseline: [u8; 2]) -> Result<(), E> {
        let data: [u8; 3] = [AppRegister::Baseline as u8, baseline[0], baseline[1]];
        self.i2c.write(self.address, &data)
    }

    /// Set environment data.
    ///
    /// "The internal algorithm uses these values (or default values if
    /// not set by the application) to compensate for changes in
    /// relative humidity and ambient temperature."
    pub fn set_environment(
        &mut self,
        humidity_percentage: f32,
        temperature_celsius: f32,
    ) -> Result<(), E> {
        assert!(
            humidity_percentage > 0.0
                || humidity_percentage < 100.0
                || temperature_celsius > 254.998_05,
            "wrong input"
        );
        let raw_humidity = get_raw_humidity(humidity_percentage);
        let raw_temp = get_raw_temperature(temperature_celsius);
        let raw = [
            AppRegister::EnvData as u8,
            raw_humidity.0,
            raw_humidity.1,
            raw_temp.0,
            raw_temp.1,
        ];
        self.i2c.write(self.address, &raw)
    }

    /// Retrieves Baseline
    pub fn get_baseline(&mut self, baseline: &mut [u8; 2]) -> Result<(), E> {
        self.i2c
            .write_read(self.address, &[AppRegister::Baseline as u8], baseline)
    }

    /// Retrieves Status register
    pub fn get_status(&mut self, status: &mut [u8; 1]) -> Result<(), E> {
        self.i2c
            .write_read(self.address, &[AppRegister::Status as u8], status)
    }

    /// Retrieves Error_Id register
    pub fn get_error_id(&mut self) -> Result<u8, E> {
        let mut data = [0u8; 1];
        self.i2c
            .write_read(self.address, &[AppRegister::ErrorId as u8], &mut data)?;
        Ok(data[0])
    }

    /// Get result
    pub fn get_results(&mut self) -> Result<SensorData, E> {
        let mut data: [u8; 8] = [0u8; 8];
        self.i2c
            .write_read(self.address, &[AppRegister::AlgResultData as u8], &mut data)?;

        let mut ret: SensorData = SensorData::default();
        ret.e_co = (u16::from(data[0]) << 8) + u16::from(data[1]);
        ret.e_tvoc = (u16::from(data[2]) << 8) + u16::from(data[3]);
        ret.status = data[4];
        ret.error_id = data[5];
        ret.raw[0] = data[6];
        ret.raw[1] = data[7];

        Ok(ret)
    }
}

fn get_raw_humidity(humidity_percentage: f32) -> (u8, u8) {
    get_raw_environment_data(humidity_percentage)
}

fn get_raw_temperature(temperature_celsius: f32) -> (u8, u8) {
    let value = temperature_celsius + 25.0;
    if value < 0.0 {
        (0, 0)
    } else {
        get_raw_environment_data(value)
    }
}

fn get_raw_environment_data(value: f32) -> (u8, u8) {
    let main = (value as u8) << 1;
    let rest = value - f32::from(value as u8);
    let rest = (rest * 512.0) as u16;
    (main | (((rest & (1 << 8)) >> 8) as u8), rest as u8)
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

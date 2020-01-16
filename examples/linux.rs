extern crate ccs811_rs;

use ccs811_rs::{Ccs811, DriveMode, InterruptDataReady, InterruptThreshold};
use rppal::i2c::*;
use std::thread;
use std::time::Duration;

// activate uart in raspi-config
fn main() {
    if let Err(e) = run() {
        eprintln!("Program exited early with error: {}", e);
    }
}

const DEVICE_ADDRESS: u8 = 0x5b;

fn run() -> Result<()> {
    // Configure I2C
    let i2c = I2c::with_bus(1)?;
    let mut ccs = Ccs811::new(i2c, DEVICE_ADDRESS);
    let mut status = [0u8; 1];
    ccs.get_status(&mut status)?;
    println!("Result: {:x}", status[0]);

    ccs.app_start()?;

    ccs.get_status(&mut status)?;
    println!("Result: {:x}", status[0]);

    ccs.set_meas_mode(
        DriveMode::DriveMode10Sec,
        InterruptDataReady::Disabled,
        InterruptThreshold::Disabled,
    )?;

    let fw = ccs.fw_info()?;
    println!("Fw: {:02x}{:02x}{:02x}{:02x}", fw[0], fw[1], fw[2], fw[3]);

    let hw = ccs.hw_info()?;
    println!("HwId: 0x{:02x}, HwRevision: 0x{:02x}", hw[0], hw[1]);

    loop {
        thread::sleep(Duration::from_secs(15));
        let res = ccs.get_results()?;
        println!(
            " eCO2: {}, eTVOC: {}, Status: 0x{:x}, Error: 0x{:x}, Raw: 0x{:x} 0x{:x}",
            res.e_co, res.e_tvoc, res.status, res.error_id, res.raw[1], res.raw[0]
        );

        if res.error_id != 0 {
            let error_id = ccs.get_error_id()?;
            println!("Error Id = {}", error_id);
        }
    }
    //ccs.reset()?;

    //Ok(())
}

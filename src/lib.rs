//! Nordic Semi nRF91 UART logger for Drone, an Embedded Operating System.
//!
//! # Documentation
//!
//! - [Drone Book](https://book.drone-os.com/)
//! - [API documentation](https://api.drone-os.com/drone-nrf91-uart-log/0.12/)
//!
//! # Usage
//!
//! Place the following to the Cargo.toml:
//!
//! ```toml
//! [dependencies]
//! drone-nrf91-uart-log = { version = "0.12.0", features = [...] }
//! ```

#![feature(const_fn)]
#![feature(const_if_match)]
#![feature(const_panic)]
#![deny(elided_lifetimes_in_paths)]
#![warn(missing_docs)]
#![warn(clippy::pedantic)]
#![allow(clippy::cast_possible_truncation, clippy::doc_markdown, clippy::wildcard_imports)]
#![no_std]

pub mod uart_logger;

mod set_log;

#[doc(hidden)]
#[must_use]
pub const fn convert_baud_rate(baud_rate: u32) -> u32 {
    match baud_rate {
        1_200 => 0x0004_F000,
        2_400 => 0x0009_D000,
        4_800 => 0x0013_B000,
        9_600 => 0x0027_5000,
        14_400 => 0x003A_F000,
        19_200 => 0x004E_A000,
        28_800 => 0x0075_C000,
        31_250 => 0x0080_0000,
        38_400 => 0x009D_0000,
        56_000 => 0x00E5_0000,
        57_600 => 0x00EB_0000,
        76_800 => 0x013A_9000,
        115_200 => 0x01D6_0000,
        230_400 => 0x03B0_0000,
        250_000 => 0x0400_0000,
        460_800 => 0x0740_0000,
        921_600 => 0x0F00_0000,
        1_000_000 => 0x1000_0000,
        _ => panic!("Unsupported UART baud rate"),
    }
}

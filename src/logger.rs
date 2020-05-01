#![cfg_attr(feature = "std", allow(unreachable_code, unused_variables))]

use crate::{uarte::Periph, DSO_PORTS};
use core::{
    cmp::min,
    ptr,
    ptr::{read_volatile, NonNull},
};
use drone_cortexm::reg::prelude::*;
use drone_nrf_map::periph::uarte::{traits::*, UarteMap};

const KEY: u16 = 0b100_1011;
const MAX_PACKET_SIZE: usize = 16;

#[doc(hidden)]
pub unsafe trait Logger {
    type UarteMap: UarteMap;

    const BAUD_RATE: u32;
    const BUF_SIZE: u32;
    const PIN_NUMBER: u32;

    fn buf() -> NonNull<u8>;
}

#[doc(hidden)]
#[must_use]
#[inline]
pub fn is_enabled<T: Logger>(port: u8) -> bool {
    unsafe { read_volatile(&DSO_PORTS) & 1 << port != 0 }
}

#[doc(hidden)]
#[inline]
pub fn write_bytes<T: Logger>(port: u8, bytes: &[u8]) {
    for bytes in bytes.chunks(min(T::BUF_SIZE as usize - 2, MAX_PACKET_SIZE)) {
        unsafe { write_packet::<T>(port, bytes) };
    }
}

#[doc(hidden)]
#[inline]
pub fn flush<T: Logger>() {
    let periph = Periph::<T::UarteMap>::summon();
    // UART transmitter has started.
    if !(is_init(&periph) && periph.uarte_events_txstarted.events_txstarted().read_bit()) {
        return;
    }
    // Last TX byte transmitted.
    while !periph.uarte_events_endtx.events_endtx().read_bit() {}
}

fn is_init<T: UarteMap>(periph: &Periph<T>) -> bool {
    periph.uarte_enable.enable().read_bits() == 8 // enable UARTE
}

fn init<T: UarteMap>(periph: &mut Periph<T>, buf_ptr: *const u8, pin_number: u32, baud_rate: u32) {
    if is_init(periph) {
        return;
    }
    periph.uarte_psel_txd.store_val({
        let mut val = periph.uarte_psel_txd.default_val();
        // Pin number.
        periph.uarte_psel_txd.pin().write(&mut val, pin_number);
        // Connect.
        periph.uarte_psel_txd.connect().clear(&mut val);
        val
    });
    periph.uarte_psel_rxd.store_val({
        let mut val = periph.uarte_psel_rxd.default_val();
        // Disconnect.
        periph.uarte_psel_rxd.connect().set(&mut val);
        val
    });
    periph.uarte_psel_rts.store_val({
        let mut val = periph.uarte_psel_rts.default_val();
        // Disconnect.
        periph.uarte_psel_rts.connect().set(&mut val);
        val
    });
    periph.uarte_psel_cts.store_val({
        let mut val = periph.uarte_psel_cts.default_val();
        // Disconnect.
        periph.uarte_psel_cts.connect().set(&mut val);
        val
    });
    periph.uarte_txd_ptr.store_val({
        let mut val = periph.uarte_txd_ptr.default_val();
        // Data pointer.
        periph.uarte_txd_ptr.ptr().write(&mut val, buf_ptr as u32);
        val
    });
    periph.uarte_baudrate.store_val({
        let mut val = periph.uarte_baudrate.default_val();
        // Baud rate.
        periph.uarte_baudrate.baudrate().write(&mut val, baud_rate);
        val
    });
    periph.uarte_enable.store_val({
        let mut val = periph.uarte_enable.default_val();
        // Enable UARTE.
        periph.uarte_enable.enable().write(&mut val, 8);
        val
    });
}

unsafe fn write_packet<T: Logger>(port: u8, bytes: &[u8]) {
    #[cfg(feature = "std")]
    return;
    llvm_asm!("cpsid i" :::: "volatile");
    let mut periph = Periph::<T::UarteMap>::summon();
    init(&mut periph, T::buf().as_ptr(), T::PIN_NUMBER, T::BAUD_RATE);
    flush::<T>();
    let count = fill_buf(T::buf().as_ptr(), port, bytes);
    periph.uarte_txd_maxcnt.store_val({
        let mut val = periph.uarte_txd_maxcnt.default_val();
        // Maximum number of bytes in transmit buffer.
        periph.uarte_txd_maxcnt.maxcnt().write(&mut val, count);
        val
    });
    periph.uarte_events_txstarted.store_val({
        let mut val = periph.uarte_events_txstarted.default_val();
        // UART transmitter has started.
        periph.uarte_events_txstarted.events_txstarted().clear(&mut val);
        val
    });
    periph.uarte_events_endtx.store_val({
        let mut val = periph.uarte_events_endtx.default_val();
        // Last TX byte transmitted.
        periph.uarte_events_endtx.events_endtx().clear(&mut val);
        val
    });
    periph.uarte_tasks_starttx.store_val({
        let mut val = periph.uarte_tasks_starttx.default_val();
        // Start UART transmitter.
        periph.uarte_tasks_starttx.tasks_starttx().set(&mut val);
        val
    });
    llvm_asm!("cpsie i" :::: "volatile");
}

#[allow(clippy::cast_ptr_alignment)]
unsafe fn fill_buf(buf_ptr: *mut u8, port: u8, bytes: &[u8]) -> u32 {
    *(buf_ptr as *mut u16) = (KEY << 9 | u16::from(port) << 4 | (bytes.len() as u16 - 1)).to_be();
    ptr::copy_nonoverlapping(bytes.as_ptr(), buf_ptr.add(2), bytes.len());
    bytes.len() as u32 + 2
}

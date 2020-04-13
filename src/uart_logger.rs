//! UART logger.

use core::{
    mem, ptr,
    sync::atomic::{AtomicU8, Ordering},
};
use drone_cortex_m::reg::prelude::*;
use drone_nrf_map::periph::uarte::{traits::*, UarteMap, UartePeriph};

/// UART logger state.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum State {
    /// TODO docs
    Uninit = 0,
    /// TODO docs
    Init = 1,
}

/// UART logger.
///
/// # Safety
///
/// The implementation of this trait must have unique access to:
///
/// * `UarteMap` UARTE peripheral
/// * Static variable used by `state`
/// * Static variable used by `buf_ptr`
pub unsafe trait UartLogger: Sized {
    /// UARTE peripheral.
    type UarteMap: UarteMap;

    /// Output baud rate.
    const BAUD_RATE: u32;

    /// Size of the UART buffer.
    const BUF_SIZE: u32;

    /// Output pin number.
    const PIN_NUMBER: u32;

    /// Returns a new instance of UARTE peripheral.
    ///
    /// This method is zero-cost, because [`UartePeriph`] is ZST.
    fn periph() -> UartePeriph<Self::UarteMap>;

    /// Returns a reference to the static variable holding initialization state.
    fn state() -> &'static AtomicU8;

    /// Returns a mutable pointer to the UART buffer.
    fn buf_ptr() -> *mut u8;
}

/// Returns `true` if the debug probe is connected and listening to the output
/// of port number `port`.
#[must_use]
#[inline]
pub fn is_enabled<T: UartLogger>(_port: u8) -> bool {
    true
}

/// Writes `bytes` to the port number `port`.
#[inline]
pub fn write_bytes<T: UartLogger>(_port: u8, bytes: &[u8]) {
    init::<T>();
    // Last TX byte transmitted.
    T::periph().uarte_events_endtx.events_endtx().clear_bit();
    unsafe { ptr::copy_nonoverlapping(bytes.as_ptr(), T::buf_ptr(), bytes.len()) };
    T::periph().uarte_txd_ptr.store_val({
        let mut val = T::periph().uarte_txd_ptr.default_val();
        // Data pointer.
        T::periph().uarte_txd_ptr.ptr().write(&mut val, T::buf_ptr() as u32);
        val
    });
    T::periph().uarte_txd_maxcnt.store_val({
        let mut val = T::periph().uarte_txd_maxcnt.default_val();
        // Maximum number of bytes in transmit buffer.
        T::periph().uarte_txd_maxcnt.maxcnt().write(&mut val, bytes.len() as u32);
        val
    });
    T::periph().uarte_tasks_starttx.store_val({
        let mut val = T::periph().uarte_tasks_starttx.default_val();
        // Start UART transmitter.
        T::periph().uarte_tasks_starttx.tasks_starttx().set(&mut val);
        val
    });
}

/// Blocks until all pending packets are transmitted.
///
/// This function is a no-op if no debug probe is connected and listening.
#[inline]
pub fn flush<T: UartLogger>() {
    if matches!(unsafe { State::from_u8(T::state().load(Ordering::Relaxed)) }, State::Uninit) {
        return;
    }
    // Last TX byte transmitted.
    while !T::periph().uarte_events_endtx.events_endtx().read_bit() {}
}

fn init<T: UartLogger>() {
    if matches!(unsafe { State::from_u8(T::state().load(Ordering::Relaxed)) }, State::Init) {
        return;
    }
    T::periph().uarte_psel_txd.store_val({
        let mut val = T::periph().uarte_psel_txd.default_val();
        // Pin number.
        T::periph().uarte_psel_txd.pin().write(&mut val, T::PIN_NUMBER);
        // Connect.
        T::periph().uarte_psel_txd.connect().clear(&mut val);
        val
    });
    T::periph().uarte_psel_rxd.store_val({
        let mut val = T::periph().uarte_psel_rxd.default_val();
        // Disconnect.
        T::periph().uarte_psel_rxd.connect().set(&mut val);
        val
    });
    T::periph().uarte_psel_rts.store_val({
        let mut val = T::periph().uarte_psel_rts.default_val();
        // Disconnect.
        T::periph().uarte_psel_rts.connect().set(&mut val);
        val
    });
    T::periph().uarte_psel_cts.store_val({
        let mut val = T::periph().uarte_psel_cts.default_val();
        // Disconnect.
        T::periph().uarte_psel_cts.connect().set(&mut val);
        val
    });
    T::periph().uarte_baudrate.store_val({
        let mut val = T::periph().uarte_baudrate.default_val();
        // Baud rate.
        T::periph().uarte_baudrate.baudrate().write(&mut val, T::BAUD_RATE);
        val
    });
    T::periph().uarte_enable.store_val({
        let mut val = T::periph().uarte_enable.default_val();
        // Enable UARTE.
        T::periph().uarte_enable.enable().write(&mut val, 8);
        val
    });
    T::state().store(State::Init.into_u8(), Ordering::Relaxed)
}

impl State {
    unsafe fn from_u8(state: u8) -> Self {
        mem::transmute(state)
    }

    fn into_u8(self) -> u8 {
        self as _
    }
}

/// Sets DSO as default logger.
#[macro_export]
macro_rules! set_log {
    (periph: $uarte:ident,pin_number: $pin_number:expr,buf_size: $buf_size:expr,) => {
        const _: () = {
            use ::core::{ptr::NonNull, slice};
            use ::drone_core::log;
            use ::drone_cortex_m::reg;
            use ::drone_nrf_map::periph::uarte::$uarte;

            $crate::uarte_assert_taken!($uarte);

            static mut BUF: [u8; $buf_size] = [0; $buf_size];

            struct Logger;

            unsafe impl $crate::Logger for Logger {
                type UarteMap = $uarte;

                const BAUD_RATE: u32 = $crate::baud_rate(log::baud_rate!());
                const BUF_SIZE: u32 = $buf_size;
                const PIN_NUMBER: u32 = $pin_number;

                #[inline]
                fn buf() -> NonNull<u8> {
                    unsafe { NonNull::new_unchecked(BUF.as_mut_ptr()) }
                }
            }

            #[no_mangle]
            extern "C" fn drone_log_is_enabled(port: u8) -> bool {
                $crate::is_enabled::<Logger>(port)
            }

            #[no_mangle]
            extern "C" fn drone_log_write_bytes(port: u8, buffer: *const u8, count: usize) {
                $crate::write_bytes::<Logger>(port, unsafe { slice::from_raw_parts(buffer, count) })
            }

            #[no_mangle]
            extern "C" fn drone_log_write_u8(port: u8, value: u8) {
                $crate::write_bytes::<Logger>(port, &value.to_be_bytes())
            }

            #[no_mangle]
            extern "C" fn drone_log_write_u16(port: u8, value: u16) {
                $crate::write_bytes::<Logger>(port, &value.to_be_bytes())
            }

            #[no_mangle]
            extern "C" fn drone_log_write_u32(port: u8, value: u32) {
                $crate::write_bytes::<Logger>(port, &value.to_be_bytes())
            }

            #[no_mangle]
            extern "C" fn drone_log_flush() {
                $crate::flush::<Logger>();
            }
        };
    };
}

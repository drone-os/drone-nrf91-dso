/// Sets UART as default logger.
///
/// # Examples
///
/// ```
/// #![feature(proc_macro_hygiene)]
/// # drone_core::config_override! { "
/// # [memory]
/// # flash = { size = \"128K\", origin = 0x08000000 }
/// # ram = { size = \"20K\", origin = 0x20000000 }
/// # [heap]
/// # size = \"0\"
/// # pools = []
/// # [probe]
/// # gdb-client-command = \"gdb-multiarch\"
/// # [probe.uart]
/// # baud-rate = 115200
/// # endpoint = \"/dev/ttyACM0\"
/// # " }
/// use drone_nrf_map::nrf_reg_tokens;
///
/// nrf_reg_tokens! {
///     struct Regs;
///
///     !uarte0_ns_tasks_startrx; !uarte0_ns_tasks_stoprx; !uarte0_ns_tasks_starttx;
///     !uarte0_ns_tasks_stoptx; !uarte0_ns_tasks_flushrx; !uarte0_ns_subscribe_startrx;
///     !uarte0_ns_subscribe_stoprx; !uarte0_ns_subscribe_starttx; !uarte0_ns_subscribe_stoptx;
///     !uarte0_ns_subscribe_flushrx; !uarte0_ns_events_cts; !uarte0_ns_events_ncts;
///     !uarte0_ns_events_rxdrdy; !uarte0_ns_events_endrx; !uarte0_ns_events_txdrdy;
///     !uarte0_ns_events_endtx; !uarte0_ns_events_error; !uarte0_ns_events_rxto;
///     !uarte0_ns_events_rxstarted; !uarte0_ns_events_txstarted; !uarte0_ns_events_txstopped;
///     !uarte0_ns_publish_cts; !uarte0_ns_publish_ncts; !uarte0_ns_publish_rxdrdy;
///     !uarte0_ns_publish_endrx; !uarte0_ns_publish_txdrdy; !uarte0_ns_publish_endtx;
///     !uarte0_ns_publish_error; !uarte0_ns_publish_rxto; !uarte0_ns_publish_rxstarted;
///     !uarte0_ns_publish_txstarted; !uarte0_ns_publish_txstopped; !uarte0_ns_shorts; !uarte0_ns_inten;
///     !uarte0_ns_intenset; !uarte0_ns_intenclr; !uarte0_ns_errorsrc; !uarte0_ns_enable;
///     !uarte0_ns_psel_rts; !uarte0_ns_psel_txd; !uarte0_ns_psel_cts; !uarte0_ns_psel_rxd;
///     !uarte0_ns_baudrate; !uarte0_ns_rxd_ptr; !uarte0_ns_rxd_maxcnt; !uarte0_ns_rxd_amount;
///     !uarte0_ns_txd_ptr; !uarte0_ns_txd_maxcnt; !uarte0_ns_txd_amount; !uarte0_ns_config;
/// }
///
/// drone_nrf91_uart_log::set_log! {
///     periph: Uarte0S,
///     pin_number: 29,
///     buf_size: 64,
/// }
/// ```
#[macro_export]
macro_rules! set_log {
    (periph: $uarte:ident,pin_number: $pin_number:expr,buf_size: $buf_size:expr,) => {
        const _: () = {
            use ::core::{mem::MaybeUninit, slice, sync::atomic::AtomicU8};
            use ::drone_core::log;
            use ::drone_cortex_m::reg;
            use ::drone_nrf_map::periph::uarte::{$uarte, UartePeriph};
            use $crate::{uart_logger, uart_logger::UartLogger};

            $crate::reg_assert_taken!($uarte);

            static mut BUF: [u8; $buf_size] = [0; $buf_size];
            static STATE: AtomicU8 = AtomicU8::new(0);

            struct Logger;

            unsafe impl UartLogger for Logger {
                type UarteMap = $uarte;

                const BAUD_RATE: u32 = $crate::uarte_baud_rate(log::baud_rate!());
                const BUF_SIZE: u32 = $buf_size;
                const PIN_NUMBER: u32 = $pin_number;

                #[inline]
                fn periph() -> UartePeriph<Self::UarteMap> {
                    unsafe { MaybeUninit::uninit().assume_init() }
                }

                #[inline]
                fn state() -> &'static AtomicU8 {
                    &STATE
                }

                #[inline]
                fn buf_ptr() -> *mut u8 {
                    unsafe { BUF.as_mut_ptr() }
                }
            }

            #[no_mangle]
            extern "C" fn drone_log_is_enabled(port: u8) -> bool {
                uart_logger::is_enabled::<Logger>(port)
            }

            #[no_mangle]
            extern "C" fn drone_log_write_bytes(
                port: u8,
                _exclusive: bool,
                buffer: *const u8,
                count: usize,
            ) {
                uart_logger::write_bytes::<Logger>(port, unsafe {
                    slice::from_raw_parts(buffer, count)
                })
            }

            #[no_mangle]
            extern "C" fn drone_log_flush() {
                uart_logger::flush::<Logger>();
            }
        };
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! reg_assert_taken {
    (Uarte0Ns) => {
        $crate::reg_assert_taken!("uarte0_ns");
    };
    (Uarte0S) => {
        $crate::reg_assert_taken!("uarte0_ns");
    };
    (Uarte1Ns) => {
        $crate::reg_assert_taken!("uarte1_ns");
    };
    (Uarte1S) => {
        $crate::reg_assert_taken!("uarte1_ns");
    };
    (Uarte2Ns) => {
        $crate::reg_assert_taken!("uarte2_ns");
    };
    (Uarte2S) => {
        $crate::reg_assert_taken!("uarte2_ns");
    };
    (Uarte3Ns) => {
        $crate::reg_assert_taken!("uarte3_ns");
    };
    (Uarte3S) => {
        $crate::reg_assert_taken!("uarte3_ns");
    };
    ($uarte:ident) => {
        compile_error!("Unsupported peripheral");
    };
    ($uarte:literal) => {
        reg::assert_taken!(concat!($uarte, "_tasks_startrx"));
        reg::assert_taken!(concat!($uarte, "_tasks_stoprx"));
        reg::assert_taken!(concat!($uarte, "_tasks_starttx"));
        reg::assert_taken!(concat!($uarte, "_tasks_stoptx"));
        reg::assert_taken!(concat!($uarte, "_tasks_flushrx"));
        reg::assert_taken!(concat!($uarte, "_subscribe_startrx"));
        reg::assert_taken!(concat!($uarte, "_subscribe_stoprx"));
        reg::assert_taken!(concat!($uarte, "_subscribe_starttx"));
        reg::assert_taken!(concat!($uarte, "_subscribe_stoptx"));
        reg::assert_taken!(concat!($uarte, "_subscribe_flushrx"));
        reg::assert_taken!(concat!($uarte, "_events_cts"));
        reg::assert_taken!(concat!($uarte, "_events_ncts"));
        reg::assert_taken!(concat!($uarte, "_events_rxdrdy"));
        reg::assert_taken!(concat!($uarte, "_events_endrx"));
        reg::assert_taken!(concat!($uarte, "_events_txdrdy"));
        reg::assert_taken!(concat!($uarte, "_events_endtx"));
        reg::assert_taken!(concat!($uarte, "_events_error"));
        reg::assert_taken!(concat!($uarte, "_events_rxto"));
        reg::assert_taken!(concat!($uarte, "_events_rxstarted"));
        reg::assert_taken!(concat!($uarte, "_events_txstarted"));
        reg::assert_taken!(concat!($uarte, "_events_txstopped"));
        reg::assert_taken!(concat!($uarte, "_publish_cts"));
        reg::assert_taken!(concat!($uarte, "_publish_ncts"));
        reg::assert_taken!(concat!($uarte, "_publish_rxdrdy"));
        reg::assert_taken!(concat!($uarte, "_publish_endrx"));
        reg::assert_taken!(concat!($uarte, "_publish_txdrdy"));
        reg::assert_taken!(concat!($uarte, "_publish_endtx"));
        reg::assert_taken!(concat!($uarte, "_publish_error"));
        reg::assert_taken!(concat!($uarte, "_publish_rxto"));
        reg::assert_taken!(concat!($uarte, "_publish_rxstarted"));
        reg::assert_taken!(concat!($uarte, "_publish_txstarted"));
        reg::assert_taken!(concat!($uarte, "_publish_txstopped"));
        reg::assert_taken!(concat!($uarte, "_shorts"));
        reg::assert_taken!(concat!($uarte, "_inten"));
        reg::assert_taken!(concat!($uarte, "_intenset"));
        reg::assert_taken!(concat!($uarte, "_intenclr"));
        reg::assert_taken!(concat!($uarte, "_errorsrc"));
        reg::assert_taken!(concat!($uarte, "_enable"));
        reg::assert_taken!(concat!($uarte, "_psel_rts"));
        reg::assert_taken!(concat!($uarte, "_psel_txd"));
        reg::assert_taken!(concat!($uarte, "_psel_cts"));
        reg::assert_taken!(concat!($uarte, "_psel_rxd"));
        reg::assert_taken!(concat!($uarte, "_baudrate"));
        reg::assert_taken!(concat!($uarte, "_rxd_ptr"));
        reg::assert_taken!(concat!($uarte, "_rxd_maxcnt"));
        reg::assert_taken!(concat!($uarte, "_rxd_amount"));
        reg::assert_taken!(concat!($uarte, "_txd_ptr"));
        reg::assert_taken!(concat!($uarte, "_txd_maxcnt"));
        reg::assert_taken!(concat!($uarte, "_txd_amount"));
        reg::assert_taken!(concat!($uarte, "_config"));
    };
}

use core::mem::MaybeUninit;
use drone_nrf_map::periph::uarte::UarteMap;

pub(crate) struct Periph<T: UarteMap> {
    pub uarte_tasks_startrx: T::UUarteTasksStartrx,
    pub uarte_tasks_stoprx: T::UUarteTasksStoprx,
    pub uarte_tasks_starttx: T::UUarteTasksStarttx,
    pub uarte_tasks_stoptx: T::UUarteTasksStoptx,
    pub uarte_tasks_flushrx: T::UUarteTasksFlushrx,
    pub uarte_subscribe_startrx: T::UUarteSubscribeStartrx,
    pub uarte_subscribe_stoprx: T::UUarteSubscribeStoprx,
    pub uarte_subscribe_starttx: T::UUarteSubscribeStarttx,
    pub uarte_subscribe_stoptx: T::UUarteSubscribeStoptx,
    pub uarte_subscribe_flushrx: T::UUarteSubscribeFlushrx,
    pub uarte_events_cts: T::UUarteEventsCts,
    pub uarte_events_ncts: T::UUarteEventsNcts,
    pub uarte_events_rxdrdy: T::UUarteEventsRxdrdy,
    pub uarte_events_endrx: T::UUarteEventsEndrx,
    pub uarte_events_txdrdy: T::UUarteEventsTxdrdy,
    pub uarte_events_endtx: T::UUarteEventsEndtx,
    pub uarte_events_error: T::UUarteEventsError,
    pub uarte_events_rxto: T::UUarteEventsRxto,
    pub uarte_events_rxstarted: T::UUarteEventsRxstarted,
    pub uarte_events_txstarted: T::UUarteEventsTxstarted,
    pub uarte_events_txstopped: T::UUarteEventsTxstopped,
    pub uarte_publish_cts: T::UUartePublishCts,
    pub uarte_publish_ncts: T::UUartePublishNcts,
    pub uarte_publish_rxdrdy: T::UUartePublishRxdrdy,
    pub uarte_publish_endrx: T::UUartePublishEndrx,
    pub uarte_publish_txdrdy: T::UUartePublishTxdrdy,
    pub uarte_publish_endtx: T::UUartePublishEndtx,
    pub uarte_publish_error: T::UUartePublishError,
    pub uarte_publish_rxto: T::UUartePublishRxto,
    pub uarte_publish_rxstarted: T::UUartePublishRxstarted,
    pub uarte_publish_txstarted: T::UUartePublishTxstarted,
    pub uarte_publish_txstopped: T::UUartePublishTxstopped,
    pub uarte_shorts: T::UUarteShorts,
    pub uarte_inten: T::UUarteInten,
    pub uarte_intenset: T::UUarteIntenset,
    pub uarte_intenclr: T::UUarteIntenclr,
    pub uarte_errorsrc: T::UUarteErrorsrc,
    pub uarte_enable: T::UUarteEnable,
    pub uarte_psel_rts: T::UUartePselRts,
    pub uarte_psel_txd: T::UUartePselTxd,
    pub uarte_psel_cts: T::UUartePselCts,
    pub uarte_psel_rxd: T::UUartePselRxd,
    pub uarte_baudrate: T::UUarteBaudrate,
    pub uarte_rxd_ptr: T::UUarteRxdPtr,
    pub uarte_rxd_maxcnt: T::UUarteRxdMaxcnt,
    pub uarte_rxd_amount: T::UUarteRxdAmount,
    pub uarte_txd_ptr: T::UUarteTxdPtr,
    pub uarte_txd_maxcnt: T::UUarteTxdMaxcnt,
    pub uarte_txd_amount: T::UUarteTxdAmount,
    pub uarte_config: T::UUarteConfig,
}

impl<T: UarteMap> Periph<T> {
    #[allow(clippy::uninit_assumed_init)]
    pub(crate) fn summon() -> Self {
        unsafe { MaybeUninit::uninit().assume_init() }
    }
}

#[doc(hidden)]
#[must_use]
pub const fn baud_rate(baud_rate: u32) -> u32 {
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
        _ => panic!("Unsupported UARTE baud rate"),
    }
}

#[doc(hidden)]
#[macro_export]
macro_rules! uarte_assert_taken {
    (Uarte0Ns) => {
        $crate::uarte_assert_taken!("uarte0_ns");
    };
    (Uarte0S) => {
        $crate::uarte_assert_taken!("uarte0_ns");
    };
    (Uarte1Ns) => {
        $crate::uarte_assert_taken!("uarte1_ns");
    };
    (Uarte1S) => {
        $crate::uarte_assert_taken!("uarte1_ns");
    };
    (Uarte2Ns) => {
        $crate::uarte_assert_taken!("uarte2_ns");
    };
    (Uarte2S) => {
        $crate::uarte_assert_taken!("uarte2_ns");
    };
    (Uarte3Ns) => {
        $crate::uarte_assert_taken!("uarte3_ns");
    };
    (Uarte3S) => {
        $crate::uarte_assert_taken!("uarte3_ns");
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

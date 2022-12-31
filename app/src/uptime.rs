use cortex_m::peripheral::{syst::SystClkSource, SYST};
use cortex_m_rt::exception;

use app_core::common::Instant;
use fugit::Duration;
use rtt_target::rprintln;

pub struct Uptime {
    _syst: SYST,
}

impl Uptime {
    pub fn new(mut syst: SYST) -> Self {
        syst.set_clock_source(SystClkSource::External);

        let ticks_per_reload_period =
            SYST_RELOAD_PERIOD_MS * (SYST::get_ticks_per_10ms() + 1) / 10 - 1;

        syst.set_reload(ticks_per_reload_period);
        syst.clear_current();
        syst.enable_counter();
        syst.enable_interrupt();

        Uptime { _syst: syst }
    }

    /// This function assumes that [`SYST`] is owned and initialized by [`Uptime`].
    pub fn get_us() -> u64 {
        let reload_count = cortex_m::interrupt::free(|_| unsafe { SYST_RELOAD_COUNT });
        u64::from(reload_count) * u64::from(SYST_RELOAD_PERIOD_MS) * 1000
            + u64::from(SYST::get_reload() - SYST::get_current())
    }

    pub fn get_instant() -> Instant {
        Instant::from_ticks(Self::get_us())
    }

    pub fn delay_us(us: u64) {
        let start = Self::get_instant();
        let delay = Duration::<u64, 1, 1_000_000>::from_ticks(us);
        let end = start
            .checked_add_duration(delay)
            .expect("uptime must not overflow during the delay");
        loop {
            let now = Self::get_instant();
            if let Some(left) = end.checked_duration_since(now) {
                if left.to_millis() > SYST_RELOAD_PERIOD_MS as u64 {
                    cortex_m::asm::wfi();
                } else {
                    break;
                }
            } else {
                let overshot = now - end;
                rprintln!("Delay overshot by {}", overshot);
                break;
            }
        }
    }

    pub fn delay_ms(ms: u64) {
        let start = Self::get_instant();
        let delay = Duration::<u64, 1, 1_000>::from_ticks(ms);
        let end = start
            .checked_add_duration(delay)
            .expect("uptime must not overflow during the delay");
        loop {
            let left = end - Self::get_instant();
            if left.to_millis() > SYST_RELOAD_PERIOD_MS as u64 {
                cortex_m::asm::wfi();
            } else {
                break;
            }
        }
    }
}

#[exception]
fn SysTick() {
    unsafe {
        SYST_RELOAD_COUNT += 1;
    }
}

static mut SYST_RELOAD_COUNT: u32 = 0;
const SYST_RELOAD_PERIOD_MS: u32 = 1;

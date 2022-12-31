use embedded_hal::blocking::delay::{DelayMs, DelayUs};

use crate::uptime::Uptime;

impl DelayUs<u64> for Uptime {
    fn delay_us(&mut self, us: u64) {
        Uptime::delay_us(us)
    }
}

impl DelayMs<u8> for Uptime {
    fn delay_ms(&mut self, ms: u8) {
        Uptime::delay_ms(ms as u64)
    }
}

impl DelayMs<u64> for Uptime {
    fn delay_ms(&mut self, ms: u64) {
        Uptime::delay_ms(ms)
    }
}

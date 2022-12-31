#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

mod uptime;
mod uptime_delay;

extern crate alloc;

use alloc::{alloc::Layout, fmt::Debug};
use core::panic::PanicInfo;
use embedded_hal::PwmPin;

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use rp_pico as bsp;

use bsp::{
    entry, hal,
    hal::{
        clocks::ClocksManager,
        pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig},
        xosc::setup_xosc_blocking,
        Clock, Sio, Watchdog, I2C,
    },
    pac,
};

use alloc_cortex_m::CortexMHeap;

use fugit::{HertzU32, RateExtU32};
use rtt_target::{rprintln, rtt_init_print};

use crate::uptime::Uptime;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    init_heap();

    let mut pac = pac::Peripherals::take().unwrap();
    let sio = Sio::new(pac.SIO);

    let core = pac::CorePeripherals::take().unwrap();
    let _uptime = Uptime::new(core.SYST);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External crystal on the Pico board is 12 Mhz
    let xtal_freq_hz = 12_000_000u32;
    let xosc = setup_xosc_blocking(pac.XOSC, xtal_freq_hz.Hz())
        .ok()
        .unwrap();

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((xtal_freq_hz / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    // Some SYS_PLL configurations and resulting clock frequencies:
    // 11   MHz      400, 1, 6, 6
    //  8.3 MHz      600, 2, 6, 6
    //  7 083 333 Hz 510, 2, 6, 6

    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        PLLConfig {
            vco_freq: HertzU32::MHz(510),
            refdiv: 2,
            post_div1: 6,
            post_div2: 6,
        },
        &mut clocks,
        &mut pac.RESETS,
    )
    .ok()
    .unwrap();

    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .ok()
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();
    rprintln!("System clock freq: {}", clocks.system_clock.freq());

    const PWM_TOP: u16 = 0x07ff;
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm1;
    pwm.set_top(PWM_TOP);
    pwm.enable();

    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio2);

    // const FADE_TOP: u16 = (PWM_TOP as f32 * 0.95) as u16;
    const FADE_TOP: u16 = PWM_TOP;
    const FADE_STEP_DELAY_US: u64 = (30e6 / FADE_TOP as f32 / 2.0) as u64;
    loop {
        for i in 0..=FADE_TOP {
            Uptime::delay_us(FADE_STEP_DELAY_US);
            let duty = (smoothstep(0.0, FADE_TOP as f32, i as f32) * FADE_TOP as f32) as u16;
            channel.set_duty(duty);
        }

        for i in (0..=FADE_TOP).rev() {
            Uptime::delay_us(FADE_STEP_DELAY_US);
            let duty = (smoothstep(0.0, FADE_TOP as f32, i as f32) * FADE_TOP as f32) as u16;
            channel.set_duty(duty);
        }
    }
}

fn smoothstep(a: f32, b: f32, x: f32) -> f32 {
    if x < a {
        0.0
    } else if x > b {
        1.0
    } else {
        let k = (x - a) / (b - a);
        k * k * (3.0 - 2.0 * k)
    }
}

#[alloc_error_handler]
fn oom(layout: Layout) -> ! {
    rprintln!(
        "failed to allocate {} bytes aligned on {} bytes)",
        layout.size(),
        layout.align()
    );
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {
        cortex_m::asm::wfi();
    }
}

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

fn init_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 128 * 1024;
    static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }
}

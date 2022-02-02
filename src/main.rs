#![no_std]
#![no_main]
#![deny(clippy::all)]
#![deny(unsafe_code)]
#![deny(warnings)]

use core::{
  cell::RefCell,
  sync::atomic::{AtomicBool, Ordering},
};
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use panic_semihosting as _;
use rp_pico::{
  entry,
  hal::{
    self,
    gpio::{bank0::Gpio25, Output, Pin, PushPull},
    timer::Alarm0,
  },
  pac::{self, interrupt},
};

type LedPin = Pin<Gpio25, Output<PushPull>>;

static ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static LED_ON: AtomicBool = AtomicBool::new(false);
static TIMER: Mutex<RefCell<Option<hal::Timer>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
  let mut peripherals = pac::Peripherals::take().unwrap();

  let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);

  let _clocks = hal::clocks::init_clocks_and_plls(
    rp_pico::XOSC_CRYSTAL_FREQ,
    peripherals.XOSC,
    peripherals.CLOCKS,
    peripherals.PLL_SYS,
    peripherals.PLL_USB,
    &mut peripherals.RESETS,
    &mut watchdog,
  )
  .ok()
  .unwrap();

  let mut timer = hal::Timer::new(peripherals.TIMER, &mut peripherals.RESETS);
  let alarm = timer.alarm_0().unwrap();

  let sio = hal::Sio::new(peripherals.SIO);

  let pins = rp_pico::Pins::new(
    peripherals.IO_BANK0,
    peripherals.PADS_BANK0,
    sio.gpio_bank0,
    &mut peripherals.RESETS,
  );

  let led = pins.led.into_push_pull_output();

  cortex_m::interrupt::free(|cs| {
    ALARM.borrow(cs).replace(Some(alarm));
    LED.borrow(cs).replace(Some(led));
    TIMER.borrow(cs).replace(Some(timer));

    let mut alarm = ALARM.borrow(cs).borrow_mut();
    let alarm = alarm.as_mut().unwrap();
    let mut timer = TIMER.borrow(cs).borrow_mut();
    let timer = timer.as_mut().unwrap();
    let mut led = LED.borrow(cs).borrow_mut();
    let led = led.as_mut().unwrap();

    alarm.schedule(500_000.microseconds()).unwrap();
    alarm.enable_interrupt(timer);

    led.set_high().unwrap();
    LED_ON.store(true, Ordering::Release);
  });

  #[allow(unsafe_code)]
  unsafe {
    pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
  }

  // loop and set timer interrupts to switch led on and off
  // led on after 1.5 seconds and off after 0.5 seconds
  loop {
    cortex_m::interrupt::free(|cs| {
      let mut alarm = ALARM.borrow(cs).borrow_mut();
      let alarm = alarm.as_mut().unwrap();

      if alarm.finished() {
        let mut timer = TIMER.borrow(cs).borrow_mut();
        let timer = timer.as_mut().unwrap();

        if LED_ON.load(Ordering::Acquire) {
          alarm.schedule(500_000.microseconds()).unwrap();
        } else {
          alarm.schedule(1_500_000.microseconds()).unwrap();
        }
        alarm.enable_interrupt(timer);
      }
    });
  }
}

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_0() {
  cortex_m::interrupt::free(|cs| {
    let mut alarm = ALARM.borrow(cs).borrow_mut();
    let alarm = alarm.as_mut().unwrap();
    let mut timer = TIMER.borrow(cs).borrow_mut();
    let timer = timer.as_mut().unwrap();
    let mut led = LED.borrow(cs).borrow_mut();
    let led = led.as_mut().unwrap();

    let is_high = LED_ON.load(Ordering::Acquire);
    if is_high {
      led.set_low().unwrap();
    } else {
      led.set_high().unwrap();
    }
    LED_ON.store(!is_high, Ordering::Release);

    alarm.clear_interrupt(timer);
  });
}

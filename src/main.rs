#![no_std]
#![no_main]

extern crate alloc;

use cortex_m::delay::Delay;
use embedded_hal::digital::OutputPin;
use hal::{clocks::Clock, pac, watchdog::Watchdog, Sio};
use hashbrown::HashMap;
use panic_halt as _;
use rp2040_hal::gpio::bank0::Gpio15;
use rp2040_hal::gpio::{FunctionSio, Pin, PinState, PullDown, SioOutput};
use rp2040_hal::{self as hal};

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rp2040_hal::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let cm = hal::clocks::init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, cm.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin: Pin<Gpio15, FunctionSio<SioOutput>, rp2040_hal::gpio::PullDown> =
        pins.gpio15.into_push_pull_output_in_state(PinState::Low);

    let morse_code_machine = MorseCodeMachine::new();

    let message = "my name is gavin christman";

    loop {
        morse_code_machine.send(&mut led_pin, &mut delay, message);

        delay.delay_ms(5000);
    }
}

pub struct MorseCodeMachine<'a> {
    keys: HashMap<char, &'a str>,
}

impl MorseCodeMachine<'static> {
    fn new() -> MorseCodeMachine<'static> {
        let keys: HashMap<char, &str> = HashMap::from([
            ('a', ".-"),
            ('b', "-..."),
            ('c', "-.-."),
            ('d', "-.."),
            ('e', "."),
            ('f', "..-."),
            ('g', "--."),
            ('h', "...."),
            ('i', ".."),
            ('j', ".---"),
            ('k', "-.-"),
            ('l', ".-.."),
            ('m', "--"),
            ('n', "-."),
            ('o', "---"),
            ('p', ".--."),
            ('q', "--.-"),
            ('r', ".-."),
            ('s', "..."),
            ('t', "-"),
            ('u', "..-"),
            ('v', "...-"),
            ('w', ".--"),
            ('x', "-..-"),
            ('y', "-.--"),
            ('z', "--.."),
        ]);

        let morse_code_machine: MorseCodeMachine = MorseCodeMachine { keys };

        return morse_code_machine;
    }

    pub fn send(
        &self,
        pin: &mut Pin<Gpio15, FunctionSio<SioOutput>, PullDown>,
        delay: &mut Delay,
        message: &str,
    ) -> () {
        for (_, char) in message.to_lowercase().chars().enumerate() {
            if let Some(sequence) = self.keys.get(&char) {
                for (_, char) in sequence.chars().enumerate() {
                    if char == '.' {
                        pin.set_low().unwrap();
                        delay.delay_ms(60);
                    } else {
                        pin.set_low().unwrap();
                        delay.delay_ms(180);
                    }

                    pin.set_high().unwrap();
                    delay.delay_ms(100)
                }
            } else {
                // if it's whitespace
                delay.delay_ms(300)
            }

            delay.delay_ms(150)
        }
    }
}

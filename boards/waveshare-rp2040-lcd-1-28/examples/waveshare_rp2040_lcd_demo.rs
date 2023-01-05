//! Example of graphics on the LCD of the Waveshare RP2040-LCD-0.96
//!
//! Draws a red and green line with a blue rectangle.
//! After that it fills the screen line for line, at the end it starts over with
//! another colour, RED, GREEN and BLUE.
#![no_std]
#![no_main]

use cortex_m::delay::Delay;

use embedded_graphics::primitives::Line;
use fugit::RateExtU32;
use hal::pwm;
use panic_halt as _;

use waveshare_rp2040_lcd_1_28::entry;
use waveshare_rp2040_lcd_1_28::{
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        pac,
        pio::PIOExt,
        watchdog::Watchdog,
        Sio,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};

use embedded_graphics::prelude::*;
use embedded_graphics::{
    pixelcolor::Rgb565,
    primitives::{Circle, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Triangle},
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    text::Text,
};

use display_interface_spi::SPIInterface;

use gc9a01a::*;
// use st7735_lcd::{Orientation, ST7735};

const LCD_WIDTH: u32 = 240;
const LCD_HEIGHT: u32 = 240;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the delay for the first core.
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    let (mut _pio, _sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let lcd_dc = pins.gp8.into_push_pull_output();
    let mut _lcd_cs = pins.gp9.into_push_pull_output();
    let mut _lcd_clk = pins.gp10.into_mode::<hal::gpio::FunctionSpi>();
    let mut _lcd_mosi = pins.gp11.into_mode::<hal::gpio::FunctionSpi>();
    let lcd_rst = pins
        .gp12
        .into_push_pull_output_in_state(hal::gpio::PinState::High);

    let mut _lcd_bl = pins
        .gp25
        .into_mode::<rp2040_hal::gpio::FunctionPwm>();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI1);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let spi_interface = SPIInterface::new(spi, lcd_dc, _lcd_cs);

    // initialize PWM for backlight
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let mut pwm = pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM6 to GPIO 13
    let mut channel = pwm.channel_b;
    channel.output_to(_lcd_bl);
    

    let mut display = GC9A01A::new(spi_interface, lcd_rst, channel);

    // Bring out of reset
    display.reset(&mut delay).unwrap();
    // Turn on backlight
    display.set_backlight(55000);
    // Initialize registers
    display.initialize(&mut delay).unwrap();
    // Fill screen with single color
    display.clear(Rgb565::CSS_FOREST_GREEN).unwrap();

    
    // Create a new character style
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

    let display_box = display.bounding_box();
    
    // Create a text at position (20, 30) and draw it using the previously defined style
    Text::new("Hello Rust!", display_box.center(), style).draw(&mut display).unwrap();

    delay.delay_ms(3000);

    let lcd_center = display_box.center();
    let lcd_diameter = display_box.size.width;

    let bounding_circle = Circle::with_center(lcd_center, lcd_diameter );
    let lcd_zero = Point::zero();
    let lcd_max_corner = Point::new((LCD_WIDTH - 1) as i32, (LCD_HEIGHT - 1) as i32);

    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLUE)
        .build();

    bounding_circle.into_styled(style).draw(&mut display).unwrap();

    Circle::new(lcd_zero, lcd_diameter)
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();

    Circle::new(Point::new(1,1), lcd_diameter - 2 )
        .into_styled(style)
        .draw(&mut display)
        .unwrap();

    Line::new(lcd_zero, lcd_max_corner)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1))
        .draw(&mut display)
        .unwrap();

    Line::new(
        Point::new(0, (LCD_HEIGHT - 1) as i32),
        Point::new((LCD_WIDTH - 1) as i32, 0),
    )
    .into_styled(PrimitiveStyle::with_stroke(Rgb565::GREEN, 1))
    .draw(&mut display)
    .unwrap();

    // Infinite colour wheel loop
    let mut l: i32 = 0;
    let mut c = Rgb565::RED;
    loop {
        Line::new(Point::new(0, l), Point::new((LCD_WIDTH - 1) as i32, l))
            .into_styled(PrimitiveStyle::with_stroke(c, 1))
            .draw(&mut display)
            .unwrap();
        delay.delay_ms(10);
        l += 1;
        if l == LCD_HEIGHT as i32 {
            l = 0;
            c = match c {
                Rgb565::RED => Rgb565::GREEN,
                Rgb565::GREEN => Rgb565::BLUE,
                _ => Rgb565::RED,
            }
        }
    }
}

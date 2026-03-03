#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use microbit::Board;
use microbit::hal::timer::Timer;
use embedded_hal::digital::OutputPin;
use embedded_hal::delay::DelayNs;

#[entry]
fn main() -> ! 
{
    let board = Board::take().unwrap();
    let mut timer = Timer::new(board.TIMER0);

    let mut rows = board.display_pins.row1;
    let mut cols = board.display_pins.col1;

    cols.set_low().unwrap();

    loop 
    {
        rows.set_high().unwrap();
        timer.delay_ms(500);
        rows.set_low().unwrap();
        timer.delay_ms(500);
    }
}
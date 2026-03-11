//! Status LED — Visual Feedback via micro:bit LED Matrix
//!
//! Uses a single LED from the 5×5 matrix (Row 3 / Col 1) to indicate
//! the robot's state.  This keeps the control tick ISR focused on
//! the control loop while still providing useful visual feedback.
//!
//! Patterns:
//!   TRACK  → solid on
//!   COAST  → fast blink (~8 Hz)
//!   RECOV  → slow blink (~2 Hz)
//!   STOP   → off

use microbit::hal::gpio::p0::{P0_04, P0_15};
use microbit::hal::gpio::{Output, PushPull};
use embedded_hal::digital::OutputPin;

/// Encapsulates a single LED and its blink counter.
pub struct StatusLed
{
    row: P0_15<Output<PushPull>>,
    col: P0_04<Output<PushPull>>,
    tick_count: u16,
}

impl StatusLed
{
    pub fn new(
        row: P0_15<Output<PushPull>>,
        col: P0_04<Output<PushPull>>,
    ) -> Self
    {
        Self { row, col, tick_count: 0 }
    }

    /// Update LED each control tick.  `state_name` comes from the
    /// strategy module's `state_name()` method.
    #[inline]
    pub fn update(&mut self, state_name: &str)
    {
        self.tick_count = self.tick_count.wrapping_add(1);

        let on = match state_name
        {
            "TRACK" => true,
            "COAST" => (self.tick_count >> 6) & 1 == 0, // ~8 Hz
            "RECOV" => (self.tick_count >> 8) & 1 == 0, // ~2 Hz
            _       => false,                            // STOP
        };

        if on
        {
            let _ = self.col.set_low();  // sink column
            let _ = self.row.set_high(); // source row
        }
        else
        {
            let _ = self.row.set_low();
        }
    }
}
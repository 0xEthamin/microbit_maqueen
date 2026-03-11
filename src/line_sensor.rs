//! Line Sensor — IR Line-Following Sensors with Edge Detection
//!
//! Hardware wiring (micro:bit V1 edge-pin → nRF51 GPIO):
//!   Edge P13 → P0_23  (left sensor)
//!   Edge P14 → P0_22  (right sensor)
//!
//! Electrical behavior observed on real Maqueen ROB0148:
//!   GPIO Low  = sensor LED reflects → surface is BLACK (on line)
//!   GPIO High = no reflection        → surface is WHITE (off line)
//!
//! This module adds **edge detection** on top of the raw binary reading.
//! The caller receives both the instantaneous state and transition flags
//! so that higher layers can react to *changes* without polling history.

use nrf51_hal::gpio::p0::{P0_22, P0_23};
use nrf51_hal::gpio::{Floating, Input};
use embedded_hal::digital::InputPin;

// ---------------------------------------------------------------------------
// LineReading — the four possible instantaneous sensor states
// ---------------------------------------------------------------------------

/// Exhaustive enumeration of the two-sensor combination.
///
/// Named from the line's perspective: `LeftOnLine` means the left sensor
/// sees the black line (the robot has drifted *right* of center).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LineReading
{
    /// Both sensors see the line — robot is centered.
    BothOnLine,
    /// Only left sensor on line — robot drifted right.
    LeftOnLine,
    /// Only right sensor on line — robot drifted left.
    RightOnLine,
    /// Neither sensor on line — line is lost.
    NoneOnLine,
}

impl LineReading
{
    /// Map to a signed position error in Q8 fixed-point.
    ///
    /// Convention (consistent with differential_drive):
    ///   negative = line is to the LEFT  → must steer left
    ///   positive = line is to the RIGHT → must steer right
    ///   zero     = centered
    ///   None     = unmeasurable (line lost)
    #[inline]
    pub const fn to_error_q8(self) -> Option<i32>
    {
        match self
        {
            Self::BothOnLine  => Some(0),
            Self::LeftOnLine  => Some(-256),  // -1.0 in Q8
            Self::RightOnLine => Some(256),   //  1.0 in Q8
            Self::NoneOnLine  => None,
        }
    }

    /// `true` when at least one sensor sees the line.
    #[inline]
    pub const fn has_line(self) -> bool
    {
        !matches!(self, Self::NoneOnLine)
    }
}

// ---------------------------------------------------------------------------
// SensorSnapshot — reading + transition metadata
// ---------------------------------------------------------------------------

/// A single-tick sensor observation enriched with edge information.
#[derive(Clone, Copy, Debug)]
pub struct SensorSnapshot
{
    /// Current binary reading.
    pub reading: LineReading,
    /// `true` on the exact tick where the reading changed.
    pub edge_detected: bool,
}

// ---------------------------------------------------------------------------
// LineSensor — owns the two GPIO pins
// ---------------------------------------------------------------------------

/// Stateful line-sensor reader.
///
/// Keeps the previous reading so it can report edges.  Two GPIO reads
/// per call — roughly 4 cycles on Cortex-M0.
pub struct LineSensor
{
    left: P0_23<Input<Floating>>,
    right: P0_22<Input<Floating>>,
    previous: LineReading,
}

impl LineSensor
{
    pub fn new(left: P0_23<Input<Floating>>, right: P0_22<Input<Floating>>) -> Self
    {
        Self
        {
            left,
            right,
            previous: LineReading::BothOnLine,
        }
    }

    /// Sample both sensors and return an enriched snapshot.
    ///
    /// Cost: 2 GPIO reads + 1 comparison.  No heap, no float.
    #[inline]
    pub fn read(&mut self) -> SensorSnapshot
    {
        let left_on  = self.left.is_low().unwrap_or(false);
        let right_on = self.right.is_low().unwrap_or(false);

        let reading = match (left_on, right_on)
        {
            (true,  true)  => LineReading::BothOnLine,
            (true,  false) => LineReading::LeftOnLine,
            (false, true)  => LineReading::RightOnLine,
            (false, false) => LineReading::NoneOnLine,
        };

        let edge_detected = reading != self.previous;
        self.previous = reading;

        SensorSnapshot { reading, edge_detected }
    }
}
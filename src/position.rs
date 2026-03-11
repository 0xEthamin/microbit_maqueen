//! Position Estimator — Continuous Position from Binary Sensors
//!
//! The raw line sensors only provide three useful states (-1, 0, +1).
//! Feeding those step-changes directly into a PID makes the derivative
//! term explode and the proportional term coarse.
//!
//! This module maintains a **smooth estimated position** in Q8 that:
//!   1. Snaps to the known anchor when a sensor edge occurs.
//!   2. Interpolates between edges using a first-order extrapolation
//!      (last known drift rate).
//!   3. Decays gently toward center when both sensors see the line,
//!      preventing integral-like drift in the estimate itself.
//!
//! The result: the PID receives a quasi-continuous error signal even
//! though the underlying hardware is binary.

use crate::fixedpoint::{clamp_symmetric, Q8_ONE};
use crate::line_sensor::{LineReading, SensorSnapshot};

/// Maximum absolute value the position estimate can reach (in Q8).
/// Beyond ±2.0 the robot is clearly off-track; clamping here
/// prevents runaway extrapolation.
const POSITION_LIMIT_Q8: i32 = 2 * Q8_ONE; // ±512

/// Rate at which the estimate decays toward zero when both sensors
/// are on the line.  Expressed as (256 - alpha) / 256 per tick.
/// 252/256 ≈ 0.984 → half-life ~43 ticks (43 ms).
const CENTER_DECAY_FACTOR: i32 = 252;

/// Smoothing factor for drift-rate EMA.  64/256 = 0.25.
const DRIFT_ALPHA: i32 = 64;

/// Maintains a smooth position estimate between sensor edges.
pub struct PositionEstimator
{
    /// Current estimated position in Q8.  Negative = left of center.
    position: i32,
    /// Smoothed drift rate in Q8 per tick.
    drift_rate: i32,
    /// Ticks since last sensor edge (for extrapolation weighting).
    ticks_since_edge: u16,
}

impl PositionEstimator
{
    pub const fn new() -> Self
    {
        Self
        {
            position: 0,
            drift_rate: 0,
            ticks_since_edge: 0,
        }
    }

    /// Feed one sensor tick and return the refined position in Q8.
    ///
    /// Called once per control tick (1 kHz).
    /// Cost: ~15 integer ops, zero division, zero float.
    #[inline]
    pub fn update(&mut self, snapshot: SensorSnapshot) -> i32
    {
        if snapshot.edge_detected
        {
            self.handle_edge(snapshot.reading);
        }
        else
        {
            self.handle_steady(snapshot.reading);
        }

        self.position = clamp_symmetric(self.position, POSITION_LIMIT_Q8);
        self.position
    }

    /// On a sensor transition: snap to the known anchor and update
    /// the drift rate from the position delta.
    fn handle_edge(&mut self, reading: LineReading)
    {
        if let Some(anchor) = reading.to_error_q8()
        {
            // Compute instantaneous drift from the jump.
            let delta = anchor - self.position;

            // EMA on drift rate: avoids single-tick spikes.
            self.drift_rate = (DRIFT_ALPHA * delta
                + (Q8_ONE - DRIFT_ALPHA) * self.drift_rate)
                >> 8;

            self.position = anchor;
        }
        // If reading is NoneOnLine the anchor is unknown;
        // we keep extrapolating in handle_steady next tick.

        self.ticks_since_edge = 0;
    }

    /// Between edges: extrapolate or decay depending on the reading.
    fn handle_steady(&mut self, reading: LineReading)
    {
        self.ticks_since_edge = self.ticks_since_edge.saturating_add(1);

        match reading
        {
            LineReading::BothOnLine =>
            {
                // Robot is centered — decay position toward zero.
                self.position =
                    (self.position * CENTER_DECAY_FACTOR) >> 8;
                // Also decay drift rate.
                self.drift_rate =
                    (self.drift_rate * CENTER_DECAY_FACTOR) >> 8;
            }
            LineReading::NoneOnLine =>
            {
                // Line lost — extrapolate with last drift rate.
                // Attenuate over time to avoid runaway.
                if self.ticks_since_edge < 500
                {
                    self.position += self.drift_rate;
                }
            }
            _ =>
            {
                // Single sensor on line: small extrapolation nudge.
                // The anchor value is the dominant signal; drift adds
                // a sub-pixel refinement.
                self.position += self.drift_rate >> 2;
            }
        }
    }

    /// Hard reset (e.g. after a full stop / restart).
    pub fn reset(&mut self)
    {
        self.position = 0;
        self.drift_rate = 0;
        self.ticks_since_edge = 0;
    }
}
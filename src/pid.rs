//! PID Controller — Integer Fixed-Point, Derivative-Filtered
//!
//! All arithmetic uses i32 in Q8 (×256).  On Cortex-M0 this compiles
//! to pure Thumb-2 integer ops: ~20 cycles per `update()` versus
//! ~200 for the equivalent soft-float.
//!
//! Improvements over the v0.2 controller:
//!   • EMA low-pass filter on the derivative term — eliminates
//!     the spikes caused by binary sensor transitions.
//!   • Zero-crossing integral reset — when the error changes sign
//!     the robot has crossed the line; flushing the integrator
//!     prevents post-turn overshoot.
//!   • Configurable warm-up period before the integrator engages.

use crate::fixedpoint::{clamp_symmetric, from_q8, Q8_ONE};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// PID tuning parameters.  All gains are in Q8.
///
/// `Kp = 256` means an effective proportional gain of 1.0.
#[derive(Clone, Copy, Debug)]
pub struct PidConfig
{
    pub kp: i32,
    pub ki: i32,
    pub kd: i32,
    /// Maximum absolute value of the integral accumulator (Q8).
    pub integral_limit: i32,
    /// Maximum absolute value of the controller output (raw PWM units).
    pub output_limit: i32,
    /// EMA smoothing factor for the derivative (Q8, 0..256).
    /// Lower = heavier filtering.  64 ≈ 0.25 → half-life ~3 ticks.
    pub derivative_alpha: i32,
    /// Number of valid samples before the integrator activates.
    pub warmup_ticks: u8,
}

impl PidConfig
{
    /// Tuning optimized for Maqueen with the position estimator
    /// feeding Q8 errors.  The estimator outputs smooth values
    /// so Kd can be higher than with raw binary sensors.
    pub const fn default_line_follower() -> Self
    {
        Self
        {
            kp: 200,               // ~0.78
            ki: 4,                 // ~0.016  (gentle steady-state trim)
            kd: 450,               // ~1.76   (safe with filtered derivative)
            integral_limit: 3000,  // ~11.7 in raw units after >>8
            output_limit: 255,
            derivative_alpha: 80,  // ~0.31
            warmup_ticks: 4,
        }
    }
}

// ---------------------------------------------------------------------------
// Controller state
// ---------------------------------------------------------------------------

/// PID controller.  Zero-alloc, const-constructible, no floats.
#[derive(Debug)]
pub struct PidController
{
    config: PidConfig,
    integral: i32,
    prev_error: i32,
    filtered_derivative: i32,
    valid_count: u8,
    prev_sign_positive: bool,
}

impl PidController
{
    pub const fn new(config: PidConfig) -> Self
    {
        Self
        {
            config,
            integral: 0,
            prev_error: 0,
            filtered_derivative: 0,
            valid_count: 0,
            prev_sign_positive: false,
        }
    }

    /// Compute the control output from a Q8 position error.
    ///
    /// Returns a signed correction in raw PWM units (not Q8).
    /// Positive correction → steer right, negative → steer left.
    #[inline]
    pub fn update(&mut self, error_q8: i32) -> i32
    {
        // -- Proportional --
        let p_term = self.config.kp * error_q8;

        // -- Integral with zero-crossing reset & warm-up --
        let sign_positive = error_q8 >= 0;
        if self.valid_count >= self.config.warmup_ticks
            && sign_positive != self.prev_sign_positive
            && self.valid_count > 0
        {
            // Error crossed zero → line crossed → flush integrator.
            self.integral = 0;
        }
        self.prev_sign_positive = sign_positive;

        if self.valid_count < 255
        {
            self.valid_count += 1;
        }
        if self.valid_count >= self.config.warmup_ticks
        {
            self.integral += self.config.ki * error_q8;
            self.integral = clamp_symmetric(
                self.integral,
                self.config.integral_limit,
            );
        }

        // -- Derivative with EMA filter --
        let raw_derivative = error_q8 - self.prev_error;
        self.prev_error = error_q8;

        // EMA: filtered = α·raw + (1-α)·prev_filtered   (all Q8)
        let alpha = self.config.derivative_alpha;
        self.filtered_derivative = (alpha * raw_derivative
            + (Q8_ONE - alpha) * self.filtered_derivative)
            >> 8;

        let d_term = self.config.kd * self.filtered_derivative;

        // -- Sum & scale back from Q8×Q8 to raw --
        let raw = from_q8(p_term + self.integral + d_term);
        clamp_symmetric(raw, self.config.output_limit)
    }

    /// Reset all internal state.  Call when re-acquiring the line
    /// or transitioning from a non-PID mode.
    pub fn reset(&mut self)
    {
        self.integral = 0;
        self.prev_error = 0;
        self.filtered_derivative = 0;
        self.valid_count = 0;
    }
}
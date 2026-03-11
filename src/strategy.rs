//! Strategy — Line-Following State Machine
//!
//! Four states govern the robot's behavior:
//!
//!   Tracking  → normal PID-controlled differential drive.
//!   Coasting  → line just lost; coast on last correction, decaying.
//!   Recovering → line lost for a while; controlled spin toward last
//!                known direction with progressive acceleration.
//!   Stopped   → recovery timed out; emergency halt.
//!
//! The key insight: a Coasting phase between Tracking and Recovering
//! lets the robot survive sharp corners without immediately spinning.
//! On a typical 90° bend the line reappears within 50–200 ms; the
//! robot glides through instead of panicking.
//!
//! All timing constants assume a 1 kHz tick rate.

use crate::fixedpoint::clamp;
use crate::line_sensor::{LineReading, SensorSnapshot};
use crate::motor::DrivePower;
use crate::pid::PidController;
use crate::position::PositionEstimator;

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/// Nominal forward speed during tracking (PWM units, 0–255).
const BASE_SPEED: i32 = 85;

/// Minimum base speed when approaching a detected curve.
const MIN_CURVE_SPEED: i32 = 45;

/// Speed reduction per consecutive tick of single-sensor reading.
/// Spread over ~100 ticks this brings BASE_SPEED down to MIN smoothly.
const CURVE_DECEL_PER_TICK: i32 = 1;

/// Duration of the coasting phase before switching to recovery (ticks).
const COAST_DURATION: u16 = 150; // 150 ms

/// Decay factor applied to the coasting correction each tick (Q8).
/// 250/256 ≈ 0.977 → correction halves in ~30 ticks.
const COAST_DECAY: i32 = 250;

/// Initial spin speed during recovery (PWM units).
const RECOVERY_SPEED_MIN: i32 = 35;

/// Maximum spin speed during recovery (PWM units).
const RECOVERY_SPEED_MAX: i32 = 70;

/// Ticks over which recovery spin accelerates from MIN to MAX.
const RECOVERY_RAMP_TICKS: u16 = 400;

/// Total recovery timeout before full stop (ticks).
const RECOVERY_TIMEOUT: u16 = 2500; // 2.5 s

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction
{
    Left,
    Right,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State
{
    Tracking,
    Coasting,
    Recovering,
    Stopped,
}

// ---------------------------------------------------------------------------
// LineFollower
// ---------------------------------------------------------------------------

/// Top-level line-following controller.
///
/// Owns the PID controller and position estimator.  The caller
/// supplies raw sensor snapshots; this struct returns motor commands.
pub struct LineFollower
{
    state: State,
    pid: PidController,
    estimator: PositionEstimator,

    /// Last direction the line was seen (for recovery).
    last_direction: Direction,

    /// Consecutive ticks where only one sensor sees the line.
    single_sensor_streak: u16,

    /// Ticks spent in the current non-tracking state.
    phase_ticks: u16,

    /// Last PID correction — carried into coasting.
    last_correction: i32,

    /// Dynamic base speed, reduced in curves.
    current_base_speed: i32,
}

impl LineFollower
{
    pub const fn new(pid: PidController) -> Self
    {
        Self
        {
            state: State::Tracking,
            pid,
            estimator: PositionEstimator::new(),
            last_direction: Direction::Unknown,
            single_sensor_streak: 0,
            phase_ticks: 0,
            last_correction: 0,
            current_base_speed: BASE_SPEED,
        }
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// Main tick entry point.  Call exactly once per 1 kHz period.
    pub fn tick(&mut self, snapshot: SensorSnapshot) -> DrivePower
    {
        // Update direction memory regardless of state.
        self.update_direction(snapshot.reading);

        match self.state
        {
            State::Tracking   => self.do_tracking(snapshot),
            State::Coasting   => self.do_coasting(snapshot),
            State::Recovering => self.do_recovering(snapshot),
            State::Stopped    => DrivePower::STOP,
        }
    }

    /// Human-readable state name for LED / debug.
    pub fn state_name(&self) -> &'static str
    {
        match self.state
        {
            State::Tracking   => "TRACK",
            State::Coasting   => "COAST",
            State::Recovering => "RECOV",
            State::Stopped    => "STOP",
        }
    }

    /// Full reset to initial conditions.
    pub fn reset(&mut self)
    {
        self.state = State::Tracking;
        self.pid.reset();
        self.estimator.reset();
        self.last_direction = Direction::Unknown;
        self.single_sensor_streak = 0;
        self.phase_ticks = 0;
        self.last_correction = 0;
        self.current_base_speed = BASE_SPEED;
    }

    // ------------------------------------------------------------------
    // State handlers
    // ------------------------------------------------------------------

    /// Tracking: PID active, adaptive speed.
    fn do_tracking(&mut self, snapshot: SensorSnapshot) -> DrivePower
    {
        if !snapshot.reading.has_line()
        {
            // Transition → Coasting.
            self.state = State::Coasting;
            self.phase_ticks = 0;
            return self.do_coasting(snapshot);
        }

        // Adaptive speed: slow down when only one sensor is triggered
        // for many consecutive ticks (approaching a curve).
        self.update_curve_detection(snapshot.reading);

        let position = self.estimator.update(snapshot);
        let correction = self.pid.update(position);
        self.last_correction = correction;

        DrivePower::from_differential(
            self.current_base_speed,
            correction,
            false, // never reverse in tracking
        )
    }

    /// Coasting: line lost recently.  Coast on decaying last correction.
    /// If line reappears → back to Tracking.
    /// If coast timer expires → Recovering.
    fn do_coasting(&mut self, snapshot: SensorSnapshot) -> DrivePower
    {
        if snapshot.reading.has_line()
        {
            return self.transition_to_tracking(snapshot);
        }

        // Keep the estimator running so it extrapolates.
        let _position = self.estimator.update(snapshot);

        self.phase_ticks += 1;

        if self.phase_ticks >= COAST_DURATION
        {
            self.state = State::Recovering;
            self.phase_ticks = 0;
            self.pid.reset();
            return self.do_recovering(snapshot);
        }

        // Decay the correction smoothly.
        self.last_correction =
            (self.last_correction * COAST_DECAY) >> 8;

        // Coast at reduced speed.
        let coast_speed = clamp(
            self.current_base_speed - (self.phase_ticks as i32 / 4),
            MIN_CURVE_SPEED,
            BASE_SPEED,
        );

        DrivePower::from_differential(
            coast_speed,
            self.last_correction,
            false,
        )
    }

    /// Recovering: controlled spin toward last known direction.
    /// Speed ramps up progressively to avoid violent jolts.
    fn do_recovering(&mut self, snapshot: SensorSnapshot) -> DrivePower
    {
        if snapshot.reading.has_line()
        {
            return self.transition_to_tracking(snapshot);
        }

        self.phase_ticks += 1;

        if self.phase_ticks >= RECOVERY_TIMEOUT
        {
            self.state = State::Stopped;
            return DrivePower::STOP;
        }

        // Progressive ramp from MIN to MAX spin speed.
        let ramp_frac = if self.phase_ticks < RECOVERY_RAMP_TICKS
        {
            self.phase_ticks as i32
        }
        else
        {
            RECOVERY_RAMP_TICKS as i32
        };

        let spin_speed = RECOVERY_SPEED_MIN
            + ((RECOVERY_SPEED_MAX - RECOVERY_SPEED_MIN) * ramp_frac)
                / (RECOVERY_RAMP_TICKS as i32);

        // Spin direction: positive correction → right wheel slower.
        let spin_correction = match self.last_direction
        {
            Direction::Left  => -spin_speed,
            Direction::Right | Direction::Unknown => spin_speed,
        };

        DrivePower::from_differential(0, spin_correction, true)
    }

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------

    /// Common transition back to Tracking from any other state.
    fn transition_to_tracking(
        &mut self,
        snapshot: SensorSnapshot,
    ) -> DrivePower
    {
        self.state = State::Tracking;
        self.phase_ticks = 0;
        self.single_sensor_streak = 0;
        self.current_base_speed = BASE_SPEED;
        // Don't reset PID — the estimator has been running, so
        // the derivative history is still relevant.
        self.do_tracking(snapshot)
    }

    /// Track how long a single sensor has been triggered (curve detection).
    fn update_curve_detection(&mut self, reading: LineReading)
    {
        match reading
        {
            LineReading::LeftOnLine | LineReading::RightOnLine =>
            {
                self.single_sensor_streak =
                    self.single_sensor_streak.saturating_add(1);

                // Gradually reduce base speed in curves.
                let reduction =
                    (self.single_sensor_streak as i32) * CURVE_DECEL_PER_TICK;
                self.current_base_speed = clamp(
                    BASE_SPEED - reduction,
                    MIN_CURVE_SPEED,
                    BASE_SPEED,
                );
            }
            _ =>
            {
                // Both on line or none on line → reset streak.
                if self.single_sensor_streak > 0
                {
                    self.single_sensor_streak = 0;
                    self.current_base_speed = BASE_SPEED;
                }
            }
        }
    }

    /// Remember which side the line was last seen.
    fn update_direction(&mut self, reading: LineReading)
    {
        match reading
        {
            LineReading::LeftOnLine  =>
            {
                self.last_direction = Direction::Left;
            }
            LineReading::RightOnLine =>
            {
                self.last_direction = Direction::Right;
            }
            _ => {}
        }
    }
}
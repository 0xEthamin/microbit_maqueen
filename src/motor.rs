//! Motor Driver — Maqueen ROB0148 I2C Motor Control
//!
//! The Maqueen on-board MCU acts as an I2C slave at address 0x10.
//! Each motor command is a 3-byte write: [REGISTER, DIRECTION, SPEED].
//!
//! This module owns the TWI0 peripheral and exposes a safe, typed API
//! that prevents accidental reverse-drive during normal line following.

use nrf51_hal::twi::Twi;
use nrf51_hal::pac::TWI0;

use crate::fixedpoint::clamp;

// ---------------------------------------------------------------------------
// I2C protocol constants
// ---------------------------------------------------------------------------

const MAQUEEN_ADDR: u8    = 0x10;
const REG_MOTOR_LEFT: u8  = 0x00;
const REG_MOTOR_RIGHT: u8 = 0x02;
const DIR_FORWARD: u8     = 0x00;
const DIR_BACKWARD: u8    = 0x01;

// ---------------------------------------------------------------------------
// MotorCommand — single-wheel command
// ---------------------------------------------------------------------------

/// Signed speed for one wheel: [-255, +255].
/// Positive = forward, negative = backward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorCommand(i16);

impl MotorCommand
{
    /// Construct with saturation to [-255, +255].
    #[inline]
    pub const fn new(value: i16) -> Self
    {
        if value > 255 { Self(255) }
        else if value < -255 { Self(-255) }
        else { Self(value) }
    }

    /// Construct clamped to [0, 255] — never reverses.
    /// Use during normal tracking to guarantee no U-turns.
    #[inline]
    pub const fn forward_only(value: i16) -> Self
    {
        if value > 255 { Self(255) }
        else if value < 0 { Self(0) }
        else { Self(value) }
    }

    pub const STOP: Self = Self(0);

    /// Decompose into (direction_byte, speed_byte) for the I2C wire.
    #[inline]
    const fn decompose(self) -> (u8, u8)
    {
        if self.0 >= 0
        {
            (DIR_FORWARD, self.0 as u8)
        }
        else
        {
            (DIR_BACKWARD, (-self.0) as u8)
        }
    }

    #[inline]
    pub const fn raw(self) -> i16
    {
        self.0
    }
}

// ---------------------------------------------------------------------------
// DrivePower — command pair for both wheels
// ---------------------------------------------------------------------------

/// Power pair sent to the motor driver each tick.
#[derive(Clone, Copy, Debug)]
pub struct DrivePower
{
    pub left: MotorCommand,
    pub right: MotorCommand,
}

impl DrivePower
{
    pub const STOP: Self = Self
    {
        left: MotorCommand::STOP,
        right: MotorCommand::STOP,
    };

    /// Build a differential-drive command from base speed and correction.
    ///
    /// `base_speed`: nominal forward speed (0..255).
    /// `correction`: signed PID output.  Positive → steer right
    /// (slow the right wheel, speed up the left).
    ///
    /// In `allow_reverse` mode the inner wheel can go negative (spin
    /// recovery).  Otherwise the floor is zero (smooth tracking).
    #[inline]
    pub fn from_differential(
        base_speed: i32,
        correction: i32,
        allow_reverse: bool,
    ) -> Self
    {
        let left_raw  = clamp(base_speed + correction, -255, 255) as i16;
        let right_raw = clamp(base_speed - correction, -255, 255) as i16;

        if allow_reverse
        {
            Self
            {
                left:  MotorCommand::new(left_raw),
                right: MotorCommand::new(right_raw),
            }
        }
        else
        {
            Self
            {
                left:  MotorCommand::forward_only(left_raw),
                right: MotorCommand::forward_only(right_raw),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// MotorDriver — owns the TWI bus
// ---------------------------------------------------------------------------

/// Motor driver over TWI (I2C) bus.  Owns the TWI0 peripheral.
pub struct MotorDriver
{
    twi: Twi<TWI0>,
}

impl MotorDriver
{
    pub fn new(twi: Twi<TWI0>) -> Self
    {
        Self { twi }
    }

    /// Drive both motors in a single call (two I2C transactions).
    pub fn drive(&mut self, power: DrivePower) -> Result<(), nrf51_hal::twi::Error>
    {
        let (ld, lv) = power.left.decompose();
        let (rd, rv) = power.right.decompose();

        self.twi.write(MAQUEEN_ADDR, &[REG_MOTOR_LEFT, ld, lv])?;
        self.twi.write(MAQUEEN_ADDR, &[REG_MOTOR_RIGHT, rd, rv])?;
        Ok(())
    }

    /// Emergency stop — both wheels to zero.
    #[inline]
    pub fn stop(&mut self) -> Result<(), nrf51_hal::twi::Error>
    {
        self.drive(DrivePower::STOP)
    }
}
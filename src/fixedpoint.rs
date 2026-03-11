//! Fixed-Point Arithmetic Utilities
//!
//! All numeric computations in this crate use Q8 fixed-point (i32 scaled by 256).
//! This module centralizes the constants and helper functions so that every
//! consumer speaks the same numeric language.
//!
//! Why Q8?  On Cortex-M0 without FPU, a single `f32` multiply costs ~30 cycles
//! through libgcc soft-float.  A Q8 multiply is one `i32` multiply + one
//! arithmetic shift: 2 cycles.  Over a full PID iteration at 1 kHz the
//! saving is substantial.

/// Number of fractional bits in Q8 representation.
pub const Q8_SHIFT: i32 = 8;

/// Multiplicative unit in Q8: 1.0 == 256.
pub const Q8_ONE: i32 = 1 << Q8_SHIFT;

/// Convert Q8 back to plain integer (truncates toward zero).
#[inline(always)]
pub const fn from_q8(value: i32) -> i32
{
    value >> Q8_SHIFT
}

/// Saturate `value` to the symmetric range `[-limit, +limit]`.
///
/// Both arguments are in the same numeric domain (both Q8 or both raw).
#[inline(always)]
pub const fn clamp_symmetric(value: i32, limit: i32) -> i32
{
    if value > limit
    {
        limit
    }
    else if value < -limit
    {
        -limit
    }
    else
    {
        value
    }
}

/// Saturate `value` to `[min, max]`.
#[inline(always)]
pub const fn clamp(value: i32, min: i32, max: i32) -> i32
{
    if value < min
    {
        min
    }
    else if value > max
    {
        max
    }
    else
    {
        value
    }
}
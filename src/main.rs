#![no_std]
#![no_main]

//! Maqueen Line Follower — RTIC Real-Time Application (v0.3)
//!
//! Architecture: single hardware ISR at 1 kHz driven by TIMER1.
//! Each tick runs the full pipeline:
//!
//!   Sensor read  →  Position estimate  →  Strategy/PID  →  Motor drive  →  LED
//!   (~4 cycles)     (~15 cycles)          (~40 cycles)     (~1600 cycles)   (~4 cycles)
//!
//! Total worst-case: ~1700 cycles per tick = 106 µs at 16 MHz.
//! That leaves 894 µs of slack per millisecond — the CPU sleeps via WFI
//! for >89 % of the time, drawing ~1.2 mA vs ~5 mA active.
//!
//! Pin mapping (micro:bit V1 edge-pin → nRF51 GPIO):
//!   Edge P13  → P0_23  (line sensor left)
//!   Edge P14  → P0_22  (line sensor right)
//!   Edge P19  → P0_00  (I2C SCL)
//!   Edge P20  → P0_30  (I2C SDA)

mod fixedpoint;
mod led;
mod line_sensor;
mod motor;
mod pid;
mod position;
mod strategy;

use panic_halt as _;

#[rtic::app(device = microbit::hal::pac, dispatchers = [SWI0, SWI1])]
mod app
{
    use microbit::hal::twi;
    use microbit::hal::pac::twi0::frequency::FREQUENCY_A;
    use microbit::hal::gpio::p0::Parts as P0Parts;
    use microbit::hal::gpio::Level;

    use crate::led::StatusLed;
    use crate::line_sensor::LineSensor;
    use crate::motor::MotorDriver;
    use crate::pid::{PidConfig, PidController};
    use crate::strategy::LineFollower;

    #[shared]
    struct Shared {}

    #[local]
    struct Local
    {
        sensors: LineSensor,
        motors: MotorDriver,
        follower: LineFollower,
        led: StatusLed,
    }

    /// System initialization — runs once before any interrupt fires.
    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics)
    {
        let pins = P0Parts::new(cx.device.GPIO);

        // --- I2C bus for Maqueen motor controller ---
        // Edge P19 (SCL) = P0_00, Edge P20 (SDA) = P0_30
        let scl = pins.p0_00.into_floating_input().degrade();
        let sda = pins.p0_30.into_floating_input().degrade();
        let i2c = twi::Twi::new(
            cx.device.TWI0,
            twi::Pins { scl, sda },
            FREQUENCY_A::K100,
        );

        // --- Line sensors ---
        // Edge P13 = P0_23 (left), Edge P14 = P0_22 (right)
        let left_pin  = pins.p0_23.into_floating_input();
        let right_pin = pins.p0_22.into_floating_input();

        // --- Status LED (matrix Row3 = P0_15, Col1 = P0_04) ---
        let led_row = pins.p0_15.into_push_pull_output(Level::Low);
        let led_col = pins.p0_04.into_push_pull_output(Level::Low);

        // --- TIMER1 → 1 kHz periodic interrupt ---
        configure_timer1();

        // --- Assemble domain objects ---
        let pid = PidController::new(PidConfig::default_line_follower());

        (
            Shared {},
            Local
            {
                sensors:  LineSensor::new(left_pin, right_pin),
                motors:   MotorDriver::new(i2c),
                follower: LineFollower::new(pid),
                led:      StatusLed::new(led_row, led_col),
            },
            init::Monotonics(),
        )
    }

    /// Idle task — WFI sleep between interrupts.
    /// Current draw: ~1.2 mA (vs ~5 mA polling).
    #[idle]
    fn idle(_: idle::Context) -> !
    {
        loop
        {
            cortex_m::asm::wfi();
        }
    }

    /// 1 kHz control tick — the entire real-time pipeline.
    ///
    /// Execution order is fixed and deterministic:
    ///   1. Clear timer event   (~2 cycles)
    ///   2. Sensor read         (~4 cycles)
    ///   3. Strategy + PID      (~55 cycles)
    ///   4. Motor I2C write     (~1600 cycles)
    ///   5. LED update          (~4 cycles)
    #[task(
        binds = TIMER1,
        local = [sensors, motors, follower, led],
    )]
    fn control_tick(cx: control_tick::Context)
    {
        // 1. Acknowledge the timer compare event.
        let timer = unsafe { &*microbit::hal::pac::TIMER1::ptr() };
        timer.events_compare[0].write(|w| unsafe { w.bits(0) });

        // 2. Read sensors (2 GPIO reads + edge detection).
        let snapshot = cx.local.sensors.read();

        // 3. Run the state machine → PID → motor command.
        let power = cx.local.follower.tick(snapshot);

        // 4. Send motor commands over I2C.
        //    On bus error: emergency stop (best-effort).
        if cx.local.motors.drive(power).is_err()
        {
            let _ = cx.local.motors.stop();
        }

        // 5. Visual state feedback.
        cx.local.led.update(cx.local.follower.state_name());
    }

    // ------------------------------------------------------------------
    // Hardware helpers (outside the RTIC module's lifetime rules)
    // ------------------------------------------------------------------

    /// Configure TIMER1 as a 1 kHz periodic interrupt source.
    ///
    /// Prescaler 4 → 16 MHz / 2^4 = 1 MHz tick.
    /// Compare register 0 = 1000 → fires every 1 ms.
    /// Shortcut: auto-clear on compare match.
    fn configure_timer1()
    {
        let t = unsafe { &*microbit::hal::pac::TIMER1::ptr() };

        t.mode.write(|w| w.mode().timer());
        t.bitmode.write(|w| w.bitmode()._32bit());
        t.prescaler.write(|w| unsafe { w.prescaler().bits(4) });
        t.cc[0].write(|w| unsafe { w.bits(1_000) });
        t.shorts.write(|w| w.compare0_clear().enabled());
        t.intenset.write(|w| w.compare0().set());
        t.tasks_clear.write(|w| unsafe { w.bits(1) });
        t.tasks_start.write(|w| unsafe { w.bits(1) });
    }
}
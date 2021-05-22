//! Interrupts

// NOTE: Adapted from cortex-m/src/interrupt.rs
use crate::register::mstatus;

/// Disables all interrupts
#[inline]
pub unsafe fn disable() {
    if cfg!(riscv) {
        mstatus::clear_mie();
    } else {
        unimplemented!();
    }
}

/// Enables all the interrupts
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
#[inline]
pub unsafe fn enable() {
    if cfg!(riscv) {
        mstatus::set_mie();
    } else {
        unimplemented!();
    }
}

/// Execute closure `f` in an interrupt-free context.
///
/// This as also known as a "critical section".
pub fn free<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    let mstatus = mstatus::read();

    // disable interrupts
    if mstatus.mie() {
        unsafe {
            disable();
        }
    }

    let r = f();

    // If the interrupts were active before our `disable` call, then re-enable
    // them. Otherwise, keep them disabled
    if mstatus.mie() {
        unsafe {
            enable();
        }
    }

    r
}

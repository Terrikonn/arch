//! Interrupts

use crate::register::mstatus;

/// Disables all interrupts
pub unsafe fn disable() {
    mstatus::clear_mie();
}

/// Enables all the interrupts
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
pub unsafe fn enable() {
    mstatus::set_mie();
}

/// Execute closure `f` in an interrupt-free context.
///
/// This as also known as a "critical section".
pub fn free<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    // true if the interrupts enabled
    let saved_mstatus = mstatus::read();

    // disable interrupts if enabled
    if saved_mstatus.mie() {
        unsafe {
            disable();
        }
    }

    let ret = f();

    // If the interrupts were active before our `disable` call, then re-enable
    // them. Otherwise, keep them disabled
    if saved_mstatus.mie() {
        unsafe {
            enable();
        }
    }

    // return the result of `f` to the caller
    ret
}

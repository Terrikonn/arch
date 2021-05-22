/// Disables all interrupts
pub unsafe fn disable() {
    cfg_if! {
        if #[cfg(riscv)] {
            crate::intrinsics::riscv::interrupts::disable();
        } else if #[cfg(x86_64)] {
            crate::intrinsics::x86_64::instructions::interrupts::disable();
        } else {
            unimplemented!();
        }
    }
}

/// Enables all the interrupts
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
pub unsafe fn enable() {
    cfg_if! {
        if #[cfg(riscv)] {
            crate::intrinsics::riscv::interrupts::enable();
        } else if #[cfg(x86_64)] {
            crate::intrinsics::x86_64::instructions::interrupts::enable();
        } else {
            unimplemented!();
        }
    }
}

/// Execute closure `f` in an interrupt-free context.
///
/// This as also known as a "critical section".
pub fn free<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    cfg_if! {
        if #[cfg(riscv)] {
            crate::intrinsics::riscv::interrupts::free(f)
        } else if #[cfg(x86_64)] {
            crate::intrinsics::x86_64::instructions::interrupts::free(f)
        } else {
            unimplemented!()
        }
    }
}

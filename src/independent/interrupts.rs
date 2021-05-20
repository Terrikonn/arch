/// Disables all interrupts
pub unsafe fn disable() {
    match () {
        #[cfg(target_arch = "riscv")]
        () => riscv::interrupts::disable(),
        #[cfg(target_arch = "x86_64")]
        () => x86_64::instructions::interrupts::disable(),
        () => unimplemented!(),
    }
}

/// Enables all the interrupts
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
pub unsafe fn enable() {
    match () {
        #[cfg(target_arch = "riscv")]
        () => riscv::interrupts::enable(),
        #[cfg(target_arch = "x86_64")]
        () => x86_64::instructions::interrupts::enable(),
        () => unimplemented!(),
    }
}

/// Execute closure `f` in an interrupt-free context.
///
/// This as also known as a "critical section".
pub fn free<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    match () {
        #[cfg(target_arch = "riscv")]
        () => riscv::interrupts::free(f),
        #[cfg(target_arch = "x86_64")]
        () => x86_64::instructions::interrupts::free(f),
        () => unimplemented!(),
    }
}

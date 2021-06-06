pub mod interrupts;

pub fn low_power_loop() -> ! {
    cfg_if! {
        if #[cfg(riscv)] {
            loop {
                unsafe {
                    crate::intrinsics::riscv::asm::wfi();
                }
            }
        } else if #[cfg(x86_64)] {
            loop {
                crate::intrinsics::x86_64::instructions::hlt();
            }
        } else {
            unimplemented!();
        }
    }
}

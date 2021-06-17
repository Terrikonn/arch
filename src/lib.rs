#![no_std]

#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
pub use riscv;
#[cfg(target_arch = "x86_64")]
pub use x86_64;

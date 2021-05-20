//! Architecture intrinsics
#[cfg(target_arch = "riscv")]
pub use riscv;
#[cfg(target_arch = "x86_64")]
pub use x86_64;

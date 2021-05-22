//! Architecture intrinsics
#[cfg(riscv)]
pub use riscv;
#[cfg(x86_64)]
pub use x86_64;

//! Low level access to RISC-V processors
//!
//! # Minimum Supported Rust Version (MSRV)
//!
//! This crate is guaranteed to compile on stable Rust 1.42 and up. It *might*
//! compile with older versions but that may change in any new patch release.
//!
//! # Features
//!
//! This crate provides:
//!
//! - Access to core registers like `mstatus` or `mcause`.
//! - Interrupt manipulation mechanisms.
//! - Wrappers around assembly instructions like `WFI`.

#![no_std]
#![feature(llvm_asm)]

#[cfg(not(any(target_arch = "riscv32", target_arch = "riscv64")))]
compile_error!("Target must be riscv");

pub mod asm;
pub mod interrupts;
pub mod register;

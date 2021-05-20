//! # The kernel architecture library
//! The kernel architecture library foundation of hardware abstract kernel. It is the glue
//! between hardware and high-level kernel abstractions for managing resources.
//! # How to add a new architecture
//!  1. Create a new crate to arch workspace and follow module naming conversation like in other
//! crates from this workspace.
//!  2. Add abstractions to all required functionality from `independent`
//! module.
//!  3. Where it need it manually uses intrinsics for your architecture to implement
//! target-specific things.
#![cfg_attr(not(test), no_std)]
#![allow(unreachable_patterns)]

/// Abstractions for common low-level tasks
pub mod independent;
/// Reexport arch crates
pub mod intrinsics;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

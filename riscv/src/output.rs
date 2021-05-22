#![feature(prelude_import)]
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
#[prelude_import]
use core::prelude::rust_2018::*;
#[macro_use]
extern crate core;
#[macro_use]
extern crate compiler_builtins;
pub mod asm {
    //! Assembly instructions
    /// `EBREAK` instruction wrapper
    ///
    /// Generates a breakpoint exception.
    #[inline]
    pub unsafe fn ebreak() {
        match () {
            #[cfg(riscv)]
            () => llvm_asm ! ("ebreak" : : : : "volatile"),
        }
    }
    /// `WFI` instruction wrapper
    ///
    /// Provides a hint to the implementation that the current hart can be stalled until an interrupt might need servicing.
    /// The WFI instruction is just a hint, and a legal implementation is to implement WFI as a NOP.
    #[inline]
    pub unsafe fn wfi() {
        match () {
            #[cfg(riscv)]
            () => llvm_asm ! ("wfi" : : : : "volatile"),
        }
    }
    /// `SFENCE.VMA` instruction wrapper (all address spaces and page table levels)
    ///
    /// Synchronizes updates to in-memory memory-management data structures with current execution.
    /// Instruction execution causes implicit reads and writes to these data structures; however, these implicit references
    /// are ordinarily not ordered with respect to loads and stores in the instruction stream.
    /// Executing an `SFENCE.VMA` instruction guarantees that any stores in the instruction stream prior to the
    /// `SFENCE.VMA` are ordered before all implicit references subsequent to the `SFENCE.VMA`.
    #[inline]
    pub unsafe fn sfence_vma_all() {
        match () {
            #[cfg(riscv)]
            () => llvm_asm ! ("sfence.vma" : : : : "volatile"),
        }
    }
    /// `SFENCE.VMA` instruction wrapper
    ///
    /// Synchronizes updates to in-memory memory-management data structures with current execution.
    /// Instruction execution causes implicit reads and writes to these data structures; however, these
    /// implicit references are ordinarily not ordered with respect to loads and stores in the
    /// instruction stream. Executing an `SFENCE.VMA` instruction guarantees that any stores in the
    /// instruction stream prior to the `SFENCE.VMA` are ordered before all implicit references
    /// subsequent to the `SFENCE.VMA`.
    #[inline]
    #[allow(unused_variables)]
    pub unsafe fn sfence_vma(asid: usize, addr: usize) {
        match () {
            #[cfg(riscv)]
            () => llvm_asm ! ("sfence.vma $0, $1" : : "r" (asid) , "r" (addr) : : "volatile"),
        }
    }
}
pub mod interrupts {
    //! Interrupts
    use crate::register::mstatus;
    /// Disables all interrupts
    #[inline]
    pub unsafe fn disable() {
        if true {
            mstatus::clear_mie();
        } else {
            ::core::panicking::panic("not implemented");
        }
    }
    /// Enables all the interrupts
    ///
    /// # Safety
    ///
    /// - Do not call this function inside an `interrupt::free` critical section
    #[inline]
    pub unsafe fn enable() {
        if true {
            mstatus::set_mie();
        } else {
            ::core::panicking::panic("not implemented");
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
        if mstatus.mie() {
            unsafe {
                disable();
            }
        }
        let r = f();
        if mstatus.mie() {
            unsafe {
                enable();
            }
        }
        r
    }
}
pub mod register {
    //! RISC-V CSR's
    //!
    //! The following registers are not available on 64-bit implementations.
    //!
    //! - cycleh
    //! - timeh
    //! - instreth
    //! - hpmcounter[3-31]h
    //! - mcycleh
    //! - minstreth
    //! - mhpmcounter[3-31]h
    #[macro_use]
    mod macros {}
    pub mod uie {
        //! uie register
        use bit_field::BitField;
        /// uie register
        pub struct Uie {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Uie {
            #[inline]
            fn clone(&self) -> Uie {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Uie {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Uie {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Uie {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Uie");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Uie {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Enable
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// User Timer Interrupt Enable
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// User External Interrupt Enable
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x004) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Uie {
            Uie {
                bits: unsafe { _read() },
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x004) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x004) : : "volatile"),
            }
        }
        /// User Software Interrupt Enable
        #[inline]
        pub unsafe fn set_usoft() {
            _set(1 << 0);
        }
        /// User Software Interrupt Enable
        #[inline]
        pub unsafe fn clear_usoft() {
            _clear(1 << 0);
        }
        /// User Timer Interrupt Enable
        #[inline]
        pub unsafe fn set_utimer() {
            _set(1 << 4);
        }
        /// User Timer Interrupt Enable
        #[inline]
        pub unsafe fn clear_utimer() {
            _clear(1 << 4);
        }
        /// User External Interrupt Enable
        #[inline]
        pub unsafe fn set_uext() {
            _set(1 << 8);
        }
        /// User External Interrupt Enable
        #[inline]
        pub unsafe fn clear_uext() {
            _clear(1 << 8);
        }
    }
    pub mod ustatus {
        //! ustatus register
        use bit_field::BitField;
        /// ustatus register
        pub struct Ustatus {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Ustatus {
            #[inline]
            fn clone(&self) -> Ustatus {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Ustatus {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Ustatus {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Ustatus {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Ustatus");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Ustatus {
            /// User Interrupt Enable
            #[inline]
            pub fn uie(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// User Previous Interrupt Enable
            #[inline]
            pub fn upie(&self) -> bool {
                self.bits.get_bit(4)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x000) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Ustatus {
            Ustatus {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x000) : : "volatile"),
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x000) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x000) : : "volatile"),
            }
        }
        /// User Interrupt Enable
        #[inline]
        pub unsafe fn set_uie() {
            _set(1 << 0);
        }
        /// User Interrupt Enable
        #[inline]
        pub unsafe fn clear_uie() {
            _clear(1 << 0);
        }
        /// User Previous Interrupt Enable
        #[inline]
        pub unsafe fn set_upie() {
            _set(1 << 4);
        }
    }
    pub mod utvec {
        //! stvec register
        pub use crate::register::mtvec::TrapMode;
        /// stvec register
        pub struct Utvec {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Utvec {
            #[inline]
            fn clone(&self) -> Utvec {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Utvec {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Utvec {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Utvec {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Utvec");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Utvec {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Returns the trap-vector base-address
            pub fn address(&self) -> usize {
                self.bits - (self.bits & 0b11)
            }
            /// Returns the trap-vector mode
            pub fn trap_mode(&self) -> Option<TrapMode> {
                let mode = self.bits & 0b11;
                match mode {
                    0 => Some(TrapMode::Direct),
                    1 => Some(TrapMode::Vectored),
                    _ => None,
                }
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x005) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Utvec {
            Utvec {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x005) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(addr: usize, mode: TrapMode) {
            _write(addr + mode as usize);
        }
    }
    pub mod ucause {
        //! ucause register
        /// ucause register
        pub struct Ucause {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Ucause {
            #[inline]
            fn clone(&self) -> Ucause {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Ucause {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Ucause {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Ucause {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Ucause");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Ucause {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x042) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Ucause {
            Ucause {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x042) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(bits: usize) {
            _write(bits)
        }
    }
    pub mod uepc {
        //! uepc register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x041) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x041) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    }
    pub mod uip {
        //! uip register
        use bit_field::BitField;
        /// uip register
        pub struct Uip {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Uip {
            #[inline]
            fn clone(&self) -> Uip {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Uip {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Uip {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Uip {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Uip");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Uip {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Pending
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// User Timer Interrupt Pending
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// User External Interrupt Pending
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x044) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Uip {
            Uip {
                bits: unsafe { _read() },
            }
        }
    }
    pub mod uscratch {
        //! uscratch register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x040) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x040) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    }
    pub mod utval {
        //! utval register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x043) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x043) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(bits: usize) {
            _write(bits)
        }
    }
    pub mod fcsr {
        //! Floating-point control and status register
        use bit_field::BitField;
        /// Floating-point control and status register
        pub struct FCSR {
            bits: u32,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for FCSR {
            #[inline]
            fn clone(&self) -> FCSR {
                {
                    let _: ::core::clone::AssertParamIsClone<u32>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for FCSR {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for FCSR {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    FCSR {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "FCSR");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Accrued Exception Flags
        pub struct Flags(u32);
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Flags {
            #[inline]
            fn clone(&self) -> Flags {
                {
                    let _: ::core::clone::AssertParamIsClone<u32>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Flags {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Flags {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Flags(ref __self_0_0) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Flags");
                        let _ =
                            ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_0));
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Accrued Exception Flag
        pub enum Flag {
            /// Inexact
            NX = 0b00001,
            /// Underflow
            UF = 0b00010,
            /// Overflow
            OF = 0b00100,
            /// Divide by Zero
            DZ = 0b01000,
            /// Invalid Operation
            NV = 0b10000,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Flag {
            #[inline]
            fn clone(&self) -> Flag {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Flag {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Flag {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Flag::NX,) => {
                        let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "NX");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Flag::UF,) => {
                        let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "UF");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Flag::OF,) => {
                        let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "OF");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Flag::DZ,) => {
                        let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "DZ");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Flag::NV,) => {
                        let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(f, "NV");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Flags {
            /// Inexact
            #[inline]
            pub fn nx(&self) -> bool {
                self.0.get_bit(0)
            }
            /// Underflow
            #[inline]
            pub fn uf(&self) -> bool {
                self.0.get_bit(1)
            }
            /// Overflow
            #[inline]
            pub fn of(&self) -> bool {
                self.0.get_bit(2)
            }
            /// Divide by Zero
            #[inline]
            pub fn dz(&self) -> bool {
                self.0.get_bit(3)
            }
            /// Invalid Operation
            #[inline]
            pub fn nv(&self) -> bool {
                self.0.get_bit(4)
            }
        }
        /// Rounding Mode
        pub enum RoundingMode {
            RoundToNearestEven = 0b000,
            RoundTowardsZero = 0b001,
            RoundDown = 0b010,
            RoundUp = 0b011,
            RoundToNearestMaxMagnitude = 0b100,
            Invalid = 0b111,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for RoundingMode {
            #[inline]
            fn clone(&self) -> RoundingMode {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for RoundingMode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for RoundingMode {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&RoundingMode::RoundToNearestEven,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "RoundToNearestEven");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&RoundingMode::RoundTowardsZero,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "RoundTowardsZero");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&RoundingMode::RoundDown,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "RoundDown");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&RoundingMode::RoundUp,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "RoundUp");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&RoundingMode::RoundToNearestMaxMagnitude,) => {
                        let debug_trait_builder = &mut ::core::fmt::Formatter::debug_tuple(
                            f,
                            "RoundToNearestMaxMagnitude",
                        );
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&RoundingMode::Invalid,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Invalid");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for RoundingMode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for RoundingMode {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for RoundingMode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for RoundingMode {
            #[inline]
            fn eq(&self, other: &RoundingMode) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl FCSR {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> u32 {
                self.bits
            }
            /// Accrued Exception Flags
            #[inline]
            pub fn fflags(&self) -> Flags {
                Flags(self.bits.get_bits(0..5))
            }
            /// Rounding Mode
            #[inline]
            pub fn frm(&self) -> RoundingMode {
                match self.bits.get_bits(5..8) {
                    0b000 => RoundingMode::RoundToNearestEven,
                    0b001 => RoundingMode::RoundTowardsZero,
                    0b010 => RoundingMode::RoundDown,
                    0b011 => RoundingMode::RoundUp,
                    0b100 => RoundingMode::RoundToNearestMaxMagnitude,
                    _ => RoundingMode::Invalid,
                }
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x003) : : "volatile");
                    r
                }
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x003) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x003) : : "volatile"),
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> FCSR {
            FCSR {
                bits: unsafe { _read() as u32 },
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn set_rounding_mode(frm: RoundingMode) {
            let old = read();
            let bits = ((frm as u32) << 5) | old.fflags().0;
            _write(bits as usize);
        }
        /// Resets `fflags` field bits
        #[inline]
        pub unsafe fn clear_flags() {
            let mask = 0b11111;
            _clear(mask);
        }
        /// Resets `fflags` field bit
        #[inline]
        pub unsafe fn clear_flag(flag: Flag) {
            _clear(flag as usize);
        }
    }
    pub mod cycle {
        //! cycle register
        //!
        //! Shadow of mcycle register
        //! must have `scounteren::cy` or `mcounteren::cy` bit enabled depending on whether
        //! S-mode is implemented or not
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC00) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Reads the CSR as a 64-bit value
        #[inline]
        pub fn read64() -> u64 {
            match () {
                #[cfg(not(riscv32))]
                () => read() as u64,
            }
        }
    }
    pub mod cycleh {
        //! cycleh register
        //!
        //! Shadow of mcycleh register (rv32)
        //! must have `scounteren::cy` or `mcounteren::cy` bit enabled depending on whether
        //! S-mode is implemented or not
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(not(riscv32))]
                () => ::core::panicking::panic("not implemented"),
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    mod hpmcounterx {
        /// Performance-monitoring counter
        pub mod hpmcounter3 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC03) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter4 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC04) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter5 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC05) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter6 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC06) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter7 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC07) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter8 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC08) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter9 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC09) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter10 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC0A) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter11 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC0B) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter12 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC0C) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter13 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC0D) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter14 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC0E) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter15 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC0F) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter16 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC10) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter17 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC11) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter18 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC12) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter19 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC13) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter20 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC14) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter21 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC15) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter22 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC16) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter23 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC17) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter24 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC18) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter25 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC19) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter26 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC1A) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter27 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC1B) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter28 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC1C) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter29 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC1D) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter30 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC1E) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Performance-monitoring counter
        pub mod hpmcounter31 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC1F) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter3h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter4h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter5h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter6h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter7h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter8h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter9h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter10h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter11h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter12h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter13h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter14h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter15h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter16h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter17h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter18h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter19h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter20h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter21h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter22h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter23h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter24h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter25h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter26h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter27h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter28h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter29h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter30h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
        /// Upper 32 bits of performance-monitoring counter (RV32I only)
        pub mod hpmcounter31h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
        }
    }
    pub use self::hpmcounterx::*;
    pub mod instret {
        //! instret register
        //!
        //! Shadow of minstret register
        //! must have `scounteren::ir` or `mcounteren::ir` bit enabled depending on whether
        //! S-mode is implemented or not
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC02) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Reads the CSR as a 64-bit value
        #[inline]
        pub fn read64() -> u64 {
            match () {
                #[cfg(not(riscv32))]
                () => read() as u64,
            }
        }
    }
    pub mod instreth {
        //! instreth register
        //!
        //! Shadow of minstreth register (rv32)
        //! must have `scounteren::ir` or `mcounteren::ir` bit enabled depending on whether
        //! S-mode is implemented or not
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC82) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    pub mod time {
        //! time register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xC01) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Reads the CSR as a 64-bit value
        #[inline]
        pub fn read64() -> u64 {
            match () {
                #[cfg(not(riscv32))]
                () => read() as u64,
            }
        }
    }
    pub mod timeh {
        //! timeh register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(not(riscv32))]
                () => ::core::panicking::panic("not implemented"),
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    pub mod scounteren {
        //! scounteren register
        use bit_field::BitField;
        /// scounteren register
        pub struct Scounteren {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Scounteren {
            #[inline]
            fn clone(&self) -> Scounteren {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Scounteren {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Scounteren {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Scounteren {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Scounteren");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Scounteren {
            /// User "cycle\[h\]" Enable
            #[inline]
            pub fn cy(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// User "time\[h\]" Enable
            #[inline]
            pub fn tm(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// User "instret\[h]\" Enable
            #[inline]
            pub fn ir(&self) -> bool {
                self.bits.get_bit(2)
            }
            /// User "hpm\[x\]" Enable (bits 3-31)
            #[inline]
            pub fn hpm(&self, index: usize) -> bool {
                if !(3 <= index && index < 32) {
                    ::core::panicking::panic("assertion failed: 3 <= index && index < 32")
                };
                self.bits.get_bit(index)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x106) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Scounteren {
            Scounteren {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x106) : : "volatile"),
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x106) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x106) : : "volatile"),
            }
        }
        /// User cycle Enable
        #[inline]
        pub unsafe fn set_cy() {
            _set(1 << 0);
        }
        /// User cycle Enable
        #[inline]
        pub unsafe fn clear_cy() {
            _clear(1 << 0);
        }
        /// User time Enable
        #[inline]
        pub unsafe fn set_tm() {
            _set(1 << 1);
        }
        /// User time Enable
        #[inline]
        pub unsafe fn clear_tm() {
            _clear(1 << 1);
        }
        /// User instret Enable
        #[inline]
        pub unsafe fn set_ir() {
            _set(1 << 2);
        }
        /// User instret Enable
        #[inline]
        pub unsafe fn clear_ir() {
            _clear(1 << 2);
        }
        #[inline]
        pub unsafe fn set_hpm(index: usize) {
            if !(3 <= index && index < 32) {
                ::core::panicking::panic("assertion failed: 3 <= index && index < 32")
            };
            _set(1 << index);
        }
        #[inline]
        pub unsafe fn clear_hpm(index: usize) {
            if !(3 <= index && index < 32) {
                ::core::panicking::panic("assertion failed: 3 <= index && index < 32")
            };
            _clear(1 << index);
        }
    }
    pub mod sie {
        //! sie register
        use bit_field::BitField;
        /// sie register
        pub struct Sie {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Sie {
            #[inline]
            fn clone(&self) -> Sie {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Sie {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Sie {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Sie {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Sie");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Sie {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Enable
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Software Interrupt Enable
            #[inline]
            pub fn ssoft(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// User Timer Interrupt Enable
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Timer Interrupt Enable
            #[inline]
            pub fn stimer(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// User External Interrupt Enable
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
            /// Supervisor External Interrupt Enable
            #[inline]
            pub fn sext(&self) -> bool {
                self.bits.get_bit(9)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x104) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Sie {
            Sie {
                bits: unsafe { _read() },
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x104) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x104) : : "volatile"),
            }
        }
        /// User Software Interrupt Enable
        #[inline]
        pub unsafe fn set_usoft() {
            _set(1 << 0);
        }
        /// User Software Interrupt Enable
        #[inline]
        pub unsafe fn clear_usoft() {
            _clear(1 << 0);
        }
        /// Supervisor Software Interrupt Enable
        #[inline]
        pub unsafe fn set_ssoft() {
            _set(1 << 1);
        }
        /// Supervisor Software Interrupt Enable
        #[inline]
        pub unsafe fn clear_ssoft() {
            _clear(1 << 1);
        }
        /// User Timer Interrupt Enable
        #[inline]
        pub unsafe fn set_utimer() {
            _set(1 << 4);
        }
        /// User Timer Interrupt Enable
        #[inline]
        pub unsafe fn clear_utimer() {
            _clear(1 << 4);
        }
        /// Supervisor Timer Interrupt Enable
        #[inline]
        pub unsafe fn set_stimer() {
            _set(1 << 5);
        }
        /// Supervisor Timer Interrupt Enable
        #[inline]
        pub unsafe fn clear_stimer() {
            _clear(1 << 5);
        }
        /// User External Interrupt Enable
        #[inline]
        pub unsafe fn set_uext() {
            _set(1 << 8);
        }
        /// User External Interrupt Enable
        #[inline]
        pub unsafe fn clear_uext() {
            _clear(1 << 8);
        }
        /// Supervisor External Interrupt Enable
        #[inline]
        pub unsafe fn set_sext() {
            _set(1 << 9);
        }
        /// Supervisor External Interrupt Enable
        #[inline]
        pub unsafe fn clear_sext() {
            _clear(1 << 9);
        }
    }
    pub mod sstatus {
        //! sstatus register
        pub use super::mstatus::FS;
        use bit_field::BitField;
        use core::mem::size_of;
        /// Supervisor Status Register
        pub struct Sstatus {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Sstatus {
            #[inline]
            fn clone(&self) -> Sstatus {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Sstatus {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Sstatus {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Sstatus {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Sstatus");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Supervisor Previous Privilege Mode
        pub enum SPP {
            Supervisor = 1,
            User = 0,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for SPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for SPP {
            #[inline]
            fn clone(&self) -> SPP {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for SPP {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&SPP::Supervisor,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Supervisor");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&SPP::User,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "User");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for SPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for SPP {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for SPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for SPP {
            #[inline]
            fn eq(&self, other: &SPP) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl Sstatus {
            /// User Interrupt Enable
            #[inline]
            pub fn uie(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Interrupt Enable
            #[inline]
            pub fn sie(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// User Previous Interrupt Enable
            #[inline]
            pub fn upie(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Previous Interrupt Enable
            #[inline]
            pub fn spie(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// Supervisor Previous Privilege Mode
            #[inline]
            pub fn spp(&self) -> SPP {
                match self.bits.get_bit(8) {
                    true => SPP::Supervisor,
                    false => SPP::User,
                }
            }
            /// The status of the floating-point unit
            #[inline]
            pub fn fs(&self) -> FS {
                match self.bits.get_bits(13..15) {
                    0 => FS::Off,
                    1 => FS::Initial,
                    2 => FS::Clean,
                    3 => FS::Dirty,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// The status of additional user-mode extensions
            /// and associated state
            #[inline]
            pub fn xs(&self) -> FS {
                match self.bits.get_bits(15..17) {
                    0 => FS::Off,
                    1 => FS::Initial,
                    2 => FS::Clean,
                    3 => FS::Dirty,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// Permit Supervisor User Memory access
            #[inline]
            pub fn sum(&self) -> bool {
                self.bits.get_bit(18)
            }
            /// Make eXecutable Readable
            #[inline]
            pub fn mxr(&self) -> bool {
                self.bits.get_bit(19)
            }
            /// Whether either the FS field or XS field
            /// signals the presence of some dirty state
            #[inline]
            pub fn sd(&self) -> bool {
                self.bits.get_bit(size_of::<usize>() * 8 - 1)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x100) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Sstatus {
            Sstatus {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x100) : : "volatile"),
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x100) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x100) : : "volatile"),
            }
        }
        /// User Interrupt Enable
        #[inline]
        pub unsafe fn set_uie() {
            _set(1 << 0);
        }
        /// User Interrupt Enable
        #[inline]
        pub unsafe fn clear_uie() {
            _clear(1 << 0);
        }
        /// Supervisor Interrupt Enable
        #[inline]
        pub unsafe fn set_sie() {
            _set(1 << 1);
        }
        /// Supervisor Interrupt Enable
        #[inline]
        pub unsafe fn clear_sie() {
            _clear(1 << 1);
        }
        /// User Previous Interrupt Enable
        #[inline]
        pub unsafe fn set_upie() {
            _set(1 << 4);
        }
        /// Supervisor Previous Interrupt Enable
        #[inline]
        pub unsafe fn set_spie() {
            _set(1 << 5);
        }
        /// Permit Supervisor User Memory access
        #[inline]
        pub unsafe fn set_sum() {
            _set(1 << 18);
        }
        /// Permit Supervisor User Memory access
        #[inline]
        pub unsafe fn clear_sum() {
            _clear(1 << 18);
        }
        /// Make eXecutable Readable
        #[inline]
        pub unsafe fn set_mxr() {
            _set(1 << 19);
        }
        /// Make eXecutable Readable
        #[inline]
        pub unsafe fn clear_mxr() {
            _clear(1 << 19);
        }
        /// Supervisor Previous Privilege Mode
        #[inline]
        pub unsafe fn set_spp(spp: SPP) {
            match spp {
                SPP::Supervisor => _set(1 << 8),
                SPP::User => _clear(1 << 8),
            }
        }
        /// The status of the floating-point unit
        #[inline]
        pub unsafe fn set_fs(fs: FS) {
            let mut value = _read();
            value.set_bits(13..15, fs as usize);
            _write(value);
        }
    }
    pub mod stvec {
        //! stvec register
        pub use crate::register::mtvec::TrapMode;
        /// stvec register
        pub struct Stvec {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Stvec {
            #[inline]
            fn clone(&self) -> Stvec {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Stvec {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Stvec {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Stvec {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Stvec");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Stvec {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Returns the trap-vector base-address
            pub fn address(&self) -> usize {
                self.bits - (self.bits & 0b11)
            }
            /// Returns the trap-vector mode
            pub fn trap_mode(&self) -> Option<TrapMode> {
                let mode = self.bits & 0b11;
                match mode {
                    0 => Some(TrapMode::Direct),
                    1 => Some(TrapMode::Vectored),
                    _ => None,
                }
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x105) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Stvec {
            Stvec {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x105) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(addr: usize, mode: TrapMode) {
            _write(addr + mode as usize);
        }
    }
    pub mod scause {
        //! scause register
        use bit_field::BitField;
        use core::mem::size_of;
        /// scause register
        pub struct Scause {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Scause {
            #[inline]
            fn clone(&self) -> Scause {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Scause {}
        /// Trap Cause
        pub enum Trap {
            Interrupt(Interrupt),
            Exception(Exception),
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Trap {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Trap {
            #[inline]
            fn clone(&self) -> Trap {
                {
                    let _: ::core::clone::AssertParamIsClone<Interrupt>;
                    let _: ::core::clone::AssertParamIsClone<Exception>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Trap {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Trap::Interrupt(ref __self_0),) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Interrupt");
                        let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0));
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Trap::Exception(ref __self_0),) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Exception");
                        let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0));
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Trap {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Trap {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {
                    let _: ::core::cmp::AssertParamIsEq<Interrupt>;
                    let _: ::core::cmp::AssertParamIsEq<Exception>;
                }
            }
        }
        impl ::core::marker::StructuralPartialEq for Trap {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Trap {
            #[inline]
            fn eq(&self, other: &Trap) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            (&Trap::Interrupt(ref __self_0), &Trap::Interrupt(ref __arg_1_0)) => {
                                (*__self_0) == (*__arg_1_0)
                            }
                            (&Trap::Exception(ref __self_0), &Trap::Exception(ref __arg_1_0)) => {
                                (*__self_0) == (*__arg_1_0)
                            }
                            _ => unsafe { ::core::intrinsics::unreachable() },
                        }
                    } else {
                        false
                    }
                }
            }
            #[inline]
            fn ne(&self, other: &Trap) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            (&Trap::Interrupt(ref __self_0), &Trap::Interrupt(ref __arg_1_0)) => {
                                (*__self_0) != (*__arg_1_0)
                            }
                            (&Trap::Exception(ref __self_0), &Trap::Exception(ref __arg_1_0)) => {
                                (*__self_0) != (*__arg_1_0)
                            }
                            _ => unsafe { ::core::intrinsics::unreachable() },
                        }
                    } else {
                        true
                    }
                }
            }
        }
        /// Interrupt
        pub enum Interrupt {
            UserSoft,
            SupervisorSoft,
            UserTimer,
            SupervisorTimer,
            UserExternal,
            SupervisorExternal,
            Unknown,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Interrupt {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Interrupt {
            #[inline]
            fn clone(&self) -> Interrupt {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Interrupt {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Interrupt::UserSoft,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserSoft");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::SupervisorSoft,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorSoft");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::UserTimer,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserTimer");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::SupervisorTimer,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorTimer");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::UserExternal,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserExternal");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::SupervisorExternal,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorExternal");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::Unknown,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Unknown");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Interrupt {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Interrupt {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for Interrupt {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Interrupt {
            #[inline]
            fn eq(&self, other: &Interrupt) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        /// Exception
        pub enum Exception {
            InstructionMisaligned,
            InstructionFault,
            IllegalInstruction,
            Breakpoint,
            LoadFault,
            StoreMisaligned,
            StoreFault,
            UserEnvCall,
            InstructionPageFault,
            LoadPageFault,
            StorePageFault,
            Unknown,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Exception {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Exception {
            #[inline]
            fn clone(&self) -> Exception {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Exception {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Exception::InstructionMisaligned,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "InstructionMisaligned");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::InstructionFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "InstructionFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::IllegalInstruction,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "IllegalInstruction");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::Breakpoint,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Breakpoint");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::LoadFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "LoadFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::StoreMisaligned,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "StoreMisaligned");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::StoreFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "StoreFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::UserEnvCall,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserEnvCall");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::InstructionPageFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "InstructionPageFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::LoadPageFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "LoadPageFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::StorePageFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "StorePageFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::Unknown,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Unknown");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Exception {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Exception {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for Exception {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Exception {
            #[inline]
            fn eq(&self, other: &Exception) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl Interrupt {
            pub fn from(nr: usize) -> Self {
                match nr {
                    0 => Interrupt::UserSoft,
                    1 => Interrupt::SupervisorSoft,
                    4 => Interrupt::UserTimer,
                    5 => Interrupt::SupervisorTimer,
                    8 => Interrupt::UserExternal,
                    9 => Interrupt::SupervisorExternal,
                    _ => Interrupt::Unknown,
                }
            }
        }
        impl Exception {
            pub fn from(nr: usize) -> Self {
                match nr {
                    0 => Exception::InstructionMisaligned,
                    1 => Exception::InstructionFault,
                    2 => Exception::IllegalInstruction,
                    3 => Exception::Breakpoint,
                    5 => Exception::LoadFault,
                    6 => Exception::StoreMisaligned,
                    7 => Exception::StoreFault,
                    8 => Exception::UserEnvCall,
                    12 => Exception::InstructionPageFault,
                    13 => Exception::LoadPageFault,
                    15 => Exception::StorePageFault,
                    _ => Exception::Unknown,
                }
            }
        }
        impl Scause {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Returns the code field
            pub fn code(&self) -> usize {
                let bit = 1 << (size_of::<usize>() * 8 - 1);
                self.bits & !bit
            }
            /// Trap Cause
            #[inline]
            pub fn cause(&self) -> Trap {
                if self.is_interrupt() {
                    Trap::Interrupt(Interrupt::from(self.code()))
                } else {
                    Trap::Exception(Exception::from(self.code()))
                }
            }
            /// Is trap cause an interrupt.
            #[inline]
            pub fn is_interrupt(&self) -> bool {
                self.bits.get_bit(size_of::<usize>() * 8 - 1)
            }
            /// Is trap cause an exception.
            #[inline]
            pub fn is_exception(&self) -> bool {
                !self.is_interrupt()
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x142) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Scause {
            Scause {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x142) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(bits: usize) {
            _write(bits)
        }
        /// Set supervisor cause register to corresponding cause.
        #[inline]
        pub unsafe fn set(cause: Trap) {
            let bits = match cause {
                Trap::Interrupt(i) => {
                    (match i {
                        Interrupt::UserSoft => 0,
                        Interrupt::SupervisorSoft => 1,
                        Interrupt::UserTimer => 4,
                        Interrupt::SupervisorTimer => 5,
                        Interrupt::UserExternal => 8,
                        Interrupt::SupervisorExternal => 9,
                        Interrupt::Unknown => ::core::panicking::panic("unknown interrupt"),
                    } | (1 << (size_of::<usize>() * 8 - 1)))
                }
                Trap::Exception(e) => match e {
                    Exception::InstructionMisaligned => 0,
                    Exception::InstructionFault => 1,
                    Exception::IllegalInstruction => 2,
                    Exception::Breakpoint => 3,
                    Exception::LoadFault => 5,
                    Exception::StoreMisaligned => 6,
                    Exception::StoreFault => 7,
                    Exception::UserEnvCall => 8,
                    Exception::InstructionPageFault => 12,
                    Exception::LoadPageFault => 13,
                    Exception::StorePageFault => 15,
                    Exception::Unknown => ::core::panicking::panic("unknown exception"),
                },
            };
            _write(bits);
        }
    }
    pub mod sepc {
        //! sepc register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x141) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x141) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    }
    pub mod sip {
        //! sip register
        use bit_field::BitField;
        /// sip register
        pub struct Sip {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Sip {
            #[inline]
            fn clone(&self) -> Sip {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Sip {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Sip {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Sip {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Sip");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Sip {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Pending
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Software Interrupt Pending
            #[inline]
            pub fn ssoft(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// User Timer Interrupt Pending
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Timer Interrupt Pending
            #[inline]
            pub fn stimer(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// User External Interrupt Pending
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
            /// Supervisor External Interrupt Pending
            #[inline]
            pub fn sext(&self) -> bool {
                self.bits.get_bit(9)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x144) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Sip {
            Sip {
                bits: unsafe { _read() },
            }
        }
    }
    pub mod sscratch {
        //! sscratch register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x140) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x140) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    }
    pub mod stval {
        //! stval register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x143) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x143) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(bits: usize) {
            _write(bits)
        }
    }
    pub mod satp {
        //! satp register
        use bit_field::BitField;
        /// satp register
        pub struct Satp {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Satp {
            #[inline]
            fn clone(&self) -> Satp {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Satp {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Satp {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Satp {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Satp");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Satp {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Current address-translation scheme
            #[inline]
            #[cfg(target_pointer_width = "64")]
            pub fn mode(&self) -> Mode {
                match self.bits.get_bits(60..64) {
                    0 => Mode::Bare,
                    8 => Mode::Sv39,
                    9 => Mode::Sv48,
                    10 => Mode::Sv57,
                    11 => Mode::Sv64,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// Address space identifier
            #[inline]
            #[cfg(target_pointer_width = "64")]
            pub fn asid(&self) -> usize {
                self.bits.get_bits(44..60)
            }
            /// Physical page number
            #[inline]
            #[cfg(target_pointer_width = "64")]
            pub fn ppn(&self) -> usize {
                self.bits.get_bits(0..44)
            }
        }
        /// 64-bit satp mode
        #[cfg(target_pointer_width = "64")]
        pub enum Mode {
            /// No translation or protection
            Bare = 0,
            /// Page-based 39-bit virtual addressing
            Sv39 = 8,
            /// Page-based 48-bit virtual addressing
            Sv48 = 9,
            /// Page-based 57-bit virtual addressing
            Sv57 = 10,
            /// Page-based 64-bit virtual addressing
            Sv64 = 11,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mode {
            #[inline]
            fn clone(&self) -> Mode {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mode {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Mode::Bare,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Bare");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Mode::Sv39,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Sv39");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Mode::Sv48,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Sv48");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Mode::Sv57,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Sv57");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Mode::Sv64,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Sv64");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Mode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Mode {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for Mode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Mode {
            #[inline]
            fn eq(&self, other: &Mode) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x180) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Satp {
            Satp {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x180) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
        /// Sets the register to corresponding page table mode, physical page number and address space id.
        #[inline]
        #[cfg(target_pointer_width = "64")]
        pub unsafe fn set(mode: Mode, asid: usize, ppn: usize) {
            let mut bits = 0usize;
            bits.set_bits(60..64, mode as usize);
            bits.set_bits(44..60, asid);
            bits.set_bits(0..44, ppn);
            _write(bits);
        }
    }
    pub mod marchid {
        //! marchid register
        use core::num::NonZeroUsize;
        /// marchid register
        pub struct Marchid {
            bits: NonZeroUsize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Marchid {
            #[inline]
            fn clone(&self) -> Marchid {
                {
                    let _: ::core::clone::AssertParamIsClone<NonZeroUsize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Marchid {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Marchid {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Marchid {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Marchid");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Marchid {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits.get()
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xF11) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Option<Marchid> {
            let r = unsafe { _read() };
            NonZeroUsize::new(r).map(|bits| Marchid { bits })
        }
    }
    pub mod mhartid {
        //! mhartid register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xf14) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    pub mod mimpid {
        //! mimpid register
        use core::num::NonZeroUsize;
        /// mimpid register
        pub struct Mimpid {
            bits: NonZeroUsize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mimpid {
            #[inline]
            fn clone(&self) -> Mimpid {
                {
                    let _: ::core::clone::AssertParamIsClone<NonZeroUsize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mimpid {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mimpid {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mimpid {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mimpid");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Mimpid {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits.get()
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xF11) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Option<Mimpid> {
            let r = unsafe { _read() };
            NonZeroUsize::new(r).map(|bits| Mimpid { bits })
        }
    }
    pub mod mvendorid {
        //! mvendorid register
        use core::num::NonZeroUsize;
        /// mvendorid register
        pub struct Mvendorid {
            bits: NonZeroUsize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mvendorid {
            #[inline]
            fn clone(&self) -> Mvendorid {
                {
                    let _: ::core::clone::AssertParamIsClone<NonZeroUsize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mvendorid {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mvendorid {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mvendorid {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mvendorid");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Mvendorid {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits.get()
            }
            /// Returns the JEDEC manufacturer ID
            pub fn jedec_manufacturer(&self) -> usize {
                self.bits() >> 7
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xF11) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Option<Mvendorid> {
            let r = unsafe { _read() };
            NonZeroUsize::new(r).map(|bits| Mvendorid { bits })
        }
    }
    pub mod mcounteren {
        //! mcounteren register
        use bit_field::BitField;
        /// mcounteren register
        pub struct Mcounteren {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mcounteren {
            #[inline]
            fn clone(&self) -> Mcounteren {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mcounteren {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mcounteren {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mcounteren {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mcounteren");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Mcounteren {
            /// Supervisor "cycle\[h\]" Enable
            #[inline]
            pub fn cy(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor "time\[h\]" Enable
            #[inline]
            pub fn tm(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// Supervisor "instret\[h\]" Enable
            #[inline]
            pub fn ir(&self) -> bool {
                self.bits.get_bit(2)
            }
            /// Supervisor "hpm\[x\]" Enable (bits 3-31)
            #[inline]
            pub fn hpm(&self, index: usize) -> bool {
                if !(3 <= index && index < 32) {
                    ::core::panicking::panic("assertion failed: 3 <= index && index < 32")
                };
                self.bits.get_bit(index)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x306) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mcounteren {
            Mcounteren {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x306) : : "volatile"),
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x306) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x306) : : "volatile"),
            }
        }
        /// Supervisor cycle Enable
        #[inline]
        pub unsafe fn set_cy() {
            _set(1 << 0);
        }
        /// Supervisor cycle Enable
        #[inline]
        pub unsafe fn clear_cy() {
            _clear(1 << 0);
        }
        /// Supervisor time Enable
        #[inline]
        pub unsafe fn set_tm() {
            _set(1 << 1);
        }
        /// Supervisor time Enable
        #[inline]
        pub unsafe fn clear_tm() {
            _clear(1 << 1);
        }
        /// Supervisor instret Enable
        #[inline]
        pub unsafe fn set_ir() {
            _set(1 << 2);
        }
        /// Supervisor instret Enable
        #[inline]
        pub unsafe fn clear_ir() {
            _clear(1 << 2);
        }
        #[inline]
        pub unsafe fn set_hpm(index: usize) {
            if !(3 <= index && index < 32) {
                ::core::panicking::panic("assertion failed: 3 <= index && index < 32")
            };
            _set(1 << index);
        }
        #[inline]
        pub unsafe fn clear_hpm(index: usize) {
            if !(3 <= index && index < 32) {
                ::core::panicking::panic("assertion failed: 3 <= index && index < 32")
            };
            _clear(1 << index);
        }
    }
    pub mod medeleg {
        //! medeleg register
        use bit_field::BitField;
        /// medeleg register
        pub struct Medeleg {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Medeleg {
            #[inline]
            fn clone(&self) -> Medeleg {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Medeleg {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Medeleg {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Medeleg {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Medeleg");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Medeleg {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Instruction Address Misaligned Delegate
            #[inline]
            pub fn instruction_misaligned(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Instruction Access Fault Delegate
            #[inline]
            pub fn instruction_fault(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// Illegal Instruction Delegate
            #[inline]
            pub fn illegal_instruction(&self) -> bool {
                self.bits.get_bit(2)
            }
            /// Breakpoint Delegate
            #[inline]
            pub fn breakpoint(&self) -> bool {
                self.bits.get_bit(3)
            }
            /// Load Address Misaligned Delegate
            #[inline]
            pub fn load_misaligned(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Load Access Fault Delegate
            #[inline]
            pub fn load_fault(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// Store/AMO Address Misaligned Delegate
            #[inline]
            pub fn store_misaligned(&self) -> bool {
                self.bits.get_bit(6)
            }
            /// Store/AMO Access Fault Delegate
            #[inline]
            pub fn store_fault(&self) -> bool {
                self.bits.get_bit(7)
            }
            /// Environment Call from U-mode Delegate
            #[inline]
            pub fn user_env_call(&self) -> bool {
                self.bits.get_bit(8)
            }
            /// Environment Call from S-mode Delegate
            #[inline]
            pub fn supervisor_env_call(&self) -> bool {
                self.bits.get_bit(9)
            }
            /// Environment Call from M-mode Delegate
            #[inline]
            pub fn machine_env_call(&self) -> bool {
                self.bits.get_bit(11)
            }
            /// Instruction Page Fault Delegate
            #[inline]
            pub fn instruction_page_fault(&self) -> bool {
                self.bits.get_bit(12)
            }
            /// Load Page Fault Delegate
            #[inline]
            pub fn load_page_fault(&self) -> bool {
                self.bits.get_bit(13)
            }
            /// Store/AMO Page Fault Delegate
            #[inline]
            pub fn store_page_fault(&self) -> bool {
                self.bits.get_bit(15)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x302) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Medeleg {
            Medeleg {
                bits: unsafe { _read() },
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x302) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x302) : : "volatile"),
            }
        }
        /// Instruction Address Misaligned Delegate
        #[inline]
        pub unsafe fn set_instruction_misaligned() {
            _set(1 << 0);
        }
        /// Instruction Address Misaligned Delegate
        #[inline]
        pub unsafe fn clear_instruction_misaligned() {
            _clear(1 << 0);
        }
        /// Instruction Access Fault Delegate
        #[inline]
        pub unsafe fn set_instruction_fault() {
            _set(1 << 1);
        }
        /// Instruction Access Fault Delegate
        #[inline]
        pub unsafe fn clear_instruction_fault() {
            _clear(1 << 1);
        }
        /// Illegal Instruction Delegate
        #[inline]
        pub unsafe fn set_illegal_instruction() {
            _set(1 << 2);
        }
        /// Illegal Instruction Delegate
        #[inline]
        pub unsafe fn clear_illegal_instruction() {
            _clear(1 << 2);
        }
        /// Breakpoint Delegate
        #[inline]
        pub unsafe fn set_breakpoint() {
            _set(1 << 3);
        }
        /// Breakpoint Delegate
        #[inline]
        pub unsafe fn clear_breakpoint() {
            _clear(1 << 3);
        }
        /// Load Address Misaligned Delegate
        #[inline]
        pub unsafe fn set_load_misaligned() {
            _set(1 << 4);
        }
        /// Load Address Misaligned Delegate
        #[inline]
        pub unsafe fn clear_load_misaligned() {
            _clear(1 << 4);
        }
        /// Load Access Fault Delegate
        #[inline]
        pub unsafe fn set_load_fault() {
            _set(1 << 5);
        }
        /// Load Access Fault Delegate
        #[inline]
        pub unsafe fn clear_load_fault() {
            _clear(1 << 5);
        }
        /// Store/AMO Address Misaligned Delegate
        #[inline]
        pub unsafe fn set_store_misaligned() {
            _set(1 << 6);
        }
        /// Store/AMO Address Misaligned Delegate
        #[inline]
        pub unsafe fn clear_store_misaligned() {
            _clear(1 << 6);
        }
        /// Store/AMO Access fault
        #[inline]
        pub unsafe fn set_store_fault() {
            _set(1 << 7);
        }
        /// Store/AMO Access fault
        #[inline]
        pub unsafe fn clear_store_fault() {
            _clear(1 << 7);
        }
        /// Environment Call from U-mode Delegate
        #[inline]
        pub unsafe fn set_user_env_call() {
            _set(1 << 8);
        }
        /// Environment Call from U-mode Delegate
        #[inline]
        pub unsafe fn clear_user_env_call() {
            _clear(1 << 8);
        }
        /// Environment Call from S-mode Delegate
        #[inline]
        pub unsafe fn set_supervisor_env_call() {
            _set(1 << 9);
        }
        /// Environment Call from S-mode Delegate
        #[inline]
        pub unsafe fn clear_supervisor_env_call() {
            _clear(1 << 9);
        }
        /// Environment Call from M-mode Delegate
        #[inline]
        pub unsafe fn set_machine_env_call() {
            _set(1 << 11);
        }
        /// Environment Call from M-mode Delegate
        #[inline]
        pub unsafe fn clear_machine_env_call() {
            _clear(1 << 11);
        }
        /// Instruction Page Fault Delegate
        #[inline]
        pub unsafe fn set_instruction_page_fault() {
            _set(1 << 12);
        }
        /// Instruction Page Fault Delegate
        #[inline]
        pub unsafe fn clear_instruction_page_fault() {
            _clear(1 << 12);
        }
        /// Load Page Fault Delegate
        #[inline]
        pub unsafe fn set_load_page_fault() {
            _set(1 << 13);
        }
        /// Load Page Fault Delegate
        #[inline]
        pub unsafe fn clear_load_page_fault() {
            _clear(1 << 13);
        }
        /// Store/AMO Page Fault Delegate
        #[inline]
        pub unsafe fn set_store_page_fault() {
            _set(1 << 15);
        }
        /// Store/AMO Page Fault Delegate
        #[inline]
        pub unsafe fn clear_store_page_fault() {
            _clear(1 << 15);
        }
    }
    pub mod mideleg {
        //! mideleg register
        use bit_field::BitField;
        /// mideleg register
        pub struct Mideleg {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mideleg {
            #[inline]
            fn clone(&self) -> Mideleg {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mideleg {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mideleg {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mideleg {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mideleg");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Mideleg {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Delegate
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Software Interrupt Delegate
            #[inline]
            pub fn ssoft(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// User Timer Interrupt Delegate
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Timer Interrupt Delegate
            #[inline]
            pub fn stimer(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// User External Interrupt Delegate
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
            /// Supervisor External Interrupt Delegate
            #[inline]
            pub fn sext(&self) -> bool {
                self.bits.get_bit(9)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x303) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mideleg {
            Mideleg {
                bits: unsafe { _read() },
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x303) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x303) : : "volatile"),
            }
        }
        /// User Software Interrupt Delegate
        #[inline]
        pub unsafe fn set_usoft() {
            _set(1 << 0);
        }
        /// User Software Interrupt Delegate
        #[inline]
        pub unsafe fn clear_usoft() {
            _clear(1 << 0);
        }
        /// Supervisor Software Interrupt Delegate
        #[inline]
        pub unsafe fn set_ssoft() {
            _set(1 << 1);
        }
        /// Supervisor Software Interrupt Delegate
        #[inline]
        pub unsafe fn clear_ssoft() {
            _clear(1 << 1);
        }
        /// User Timer Interrupt Delegate
        #[inline]
        pub unsafe fn set_utimer() {
            _set(1 << 4);
        }
        /// User Timer Interrupt Delegate
        #[inline]
        pub unsafe fn clear_utimer() {
            _clear(1 << 4);
        }
        /// Supervisor Timer Interrupt Delegate
        #[inline]
        pub unsafe fn set_stimer() {
            _set(1 << 5);
        }
        /// Supervisor Timer Interrupt Delegate
        #[inline]
        pub unsafe fn clear_stimer() {
            _clear(1 << 5);
        }
        /// User External Interrupt Delegate
        #[inline]
        pub unsafe fn set_uext() {
            _set(1 << 8);
        }
        /// User External Interrupt Delegate
        #[inline]
        pub unsafe fn clear_uext() {
            _clear(1 << 8);
        }
        /// Supervisor External Interrupt Delegate
        #[inline]
        pub unsafe fn set_sext() {
            _set(1 << 9);
        }
        /// Supervisor External Interrupt Delegate
        #[inline]
        pub unsafe fn clear_sext() {
            _clear(1 << 9);
        }
    }
    pub mod mie {
        //! mie register
        use bit_field::BitField;
        /// mie register
        pub struct Mie {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mie {
            #[inline]
            fn clone(&self) -> Mie {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mie {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mie {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mie {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mie");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Mie {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Enable
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Software Interrupt Enable
            #[inline]
            pub fn ssoft(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// Machine Software Interrupt Enable
            #[inline]
            pub fn msoft(&self) -> bool {
                self.bits.get_bit(3)
            }
            /// User Timer Interrupt Enable
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Timer Interrupt Enable
            #[inline]
            pub fn stimer(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// Machine Timer Interrupt Enable
            #[inline]
            pub fn mtimer(&self) -> bool {
                self.bits.get_bit(7)
            }
            /// User External Interrupt Enable
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
            /// Supervisor External Interrupt Enable
            #[inline]
            pub fn sext(&self) -> bool {
                self.bits.get_bit(9)
            }
            /// Machine External Interrupt Enable
            #[inline]
            pub fn mext(&self) -> bool {
                self.bits.get_bit(11)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x304) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mie {
            Mie {
                bits: unsafe { _read() },
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x304) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x304) : : "volatile"),
            }
        }
        /// User Software Interrupt Enable
        #[inline]
        pub unsafe fn set_usoft() {
            _set(1 << 0);
        }
        /// User Software Interrupt Enable
        #[inline]
        pub unsafe fn clear_usoft() {
            _clear(1 << 0);
        }
        /// Supervisor Software Interrupt Enable
        #[inline]
        pub unsafe fn set_ssoft() {
            _set(1 << 1);
        }
        /// Supervisor Software Interrupt Enable
        #[inline]
        pub unsafe fn clear_ssoft() {
            _clear(1 << 1);
        }
        /// Machine Software Interrupt Enable
        #[inline]
        pub unsafe fn set_msoft() {
            _set(1 << 3);
        }
        /// Machine Software Interrupt Enable
        #[inline]
        pub unsafe fn clear_msoft() {
            _clear(1 << 3);
        }
        /// User Timer Interrupt Enable
        #[inline]
        pub unsafe fn set_utimer() {
            _set(1 << 4);
        }
        /// User Timer Interrupt Enable
        #[inline]
        pub unsafe fn clear_utimer() {
            _clear(1 << 4);
        }
        /// Supervisor Timer Interrupt Enable
        #[inline]
        pub unsafe fn set_stimer() {
            _set(1 << 5);
        }
        /// Supervisor Timer Interrupt Enable
        #[inline]
        pub unsafe fn clear_stimer() {
            _clear(1 << 5);
        }
        /// Machine Timer Interrupt Enable
        #[inline]
        pub unsafe fn set_mtimer() {
            _set(1 << 7);
        }
        /// Machine Timer Interrupt Enable
        #[inline]
        pub unsafe fn clear_mtimer() {
            _clear(1 << 7);
        }
        /// User External Interrupt Enable
        #[inline]
        pub unsafe fn set_uext() {
            _set(1 << 8);
        }
        /// User External Interrupt Enable
        #[inline]
        pub unsafe fn clear_uext() {
            _clear(1 << 8);
        }
        /// Supervisor External Interrupt Enable
        #[inline]
        pub unsafe fn set_sext() {
            _set(1 << 9);
        }
        /// Supervisor External Interrupt Enable
        #[inline]
        pub unsafe fn clear_sext() {
            _clear(1 << 9);
        }
        /// Machine External Interrupt Enable
        #[inline]
        pub unsafe fn set_mext() {
            _set(1 << 11);
        }
        /// Machine External Interrupt Enable
        #[inline]
        pub unsafe fn clear_mext() {
            _clear(1 << 11);
        }
    }
    pub mod misa {
        //! misa register
        use core::num::NonZeroUsize;
        /// misa register
        pub struct Misa {
            bits: NonZeroUsize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Misa {
            #[inline]
            fn clone(&self) -> Misa {
                {
                    let _: ::core::clone::AssertParamIsClone<NonZeroUsize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Misa {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Misa {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Misa {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Misa");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Machine XLEN
        pub enum MXL {
            XLEN32,
            XLEN64,
            XLEN128,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for MXL {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for MXL {
            #[inline]
            fn clone(&self) -> MXL {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for MXL {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&MXL::XLEN32,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "XLEN32");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&MXL::XLEN64,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "XLEN64");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&MXL::XLEN128,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "XLEN128");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for MXL {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for MXL {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for MXL {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for MXL {
            #[inline]
            fn eq(&self, other: &MXL) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl Misa {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits.get()
            }
            /// Returns the machine xlen.
            pub fn mxl(&self) -> MXL {
                let value = match () {
                    #[cfg(target_pointer_width = "64")]
                    () => (self.bits() >> 62) as u8,
                };
                match value {
                    1 => MXL::XLEN32,
                    2 => MXL::XLEN64,
                    3 => MXL::XLEN128,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// Returns true when the atomic extension is implemented.
            pub fn has_extension(&self, extension: char) -> bool {
                let bit = extension as u8 - 65;
                if bit > 25 {
                    return false;
                }
                self.bits() & (1 << bit) == (1 << bit)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x301) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Option<Misa> {
            let r = unsafe { _read() };
            NonZeroUsize::new(r).map(|bits| Misa { bits })
        }
    }
    pub mod mstatus {
        //! mstatus register
        use bit_field::BitField;
        use core::mem::size_of;
        /// mstatus register
        pub struct Mstatus {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mstatus {
            #[inline]
            fn clone(&self) -> Mstatus {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mstatus {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mstatus {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mstatus {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mstatus");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Additional extension state
        pub enum XS {
            /// All off
            AllOff = 0,
            /// None dirty or clean, some on
            NoneDirtyOrClean = 1,
            /// None dirty, some clean
            NoneDirtySomeClean = 2,
            /// Some dirty
            SomeDirty = 3,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for XS {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for XS {
            #[inline]
            fn clone(&self) -> XS {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for XS {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&XS::AllOff,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "AllOff");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&XS::NoneDirtyOrClean,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "NoneDirtyOrClean");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&XS::NoneDirtySomeClean,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "NoneDirtySomeClean");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&XS::SomeDirty,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SomeDirty");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for XS {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for XS {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for XS {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for XS {
            #[inline]
            fn eq(&self, other: &XS) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        /// Floating-point extension state
        pub enum FS {
            Off = 0,
            Initial = 1,
            Clean = 2,
            Dirty = 3,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for FS {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for FS {
            #[inline]
            fn clone(&self) -> FS {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for FS {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&FS::Off,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Off");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&FS::Initial,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Initial");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&FS::Clean,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Clean");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&FS::Dirty,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Dirty");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for FS {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for FS {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for FS {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for FS {
            #[inline]
            fn eq(&self, other: &FS) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        /// Machine Previous Privilege Mode
        pub enum MPP {
            Machine = 3,
            Supervisor = 1,
            User = 0,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for MPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for MPP {
            #[inline]
            fn clone(&self) -> MPP {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for MPP {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&MPP::Machine,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Machine");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&MPP::Supervisor,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Supervisor");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&MPP::User,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "User");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for MPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for MPP {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for MPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for MPP {
            #[inline]
            fn eq(&self, other: &MPP) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        /// Supervisor Previous Privilege Mode
        pub enum SPP {
            Supervisor = 1,
            User = 0,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for SPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for SPP {
            #[inline]
            fn clone(&self) -> SPP {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for SPP {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&SPP::Supervisor,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Supervisor");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&SPP::User,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "User");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for SPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for SPP {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for SPP {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for SPP {
            #[inline]
            fn eq(&self, other: &SPP) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl Mstatus {
            /// User Interrupt Enable
            #[inline]
            pub fn uie(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Interrupt Enable
            #[inline]
            pub fn sie(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// Machine Interrupt Enable
            #[inline]
            pub fn mie(&self) -> bool {
                self.bits.get_bit(3)
            }
            /// User Previous Interrupt Enable
            #[inline]
            pub fn upie(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Previous Interrupt Enable
            #[inline]
            pub fn spie(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// Machine Previous Interrupt Enable
            #[inline]
            pub fn mpie(&self) -> bool {
                self.bits.get_bit(7)
            }
            /// Supervisor Previous Privilege Mode
            #[inline]
            pub fn spp(&self) -> SPP {
                match self.bits.get_bit(8) {
                    true => SPP::Supervisor,
                    false => SPP::User,
                }
            }
            /// Machine Previous Privilege Mode
            #[inline]
            pub fn mpp(&self) -> MPP {
                match self.bits.get_bits(11..13) {
                    0b00 => MPP::User,
                    0b01 => MPP::Supervisor,
                    0b11 => MPP::Machine,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// Floating-point extension state
            ///
            /// Encodes the status of the floating-point unit,
            /// including the CSR `fcsr` and floating-point data registers `f0–f31`.
            #[inline]
            pub fn fs(&self) -> FS {
                match self.bits.get_bits(13..15) {
                    0b00 => FS::Off,
                    0b01 => FS::Initial,
                    0b10 => FS::Clean,
                    0b11 => FS::Dirty,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// Additional extension state
            ///
            /// Encodes the status of additional user-mode extensions and associated state.
            #[inline]
            pub fn xs(&self) -> XS {
                match self.bits.get_bits(15..17) {
                    0b00 => XS::AllOff,
                    0b01 => XS::NoneDirtyOrClean,
                    0b10 => XS::NoneDirtySomeClean,
                    0b11 => XS::SomeDirty,
                    _ => ::core::panicking::panic("internal error: entered unreachable code"),
                }
            }
            /// Permit Supervisor User Memory access
            #[inline]
            pub fn sum(&self) -> bool {
                self.bits.get_bit(18)
            }
            /// Make eXecutable Readable
            #[inline]
            pub fn mxr(&self) -> bool {
                self.bits.get_bit(19)
            }
            /// Trap Virtual Memory
            ///
            /// If this bit is set, reads or writes to `satp` CSR or execute `sfence.vma`
            /// instruction when in S-mode will raise an illegal instruction exception.
            ///
            /// TVM is hard-wired to 0 when S-mode is not supported.
            #[inline]
            pub fn tvm(&self) -> bool {
                self.bits.get_bit(20)
            }
            /// Timeout Wait
            ///
            /// Indicates that if WFI instruction should be intercepted.
            ///
            /// If this bit is set, when WFI is executed in S-mode, and it does not complete
            /// within an implementation specific, bounded time limit, the WFI instruction will cause
            /// an illegal instruction trap; or could always cause trap then the time limit is zero.
            ///
            /// TW is hard-wired to 0 when S-mode is not supported.
            #[inline]
            pub fn tw(&self) -> bool {
                self.bits.get_bit(21)
            }
            /// Trap SRET
            ///
            /// Indicates that if SRET instruction should be trapped to raise illegal
            /// instruction exception.
            ///
            /// If S-mode is not supported, TSR bit is hard-wired to 0.
            #[inline]
            pub fn tsr(&self) -> bool {
                self.bits.get_bit(22)
            }
            /// Whether either the FS field or XS field
            /// signals the presence of some dirty state
            #[inline]
            pub fn sd(&self) -> bool {
                self.bits.get_bit(size_of::<usize>() * 8 - 1)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x300) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mstatus {
            Mstatus {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x300) : : "volatile"),
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x300) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x300) : : "volatile"),
            }
        }
        /// User Interrupt Enable
        #[inline]
        pub unsafe fn set_uie() {
            _set(1 << 0);
        }
        /// User Interrupt Enable
        #[inline]
        pub unsafe fn clear_uie() {
            _clear(1 << 0);
        }
        /// Supervisor Interrupt Enable
        #[inline]
        pub unsafe fn set_sie() {
            _set(1 << 1);
        }
        /// Supervisor Interrupt Enable
        #[inline]
        pub unsafe fn clear_sie() {
            _clear(1 << 1);
        }
        /// Machine Interrupt Enable
        #[inline]
        pub unsafe fn set_mie() {
            _set(1 << 3);
        }
        /// Machine Interrupt Enable
        #[inline]
        pub unsafe fn clear_mie() {
            _clear(1 << 3);
        }
        /// User Previous Interrupt Enable
        #[inline]
        pub unsafe fn set_upie() {
            _set(1 << 4);
        }
        /// Supervisor Previous Interrupt Enable
        #[inline]
        pub unsafe fn set_spie() {
            _set(1 << 5);
        }
        /// Machine Previous Interrupt Enable
        #[inline]
        pub unsafe fn set_mpie() {
            _set(1 << 7);
        }
        /// Permit Supervisor User Memory access
        #[inline]
        pub unsafe fn set_sum() {
            _set(1 << 18);
        }
        /// Permit Supervisor User Memory access
        #[inline]
        pub unsafe fn clear_sum() {
            _clear(1 << 18);
        }
        /// Make eXecutable Readable
        #[inline]
        pub unsafe fn set_mxr() {
            _set(1 << 19);
        }
        /// Make eXecutable Readable
        #[inline]
        pub unsafe fn clear_mxr() {
            _clear(1 << 19);
        }
        /// Trap Virtual Memory
        #[inline]
        pub unsafe fn set_tvm() {
            _set(1 << 20);
        }
        /// Trap Virtual Memory
        #[inline]
        pub unsafe fn clear_tvm() {
            _clear(1 << 20);
        }
        /// Timeout Wait
        #[inline]
        pub unsafe fn set_tw() {
            _set(1 << 21);
        }
        /// Timeout Wait
        #[inline]
        pub unsafe fn clear_tw() {
            _clear(1 << 21);
        }
        /// Trap SRET
        #[inline]
        pub unsafe fn set_tsr() {
            _set(1 << 22);
        }
        /// Trap SRET
        #[inline]
        pub unsafe fn clear_tsr() {
            _clear(1 << 22);
        }
        /// Supervisor Previous Privilege Mode
        #[inline]
        pub unsafe fn set_spp(spp: SPP) {
            match spp {
                SPP::Supervisor => _set(1 << 8),
                SPP::User => _clear(1 << 8),
            }
        }
        /// Machine Previous Privilege Mode
        #[inline]
        pub unsafe fn set_mpp(mpp: MPP) {
            let mut value = _read();
            value.set_bits(11..13, mpp as usize);
            _write(value);
        }
        /// Floating-point extension state
        #[inline]
        pub unsafe fn set_fs(fs: FS) {
            let mut value = _read();
            value.set_bits(13..15, fs as usize);
            _write(value);
        }
    }
    pub mod mtvec {
        //! mtvec register
        /// mtvec register
        pub struct Mtvec {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mtvec {
            #[inline]
            fn clone(&self) -> Mtvec {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mtvec {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mtvec {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mtvec {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mtvec");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Trap mode
        pub enum TrapMode {
            Direct = 0,
            Vectored = 1,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for TrapMode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for TrapMode {
            #[inline]
            fn clone(&self) -> TrapMode {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for TrapMode {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&TrapMode::Direct,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Direct");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&TrapMode::Vectored,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Vectored");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for TrapMode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for TrapMode {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl ::core::marker::StructuralPartialEq for TrapMode {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for TrapMode {
            #[inline]
            fn eq(&self, other: &TrapMode) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl Mtvec {
            /// Returns the contents of the register as raw bits
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Returns the trap-vector base-address
            pub fn address(&self) -> usize {
                self.bits - (self.bits & 0b11)
            }
            /// Returns the trap-vector mode
            pub fn trap_mode(&self) -> Option<TrapMode> {
                let mode = self.bits & 0b11;
                match mode {
                    0 => Some(TrapMode::Direct),
                    1 => Some(TrapMode::Vectored),
                    _ => None,
                }
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x305) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mtvec {
            Mtvec {
                bits: unsafe { _read() },
            }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x305) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub unsafe fn write(addr: usize, mode: TrapMode) {
            let bits = addr + mode as usize;
            _write(bits);
        }
    }
    pub mod mcause {
        //! mcause register
        /// mcause register
        pub struct Mcause {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mcause {
            #[inline]
            fn clone(&self) -> Mcause {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mcause {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mcause {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mcause {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mcause");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        /// Trap Cause
        pub enum Trap {
            Interrupt(Interrupt),
            Exception(Exception),
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Trap {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Trap {
            #[inline]
            fn clone(&self) -> Trap {
                {
                    let _: ::core::clone::AssertParamIsClone<Interrupt>;
                    let _: ::core::clone::AssertParamIsClone<Exception>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Trap {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Trap::Interrupt(ref __self_0),) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Interrupt");
                        let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0));
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Trap::Exception(ref __self_0),) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Exception");
                        let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0));
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralPartialEq for Trap {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Trap {
            #[inline]
            fn eq(&self, other: &Trap) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            (&Trap::Interrupt(ref __self_0), &Trap::Interrupt(ref __arg_1_0)) => {
                                (*__self_0) == (*__arg_1_0)
                            }
                            (&Trap::Exception(ref __self_0), &Trap::Exception(ref __arg_1_0)) => {
                                (*__self_0) == (*__arg_1_0)
                            }
                            _ => unsafe { ::core::intrinsics::unreachable() },
                        }
                    } else {
                        false
                    }
                }
            }
            #[inline]
            fn ne(&self, other: &Trap) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            (&Trap::Interrupt(ref __self_0), &Trap::Interrupt(ref __arg_1_0)) => {
                                (*__self_0) != (*__arg_1_0)
                            }
                            (&Trap::Exception(ref __self_0), &Trap::Exception(ref __arg_1_0)) => {
                                (*__self_0) != (*__arg_1_0)
                            }
                            _ => unsafe { ::core::intrinsics::unreachable() },
                        }
                    } else {
                        true
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Trap {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Trap {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {
                    let _: ::core::cmp::AssertParamIsEq<Interrupt>;
                    let _: ::core::cmp::AssertParamIsEq<Exception>;
                }
            }
        }
        /// Interrupt
        pub enum Interrupt {
            UserSoft,
            SupervisorSoft,
            MachineSoft,
            UserTimer,
            SupervisorTimer,
            MachineTimer,
            UserExternal,
            SupervisorExternal,
            MachineExternal,
            Unknown,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Interrupt {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Interrupt {
            #[inline]
            fn clone(&self) -> Interrupt {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Interrupt {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Interrupt::UserSoft,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserSoft");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::SupervisorSoft,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorSoft");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::MachineSoft,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "MachineSoft");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::UserTimer,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserTimer");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::SupervisorTimer,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorTimer");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::MachineTimer,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "MachineTimer");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::UserExternal,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserExternal");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::SupervisorExternal,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorExternal");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::MachineExternal,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "MachineExternal");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Interrupt::Unknown,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Unknown");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralPartialEq for Interrupt {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Interrupt {
            #[inline]
            fn eq(&self, other: &Interrupt) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Interrupt {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Interrupt {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        /// Exception
        pub enum Exception {
            InstructionMisaligned,
            InstructionFault,
            IllegalInstruction,
            Breakpoint,
            LoadMisaligned,
            LoadFault,
            StoreMisaligned,
            StoreFault,
            UserEnvCall,
            SupervisorEnvCall,
            MachineEnvCall,
            InstructionPageFault,
            LoadPageFault,
            StorePageFault,
            Unknown,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Exception {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Exception {
            #[inline]
            fn clone(&self) -> Exception {
                {
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Exception {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match (&*self,) {
                    (&Exception::InstructionMisaligned,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "InstructionMisaligned");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::InstructionFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "InstructionFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::IllegalInstruction,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "IllegalInstruction");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::Breakpoint,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Breakpoint");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::LoadMisaligned,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "LoadMisaligned");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::LoadFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "LoadFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::StoreMisaligned,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "StoreMisaligned");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::StoreFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "StoreFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::UserEnvCall,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "UserEnvCall");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::SupervisorEnvCall,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "SupervisorEnvCall");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::MachineEnvCall,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "MachineEnvCall");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::InstructionPageFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "InstructionPageFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::LoadPageFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "LoadPageFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::StorePageFault,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "StorePageFault");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                    (&Exception::Unknown,) => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_tuple(f, "Unknown");
                        ::core::fmt::DebugTuple::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl ::core::marker::StructuralPartialEq for Exception {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::PartialEq for Exception {
            #[inline]
            fn eq(&self, other: &Exception) -> bool {
                {
                    let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                    let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                    if true && __self_vi == __arg_1_vi {
                        match (&*self, &*other) {
                            _ => true,
                        }
                    } else {
                        false
                    }
                }
            }
        }
        impl ::core::marker::StructuralEq for Exception {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::cmp::Eq for Exception {
            #[inline]
            #[doc(hidden)]
            #[no_coverage]
            fn assert_receiver_is_total_eq(&self) -> () {
                {}
            }
        }
        impl Interrupt {
            pub fn from(nr: usize) -> Self {
                match nr {
                    0 => Interrupt::UserSoft,
                    1 => Interrupt::SupervisorSoft,
                    3 => Interrupt::MachineSoft,
                    4 => Interrupt::UserTimer,
                    5 => Interrupt::SupervisorTimer,
                    7 => Interrupt::MachineTimer,
                    8 => Interrupt::UserExternal,
                    9 => Interrupt::SupervisorExternal,
                    11 => Interrupt::MachineExternal,
                    _ => Interrupt::Unknown,
                }
            }
        }
        impl Exception {
            pub fn from(nr: usize) -> Self {
                match nr {
                    0 => Exception::InstructionMisaligned,
                    1 => Exception::InstructionFault,
                    2 => Exception::IllegalInstruction,
                    3 => Exception::Breakpoint,
                    4 => Exception::LoadMisaligned,
                    5 => Exception::LoadFault,
                    6 => Exception::StoreMisaligned,
                    7 => Exception::StoreFault,
                    8 => Exception::UserEnvCall,
                    9 => Exception::SupervisorEnvCall,
                    11 => Exception::MachineEnvCall,
                    12 => Exception::InstructionPageFault,
                    13 => Exception::LoadPageFault,
                    15 => Exception::StorePageFault,
                    _ => Exception::Unknown,
                }
            }
        }
        impl Mcause {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// Returns the code field
            pub fn code(&self) -> usize {
                match () {
                    #[cfg(target_pointer_width = "64")]
                    () => self.bits & !(1 << 63),
                }
            }
            /// Trap Cause
            #[inline]
            pub fn cause(&self) -> Trap {
                if self.is_interrupt() {
                    Trap::Interrupt(Interrupt::from(self.code()))
                } else {
                    Trap::Exception(Exception::from(self.code()))
                }
            }
            /// Is trap cause an interrupt.
            #[inline]
            pub fn is_interrupt(&self) -> bool {
                match () {
                    #[cfg(target_pointer_width = "64")]
                    () => self.bits & (1 << 63) == 1 << 63,
                }
            }
            /// Is trap cause an exception.
            #[inline]
            pub fn is_exception(&self) -> bool {
                !self.is_interrupt()
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x342) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mcause {
            Mcause {
                bits: unsafe { _read() },
            }
        }
    }
    pub mod mepc {
        //! mepc register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x341) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x341) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    }
    pub mod mip {
        //! mip register
        use bit_field::BitField;
        /// mip register
        pub struct Mip {
            bits: usize,
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::clone::Clone for Mip {
            #[inline]
            fn clone(&self) -> Mip {
                {
                    let _: ::core::clone::AssertParamIsClone<usize>;
                    *self
                }
            }
        }
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::marker::Copy for Mip {}
        #[automatically_derived]
        #[allow(unused_qualifications)]
        impl ::core::fmt::Debug for Mip {
            fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
                match *self {
                    Mip {
                        bits: ref __self_0_0,
                    } => {
                        let debug_trait_builder =
                            &mut ::core::fmt::Formatter::debug_struct(f, "Mip");
                        let _ = ::core::fmt::DebugStruct::field(
                            debug_trait_builder,
                            "bits",
                            &&(*__self_0_0),
                        );
                        ::core::fmt::DebugStruct::finish(debug_trait_builder)
                    }
                }
            }
        }
        impl Mip {
            /// Returns the contents of the register as raw bits
            #[inline]
            pub fn bits(&self) -> usize {
                self.bits
            }
            /// User Software Interrupt Pending
            #[inline]
            pub fn usoft(&self) -> bool {
                self.bits.get_bit(0)
            }
            /// Supervisor Software Interrupt Pending
            #[inline]
            pub fn ssoft(&self) -> bool {
                self.bits.get_bit(1)
            }
            /// Machine Software Interrupt Pending
            #[inline]
            pub fn msoft(&self) -> bool {
                self.bits.get_bit(3)
            }
            /// User Timer Interrupt Pending
            #[inline]
            pub fn utimer(&self) -> bool {
                self.bits.get_bit(4)
            }
            /// Supervisor Timer Interrupt Pending
            #[inline]
            pub fn stimer(&self) -> bool {
                self.bits.get_bit(5)
            }
            /// Machine Timer Interrupt Pending
            #[inline]
            pub fn mtimer(&self) -> bool {
                self.bits.get_bit(7)
            }
            /// User External Interrupt Pending
            #[inline]
            pub fn uext(&self) -> bool {
                self.bits.get_bit(8)
            }
            /// Supervisor External Interrupt Pending
            #[inline]
            pub fn sext(&self) -> bool {
                self.bits.get_bit(9)
            }
            /// Machine External Interrupt Pending
            #[inline]
            pub fn mext(&self) -> bool {
                self.bits.get_bit(11)
            }
        }
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x344) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> Mip {
            Mip {
                bits: unsafe { _read() },
            }
        }
        /// Set the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _set(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrs x0, $1, $0" : : "r" (bits) , "i" (0x344) : : "volatile"),
            }
        }
        /// Clear the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _clear(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrc x0, $1, $0" : : "r" (bits) , "i" (0x344) : : "volatile"),
            }
        }
        /// User Software Interrupt Pending
        #[inline]
        pub unsafe fn set_usoft() {
            _set(1 << 0);
        }
        /// User Software Interrupt Pending
        #[inline]
        pub unsafe fn clear_usoft() {
            _clear(1 << 0);
        }
        /// Supervisor Software Interrupt Pending
        #[inline]
        pub unsafe fn set_ssoft() {
            _set(1 << 1);
        }
        /// Supervisor Software Interrupt Pending
        #[inline]
        pub unsafe fn clear_ssoft() {
            _clear(1 << 1);
        }
        /// Machine Software Interrupt Pending
        #[inline]
        pub unsafe fn set_msoft() {
            _set(1 << 3);
        }
        /// Machine Software Interrupt Pending
        #[inline]
        pub unsafe fn clear_msoft() {
            _clear(1 << 3);
        }
        /// User Timer Interrupt Pending
        #[inline]
        pub unsafe fn set_utimer() {
            _set(1 << 4);
        }
        /// User Timer Interrupt Pending
        #[inline]
        pub unsafe fn clear_utimer() {
            _clear(1 << 4);
        }
        /// Supervisor Timer Interrupt Pending
        #[inline]
        pub unsafe fn set_stimer() {
            _set(1 << 5);
        }
        /// Supervisor Timer Interrupt Pending
        #[inline]
        pub unsafe fn clear_stimer() {
            _clear(1 << 5);
        }
        /// Machine Timer Interrupt Pending
        #[inline]
        pub unsafe fn set_mtimer() {
            _set(1 << 7);
        }
        /// Machine Timer Interrupt Pending
        #[inline]
        pub unsafe fn clear_mtimer() {
            _clear(1 << 7);
        }
        /// User External Interrupt Pending
        #[inline]
        pub unsafe fn set_uext() {
            _set(1 << 8);
        }
        /// User External Interrupt Pending
        #[inline]
        pub unsafe fn clear_uext() {
            _clear(1 << 8);
        }
        /// Supervisor External Interrupt Pending
        #[inline]
        pub unsafe fn set_sext() {
            _set(1 << 9);
        }
        /// Supervisor External Interrupt Pending
        #[inline]
        pub unsafe fn clear_sext() {
            _clear(1 << 9);
        }
    }
    pub mod mscratch {
        //! mscratch register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x340) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Writes the CSR
        #[inline]
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            match () {
                #[cfg(riscv)]
                () => llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x340) : : "volatile"),
            }
        }
        /// Writes the CSR
        #[inline]
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    }
    pub mod mtval {
        //! mtval register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x343) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    mod pmpcfgx {
        /// Physical memory protection configuration
        pub mod pmpcfg0 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3A0) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3A0) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection configuration, RV32 only
        pub mod pmpcfg1 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection configuration
        pub mod pmpcfg2 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3A2) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3A2) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection configuration, RV32 only
        pub mod pmpcfg3 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
    }
    pub use self::pmpcfgx::*;
    mod pmpaddrx {
        /// Physical memory protection address register
        pub mod pmpaddr0 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B0) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B0) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr1 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B1) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B1) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr2 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B2) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B2) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr3 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B3) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B3) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr4 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B4) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B4) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr5 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B5) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B5) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr6 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B6) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B6) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr7 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B7) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B7) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr8 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B8) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B8) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr9 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3B9) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3B9) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr10 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3BA) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3BA) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr11 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3BB) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3BB) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr12 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3BC) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3BC) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr13 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3BD) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3BD) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr14 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3BE) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3BE) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Physical memory protection address register
        pub mod pmpaddr15 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x3BF) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x3BF) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
    }
    pub use self::pmpaddrx::*;
    pub mod mcycle {
        //! mcycle register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB00) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Reads the CSR as a 64-bit value
        #[inline]
        pub fn read64() -> u64 {
            match () {
                #[cfg(not(riscv32))]
                () => read() as u64,
            }
        }
    }
    pub mod mcycleh {
        //! mcycleh register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(not(riscv32))]
                () => ::core::panicking::panic("not implemented"),
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    mod mhpmcounterx {
        /// Machine performance-monitoring counter
        pub mod mhpmcounter3 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB03) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB03) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter4 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB04) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB04) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter5 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB05) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB05) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter6 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB06) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB06) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter7 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB07) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB07) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter8 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB08) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB08) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter9 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB09) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB09) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter10 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB0A) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB0A) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter11 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB0B) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB0B) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter12 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB0C) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB0C) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter13 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB0D) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB0D) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter14 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB0E) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB0E) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter15 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB0F) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB0F) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter16 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB10) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB10) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter17 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB11) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB11) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter18 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB12) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB12) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter19 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB13) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB13) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter20 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB14) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB14) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter21 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB15) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB15) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter22 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB16) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB16) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter23 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB17) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB17) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter24 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB18) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB18) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter25 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB19) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB19) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter26 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB1A) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB1A) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter27 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB1B) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB1B) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter28 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB1C) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB1C) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter29 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB1D) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB1D) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter30 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB1E) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB1E) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Machine performance-monitoring counter
        pub mod mhpmcounter31 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB1F) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0xB1F) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
            /// Reads the CSR as a 64-bit value
            #[inline]
            pub fn read64() -> u64 {
                match () {
                    #[cfg(not(riscv32))]
                    () => read() as u64,
                }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter3h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter4h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter5h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter6h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter7h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter8h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter9h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter10h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter11h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter12h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter13h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter14h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter15h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter16h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter17h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter18h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter19h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter20h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter21h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter22h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter23h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter24h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter25h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter26h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter27h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter28h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter29h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter30h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Upper 32 bits of machine performance-monitoring counter (RV32I only)
        pub mod mhpmcounter31h {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(not(riscv32))]
                    () => ::core::panicking::panic("not implemented"),
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
    }
    pub use self::mhpmcounterx::*;
    pub mod minstret {
        //! minstret register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(riscv)]
                () => {
                    let r: usize;
                    llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0xB02) : : "volatile");
                    r
                }
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
        /// Reads the CSR as a 64-bit value
        #[inline]
        pub fn read64() -> u64 {
            match () {
                #[cfg(not(riscv32))]
                () => read() as u64,
            }
        }
    }
    pub mod minstreth {
        //! minstreth register
        /// Reads the CSR
        #[inline]
        unsafe fn _read() -> usize {
            match () {
                #[cfg(not(riscv32))]
                () => ::core::panicking::panic("not implemented"),
            }
        }
        /// Reads the CSR
        #[inline]
        pub fn read() -> usize {
            unsafe { _read() }
        }
    }
    mod mhpmeventx {
        /// Machine performance-monitoring event selector
        pub mod mhpmevent3 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x323) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x323) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent4 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x324) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x324) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent5 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x325) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x325) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent6 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x326) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x326) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent7 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x327) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x327) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent8 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x328) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x328) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent9 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x329) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x329) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent10 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x32A) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x32A) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent11 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x32B) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x32B) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent12 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x32C) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x32C) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent13 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x32D) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x32D) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent14 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x32E) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x32E) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent15 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x32F) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x32F) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent16 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x330) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x330) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent17 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x331) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x331) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent18 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x332) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x332) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent19 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x333) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x333) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent20 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x334) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x334) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent21 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x335) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x335) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent22 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x336) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x336) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent23 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x337) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x337) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent24 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x338) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x338) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent25 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x339) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x339) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent26 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x33A) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x33A) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent27 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x33B) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x33B) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent28 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x33C) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x33C) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent29 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x33D) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x33D) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent30 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x33E) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x33E) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
        /// Machine performance-monitoring event selector
        pub mod mhpmevent31 {
            /// Reads the CSR
            #[inline]
            unsafe fn _read() -> usize {
                match () {
                    #[cfg(riscv)]
                    () => {
                        let r: usize;
                        llvm_asm ! ("csrrs $0, $1, x0" : "=r" (r) : "i" (0x33F) : : "volatile");
                        r
                    }
                }
            }
            /// Reads the CSR
            #[inline]
            pub fn read() -> usize {
                unsafe { _read() }
            }
            /// Writes the CSR
            #[inline]
            #[allow(unused_variables)]
            unsafe fn _write(bits: usize) {
                match () {
                    #[cfg(riscv)]
                    () => {
                        llvm_asm ! ("csrrw x0, $1, $0" : : "r" (bits) , "i" (0x33F) : : "volatile")
                    }
                }
            }
            /// Writes the CSR
            #[inline]
            pub fn write(bits: usize) {
                unsafe { _write(bits) }
            }
        }
    }
    pub use self::mhpmeventx::*;
}

//! Assembly instructions

/// `EBREAK` instruction wrapper
///
/// Generates a breakpoint exception.
pub unsafe fn ebreak() {
    asm!("ebreak");
}

/// `WFI` instruction wrapper
///
/// Provides a hint to the implementation that the current hart can be stalled until an interrupt
/// might need servicing. The WFI instruction is just a hint, and a legal implementation is to
/// implement WFI as a NOP.
pub unsafe fn wfi() {
    asm!("wfi")
}

/// `SFENCE.VMA` instruction wrapper (all address spaces and page table levels)
///
/// Synchronizes updates to in-memory memory-management data structures with current execution.
/// Instruction execution causes implicit reads and writes to these data structures; however, these
/// implicit references are ordinarily not ordered with respect to loads and stores in the
/// instruction stream. Executing an `SFENCE.VMA` instruction guarantees that any stores in the
/// instruction stream prior to the `SFENCE.VMA` are ordered before all implicit references
/// subsequent to the `SFENCE.VMA`.
pub unsafe fn sfence_vma_all() {
    asm!("sfence.vma")
}

/// `SFENCE.VMA` instruction wrapper
///
/// Synchronizes updates to in-memory memory-management data structures with current execution.
/// Instruction execution causes implicit reads and writes to these data structures; however, these
/// implicit references are ordinarily not ordered with respect to loads and stores in the
/// instruction stream. Executing an `SFENCE.VMA` instruction guarantees that any stores in the
/// instruction stream prior to the `SFENCE.VMA` are ordered before all implicit references
/// subsequent to the `SFENCE.VMA`.
pub unsafe fn sfence_vma(asid: usize, addr: usize) {
    asm!("sfence.vma {}, {}", in(reg) asid, in(reg) addr);
}

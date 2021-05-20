//! Provides functions to read and write segment registers.

use crate::structures::gdt::SegmentSelector;

/// Reload code segment register.
///
/// Note this is special since we can not directly move
/// to cs. Instead we push the new segment selector
/// and return value on the stack and use retf
/// to reload cs and continue at 1:.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that `sel`
/// is a valid code segment descriptor.
pub unsafe fn set_cs(sel: SegmentSelector) {
    asm!(
        "push {sel}",
        "lea {tmp}, [1f + rip]",
        "push {tmp}",
        "retfq",
        "1:",
        sel = in(reg) u64::from(sel.0),
        tmp = lateout(reg) _,
        options(preserves_flags),
    );
}

/// Reload stack segment register.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that `sel`
/// is a valid stack segment descriptor.
pub unsafe fn load_ss(sel: SegmentSelector) {
    asm!("mov ss, {0:x}", in(reg) sel.0, options(nostack, preserves_flags));
}

/// Reload data segment register.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that `sel`
/// is a valid data segment descriptor.
pub unsafe fn load_ds(sel: SegmentSelector) {
    asm!("mov ds, {0:x}", in(reg) sel.0, options(nostack, preserves_flags));
}

/// Reload es segment register.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that `sel`
/// is a valid extra segment descriptor.
pub unsafe fn load_es(sel: SegmentSelector) {
    asm!("mov es, {0:x}", in(reg) sel.0, options(nostack, preserves_flags));
}

/// Reload fs segment register.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that `sel`
/// is a valid fs segment descriptor.
pub unsafe fn load_fs(sel: SegmentSelector) {
    asm!("mov fs, {0:x}", in(reg) sel.0, options(nostack, preserves_flags));
}

/// Reload gs segment register.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that `sel`
/// is a valid gs segment descriptor.
pub unsafe fn load_gs(sel: SegmentSelector) {
    asm!("mov gs, {0:x}", in(reg) sel.0, options(nostack, preserves_flags));
}

/// Swap `KernelGsBase` MSR and `GsBase` MSR.
///
/// ## Safety
///
/// This function is unsafe because the caller must ensure that the
/// swap operation cannot lead to undefined behavior.
pub unsafe fn swap_gs() {
    asm!("swapgs", options(nostack, preserves_flags));
}

/// Returns the current value of the code segment register.
pub fn cs() -> SegmentSelector {
    let segment: u16;

    unsafe {
        asm!("mov {0:x}, cs", out(reg) segment, options(nomem, nostack, preserves_flags));
    }

    SegmentSelector(segment)
}

/// Writes the FS segment base address
///
/// ## Safety
///
/// If `CR4.FSGSBASE` is not set, this instruction will throw an `#UD`.
///
/// The caller must ensure that this write operation has no unsafe side
/// effects, as the FS segment base address is often used for thread
/// local storage.
pub unsafe fn wrfsbase(val: u64) {
    asm!("wrfsbase {}", in(reg) val, options(nostack, preserves_flags));
}

/// Reads the FS segment base address
///
/// ## Safety
///
/// If `CR4.FSGSBASE` is not set, this instruction will throw an `#UD`.
pub unsafe fn rdfsbase() -> u64 {
    let val: u64;
    asm!("rdfsbase {}", out(reg) val, options(nomem, nostack, preserves_flags));
    val
}

/// Writes the GS segment base address
///
/// ## Safety
///
/// If `CR4.FSGSBASE` is not set, this instruction will throw an `#UD`.
///
/// The caller must ensure that this write operation has no unsafe side
/// effects, as the GS segment base address might be in use.
pub unsafe fn wrgsbase(val: u64) {
    asm!("wrgsbase {}", in(reg) val, options(nostack, preserves_flags));
}

/// Reads the GS segment base address
///
/// ## Safety
///
/// If `CR4.FSGSBASE` is not set, this instruction will throw an `#UD`.
pub unsafe fn rdgsbase() -> u64 {
    let val: u64;
    asm!("rdgsbase {}", out(reg) val, options(nomem, nostack, preserves_flags));
    val
}

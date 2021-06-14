#![allow(missing_docs)]

use self::handlers::*;
use crate::structures::idt::InterruptDescriptorTable;

mod gdt;
mod handlers;

lazy_static! {
    static ref IDT: InterruptDescriptorTable = {
        let mut idt = InterruptDescriptorTable::new();
        idt.breakpoint.set_handler_fn(breakpoint_handler);
        unsafe {
            idt.double_fault
                .set_handler_fn(double_fault_handler)
                .set_stack_index(self::gdt::DOUBLE_FAULT_IST_INDEX);
        }
        idt
    };
}

/// Initialize interrupts handlers
pub fn init() {
    self::gdt::init();
    IDT.load();
}

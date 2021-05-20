//! Abstractions for page tables and other paging related structures.
//!
//! Page tables translate virtual memory “pages” to physical memory “frames”.

#[doc(no_inline)]
pub use self::mapper::MappedPageTable;
#[cfg(target_pointer_width = "64")]
#[doc(no_inline)]
pub use self::mapper::OffsetPageTable;
#[doc(no_inline)]
pub use self::mapper::RecursivePageTable;
pub use self::{
    frame::PhysFrame,
    frame_alloc::{
        FrameAllocator,
        FrameDeallocator,
    },
    mapper::{
        Mapper,
        Translate,
    },
    page::{
        Page,
        PageSize,
        Size1GiB,
        Size2MiB,
        Size4KiB,
    },
    page_table::{
        PageOffset,
        PageTable,
        PageTableFlags,
        PageTableIndex,
    },
};

pub mod frame;
mod frame_alloc;
pub mod mapper;
pub mod page;
pub mod page_table;

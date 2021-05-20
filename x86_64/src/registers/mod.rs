//! Access to various system and model specific registers.

pub mod control;
pub mod model_specific;
pub mod rflags;
pub mod xcontrol;

pub use crate::instructions::{
    read_rip,
    segmentation::{
        rdfsbase,
        rdgsbase,
        wrfsbase,
        wrgsbase,
    },
};

macro_rules! read_csr {
    ($csr_number:expr, $asm_fn: ident) => {
        /// Reads the CSR
        unsafe fn _read() -> usize {
            let r: usize;
            asm!("csrrs {}, {}, x0", lateout(reg) r, in(reg) $csr_number);
            r
        }
    };
}

macro_rules! read_csr_rv32 {
    ($csr_number:expr, $asm_fn: ident) => {
        /// Reads the CSR
        unsafe fn _read() -> usize {
            let r: usize;
            asm!("csrrs {}, {}, x0", lateout(reg) r, in(reg) $csr_number);
            r
        }
    };
}

macro_rules! read_csr_as {
    ($register:ident, $csr_number:expr, $asm_fn:ident) => {
        read_csr!($csr_number, $asm_fn);

        /// Reads the CSR
        pub fn read() -> $register {
            $register {
                bits: unsafe { _read() },
            }
        }
    };
}

macro_rules! read_csr_as_usize {
    ($csr_number:expr, $asm_fn:ident) => {
        read_csr!($csr_number, $asm_fn);

        /// Reads the CSR
        pub fn read() -> usize {
            unsafe { _read() }
        }
    };
}

macro_rules! read_csr_as_usize_rv32 {
    ($csr_number:expr, $asm_fn:ident) => {
        read_csr_rv32!($csr_number, $asm_fn);

        /// Reads the CSR
        pub fn read() -> usize {
            unsafe { _read() }
        }
    };
}

macro_rules! write_csr {
    ($csr_number:expr, $asm_fn: ident) => {
        /// Writes the CSR
        #[allow(unused_variables)]
        unsafe fn _write(bits: usize) {
            asm!("csrrw x0, {}, {}", in(reg) $csr_number, in(reg) bits);
        }
    };
}

macro_rules! write_csr_rv32 {
    ($csr_number:expr, $asm_fn: ident) => {
        /// Writes the CSR
        unsafe fn _write(bits: usize) {
            asm!("csrrw x0, {}, {}", in(reg) $csr_number, in(reg) bits);
        }
    };
}

macro_rules! write_csr_as_usize {
    ($csr_number:expr, $asm_fn:ident) => {
        write_csr!($csr_number, $asm_fn);

        /// Writes the CSR
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    };
}

macro_rules! write_csr_as_usize_rv32 {
    ($csr_number:expr, $asm_fn:ident) => {
        write_csr_rv32!($csr_number, $asm_fn);

        /// Writes the CSR
        pub fn write(bits: usize) {
            unsafe { _write(bits) }
        }
    };
}

macro_rules! set {
    ($csr_number:expr, $asm_fn: ident) => {
        /// Set the CSR
        unsafe fn _set(bits: usize) {
            asm!("csrrs x0, {}, {}", in(reg) $csr_number, in(reg) bits);
        }
    };
}

macro_rules! clear {
    ($csr_number:expr, $asm_fn: ident) => {
        /// Clear the CSR
        unsafe fn _clear(bits: usize) {
            asm!("csrrc x0, {}, {}", in(reg) $csr_number, in(reg) bits);
        }
    };
}

macro_rules! set_csr {
    ($(#[$attr:meta])*, $set_field:ident, $e:expr) => {
        $(#[$attr])*
        pub unsafe fn $set_field() {
            _set($e);
        }
    }
}

macro_rules! clear_csr {
    ($(#[$attr:meta])*, $clear_field:ident, $e:expr) => {
        $(#[$attr])*
        pub unsafe fn $clear_field() {
            _clear($e);
        }
    }
}

macro_rules! set_clear_csr {
    ($(#[$attr:meta])*, $set_field:ident, $clear_field:ident, $e:expr) => {
        set_csr!($(#[$attr])*, $set_field, $e);
        clear_csr!($(#[$attr])*, $clear_field, $e);
    }
}

macro_rules! read_composite_csr {
    ($hi:expr, $lo:expr) => {
        /// Reads the CSR as a 64-bit value
        pub fn read64() -> u64 {
            match () {
                #[cfg(riscv32)]
                () => loop {
                    let hi = $hi;
                    let lo = $lo;
                    if hi == $hi {
                        return ((hi as u64) << 32) | lo as u64;
                    }
                },

                #[cfg(not(riscv32))]
                () => $lo as u64,
            }
        }
    };
}

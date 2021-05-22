fn main() {
    cfg_aliases::cfg_aliases! {
        riscv: { any(riscv32, riscv64) },
        riscv32: { target_arch = "riscv32" },
        riscv64: { target_arch = "riscv64" },
    }
}

fn main() {
    cfg_aliases::cfg_aliases! {
        riscv: { any(target_arch = "riscv32", target_arch = "riscv64") },
        x86_64: { target_arch = "x86_64" },
    }
}

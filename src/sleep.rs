use riscv::register::mcycle;

pub fn cycle_sleep(n: usize) {
    let start = mcycle::read();
    while (mcycle::read().wrapping_sub(start)) < n {
        // IDLE
    }
}

pub fn usleep(n: usize) {
    let freq = 390_000_000 as usize;
    cycle_sleep(freq * n / 1000000);
}

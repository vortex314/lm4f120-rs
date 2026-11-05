//! µDMA (micro-DMA) controller driver.
//!
//! The LM4F120 has a 32-channel µDMA controller with a software-managed
//! control table (1024-byte aligned) in SRAM. Each channel has a primary and
//! an alternate 4-word control structure; only the primary is used here.

use crate::{read_reg, write_reg};

// µDMA controller registers.
const UDMA_CFG: u32 = 0x400F_F004;
const UDMA_CTLBASE: u32 = 0x400F_F008;
const UDMA_ENASET: u32 = 0x400F_F028;
const UDMA_ENACLR: u32 = 0x400F_F02C;
const SYSCTL_RCGCDMA: u32 = 0x400F_E60C;

/// Control table: 32 primary + 32 alternate control structures, 4 words each.
#[repr(C, align(1024))]
struct ControlTable {
    ch: [[u32; 4]; 64],
}

static mut CONTROL_TABLE: ControlTable = ControlTable { ch: [[0; 4]; 64] };

/// Enable the µDMA controller and point it at the control table.
pub fn init() {
    unsafe {
        // Enable the µDMA clock (gated at reset).
        write_reg(SYSCTL_RCGCDMA, read_reg(SYSCTL_RCGCDMA) | 0x01);
        for _ in 0..32 {
            core::hint::spin_loop();
        }

        let base = core::ptr::addr_of!(CONTROL_TABLE) as u32;
        write_reg(UDMA_CTLBASE, base);
        write_reg(UDMA_CFG, read_reg(UDMA_CFG) | 0x01); // MASTEN
    }
}

/// Configure a channel's primary control structure for a basic transfer.
///
/// `src_end`/`dst_end` are the last (inclusive) byte addresses. `src_inc` and
/// `dst_inc` select whether the address increments after each byte; `count`
/// is the number of bytes (1..=1024).
pub fn setup_channel(channel: u32, src_end: u32, dst_end: u32, src_inc: bool, dst_inc: bool, count: u32) {
    let ctl = build_chctl(src_inc, dst_inc, count);
    unsafe {
        let p = (core::ptr::addr_of_mut!(CONTROL_TABLE) as *mut u32).add((channel as usize) * 4);
        p.add(0).write_volatile(src_end);
        p.add(1).write_volatile(dst_end);
        p.add(2).write_volatile(ctl);
        p.add(3).write_volatile(0);
    }
}

/// Enable a channel (begins the transfer).
pub fn enable_channel(channel: u32) {
    unsafe { write_reg(UDMA_ENASET, 1 << channel) };
}

/// Disable a channel.
pub fn disable_channel(channel: u32) {
    unsafe { write_reg(UDMA_ENACLR, 1 << channel) };
}

/// Returns true if the channel has finished (it auto-disables on completion).
pub fn channel_done(channel: u32) -> bool {
    unsafe { read_reg(UDMA_ENASET) & (1 << channel) == 0 }
}

fn build_chctl(src_inc: bool, dst_inc: bool, count: u32) -> u32 {
    // 8-bit data size, single-transfer arbitration.
    let mut ctl = (0b00 << 28) | (0b00 << 24) | (0b0000 << 18);
    // Source increment: +1 byte if memory, none if peripheral register.
    ctl |= if src_inc { 0b00 << 26 } else { 0b11 << 26 };
    // Destination increment: +1 byte if memory, none if peripheral register.
    ctl |= if dst_inc { 0b00 << 30 } else { 0b11 << 30 };
    // Transfer size (count - 1), 10 bits, in basic transfer mode.
    ctl | ((count.wrapping_sub(1) & 0x3FF) << 4) | 0x01
}

use crate::clock::Clocks;
use crate::dmac::{Dmac, DmacChannel, Inc, Msize, TrWidth};
use crate::pac::SPI0;
use crate::sysctl::{self, DmaSelect, APB2};
use crate::time::Hertz;
pub use embedded_hal::spi::{Mode, Phase, Polarity};

/** Borrow frame format from pac */
pub use crate::pac::spi0::ctrlr0::FRAME_FORMAT_A as FrameFormat;
/** Borrow tmod from pac */
pub use crate::pac::spi0::ctrlr0::TMOD_A as Tmod;
/** Borrow work mode from pac */
pub use crate::pac::spi0::ctrlr0::WORK_MODE_A as WorkMode;
/** Borrow aitm from pac */
pub use crate::pac::spi0::spi_ctrlr0::AITM_A as Aitm;

pub trait Spi0Ext: Sized {
    fn constrain(self, apb2: &mut APB2) -> Spi0;
}

impl Spi0Ext for SPI0 {
    fn constrain(self, apb2: &mut APB2) -> Spi0 {
        apb2.enable();
        Spi0 { spi: self }
    }
}

pub struct Spi0 {
    spi: SPI0,
}

pub trait Spi {
    fn configure(
        &mut self,
        work_mode: WorkMode,
        frame_format: FrameFormat,
        data_length: u8,
        endian: u32,
        instruction_length: u8,
        address_length: u8,
        wait_cycles: u8,
        instruction_address_trans_mode: Aitm,
        tmod: Tmod,
    );
    fn set_clk_rate(&mut self, spi_clk: Hertz, clocks: &Clocks) -> Hertz;
    fn send_data<X: Into<u32> + Copy>(&mut self, chip_select: u8, tx: &[X]);
    fn send_data_dma(&mut self, dmac: &mut Dmac, channel: DmacChannel, chip_select: u8, tx: &[u32]);
}

impl Spi for Spi0 {
    fn configure(
        &mut self,
        work_mode: WorkMode,
        frame_format: FrameFormat,
        data_length: u8,
        endian: u32,
        instruction_length: u8,
        address_length: u8,
        wait_cycles: u8,
        instruction_address_trans_mode: Aitm,
        tmod: Tmod,
    ) {
        assert!(data_length >= 4 && data_length <= 32);
        assert!(wait_cycles < (1 << 5));
        let inst_l: u8 = match instruction_length {
            0 => 0,
            4 => 1,
            8 => 2,
            16 => 3,
            _ => panic!("unhandled instruction length"),
        };

        assert!(address_length % 4 == 0 && address_length <= 60);
        let addr_l: u8 = address_length / 4;

        unsafe {
            self.spi.imr.write(|w| w.bits(0x00));
            self.spi.dmacr.write(|w| w.bits(0x00));
            self.spi.dmatdlr.write(|w| w.bits(0x10));
            self.spi.dmardlr.write(|w| w.bits(0x00));
            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));
            self.spi.ctrlr0.write(|w| {
                w.work_mode()
                    .variant(work_mode)
                    .tmod()
                    .variant(tmod)
                    .frame_format()
                    .variant(frame_format)
                    .data_length()
                    .bits(data_length - 1)
            });
            self.spi.spi_ctrlr0.write(|w| {
                w.aitm()
                    .variant(instruction_address_trans_mode)
                    .addr_length()
                    .bits(addr_l)
                    .inst_length()
                    .bits(inst_l)
                    .wait_cycles()
                    .bits(wait_cycles)
            });
            self.spi.endian.write(|w| w.bits(endian));
        }
    }

    fn set_clk_rate(&mut self, spi_clk: Hertz, clocks: &Clocks) -> Hertz {
        sysctl::clk_en_cent().modify(|_, w| w.apb2_clk_en().set_bit());
        sysctl::clk_en_peri().modify(|_, w| w.spi0_clk_en().set_bit());
        let freq_in = clocks.pll0().0;
        let baudrate = freq_in / spi_clk.0;
        let baudrate = baudrate.max(2).min(65534);
        unsafe {
            self.spi.baudr.write(|w| w.bits(baudrate));
        }
        Hertz(freq_in / baudrate)
    }

    fn send_data<X: Into<u32> + Copy>(&mut self, chip_select: u8, tx: &[X]) {
        unsafe {
            self.spi.ser.write(|w| w.bits(1 << chip_select));
            self.spi.ssienr.write(|w| w.bits(0x01));

            let mut fifo_len = 0;
            for &val in tx {
                while fifo_len == 0 {
                    fifo_len = 32 - self.spi.txflr.read().bits();
                }
                self.spi.dr[0].write(|w| w.bits(val.into()));
                fifo_len -= 1;
            }

            while (self.spi.sr.read().bits() & 0x05) != 0x04 {
                // IDLE
            }

            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));
        }
    }

    fn send_data_dma(
        &mut self,
        dmac: &mut Dmac,
        channel: DmacChannel,
        chip_select: u8,
        tx: &[u32],
    ) {
        unsafe {
            self.spi.dmacr.write(|w| w.bits(0x2));
            self.spi.ssienr.write(|w| w.bits(0x1));

            sysctl::set_dma_sel(channel, DmaSelect::SSI0_TX_REQ);
            dmac.set_single_mode(
                channel,
                tx.as_ptr() as u64,
                self.spi.dr.as_ptr() as u64,
                Inc::INCREMENT,
                Inc::NOCHANGE,
                TrWidth::WIDTH_32,
                Msize::LENGTH_4,
                tx.len() as u32,
            );

            self.spi.ser.write(|w| w.bits(1 << chip_select));

            // this line of code is busy waiting
            // @todo -> remove busy waiting
            dmac.wait_done(channel);

            while (self.spi.sr.read().bits() & 0x05) != 0x04 {
                // IDLE
            }

            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));
        }
    }
}

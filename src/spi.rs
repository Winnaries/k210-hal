use crate::clock::Clocks;
use crate::dmac::{Dmac, DmacChannel, Inc, Msize, TrWidth};
use crate::pac::{self, SPI0, SPI1};
use crate::sysctl::{self, DmaSelect, APB2};
use crate::time::Hertz;
use core::ops::Deref;

/** Borrow frame format from pac */
pub use crate::pac::spi0::ctrlr0::FRAME_FORMAT_A as FrameFormat;
/** Borrow tmod from pac */
pub use crate::pac::spi0::ctrlr0::TMOD_A as Tmod;
/** Borrow work mode from pac */
pub use crate::pac::spi0::ctrlr0::WORK_MODE_A as WorkMode;
/** Borrow aitm from pac */
pub use crate::pac::spi0::spi_ctrlr0::AITM_A as Aitm;
/** Embedded HAL SPI traits */
pub use embedded_hal::spi::FullDuplex;

pub trait SpiExt: Sized {
    fn constrain(self, apb2: &mut APB2) -> Spi<Self>;
}

pub trait Spi01: Deref<Target = pac::spi0::RegisterBlock> {
    const NUMBER: SpiNumber;
}

pub enum SpiNumber {
    Spi0,
    Spi1,
}

impl Spi01 for SPI0 {
    const NUMBER: SpiNumber = SpiNumber::Spi0;
}

impl Spi01 for SPI1 {
    const NUMBER: SpiNumber = SpiNumber::Spi1;
}

impl<SPI: Spi01> SpiExt for SPI {
    fn constrain(self, apb2: &mut APB2) -> Spi<SPI> {
        apb2.enable();
        Spi {
            spi: self,
            slave_select: None,
        }
    }
}
pub struct Spi<SPI> {
    spi: SPI,
    slave_select: Option<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiError {
    NoSlaveSelect,
    NoClockRateSpecified,
    WillCauseMemoryError,
}

impl<SPI: Spi01> Spi<SPI> {
    /// Configure the SPI before transferring.
    pub fn configure(
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

    /// Must call this function to set the clock rate of SPI before begin transferring,
    /// otherwise `SpiError::NoClockRateSpecified`. The rate must be sufficiently slower than CPU clock.
    pub fn set_clk_rate(&mut self, spi_clk: Hertz, clocks: &Clocks) -> Hertz {
        match SPI::NUMBER {
            SpiNumber::Spi0 => sysctl::clk_en_peri().modify(|_, w| w.spi0_clk_en().set_bit()),
            SpiNumber::Spi1 => sysctl::clk_en_peri().modify(|_, w| w.spi1_clk_en().set_bit()),
        }
        let freq_in = clocks.pll0().0;
        let baudrate = freq_in / spi_clk.0;
        let baudrate = baudrate.max(2).min(65534);
        unsafe {
            self.spi.baudr.write(|w| w.bits(baudrate));
        }
        Hertz(freq_in / baudrate)
    }

    /// Set to `None` to deselect and set to `Some([0-3])` before begin transferring.
    pub fn set_slave_select(&mut self, ss: Option<u8>) {
        self.slave_select = ss;
    }

    /// Untested, might not work
    pub fn recv_data_dma<X: Into<u32> + Copy>(
        &self,
        dmac: &mut Dmac,
        channel: DmacChannel,
        rx: &mut [X],
    ) -> nb::Result<(), SpiError> {
        if rx.len() == 0 {
            return Ok(());
        }
        unsafe {
            if self.slave_select.is_none() {
                return Err(nb::Error::Other(SpiError::NoSlaveSelect));
            }

            if match SPI::NUMBER {
                SpiNumber::Spi0 => !sysctl::clk_en_peri().read().spi0_clk_en().bit(),
                SpiNumber::Spi1 => !sysctl::clk_en_peri().read().spi1_clk_en().bit(),
            } {
                return Err(nb::Error::Other(SpiError::NoClockRateSpecified));
            }

            self.spi.ctrlr1.write(|w| w.bits(rx.len() as u32 - 1));
            self.spi.ssienr.write(|w| w.bits(0x01));
            self.spi.dmacr.write(|w| w.bits(0x3)); /*enable dma receive */

            sysctl::set_dma_sel(
                channel,
                match SPI::NUMBER {
                    SpiNumber::Spi0 => DmaSelect::SSI0_RX_REQ,
                    SpiNumber::Spi1 => DmaSelect::SSI1_RX_REQ,
                },
            );

            dmac.set_single_mode(
                channel,
                self.spi.dr.as_ptr() as u64,
                rx.as_ptr() as u64,
                Inc::NOCHANGE,
                Inc::INCREMENT,
                TrWidth::WIDTH_32,
                Msize::LENGTH_1,
                rx.len() as u32,
            );
            self.spi.dr[0].write(|w| w.bits(0xffffffff));
            self.spi
                .ser
                .write(|w| w.bits(1 << self.slave_select.unwrap()));
            dmac.wait_done(channel);

            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));

            Ok(())
        }
    }

    /// Using direct memory access to transfer data from source address to SPI.
    /// (TODO: Move this into an isolate `FullDuplex` implementation which returns `DmaTransfer`.)
    pub fn send_data_dma(
        &mut self,
        dmac: &mut Dmac,
        channel: DmacChannel,
        tx: &[u32],
    ) -> nb::Result<(), SpiError> {
        unsafe {
            if self.slave_select.is_none() {
                return Err(nb::Error::Other(SpiError::NoSlaveSelect));
            }

            self.spi.dmacr.write(|w| w.bits(0x2));
            self.spi.ssienr.write(|w| w.bits(0x1));

            sysctl::set_dma_sel(
                channel,
                match SPI::NUMBER {
                    SpiNumber::Spi0 => DmaSelect::SSI0_TX_REQ,
                    SpiNumber::Spi1 => DmaSelect::SSI1_TX_REQ,
                },
            );

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

            self.spi
                .ser
                .write(|w| w.bits(1 << self.slave_select.unwrap()));

            // this line of code is busy waiting
            // @todo -> remove busy waiting
            dmac.wait_done(channel);

            while (self.spi.sr.read().bits() & 0x05) != 0x04 {
                // IDLE
            }

            self.spi.ser.write(|w| w.bits(0x00));
            self.spi.ssienr.write(|w| w.bits(0x00));

            Ok(())
        }
    }
}

macro_rules! impl_simple_full_duplex {
    ($spi:ident; $t:ty) => {
        impl<SPI: $spi> FullDuplex<$t> for Spi<SPI> {
            type Error = SpiError;

            /// Untested, might not work
            fn try_read(&mut self) -> nb::Result<$t, Self::Error> {
                unsafe {
                    if self.slave_select.is_none() {
                        return Err(nb::Error::Other(SpiError::NoSlaveSelect));
                    }

                    if match SPI::NUMBER {
                        SpiNumber::Spi0 => !sysctl::clk_en_peri().read().spi0_clk_en().bit(),
                        SpiNumber::Spi1 => !sysctl::clk_en_peri().read().spi1_clk_en().bit(),
                    } {
                        return Err(nb::Error::Other(SpiError::NoClockRateSpecified));
                    }

                    self.spi.ctrlr1.write(|w| w.bits(0x00));
                    self.spi.ssienr.write(|w| w.bits(0x01));
                    self.spi.dr[0].write(|w| w.bits(0xffffffff));
                    self.spi
                        .ser
                        .write(|w| w.bits(1 << self.slave_select.unwrap()));

                    let rx;

                    while self.spi.rxflr.read().bits() == 0 {
                        // IDLE
                    }

                    rx = self.spi.dr[0].read().bits().max(0).min(<$t>::MAX as u32) as $t;

                    self.spi.ser.write(|w| w.bits(0x00));
                    self.spi.ssienr.write(|w| w.bits(0x00));

                    Ok(rx)
                }
            }

            fn try_send(&mut self, word: $t) -> nb::Result<(), Self::Error> {
                unsafe {
                    if self.slave_select.is_none() {
                        return Err(nb::Error::Other(SpiError::NoSlaveSelect));
                    }

                    if match SPI::NUMBER {
                        SpiNumber::Spi0 => !sysctl::clk_en_peri().read().spi0_clk_en().bit(),
                        SpiNumber::Spi1 => !sysctl::clk_en_peri().read().spi1_clk_en().bit(),
                    } {
                        return Err(nb::Error::Other(SpiError::NoClockRateSpecified));
                    }

                    self.spi
                        .ser
                        .write(|w| w.bits(1 << self.slave_select.unwrap()));
                    self.spi.ssienr.write(|w| w.bits(0x01));

                    while 32 - self.spi.txflr.read().bits() == 0 {
                        // wait until shift register is available
                    }

                    self.spi.dr[0].write(|w| w.bits(word as u32));

                    while (self.spi.sr.read().bits() & 0x05) != 0x04 {
                        // IDLE
                    }

                    self.spi.ser.write(|w| w.bits(0x00));
                    self.spi.ssienr.write(|w| w.bits(0x00));

                    Ok(())
                }
            }
        }
    };
}

impl_simple_full_duplex!(Spi01; u8);
impl_simple_full_duplex!(Spi01; u16);
impl_simple_full_duplex!(Spi01; u32);

impl<SPI: Spi01, X: Into<u32> + Copy> FullDuplex<&[X]> for Spi<SPI> {
    type Error = SpiError;

    /// Unimplemented and will return `Error`. Seems impossible with lifetime, hence the
    /// use of `'static` as a workaround to meet lifetime requirement. After all, this
    /// function might not be so useful as SPI peripheral rarely response as a stream of bytes.
    /// Any suggestion from the crowd?
    fn try_read(&mut self) -> nb::Result<&'static [X], Self::Error> {
        // unimplemented!();
        Err(nb::Error::Other(SpiError::WillCauseMemoryError))
    }

    fn try_send<'a>(&mut self, tx: &'a [X]) -> nb::Result<(), Self::Error> {
        unsafe {
            if self.slave_select.is_none() {
                return Err(nb::Error::Other(SpiError::NoSlaveSelect));
            }

            self.spi
                .ser
                .write(|w| w.bits(1 << self.slave_select.unwrap()));
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

        Ok(())
    }
}

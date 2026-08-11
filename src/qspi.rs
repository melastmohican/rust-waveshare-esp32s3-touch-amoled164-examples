//! QSPI Device Adapter for esp-hal on ESP32-S3

use defmt::info;
use display_driver_qspi::{LineMode, PhaseConfig, QspiDevice, QspiTransaction};
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    spi::master::{Address, Command, DataMode},
};

/// Wraps an esp-hal `SpiDma<Async>` to implement the [`QspiDevice`] trait
/// required by `QspiDisplayBus`.
pub struct EspHalQspiDevice<'d> {
    pub spi: Option<esp_hal::spi::master::SpiDma<'d, esp_hal::Async>>,
    pub rx_buf: Option<DmaRxBuf>,
    pub tx_descriptors: Option<&'static mut [esp_hal::dma::DmaDescriptor]>,
    pub bounce_buf: Option<&'static mut [u8]>,
}

impl<'d> QspiDevice for EspHalQspiDevice<'d> {
    type Error = esp_hal::spi::Error;

    async fn write(
        &mut self,
        transaction: &QspiTransaction,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let cmd = to_esp_command(&transaction.instruction);
        let addr = to_esp_address(&transaction.address);
        let data_mode = to_esp_data_mode(transaction.data_mode);

        let tx_descriptors = self.tx_descriptors.take().unwrap();
        let bounce_buf_full = self.bounce_buf.take().unwrap();

        let tx_buf = match DmaOperationKind::for_write(data) {
            DmaOperationKind::InPlace => {
                let data_ptr = data.as_ptr() as *mut u8;
                let data_static: &'static mut [u8] =
                    unsafe { core::slice::from_raw_parts_mut(data_ptr, data.len()) };
                DmaTxBuf::new(tx_descriptors, data_static).unwrap()
            }
            DmaOperationKind::Copied => {
                if data.len() > bounce_buf_full.len() {
                    panic!(
                        "Data length {} exceeds bounce buffer size {}",
                        data.len(),
                        bounce_buf_full.len()
                    );
                }
                let bounce_slice = &mut bounce_buf_full[..data.len()];
                bounce_slice.copy_from_slice(data);

                let data_ptr = bounce_slice.as_mut_ptr();
                let data_static: &'static mut [u8] =
                    unsafe { core::slice::from_raw_parts_mut(data_ptr, data.len()) };
                DmaTxBuf::new(tx_descriptors, data_static).unwrap()
            }
        };

        let mut transfer = self
            .spi
            .take()
            .unwrap()
            .half_duplex_write(data_mode, cmd, addr, transaction.dummy_cycles, data.len(), tx_buf)
            .map_err(|e| {
                info!("half_duplex_write error: {}", defmt::Debug2Format(&e));
                esp_hal::spi::Error::Unsupported
            })?;

        transfer.wait_for_done().await;

        let (spi, buf) = transfer.wait();
        let (desc, _) = buf.split();

        self.spi = Some(spi);
        self.tx_descriptors = Some(desc);
        self.bounce_buf = Some(bounce_buf_full);
        Ok(())
    }

    async fn read(
        &mut self,
        transaction: &QspiTransaction,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        let cmd = to_esp_command(&transaction.instruction);
        let addr = to_esp_address(&transaction.address);
        let data_mode = to_esp_data_mode(transaction.data_mode);

        let mut rx_buf = self.rx_buf.take().unwrap();
        rx_buf.set_length(buffer.len());

        let mut transfer = self
            .spi
            .take()
            .unwrap()
            .half_duplex_read(data_mode, cmd, addr, transaction.dummy_cycles, buffer.len(), rx_buf)
            .map_err(|_| esp_hal::spi::Error::Unsupported)?;

        transfer.wait_for_done().await;

        let (spi, buf) = transfer.wait();

        buffer.copy_from_slice(&buf.as_slice()[..buffer.len()]);

        self.spi = Some(spi);
        self.rx_buf = Some(buf);
        Ok(())
    }
}

fn is_slice_in_dram(data: &[u8]) -> bool {
    let ptr = data.as_ptr() as usize;
    ptr >= 0x3FC0_0000 && ptr < 0x4000_0000
}

enum DmaOperationKind {
    Copied,
    InPlace,
}

impl DmaOperationKind {
    fn for_write(buffer: &[u8]) -> Self {
        if is_slice_in_dram(buffer) {
            DmaOperationKind::InPlace
        } else {
            DmaOperationKind::Copied
        }
    }
}

fn to_esp_data_mode(mode: LineMode) -> DataMode {
    match mode {
        LineMode::None | LineMode::Single => DataMode::Single,
        LineMode::Dual => DataMode::Dual,
        LineMode::Quad => DataMode::Quad,
    }
}

fn to_esp_command(phase: &Option<PhaseConfig>) -> Command {
    match phase {
        None => Command::None,
        Some(cfg) => {
            let mode = to_esp_data_mode(cfg.mode);
            match cfg.bytes_len {
                1 => Command::_8Bit(cfg.value as u16, mode),
                2 => Command::_16Bit(cfg.value as u16, mode),
                _ => Command::_8Bit(cfg.value as u16, mode),
            }
        }
    }
}

fn to_esp_address(phase: &Option<PhaseConfig>) -> Address {
    match phase {
        None => Address::None,
        Some(cfg) => {
            let mode = to_esp_data_mode(cfg.mode);
            match cfg.bytes_len {
                1 => Address::_8Bit(cfg.value, mode),
                2 => Address::_16Bit(cfg.value, mode),
                3 => Address::_24Bit(cfg.value, mode),
                4 => Address::_32Bit(cfg.value, mode),
                _ => Address::_24Bit(cfg.value, mode),
            }
        }
    }
}

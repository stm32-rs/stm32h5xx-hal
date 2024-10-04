use embedded_hal::i2c;

use super::*;

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match self {
            Error::Bus => i2c::ErrorKind::Bus,
            Error::Arbitration => i2c::ErrorKind::ArbitrationLoss,
            // TODO: Use actual source
            Error::NotAcknowledge => {
                i2c::ErrorKind::NoAcknowledge(i2c::NoAcknowledgeSource::Unknown)
            }
            _ => i2c::ErrorKind::Other,
        }
    }
}

impl<I2C> i2c::ErrorType for I2c<I2C> {
    type Error = Error;
}

trait OperationExt {
    fn direction(&self) -> Direction;
    fn length(&self) -> usize;
}

impl OperationExt for i2c::Operation<'_> {
    fn direction(&self) -> Direction {
        match self {
            Self::Read(_) => Direction::Read,
            Self::Write(_) => Direction::Write,
        }
    }

    fn length(&self) -> usize {
        match self {
            Self::Read(b) => b.len(),
            Self::Write(b) => b.len(),
        }
    }
}

impl<I2C: Instance> I2c<I2C> {
    fn process_operation(
        &mut self,
        stop: Stop,
        op: &mut i2c::Operation<'_>,
    ) -> Result<(), Error> {
        nb::block!(self.is_start_sequence_complete_nb())?;

        match op {
            i2c::Operation::Read(buffer) => self.read_all(buffer)?,
            i2c::Operation::Write(data) => self.write_all(data)?,
        }

        match stop {
            Stop::RepeatStart => {
                nb::block!(self.is_transmit_complete_nb())
            }
            Stop::Reload => nb::block!(self.is_reload_ready_nb()),
            Stop::Automatic => nb::block!(self.is_stopped_nb()),
        }
    }

    fn transaction(
        &mut self,
        address: u16,
        address_mode: AddressMode,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Error> {
        let mut reload = false;
        for i in 0..operations.len() {
            // We need to look at the next operation to determine whether we enable automatic stop
            // tell the peripheral we're going to reload.
            let op = &operations[i];
            let is_last_op = i == operations.len() - 1;
            let stop = if is_last_op {
                // If this is the last operation, enable automatic stop, and disable reloads
                Stop::Automatic
            } else {
                let next_op = &operations[i + 1];
                if core::mem::discriminant(op)
                    == core::mem::discriminant(next_op)
                {
                    // If the next operation is the same as this one, don't send a repeat start
                    // condition
                    Stop::Reload
                } else {
                    // If the next operation is the same, condigure the peripheral to send a repeat
                    // start.
                    Stop::RepeatStart
                }
            };

            if i == 0 {
                self.start(
                    address,
                    address_mode,
                    op.length(),
                    op.direction(),
                    stop,
                );
            } else if reload {
                self.reload(op.length(), op.direction(), stop)
            } else {
                self.repeat_start(
                    address,
                    address_mode,
                    op.length(),
                    op.direction(),
                    stop,
                );
            }
            reload = stop == Stop::Reload;

            let result = self.process_operation(stop, &mut operations[i]);

            if let Err(error) = result {
                nb::block!(self.is_stopped_nb())?;
                return Err(error);
            }
        }

        Ok(())
    }
}

impl<I2C: Instance> i2c::I2c<i2c::SevenBitAddress> for I2c<I2C> {
    fn transaction(
        &mut self,
        address: i2c::SevenBitAddress,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        I2c::transaction(
            self,
            address as u16,
            AddressMode::AddressMode7bit,
            operations,
        )
    }
}

impl<I2C: Instance> i2c::I2c<i2c::TenBitAddress> for I2c<I2C> {
    fn transaction(
        &mut self,
        address: i2c::TenBitAddress,
        operations: &mut [i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        I2c::transaction(
            self,
            address,
            AddressMode::AddressMode10bit,
            operations,
        )
    }
}

//
// Created by andrew on 11/8/17.
//


#include "drivers/SPIBase.h"
#include "platform/mbed_wait_api.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_sleep.h"

#if DEVICE_SPI

namespace mbed {
    SPIBase::SPIBase(PinName mosi, PinName miso, PinName sclk, PinName ssel, Control ctrl, int frequency) :
    _spi(), _ctrl(ctrl), _hz(frequency), _bits(8), _mode(0)
    {
        for (size_t i = 0; i < sizeof _irq / sizeof _irq[0]; i++) {
            _irq[i] = NULL;
        }

        spi_init(&_spi, mosi, miso, sclk, ssel);
        spi_format(&_spi, _bits, _mode, _ctrl);
        spi_frequency(&_spi, _hz);
        spi_irq_handler(&_spi, SPIBase::_irq_handler, (uint32_t)this);
    }

    void SPIBase::format(int bits, int mode) {
        lock();
        _bits = bits;
        _mode = mode;
        spi_format(&_spi, _bits, _mode, _ctrl);
        unlock();
    }

    int SPIBase::readable() {
        lock();
        int ret = spi_readable(&_spi);
        unlock();
        return ret;
    }

    int SPIBase::writeable() {
        lock();
        int ret = spi_writable(&_spi);
        unlock();
        return ret;
    }

    void SPIBase::frequency(int hz) {
        lock();
        _hz = hz;
        spi_frequency(&_spi, _hz);
        unlock();
    }

    void SPIBase::attach(Callback<void()> func, IrqType type) {
        lock();
        // Disable interrupts when attaching interrupt handler
        core_util_critical_section_enter();
        if(func) {
            // lock deep sleep only the first time
            if(!_irq[type]) {
                sleep_manager_lock_deep_sleep();
            }
            _irq[type] = func;
            spi_irq_set(&_spi, (SpiIrq)type, 1);
        }
        else {
            // unlock dep sleep only the first time
            if(_irq[type]) {
                sleep_manager_unlock_deep_sleep();
            }
            _irq[type] = NULL;
            spi_irq_set(&_spi, (SpiIrq)type, 0);
        }
        core_util_critical_section_exit();
        unlock();
    }

    void SPIBase::set_default_write_value(char value) {
        _spi.fill = (uint8_t) value;
    }

    void SPIBase::_irq_handler(uint32_t id, SpiIrq irq_type) {
        SPIBase *handler = (SPIBase*)id;
        if (handler->_irq[irq_type]) {
            handler->_irq[irq_type]();
        }
    }

    int SPIBase::_base_transfer(int value, bool hold) {
        int inValue;
        lock();
        inValue = spi_transfer(&_spi, value, hold);
        unlock();
        return inValue;
    }

    int SPIBase::_base_transfer(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length) {
        lock();
        int total = (tx_length > rx_length) ? tx_length : rx_length;

        for(int i = 0; i < total; i++) {
            int out = (i < tx_length) ? tx_buffer[i] : get_fill();
            bool hold = (i < total-1);
            int in = spi_transfer(&_spi, out, hold);
            if(in < rx_length) {
                rx_buffer[i] = (char) in;
            }
        }

        unlock();
        return total;
    }

    void SPIBase::lock() {
        // do nothing
    }

    void SPIBase::unlock() {
        // do nothing
    }

    int SPIBase::_base_read(bool hold) {
        lock();
        int res = spi_read(&_spi);
//        int res = spi_transfer(&_spi, get_fill(), hold);
        unlock();
        return res;
    }

    void SPIBase::_base_write(int value, bool hold) {
        lock();
        if(!hold) {
            spi_write_fifo(&_spi, value);
        }
        else {
            spi_write(&_spi, value);
        }

        unlock();
    }

    uint8_t SPIBase::get_fill() {
        return _spi.fill;
    }

    void SPIBase::set_fill(uint8_t value) {
        _spi.fill = value;
    }

    bool SPIBase::active() {
        return spi_active(&_spi) != 0;
    }

    uint32_t SPIBase::transfer_count() {
        return spi_transfer_count(&_spi);
    }

    void SPIBase::flush() {
        spi_flush(&_spi);
    }
}

#endif
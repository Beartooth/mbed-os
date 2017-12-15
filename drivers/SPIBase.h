//
// Created by andrew on 11/8/17.
//

#ifndef MBED_SPIBASE_H
#define MBED_SPIBASE_H

#include "platform/platform.h"

#if defined (DEVICE_SPI) || defined(DOXYGEN_ONLY)

#include "Callback.h"
#include "spi_api.h"
#include "mbed_toolchain.h"
#include "platform/NonCopyable.h"

#if DEVICE_SPI_ASYNCH
#include "CThunk.h"
#include "dma_api.h"
#endif

namespace mbed {
    class SPIBase : private NonCopyable<SPIBase> {
    public:

        enum IrqType {
            RxIrq,
            TxIrq,
//            TransferCompleteIrq,
            IrqCnt
        };

        enum Control {
            Master,
            Slave
        };

        void blocking(bool enable);


        /** Configure the data transmission format
         *
         *  @param bits Number of bits per SPI frame (4 - 16)
         *  @param mode Clock polarity and phase mode (0 - 3)
         *
         * @code
         * mode | POL PHA
         * -----+--------
         *   0  |  0   0
         *   1  |  0   1
         *   2  |  1   0
         *   3  |  1   1
         * @endcode
         */
        void format(int bits, int mode = 0);

        /** Set the spi bus clock frequency
         *
         *  @param hz SCLK frequency in hz (default = 1MHz)
         */
        void frequency(int hz = 1000000);

        /** Determine if there is a character available to read
         *
         *  @returns
         *    1 if there is a character available to read,
         *    0 otherwise
         */
        int readable();

        /** Determine if there is space available to write a character
         *
         *  @returns
         *    1 if there is space to write a character,
         *    0 otherwise
         */
        int writeable();

        /** Check if spi is active
         *
         * @return true if active otherwise false
         */
        bool active();

        void flush();


        uint32_t transfer_count();

         /** Attach a function to call whenever a spi interrupt is generated
         *
         *  @param func A pointer to a void function, or 0 to set as none
         *  @param type Which spi interrupt to attach the member function to (SPIBase::TransferCompleteIrq for each transfer)
         */
        void attach(Callback<void()> func, IrqType type=RxIrq);

        /** Set default write data
         * SPI requires the master to send some data during a read operation.
         * Different devices may require different default byte values.
         * For example: A SD Card requires default bytes to be 0xFF.
         *
         * @param value    Default character to be transmitted while read operation
         */
        void set_default_write_value(char value);

    protected:
        /** Acquire exclusive access to this SPI bus
         */
        virtual void lock(void);

        /** Release exclusive access to this SPI bus
         */
        virtual void unlock(void);
    public:


        static void _irq_handler(uint32_t id, SpiIrq irq_type);

    protected:
        SPIBase(PinName mosi, PinName miso, PinName sclk, PinName ssel, Control ctrl, int frequency = 1000000);
        virtual ~SPIBase() {}

        /** Write to the SPI and return the response
         *
         *  @param value Data to be sent over SPI
         *
         *  @returns
         *    Response from the SPI slave
         */
        virtual int _base_transfer(int value, bool hold = false);

        /** Write to the SPI and obtain the response
         *
         *  The total number of bytes sent and recieved will be the maximum of
         *  tx_length and rx_length. The bytes written will be padded with the
         *  value 0xff.
         *
         *  @param tx_buffer Pointer to the byte-array of data to write to the device
         *  @param tx_length Number of bytes to write, may be zero
         *  @param rx_buffer Pointer to the byte-array of data to read from the device
         *  @param rx_length Number of bytes to read, may be zero
         *  @returns
         *      The number of bytes written and read from the device. This is
         *      maximum of tx_length and rx_length.
         */
        virtual int _base_transfer(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length);

        virtual int _base_read(bool hold = false);

        virtual void _base_write(int value, bool hold = false);

        uint8_t get_fill();

        void set_fill(uint8_t value);
        spi_t _spi;
        Callback<void()> _irq[IrqCnt];
        Control _ctrl;
        int _hz;
        int _mode;
        int _bits;
    };
} // mbed namespace

#endif // DEVICE_SPI

#endif //MBED_SPIBASE_H

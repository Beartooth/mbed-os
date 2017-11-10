
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_SPI_API_H
#define MBED_SPI_API_H

#include "device.h"
#include "hal/dma_api.h"
#include "hal/buffer.h"

#if DEVICE_SPI

#define SPI_EVENT_ERROR       (1 << 1)
#define SPI_EVENT_COMPLETE    (1 << 2)
#define SPI_EVENT_RX_OVERFLOW (1 << 3)
#define SPI_EVENT_ALL         (SPI_EVENT_ERROR | SPI_EVENT_COMPLETE | SPI_EVENT_RX_OVERFLOW)

#define SPI_EVENT_INTERNAL_TRANSFER_COMPLETE (1 << 30) // Internal flag to report that an event occurred

#define SPI_FILL_WORD         (0xFFFF)
#define SPI_FILL_CHAR         (0xFF)

typedef enum {
    RxDrainIrq,
    TxFillIrq,
} SpiIrq;

typedef void (*spi_isr)(uint32_t id, SpiIrq event);

#if DEVICE_SPI_ASYNCH
/** Asynch SPI HAL structure
 */
typedef struct {
    struct spi_s spi;        /**< Target specific SPI structure */
    struct buffer_s tx_buff; /**< Tx buffer */
    struct buffer_s rx_buff; /**< Rx buffer */
} spi_t;

#else
/** Non-asynch SPI HAL structure
 */
typedef struct spi_s spi_t;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup hal_GeneralSPI SPI Configuration Functions
 * @{
 */

/** Initialize the SPI peripheral
 *
 * Configures the pins used by SPI, sets a default format and frequency, and enables the peripheral
 * @param[out] obj  The SPI object to initialize
 * @param[in]  mosi The pin to use for MOSI
 * @param[in]  miso The pin to use for MISO
 * @param[in]  sclk The pin to use for SCLK
 * @param[in]  ssel The pin to use for SSEL
 */
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel);

/** Release a SPI object
 *
 * TODO: spi_free is currently unimplemented
 * This will require reference counting at the C++ level to be safe
 *
 * Return the pins owned by the SPI object to their reset state
 * Disable the SPI peripheral
 * Disable the SPI clock
 * @param[in] obj The SPI object to deinitialize
 */
void spi_free(spi_t *obj);

/** Configure the SPI format
 *
 * Set the number of bits per frame, configure clock polarity and phase, shift order and master/slave mode.
 * The default bit order is MSB.
 * @param[in,out] obj   The SPI object to configure
 * @param[in]     bits  The number of bits per frame
 * @param[in]     mode  The SPI mode (clock polarity, phase, and shift direction)
 * @param[in]     slave Zero for master mode or non-zero for slave mode
 */
void spi_format(spi_t *obj, int bits, int mode, int slave);

/** Set the SPI baud rate
 *
 * Actual frequency may differ from the desired frequency due to available dividers and bus clock
 * Configures the SPI peripheral's baud rate
 * @param[in,out] obj The SPI object to configure
 * @param[in]     hz  The baud rate in Hz
 */
void spi_frequency(spi_t *obj, int hz);

/** Check if a value is available to read
 *
 * @param[in] obj   The SPI peripheral to check
 * @return non-zero if a value is available
 */
int spi_readable(spi_t *obj);

/** Check if a space available to write a value
 *
 * @param[in] obj   The SPI peripheral to check
 * @return non-zero if there is space available to write 1 value
 */
int spi_writable(spi_t *obj);

/** Read data from fifo
 *
 * @param obj       The SPI peripheral
 * @return value read from rx fifo
 */
int spi_read_fifo(spi_t *obj);

/** Write data to fifo
 * @param obj   The SPI peripheral
 * @param value The value to write
 */
void spi_write_fifo(spi_t *obj, int value);

/**@}*/
/**
 * \defgroup SynchSPI Synchronous SPI Hardware Abstraction Layer
 * @{
 */

/** Write a single value over spi
 *
 * @param[in] obj   The SPI peripheral to use for sending
 * @param[in] value The value to send
 */
void spi_write(spi_t *obj, int value);


/** Read a single value out over spi
 *
 * @param[in] obj   The SPI peripheral to use for reading
 * @return value read from SPI
 */
int spi_read(spi_t *obj);


/** Write a block of data out over spi
 *
 *  The total number of bytes sent and recieved will be the maximum of
 *  tx_length and rx_length. The bytes written will be padded with the
 *  value 0xff.
 *
 * @param[in] obj        The SPI peripheral to use for sending
 * @param[in] buffer  Pointer to the byte-array of data to write to the device
 * @param[in] length  Number of bytes to write
 * @returns
 *      The number of bytes written and read from the device. This is
 *      maximum of tx_length and rx_length.
 */
int spi_block_write(spi_t *obj, const char *buffer, int length);

/** Read a block out in master mode
 *
 * @param[in] obj        The SPI peripheral to use for reading
 * @param[in] buffer     Pointer to the byte-array of data to read into
 * @param[in] length     Number of bytes to read
 * @param[in] write_fill Default data transmitted while performing a read
 * @return Number of bytes read
 */
int spi_block_read(spi_t *obj, char *buffer, int length);

/** Transfer multiple values over spi
 *
 *  The total number of bytes sent and recieved will be the maximum of
 *  tx_length and rx_length. The bytes written will be padded with the
 *  value 0xff.
 *
 * @param[in] obj        The SPI peripheral to use for sending
 * @param[in] tx_buffer  Pointer to the byte-array of data to write to the device
 * @param[in] tx_length  Number of bytes to write, may be zero
 * @param[in] rx_buffer  Pointer to the byte-array of data to read from the device
 * @param[in] rx_length  Number of bytes to read, may be zero
 * @param[in] write_fill Default data transmitted while performing a read
 * @returns
 *      The number of bytes written and read from the device. This is
 *      maximum of tx_length and rx_length.
 */
int spi_transfer(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length);

/** Checks if the specified SPI peripheral is in use
 *
 * @param[in] obj The SPI peripheral to check
 * @return non-zero if the peripheral is currently transmitting
 */
int spi_busy(spi_t *obj);

/** Get the module number
 *
 * @param[in] obj The SPI peripheral to check
 * @return The module number
 */
uint8_t spi_get_module(spi_t *obj);

/** Configure an spi interrupt
 *
 * @param obj       The SPI peripheral
 * @param irq       The IRQ type ()
 * @param enable    Set to non-zero to enable events, or zero to disable them
 */
void spi_irq_set(spi_t *obj, SpiIrq irq, int enable);

/** Get config for spi interrupt
 *
 * @param obj       The SPI peripheral
 * @param irq       The IRQ type
 * @return non-zero if enabled, or zero if disabled
 */
int spi_irq_get(spi_t *obj, SpiIrq irq);

/** The spi interrupt handler registration
 *
 * @param obj       The spi peripheral
 * @param handler   The interrupt handler which will be invoked when the interrupt fires
 * @param id        The SPIBase object
 */
void spi_irq_handler(spi_t *obj, spi_isr handler, uint32_t id);



/**@}*/

#if DEVICE_SPI_ASYNCH
/**
 * \defgroup AsynchSPI Asynchronous SPI Hardware Abstraction Layer
 * @{
 */

/** Begin the SPI transfer. Buffer pointers and lengths are specified in tx_buff and rx_buff
 *
 * @param[in] obj       The SPI object that holds the transfer information
 * @param[in] tx        The transmit buffer
 * @param[in] tx_length The number of bytes to transmit
 * @param[in] rx        The receive buffer
 * @param[in] rx_length The number of bytes to receive
 * @param[in] bit_width The bit width of buffer words
 * @param[in] event     The logical OR of events to be registered
 * @param[in] handler   SPI interrupt handler
 * @param[in] hint      A suggestion for how to use DMA with this transfer
 */
void spi_master_transfer(spi_t *obj, const void *tx, size_t tx_length, void *rx, size_t rx_length, uint8_t bit_width, uint32_t handler, uint32_t event, DMAUsage hint);

/** The asynchronous IRQ handler
 *
 * Reads the received values out of the RX FIFO, writes values into the TX FIFO and checks for transfer termination
 * conditions, such as buffer overflows or transfer complete.
 * @param[in] obj     The SPI object that holds the transfer information
 * @return Event flags if a transfer termination condition was met; otherwise 0.
 */
uint32_t spi_irq_handler_asynch(spi_t *obj);

/** Attempts to determine if the SPI peripheral is already in use
 *
 * If a temporary DMA channel has been allocated, peripheral is in use.
 * If a permanent DMA channel has been allocated, check if the DMA channel is in use.  If not, proceed as though no DMA
 * channel were allocated.
 * If no DMA channel is allocated, check whether tx and rx buffers have been assigned.  For each assigned buffer, check
 * if the corresponding buffer position is less than the buffer length.  If buffers do not indicate activity, check if
 * there are any bytes in the FIFOs.
 * @param[in] obj The SPI object to check for activity
 * @return Non-zero if the SPI port is active or zero if it is not.
 */
uint8_t spi_active(spi_t *obj);

/** Abort an SPI transfer
 *
 * @param obj The SPI peripheral to stop
 */
void spi_abort_asynch(spi_t *obj);


#endif

/**@}*/

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // SPI_DEVICE

#endif // MBED_SPI_API_H

/** @}*/

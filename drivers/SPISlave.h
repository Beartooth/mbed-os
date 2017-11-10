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
#ifndef MBED_SPISLAVE_H
#define MBED_SPISLAVE_H

#include "platform/platform.h"
#include "platform/NonCopyable.h"

#if defined (DEVICE_SPISLAVE) || defined(DOXYGEN_ONLY)

#include "drivers/SPIBase.h"

namespace mbed {
/** \addtogroup drivers */

/** A SPI slave, used for communicating with a SPI Master device
 *
 * The default format is set to 8-bits, mode 0, and a clock frequency of 1MHz
 *
 * @note Synchronization level: Not protected
 *
 * Example:
 * @code
 * // Reply to a SPI master as slave
 *
 * #include "mbed.h"
 *
 * SPISlave device(p5, p6, p7, p8); // mosi, miso, sclk, ssel
 *
 * int main() {
 *     device.reply(0x00);              // Prime SPI with first reply
 *     while(1) {
 *         if(device.receive()) {
 *             int v = device.read();   // Read byte from master
 *             v = (v + 1) % 0x100;     // Add one to it, modulo 256
 *             device.reply(v);         // Make this the next reply
 *         }
 *     }
 * }
 * @endcode
 * @ingroup drivers
 */
class SPISlave : public SPIBase {

public:

    /** Create a SPI slave connected to the specified pins
     *
     *  mosi or miso can be specfied as NC if not used
     *
     *  @param mosi SPI Master Out, Slave In pin
     *  @param miso SPI Master In, Slave Out pin
     *  @param sclk SPI Clock pin
     *  @param ssel SPI chip select pin
     */
    SPISlave(PinName mosi, PinName miso, PinName sclk, PinName ssel);


    /** Polls the SPI to see if data has been received
     *
     *  @returns
     *    0 if no data,
     *    1 otherwise
     */
    int receive(void);

    /** Retrieve  data from receive buffer as slave
     *
     *  @returns
     *    the data in the receive buffer
     */
    int read(void);

    /** Write data to spi
     *
     * @param value     Data to write to spi
     */
    void write(int value);

    /** Fill the transmission buffer with the value to be written out
     *  as slave on the next received message from the master.
     *
     *  @param value the data to be transmitted next
     *  @return read value received duriing transfer, ignore if not blocking
     */
    virtual int transfer(int value);

    /** Write to the SPI Slave and obtain the response
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
    virtual int transfer(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length);

};

} // namespace mbed

#endif

#endif

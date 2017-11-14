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
#include "drivers/SPISlave.h"

#if DEVICE_SPISLAVE

namespace mbed {

    SPISlave::SPISlave(PinName mosi, PinName miso, PinName sclk, PinName ssel) : SPIBase(mosi, miso, sclk, ssel,
                                                                                         Slave) {

    }

    int SPISlave::receive(void) {
        return (spi_readable(&_spi));
    }

    int SPISlave::read(void) {
        return _base_read();
    }

    void SPISlave::write(int value) {
        _base_write(value);
    }

    int SPISlave::transfer(int value) {
        return _base_transfer(value);
    }

    int SPISlave::transfer(const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length) {
        return _base_transfer(tx_buffer, tx_length, rx_buffer, rx_length);
    }

    unsigned char SPISlave::fill() {
        return get_fill();
    }

    void SPISlave::fill(unsigned char value) {
        set_fill(value);
    }

} // namespace mbed

#endif

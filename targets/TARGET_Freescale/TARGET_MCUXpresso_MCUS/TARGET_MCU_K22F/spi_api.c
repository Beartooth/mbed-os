/* mbed Microcontroller Library
 * Copyright (c) 2013 ARM Limited
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
#include <math.h>
#include <TARGET_MCUXpresso_MCUS/TARGET_MCU_K22F/TARGET_MCU_K22F512/device/cmsis.h>
#include "mbed_assert.h"

#include "spi_api.h"

#if DEVICE_SPI

#include "cmsis.h"
#include "pinmap.h"
#include "mbed_error.h"
#include "fsl_dspi.h"
#include "peripheral_clock_defines.h"
#include "PeripheralPins.h"


/* Array of SPI peripheral base address. */
static SPI_Type *const spi_address[] = SPI_BASE_PTRS;
/* Array of SPI bus clock frequencies */
static clock_name_t const spi_clocks[] = SPI_CLOCK_FREQS;
/* Irq handler */
static spi_isr irq_handler;
/* nvic irq names */
static IRQn_Type spi_irqs[] = SPI_IRQS;

/* unique id's for irq handlers */
static uint32_t spi_irq_ids[FSL_FEATURE_SOC_DSPI_COUNT] = {0};


void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel) {
    // determine the SPI to use
    uint32_t spi_mosi = pinmap_peripheral(mosi, PinMap_SPI_MOSI);
    uint32_t spi_miso = pinmap_peripheral(miso, PinMap_SPI_MISO);
    uint32_t spi_sclk = pinmap_peripheral(sclk, PinMap_SPI_SCLK);
    uint32_t spi_ssel = pinmap_peripheral(ssel, PinMap_SPI_SSEL);
    uint32_t spi_data = pinmap_merge(spi_mosi, spi_miso);
    uint32_t spi_cntl = pinmap_merge(spi_sclk, spi_ssel);

    obj->instance = pinmap_merge(spi_data, spi_cntl);
    MBED_ASSERT((int) obj->instance != NC);

    // pin out the spi pins
    pinmap_pinout(mosi, PinMap_SPI_MOSI);
    pinmap_pinout(miso, PinMap_SPI_MISO);
    pinmap_pinout(sclk, PinMap_SPI_SCLK);
    if (ssel != NC) {
        pinmap_pinout(ssel, PinMap_SPI_SSEL);
    }
    obj->fill = SPI_FILL_CHAR;
}

void spi_free(spi_t *obj) {
    DSPI_Deinit(spi_address[obj->instance]);
}

void spi_format(spi_t *obj, int bits, int mode, int slave) {
    dspi_master_config_t master_config;
    dspi_slave_config_t slave_config;

    if (slave) {
        /* Slave config */
        DSPI_SlaveGetDefaultConfig(&slave_config);
        slave_config.whichCtar = kDSPI_Ctar0;
        slave_config.ctarConfig.bitsPerFrame = (uint32_t) bits;;
        slave_config.ctarConfig.cpol = (mode & 0x2) ? kDSPI_ClockPolarityActiveLow : kDSPI_ClockPolarityActiveHigh;
        slave_config.ctarConfig.cpha = (mode & 0x1) ? kDSPI_ClockPhaseSecondEdge : kDSPI_ClockPhaseFirstEdge;

        DSPI_SlaveInit(spi_address[obj->instance], &slave_config);
    } else {
        /* Master config */
        DSPI_MasterGetDefaultConfig(&master_config);
        master_config.ctarConfig.bitsPerFrame = (uint32_t) bits;;
        master_config.ctarConfig.cpol = (mode & 0x2) ? kDSPI_ClockPolarityActiveLow : kDSPI_ClockPolarityActiveHigh;
        master_config.ctarConfig.cpha = (mode & 0x1) ? kDSPI_ClockPhaseSecondEdge : kDSPI_ClockPhaseFirstEdge;
        master_config.ctarConfig.direction = kDSPI_MsbFirst;
        master_config.ctarConfig.pcsToSckDelayInNanoSec = 0;

        DSPI_MasterInit(spi_address[obj->instance], &master_config, CLOCK_GetFreq(spi_clocks[obj->instance]));
    }
}

void spi_frequency(spi_t *obj, int hz) {
    uint32_t busClock = CLOCK_GetFreq(spi_clocks[obj->instance]);
    DSPI_MasterSetBaudRate(spi_address[obj->instance], kDSPI_Ctar0, (uint32_t) hz, busClock);
    //Half clock period delay after SPI transfer
    DSPI_MasterSetDelayTimes(spi_address[obj->instance], kDSPI_Ctar0, kDSPI_LastSckToPcs, busClock, 500000000 / hz);
}

int spi_readable(spi_t *obj) {
    return (int) (DSPI_GetStatusFlags(spi_address[obj->instance]) & kDSPI_RxFifoDrainRequestFlag);
}

int spi_writable(spi_t *obj) {
    return (int) (DSPI_GetStatusFlags(spi_address[obj->instance]) & kDSPI_TxFifoFillRequestFlag);
}

int spi_read(spi_t *obj) {
    int value = (int) DSPI_ReadData(spi_address[obj->instance]);
    DSPI_ClearStatusFlags(spi_address[obj->instance], kDSPI_RxFifoDrainRequestFlag);
    return value;
}

void spi_write(spi_t *obj, int value) {
    if (DSPI_IsMaster(spi_address[obj->instance])) {
        dspi_command_data_config_t command;
        DSPI_GetDefaultDataCommandConfig(&command);
        DSPI_MasterWriteData(spi_address[obj->instance], &command, (uint16_t) value);
    } else {
        DSPI_SlaveWriteData(spi_address[obj->instance], (uint32_t) value);
    }
    DSPI_ClearStatusFlags(spi_address[obj->instance], kDSPI_TxFifoFillRequestFlag);
}

//void spi_write(spi_t *obj, int value) {
//    spi_write_fifo(obj, value);
//}
//
//int spi_read(spi_t *obj) {
//    if (spi_readable(obj)) {
//        return spi_read_fifo(obj);
//    } else {
//        return spi_transfer(obj, obj->fill);
//    }
//}

void spi_start_transfer(spi_t *obj) {
    if(spi_irq_ids[obj->instance] != 0) {
        spi_irq_set(obj, RxDrainIrq, 1);
        spi_irq_set(obj, TxFillIrq, 1);
    }

    DSPI_StartTransfer(spi_address[obj->instance]);
}

void spi_stop_transfer(spi_t *obj) {
    if(spi_irq_ids[obj->instance] == 0) {
        spi_irq_set(obj, RxDrainIrq, 0);
        spi_irq_set(obj, TxFillIrq, 0);
    }
    else {

    }
}

int spi_transfer(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length) {
    dspi_command_data_config_t command;
    uint32_t rx_data;
    int total = (tx_length > rx_length) ? tx_length : rx_length;
    DSPI_GetDefaultDataCommandConfig(&command);
    command.isEndOfQueue = false;
    command.isPcsContinuous = true;

    if ((tx_buffer == NULL && tx_length > 0) || (rx_buffer == NULL && rx_length > 0)) {
        return 0;
    }

    DSPI_StopTransfer(spi_address[obj->instance]);
    DSPI_ClearStatusFlags(spi_address[obj->instance], kDSPI_EndOfQueueFlag);
    DSPI_StartTransfer(spi_address[obj->instance]);


    for (int i = 0; i < total; i++) {
        char out = (i < tx_length) ? tx_buffer[i] : obj->fill;
        if (i == total - 1) {
            command.isEndOfQueue = true;
            command.isPcsContinuous = false;
        }

        while (!spi_writable(obj));
        if (DSPI_IsMaster(spi_address[obj->instance])) {
            DSPI_MasterWriteData(spi_address[obj->instance], &command, (uint16_t) out);
        } else {
            DSPI_SlaveWriteData(spi_address[obj->instance], (uint32_t) out);
        }

        if (i < rx_length) {
            while (!spi_readable(obj));
            rx_buffer[i] = (char) spi_read(obj);
        }
    }

    DSPI_ClearStatusFlags(spi_address[obj->instance], kDSPI_EndOfQueueFlag);

    DSPI_StopTransfer(spi_address[obj->instance]);

    return total;
}

void spi_irq(uint32_t tx_fill, uint32_t rx_drain, uint32_t index) {
    SPI_Type *base = spi_address[index];

    if (spi_irq_ids[index] != 0) {
        if (tx_fill) {
            irq_handler(spi_irq_ids[index], TxFillIrq);
        }
        if (rx_drain) {
            irq_handler(spi_irq_ids[index], RxDrainIrq);
        }
    }
}

void spi0_irq() {
    uint32_t status_flags = DSPI_GetStatusFlags(spi_address[0]);
    spi_irq(status_flags & kDSPI_TxFifoFillRequestFlag, status_flags & kDSPI_RxFifoDrainRequestFlag, 0);

}

void spi1_irq() {
    uint32_t status_flags = DSPI_GetStatusFlags(spi_address[1]);
    spi_irq(status_flags & kDSPI_TxFifoFillRequestFlag, status_flags & kDSPI_RxFifoDrainRequestFlag, 1);
}


void spi_irq_set(spi_t *obj, SpiIrq irq, int enable) {
    uint32_t vector = 0;

    switch (obj->instance) {
        case 0:
            vector = (uint32_t) &spi0_irq;
            break;
        case 1:
            vector = (uint32_t) &spi1_irq;
            break;
        default:
            break;
    }

    if (enable) {
        switch (irq) {
            case RxDrainIrq:
                DSPI_ClearStatusFlags(spi_address[obj->instance], kDSPI_RxFifoDrainRequestFlag);
                DSPI_EnableInterrupts(spi_address[obj->instance], kDSPI_RxFifoDrainRequestInterruptEnable);
                break;
            case TxFillIrq:
                DSPI_ClearStatusFlags(spi_address[obj->instance], kDSPI_TxFifoFillRequestFlag);
                DSPI_EnableInterrupts(spi_address[obj->instance], kDSPI_TxFifoFillRequestInterruptEnable);
                break;
            default:
                break;
        }
        NVIC_SetVector(spi_irqs[obj->instance], vector);
        NVIC_EnableIRQ(spi_irqs[obj->instance]);
    } else { // disable
        switch (irq) {
            case RxDrainIrq:
                DSPI_DisableInterrupts(spi_address[obj->instance], kDSPI_RxFifoDrainRequestInterruptEnable);
                break;
            case TxFillIrq:
                DSPI_DisableInterrupts(spi_address[obj->instance], kDSPI_TxFifoFillRequestInterruptEnable);
                break;
//            case TransferCompleteIrq:
//                DSPI_DisableInterrupts(spi_address[obj->instance], kDSPI_TxCompleteInterruptEnable);
//                all_disabled = true;
//                break;
            default:
                break;
        }
        bool all_disabled = !(DSPI_GetEnabledInterrupts(spi_address[obj->instance]) &
                              (kDSPI_RxFifoDrainRequestInterruptEnable | kDSPI_TxFifoFillRequestInterruptEnable));
        if (all_disabled) {
            NVIC_DisableIRQ(spi_irqs[obj->instance]);
        }
    }
}

int spi_irq_get(spi_t *obj, SpiIrq irq) {
    uint32_t enabledInterrupts = DSPI_GetEnabledInterrupts(spi_address[obj->instance]);
    switch (irq) {
        case RxDrainIrq:
            return enabledInterrupts & kDSPI_RxFifoDrainRequestInterruptEnable;
        case TxFillIrq:
            return enabledInterrupts & kDSPI_TxFifoFillRequestInterruptEnable;
        default:
            return 0;
    }
}

void spi_irq_handler(spi_t *obj, spi_isr handler, uint32_t id) {
    DSPI_SetFifoEnable(spi_address[obj->instance], true, true);
    int len = FSL_FEATURE_DSPI_FIFO_SIZEn(spi_address[obj->instance]);
    irq_handler = handler;
    spi_irq_ids[obj->instance] = id;
}

#endif

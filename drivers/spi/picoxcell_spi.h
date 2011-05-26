/*!
* \file pc3xx_spi.h
* \brief Definitions for the PC302 SPI Block.
*
* Copyright (c) 2006-2011 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#ifndef PC302_SPI_H
#define PC302_SPI_H

#define SSI_CTRL_REG_0_REG_OFFSET                   0x00
#define SSI_CTRL_REG_1_REG_OFFSET                   0x04
#define SSI_ENABLE_REG_REG_OFFSET                   0x08
#define SSI_MW_CTRL_REG_OFFSET                      0x0C
#define SSI_SLAVE_ENABLE_REG_OFFSET                 0x10
#define SSI_BAUD_RATE_SEL_REG_OFFSET                0x14
#define SSI_TX_FIFO_THRESHOLD_REG_OFFSET            0x18
#define SSI_RX_FIFO_THRESHOLD_REG_OFFSET            0x1C
#define SSI_TX_FIFO_LEVEL_REG_OFFSET                0x20
#define SSI_RX_FIFO_LEVEL_REG_OFFSET                0x24
#define SSI_STATUS_REG_OFFSET                       0x28
#define SSI_IMR_REG_OFFSET                          0x2C
#define SSI_ISR_REG_OFFSET                          0x30
#define SSI_RAW_ISR_REG_OFFSET                      0x34
#define SSI_TX_FIFO_OVRFLOW_INT_CLEAR_REG_OFFSET    0x38
#define SSI_RX_FIFO_OVRFLOW_INT_CLEAR_REG_OFFSET    0x3C
#define SSI_RX_FIFO_UNDFLOW_INT_CLEAR_REG_OFFSET    0x40
#define SSI_MM_INT_CLEAR_REG_OFFSET                 0x44
#define SSI_INT_CLEAR_REG_OFFSET                    0x48
#define SSI_DMA_CTRL_REG_OFFSET                     0x4C
#define SSI_DMA_TX_DATA_LEVEL_REG_OFFSET            0x50
#define SSI_DMA_RX_DATA_LEVEL_REG_OFFSET            0x54
#define SSI_DATA_REG_OFFSET                         0x60

/* Identification Registers */

#define SSI_ID_REG_OFFSET                           0x58
#define SSI_COMP_VERSION_REG_OFFSET                 0x5C

/*****************************************************************************/
/* Register Reset Values                                                     */
/*****************************************************************************/

/* Functional Registers */

#define SSI_CTRL_REG_0_REG_RESET                    0x0000
#define SSI_CTRL_REG_1_REG_RESET                    0x0000
#define SSI_ENABLE_REG_REG_RESET                    0x0000
#define SSI_MW_CTRL_REG_RESET                       0x0000
#define SSI_SLAVE_ENABLE_REG_RESET                  0x0000
#define SSI_BAUD_RATE_SEL_REG_RESET                 0x0000
#define SSI_TX_FIFO_THRESHOLD_REG_RESET             0x0000
#define SSI_RX_FIFO_THRESHOLD_REG_RESET             0x0000
#define SSI_TX_FIFO_LEVEL_REG_RESET                 0x0000
#define SSI_RX_FIFO_LEVEL_REG_RESET                 0x0000
#define SSI_STATUS_REG_RESET                        0x0000
#define SSI_IMR_REG_RESET                           0x003F
#define SSI_ISR_REG_RESET                           0x0000
#define SSI_RAW_ISR_REG_RESET                       0x0000
#define SSI_TX_FIFO_OVRFLOW_INT_CLEAR_REG_RESET     0x0000
#define SSI_RX_FIFO_OVRFLOW_INT_CLEAR_REG_RESET     0x0000
#define SSI_RX_FIFO_UNDFLOW_INT_CLEAR_REG_RESET     0x0000
#define SSI_MM_INT_CLEAR_REG_RESET                  0x0000
#define SSI_INT_CLEAR_REG_RESET                     0x0000
#define SSI_DMA_CTRL_REG_RESET                      0x0000
#define SSI_DMA_TX_DATA_LEVEL_REG_RESET             0x0000
#define SSI_DMA_RX_DATA_LEVEL_REG_RESET             0x0000
#define SSI_DATA_REG_RESET                          0x0000

/* Identification Registers */

/* ** Need to check on these values */

#define SSI_ID_REG_RESET                            0x44570007
#define SSI_COMP_VERSION_REG_RESET                  0x3331312A
#define PC302_MAX_NUMBER_SPI_CS     (4)
#define PC302_MAX_NUMBER_SPI_BUSSES (1)
#define PC302_MIN_SPI_CLK_DIVIDER   (2)
#define PC302_MAX_SPI_CLK_DIVIDER   (65534)

/* SSI_CTRL_REG_0_REG_OFFSET bites */
#define PC302_SPI_LOOPBACK_MODE     (1 << 11)
#define PC302_SPI_NORMAL_MODE       (0)
#define PC302_SPI_TMOD_TX_RX        (0x0)
#define PC302_SPI_TMOD_TX           (0x1 << 8)
#define PC302_SPI_TMOD_RX           (0x2 << 8)
#define PC302_SPI_TMOD_EEPROM_RX    (0x3 << 8)
#define PC302_SPI_SCPOL             (1 << 7)
#define PC302_SPI_SCPH              (1 << 6)
#define PC302_SPI_MOTO_FORMAT       (0x0)
#define PC302_SPI_DATA_FRM_8_BIT    (0x7)


/* SSI_ENABLE_REG_REG_OFFSET bits */
#define PC302_SPI_ENABLE            (1)
#define PC302_SPI_DISABLE           (0)

/* SSI_SLAVE_ENABLE_REG_OFFSET bits */
#define PC302_SPI_SLAVES_DISABLE    (0)

/* SSI_STATUS_REG_OFFSET bits */
#define PC302_SPI_STATUS_DCOL       (1 << 6)
#define PC302_SPI_STATUS_TXE        (1 << 5)
#define PC302_SPI_STATUS_RFF        (1 << 4)
#define PC302_SPI_STATUS_RFNE       (1 << 3)
#define PC302_SPI_STATUS_TFE        (1 << 2)
#define PC302_SPI_STATUS_TFNF       (1 << 1)
#define PC302_SPI_STATUS_BUSY       (1 << 0)

/* SSI_IMR_REG_RESET bits */
#define PC302_SPI_MASK_ALL_INTS     (0xFFFF)

#define AXI2CFG_DECODE_MUX_3_IDX           11
#define AXI2CFG_DECODE_MUX_2_IDX           10
#define AXI2CFG_DECODE_MUX_1_IDX            9
#define AXI2CFG_DECODE_MUX_0_IDX            8

#define AXI2CFG_DECODE_MUX_3                (1 << AXI2CFG_DECODE_MUX_3_IDX)
#define AXI2CFG_DECODE_MUX_2                (1 << AXI2CFG_DECODE_MUX_2_IDX)
#define AXI2CFG_DECODE_MUX_1                (1 << AXI2CFG_DECODE_MUX_1_IDX)
#define AXI2CFG_DECODE_MUX_0                (1 << AXI2CFG_DECODE_MUX_0_IDX)

#endif /* PC302_SPI_H */

#ifndef _L3G4_H_
#define _L3G4_H_

#define L3G4_MAKE_OUT16(l,h) (h << 8 | l)

// OUTPUT REGISTER MAPPING

#define L3G4_WHO_AM_I           0x0F
#define L3G4_CTRL_REG1          0x20
#define L3G4_CTRL_REG2          0x21
#define L3G4_CTRL_REG3          0x22
#define L3G4_CTRL_REG4          0x23
#define L3G4_CTRL_REG5          0x24
#define L3G4_REFERENCE          0x25
#define L3G4_OUT_TEMP           0x26
#define L3G4_STATUS_REG         0x27
#define L3G4_OUT_X_L            0x28
#define L3G4_OUT_X_H            0x29
#define L3G4_OUT_Y_L            0x2A
#define L3G4_OUT_Y_H            0x2B
#define L3G4_OUT_Z_L            0x2C
#define L3G4_OUT_Z_H            0x2D
#define L3G4_FIFO_CTRL_REG      0x2E
#define L3G4_FIFO_SRC_REG       0x2F
#define L3G4_INT1_CFG           0x30
#define L3G4_INT1_SRC           0x31
#define L3G4_INT1_THS_XH        0x32
#define L3G4_INT1_THS_XL        0x33
#define L3G4_INT1_THS_YH        0x34
#define L3G4_INT1_THS_YL        0x35
#define L3G4_INT1_THS_ZH        0x36
#define L3G4_INT1_THS_ZL        0x37
#define L3G4_INT1_DURATION      0x38

// L3G4_CTRL_REG1

// Data Rate
#define L3G4_ODR_100                         ((uint8_t)0x00)
#define L3G4_ODR_200                         ((uint8_t)0x40)
#define L3G4_ODR_400                         ((uint8_t)0x80)
#define L3G4_ODR_800                         ((uint8_t)0xC0)

// Cut-off frequency varies depending on ODR. See table 22 for reference.

//  BW(1:0)   ODR=100Hz ODR=200Hz ODR=400Hz ODR=800Hz
//  ---------------------------------------------------------
//    00        12.5       12.5     20        30
//    01        25         25       25        25
//    10        25         50       50        50
//    11        25         70       110       110
#define L3G4_BW_0            ((uint8_t)0x00)
#define L3G4_BW_1            ((uint8_t)0x10)
#define L3G4_BW_2            ((uint8_t)0x20)
#define L3G4_BW_3            ((uint8_t)0x30)

// Sets power down or normal or sleep mode
#define L3G4_PD_MOD                 ((uint8_t)0x00) //Default
#define L3G4_NOR_SLE_MOD            ((uint8_t)0x08)

//Enable axis data
#define L3G4_XEN                             ((uint8_t)0x01)
#define L3G4_YEN                             ((uint8_t)0x02)
#define L3G4_ZEN                             ((uint8_t)0x04)
#define L3G4_XYZEN                           ((uint8_t)0x07) //Default

// L3G4_CTRL_REG2

// High Pass Filter Mode
#define L3G4_HPM_NOR_RES                     ((uint8_t)0x00)
#define L3G4_HPM_REF_SIG                     ((uint8_t)0x10)
#define L3G4_HPM_NOR                         ((uint8_t)0x20)
#define L3G4_HPM_AUTO_RES                    ((uint8_t)0x30)

// Cut-off frequency varies depending on ODR. See table 27 for reference.
#define L3G4_HPCF_0                    ((uint8_t)0x00)
#define L3G4_HPCF_1                    ((uint8_t)0x01)
#define L3G4_HPCF_2                    ((uint8_t)0x02)
#define L3G4_HPCF_3                    ((uint8_t)0x03)
#define L3G4_HPCF_4                    ((uint8_t)0x04)
#define L3G4_HPCF_5                    ((uint8_t)0x05)
#define L3G4_HPCF_6                    ((uint8_t)0x06)
#define L3G4_HPCF_7                    ((uint8_t)0x07)
#define L3G4_HPCF_8                    ((uint8_t)0x08)
#define L3G4_HPCF_9                    ((uint8_t)0x09)

// L3G4_CTRL_REG3

// Interrupt enable/disable on INT1
#define L3G4_INT1_DIS             ((uint8_t)0x00) //Default
#define L3G4_INT1_ENA             ((uint8_t)0x80)

// Boot available on INT1
#define L3G4_BOOT_INT1_DIS       ((uint8_t)0x00) //Default
#define L3G4_BOOT_INT1_ENA       ((uint8_t)0x40)

// Interrupt active configuration on INT1
#define L3G4_INT1_ACT_H           ((uint8_t)0x00) //Default
#define L3G4_INT1_ACT_L           ((uint8_t)0x20)

// Push- Pull / Open drain on INT1
#define L3G4_INT1_PP              ((uint8_t)0x00) //Default
#define L3G4_INT1_OD              ((uint8_t)0x10)

// Date Ready on DRDY/INT2
#define L3G4_DRDY_INT2_DIS       ((uint8_t)0x00) //Default
#define L3G4_DRDY_INT2_ENA       ((uint8_t)0x08)

// FIFO Watermark interrupt on DRDY/INT2
#define L3G4_WTM_INT2_DIS       ((uint8_t)0x00) //Default
#define L3G4_WTM_INT2_ENA       ((uint8_t)0x04)

// FIFO Overrun interrupt on DRDY/INT2
#define L3G4_ORUN_INT2_DIS       ((uint8_t)0x00) //Default
#define L3G4_ORUN_INT2_ENA       ((uint8_t)0x02)

// FIFO Empty interrupt on DRDY/INT2
#define L3G4_EMP_INT2_DIS       ((uint8_t)0x00) //Default
#define L3G4_EMP_INT2_ENA       ((uint8_t)0x01)

// L3G4_CTRL_REG4

// Block Data Update. (0: continous update; 1: output registers not updated until MSB and LSB reading)
#define L3G4_BDU_CONTINOUS                   ((uint8_t)0x00) //Default
#define L3G4_BDU_WAIT_READ                   ((uint8_t)0x80)

// Big/Little Endian Data Selection
#define L3G4_BLE_LSB                      ((uint8_t)0x00) //Default
#define L3G4_BLE_MSB                      ((uint8_t)0x40)

// Full Scale selection
#define L3G4_FS_250                          ((uint8_t)0x00) //Default
#define L3G4_FS_500                          ((uint8_t)0x10)
#define L3G4_FS_2000                         ((uint8_t)0x20)

// Self Test Enable
#define L3G4_ST_NOR                        ((uint8_t)0x00) //Default
#define L3G4_ST_0                          ((uint8_t)0x02)
#define L3G4_ST_1                          ((uint8_t)0x06)

// SPI Serial Interface Mode selection
#define L3G4_SPI_4_WIRE                    ((uint8_t)0x00) //Default
#define L3G4_SPI_3_WIRE                    ((uint8_t)0x01)

// L3G4_CTRL_REG5

// Reboot memory content
#define L3G4_BOOT_NOR                     ((uint8_t)0x00) //Default
#define L3G4_BOOT_REBOOT                  ((uint8_t)0x80)

// FIFO enable
#define L3G4_FIFO_DIS                     ((uint8_t)0x00) //Default
#define L3G4_FIFO_ENA                     ((uint8_t)0x40)

// High Pass filter Enable (Hpen)
#define L3G4_HPF_DIS             ((uint8_t)0x00) //Default
#define L3G4_HPF_ENA             ((uint8_t)0x10)

// INT1 selection configuration
// Hpen  INT_SEL1  INT_SEL2   Description
// --------------------------------------
//  x       0         0       Non-high-pass-filtered data are used for interrupt generation
//  x       0         1       High-pass-filtered data are used for interrupt generation
//  0       1         x       Low-pass-filtered data are used for interrupt generation
//  1       1         x       High-pass and low-pass-filtered data are used for interrupt generation

#define L3G4_INT1_SEL_0         ((uint8_t)0x00) //Default
#define L3G4_INT1_SEL_1         ((uint8_t)0x04)
#define L3G4_INT1_SEL_2         ((uint8_t)0x08)
#define L3G4_INT1_SEL_3         ((uint8_t)0x0C)

// Out selection configuration
// Hpen  OUT_SEL1  OUT_SEL0   Description
// --------------------------------------
//  x       0         0       Data in DataReg and FIFO are non-highpass-filtered
//  x       0         1       Data in DataReg and FIFO are high-passfiltered
//  0       1         x       Data in DataReg and FIFO are low-passfiltered by LPF2
//  1       1         x       Data in DataReg and FIFO are high-pass and low-pass-filtered by LPF2

#define L3G4_OUT_SEL_0         ((uint8_t)0x00) //Default
#define L3G4_OUT_SEL_1         ((uint8_t)0x01)
#define L3G4_OUT_SEL_2         ((uint8_t)0x02)
#define L3G4_OUT_SEL_3         ((uint8_t)0x03)

// L3G4_STATUS_REG
#define L3G4_ZYXOR_BIT        ((uint8_t)(0x01 << 7))
#define L3G4_ZOR_BIT          ((uint8_t)(0x01 << 6))
#define L3G4_YOR_BIT          ((uint8_t)(0x01 << 5))
#define L3G4_XOR_BIT          ((uint8_t)(0x01 << 4))
#define L3G4_ZYXDA_BIT        ((uint8_t)(0x01 << 3))
#define L3G4_ZDA_BIT          ((uint8_t)(0x01 << 2))
#define L3G4_YDA_BIT          ((uint8_t)(0x01 << 1))
#define L3G4_XDA_BIT          ((uint8_t)0x01)

// L3G4_FIFO_CTRL_REG

// FIFO mode selection
#define L3G4_BYBASS_MOD       ((uint8_t)0x00) //Default
#define L3G4_FIFO_MOD         ((uint8_t)0x20)
#define L3G4_STREAM_MOD       ((uint8_t)0x40)
#define L3G4_ST_FI_MOD        ((uint8_t)0x60)
#define L3G4_BY_ST_MOD        ((uint8_t)0x80)

// FIFO threshold. Watermark level setting
// WTM4-WTM0

// L3G4_FIFO_SRC_REG

// Watermark status
#define L3G4_WTM_NOT_FULL         ((uint8_t)0x00) //Default
#define L3G4_WTM_FULL             ((uint8_t)0x80)

// Overrun bit status
#define L3G4_FIFO_NOT_FULL         ((uint8_t)0x00) //Default
#define L3G4_FIFO_FULL             ((uint8_t)0x60)

// FIFO empty bit
#define L3G4_FIFO_NOT_EMPTY         ((uint8_t)0x00) //Default
#define L3G4_FIFO_EMPTY             ((uint8_t)0x40)

// FIFO stored data level
// FSS4-FSS1

// L3G4_INT1_CFG
// L3G4_INT1_SRC
// L3G4_INT1_THS_XH
// L3G4_INT1_THS_XL
// L3G4_INT1_THS_YH
// L3G4_INT1_THS_YL
// L3G4_INT1_THS_ZH
// L3G4_INT1_THS_ZL
// L3G4INT1_DURATION

#define L3G4_Gyr_Coeff           0.0001527

#endif

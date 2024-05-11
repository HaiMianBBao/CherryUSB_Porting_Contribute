/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _USB_CH32_USBFS_REG_H
#define _USB_CH32_USBFS_REG_H

#define __IO volatile /* defines 'read / write' permissions */

/* USBOTG_FS Registers */
typedef struct
{
    __IO uint8_t BASE_CTRL; //0x40023400
    __IO uint8_t UDEV_CTRL; //0x40023401
    __IO uint8_t INT_EN; //0x40023402
    __IO uint8_t DEV_ADDR; //0x40023403
    __IO uint8_t __reserved0; //0x40023404
    __IO uint8_t MIS_ST; //0x40023405
    __IO uint8_t INT_FG; //0x40023406
    __IO uint8_t INT_ST; //0x40023407
    __IO uint16_t RX_LEN; //0x40023408
    __IO uint16_t __reserved1; //0x4002340A
    __IO uint8_t UEP4_1_MOD; //0x4002340C
    __IO uint8_t UEP2_3_MOD; //0x4002340D
    __IO uint8_t UEP5_6_MOD; //0x4002340E
    __IO uint8_t UEP7_MOD; //0x4002340F
    __IO uint32_t UEP0_DMA; //0x40023410
    __IO uint32_t UEP1_DMA; //0x40023414
    __IO uint32_t UEP2_DMA; //0x40023418
    __IO uint32_t UEP3_DMA; //0x4002341C
    __IO uint32_t UEP4_DMA; //0x40023420
    __IO uint32_t UEP5_DMA; //0x40023424
    __IO uint32_t UEP6_DMA; //0x40023428
    __IO uint32_t UEP7_DMA; //0x4002342C
    __IO uint16_t UEP0_TX_LEN; //0x40023430
    __IO uint8_t UEP0_CTRL; //0x40023432
    __IO uint8_t __reserved2;
    __IO uint16_t UEP1_TX_LEN; //0x40023434
    __IO uint8_t UEP1_CTRL; //0x40023436
    __IO uint8_t __reserved3;
    __IO uint16_t UEP2_TX_LEN; //0x40023438
    __IO uint8_t UEP2_CTRL; //0x4002343A
    __IO uint8_t __reserved4;
    __IO uint16_t UEP3_TX_LEN; //0x4002343C
    __IO uint8_t UEP3_CTRL; //0x4002343E
    __IO uint8_t __reserved5;
    __IO uint16_t UEP4_TX_LEN; //0x40023440
    __IO uint8_t UEP4_CTRL; //0x40023442
    __IO uint8_t __reserved6;
    __IO uint16_t UEP5_TX_LEN; //0x40023444
    __IO uint8_t UEP5_CTRL; //0x40023446
    __IO uint8_t __reserved7;
    __IO uint16_t UEP6_TX_LEN; //0x40023448
    __IO uint8_t UEP6_CTRL; //0x4002344a
    __IO uint8_t __reserved8;
    __IO uint16_t UEP7_TX_LEN; //0x4002344C
    __IO uint8_t UEP7_CTRL; //0x4002344E
    __IO uint8_t __reserved9;
} USB_FS_TypeDef;

#define USBFS_DEVICE ((USB_FS_TypeDef *)(uint32_t)0x40023400)

/******************* GLOBAL ******************/

/* BASE USB_CTRL */
#define USBFS_BASE_CTRL_OFFSET 0x00 // USB base control
#define USBFS_UC_HOST_MODE     0x80 // enable USB host mode: 0=device mode, 1=host mode
#define USBFS_UC_LOW_SPEED     0x40 // enable USB low speed: 0=12Mbps, 1=1.5Mbps
#define USBFS_UC_DEV_PU_EN     0x20 // USB device enable and internal pullup resistance enable
#define USBFS_UC_SYS_CTRL1     0x20 // USB system control high bit
#define USBFS_UC_SYS_CTRL0     0x10 // USB system control low bit
#define USBFS_UC_SYS_CTRL_MASK 0x30 // bit mask of USB system control
// UC_HOST_MODE & UC_SYS_CTRL1 & UC_SYS_CTRL0: USB system control
//   0 00: disable USB device and disable internal pullup resistance
//   0 01: enable USB device and disable internal pullup resistance, need external pullup resistance
//   0 1x: enable USB device and enable internal pullup resistance
//   1 00: enable USB host and normal status
//   1 01: enable USB host and force UDP/UDM output SE0 state
//   1 10: enable USB host and force UDP/UDM output J state
//   1 11: enable USB host and force UDP/UDM output resume or K state
#define USBFS_UC_INT_BUSY  0x08 // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
#define USBFS_UC_RESET_SIE 0x04 // force reset USB SIE, need software clear
#define USBFS_UC_CLR_ALL   0x02 // force clear FIFO and count of USB
#define USBFS_UC_DMA_EN    0x01 // DMA enable and DMA interrupt enable for USB

/* USB INT EN */
#define USBFS_INT_EN_OFFSET 0x02
#define USBFS_UIE_DEV_SOF   0x80 // enable interrupt for SOF received for USB device mode
#define USBFS_UIE_DEV_NAK   0x40 // enable interrupt for NAK responded for USB device mode
#define USBFS_UIE_FIFO_OV   0x10 // enable interrupt for FIFO overflow
#define USBFS_UIE_HST_SOF   0x08 // enable interrupt for host SOF timer action for USB host mode
#define USBFS_UIE_SUSPEND   0x04 // enable interrupt for USB suspend or resume event
#define USBFS_UIE_TRANSFER  0x02 // enable interrupt for USB transfer completion
#define USBFS_UIE_DETECT    0x01 // enable interrupt for USB device detected event for USB host mode
#define USBFS_UIE_BUS_RST   0x01 // enable interrupt for USB bus reset event for USB device mode
/* USB_DEV_ADDR */
#define USBFS_DEV_ADDR_OFFSET 0x03
#define USBFS_UDA_GP_BIT      0x80 // general purpose bit
#define USBFS_USB_ADDR_MASK   0x7F // bit mask for USB device address

/* USB_STATUS */
#define USBFS_USB_STATUS_OFFSET 0x04

/* USB_MIS_ST */
#define USBFS_MIS_ST_OFFSET  0x05
#define USBFS_UMS_SOF_PRES   0x80 // RO, indicate host SOF timer presage status
#define USBFS_UMS_SOF_ACT    0x40 // RO, indicate host SOF timer action status for USB host
#define USBFS_UMS_SIE_FREE   0x20 // RO, indicate USB SIE free status
#define USBFS_UMS_R_FIFO_RDY 0x10 // RO, indicate USB receiving FIFO ready status (not empty)
#define USBFS_UMS_BUS_RESET  0x08 // RO, indicate USB bus reset status
#define USBFS_UMS_SUSPEND    0x04 // RO, indicate USB suspend status
#define USBFS_UMS_DM_LEVEL   0x02 // RO, indicate UDM level saved at device attached to USB host
#define USBFS_UMS_DEV_ATTACH 0x01 // RO, indicate device attached status on USB host

/* USB_INT_FG */
#define USBFS_INT_FG_OFFSET 0x06
#define USBFS_U_IS_NAK      0x80 // RO, indicate current USB transfer is NAK received
#define USBFS_U_TOG_OK      0x40 // RO, indicate current USB transfer toggle is OK
#define USBFS_U_SIE_FREE    0x20 // RO, indicate USB SIE free status
#define USBFS_UIF_FIFO_OV   0x10 // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
#define USBFS_UIF_HST_SOF   0x08 // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
#define USBFS_UIF_SUSPEND   0x04 // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
#define USBFS_UIF_TRANSFER  0x02 // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
#define USBFS_UIF_DETECT    0x01 // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
#define USBFS_UIF_BUS_RST   0x01 // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear

/* USB_INT_ST */
#define USBFS_INT_ST_OFFSET   0x07
#define USBFS_UIS_IS_SETUP    0x80 // RO, indicate current USB transfer is setup received for USB device mode
#define USBFS_UIS_IS_NAK      0x80 // RO, indicate current USB transfer is NAK received for USB device mode
#define USBFS_UIS_TOG_OK      0x40 // RO, indicate current USB transfer toggle is OK
#define USBFS_UIS_TOKEN1      0x20 // RO, current token PID code bit 1 received for USB device mode
#define USBFS_UIS_TOKEN0      0x10 // RO, current token PID code bit 0 received for USB device mode
#define USBFS_UIS_TOKEN_MASK  0x30 // RO, bit mask of current token PID code received for USB device mode
#define USBFS_UIS_TOKEN_OUT   0x00
#define USBFS_UIS_TOKEN_SOF   0x10
#define USBFS_UIS_TOKEN_IN    0x20
#define USBFS_UIS_TOKEN_SETUP 0x30
// UIS_TOKEN1 & UIS_TOKEN0: current token PID code received for USB device mode
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: SETUP token PID received
#define USBFS_UIS_ENDP_MASK 0x0F // RO, bit mask of current transfer endpoint number for USB device mode
/* USB_RX_LEN */
#define USBFS_RX_LEN_OFFSET 0x08

/******************* DEVICE ******************/

/* UDEV_CTRL */
#define USBFS_UDEV_CTRL_OFFSET 0x01
#define USBFS_UD_PD_DIS        0x80 // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define USBFS_UD_DP_PIN        0x20 // ReadOnly: indicate current UDP pin level
#define USBFS_UD_DM_PIN        0x10 // ReadOnly: indicate current UDM pin level
#define USBFS_UD_LOW_SPEED     0x04 // enable USB physical port low speed: 0=full speed, 1=low speed
#define USBFS_UD_GP_BIT        0x02 // general purpose bit
#define USBFS_UD_PORT_EN       0x01 // enable USB physical port I/O: 0=disable, 1=enable

/* UEP4_1_MOD */
#define USBFS_UEP4_1_MOD_OFFSET 0x0C
#define USBFS_UEP1_RX_EN        0x80 // enable USB endpoint 1 receiving (OUT)
#define USBFS_UEP1_TX_EN        0x40 // enable USB endpoint 1 transmittal (IN)
#define USBFS_UEP1_BUF_MOD      0x10 // buffer mode of USB endpoint 1
// UEPn_RX_EN & UEPn_TX_EN & UEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
//   0 0 x:  disable endpoint and disable buffer
//   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
//   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
//   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
//   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
//   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
//   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
#define USBFS_UEP4_RX_EN 0x08 // enable USB endpoint 4 receiving (OUT)
#define USBFS_UEP4_TX_EN 0x04 // enable USB endpoint 4 transmittal (IN)
// UEP4_RX_EN & UEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
//   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
//   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
//   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes

/* UEP2_3_MOD */
#define USBFS_UEP2_3_MOD_OFFSET 0x0D
#define USBFS_UEP3_RX_EN        0x80 // enable USB endpoint 3 receiving (OUT)
#define USBFS_UEP3_TX_EN        0x40 // enable USB endpoint 3 transmittal (IN)
#define USBFS_UEP3_BUF_MOD      0x10 // buffer mode of USB endpoint 3
#define USBFS_UEP2_RX_EN        0x08 // enable USB endpoint 2 receiving (OUT)
#define USBFS_UEP2_TX_EN        0x04 // enable USB endpoint 2 transmittal (IN)
#define USBFS_UEP2_BUF_MOD      0x01 // buffer mode of USB endpoint 2

/* UEP5_6_MOD */
#define USBFS_UEP5_6_MOD_OFFSET 0x0E
#define USBFS_UEP6_RX_EN        0x80 // enable USB endpoint 6 receiving (OUT)
#define USBFS_UEP6_TX_EN        0x40 // enable USB endpoint 6 transmittal (IN)
#define USBFS_UEP6_BUF_MOD      0x10 // buffer mode of USB endpoint 6
#define USBFS_UEP5_RX_EN        0x08 // enable USB endpoint 5 receiving (OUT)
#define USBFS_UEP5_TX_EN        0x04 // enable USB endpoint 5 transmittal (IN)
#define USBFS_UEP5_BUF_MOD      0x01 // buffer mode of USB endpoint 5

/* UEP7_MOD */
#define USBFS_UEP7_MOD_OFFSET 0x0F
#define USBFS_UEP7_RX_EN      0x08 // enable USB endpoint 7 receiving (OUT)
#define USBFS_UEP7_TX_EN      0x04 // enable USB endpoint 7 transmittal (IN)
#define USBFS_UEP7_BUF_MOD    0x01 // buffer mode of USB endpoint 7

/* USB_DMA */
#define USBFS_UEPx_DMA_OFFSET(n) (0x10 + 4 * (n)) // endpoint x DMA buffer address
#define USBFS_UEP0_DMA_OFFSET    0x10 // endpoint 0 DMA buffer address
#define USBFS_UEP1_DMA_OFFSET    0x14 // endpoint 1 DMA buffer address
#define USBFS_UEP2_DMA_OFFSET    0x18 // endpoint 2 DMA buffer address
#define USBFS_UEP3_DMA_OFFSET    0x1c // endpoint 3 DMA buffer address
#define USBFS_UEP4_DMA_OFFSET    0x20 // endpoint 4 DMA buffer address
#define USBFS_UEP5_DMA_OFFSET    0x24 // endpoint 5 DMA buffer address
#define USBFS_UEP6_DMA_OFFSET    0x28 // endpoint 6 DMA buffer address
#define USBFS_UEP7_DMA_OFFSET    0x2c // endpoint 7 DMA buffer address
/* USB_EP_CTRL */
#define USBFS_UEPx_T_LEN_OFFSET(n)   (0x30 + 4 * (n)) // endpoint x DMA buffer address
#define USBFS_UEPx_TX_CTRL_OFFSET(n) (0x30 + 4 * (n) + 2) // endpoint x DMA buffer address
#define USBFS_UEPx_RX_CTRL_OFFSET(n) (0x30 + 4 * (n) + 3) // endpoint x DMA buffer address

#define USBFS_UEP_AUTO_TOG 0x10 // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define USBFS_UEP_R_TOG    0x80 // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define USBFS_UEP_T_TOG    0x40 // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1

#define USBFS_UEP_R_RES_MASK  0x0C // bit mask of handshake response type for USB endpoint X receiving (OUT)
#define USBFS_UEP_R_RES_ACK   0x00
#define USBFS_UEP_R_RES_TOUT  0x04
#define USBFS_UEP_R_RES_NAK   0x08
#define USBFS_UEP_R_RES_STALL 0x0C
// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
#define USBFS_UEP_T_RES_MASK  0x03 // bit mask of handshake response type for USB endpoint X transmittal (IN)
#define USBFS_UEP_T_RES_ACK   0x00
#define USBFS_UEP_T_RES_TOUT  0x01
#define USBFS_UEP_T_RES_NAK   0x02
#define USBFS_UEP_T_RES_STALL 0x03
// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: TALL (error)

#endif

// Hardware interface to "USB OTG (on the go) controller" on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2023  Drahflow <drahflow@gmx.de>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "usb.h"

#include "config.h"
#include "io.h"
#include "irq.h"
#include "gpio.h"
#include "stm32h723xx.h"
#include "cmsis_compiler.h"
#include "core_cm7.h"
#include "console.h"
#include "beep.h"

#include <string.h>
#include <stdbool.h>

#define USB_PERIPH_BASE USB_OTG_HS_PERIPH_BASE
#define OTG_IRQn OTG_HS_IRQn
#define USBOTGEN RCC_AHB1ENR_USB1OTGHSEN

#define OTG ((USB_OTG_GlobalTypeDef*)USB_PERIPH_BASE)
#define OTGD ((USB_OTG_DeviceTypeDef*)(USB_PERIPH_BASE + USB_OTG_DEVICE_BASE))
#define EPFIFO(EP) ((void*)(USB_PERIPH_BASE + USB_OTG_FIFO_BASE + ((EP) << 12)))
#define EPIN(EP) ((USB_OTG_INEndpointTypeDef*)                          \
                  (USB_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + ((EP) << 5)))
#define EPOUT(EP) ((USB_OTG_OUTEndpointTypeDef*)                        \
                   (USB_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((EP) << 5)))


#define USB_DIR_OUT                     0               /* to device */
#define USB_DIR_IN                      0x80            /* to host */

#define USB_TYPE_MASK                   (0x03 << 5)
#define USB_TYPE_STANDARD               (0x00 << 5)
#define USB_TYPE_CLASS                  (0x01 << 5)
#define USB_TYPE_VENDOR                 (0x02 << 5)
#define USB_TYPE_RESERVED               (0x03 << 5)

#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C

struct usb_ctrlrequest {
    uint8_t bRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} PACKED;

#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05
#define USB_DT_DEVICE_QUALIFIER         0x06
#define USB_DT_OTHER_SPEED_CONFIG       0x07
#define USB_DT_ENDPOINT_COMPANION       0x30

#define USB_CDC_SUBCLASS_ACM 0x02

#define USB_CDC_ACM_PROTO_AT_V25TER 1

struct usb_cdc_header_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint16_t bcdCDC;
} PACKED;

#define USB_CDC_HEADER_TYPE 0x00
#define USB_CDC_ACM_TYPE 0x02
#define USB_CDC_UNION_TYPE 0x06

#define USB_CDC_CS_INTERFACE 0x24
#define USB_CDC_CS_ENDPOINT 0x25

struct usb_cdc_acm_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bmCapabilities;
} PACKED;

struct usb_cdc_union_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bMasterInterface0;
    uint8_t bSlaveInterface0;
} PACKED;

#define USB_CDC_REQ_SET_LINE_CODING 0x20
#define USB_CDC_REQ_GET_LINE_CODING 0x21
#define USB_CDC_REQ_SET_CONTROL_LINE_STATE 0x22

struct usb_cdc_line_coding {
    uint32_t dwDTERate;
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
} PACKED;
struct usb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} PACKED;

#define USB_CLASS_PER_INTERFACE         0       /* for DeviceClass */
#define USB_CLASS_AUDIO                 1
#define USB_CLASS_COMM                  2
#define USB_CLASS_HID                   3
#define USB_CLASS_PHYSICAL              5
#define USB_CLASS_STILL_IMAGE           6
#define USB_CLASS_PRINTER               7
#define USB_CLASS_MASS_STORAGE          8
#define USB_CLASS_HUB                   9

struct usb_config_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} PACKED;

struct usb_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} PACKED;

struct usb_endpoint_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} PACKED;

#define USB_ENDPOINT_NUMBER_MASK        0x0f    /* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK           0x80

#define USB_ENDPOINT_XFERTYPE_MASK      0x03    /* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL       0
#define USB_ENDPOINT_XFER_ISOC          1
#define USB_ENDPOINT_XFER_BULK          2
#define USB_ENDPOINT_XFER_INT           3
#define USB_ENDPOINT_MAX_ADJUSTABLE     0x80

struct usb_string_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    //uint16_t data[];
    typeof(*u"") data[];
} PACKED;

#define USB_LANGID_ENGLISH_US 0x0409

enum {
    USB_CDC_EP0_SIZE = 16,
    USB_CDC_EP_ACM_SIZE = 8,
    USB_CDC_EP_BULK_OUT_SIZE = 64,
    USB_CDC_EP_BULK_IN_SIZE = 64,
};

enum {
    USB_CDC_EP_ACM = 3,
    USB_CDC_EP_BULK_OUT = 1,
    USB_CDC_EP_BULK_IN = 2,
};

// State tracking
enum {
    UX_READ = 1<<0, UX_SEND = 1<<1, UX_SEND_PROGMEM = 1<<2, UX_SEND_ZLP = 1<<3
};

static void
usb_irq_disable(void)
{
    NVIC_DisableIRQ(OTG_IRQn);
}

static void
usb_irq_enable(void)
{
    NVIC_EnableIRQ(OTG_IRQn);
}

bool todo_usb_ep0 = false;
bool todo_usb_bulk_in = false;
bool todo_usb_bulk_out = false;

void usb_notify_ep0() { todo_usb_ep0 = true; }
void usb_notify_bulk_in() { todo_usb_bulk_in = true; }
void usb_notify_bulk_out() { todo_usb_bulk_out = true; }

/****************************************************************
 * USB transfer memory
 ****************************************************************/

// Setup the USB fifos
static void
fifo_configure(void)
{
    // Reserve memory for Rx fifo
    uint32_t sz = ((4 * 1 + 6)
                   + 4 * ((USB_CDC_EP_BULK_OUT_SIZE / 4) + 1)
                   + (2 * 1));
    OTG->GRXFSIZ = sz;

    // Tx fifos
    uint32_t fpos = sz, ep_size = 0x10;
    OTG->DIEPTXF0_HNPTXFSIZ = ((fpos << USB_OTG_TX0FSA_Pos)
                               | (ep_size << USB_OTG_TX0FD_Pos));
    fpos += ep_size;

    OTG->DIEPTXF[USB_CDC_EP_ACM - 1] = (
        (fpos << USB_OTG_DIEPTXF_INEPTXSA_Pos)
        | (ep_size << USB_OTG_DIEPTXF_INEPTXFD_Pos));
    fpos += ep_size;

    OTG->DIEPTXF[USB_CDC_EP_BULK_IN - 1] = (
        (fpos << USB_OTG_DIEPTXF_INEPTXSA_Pos)
        | (ep_size << USB_OTG_DIEPTXF_INEPTXFD_Pos));
    fpos += ep_size;
}

// Write a packet to a tx fifo
static int_fast8_t
fifo_write_packet(uint32_t ep, const uint8_t *src, uint32_t len)
{
    void *fifo = EPFIFO(ep);
    USB_OTG_INEndpointTypeDef *epi = EPIN(ep);
    epi->DIEPINT = USB_OTG_DIEPINT_XFRC;
    epi->DIEPTSIZ = len | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos);
    epi->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    int32_t count = len;
    while (count >= 4) {
        uint32_t data;
        memcpy(&data, src, 4);
        writel(fifo, data);
        count -= 4;
        src += 4;
    }
    if (count) {
        uint32_t data = 0;
        memcpy(&data, src, count);
        writel(fifo, data);
    }
    return len;
}

// Read a packet from the rx queue
static int_fast8_t
fifo_read_packet(uint8_t *dest, uint_fast8_t max_len)
{
    // Transfer data
    void *fifo = EPFIFO(0);
    uint32_t grx = OTG->GRXSTSP;
    uint32_t bcnt = (grx & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
    uint32_t xfer = bcnt > max_len ? max_len : bcnt, count = xfer;
    while (count >= 4) {
        uint32_t data = readl(fifo);
        memcpy(dest, &data, 4);
        count -= 4;
        dest += 4;
    }
    if (count) {
        uint32_t data = readl(fifo);
        memcpy(dest, &data, count);
    }
    uint32_t extra = DIV_ROUND_UP(bcnt, 4) - DIV_ROUND_UP(xfer, 4);
    while (extra--)
        readl(fifo);
    return xfer;
}

// Reenable packet reception if it got disabled by controller
static void
enable_rx_endpoint(uint32_t ep)
{
    USB_OTG_OUTEndpointTypeDef *epo = EPOUT(ep);
    uint32_t ctl = epo->DOEPCTL;
    if (!(ctl & USB_OTG_DOEPCTL_EPENA) || ctl & USB_OTG_DOEPCTL_NAKSTS) {
        epo->DOEPTSIZ = 64 | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
        epo->DOEPCTL = ctl | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
    }
}

// Inspect the next packet on the rx queue
static uint32_t
peek_rx_queue(uint32_t ep)
{
    for (;;) {
        uint32_t sts = OTG->GINTSTS;
        if (!(sts & USB_OTG_GINTSTS_RXFLVL))
            // No packet ready
            return 0;
        uint32_t grx = OTG->GRXSTSR, grx_ep = grx & USB_OTG_GRXSTSP_EPNUM_Msk;
        uint32_t pktsts = ((grx & USB_OTG_GRXSTSP_PKTSTS_Msk)
                           >> USB_OTG_GRXSTSP_PKTSTS_Pos);
        if ((grx_ep == 0 || grx_ep == USB_CDC_EP_BULK_OUT)
            && (pktsts == 2 || pktsts == 4 || pktsts == 6)) {
            // A packet is ready
            if (grx_ep != ep)
                return 0;
            return grx;
        }
        if ((grx_ep != 0 && grx_ep != USB_CDC_EP_BULK_OUT)
            || (pktsts != 1 && pktsts != 3 && pktsts != 4)) {
            // Rx queue has bogus value - just pop it
            sts = OTG->GRXSTSP;
            continue;
        }
        // Discard informational entries from queue
        fifo_read_packet(NULL, 0);
    }
}

static void *usb_xfer_data;
static uint8_t usb_xfer_size, usb_xfer_flags;

static void
usb_stall_ep0(void)
{
    usb_irq_disable();
    EPIN(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
    usb_notify_ep0();
    usb_irq_enable();
}

int_fast8_t
usb_read_ep0(void *data, uint_fast8_t max_len)
{
    usb_irq_disable();
    uint32_t grx = peek_rx_queue(0);
    if (!grx) {
        // Wait for packet
        OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
        usb_irq_enable();
        return -1;
    }
    uint32_t pktsts = ((grx & USB_OTG_GRXSTSP_PKTSTS_Msk)
                       >> USB_OTG_GRXSTSP_PKTSTS_Pos);
    if (pktsts != 2) {
        // Transfer interrupted
        usb_irq_enable();
        return -2;
    }
    int_fast8_t ret = fifo_read_packet(data, max_len);
    enable_rx_endpoint(0);
    usb_irq_enable();
    return ret;
}

int_fast8_t
usb_read_ep0_setup(void *data, uint_fast8_t max_len)
{
    static uint8_t setup_buf[8];
    usb_irq_disable();
    for (;;) {
        uint32_t grx = peek_rx_queue(0);
        if (!grx) {
            // Wait for packet
            OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
            usb_irq_enable();
            return -1;
        }
        uint32_t pktsts = ((grx & USB_OTG_GRXSTSP_PKTSTS_Msk)
                           >> USB_OTG_GRXSTSP_PKTSTS_Pos);
        if (pktsts == 6)
            // Store setup packet
            fifo_read_packet(setup_buf, sizeof(setup_buf));
        else
            // Discard other packets
            fifo_read_packet(NULL, 0);
        if (pktsts == 4)
            // Setup complete
            break;
    }
    uint32_t ctl = EPIN(0)->DIEPCTL;
    if (ctl & USB_OTG_DIEPCTL_EPENA) {
        // Flush any pending tx packets
        EPIN(0)->DIEPCTL = ctl | USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        while (EPIN(0)->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
            ;
        OTG->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH;
        while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH)
            ;
    }
    enable_rx_endpoint(0);
    EPOUT(0)->DOEPINT = USB_OTG_DOEPINT_STUP;
    usb_irq_enable();
    // Return previously read setup packet
    memcpy(data, setup_buf, max_len);
    return max_len;
}

int_fast8_t
usb_send_ep0(const void *data, uint_fast8_t len)
{
    usb_irq_disable();
    uint32_t grx = peek_rx_queue(0);
    if (grx) {
        // Transfer interrupted
        usb_irq_enable();
        return -2;
    }
    if (EPIN(0)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        // Wait for space to transmit
        OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
        OTGD->DAINTMSK |= 1 << 0;
        usb_irq_enable();
        return -1;
    }
    int_fast8_t ret = fifo_write_packet(0, data, len);
    usb_irq_enable();
    return ret;
}

// Set the USB "stall" condition
static void
usb_do_stall(void)
{
    usb_stall_ep0();
    usb_xfer_flags = 0;
}

// Transfer data on the usb endpoint 0
static void
usb_do_xfer(void *data, uint_fast8_t size, uint_fast8_t flags)
{
    for (;;) {
        uint_fast8_t xs = size;
        if (xs > USB_CDC_EP0_SIZE)
            xs = USB_CDC_EP0_SIZE;
        int_fast8_t ret;
        if (flags & UX_READ)
            ret = usb_read_ep0(data, xs);
        else
            ret = usb_send_ep0(data, xs);
        if (ret == xs) {
            // Success
            data += xs;
            size -= xs;
            if (!size) {
                // Entire transfer completed successfully
                if (flags & UX_READ) {
                    // Send status packet at end of read
                    flags = UX_SEND;
                    continue;
                }
                if (xs == USB_CDC_EP0_SIZE && flags & UX_SEND_ZLP)
                    // Must send zero-length-packet
                    continue;
                usb_xfer_flags = 0;
                usb_notify_ep0();
                return;
            }
            continue;
        }
        if (ret == -1) {
            // Interface busy - retry later
            usb_xfer_data = data;
            usb_xfer_size = size;
            usb_xfer_flags = flags;
            return;
        }
        // Error
        usb_do_stall();
        return;
    }
}

#define GPIO_D_NEG GPIO('A', 11)
#define GPIO_D_POS GPIO('A', 12)
#define GPIO_FUNC GPIO_FUNCTION(10)

/****************************************************************
 * USB interface
 ****************************************************************/

int_fast8_t
usb_read_bulk_out(void *data, uint_fast8_t max_len)
{
    usb_irq_disable();
    uint32_t grx = peek_rx_queue(USB_CDC_EP_BULK_OUT);
    if (!grx) {
        // Wait for packet
        OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
        usb_irq_enable();
        return -1;
    }
    int_fast8_t ret = fifo_read_packet(data, max_len);
    enable_rx_endpoint(USB_CDC_EP_BULK_OUT);
    usb_irq_enable();
    return ret;
}

int_fast8_t
usb_send_bulk_in(void *data, uint_fast8_t len)
{
    usb_irq_disable();
    uint32_t ctl = EPIN(USB_CDC_EP_BULK_IN)->DIEPCTL;
    if (!(ctl & USB_OTG_DIEPCTL_USBAEP)) {
        // Controller not enabled - discard data
        usb_irq_enable();
        return len;
    }
    if (ctl & USB_OTG_DIEPCTL_EPENA) {
        // Wait for space to transmit
        OTGD->DAINTMSK |= 1 << USB_CDC_EP_BULK_IN;
        usb_irq_enable();
        return -1;
    }
    int_fast8_t ret = fifo_write_packet(USB_CDC_EP_BULK_IN, data, len);
    usb_irq_enable();
    return ret;
}

void
usb_set_address(uint_fast8_t addr)
{
    OTGD->DCFG = ((OTGD->DCFG & ~USB_OTG_DCFG_DAD_Msk)
                  | (addr << USB_OTG_DCFG_DAD_Pos));
    usb_send_ep0(NULL, 0);
    usb_notify_ep0();
}

void
usb_set_configure(void)
{
    usb_irq_disable();
    // Configure and enable USB_CDC_EP_ACM
    USB_OTG_INEndpointTypeDef *epi = EPIN(USB_CDC_EP_ACM);
    epi->DIEPTSIZ = (USB_CDC_EP_ACM_SIZE
                     | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
    epi->DIEPCTL = (
        USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_USBAEP
        | (0x03 << USB_OTG_DIEPCTL_EPTYP_Pos) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
        | (USB_CDC_EP_ACM << USB_OTG_DIEPCTL_TXFNUM_Pos)
        | (USB_CDC_EP_ACM_SIZE << USB_OTG_DIEPCTL_MPSIZ_Pos));

    // Configure and enable USB_CDC_EP_BULK_OUT
    USB_OTG_OUTEndpointTypeDef *epo = EPOUT(USB_CDC_EP_BULK_OUT);
    epo->DOEPTSIZ = 64 | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos);
    epo->DOEPCTL = (
        USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_USBAEP | USB_OTG_DOEPCTL_EPENA
        | (0x02 << USB_OTG_DOEPCTL_EPTYP_Pos) | USB_OTG_DOEPCTL_SD0PID_SEVNFRM
        | (USB_CDC_EP_BULK_OUT_SIZE << USB_OTG_DOEPCTL_MPSIZ_Pos));

    // Configure and flush USB_CDC_EP_BULK_IN
    epi = EPIN(USB_CDC_EP_BULK_IN);
    epi->DIEPTSIZ = (USB_CDC_EP_BULK_IN_SIZE
                     | (1 << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
    epi->DIEPCTL = (
        USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_USBAEP
        | (0x02 << USB_OTG_DIEPCTL_EPTYP_Pos) | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
        | (USB_CDC_EP_BULK_IN << USB_OTG_DIEPCTL_TXFNUM_Pos)
        | (USB_CDC_EP_BULK_IN_SIZE << USB_OTG_DIEPCTL_MPSIZ_Pos));
    while (epi->DIEPCTL & USB_OTG_DIEPCTL_EPENA)
        ;
    OTG->GRSTCTL = ((USB_CDC_EP_BULK_IN << USB_OTG_GRSTCTL_TXFNUM_Pos)
                    | USB_OTG_GRSTCTL_TXFFLSH);
    while (OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH)
        ;
    usb_irq_enable();
}


/****************************************************************
 * Setup and interrupts
 ****************************************************************/

// Main irq handler
void
OTG_FS_IRQHandler(void)
{
    uint32_t sts = OTG->GINTSTS;
    if (sts & USB_OTG_GINTSTS_RXFLVL) {
        // Received data - disable irq and notify endpoint
        OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
        uint32_t grx = OTG->GRXSTSR, ep = grx & USB_OTG_GRXSTSP_EPNUM_Msk;
        if (ep == 0)
            usb_notify_ep0();
        else
            usb_notify_bulk_out();
    }
    if (sts & USB_OTG_GINTSTS_IEPINT) {
        // Can transmit data - disable irq and notify endpoint
        uint32_t daint = OTGD->DAINT, msk = OTGD->DAINTMSK, pend = daint & msk;
        OTGD->DAINTMSK = msk & ~daint;
        if (pend & (1 << 0))
            usb_notify_ep0();
        if (pend & (1 << USB_CDC_EP_BULK_IN))
            usb_notify_bulk_in();
    }
}

#define CHIP_UID_LEN 12

static struct {
    struct usb_string_descriptor desc;
    uint16_t data[CHIP_UID_LEN * 2];
} cdc_chipid;

// Fill in a USB serial string descriptor from a chip id
void
usb_fill_serial(struct usb_string_descriptor *desc, int strlen, void *id)
{
    desc->bLength = sizeof(*desc) + strlen * sizeof(desc->data[0]);
    desc->bDescriptorType = USB_DT_STRING;

    uint8_t *src = id;
    int i;
    for (i = 0; i < strlen; i++) {
        uint8_t c = i & 1 ? src[i/2] & 0x0f : src[i/2] >> 4;
        desc->data[i] = c < 10 ? c + '0' : c - 10 + 'A';
    }
}

// Initialize the usb controller
void initUSB(void) {
    // Setup Chip ID
    if (CONFIG_USB_SERIAL_NUMBER_CHIPID)
        usb_fill_serial(&cdc_chipid.desc, ARRAY_SIZE(cdc_chipid.data)
                        , (void*)UID_BASE);

    // Enable USB clock
    if (READ_BIT(PWR->CR3, PWR_CR3_USB33RDY) != (PWR_CR3_USB33RDY)) {
        SET_BIT(PWR->CR3, PWR_CR3_USB33DEN);
    }
    SET_BIT(RCC->AHB1ENR, USBOTGEN);
    while (!(OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL))
        ;

    // Configure USB in full-speed device mode
    OTG->GUSBCFG = (USB_OTG_GUSBCFG_FDMOD | USB_OTG_GUSBCFG_PHYSEL
                    | (6 << USB_OTG_GUSBCFG_TRDT_Pos));
    OTGD->DCFG |= (3 << USB_OTG_DCFG_DSPD_Pos);
    OTG->GOTGCTL = USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;

    // Route pins
    gpio_peripheral(GPIO_D_NEG, GPIO_FUNC, 0);
    gpio_peripheral(GPIO_D_POS, GPIO_FUNC, 0);

    // Setup USB packet memory
    fifo_configure();

    // Configure and enable ep0
    uint32_t mpsize_ep0 = 2;
    USB_OTG_INEndpointTypeDef *epi = EPIN(0);
    USB_OTG_OUTEndpointTypeDef *epo = EPOUT(0);
    epi->DIEPCTL = mpsize_ep0 | USB_OTG_DIEPCTL_SNAK;
    epo->DOEPTSIZ = (64 | (1 << USB_OTG_DOEPTSIZ_STUPCNT_Pos)
                     | (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos));
    epo->DOEPCTL = mpsize_ep0 | USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_SNAK;

    // Enable interrupts
    OTGD->DIEPMSK = USB_OTG_DIEPMSK_XFRCM;
    OTG->GINTMSK = USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_IEPINT;
    OTG->GAHBCFG = USB_OTG_GAHBCFG_GINT;

    NVIC_SetPriority(OTG_IRQn, 1);
    usb_irq_enable();

    // Enable USB
    OTG->GCCFG |= USB_OTG_GCCFG_PWRDWN;
    OTGD->DCTL = 0;
}

#define CONCAT1(a, b) a ## b
#define CONCAT(a, b) CONCAT1(a, b)
#define USB_STR_MANUFACTURER CONCAT(u,CONFIG_USB_MANUFACTURER)
#define USB_STR_PRODUCT CONCAT(u,CONFIG_USB_PRODUCT)
#define USB_STR_SERIAL CONCAT(u,CONFIG_USB_SERIAL_NUMBER)

// String descriptors
enum {
    USB_STR_ID_MANUFACTURER = 1, USB_STR_ID_PRODUCT, USB_STR_ID_SERIAL,
};

#define SIZE_cdc_string_langids (sizeof(cdc_string_langids) + 2)

static const struct usb_string_descriptor cdc_string_langids = {
    .bLength = SIZE_cdc_string_langids,
    .bDescriptorType = USB_DT_STRING,
    .data = { cpu_to_le16(USB_LANGID_ENGLISH_US) },
};

#define SIZE_cdc_string_manufacturer \
    (sizeof(cdc_string_manufacturer) + sizeof(USB_STR_MANUFACTURER) - 2)

static const struct usb_string_descriptor cdc_string_manufacturer = {
    .bLength = SIZE_cdc_string_manufacturer,
    .bDescriptorType = USB_DT_STRING,
    .data = USB_STR_MANUFACTURER,
};

#define SIZE_cdc_string_product \
    (sizeof(cdc_string_product) + sizeof(USB_STR_PRODUCT) - 2)

static const struct usb_string_descriptor cdc_string_product = {
    .bLength = SIZE_cdc_string_product,
    .bDescriptorType = USB_DT_STRING,
    .data = USB_STR_PRODUCT,
};

#define SIZE_cdc_string_serial \
    (sizeof(cdc_string_serial) + sizeof(USB_STR_SERIAL) - 2)

static const struct usb_string_descriptor cdc_string_serial = {
    .bLength = SIZE_cdc_string_serial,
    .bDescriptorType = USB_DT_STRING,
    .data = USB_STR_SERIAL,
};

// Device descriptor
static const struct usb_device_descriptor cdc_device_descriptor = {
    .bLength = sizeof(cdc_device_descriptor),
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = cpu_to_le16(0x0200),
    .bDeviceClass = USB_CLASS_COMM,
    .bMaxPacketSize0 = USB_CDC_EP0_SIZE,
    .idVendor = cpu_to_le16(CONFIG_USB_VENDOR_ID),
    .idProduct = cpu_to_le16(CONFIG_USB_DEVICE_ID),
    .bcdDevice = cpu_to_le16(0x0100),
    .iManufacturer = USB_STR_ID_MANUFACTURER,
    .iProduct = USB_STR_ID_PRODUCT,
    .iSerialNumber = USB_STR_ID_SERIAL,
    .bNumConfigurations = 1,
};

// Config descriptor
static const struct config_s {
    struct usb_config_descriptor config;
    struct usb_interface_descriptor iface0;
    struct usb_cdc_header_descriptor cdc_hdr;
    struct usb_cdc_acm_descriptor cdc_acm;
    struct usb_cdc_union_descriptor cdc_union;
    struct usb_endpoint_descriptor ep1;
    struct usb_interface_descriptor iface1;
    struct usb_endpoint_descriptor ep2;
    struct usb_endpoint_descriptor ep3;
} PACKED cdc_config_descriptor = {
    .config = {
        .bLength = sizeof(cdc_config_descriptor.config),
        .bDescriptorType = USB_DT_CONFIG,
        .wTotalLength = cpu_to_le16(sizeof(cdc_config_descriptor)),
        .bNumInterfaces = 2,
        .bConfigurationValue = 1,
        .bmAttributes = 0xC0,
        .bMaxPower = 50,
    },
    .iface0 = {
        .bLength = sizeof(cdc_config_descriptor.iface0),
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = USB_CLASS_COMM,
        .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol = USB_CDC_ACM_PROTO_AT_V25TER,
    },
    .cdc_hdr = {
        .bLength = sizeof(cdc_config_descriptor.cdc_hdr),
        .bDescriptorType = USB_CDC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_HEADER_TYPE,
        .bcdCDC = 0x0110,
    },
    .cdc_acm = {
        .bLength = sizeof(cdc_config_descriptor.cdc_acm),
        .bDescriptorType = USB_CDC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_ACM_TYPE,
        .bmCapabilities = 0x06,
    },
    .cdc_union = {
        .bLength = sizeof(cdc_config_descriptor.cdc_union),
        .bDescriptorType = USB_CDC_CS_INTERFACE,
        .bDescriptorSubType = USB_CDC_UNION_TYPE,
        .bMasterInterface0 = 0,
        .bSlaveInterface0 = 1,
    },
    .ep1 = {
        .bLength = sizeof(cdc_config_descriptor.ep1),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_CDC_EP_ACM | USB_DIR_IN,
        .bmAttributes = USB_ENDPOINT_XFER_INT,
        .wMaxPacketSize = cpu_to_le16(USB_CDC_EP_ACM_SIZE),
        .bInterval = 255,
    },
    .iface1 = {
        .bLength = sizeof(cdc_config_descriptor.iface1),
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 1,
        .bNumEndpoints = 2,
        .bInterfaceClass = 0x0A,
    },
    .ep2 = {
        .bLength = sizeof(cdc_config_descriptor.ep2),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_CDC_EP_BULK_OUT,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = cpu_to_le16(USB_CDC_EP_BULK_OUT_SIZE),
    },
    .ep3 = {
        .bLength = sizeof(cdc_config_descriptor.ep3),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_CDC_EP_BULK_IN | USB_DIR_IN,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = cpu_to_le16(USB_CDC_EP_BULK_IN_SIZE),
    },
};

// List of available descriptors
static const struct descriptor_s {
    uint_fast16_t wValue;
    uint_fast16_t wIndex;
    const void *desc;
    uint_fast8_t size;
} cdc_descriptors[] = {
    { USB_DT_DEVICE<<8, 0x0000,
      &cdc_device_descriptor, sizeof(cdc_device_descriptor) },
    { USB_DT_CONFIG<<8, 0x0000,
      &cdc_config_descriptor, sizeof(cdc_config_descriptor) },
    { USB_DT_STRING<<8, 0x0000,
      &cdc_string_langids, SIZE_cdc_string_langids },
    { (USB_DT_STRING<<8) | USB_STR_ID_MANUFACTURER, USB_LANGID_ENGLISH_US,
      &cdc_string_manufacturer, SIZE_cdc_string_manufacturer },
    { (USB_DT_STRING<<8) | USB_STR_ID_PRODUCT, USB_LANGID_ENGLISH_US,
      &cdc_string_product, SIZE_cdc_string_product },
#if !CONFIG_USB_SERIAL_NUMBER_CHIPID
    { (USB_DT_STRING<<8) | USB_STR_ID_SERIAL, USB_LANGID_ENGLISH_US,
      &cdc_string_serial, SIZE_cdc_string_serial },
#endif
};

struct usb_string_descriptor *
usbserial_get_serialid(void)
{
   return &cdc_chipid.desc;
}

static void
usb_req_get_descriptor(struct usb_ctrlrequest *req)
{
    if (req->bRequestType != USB_DIR_IN)
        goto fail;
    void *desc = NULL;
    uint_fast8_t flags, size, i;
    for (i=0; i<ARRAY_SIZE(cdc_descriptors); i++) {
        const struct descriptor_s *d = &cdc_descriptors[i];
        if (d->wValue == req->wValue
            && d->wIndex == req->wIndex) {
            flags = UX_SEND;
            size = d->size;
            desc = (void*)d->desc;
        }
    }
    if (CONFIG_USB_SERIAL_NUMBER_CHIPID
        && req->wValue == ((USB_DT_STRING<<8) | USB_STR_ID_SERIAL)
        && req->wIndex == USB_LANGID_ENGLISH_US) {
            struct usb_string_descriptor *usbserial_serialid;
            usbserial_serialid = usbserial_get_serialid();
            flags = UX_SEND;
            size = usbserial_serialid->bLength;
            desc = (void*)usbserial_serialid;
    }
    if (desc) {
        if (size > req->wLength)
            size = req->wLength;
        else if (size < req->wLength)
            flags |= UX_SEND_ZLP;
        usb_do_xfer(desc, size, flags);
        return;
    }
fail:
    usb_do_stall();
}

static void
usb_req_set_address(struct usb_ctrlrequest *req)
{
    if (req->bRequestType || req->wIndex || req->wLength) {
        usb_do_stall();
        return;
    }
    usb_set_address(req->wValue);
}

static void
usb_req_set_configuration(struct usb_ctrlrequest *req)
{
    if (req->bRequestType || req->wValue != 1 || req->wIndex || req->wLength) {
        usb_do_stall();
        return;
    }
    usb_set_configure();
    usb_notify_bulk_in();
    usb_notify_bulk_out();
    usb_do_xfer(NULL, 0, UX_SEND);
}

static struct usb_cdc_line_coding line_coding;
static uint8_t line_control_state;

// Flag that bootloader is desired and reboot
static void
dfu_reboot(void)
{
    irq_disable();
    uint64_t *bflag = (void*)USB_BOOT_FLAG_ADDR;
    *bflag = USB_BOOT_FLAG;
    SCB_CleanDCache_by_Addr((void*)bflag, sizeof(*bflag));
    NVIC_SystemReset();
}

#define CANBOOT_SIGNATURE 0x21746f6f426e6143
#define CANBOOT_REQUEST   0x5984E3FA6CA1589B
#define CANBOOT_BYPASS    0x7b06ec45a9a8243d

static void
canboot_reset(uint64_t req_signature)
{
    uint32_t *bl_vectors = (uint32_t *)CONFIG_FLASH_BOOT_ADDRESS;
    uint64_t *boot_sig = (uint64_t *)(bl_vectors[1] - 9);
    uint64_t *req_sig = (uint64_t *)bl_vectors[0];
    if (boot_sig != (void*)ALIGN((size_t)boot_sig, 8)
        || *boot_sig != CANBOOT_SIGNATURE
        || req_sig != (void*)ALIGN((size_t)req_sig, 8))
        return;
    irq_disable();
    *req_sig = req_signature;
    SCB_CleanDCache_by_Addr((void*)req_sig, sizeof(*req_sig));
    NVIC_SystemReset();
}

void
try_request_canboot(void)
{
    canboot_reset(CANBOOT_REQUEST);
}

static void
bootloader_request(void) {
  try_request_canboot();
  dfu_reboot();
}

static void
check_reboot(void)
{
    if (line_coding.dwDTERate == 1200 && !(line_control_state & 0x01))
        // A baud of 1200 is an Arduino style request to enter the bootloader
        bootloader_request();
}

static void
usb_req_set_line_coding(struct usb_ctrlrequest *req)
{
    if (req->bRequestType != 0x21 || req->wValue || req->wIndex
        || req->wLength != sizeof(line_coding)) {
        usb_do_stall();
        return;
    }
    usb_do_xfer(&line_coding, sizeof(line_coding), UX_READ);
    check_reboot();
}

static void
usb_req_get_line_coding(struct usb_ctrlrequest *req)
{
    if (req->bRequestType != 0xa1 || req->wValue || req->wIndex
        || req->wLength < sizeof(line_coding)) {
        usb_do_stall();
        return;
    }
    usb_do_xfer(&line_coding, sizeof(line_coding), UX_SEND);
}

static void
usb_req_set_line(struct usb_ctrlrequest *req)
{
    if (req->bRequestType != 0x21 || req->wIndex || req->wLength) {
        usb_do_stall();
        return;
    }
    line_control_state = req->wValue;
    usb_do_xfer(NULL, 0, UX_SEND);
    check_reboot();
}

static void
usb_state_ready(void)
{
    struct usb_ctrlrequest req;
    int_fast8_t ret = usb_read_ep0_setup(&req, sizeof(req));
    if (ret != sizeof(req))
        return;
    switch (req.bRequest) {
    case USB_REQ_GET_DESCRIPTOR: usb_req_get_descriptor(&req); break;
    case USB_REQ_SET_ADDRESS: usb_req_set_address(&req); break;
    case USB_REQ_SET_CONFIGURATION: usb_req_set_configuration(&req); break;
    case USB_CDC_REQ_SET_LINE_CODING: usb_req_set_line_coding(&req); break;
    case USB_CDC_REQ_GET_LINE_CODING: usb_req_get_line_coding(&req); break;
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: usb_req_set_line(&req); break;
    default: usb_do_stall(); break;
    }
}

static uint8_t transmit_buf[192], transmit_pos;
static uint8_t receive_buf[128], receive_pos;

void usb_ep0_task(void)
{
    if (usb_xfer_flags)
        usb_do_xfer(usb_xfer_data, usb_xfer_size, usb_xfer_flags);
    else
        usb_state_ready();
}

void
usb_bulk_in_task(void)
{
    uint_fast8_t tpos = transmit_pos;
    if (!tpos)
        return;
    uint_fast8_t max_tpos = (tpos > USB_CDC_EP_BULK_IN_SIZE
                             ? USB_CDC_EP_BULK_IN_SIZE : tpos);
    int_fast8_t ret = usb_send_bulk_in(transmit_buf, max_tpos);
    if (ret <= 0)
        return;
    uint_fast8_t needcopy = tpos - ret;
    if (needcopy) {
        memmove(transmit_buf, &transmit_buf[ret], needcopy);
        usb_notify_bulk_in();
    }
    transmit_pos = needcopy;
}

void
usb_bulk_out_task(void)
{
    // Read data
    uint_fast8_t rpos = receive_pos;
    if (rpos + USB_CDC_EP_BULK_OUT_SIZE <= sizeof(receive_buf)) {
        int_fast8_t ret = usb_read_bulk_out(
            &receive_buf[rpos], USB_CDC_EP_BULK_OUT_SIZE);
        if (ret > 0) {
            rpos += ret;
            usb_notify_bulk_out();
        }
    } else {
        usb_notify_bulk_out();
    }
    // Process input
    int_fast8_t pop_count = console_receive(receive_buf, rpos);
    if (pop_count) {
        // Move buffer
        uint_fast8_t needcopy = rpos - pop_count;
        if (needcopy) {
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
            usb_notify_bulk_out();
        }
        rpos = needcopy;
    }
    receive_pos = rpos;
}

void runUSB() {
  if(todo_usb_ep0) {
    todo_usb_ep0 = false;

    usb_ep0_task();
  }

  if(todo_usb_bulk_in) {
    todo_usb_bulk_in = false;

    usb_bulk_in_task();
  }

  if(todo_usb_bulk_out) {
    todo_usb_bulk_out = false;

    usb_bulk_out_task();
  }
}

void usb_console_send(uint8_t *buf, uint_fast8_t buf_len) {
  // Verify space for message
  uint_fast8_t tpos = transmit_pos;
  if (tpos + buf_len > sizeof(transmit_buf))
      // Not enough space for message
      return;

  // Copy message
  memcpy(transmit_buf + tpos, buf, buf_len);

  // Start message transmit
  transmit_pos = tpos + buf_len;
  usb_notify_bulk_in();
}

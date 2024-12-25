/*
* The MIT License (MIT)
*
* Copyright (c) 2018, hathach (tinyusb.org)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* This file is part of the TinyUSB stack.
*/

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_SAM7

#include "AT91SAM7A3.h"
#include "device/dcd.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

__attribute__((always_inline)) static inline void delay (void)
{
    for (uint8_t nop_count = 0; nop_count < 10; nop_count++)
        __asm volatile("nop");
}

static TU_ATTR_ALIGNED(4) uint8_t _packet[8];
void dcd_int_handler (uint8_t rhport);

typedef struct
{
    tusb_xfer_type_t type;
    bool             complete;
    uint8_t          ep_bank;
    uint8_t         *buffer;
    tu_fifo_t       *ff;
    uint16_t         total_len;
    uint16_t         queued_len;
    uint16_t         max_size;
} xfer_ctl_t;

static xfer_ctl_t xfer_status[TUP_DCD_ENDPOINT_MAX];

// Start of Bus Reset
static void bus_reset ()
{
    tu_memclr(xfer_status, sizeof xfer_status);

    xfer_status[0].max_size = 8;
    xfer_status[0].buffer   = _packet;

    //Disable UDP IT
    AT91C_BASE_UDP->UDP_IDR = AT91C_UDP_EPINT0 | AT91C_UDP_EPINT1 | AT91C_UDP_EPINT2
                            | AT91C_UDP_EPINT3 | AT91C_UDP_EPINT4 | AT91C_UDP_EPINT5
                            | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | AT91C_UDP_EXTRSM
                            | AT91C_UDP_SOFINT | AT91C_UDP_WAKEUP;

    //Clear Interrupt Register
    AT91C_BASE_UDP->UDP_ICR = AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | AT91C_UDP_EXTRSM | AT91C_UDP_SOFINT | AT91C_UDP_ENDBUSRES | AT91C_UDP_WAKEUP;

    //Disable Endpoint
    for (uint8_t epnum = 0; epnum < 6; epnum++)
    {
        AT91C_BASE_UDP->UDP_CSR[epnum] &= ~AT91C_UDP_EPEDS;
    }

    //Reset UDP endpoint
    AT91C_BASE_UDP->UDP_RSTEP = AT91C_UDP_EP0 | AT91C_UDP_EP1 | AT91C_UDP_EP2 | AT91C_UDP_EP3 | AT91C_UDP_EP4 | AT91C_UDP_EP5;
    AT91C_BASE_UDP->UDP_RSTEP = 0;


    //Configure default control endpoint
    //Enable Endpoint
    AT91C_BASE_UDP->UDP_CSR[0] |= AT91C_UDP_EPEDS;
    //Enable UDP IT
    AT91C_BASE_UDP->UDP_IER     = AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | AT91C_UDP_EXTRSM;

    //Enable transceiver
    AT91C_BASE_UDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;

    dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
}

/*------------------------------------------------------------------*/
/* Device API
/*------------------------------------------------------------------*/

// Initialize controller to device mode
bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init)
{
    (void) rh_init;
    tu_memclr(xfer_status, sizeof(xfer_status));

    xfer_status[0].max_size = 8;
    xfer_status[0].buffer   = _packet;

    // Specific Chip USB Initialisation
    /* Set the PLL USB Divider */
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1;
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER    = AT91C_PMC_UDP;
    // Enables the corresponding peripheral clock.
    AT91C_BASE_PMC->PMC_PCER    = (1 << AT91C_ID_UDP);

    /* When using the USB debugger the peripheral registers do not always get
    set to the correct default values.  To make sure set the relevant registers
    manually here. */

    //Disable UDP IT
    AT91C_BASE_UDP->UDP_IDR = AT91C_UDP_EPINT0 | AT91C_UDP_EPINT1 | AT91C_UDP_EPINT2
                            | AT91C_UDP_EPINT3 | AT91C_UDP_EPINT4 | AT91C_UDP_EPINT5
                            | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | AT91C_UDP_EXTRSM
                            | AT91C_UDP_SOFINT | AT91C_UDP_WAKEUP;

    //Clear Interrupt Register
    AT91C_BASE_UDP->UDP_ICR = AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | AT91C_UDP_EXTRSM | AT91C_UDP_SOFINT | AT91C_UDP_ENDBUSRES | AT91C_UDP_WAKEUP;

    //Disable Endpoint
    for (uint8_t epnum = 0; epnum < 6; epnum++)
    {
        AT91C_BASE_UDP->UDP_CSR[epnum] &= ~AT91C_UDP_EPEDS;
    }

    //Reset UDP endpoint
    AT91C_BASE_UDP->UDP_RSTEP = AT91C_UDP_EP0 | AT91C_UDP_EP1 | AT91C_UDP_EP2 | AT91C_UDP_EP3 | AT91C_UDP_EP4 | AT91C_UDP_EP5;
    AT91C_BASE_UDP->UDP_RSTEP = 0;

    //Enable transceiver
    AT91C_BASE_UDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;

    //Configure default control endpoint
    //Enable Endpoint
    AT91C_BASE_UDP->UDP_CSR[0] |= AT91C_UDP_EPEDS;
    //Enable UDP IT
    AT91C_BASE_UDP->UDP_IER     = AT91C_UDP_EPINT0 | AT91C_UDP_RXSUSP | AT91C_UDP_RXRSM | AT91C_UDP_EXTRSM;

    // Disable the interrupt first
    AT91C_BASE_AIC->AIC_IDCR = (1 << AT91C_ID_UDP);

    // Configure mode and handler
    AT91C_BASE_AIC->AIC_SMR[AT91C_ID_UDP] = AT91C_AIC_PRIOR_HIGHEST;
    AT91C_BASE_AIC->AIC_SVR[AT91C_ID_UDP] = (unsigned int) dcd_int_handler;

    // Clear interrupt
    AT91C_BASE_AIC->AIC_ICCR = (1 << AT91C_ID_UDP);

    dcd_connect(0);
    return true;
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
    (void) rhport;
    AT91C_BASE_AIC->AIC_IECR = (1 << AT91C_ID_UDP);
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
    (void) rhport;
    AT91C_BASE_AIC->AIC_IDCR = (1 << AT91C_ID_UDP);
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
    (void) rhport;
}

void dcd_sof_enable (uint8_t rhport, bool en)
{
    (void) rhport;

    if (en)
        AT91C_BASE_UDP->UDP_IER = AT91C_UDP_SOFINT;
    else
        AT91C_BASE_UDP->UDP_IDR = AT91C_UDP_SOFINT;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

void dcd_edpt0_status_complete (uint8_t rhport, tusb_control_request_t const *request)
{
    (void) rhport;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE && request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD)
    {
        if (request->bRequest == TUSB_REQ_SET_ADDRESS)
        {
            uint8_t const dev_addr = (uint8_t) request->wValue;

            // Enable addressed state
            AT91C_BASE_UDP->UDP_GLBSTATE |= AT91C_UDP_FADDEN;

            // Set new address & Function enable bit
            AT91C_BASE_UDP->UDP_FADDR = (AT91C_UDP_FEN | dev_addr);
        }
        else if (request->bRequest == TUSB_REQ_SET_CONFIGURATION)
        {
            // Configured State
            AT91C_BASE_UDP->UDP_GLBSTATE |= AT91C_UDP_CONFG;
        }
    }
}

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const *ep_desc)
{
    (void) rhport;
    uint8_t const epnum = tu_edpt_number(ep_desc->bEndpointAddress);
    uint8_t const dir   = tu_edpt_dir(ep_desc->bEndpointAddress);

    xfer_status[epnum].max_size = tu_edpt_packet_size(ep_desc);
    xfer_status[epnum].type     = ep_desc->bmAttributes.xfer;

    switch (ep_desc->bmAttributes.xfer)
    {
        case TUSB_XFER_CONTROL:
            AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_EPTYPE_CTRL;
            delay();
            break;
        case TUSB_XFER_ISOCHRONOUS:
            AT91C_BASE_UDP->UDP_CSR[epnum] |= (dir == TUSB_DIR_IN) ? AT91C_UDP_EPTYPE_ISO_IN : AT91C_UDP_EPTYPE_ISO_OUT;
            delay();
            break;
        case TUSB_XFER_BULK:
            AT91C_BASE_UDP->UDP_CSR[epnum] |= (dir == TUSB_DIR_IN) ? AT91C_UDP_EPTYPE_BULK_IN : AT91C_UDP_EPTYPE_BULK_OUT;
            delay();
            break;
        case TUSB_XFER_INTERRUPT:
            AT91C_BASE_UDP->UDP_CSR[epnum] |= (dir == TUSB_DIR_IN) ? AT91C_UDP_EPTYPE_INT_IN : AT91C_UDP_EPTYPE_INT_OUT;
            delay();
            break;
    }

    //Enable Endpoint
    AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_EPEDS;
    delay();

    //Enable UDP IT
    if (dir == TUSB_DIR_IN || (ep_desc->bmAttributes.xfer == TUSB_XFER_CONTROL))
        AT91C_BASE_UDP->UDP_IER = (1 << epnum);
    //    else
    if (dir == TUSB_DIR_OUT)
        xfer_status[epnum].ep_bank = AT91C_UDP_RX_DATA_BK0;
    else
        xfer_status[epnum].ep_bank = 0;

    return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
    (void) rhport;
}

static void dcd_transmit (uint8_t epnum, bool in_isr)
{
    if (xfer_status[epnum].complete == true)
        return;

    if ((xfer_status[epnum].total_len == xfer_status[epnum].queued_len) && (xfer_status[epnum].ep_bank != 0))
    {
        xfer_status[epnum].complete = true;
        dcd_event_xfer_complete(0, tu_edpt_addr(epnum, TUSB_DIR_IN), xfer_status[epnum].total_len, XFER_RESULT_SUCCESS, in_isr);
        return;
    }

    uint16_t len = (uint16_t) (xfer_status[epnum].total_len - xfer_status[epnum].queued_len);
    if (len > xfer_status[epnum].max_size)
        len = xfer_status[epnum].max_size;

    if (xfer_status[epnum].ff != NULL)
        for (int i = 0; i < len; i++)
        {
            uint8_t data;
            tu_fifo_read(xfer_status[epnum].ff, &data);
            AT91C_BASE_UDP->UDP_FDR[epnum] = data;
        }
    else
    {
        uint8_t *data = &(xfer_status[epnum].buffer[xfer_status[epnum].queued_len]);
        for (int i = 0; i < len; i++)
            AT91C_BASE_UDP->UDP_FDR[epnum] = data[i];
    }

    xfer_status[epnum].ep_bank++;
    xfer_status[epnum].queued_len = xfer_status[epnum].queued_len + len;

    // TX ready for transfer
    if (xfer_status[epnum].ep_bank == 1)
    {
        AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_TXPKTRDY;
    }
}

static void dcd_transmit_ping_pong (uint8_t epnum, bool in_isr)
{
    if (xfer_status[epnum].complete == true)
        return;

    if ((xfer_status[epnum].total_len == xfer_status[epnum].queued_len) && (xfer_status[epnum].ep_bank != 0))
    {
        xfer_status[epnum].complete = true;
        dcd_event_xfer_complete(0, tu_edpt_addr(epnum, TUSB_DIR_IN), xfer_status[epnum].total_len, XFER_RESULT_SUCCESS, in_isr);
        return;
    }

    uint16_t len = xfer_status[epnum].total_len - xfer_status[epnum].queued_len;
    if (len > xfer_status[epnum].max_size)
        len = xfer_status[epnum].max_size;

    if (xfer_status[epnum].ff != NULL)
        for (int i = 0; i < len; i++)
        {
            uint8_t data;
            tu_fifo_read(xfer_status[epnum].ff, &data);
            AT91C_BASE_UDP->UDP_FDR[epnum] = data;
        }
    else
    {
        uint8_t *data = &(xfer_status[epnum].buffer[xfer_status[epnum].queued_len]);
        for (int i = 0; i < len; i++)
            AT91C_BASE_UDP->UDP_FDR[epnum] = data[i];
    }

    xfer_status[epnum].ep_bank++;
    xfer_status[epnum].queued_len = xfer_status[epnum].queued_len + len;

    // TX ready for transfer
    if (xfer_status[epnum].ep_bank == 1)
        AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_TXPKTRDY;

    if ((xfer_status[epnum].total_len != xfer_status[epnum].queued_len) && xfer_status[epnum].ep_bank != 2)
        dcd_transmit_ping_pong(epnum, in_isr);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
    // Response with zlp status
    xfer_status[0].buffer     = NULL;
    xfer_status[0].ff         = NULL;
    xfer_status[0].total_len  = 0;
    xfer_status[0].queued_len = 0;
    xfer_status[0].complete   = false;
    dcd_transmit(0, true);
}

// RX/OUT direction
static void dcd_receive (uint8_t epnum)
{
    uint16_t count = (uint16_t) ((AT91C_BASE_UDP->UDP_CSR[epnum] & AT91C_UDP_RXBYTECNT) >> 16);

    if (count != 0U)
    {
        if (xfer_status[epnum].ff != NULL)
            for (uint16_t i = 0; i < count; i++)
            {
                uint8_t data = (uint8_t) AT91C_BASE_UDP->UDP_FDR[epnum];
                tu_fifo_write(xfer_status[epnum].ff, &data);
            }
        else if (xfer_status[epnum].buffer != NULL)
        {
            uint8_t *data = &(xfer_status[epnum].buffer[xfer_status[epnum].queued_len]);
            for (int i = 0; i < count; i++)
                data[i] = AT91C_BASE_UDP->UDP_FDR[epnum];
        }
        xfer_status[epnum].queued_len = (uint16_t) (xfer_status[epnum].queued_len + count);
    }

    if ((count < xfer_status[epnum].max_size) || (xfer_status[epnum].queued_len == xfer_status[epnum].total_len))
    {
        if (xfer_status[epnum].type != TUSB_XFER_CONTROL)
            AT91C_BASE_UDP->UDP_IDR = (1 << epnum);
        dcd_event_xfer_complete(0, tu_edpt_addr(epnum, TUSB_DIR_OUT), xfer_status[epnum].queued_len, XFER_RESULT_SUCCESS, true);
    }

    if ((epnum == 0) || (epnum == 3))
        AT91C_BASE_UDP->UDP_CSR[epnum] &= ~AT91C_UDP_RX_DATA_BK0;
    else
    {
        AT91C_BASE_UDP->UDP_CSR[epnum] &= ~xfer_status[epnum].ep_bank;
        xfer_status[epnum].ep_bank      = (xfer_status[epnum].ep_bank == AT91C_UDP_RX_DATA_BK0) ? AT91C_UDP_RX_DATA_BK1 : AT91C_UDP_RX_DATA_BK0;
    }
}

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
    (void) rhport;

    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);

    xfer_status[epnum].buffer     = buffer;
    xfer_status[epnum].ff         = NULL;
    xfer_status[epnum].total_len  = total_bytes;
    xfer_status[epnum].queued_len = 0;
    xfer_status[epnum].complete   = false;

    if (dir == TUSB_DIR_IN)
        if ((epnum == 0) || (epnum == 3))
            dcd_transmit(epnum, false);
        else
        {
            AT91C_BASE_UDP->UDP_IDR = (1 << epnum);
            dcd_transmit_ping_pong(epnum, false);
            AT91C_BASE_UDP->UDP_IER = (1 << epnum);
        }
    else
    {
        if (AT91C_BASE_UDP->UDP_CSR[epnum] & (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1))
            dcd_receive(epnum);
        if ((xfer_status[epnum].type != TUSB_XFER_CONTROL) && (xfer_status[epnum].total_len != xfer_status[epnum].queued_len))
            AT91C_BASE_UDP->UDP_IER = (1 << epnum);
    }
    return true;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo (uint8_t rhport, uint8_t ep_addr, tu_fifo_t *ff, uint16_t total_bytes)
{
    (void) rhport;
    // USB buffers always work in bytes so to avoid unnecessary divisions we demand item_size = 1
    TU_ASSERT(ff->item_size == 1);

    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);

    xfer_status[epnum].buffer     = NULL;
    xfer_status[epnum].ff         = ff;
    xfer_status[epnum].total_len  = total_bytes;
    xfer_status[epnum].queued_len = 0;
    xfer_status[epnum].complete   = false;

    if (dir == TUSB_DIR_IN)
        if ((epnum == 0) || (epnum == 3))
            dcd_transmit(epnum, false);
        else
        {
            AT91C_BASE_UDP->UDP_IDR = (1 << epnum);
            dcd_transmit_ping_pong(epnum, false);
            AT91C_BASE_UDP->UDP_IER = (1 << epnum);
        }
    else
    {
        if (AT91C_BASE_UDP->UDP_CSR[epnum] & (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1))
            dcd_receive(epnum);
        if ((xfer_status[epnum].type != TUSB_XFER_CONTROL) && (xfer_status[epnum].total_len != xfer_status[epnum].queued_len))
            AT91C_BASE_UDP->UDP_IER = (1 << epnum);
    }
    return true;
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;
    uint8_t const epnum = tu_edpt_number(ep_addr);

    AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_FORCESTALL;
}

// clear stall
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;

    uint8_t const epnum = tu_edpt_number(ep_addr);

    //Reset UDP endpoint
    AT91C_BASE_UDP->UDP_RSTEP = (1 << epnum);
    AT91C_BASE_UDP->UDP_RSTEP = 0;

    AT91C_BASE_UDP->UDP_CSR[epnum] &= ~(AT91C_UDP_STALLSENT | AT91C_UDP_FORCESTALL);
}

void dcd_int_handler (uint8_t rhport)
{
    uint32_t const intr_mask   = AT91C_BASE_UDP->UDP_IMR;
    uint32_t const intr_status = AT91C_BASE_UDP->UDP_ISR & intr_mask;

    // Bus reset
    if (intr_status & AT91C_UDP_ENDBUSRES)
        bus_reset();

    // Suspend
    if (intr_status & AT91C_UDP_RXSUSP)
        dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);

    // Resume
    if (intr_status & AT91C_UDP_RXRSM)
        dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);

    // Wakeup
    if (intr_status & AT91C_UDP_WAKEUP)
        dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);

    // SOF
    if (intr_status & AT91C_UDP_SOFINT)
        dcd_event_sof(0, (AT91C_BASE_UDP->UDP_NUM & AT91C_UDP_FRM_NUM), true);

    for (uint8_t epnum = 0; epnum < 6; epnum++)
    {
        if (intr_status & (1 << epnum))
        {
            //USB_TRANSACTION_SETUP
            if (AT91C_BASE_UDP->UDP_CSR[epnum] & AT91C_UDP_RXSETUP)
            {
                uint16_t count = (uint16_t) ((AT91C_BASE_UDP->UDP_CSR[epnum] & AT91C_UDP_RXBYTECNT) >> 16);
                for (int i = 0; i < count; i++)
                    _packet[i] = (uint8_t) AT91C_BASE_UDP->UDP_FDR[epnum];
                // Set EP direction bit according to DATA stage
                // MUST only be set before RXSETUP is clear per specs
                if (tu_edpt_dir(_packet[0]))
                {
                    AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_DIR;
                    delay();
                }
                else
                {
                    AT91C_BASE_UDP->UDP_CSR[epnum] &= ~AT91C_UDP_DIR;
                    delay();
                }
                // Clear Setup, stall and other on-going transfer bits
                AT91C_BASE_UDP->UDP_CSR[epnum] &= ~(AT91C_UDP_RXSETUP | AT91C_UDP_STALLSENT | AT91C_UDP_FORCESTALL);
                delay();
                // notify usbd
                dcd_event_setup_received(rhport, _packet, true);
            }

            //USB_TRANSACTION_IN
            if (AT91C_BASE_UDP->UDP_CSR[epnum] & AT91C_UDP_TXCOMP)
            {
                AT91C_BASE_UDP->UDP_CSR[epnum] &= ~AT91C_UDP_TXCOMP;
                if ((epnum == 0) || (epnum == 3))
                {
                    dcd_transmit(epnum, true);
                    xfer_status[epnum].ep_bank--;
                }
                else
                {
                    if (xfer_status[epnum].ep_bank == 2)
                    {
                        AT91C_BASE_UDP->UDP_CSR[epnum] |= AT91C_UDP_TXPKTRDY;
                        xfer_status[epnum].ep_bank--;
                        dcd_transmit_ping_pong(epnum, true);
                    }
                    else
                    {
                        dcd_transmit_ping_pong(epnum, true);
                        xfer_status[epnum].ep_bank--;
                    }
                }
            }

            //USB_TRANSACTION_OUT
            if (AT91C_BASE_UDP->UDP_CSR[epnum] & (AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1))
                dcd_receive(epnum);

            //STALL
            if (AT91C_BASE_UDP->UDP_CSR[epnum] & (AT91C_UDP_STALLSENT))
            {
                if (epnum == 0)
                    AT91C_BASE_UDP->UDP_CSR[epnum] &= ~(AT91C_UDP_RXSETUP | AT91C_UDP_TXPKTRDY | AT91C_UDP_TXCOMP | AT91C_UDP_RX_DATA_BK0 | AT91C_UDP_RX_DATA_BK1 | AT91C_UDP_STALLSENT | AT91C_UDP_FORCESTALL);
            }
            break;
        }
    }
    // clear interrupt
    AT91C_BASE_UDP->UDP_ICR = intr_status;

    AT91C_BASE_PIOB->PIO_CODR = AT91C_PIO_PB0;
}

#endif

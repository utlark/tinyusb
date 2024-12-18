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

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_MDR32F92QI

#include "device/dcd.h"
#include "MDR32F9Q2I.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define RST_CLK_PER_CLOCK_PCLK_EN_USB    ((uint32_t) 0x00000004)
#define RST_CLK_USB_CLOCK_USB_C1_SEL_HSE (0x02 << RST_CLK_USB_CLOCK_USB_C1_SEL_Pos)


static TU_ATTR_ALIGNED(4) uint8_t _packet[64];

typedef struct
{
    bool       working;
    uint8_t   *buffer;
    tu_fifo_t *ff;
    uint16_t   total_len;
    uint16_t   queued_len;
    uint16_t   max_size;
} xfer_ctl_t;

static xfer_ctl_t xfer_status[4];

// Start of Bus Reset
static void bus_reset ()
{
    // clear device address
    MDR_USB->SA = 0;

    tu_memclr(xfer_status, sizeof xfer_status);

    xfer_status[0].max_size = 64;

    for (uint8_t epnum = 0; epnum < 3; epnum++)
    {
        MDR_USB->USB_SEP[epnum].CTRL       = 0;
        MDR_USB->USB_SEP_FIFO[epnum].TXFDC = 1;
        MDR_USB->USB_SEP_FIFO[epnum].RXFC  = 1;
    }
    xfer_status[0].working   = true;
    MDR_USB->USB_SEP[0].CTRL = USB_SEP_CTRL_EPEN | USB_SEP_CTRL_EPRDY;

    dcd_event_bus_reset(0, TUSB_SPEED_FULL, true);
}

/*------------------------------------------------------------------*/
/* Device API
/*------------------------------------------------------------------*/

// Initialize controller to device mode
void dcd_init (uint8_t rhport)
{
    MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_USB;

    MDR_RST_CLK->USB_CLOCK   |= RST_CLK_USB_CLOCK_USB_C2_SEL | RST_CLK_USB_CLOCK_USB_C1_SEL_HSE | RST_CLK_USB_CLOCK_USB_CLK_EN;
    MDR_RST_CLK->PLL_CONTROL |= ((6 - 1) << RST_CLK_PLL_CONTROL_PLL_USB_MUL_Pos) | RST_CLK_PLL_CONTROL_PLL_USB_ON;
    while ((MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_USB_RDY) != RST_CLK_CLOCK_STATUS_PLL_USB_RDY)
        __NOP();

    MDR_USB->HSCR |= USB_HSCR_RESET_CORE;
    for (uint32_t i = 0; i < 1000; i++)
        __NOP();
    MDR_USB->HSCR &= ~USB_HSCR_RESET_CORE;

    MDR_USB->SC  |= USB_SCFSR | USB_SCFSP | USB_SCGEN;
    MDR_USB->SIM |= USB_SIM_SCTDONEIE | USB_SIM_SCRESUMEIE | USB_SIM_SCRESETEVIE;

    MDR_USB->SA = 0;

    for (uint8_t epnum = 0; epnum < 3; epnum++)
    {
        MDR_USB->USB_SEP[epnum].CTRL       = 0;
        MDR_USB->USB_SEP_FIFO[epnum].TXFDC = 1;
        MDR_USB->USB_SEP_FIFO[epnum].RXFC  = 1;
    }

    MDR_USB->HSCR |= USB_HSCR_EN_RX | USB_HSCR_EN_TX;

    tu_memclr(xfer_status, sizeof(xfer_status));

    xfer_status[0].max_size = 64;
    xfer_status[0].working  = true;

    MDR_USB->USB_SEP[0].CTRL = USB_SEP_CTRL_EPEN | USB_SEP_CTRL_EPRDY;

    dcd_connect(0);
}

// Enable device interrupt
void dcd_int_enable (uint8_t rhport)
{
    (void) rhport;
    NVIC_EnableIRQ(USB_IRQn);
}

// Disable device interrupt
void dcd_int_disable (uint8_t rhport)
{
    (void) rhport;
    NVIC_DisableIRQ(USB_IRQn);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
    (void) rhport;
    // Response with status after changing device address
    MDR_USB->USB_SEP[0].CTRL = USB_SEP_CTRL_EPEN | USB_SEP_CTRL_EPRDY | USB_SEP_CTRL_EPDATASEQ;
}

// Wake up host
void dcd_remote_wakeup (uint8_t rhport)
{
    (void) rhport;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect (uint8_t rhport)
{
    (void) rhport;
    MDR_USB->HSCR |= USB_HSCR_DP_PULLUP;
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect (uint8_t rhport)
{
    (void) rhport;
    MDR_USB->HSCR &= ~USB_HSCR_DP_PULLUP;
}

void dcd_sof_enable (uint8_t rhport, bool en)
{
    (void) rhport;

    if (en)
        MDR_USB->SIM |= USB_SIM_SCSOFRECIE;
    else
        MDR_USB->SIM &= ~USB_SIM_SCSOFRECIE;
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

void dcd_edpt0_status_complete (uint8_t rhport, tusb_control_request_t const *request)
{
    (void) rhport;
    (void) request;
    xfer_status[0].working   = true;
    MDR_USB->USB_SEP[0].CTRL = USB_SEP_CTRL_EPEN | USB_SEP_CTRL_EPRDY;

    if (request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_DEVICE && request->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD)
        if (request->bRequest == TUSB_REQ_SET_ADDRESS)
        {
            uint8_t const dev_addr = (uint8_t) request->wValue;
            MDR_USB->SA            = dev_addr;
        }
}

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const *ep_desc)
{
    (void) rhport;
    uint8_t const epnum = tu_edpt_number(ep_desc->bEndpointAddress);
    uint8_t const dir   = tu_edpt_dir(ep_desc->bEndpointAddress);

    xfer_status[epnum].max_size = tu_edpt_packet_size(ep_desc);

    MDR_USB->USB_SEP[epnum].CTRL = 0;

    if (dir == TUSB_DIR_IN)
        MDR_USB->USB_SEP[epnum].CTRL |= USB_SEP_CTRL_EPDATASEQ;

    if (ep_desc->bmAttributes.xfer == TUSB_XFER_CONTROL)
        MDR_USB->USB_SEP[epnum].CTRL |= USB_SEP_CTRL_EPRDY;

    if (ep_desc->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS)
        MDR_USB->USB_SEP[epnum].CTRL |= USB_SEP_CTRL_EPISOEN;

    MDR_USB->USB_SEP_FIFO[epnum].TXFDC  = 1;
    MDR_USB->USB_SEP_FIFO[epnum].RXFC   = 1;
    MDR_USB->USB_SEP[epnum].CTRL       |= USB_SEP_CTRL_EPEN;

    return true;
}

void dcd_edpt_close_all (uint8_t rhport)
{
    (void) rhport;
}

static void dcd_transmit_packet (uint8_t epnum)
{
    uint16_t len = (uint16_t) (xfer_status[epnum].total_len - xfer_status[epnum].queued_len);
    if (len > xfer_status[epnum].max_size)
        len = xfer_status[epnum].max_size;

    MDR_USB->USB_SEP_FIFO[epnum].TXFDC = 1;

    if (xfer_status[epnum].ff != NULL)
        for (int i = 0; i < len; i++)
        {
            uint8_t data;
            tu_fifo_read(xfer_status[epnum].ff, &data);
            MDR_USB->USB_SEP_FIFO[epnum].TXFD = data;
        }
    else
        for (int i = 0; i < len; i++)
            MDR_USB->USB_SEP_FIFO[epnum].TXFD = xfer_status[epnum].buffer[xfer_status[epnum].queued_len + i];

    xfer_status[epnum].queued_len  = (uint16_t) (xfer_status[epnum].queued_len + len);
    xfer_status[epnum].working     = true;
    MDR_USB->USB_SEP[epnum].CTRL  ^= USB_SEP_CTRL_EPDATASEQ;
    MDR_USB->USB_SEP[epnum].CTRL  |= USB_SEP_CTRL_EPRDY;
}

static bool edpt_xfer (uint8_t const epnum, uint8_t const dir)
{
    if (dir == TUSB_DIR_IN)
        dcd_transmit_packet(epnum);
    else
    {
        MDR_USB->USB_SEP_FIFO[epnum].RXFC  = 1;
        xfer_status[epnum].working         = true;
        MDR_USB->USB_SEP[epnum].CTRL      |= USB_SEP_CTRL_EPRDY;
    }
    return true;
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

    return edpt_xfer(epnum, dir);
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

    return edpt_xfer(epnum, dir);
}

// Stall endpoint
void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;

    uint8_t const epnum = tu_edpt_number(ep_addr);

    xfer_status[epnum].working    = true;
    MDR_USB->USB_SEP[epnum].CTRL |= USB_SEP_CTRL_EPSSTALL | USB_SEP_CTRL_EPRDY;
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
    (void) rhport;

    uint8_t const epnum = tu_edpt_number(ep_addr);
    uint8_t const dir   = tu_edpt_dir(ep_addr);

    MDR_USB->USB_SEP_FIFO[epnum].RXFC  = 1;
    MDR_USB->USB_SEP_FIFO[epnum].TXFDC = 1;
    if (dir == TUSB_DIR_OUT)
    {
        xfer_status[epnum].working   = true;
        MDR_USB->USB_SEP[epnum].CTRL = USB_SEP_CTRL_EPRDY | USB_SEP_CTRL_EPEN;
    }
    else
    {
        xfer_status[epnum].working   = true;
        MDR_USB->USB_SEP[epnum].CTRL = USB_SEP_CTRL_EPDATASEQ | USB_SEP_CTRL_EPEN;
    }
}

// Handle interrupt for the TX/IN direction
static void dcd_ep_ctr_tx_handler (uint8_t epnum)
{
    if (xfer_status[epnum].total_len != xfer_status[epnum].queued_len)
        dcd_transmit_packet(epnum);
    else
        dcd_event_xfer_complete(0, tu_edpt_addr(epnum, TUSB_DIR_IN), xfer_status[epnum].total_len, XFER_RESULT_SUCCESS, true);
}

// Handle interrupt for the RX/OUT direction
static void dcd_ep_ctr_rx_handler (uint8_t epnum)
{
    uint32_t count;
    count = MDR_USB->USB_SEP_FIFO[epnum].RXFDC_H;

    TU_ASSERT(count <= xfer_status[epnum].max_size, /**/);

    if (count != 0U)
    {
        if (xfer_status[epnum].ff)
        {
            for (uint8_t i = 0; i < count; i++)
            {
                uint8_t data = (uint8_t) MDR_USB->USB_SEP_FIFO[epnum].RXFD;
                tu_fifo_write(xfer_status[epnum].ff, &data);
            }
        }
        else
        {
            for (uint8_t i = 0; i < count; i++)
                xfer_status[epnum].buffer[xfer_status[epnum].queued_len + i] = (uint8_t) MDR_USB->USB_SEP_FIFO[epnum].RXFD;
        }
        xfer_status[epnum].queued_len = (uint16_t) (xfer_status[epnum].queued_len + count);
    }

    if ((count < xfer_status[epnum].max_size) || (xfer_status[epnum].queued_len == xfer_status[epnum].total_len))
        dcd_event_xfer_complete(0, tu_edpt_addr(epnum, TUSB_DIR_OUT), xfer_status[epnum].queued_len, XFER_RESULT_SUCCESS, true);
    else
    {
        MDR_USB->USB_SEP_FIFO[epnum].RXFC  = 1;
        MDR_USB->USB_SEP[epnum].CTRL      |= USB_SEP_CTRL_EPRDY;
        xfer_status[epnum].working         = true;
    }
}

void dcd_int_handler (uint8_t rhport)
{
    uint32_t regSIS;

    regSIS = MDR_USB->SIS;

    if (regSIS & USB_SIS_SCRESETEV)
    {
        MDR_USB->SIS = USB_SIS_SCRESETEV;
        bus_reset();
    }

    if (regSIS & USB_SIS_SCTDONE)
    {
        MDR_USB->SIS = USB_SIS_SCTDONE;
        for (uint8_t epnum = 0; epnum < 3; epnum++)
        {
            if (((MDR_USB->USB_SEP[epnum].CTRL & USB_SEP_CTRL_EPRDY) == 0) && xfer_status[epnum].working)
            {
                switch (MDR_USB->USB_SEP[epnum].TS & 0x03)
                {
                    case 0: //USB_TRANSACTION_SETUP
                        uint32_t count = MDR_USB->USB_SEP_FIFO[epnum].RXFDC_H;
                        for (int i = 0; i < count; i++)
                            _packet[i] = (uint8_t) MDR_USB->USB_SEP_FIFO[epnum].RXFD;
                        MDR_USB->USB_SEP_FIFO[epnum].RXFC = 1;
                        dcd_event_setup_received(0, _packet, true);
                        break;
                    case 1: //USB_TRANSACTION_IN
                        if ((epnum == 0) && ((MDR_USB->USB_SEP[0].STS & USB_SEP_STS_SCSTALLSENT) != 0))
                            MDR_USB->USB_SEP[0].CTRL = USB_SEP_CTRL_EPEN | USB_SEP_CTRL_EPRDY;
                        else
                        {
                            xfer_status[epnum].working = false;
                            dcd_ep_ctr_tx_handler(epnum);
                        }
                        break;
                    case 2: //USB_TRANSACTION_OUT
                        xfer_status[epnum].working = false;
                        dcd_ep_ctr_rx_handler(epnum);
                        break;
                }
            }
        }
    }

    if (regSIS & USB_SIS_SCSOFREC)
    {
        MDR_USB->SIS = USB_SIS_SCSOFREC;
        dcd_event_sof(0, MDR_USB->SFN_H, true);
    }
    if (regSIS & USB_SIS_SCRESUME)
    {
        MDR_USB->SIS = USB_SIS_SCRESUME;
        dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
    }
}

#endif

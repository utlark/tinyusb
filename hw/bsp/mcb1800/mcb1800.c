/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include "chip.h"
#include "../board.h"

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+
void USB0_IRQHandler(void)
{
  #if CFG_TUSB_RHPORT0_MODE & OPT_MODE_HOST
    tuh_int_handler(0);
  #endif

  #if CFG_TUSB_RHPORT0_MODE & OPT_MODE_DEVICE
    tud_int_handler(0);
  #endif
}

void USB1_IRQHandler(void)
{
  #if CFG_TUSB_RHPORT1_MODE & OPT_MODE_HOST
    tuh_int_handler(1);
  #endif

  #if CFG_TUSB_RHPORT1_MODE & OPT_MODE_DEVICE
    tud_int_handler(1);
  #endif
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

// PD_10
#define LED_PORT      6
#define LED_PIN       24

// P4_0
#define BUTTON_PORT   2
#define BUTTON_PIN    0

#define UART_DEV        LPC_USART3
#define UART_PORT       0x02
#define UART_PIN_TX     3
#define UART_PIN_RX     4


/* System configuration variables used by chip driver */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

static const PINMUX_GRP_T pinmuxing[] =
{
  // LEDs
  { 0xD, 10, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4) },
  { 0xD, 11, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
  { 0xD, 12, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
  { 0xD, 13, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
  { 0xD, 14, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC4 | SCU_MODE_PULLDOWN) },
  { 0x9,  0, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLDOWN) },
  { 0x9,  1, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLDOWN) },
  { 0x9,  2, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLDOWN) },

  // Button
  { 0x4, 0, (SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_PULLUP) },

  // UART
  { UART_PORT, UART_PIN_TX, SCU_MODE_PULLDOWN | SCU_MODE_FUNC2 },
  { UART_PORT, UART_PIN_RX, SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2 },

  // USB0
  { 0x6, 3, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC1 },		                // P6_3 USB0_PWR_EN, USB0 VBus function

  { 0x9, 5, SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2 },			              // P9_5 USB1_VBUS_EN, USB1 VBus function
  { 0x2, 5, SCU_MODE_INACT  | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC2 }, // P2_5 USB1_VBUS, MUST CONFIGURE THIS SIGNAL FOR USB1 NORMAL OPERATION
};

/* Pin clock mux values, re-used structure, value in first index is meaningless */
static const PINMUX_GRP_T pinclockmuxing[] =
{
	{ 0, 0,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
	{ 0, 1,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
	{ 0, 2,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
	{ 0, 3,  (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_FUNC0)},
};

// Invoked by startup code
void SystemInit(void)
{
#ifdef __USE_LPCOPEN
	extern void (* const g_pfnVectors[])(void);
  unsigned int *pSCB_VTOR = (unsigned int *) 0xE000ED08;
	*pSCB_VTOR = (unsigned int) g_pfnVectors;
#endif

  /* Setup system level pin muxing */
  Chip_SCU_SetPinMuxing(pinmuxing, sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));

  /* Clock pins only, group field not used */
  for (uint32_t i = 0; i < (sizeof(pinclockmuxing) / sizeof(pinclockmuxing[0])); i++)
  {
    Chip_SCU_ClockPinMuxSet(pinclockmuxing[i].pinnum, pinclockmuxing[i].modefunc);
  }

  Chip_SetupXtalClocking();
}

void board_init(void)
{
  SystemCoreClockUpdate();

#if CFG_TUSB_OS == OPT_OS_NONE
  // 1ms tick timer
  SysTick_Config(SystemCoreClock / 1000);
#elif CFG_TUSB_OS == OPT_OS_FREERTOS
  // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
  //NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY );
#endif

  Chip_GPIO_Init(LPC_GPIO_PORT);

  // LED
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, LED_PORT, LED_PIN);

  // Button
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, BUTTON_PORT, BUTTON_PIN);

  //------------- UART -------------//
  Chip_UART_Init(UART_DEV);
  Chip_UART_SetBaud(UART_DEV, CFG_BOARD_UART_BAUDRATE);
  Chip_UART_ConfigData(UART_DEV, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
  Chip_UART_TXEnable(UART_DEV);

  //------------- USB -------------//
#if CFG_TUSB_RHPORT0_MODE
  Chip_USB0_Init();
#endif

#if CFG_TUSB_RHPORT1_MODE
  Chip_USB1_Init();
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
  Chip_GPIO_SetPinState(LPC_GPIO_PORT, LED_PORT, LED_PIN, state);
}

uint32_t board_button_read(void)
{
  // active low
  return Chip_GPIO_GetPinState(LPC_GPIO_PORT, BUTTON_PORT, BUTTON_PIN) ? 0 : 1;
}

int board_uart_read(uint8_t* buf, int len)
{
  //return UART_ReceiveByte(BOARD_UART_PORT);
  (void) buf; (void) len;
  return 0;
}

int board_uart_write(void const * buf, int len)
{
  uint8_t const* buf8 = (uint8_t const*) buf;
  for(int i=0; i<len; i++)
  {
    while ((Chip_UART_ReadLineStatus(UART_DEV) & UART_LSR_THRE) == 0) {}
    Chip_UART_SendByte(UART_DEV, buf8[i]);
  }

  return len;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;
void SysTick_Handler (void)
{
  system_ticks++;
}

uint32_t board_millis(void)
{
  return system_ticks;
}
#endif

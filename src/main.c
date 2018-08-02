#include <stdio.h>
#include "stm32f0xx.h"

#define NRF24_RF_CHANNEL        69

#define ANALOG_MODE_FOR_ALL_PINS 0xFFFFFFFF
#define PIN_CONF(PIN, MODE)     ((uint32_t)MODE << (PIN * 2))
#define PIN_AF(PIN, AF)         ((uint32_t)AF << (4 * (PIN & 7)))

#define PIN(PIN_NO)             (PIN_NO)
#define AF(PIN_NO)              (PIN_NO)

#define PIN_MODE_INPUT          0
#define PIN_MODE_OUTPUT         1
#define PIN_MODE_ALT_FUNC       2
#define PIN_MODE_ANALOG         3

#define PINV_INPUT              3
#define PINV_OUTPUT             2
#define PINV_ALT_FUNC           1
#define PINV_ANALOG             0

#define PIN_SPEED_LOW           0
#define PIN_SPEED_MEDIUM        1
#define PIN_SPEED_HIGH          3

#define PIN_PULL_NO             0
#define PIN_PULL_UP             1
#define PIN_PULL_DOWN           2


void __STATIC_INLINE initUSART1(uint32_t pclk, uint32_t baudrate) {

  USART1->BRR = (pclk + baudrate / 2) / baudrate;
  USART1->CR1 = USART_CR1_TE | USART_CR1_UE; 

  // polling idle frame Transmission
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC) {
    // timeout handler follows here
  }
  USART1->ICR |= USART_ICR_TCCF;    // clear TC flag
}

void __STATIC_INLINE uputc(uint8_t c) {
    // polling idle frame Transmission
  while((USART1->ISR & USART_ISR_TXE) != USART_ISR_TXE) {
    // timeout handler follows here
  }
  USART1->ICR |= USART_ICR_TCCF;    // clear TC flag
  USART1->TDR = c;                  // send char
}

void __STATIC_INLINE uputs(char *s) {
  while (*s != 0) uputc(*s++);
}

void __STATIC_INLINE uputx(uint8_t c) {
  uputc(c + ((c < 10) ? '0' : 'A' - 10));
}

#define uprintf(...) for(char _b[100]; snprintf(_b, sizeof(_b), __VA_ARGS__), uputs(_b), 0;){}

void __STATIC_INLINE initSPI1(void) {

  SPI1->CR2 = (      // SPI Control register 2
    0 * SPI_CR2_RXDMAEN          |    // Rx Buffer DMA Enable
    0 * SPI_CR2_TXDMAEN          |    // Tx Buffer DMA Enable
    0 * SPI_CR2_SSOE             |    // SS Output Enable
    0 * SPI_CR2_NSSP             |    // NSS pulse management Enable
    0 * SPI_CR2_FRF              |    // Frame Format Enable
    0 * SPI_CR2_ERRIE            |    // Error Interrupt Enable
    0 * SPI_CR2_RXNEIE           |    // RX buffer Not Empty Interrupt Enable
    0 * SPI_CR2_TXEIE            |    // Tx buffer Empty Interrupt Enable
      // DS[3:0] Data Size
      // These bits configure the data length for SPI transfers:
      //    0000: Not used
      //    0001: Not used
      //    0010: Not used
      //    0011: 4-bit
      //    0100: 5-bit
      //    0101: 6-bit
      //    0110: 7-bit
      //    0111: 8-bit
      //    1000: 9-bit
      //    1001: 10-bit
      //    1010: 11-bit
      //    1011: 12-bit
      //    1100: 13-bit
      //    1101: 14-bit
      //    1110: 15-bit
      //    1111: 16-bit
    1 * SPI_CR2_DS_0             |    // Data length bit 0
    1 * SPI_CR2_DS_1             |    // Data length Bit 1
    1 * SPI_CR2_DS_2             |    // Data length Bit 2
    0 * SPI_CR2_DS_3             |    // Data length Bit 3
    1 * SPI_CR2_FRXTH            |    // FIFO reception Threshold
    0 * SPI_CR2_LDMARX           |    // Last DMA transfer for reception
    0 * SPI_CR2_LDMATX               // Last DMA transfer for transmission
  );

  SPI1->CR1 = (       // SPI Control register 1 (not used in I2S mode)
    0 * SPI_CR1_CPHA             |    // Clock Phase
    0 * SPI_CR1_CPOL             |    // Clock Polarity
    1 * SPI_CR1_MSTR             |    // Master Selection
      // BR[2:0] bits (Baud Rate Control)
      // 000: fPCLK/2
      // 001: fPCLK/4
      // 010: fPCLK/8
      // 011: fPCLK/16
      // 100: fPCLK/32
      // 101: fPCLK/64
      // 110: fPCLK/128
      // 111: fPCLK/256
    0 * SPI_CR1_BR_0             |      // Baud Rate Control bit 0
    0 * SPI_CR1_BR_1             |      // Baud Rate Control bit 1
    0 * SPI_CR1_BR_2             |      // Baud Rate Control bit 2
    1 * SPI_CR1_SPE              |      // SPI Enable
    0 * SPI_CR1_LSBFIRST         |      // Frame Format
    1 * SPI_CR1_SSI              |      // Internal slave select
    1 * SPI_CR1_SSM              |      // Software slave management
    0 * SPI_CR1_RXONLY           |      // Receive only
    0 * SPI_CR1_CRCL             |      // CRC Length
    0 * SPI_CR1_CRCNEXT          |      // Transmit CRC next
    0 * SPI_CR1_CRCEN            |      // Hardware CRC calculation enable
    0 * SPI_CR1_BIDIOE           |      // Output enable in bidirectional mode
    0 * SPI_CR1_BIDIMODE                // Bidirectional data mode enable
  );
}

#define R_REGISTER                0x00
#define W_REGISTER                0x20
#define REGISTER_MASK             0x1F
#define NRF24_R_RX_PL_WID         0x60
#define NRF24_R_RX_PAYLOAD        0x61
#define FLUSH_TX                  0xE1
#define FLUSH_RX                  0xE2
#define NRF24_NOP                 0xFF
  
#define NRF24_CONFIG_REG          0x00
#define NRF24_EN_AA_REG           0x01
#define NRF24_SETUP_AW_REG        0x03
#define NRF24_RF_CH_REG           0x05
#define NRF24_RF_SETUP_REG        0x06
#define NRF24_STATUS_REG          0x07
#define NRF24_FIFO_STATUS_REG     0x17
#define NRF24_DYNPD_REG	          0x1C
#define NRF24_FEATURE_REG	        0x1D

// 00: CONFIG register bits from 0 t0 6, bit 7 must be set to 0
// ============================================================
#define NRF24_CONFIG_PRIM_RX      (1 << 0)
#define NRF24_CONFIG_PWR_UP       (1 << 1)
#define NRF24_CONFIG_CRCO         (1 << 2)
#define NRF24_CONFIG_EN_CRC       (1 << 3)
#define NRF24_CONFIG_MAX_RT       (1 << 4)
#define NRF24_CONFIG_TX_DS        (1 << 5)
#define NRF24_CONFIG_RX_DR        (1 << 6)

// 03: SETUP_AW -- Setup of Address Widths (bits from 0 to 1)
// ==========================================================
//    RX/TX Address field width
//            '00' - Illegal
//            '01' - 3 bytes
//            '10' - 4 bytes
//            '11' – 5 bytes
// LSByte is used if address width is below 5 bytes
//
#define NRF24_SETUP_AW_0          (1 << 0)
#define NRF24_SETUP_AW_1          (1 << 1)

// 06: RF_SETUP -- RF Setup Register (bits from 0 to 6)
// ====================================================
//  Set RF output power in TX mode
//  '00' – -18dBm
//  '01' – -12dBm
//  '10' – -6dBm
//  '11' – 0dBm

#define NRF24_RF_SETUP_RF_PWR_0   (1 << 0)
#define NRF24_RF_SETUP_RF_PWR_1   (1 << 1)
//  Select between the high speed data rates. This bit
//  is don’t care if RF_DR_LOW is set.
//  Encoding:
//  [RF_DR_LOW, RF_DR_HIGH]:
//  ‘00’ – 1Mbps
//  ‘01’ – 2Mbps
//  ‘10’ – 250kbps
//  ‘11’ – Reserved
//
#define NRF24_RF_SETUP_RF_DR_HIGH (1 << 3)
#define NRF24_RF_SETUP_RF_DR_LOW  (1 << 5)

#define NRF24_FIFO_STATUS_RX_EMPTY      (1 << 0)

// 07: STATUS -- Status Register (In parallel to the SPI command
//               word applied on the MOSI pin, the STATUS register
//               is shifted serially out on the MISO pin)   (bits from 0 to 6)
// ===============================================================
#define NRF24_STATUS_MAX_RT       (1 << 4)
#define NRF24_STATUS_TX_DS        (1 << 5)
#define NRF24_STATUS_RX_DR        (1 << 6)

// 1D: FEATURE -- Feature Register (bits from 0 to 2)
// ==================================================
#define NRF24_FEATURE_EN_DYN_ACK  (1 << 0)
#define NRF24_FEATURE_EN_ACK_PAY  (1 << 1)
#define NRF24_FEATURE_EN_DPL      (1 << 2)

#define SET_HIGH                  GPIO_BSRR_BS_
#define SET_LOW                   GPIO_BSRR_BR_

#define GLUE2(A, B)               A##B
#define GLUE(A, B)                GLUE2(A, B)

#define CSN(STATE)                GPIOA->BSRR = GLUE(STATE, 3)
#define CE(STATE)                 GPIOA->BSRR = GLUE(STATE, 2)

uint8_t __STATIC_INLINE spi(uint8_t data) {
  while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);
  *(uint8_t*)&SPI1->DR = data;
  while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
  return (uint8_t)SPI1->DR;
}

uint8_t __STATIC_INLINE nrf_read_register(uint8_t regNo) {
  CSN(SET_LOW);
  spi(R_REGISTER | (REGISTER_MASK & regNo));
  uint8_t r = spi(NRF24_NOP);
  CSN(SET_HIGH);
  return r;
}

uint8_t __STATIC_INLINE nrf_write_register(uint8_t regNo, uint8_t regVal) {
  CSN(SET_LOW);
  uint8_t s = spi(W_REGISTER | (REGISTER_MASK & regNo));
  spi(regVal);
  CSN(SET_HIGH);
  return s;
}

uint8_t __STATIC_INLINE nrf_write_cmd(uint8_t cmd) {
  CSN(SET_LOW);
  uint8_t s = spi(cmd);
  CSN(SET_HIGH);
  return s;
}

int main() {

  uint8_t _buf[32];

  RCC->AHBENR = (           // RCC AHB peripheral clock enable register
    0 * RCC_AHBENR_DMAEN         |  // DMA1 clock enable
    1 * RCC_AHBENR_SRAMEN        |  // SRAM interface clock enable (reset state = 1)
    1 * RCC_AHBENR_FLITFEN       |  // FLITF clock enable          (reset state = 1)
    0 * RCC_AHBENR_CRCEN         |  // CRC clock enable
    1 * RCC_AHBENR_GPIOAEN       |  // GPIOA clock enable
    0 * RCC_AHBENR_GPIOBEN       |  // GPIOB clock enable
    0 * RCC_AHBENR_GPIOCEN       |  // GPIOC clock enable
    0 * RCC_AHBENR_GPIOFEN          // GPIOF clock enable
  );

  RCC->APB2ENR = (          // RCC APB2 peripheral clock enable register
    0 * RCC_APB2ENR_SYSCFGCOMPEN |  // SYSCFG and comparator clock enable
    0 * RCC_APB2ENR_ADCEN        |  // ADC1 clock enable
    0 * RCC_APB2ENR_TIM1EN       |  // TIM1 clock enable
    1 * RCC_APB2ENR_SPI1EN       |  // SPI1 clock enable
    1 * RCC_APB2ENR_USART1EN     |  // USART1 clock enable
    0 * RCC_APB2ENR_TIM16EN      |  // TIM16 clock enable
    0 * RCC_APB2ENR_TIM17EN      |  // TIM17 clock enable
    0 * RCC_APB2ENR_DBGMCUEN        // DBGMCU clock enable
  );

  RCC->APB1ENR = (          // RCC APB1 peripheral clock enable register
    0 * RCC_APB1ENR_TIM2EN       |  // Timer 2 clock enable
    0 * RCC_APB1ENR_TIM3EN       |  // Timer 3 clock enable
    0 * RCC_APB1ENR_TIM14EN      |  // Timer 14 clock enable
    0 * RCC_APB1ENR_WWDGEN       |  // Window Watchdog clock enable
    0 * RCC_APB1ENR_I2C1EN       |  // I2C1 clock enable
    0 * RCC_APB1ENR_PWREN           // PWR clock enable
  );

  GPIOA->PUPDR = GPIO_PUPDR_PUPDR4_0 << GPIO_PUPDR_PUPDR4_Pos; // Set pullup mode for PA4
  GPIOA->ODR = GPIO_ODR_3;                                     // prepare PA3 (NRF24_CSN) to set HIGH

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( // Configure GPIOA
#ifndef SWD_DISABLED
    PIN_CONF(PIN(14), PINV_ALT_FUNC)  |  // PA14 AF0 -- SYS_SWDCLK
    PIN_CONF(PIN(13), PINV_ALT_FUNC)  |  // PA13 AF0 -- SYS_SWDIO
#endif
    PIN_CONF(PIN(10), PINV_ALT_FUNC)  |  // PA10 AF1 -- USART1 RX
    PIN_CONF(PIN(9),  PINV_ALT_FUNC)  |  // PA9  AF1 -- USART1 TX
    PIN_CONF(PIN(7),  PINV_ALT_FUNC)  |  // PA7  AF0 -- SPI1_MOSI
    PIN_CONF(PIN(6),  PINV_ALT_FUNC)  |  // PA6  AF0 -- SPI1_MISO
    PIN_CONF(PIN(5),  PINV_ALT_FUNC)  |  // PA5  AF0 -- SPI1_SCK
    PIN_CONF(PIN(4),  PINV_INPUT)     |  // PA4  IN  -- NRF24_IRQ (PullUp)
    PIN_CONF(PIN(3),  PINV_OUTPUT)    |  // PA3  OUT -- NRF24_CSN (set HIGH)
    PIN_CONF(PIN(2),  PINV_OUTPUT)       // PA2  OUT -- NRF24_CE  (set LOW)
  );

  GPIOA->AFR[1] = (     // with High 8 bits of AFR
    PIN_AF(PIN(10), AF(1))            |  // PA10 AF1  -- USART1 RX
    PIN_AF(PIN(9),  AF(1))               // PA9  AF1  -- USART1 TX
  );

  RCC->CFGR = RCC_CFGR_HPRE_DIV4;   /* set 2mhz core clock */

  initSPI1();
  initUSART1(2000000, 115200);

  uputs("Init done.\n");

  nrf_write_register(NRF24_SETUP_AW_REG,   //    RX/TX Address field width
                                 //            '00' - Illegal
                                 //            '01' - 3 bytes
                                 //            '10' - 4 bytes
                                 //            '11' – 5 bytes
                                 // LSByte is used if address width is below 5 bytes
     1 * NRF24_SETUP_AW_0        |
     0 * NRF24_SETUP_AW_1
  );

  nrf_write_register(NRF24_EN_AA_REG, 0);  // disable 'Auto Acknowledgment'

  nrf_write_register(NRF24_RF_CH_REG, NRF24_RF_CHANNEL); // set radio channel

  nrf_write_register(NRF24_RF_SETUP_REG,   // set radio channel
      //  Set RF output power in TX mode
      //  '00' – -18dBm
      //  '01' – -12dBm
      //  '10' – -6dBm
      //  '11' – 0dBm
    1 * NRF24_RF_SETUP_RF_PWR_0  |
    1 * NRF24_RF_SETUP_RF_PWR_1  |
      //  Select between the high speed data rates. This bit
      //  is don’t care if RF_DR_LOW is set.
      //  Encoding:
      //  [RF_DR_LOW, RF_DR_HIGH]:
      //  ‘00’ – 1Mbps
      //  ‘01’ – 2Mbps
      //  ‘10’ – 250kbps
      //  ‘11’ – Reserved
    1 * NRF24_RF_SETUP_RF_DR_HIGH |
    0 * NRF24_RF_SETUP_RF_DR_LOW
  );

  nrf_write_register(NRF24_FEATURE_REG,    // set payload parameters
    1 * NRF24_FEATURE_EN_DYN_ACK  |  // enable W_TX_PAYLOAD_NOACK command
    1 * NRF24_FEATURE_EN_DPL         // enable dynamic payload
  );

  nrf_write_register(NRF24_DYNPD_REG, 1);  // Enable dynamic payload for pipe 1

  nrf_write_register(NRF24_STATUS_REG,     // clear status bits
    1 * NRF24_STATUS_MAX_RT       |
    1 * NRF24_STATUS_TX_DS        |
    1 * NRF24_STATUS_RX_DR
  );

  nrf_write_cmd(FLUSH_RX);           // Flush RX FIFO
  nrf_write_cmd(FLUSH_TX);           // Flush TX FIFO

  nrf_write_register(NRF24_CONFIG_REG,     // Power transiever up
    1 * NRF24_CONFIG_PRIM_RX      |  // Mode control: 0 - TX, 1 - RX;
    1 * NRF24_CONFIG_PWR_UP       |  // Power control: 0 - OFF, 1 - ON;
    1 * NRF24_CONFIG_EN_CRC       |  // CRC control: 0 - disable, 1 - enable;
    1 * NRF24_CONFIG_CRCO         |  // CRC encoding scheme: 0 - 1 byte, 1 – 2 bytes
    1 * NRF24_CONFIG_MAX_RT       |  // MAX_RT interrupt reflection on the IRQ pin: 0 - enable, 1 - disable;
    1 * NRF24_CONFIG_TX_DS        |  // TX_DS interrupt reflection on the IRQ pin: 0 - enable, 1 - disable;
    0 * NRF24_CONFIG_RX_DR           // RX_DR interrupt reflection on the IRQ pin: 0 - enable, 1 - disable;
  );

  CE(SET_HIGH);                      // Start receiving

  RCC->APB2ENR = 0;                  // Disabe SPI1 and USART1 peripherials

  for (;;) {

    RCC->CFGR = RCC_CFGR_HPRE_DIV512;   /* set 2mhz core clock */

    while ((GPIOA->IDR & GPIO_IDR_4) != 0) {
      // reset watchdog follows here
    }

    RCC->CFGR = RCC_CFGR_HPRE_DIV4;   /* set 2mhz core clock */

    RCC->APB2ENR = (          // Enable SPI1 and USART1 peripherials
      1 * RCC_APB2ENR_SPI1EN       |  // SPI1 clock enable
      1 * RCC_APB2ENR_USART1EN        // USART1 clock enable
    );

    while ((nrf_read_register(NRF24_FIFO_STATUS_REG) & NRF24_FIFO_STATUS_RX_EMPTY) == 0) {

      CSN(SET_LOW);
      spi(NRF24_R_RX_PL_WID);  // 0x60 = "Read RX Payload Width" command
      uint8_t pSize = spi(NRF24_NOP);
      CSN(SET_HIGH);

      if (pSize > 32) {          // Payload size cannot be more than 32 bytes
        uputc('*');
        nrf_write_cmd(FLUSH_RX); // 0xE2 = clear RX FIFO
        break;                   // Leave loop
      }

      CSN(SET_LOW);
      spi(NRF24_R_RX_PAYLOAD);  // 0x61 = "Read RX Payload" CMD
      for (int i = 0; i < pSize; i++) {
       _buf[i] = spi(NRF24_NOP);
       uputx(_buf[i] >> 4);
       uputx(_buf[i] & 0x0F);
      }
      CSN(SET_HIGH);
      uputc(10);
    }

    while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC) {
    // timeout handler follows here
    }
    USART1->ICR |= USART_ICR_TCCF;    // clear TC flag

    nrf_write_register(NRF24_STATUS_REG, NRF24_STATUS_RX_DR); // clear RX_DR bit, release IRQ line
    RCC->APB2ENR = 0; // Disabe SPI1 and USART1 peripherials

  }
}

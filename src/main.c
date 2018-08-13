#include <stdio.h>
#include <inttypes.h>
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

#define IWDG_REFRESH            0xAAAAUL
#define IWDG_WRITE_ACCESS       0x5555UL
#define IWDG_START              0xCCCCUL
#define IWDG_RELOAD             1000

void __STATIC_INLINE initIWDG(void) {

  /* Enable the LSI, wait while it is not ready */
  RCC->CSR = RCC_CSR_LSION;
  while((RCC->CSR & RCC_CSR_LSIRDY) != RCC_CSR_LSIRDY) {
    /* Timeout handler follows here */
  }

  IWDG->KR = IWDG_START;        /* Activate IWDG            */
  IWDG->KR = IWDG_WRITE_ACCESS; /* Enable write access      */
  IWDG->PR = IWDG_PR_PR_0;      /* Set prescaler by 8       */
  IWDG->RLR = IWDG_RELOAD;      /* Set reload value         */
  while(IWDG->SR) {             /* Check if flags are reset */
    /* Timeout handler follows here */
  }
  IWDG->KR = IWDG_REFRESH;      /* Refresh counter          */
}

void __STATIC_INLINE initUSART1(uint32_t pclk, uint32_t baudrate) {

  USART1->BRR = (pclk + baudrate / 2) / baudrate;
  USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    /* Polling idle frame Transmission */
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC) {
    /* Timeout handler follows here    */
  }
  USART1->ICR = USART_ICR_TCCF;    /* Сlear TC flag */
}

  /* Send a char via UART */
void __STATIC_INLINE uputc(uint8_t c) {
    /* Polling idle frame Transmission */
  while((USART1->ISR & USART_ISR_TXE) != USART_ISR_TXE) {
    /* Timeout handler follows here */
  }
  USART1->ICR = USART_ICR_TCCF;    /* Сlear TC flag */
  USART1->TDR = c;                 /* Send char     */
}

  /* Send a string via UART */
void __STATIC_INLINE uputs(char *s) {
  while (*s != 0) uputc(*s++);     /* Send string char by char */
}

  /* Convert a nibble to HEX and send it via UART */
void __STATIC_INLINE uputx(uint8_t c) {
  /* Convert a nibble to HEX char and send it via UART */
  uputc(c + ((c < 10) ? '0' : 'A' - 10)); 
}

  /* Convert a byte to HEX and send it via UART */
void __STATIC_INLINE u_print_hex(uint8_t c) {
  uputx(c >> 4);
  uputx(c & 0x0F);
}
  /* MACRO to emulate printf() via UART */
#define uprintf(...) for(char _b[100]; snprintf(_b, sizeof(_b), __VA_ARGS__), uputs(_b), 0;){}

void __STATIC_INLINE initSPI1(void) {

  SPI1->CR2 = (      /* SPI Control register 2                                */
    0 * SPI_CR2_RXDMAEN          |    /* Rx Buffer DMA Enable                 */
    0 * SPI_CR2_TXDMAEN          |    /* Tx Buffer DMA Enable                 */
    0 * SPI_CR2_SSOE             |    /* SS Output Enable                     */
    0 * SPI_CR2_NSSP             |    /* NSS pulse management Enable          */
    0 * SPI_CR2_FRF              |    /* Frame Format Enable                  */
    0 * SPI_CR2_ERRIE            |    /* Error Interrupt Enable               */
    0 * SPI_CR2_RXNEIE           |    /* RX buffer Not Empty Interrupt Enable */
    0 * SPI_CR2_TXEIE            |    /* Tx buffer Empty Interrupt Enable     */
      /* DS[3:0] Data Size                                                    */
      /* These bits configure the data length for SPI transfers:              */
      /*    0000: Not used                                                    */
      /*    0001: Not used                                                    */
      /*    0010: Not used                                                    */
      /*    0011: 4-bit                                                       */
      /*    0100: 5-bit                                                       */
      /*    0101: 6-bit                                                       */
      /*    0110: 7-bit                                                       */
      /*    0111: 8-bit                                                       */
      /*    1000: 9-bit                                                       */
      /*    1001: 10-bit                                                      */
      /*    1010: 11-bit                                                      */
      /*    1011: 12-bit                                                      */
      /*    1100: 13-bit                                                      */
      /*    1101: 14-bit                                                      */
      /*    1110: 15-bit                                                      */
      /*    1111: 16-bit                                                      */
    1 * SPI_CR2_DS_0             |    /* Data length bit 0                    */
    1 * SPI_CR2_DS_1             |    /* Data length Bit 1                    */
    1 * SPI_CR2_DS_2             |    /* Data length Bit 2                    */
    0 * SPI_CR2_DS_3             |    /* Data length Bit 3                    */
    1 * SPI_CR2_FRXTH            |    /* FIFO reception Threshold             */
    0 * SPI_CR2_LDMARX           |    /* Last DMA transfer for reception      */
    0 * SPI_CR2_LDMATX                /* Last DMA transfer for transmission   */
  );

  SPI1->CR1 = (       /* SPI Control register 1 (not used in I2S mode)        */
    0 * SPI_CR1_CPHA             |    /* Clock Phase                          */
    0 * SPI_CR1_CPOL             |    /* Clock Polarity                       */
    1 * SPI_CR1_MSTR             |    /* Master Selection                     */
      /* BR[2:0] bits (Baud Rate Control)                                     */
      /* 000: fPCLK/2                                                         */
      /* 001: fPCLK/4                                                         */
      /* 010: fPCLK/8                                                         */
      /* 011: fPCLK/16                                                        */
      /* 100: fPCLK/32                                                        */
      /* 101: fPCLK/64                                                        */
      /* 110: fPCLK/128                                                       */
      /* 111: fPCLK/256                                                       */
    0 * SPI_CR1_BR_0             |    /* Baud Rate Control bit 0              */
    0 * SPI_CR1_BR_1             |    /* Baud Rate Control bit 1              */
    0 * SPI_CR1_BR_2             |    /* Baud Rate Control bit 2              */
    1 * SPI_CR1_SPE              |    /* SPI Enable                           */
    0 * SPI_CR1_LSBFIRST         |    /* Frame Format                         */
    1 * SPI_CR1_SSI              |    /* Internal slave select                */
    1 * SPI_CR1_SSM              |    /* Software slave management            */
    0 * SPI_CR1_RXONLY           |    /* Receive only                         */
    0 * SPI_CR1_CRCL             |    /* CRC Length                           */
    0 * SPI_CR1_CRCNEXT          |    /* Transmit CRC next                    */
    0 * SPI_CR1_CRCEN            |    /* Hardware CRC calculation enable      */
    0 * SPI_CR1_BIDIOE           |    /* Output enable in bidirectional mode  */
    0 * SPI_CR1_BIDIMODE              /* Bidirectional data mode enable       */
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
#define NRF24_STATUS_REG          0x07
#define NRF24_FIFO_STATUS_REG     0x17
#define NRF24_DYNPD_REG           0x1C
#define NRF24_FEATURE_REG         0x1D

/* 00: CONFIG register bits from 0 t0 6, bit 7 must be set to 0               */
/* ============================================================               */
#define NRF24_CONFIG_PRIM_RX      (1 << 0)
#define NRF24_CONFIG_PWR_UP       (1 << 1)
#define NRF24_CONFIG_CRCO         (1 << 2)
#define NRF24_CONFIG_EN_CRC       (1 << 3)
#define NRF24_CONFIG_MAX_RT       (1 << 4)
#define NRF24_CONFIG_TX_DS        (1 << 5)
#define NRF24_CONFIG_RX_DR        (1 << 6)

/* 07: STATUS -- Status Register                                              */
/* ===============================================================            */
#define NRF24_STATUS_MAX_RT       NRF24_CONFIG_MAX_RT
#define NRF24_STATUS_TX_DS        NRF24_CONFIG_TX_DS 
#define NRF24_STATUS_RX_DR        NRF24_CONFIG_RX_DR 

/* 17: FIFO STATUS register                                                   */
/* ============================================================               */
#define NRF24_FIFO_STATUS_RX_EMPTY      (1 << 0)

/* 1D: FEATURE -- Feature Register (bits from 0 to 2)                         */
/* ==================================================                         */
#define NRF24_FEATURE_EN_DYN_ACK  (1 << 0)
#define NRF24_FEATURE_EN_DPL      (1 << 2)

#define SET_HIGH                  GPIO_BSRR_BS_
#define SET_LOW                   GPIO_BSRR_BR_

#define GLUE2(A, B)               A##B
#define GLUE(A, B)                GLUE2(A, B)

#define CSN(STATE)                GPIOA->BSRR = GLUE(STATE, 3)
#define CE(STATE)                 GPIOA->BSRR = GLUE(STATE, 2)

  /* Semd/Receive a byte via SPI */
uint8_t __STATIC_INLINE spi(uint8_t data) {
  while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE);
  *(uint8_t*)&SPI1->DR = data;
  while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
  return (uint8_t)SPI1->DR;
}
  
  /* Write a byte to the NRF24L01's register */
uint8_t __STATIC_INLINE nrf_read_register(uint8_t regNo) {
  CSN(SET_LOW);
  spi(R_REGISTER | (REGISTER_MASK & regNo));
  uint8_t r = spi(NRF24_NOP);
  CSN(SET_HIGH);
  return r;
}

  /* Read a byte from the NRF24L01's register */
uint8_t __STATIC_INLINE nrf_write_register(uint8_t regNo, uint8_t regVal) {
  CSN(SET_LOW);
  uint8_t s = spi(W_REGISTER | (REGISTER_MASK & regNo));
  spi(regVal);
  CSN(SET_HIGH);
  return s;
}

  /* Send a command to NRF24L01 */
uint8_t __STATIC_INLINE nrf_write_cmd(uint8_t cmd) {
  CSN(SET_LOW);
  uint8_t s = spi(cmd);
  CSN(SET_HIGH);
  return s;
}

int main() {

  RCC->AHBENR = (           /* RCC AHB peripheral clock enable register       */
    0 * RCC_AHBENR_DMAEN         |  /* DMA1 clock enable                      */ 
    1 * RCC_AHBENR_SRAMEN        |  /* SRAM interface clock enable (rst.st=1) */ 
    1 * RCC_AHBENR_FLITFEN       |  /* FLITF clock enable          (rst.st=1) */
    0 * RCC_AHBENR_CRCEN         |  /* CRC clock enable                       */
    1 * RCC_AHBENR_GPIOAEN       |  /* GPIOA clock enable                     */
    0 * RCC_AHBENR_GPIOBEN       |  /* GPIOB clock enable                     */
    0 * RCC_AHBENR_GPIOCEN       |  /* GPIOC clock enable                     */
    0 * RCC_AHBENR_GPIOFEN          /* GPIOF clock enable                     */
  );

  RCC->APB2ENR = (          /* RCC APB2 peripheral clock enable register      */
    0 * RCC_APB2ENR_SYSCFGCOMPEN |  /* SYSCFG and comparator clock enable     */
    0 * RCC_APB2ENR_ADCEN        |  /* ADC1 clock enable                      */
    0 * RCC_APB2ENR_TIM1EN       |  /* TIM1 clock enable                      */
    1 * RCC_APB2ENR_SPI1EN       |  /* SPI1 clock enable                      */
    1 * RCC_APB2ENR_USART1EN     |  /* USART1 clock enable                    */
    0 * RCC_APB2ENR_TIM16EN      |  /* TIM16 clock enable                     */
    0 * RCC_APB2ENR_TIM17EN      |  /* TIM17 clock enable                     */
    0 * RCC_APB2ENR_DBGMCUEN        /* DBGMCU clock enable                    */
  );

  RCC->APB1ENR = (          /* RCC APB1 peripheral clock enable register      */
    0 * RCC_APB1ENR_TIM2EN       |  /* Timer 2 clock enable                   */
    0 * RCC_APB1ENR_TIM3EN       |  /* Timer 3 clock enable                   */
    0 * RCC_APB1ENR_TIM14EN      |  /* Timer 14 clock enable                  */
    0 * RCC_APB1ENR_WWDGEN       |  /* Window Watchdog clock enable           */
    0 * RCC_APB1ENR_I2C1EN       |  /* I2C1 clock enable                      */
    0 * RCC_APB1ENR_PWREN           /* PWR clock enable                       */
  );

  GPIOA->PUPDR = GPIO_PUPDR_PUPDR4_0 << GPIO_PUPDR_PUPDR4_Pos; /* Set pullup mode for PA4              */
  GPIOA->ODR = GPIO_ODR_3;                                     /* prepare PA3 (NRF24_CSN) to set HIGH  */

  GPIOA->MODER = ANALOG_MODE_FOR_ALL_PINS - ( /* Configure GPIOA              */
#ifndef SWD_DISABLED
    PIN_CONF(PIN(14), PINV_ALT_FUNC)  |  /* PA14 AF0 -- SYS_SWDCLK            */
    PIN_CONF(PIN(13), PINV_ALT_FUNC)  |  /* PA13 AF0 -- SYS_SWDIO             */
#endif
    PIN_CONF(PIN(10), PINV_ALT_FUNC)  |  /* PA10 AF1 -- USART1 RX             */
    PIN_CONF(PIN(9),  PINV_ALT_FUNC)  |  /* PA9  AF1 -- USART1 TX             */
    PIN_CONF(PIN(7),  PINV_ALT_FUNC)  |  /* PA7  AF0 -- SPI1_MOSI             */
    PIN_CONF(PIN(6),  PINV_ALT_FUNC)  |  /* PA6  AF0 -- SPI1_MISO             */
    PIN_CONF(PIN(5),  PINV_ALT_FUNC)  |  /* PA5  AF0 -- SPI1_SCK              */
    PIN_CONF(PIN(4),  PINV_INPUT)     |  /* PA4  IN  -- NRF24_IRQ  (PullUp)   */
    PIN_CONF(PIN(3),  PINV_OUTPUT)    |  /* PA3  OUT -- NRF24_CSN  (set HIGH) */
    PIN_CONF(PIN(2),  PINV_OUTPUT)       /* PA2  OUT -- NRF24_CE   (set LOW)  */
  );

  GPIOA->AFR[1] = (     /* with High 8 bits of AFR  */
    PIN_AF(PIN(10), AF(1))            |  /* PA10 AF1 -- USART1 RX             */
    PIN_AF(PIN(9),  AF(1))               /* PA9  AF1 -- USART1 TX             */
  );

  RCC->CFGR = RCC_CFGR_HPRE_DIV4;   /* set core clock speed to 2mhz           */

  initSPI1();                       /* initialize SPI peripherial             */
  initUSART1(2000000, 115200);      /* initialize USART peripherial           */
  initIWDG();                       /* initialize watchdog                    */

    /* Set the address width: 0 = Illegal, 1 = 3 bytes, 2 = 4 bytes, 3 = 5 bytes */
  nrf_write_register(NRF24_SETUP_AW_REG,  1);
    /* Disable 'Auto Acknowledgment' */
  nrf_write_register(NRF24_EN_AA_REG, 0);
    /* Set the channel */
  nrf_write_register(NRF24_RF_CH_REG, NRF24_RF_CHANNEL);
    /* Set options */
  nrf_write_register(NRF24_FEATURE_REG, 
    1 * NRF24_FEATURE_EN_DYN_ACK  |  /* enable W_TX_PAYLOAD_NOACK command     */
    1 * NRF24_FEATURE_EN_DPL         /* enable dynamic payload                */
  );
    /* Enable dynamic payload for pipe 1 */
  nrf_write_register(NRF24_DYNPD_REG, 1);
    /* Clear status bits */
  nrf_write_register(NRF24_STATUS_REG, 
    1 * NRF24_STATUS_MAX_RT       |
    1 * NRF24_STATUS_TX_DS        |
    1 * NRF24_STATUS_RX_DR
  );
    /* Clear RX/TX FIFOs */
  nrf_write_cmd(FLUSH_RX);           /* Flush RX FIFO                         */
  nrf_write_cmd(FLUSH_TX);           /* Flush TX FIFO                         */
    /* Power transeiver ON */
  nrf_write_register(NRF24_CONFIG_REG,
    1 * NRF24_CONFIG_PRIM_RX      |  /* Mode control: 0 - TX, 1 - RX;         */
    1 * NRF24_CONFIG_PWR_UP       |  /* Power control: 0 - OFF, 1 - ON;       */
    1 * NRF24_CONFIG_EN_CRC       |  /* CRC control: 0 - disable, 1 - enable; */
    1 * NRF24_CONFIG_CRCO         |  /* CRC encoding scheme: 0 - 1 byte, 1 â€“ 2 bytes                       */
    1 * NRF24_CONFIG_MAX_RT       |  /* MAX_RT interrupt reflection on the IRQ pin: 0 - enable, 1 - disable; */
    1 * NRF24_CONFIG_TX_DS        |  /* TX_DS interrupt reflection on the IRQ pin: 0 - enable, 1 - disable;  */
    0 * NRF24_CONFIG_RX_DR           /* RX_DR interrupt reflection on the IRQ pin: 0 - enable, 1 - disable;  */
  );

  CE(SET_HIGH);                      /* Start receiving */

  uputs("Init done. Waiting for incoming data...\n");

    /* Wait until UART becomes idle */
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC) {
    /* timeout handler follows here */
  }
  USART1->ICR = USART_ICR_TCCF;      /* Clear TC flag */

  RCC->APB2ENR = 0;                  /* Disable SPI1 and USART1 peripherials  */

  for (;;) { 

    RCC->CFGR = RCC_CFGR_HPRE_DIV512;  /* Slow down the core clock speed      */
      /* Wait until IRQ line becomes LOW */
    while ((GPIOA->IDR & GPIO_IDR_4) != 0) { 
      IWDG->KR = IWDG_REFRESH;         /* Reset watchdog                      */
    }

    RCC->CFGR = RCC_CFGR_HPRE_DIV4;    /* Set the core clock speed to 2mhz    */

    RCC->APB2ENR = (    /* Enable SPI1 and USART1 peripherials                */
      1 * RCC_APB2ENR_SPI1EN       |   /* SPI1 clock enable                   */
      1 * RCC_APB2ENR_USART1EN         /* USART1 clock enable                 */
    );

    do {
                       /*** Get payload size ***/

      CSN(SET_LOW);
      spi(NRF24_R_RX_PL_WID);          /* "Read RX Payload Width" command     */
      uint8_t pSize = spi(NRF24_NOP);  /* Read payload size value             */
      CSN(SET_HIGH);

      IWDG->KR = IWDG_REFRESH;   /* Reset watchdog  */

                     /*** Check for payload size ***/

      if (pSize > 32) {          /* Payload size cannot be more than 32 bytes */
        nrf_write_cmd(FLUSH_RX); /* Clear RX FIFO if paylaod size is wrong    */ 
        continue;                /* Leave loop                                */
      }
      
            /*** Read payload and print it in HEX via UART ***/

      CSN(SET_LOW);
      spi(NRF24_R_RX_PAYLOAD);   /* 0x61 = "Read RX Payload" CMD              */
      for (int i = 0; i < pSize; i++) {
        u_print_hex(spi(NRF24_NOP));  /* send payload bytes in HEX via UART   */
      }
      CSN(SET_HIGH);

      IWDG->KR = IWDG_REFRESH;   /* reset watchdog  */

      uputc(10);                 /* send LF         */

              /*** Leave loop if there is no data in RX FIFO  ***/

    } while ((nrf_read_register(NRF24_FIFO_STATUS_REG) & NRF24_FIFO_STATUS_RX_EMPTY) == 0);

    uputs("\b");                 /* send BackSpace  */

      /* Clear RX_DR bit, release IRQ line */
    nrf_write_register(NRF24_STATUS_REG, NRF24_STATUS_RX_DR); 

    while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC) {
    /* Timeout handler follows here  */
    }
    USART1->ICR = USART_ICR_TCCF;    /* clear TC flag */

    RCC->APB2ENR = 0;  /* Disabe SPI1 and USART1 peripherials */

  }
}

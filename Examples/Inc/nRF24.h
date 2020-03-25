/**
 * 
 * 
 *
 * 
 */

#ifndef __NRF24_H__
#define __NRF24_H__

#include "stm32f1xx_hal.h"
#include "dwt_stm32_delay.h"
extern SPI_HandleTypeDef hspi1;
#define PACKET_SIZE     4

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
    #define nRF_250kBit      0x20   // 250k
    #define nRF_1MBit        0x00   // 1M
    #define nRF_2MBit        0x08   // 2M
    #define nRF_PA_MIN       0x00   // -18 dbm 
    #define nRF_PA_LOW       0x02   // -12 dbm
    #define nRF_PA_HIGH      0x04   // - 6 dbm
    #define nRF_PA_MAX       0x06   //   0 dbm
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define iRF_BANK0_DYNPD         0x1C 
#define iRF_BANK0_FEATURE       0x1D            // 'Feature' register address
#define iRF_BANK0_SETUP_VALUE   0x1E
#define iRF_BANK0_PRE_GURD      0x1F

//  Bit Mnemonics
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0

#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1
#define LNA_HCURR   0
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

// Instruction Mnemonics
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF
#define iRF_CMD_ACTIVATE        0x50
#define iBANK0                  0x00
#define iBANK1                  0x80

#define nrf_ce          GPIOB, GPIO_PIN_1 //  IRQ
#define nrf_csn         GPIOA, GPIO_PIN_4 //  CE
#define nrf_irq         GPIOB, GPIO_PIN_0 //  CSN
//#define nrf_sck         GPIOC, GPIO_PIN_5 //  SCK
//#define nrf_miso        GPIOC, GPIO_PIN_7 //  MISO
//#define nrf_mosi        GPIOC, GPIO_PIN_6 //  MOSI

#define nrf_ce_low  	HAL_GPIO_WritePin(nrf_ce, GPIO_PIN_RESET);//GPIO_WriteLow(nrf_ce) //
#define nrf_ce_high  	HAL_GPIO_WritePin(nrf_ce, GPIO_PIN_SET);//GPIO_WriteHigh(nrf_ce) //
#define nrf_csn_low  	HAL_GPIO_WritePin(nrf_csn, GPIO_PIN_RESET);//GPIO_WriteLow(nrf_csn) //
#define nrf_csn_high  	HAL_GPIO_WritePin(nrf_csn, GPIO_PIN_SET);//GPIO_WriteHigh(nrf_csn) //

typedef struct {
	unsigned char config;
	unsigned char status;
	unsigned char fifo_status;
	unsigned char plos_cnt;
	unsigned char en_rxaddr;
	unsigned char en_aa;
	unsigned char rx_dr;
	unsigned char tx_ds;
	unsigned char max_rt;
	unsigned char rx_empty;
	unsigned char rx_full;
	unsigned char rf_setup;
	unsigned char tx_empty;
	unsigned char tx_full;
	unsigned char rf_ch;
	unsigned char observe_tx;
	unsigned char setup_retr;
	unsigned char check_addr[5];
	unsigned char tx_addr[5];
	unsigned char rx_data[PACKET_SIZE];
} nrf24l01_t;

void TX_POWERUP ( void );
void RX_POWERUP ( void );
void POWERUP ( void );
void POWERDOWN(void);
void RX_PAYLOAD_SIZE(unsigned char size);
unsigned char SPI_RW(unsigned char byte);
unsigned char SPI_Read_Reg(unsigned char reg);
unsigned char SPI_Write_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_Write_Cmd(unsigned char reg, unsigned char value);
unsigned char SPI_Write_Packet(unsigned char reg, unsigned char *pBuf, unsigned char data_size);
unsigned char SPI_Read_Packet(unsigned char reg, unsigned char *pBuf, unsigned char data_size);
void prx(unsigned char size);
void ptx(void);
void Send_Packet( unsigned char *pBuf1 );
void nrf_init(void);





#endif // __NRF24_H__

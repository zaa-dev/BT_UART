#include "nRF24.h"
/*
__STATIC_INLINE void delay_us(__IO uint32_t micros)

{

        micros *=(SystemCoreClock / 1000000) / 5;

        while (micros--);

}
*/

nrf24l01_t nrf;


void TX_POWERUP(void)
{
  register unsigned char tmp = 0;
  nrf_ce_low;
  SPI_Write_Reg(CONFIG, tmp | ((1 << PWR_UP) | (0 << PRIM_RX) | (1 << EN_CRC)));
}

void RX_POWERUP(void)
{
  register unsigned char tmp = 0;
  nrf_ce_low;
  SPI_Write_Reg(CONFIG, tmp | ((1 << PWR_UP) |(1 << PRIM_RX) | (1 << EN_CRC)));
}

void POWERUP(void)
{
  register unsigned char tmp = 0;
  SPI_Write_Reg(CONFIG, tmp | ((1 << PWR_UP) | (1 << EN_CRC) ));
}

void POWERDOWN(void)
{
  register unsigned char tmp = 0;
  SPI_Write_Reg(CONFIG, tmp | ((0 << PWR_UP) | (1 << EN_CRC) ));
}


/*Размер принимаемого пакета
Значения size: 0 - Пайп не используется
               1 = 1 байт
               ...
               32 = 32 байта
*/

void RX_PAYLOAD_SIZE(unsigned char size)
{
  register unsigned char tmp = 0;
  SPI_Write_Reg(RX_PW_P0, tmp | (size));
}

/**************************************************
Функция: SPI_RW();
Описание:
Пишет один байт на nRF24L01 и возвращает байт прочитанный из nRF24L01 в течение записи,
согласно протоколу SPI
 **************************************************/
unsigned char SPI_RW(unsigned char byte)
{
	uint8_t dt = 0;
  HAL_SPI_TransmitReceive(&hspi1,(uint8_t *) &byte,(uint8_t *)&dt,1,HAL_MAX_DELAY);  
  return (dt);
}

/**************************************************
Функция: SPI_Read_Reg();
Описание:
  Читает значение регистра reg
 ************************************************/
unsigned char SPI_Read_Reg(unsigned char reg)
{
  unsigned char nrf_register;
  nrf_csn_low;                  // CSN low, init SPI transaction
  SPI_RW(reg);                  // select register
  nrf_register = SPI_RW(NOP);   // ..and write dummy to it..
  nrf_csn_high;                 // CSN high again
  return (nrf_register);        // return nRF24L01 register byte
}
//*********************************************************
unsigned char SPI_Write_Reg(unsigned char reg, unsigned char value)
{
  unsigned char nrf_status;
  nrf_csn_low;          // CSN low, init SPI transaction
  nrf_status = SPI_RW(W_REGISTER | reg); // select register
  SPI_RW(value);        // ..and write value to it..
  nrf_csn_high;         // CSN high again
  return (nrf_status);  // return nRF24L01 status byte
}

unsigned char SPI_Write_Cmd(unsigned char reg, unsigned char value)
{
  unsigned char nrf_status;
  nrf_csn_low;          // CSN low, init SPI transaction
  nrf_status = SPI_RW(reg); // select register
  SPI_RW(value);        // ..and write value to it..
  nrf_csn_high;         // CSN high again
  return (nrf_status);  // return nRF24L01 status byte
}
//*********************************************************
unsigned char SPI_Write_Packet(unsigned char reg, unsigned char *pBuf, unsigned char data_size)
{
  unsigned char nrf_status;
	unsigned char i_tx;
  //unsigned char byte_num;
  nrf_csn_low;          // CSN low, init SPI transaction
  nrf_status = SPI_RW(W_REGISTER | reg); // select register
  for (i_tx = 0; i_tx < data_size; i_tx++)
      {
        //x_data[i_tx] = *pBuf;
        SPI_RW(*pBuf++);
      }
  nrf_csn_high;         // CSN high again
  return (nrf_status);  // return nRF24L01 status byte
}

unsigned char SPI_Read_Packet(unsigned char reg, unsigned char *pBuf, unsigned char data_size)
{
  unsigned char nrf_status;
  unsigned char byte_num;
  nrf_csn_low;          // CSN low, init SPI transaction
  nrf_status = SPI_RW(R_REGISTER | reg); // select register
  for (byte_num = 0; byte_num < data_size; byte_num++)
      {
        pBuf[byte_num] = SPI_RW(0);
      }
  nrf_csn_high;         // CSN high again
  return (nrf_status);  // return nRF24L01 status byte
}
//**********************************************************
void prx( unsigned char size )
//Настроим nRF на прием с размером приема "size" байт
{
  RX_PAYLOAD_SIZE( size );
  RX_POWERUP();
  DWT_Delay_us (150);
  nrf_ce_high;
}

//**********************************************************
void ptx( void )
//Настройка nRF на передачу и сразу же отправка байта в пространство
{
  nrf_ce_low;
  DWT_Delay_us (150);
  TX_POWERUP();
  DWT_Delay_us (150);
  nrf_ce_high;
  DWT_Delay_us (15);
	/*TX FIFO empty flag.
1: TX FIFO empty.
0: Data in TX FIFO.*/
	/*do
	{
		nrf.tx_empty = (SPI_Read_Reg(FIFO_STATUS)>>TX_EMPTY) & 0x01;
	}
	while (!nrf.tx_empty);*/
	DWT_Delay_us (1000 ); //nrf.tx_empty = (SPI_Read_Reg(FIFO_STATUS)>>TX_EMPTY) & 0x01;
  nrf_ce_low;
}
//***********************************************************
void Send_Packet( unsigned char *pBuf1 )
{
    SPI_Write_Reg(FLUSH_TX, NOP);  
    SPI_Write_Packet( W_TX_PAYLOAD, (unsigned char*)pBuf1, PACKET_SIZE );
    DWT_Delay_us (150); //delay_us(135);
    ptx();
}

void nrf_init(void)
{
	nrf.tx_addr[0] = 0x46;
	nrf.tx_addr[1] = 0x20;
	nrf.tx_addr[2] = 0x88;
	nrf.tx_addr[3] = 0x41;
	nrf.tx_addr[4] = 0x70;
  POWERDOWN();
  HAL_Delay(2);
  nrf.config = SPI_Read_Reg(CONFIG);
  nrf.status = SPI_Read_Reg(STATUS);
  POWERUP();
  HAL_Delay(2);;
  prx(PACKET_SIZE);
  nrf.config = SPI_Read_Reg(CONFIG);
  nrf.status = SPI_Read_Reg(STATUS);
  SPI_Write_Reg(STATUS, nrf.status);
  nrf.status = SPI_Read_Reg(STATUS);
  //SPI_Write_Reg(STATUS, (1<< MAX_RT));
  nrf.status = SPI_Read_Reg(STATUS);
  nrf.fifo_status = SPI_Read_Reg(FIFO_STATUS);
  SPI_Write_Reg(FLUSH_TX, NOP);
  SPI_Write_Reg(FLUSH_RX, NOP);
  nrf.fifo_status = SPI_Read_Reg(FIFO_STATUS);
  SPI_Write_Packet(TX_ADDR, nrf.tx_addr, 5);
  SPI_Write_Packet(RX_ADDR_P0, nrf.tx_addr, 5);
  SPI_Read_Packet(RX_ADDR_P0, nrf.check_addr, 5);
  SPI_Write_Reg(RF_SETUP, nRF_1MBit|nRF_PA_MAX);
  nrf.rf_setup = SPI_Read_Reg(RF_SETUP);
  nrf.config = SPI_Read_Reg(CONFIG);
  nrf.status = SPI_Read_Reg(STATUS);
  nrf.observe_tx = SPI_Read_Reg(OBSERVE_TX);
  nrf.setup_retr = SPI_Read_Reg(SETUP_RETR);
	SPI_Write_Reg(EN_AA, 0x00);
  nrf.en_aa = SPI_Read_Reg(EN_AA);
  nrf.en_rxaddr = SPI_Read_Reg(EN_RXADDR);
  nrf.rf_ch = SPI_Read_Reg(RF_CH);
  SPI_Write_Reg(RF_CH, nrf.rf_ch);
  nrf.setup_retr = 15;
  SPI_Write_Reg(SETUP_RETR, nrf.setup_retr);
}


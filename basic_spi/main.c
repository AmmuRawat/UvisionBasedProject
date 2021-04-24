
//PA7 MOSI
//PA5 SCK
//PA6 MISO
//PA4  SS
#include "stm32f4xx.h"                  // Device header
//#include "system_stm32f4xx.h"


void  SPI_Init(void){

	 RCC->AHB1ENR |= 0x11;      //Enable clock to GPIOA and GPIOE
	 RCC->APB2ENR |= 0x1000;  //Enable clock to SPI1 
	
	 GPIOA->MODER &= ~0x0000FC00;    /* clear bit5, bit6 and bit7 of moder register */
	 GPIOA->MODER |=  0x0000A800;    //set bit5, bit6 and bit7 as alternate function mode
   GPIOA->AFR[0] &= ~0xFFF00000;   // clear bit5, bit6 and bit7 of alternate function low register
	 GPIOA->AFR[0] |=  0x55500000;   // set bit5, bit6 and bit7 as alternate function spi 
	
	 GPIOE->MODER &= ~0x00000C0;  // clear bit3 of moder register
	 GPIOE->MODER |=  0x0000040;  //set bit3 as general purpose output mode set PE3 as output
	// GPIOE->BSRR = 0x00000008;   //Deassert slave i.e. disable slave by setting
	 
	//SPI Config
	 SPI1->CR1 = 0x314;    // Set baudrate fpclk/8, SSM,SSI and master mode
   SPI1->CR2 = 0;
	 SPI1->CR1 |= 0x40;    //enable spi modes
}


void spi_write_data(uint8_t data){
	
	//while(!(SPI1->SR & 2)){}     //Wait until transfer buffer is empty
	//GPIOE->BSRR = 0x00080000;   // assert slave select i.e. enable slave by reseting
	//SPI1->DR = data;						 // write data
  //while (SPI1->SR & 0x80) {}   //wait for transmission completion
  //GPIOE->BSRR = 0x00000008;   //Deassert slave i.e. disable slave by seting
	
	
	  SPI1->DR = data;						 // write data
	
 // wait until transmit is done (TXE flag)
    while (!(SPI1->SR & (1 << 1)));
 // wait until rx buf is not empty (RXNE flag)
   // while (!(SPI1->SR & (1 << 0)));
	  while (SPI1->SR & 0x80) {}   //wait for transmission completion
	  //(void)SPI1->DR; // dummy read
	  

}

uint8_t spi_read_data(){
	
	//GPIOE->BSRR = 0x00080000;  // assert slave select i.e. enable slave by reseting
	
	// Send data 
  //SPI1->DR = reg;
	
	// wait until tx buf is empty (TXE flag)
	
	//while (!(SPI1->SR & (1 << 1))){};   
	//SPI1->DR = 0xff;
	while (SPI1->SR & 0x80) {}   //wait for transmission completion
  
	// wait until rx buf is not empty (RXNE flag)
  while ((SPI1->SR & (1 << 0))){};
		
	uint8_t b = SPI1->DR;
		

  //GPIOE->BSRR = 0x00000008;   //Deassert slave i.e. disable slave by setting
  return b;
}

uint8_t txData=5;
uint8_t rxData;
uint8_t whoami =0x0F|0x80;
int  main(){
	SPI_Init();
	while (1){
		
		GPIOE->BSRR = 0x00080000;  // assert slave select i.e. enable slave by reseting
	  spi_write_data(whoami);
	  //for(int i=0; i<10000000; i++);
	  rxData= spi_read_data();
	  GPIOE->BSRR = 0x00000008;   //Deassert slave i.e. disable slave by setting
		
	}
		
	//GPIOE->BSRR = 0x00080000;  // assert slave select i.e. enable slave by reseting
	//spi_write_data(whoami);
	//for(int i=0; i<10000000; i++);
	//rxData= spi_read_data();
	//GPIOE->BSRR = 0x00000008;   //Deassert slave i.e. disable slave by setting
	
}
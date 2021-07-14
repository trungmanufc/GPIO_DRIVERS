/*
 * stm32f407xx.h
 *
 *  Created on: Apr 30, 2021
 *      Author: CHOCOPI
 */
#include<stdint.h>
#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stddef.h>
#define __vo volatile
#define __weak __attribute__((weak))

/*
***************************PROCESSOR SPECIFIC DETAILS*********************
*/
#include"stm32f407xx_rcc_driver.h"
#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_i2c_driver.h"

//ARM cortex Mx Processor NVIC ISERx register address
#define  NVIC_ISER0			( (__vo uint32_t*)0xE000E100 )
#define  NVIC_ISER1			( (__vo uint32_t*)0xE000E104 )
#define  NVIC_ISER2			( (__vo uint32_t*)0xE000E108 )
#define  NVIC_ISER3			( (__vo uint32_t*)0xE000E10C )

//ARM cortex Mx Processor NVIC ICERx register address
#define NVIC_ICER0			( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1			( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2			( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3			( (__vo uint32_t*)0XE000E18C )

//ARM cortex Mx Processor Priority Register Address
#define NVIC_PR_BASE_ADDR 	( (__vo uint32_t*)0xE000E400 )

//BASE ADDRESS of Flash and SRAM memory
#define FLASH_BASEADDR		0x08000000U
#define SRAM1_BASEADDR		0x20000000U
#define SRAM SRAM1_BASEADDR
#define SRAM2_BASEADDR		0x20001C00U
#define ROM_BASEADDR 		0x1FFF0000U

//BASE ADDRESS of Bus
#define AP1PERIPH_BASE		0x40000000U
#define PERIPH_BASE 		AP1PERIPH_BASE
#define AP2PERIPH_BASE		0X40010000U
#define AHB1PERIPH_BASE		0X40020000U
#define AHB2PERIPH_BASE		0X50000000U

//BASE ADDRESS of GPIO on AHB1
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000)

//BASE ADDRESS of RCC on AHB1
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

//BASE ADDRESS of GPIO on APB1
#define I2C1_BASEADDR		(AP1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(AP1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(AP1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR		(AP1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(AP1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR		(AP1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(AP1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(AP1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(AP1PERIPH_BASE + 0x5000)

//BASE ADDRESS of GPIO on APB2
#define EXTI_BASEADDR		(AP2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR		(AP2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR		(AP2PERIPH_BASE + 0x3800)
#define USART1_BASEADDR		(AP2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(AP2PERIPH_BASE + 0x1400)

//GPIO registers definition
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
	}GPIO_Periph;

#define GPIOA ((GPIO_Periph*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_Periph*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_Periph*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_Periph*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_Periph*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_Periph*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_Periph*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_Periph*)GPIOH_BASEADDR)
#define GPIOI ((GPIO_Periph*)GPIOI_BASEADDR)



  extern  GPIO_Periph *pGPIOA ;
  extern  GPIO_Periph *pGPIOB ;
  extern  GPIO_Periph *pGPIOC ;
  extern  GPIO_Periph *pGPIOD ;
  extern  GPIO_Periph *pGPIOE ;
  extern  GPIO_Periph *pGPIOF ;
  extern  GPIO_Periph *pGPIOG ;
  extern  GPIO_Periph *pGPIOH ;
  extern  GPIO_Periph *pGPIOI ;


//RCC registers definition
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

	}RCC_Periph;

extern RCC_Periph *RCC;



//EXTI 	address
#define EXTI ((EXTI_Periph*)EXTI_BASEADDR)

//EXTI peripheral register definition
typedef struct
{
	__vo uint32_t IMR;     	//offset 0x00
	__vo uint32_t EMR;		//offset 0x04
	__vo uint32_t RTSR;		//offset 0x08
	__vo uint32_t FTSR;		//offset 0x0C
	__vo uint32_t SWIER;	//offset 0x10
	__vo uint32_t PR; 		//offset 0x14
	}EXTI_Periph;


/******************************************/
//Peripheral register definition for SYSCFG
/******************************************/

#define SYSCFG 	((SYSCFG_Periph*)SYSCFG_BASEADDR)

typedef struct
{
	__vo uint32_t 	MEMRMP;
	__vo uint32_t 	PMC;
	__vo uint32_t 	EXTICR[4];
	uint32_t 		RESERVED1[2];
	__vo uint32_t	CMPCR;
	uint32_t		RESERVED2[2];
	__vo uint32_t 	CFGR;

} SYSCFG_Periph;



/**************************************/
//Peripheral register definition for SPI
/**************************************/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_Periph;

#define SPI1 	((SPI_Periph*)SPI1_BASEADDR)
#define SPI2 	((SPI_Periph*)SPI2_BASEADDR)
#define SPI3 	((SPI_Periph*)SPI3_BASEADDR)



/**************************************/
//Peripheral register definition for I2C
/**************************************/

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;


}I2C_Periph;

#define I2C1  ((I2C_Periph*)I2C1_BASEADDR)
#define I2C2  ((I2C_Periph*)I2C2_BASEADDR)
#define I2C3  ((I2C_Periph*)I2C3_BASEADDR)


/*****************************************/
//Peripheral register definition for USART
/*****************************************/

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_Periph;

#define USART1 	((USART_Periph*)USART1_BASEADDR)
#define USART2 	((USART_Periph*)USART2_BASEADDR)
#define USART3 	((USART_Periph*)USART3_BASEADDR)
#define UART4	((USART_Periph*)UART4_BASEADDR)
#define UART5	((USART_Periph*)UART5_BASEADDR)
#define USART6	((USART_Periph*)USART6_BASEADDR)


//ENABLE PClock for GPIO PORT
#define GPIOA_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 	( RCC->AHB1ENR |= (1 << 8))

//Enable Clock for SPIx peripherals
#define SPI1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= (1 << 13))

//Enable clock for I2Cx peripherals
#define I2C1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23))

//Enable clock for USARTx peripherals
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= ( 1 << 4 ))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= ( 1 << 18 ))
#define UART4_PCLK_EN() 	(RCC->APB1ENR |= ( 1 << 19 ))
#define UART5_PCLK_EN() 	(RCC->APB1ENR |= ( 1 << 20 ))
#define USART6_PCLK_EN() 	(RCC->APB2ENR |= ( 1 << 5 ))

//Enable clock for SYSCFG peripherals
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

//Clock Disable Macro for GPIOx
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~( 1 << 8 ))

//Disable Clock for SPIx peripherals
#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ))
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ))
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ))
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13 ))

//Disable clock for I2Cx peripherals
#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ))

//Enable clock for USARTx peripherals
#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~( 1 << 4 ))
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~( 1 << 17 ))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= ~( 1 << 18 ))
#define UART4_PCLK_DI() 	(RCC->APB1ENR &= ~( 1 << 19 ))
#define UART5_PCLK_DI() 	(RCC->APB1ENR &= ~( 1 << 20 ))
#define USART6_PCLK_DI() 	(RCC->APB2ENR &= ~( 1 << 5 ))

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 0));	( RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 1));	( RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 2));	( RCC->AHB1RSTR &= ~(1 << 2));}	while(0)
#define GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 3));	( RCC->AHB1RSTR &= ~(1 << 3));}	while(0)
#define GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 4));	( RCC->AHB1RSTR &= ~(1 << 4));}	while(0)
#define GPIOF_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 5));	( RCC->AHB1RSTR &= ~(1 << 5));}	while(0)
#define GPIOG_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 6));	( RCC->AHB1RSTR &= ~(1 << 6));}	while(0)
#define GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 7));	( RCC->AHB1RSTR &= ~(1 << 7));}	while(0)
#define GPIOI_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 8));	( RCC->AHB1RSTR &= ~(1 << 8));}	while(0)

//Returns port code for given GPIOx base address
#define GPIO_BASEADDR_TO_CODE(x) (	( x == GPIOA ) ? 0 :\
									( x == GPIOB ) ? 1 :\
									( x == GPIOC ) ? 2 :\
									( x == GPIOD ) ? 3 :\
									( x == GPIOE ) ? 4 :\
									( x == GPIOF ) ? 5 :\
									( x == GPIOG ) ? 6 :\
									( x == GPIOH ) ? 7 :0 )

//IRQ(Interrupt Request) Numbers
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C3_ER		34
//#define IRQ_NO_I2C1_EV		31
//#define IRQ_NO_I2C1_ER		32

//Some generic macros
#define NO_PR_BITS_IMPLEMENTED 	4
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET 			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET

//SPI DeviceMode macros
#define SPI_DEVICE_MODE_MASTER 	1
#define SPI_DEVICE_MODE_SLAVE	0

//Bit position definitions of SPI peripheral
//Bit position definitions SPI_CR1
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//Bit position definitions SPI_CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//Bit positions definitions SPI_SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

//Bit positions for I2C_CR1
#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15

//Bit positions for I2C_CR2
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10

//Bit position definitions I2C_SR1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

//Bit position definitions I2C_SR2
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

//Bit position definitions for I2C_CCR
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#endif /* INC_STM32F407XX_H_ */

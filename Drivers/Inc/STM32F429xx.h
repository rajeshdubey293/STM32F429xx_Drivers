/*
 * STM32F429xx.h
 *
 *  Created on: Mar 14, 2020
 *      Author: vicky
 */

#ifndef INC_STM32F429XX_H_
#define INC_STM32F429XX_H_


#include<stdint.h>
#include<stddef.h>

#define __vo volatile
/*************************************Processor Specific Details***********************
 *************************************************************************************/

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
  */
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1U       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       /*!< FPU present                                   */

/*
* ARM Cortex M4 Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0							((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE000E10C)

/*
* ARM Cortex M4 Processor NVIC ICERx register Addresses                              *
 */

#define NVIC_ICER0							((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 Processor Priority Register Addresses
 */

#define NVIC_PR_BASEADDR					((__vo uint32_t*)0xE000E400)
/*
 * ARM Cortex M4 Processor number of priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED				4

/*
 * Base Addresses of Flash and SRAM memories
 */


#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR						0x2001C000U
#define SRAM3_BASEADDR						0x20020000U
#define SRAM_BASEADDR						SRAM1_BASEADDR
#define ROM_BASEADDR						0x1FFF0000U





/*
 * Base Addresses of AHBx and APBx Peripherals
 */

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR 				0x40020000U
#define AHB2PERIPH_BASEADDR 				0x50000000U

#define TIM2_BASE             				(APB1PERIPH_BASEADDR + 0x0000U)
#define TIM3_BASE             				(APB1PERIPH_BASEADDR + 0x0400U)
#define TIM4_BASE             				(APB1PERIPH_BASEADDR + 0x0800U)
#define TIM5_BASE             				(APB1PERIPH_BASEADDR + 0x0C00U)
#define TIM6_BASE             				(APB1PERIPH_BASEADDR + 0x1000U)
#define TIM7_BASE             				(APB1PERIPH_BASEADDR + 0x1400U)
#define TIM12_BASE            				(APB1PERIPH_BASEADDR + 0x1800U)
#define TIM13_BASE            				(APB1PERIPH_BASEADDR + 0x1C00U)
#define TIM14_BASE            				(APB1PERIPH_BASEADDR + 0x2000U)
#define RTC_BASE              				(APB1PERIPH_BASEADDR + 0x2800U)
#define WWDG_BASE             				(APB1PERIPH_BASEADDR + 0x2C00U)
#define IWDG_BASE             				(APB1PERIPH_BASEADDR + 0x3000U)
/*
 * Base Addresses of GPIOx
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2000)
//#define GPIOJ_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2400)
//#define GPIOK_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2800)

/*
 * Base Addresses of SPIx
 */
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR						(APB2PERIPH_BASEADDR + 0x5400)


/*
 * Base Addresses of I2Cx
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)


/*
 * Base Addresses of UARTx and USARTx
 */

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR 						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 						(APB1PERIPH_BASEADDR + 0x5000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)
//#define UART7_BASEADDR 					(APB1PERIPH_BASEADDR + 0x7800)
//#define UART8_BASEADDR 					(APB1PERIPH_BASEADDR + 0x7C00)

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
/*
 * Base Address of Reset and Clock Control
 */

#define RCC_BASEADDR 						(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base Addresses of Some Peripherals, which are hanging on APB2
 */

#define EXTI_BASEADDR 						(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)

/*
 *   Peripheral register definition structure FOR GPIOx
 */

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
}GPIO_RegDef_t;

/*
 * Peripheral register definitions structure of EXTI
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/*
 * Peripheral register definitions structure of SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

/*
 *   Peripheral register definition structure For RCC
 */

typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                              */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                        */
  DMA2D_IRQn                  = 90      /*!< DMA2D global Interrupt                                            */
} IRQn_Type;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t OAR1;       /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t OAR2;       /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t DR;         /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t SR1;        /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t SR2;        /*!< TODO,     										Address offset: 0x18 */
  __vo uint32_t CCR;        /*!< TODO,     										Address offset: 0x1C */
  __vo uint32_t TRISE;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t FLTR;       /*!< TODO,     										Address offset: 0x24 */
}I2C_RegDef_t;


/**
  * @brief TIM
  */

typedef struct
{
  __vo uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  __vo uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  __vo uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  __vo uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  __vo uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  __vo uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  __vo uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  __vo uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  __vo uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  __vo uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  __vo uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  __vo uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  __vo uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  __vo uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  __vo uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  __vo uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  __vo uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  __vo uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  __vo uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  __vo uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  __vo uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/**
  * @brief DMA Controller
  */

typedef struct
{
  __vo uint32_t CR;     /*!< DMA stream x configuration register      */
  __vo uint32_t NDTR;   /*!< DMA stream x number of data register     */
  __vo uint32_t PAR;    /*!< DMA stream x peripheral address register */
  __vo uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  __vo uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  __vo uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
  __vo uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  __vo uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  __vo uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  __vo uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;
/*
 *		Peripheral definitions of GPIOx
 */

#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI								((GPIO_RegDef_t*)GPIOI_BASEADDR)
//#define GPIOJ								((GPIO_RegDef_t*)GPIOJ_BASEADDR)
//#define GPIOK								((GPIO_RegDef_t*)GPIOK_BASEADDR)


/*
 * Some Peripherals definitions
 */

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



/*
 * 	Peripheral definitions of SPIx
 */


#define SPI1								((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4								((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5								((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6								((SPI_RegDef_t*)SPI6_BASEADDR)

/*
 * 	Peripheral definitions of I2Cx
 */


#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * Clock enable Macros for SYSCFG
 */

#define SYSCFG_PCLK_EN()     				(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Enable Macros For GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 8 ) )
//#define GPIOJ_PCLK_EN()					( RCC->AHB1ENR |= ( 	   ) )
//#define GPIOK_PCLK_EN()					( RCC->AHB1ENR |= (        ) )

/*
 * Clock Enable Macros For I2Cx Peripherals
 */

#define I2C1_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros For SPIx Peripherals
 */

#define SPI1_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 15 ) )
//#define SPI4_PCLK_EN()					( RCC->APB2ENR |= ( 	    ) )
//#define SPI5_PCLK_EN()					( RCC->APB2ENR |= ( 	    ) )
//#define SPI6_PCLK_EN()					( RCC->APB2ENR |= (         ) )


/*
 * Clock Enable Macros For UARTx and USARTx Peripherals
 */

#define USART1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 14 ) )
#define UART5_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 15 ) )
#define USART6_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 5 ) )




/*
 * Clock Disable Macros For GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 8 ) )
//#define GPIOJ_PCLK_DI()					( RCC->AHB1ENR &= ~( 	   ) )
//#define GPIOK_PCLK_DI()					( RCC->AHB1ENR &= ~(        ) )

/*
 * Clock Disable Macros For I2Cx Peripherals
 */

#define I2C1_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock Disable Macros For SPIx Peripherals
 */

#define SPI1_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 15 ) )
//#define SPI4_PCLK_DI()					( RCC->APB2ENR &= ~( 	    ) )
//#define SPI5_PCLK_DI()					( RCC->APB2ENR &= ~( 	    ) )
//#define SPI6_PCLK_DI()					( RCC->APB2ENR &= ~(         ) )


/*
 * Clock Disable Macros For UARTx and USARTx Peripherals
 */

#define USART1_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 14 ) )
#define UART5_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 15 ) )
#define USART6_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 5 ) )


/*
 * Macros to Reset GPIOx peripheral
 */
#define GPIOA_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 0 ) ); (RCC->AHB1RSTR &= ~( 1 << 0));} while(0)
#define GPIOB_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 1 ) ); (RCC->AHB1RSTR &= ~( 1 << 1));} while(0)
#define GPIOC_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 2 ) ); (RCC->AHB1RSTR &= ~( 1 << 2));} while(0)
#define GPIOD_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 3 ) ); (RCC->AHB1RSTR &= ~( 1 << 3));} while(0)
#define GPIOE_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 4 ) ); (RCC->AHB1RSTR &= ~( 1 << 4));} while(0)
#define GPIOF_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 5 ) ); (RCC->AHB1RSTR &= ~( 1 << 5));} while(0)
#define GPIOG_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 6 ) ); (RCC->AHB1RSTR &= ~( 1 << 6));} while(0)
#define GPIOH_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 7 ) ); (RCC->AHB1RSTR &= ~( 1 << 7));} while(0)
#define GPIOI_REG_RESET()					do{( RCC->AHB1RSTR |= ( 1 << 8 ) ); (RCC->AHB1RSTR &= ~( 1 << 8));} while(0)

/*
 *   Peripheral register definition structure For SPIx
 */

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
}SPI_RegDef_t;


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0   		0
#define NVIC_IRQ_PRI15    		15

/*
 * Some Generic Macros
 */

#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET         		RESET
#define FLAG_SET 				SET
#define HIGH 					SET
#define LOW 					RESET


/************************************************************************************************************
 * 			Bit position definitions of SPI peripherals														*
 ************************************************************************************************************/

/*
 * 	Bit position definition of SPI_CR1 register
 */

#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15

/*
 * 	Bit position definition of SPI_CR2 register
 */
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7


/*
 * 	Bit position definition of SPI_SR register
 */


#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRCERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8



/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				0
#define I2C_OAR1_ADD71 				 	1
#define I2C_OAR1_ADD98  			 	8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA)?0:\
										(x == GPIOB)?1:\
		 	 	 	 	 	 	 	 	(x == GPIOC)?2:\
		 	 	 	 	 	 	 	 	(x == GPIOD)?3:\
		 	 	 	 	 	 	 	 	(x == GPIOE)?4:\
		 	 	 	 	 	 	 	 	(x == GPIOF)?5:\
		 	 	 	 	 	 	 	 	(x == GPIOG)?6:\
		 	 	 	 	 	 	 	 	(x == GPIOH)?7:\
		 	 	 	 	 	 	 	 	(x == GPIOI)?8:0)


/*
 * IRQ (Interrupt Request) numbers of STM32F4xx MCU
 */
#define IRQ_NO_EXTI0						6
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40

#endif /* INC_STM32F429XX_H_ */

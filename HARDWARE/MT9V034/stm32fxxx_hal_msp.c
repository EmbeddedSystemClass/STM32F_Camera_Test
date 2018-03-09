/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * HAL MSP.
 *
 */
 #include "sys.h"
#include "delay.h"
#include "usart.h"			 
#include "sccb.h"	
#include "pcf8574.h"  
#include "ltdc.h"
 
 #include "cambus.h"
 
//#include STM32_HAL_H
#include "omv_boardconfig.h"

void __fatal_error(const char *msg);
#define Error_Handler() __fatal_error("SystemClock_Config"

/* GPIO struct */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} gpio_t;

/* DCMI GPIOs */
static const gpio_t dcmi_pins[] = {
    {DCMI_D0_PORT, DCMI_D0_PIN},
    {DCMI_D1_PORT, DCMI_D1_PIN},
    {DCMI_D2_PORT, DCMI_D2_PIN},
    {DCMI_D3_PORT, DCMI_D3_PIN},
    {DCMI_D4_PORT, DCMI_D4_PIN},
    {DCMI_D5_PORT, DCMI_D5_PIN},
    {DCMI_D6_PORT, DCMI_D6_PIN},
    {DCMI_D7_PORT, DCMI_D7_PIN},
    {DCMI_HSYNC_PORT, DCMI_HSYNC_PIN},
    {DCMI_VSYNC_PORT, DCMI_VSYNC_PIN},
    {DCMI_PXCLK_PORT, DCMI_PXCLK_PIN},
};

#define NUM_DCMI_PINS   (sizeof(dcmi_pins)/sizeof(dcmi_pins[0]))

//void SystemClock_Config(void);

//void SystemClock_Config(void)
//{
//    uint32_t flash_latency;
//    RCC_ClkInitTypeDef RCC_ClkInitStruct;
//    RCC_OscInitTypeDef RCC_OscInitStruct;
//#if defined(MCU_SERIES_H7)
//    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
//#endif

//#if defined (STM32H743xx)// 400MHz/48MHz
//    /* Supply configuration update enable */
//    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
//#else
//    /* Enable Power Control clock */
//    __PWR_CLK_ENABLE();
//#endif

//    /* The voltage scaling allows optimizing the power consumption when the device is
//       clocked below the maximum system frequency, to update the voltage scaling value
//       regarding system frequency refer to product datasheet.  */
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//#if defined (STM32H743xx)// 400MHz/48MHz
//    while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) {
//    }
//#endif

//    /* Enable HSE Oscillator and activate PLL with HSE as source */
//    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
//#if defined(MCU_SERIES_H7)
//    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
//#endif
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

//#if   defined (STM32F407xx) // 168MHz/48MHz
//    flash_latency = FLASH_LATENCY_5;
//    RCC_OscInitStruct.PLL.PLLM = 12;
//    RCC_OscInitStruct.PLL.PLLN = 336;
//    RCC_OscInitStruct.PLL.PLLQ = 7;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//#elif defined (STM32F427xx)// 192MHz/48MHz
//    flash_latency = FLASH_LATENCY_7;
//    RCC_OscInitStruct.PLL.PLLM = 12;
//    RCC_OscInitStruct.PLL.PLLN = 384;
//    RCC_OscInitStruct.PLL.PLLQ = 8;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//#elif defined (STM32F765xx)// 216MHz/48MHz
//    flash_latency = FLASH_LATENCY_7;
//    RCC_OscInitStruct.PLL.PLLM = 12;
//    RCC_OscInitStruct.PLL.PLLN = 432;
//    RCC_OscInitStruct.PLL.PLLQ = 9;
//    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//    RCC_OscInitStruct.PLL.PLLR = 2;
//#elif defined (STM32H743xx)// 400MHz/48MHz
//    flash_latency = FLASH_LATENCY_4;
//    RCC_OscInitStruct.PLL.PLLM = 3;
//    RCC_OscInitStruct.PLL.PLLN = 200;
//    RCC_OscInitStruct.PLL.PLLP = 2;
//    RCC_OscInitStruct.PLL.PLLQ = 8;
//    RCC_OscInitStruct.PLL.PLLR = 2;
//    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
//    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//    RCC_OscInitStruct.PLL.PLLFRACN = 0;
//#endif
//    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//        /* Initialization Error */
//				while(1){}
//        //Error_Handler();
//    }

//#if defined(STM32H743xx)
//    /* PLL3 for USB Clock */
//    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
//    PeriphClkInitStruct.PLL3.PLL3M = 6;
//    PeriphClkInitStruct.PLL3.PLL3N = 192;
//    PeriphClkInitStruct.PLL3.PLL3P = 2;
//    PeriphClkInitStruct.PLL3.PLL3Q = 8;
//    PeriphClkInitStruct.PLL3.PLL3R = 2;
//    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_1;
//    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
//    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
//        /* Initialization Error */
//        Error_Handler();
//    }
//#endif


//#if defined(MCU_SERIES_H7)
//    HAL_PWREx_EnableUSBVoltageDetector();
//#endif

//#if defined(MCU_SERIES_F4) || defined(MCU_SERIES_F7)
//    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
//        /* Initialization Error */
//        Error_Handler();
//    }
//#endif

//    /* Select PLL as system clock source and configure the HCLK, PCLK clocks dividers */
//#if defined(MCU_SERIES_H7)
//    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 |
//                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);
//    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
//#else
//    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//#endif

//    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, flash_latency) != HAL_OK) {
//        /* Initialization Error */
//				while(1){}
//        //Error_Handler();
//    }

//#if defined(MCU_SERIES_H7)
//    // WTF is I/O Compensation Cell ?
//    ///*activate CSI clock mondatory for I/O Compensation Cell*/
//    //__HAL_RCC_CSI_ENABLE() ;

//    ///* Enable SYSCFG clock mondatory for I/O Compensation Cell */
//    __HAL_RCC_SYSCFG_CLK_ENABLE() ;

//    ///* Enables the I/O Compensation Cell */
//    //HAL_EnableCompensationCell();
//#endif

//    SystemCoreClockUpdate();
//}

void HAL_MspInit(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		
    #if defined(MCU_SERIES_F7) || defined(MCU_SERIES_H7)
    // Invalidate each cache before enabling it
    SCB_InvalidateICache();
    SCB_InvalidateDCache();

    /* Enable the CPU Cache */
    SCB_EnableICache();
    SCB_EnableDCache();
    #endif

    /* Set the system clock */
   // SystemClock_Config();

    /* Config Systick */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    /* Enable GPIO clocks */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __GPIOD_CLK_ENABLE();
    __GPIOE_CLK_ENABLE();

#if defined(STM32F769xx)
    __GPIOF_CLK_ENABLE();
    __GPIOG_CLK_ENABLE();
    __GPIOH_CLK_ENABLE();
    __GPIOI_CLK_ENABLE();
    __GPIOJ_CLK_ENABLE();
    __GPIOK_CLK_ENABLE();

    /* Enable JPEG clock */
    __HAL_RCC_JPEG_CLK_ENABLE();
#endif

    /* Enable DMA clocks */
    __DMA1_CLK_ENABLE();
    __DMA2_CLK_ENABLE();

    /* Configure DCMI GPIO */
    
    GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStructure.Pin = DCMI_RESET_PIN;
    HAL_GPIO_Init(DCMI_RESET_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_PWDN_PIN;
    HAL_GPIO_Init(DCMI_PWDN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = DCMI_FSIN_PIN;
    HAL_GPIO_Init(DCMI_FSIN_PORT, &GPIO_InitStructure);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
    if (hi2c->Instance == SCCB_I2C) {
        /* Enable I2C clock */
        SCCB_CLK_ENABLE();

        /* Configure SCCB GPIOs */
        
        GPIO_InitStructure.Pull      = GPIO_NOPULL;
        GPIO_InitStructure.Speed     = GPIO_SPEED_LOW;
        GPIO_InitStructure.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStructure.Alternate = SCCB_AF;

        GPIO_InitStructure.Pin = SCCB_SCL_PIN;
        HAL_GPIO_Init(SCCB_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin = SCCB_SDA_PIN;
        HAL_GPIO_Init(SCCB_PORT, &GPIO_InitStructure);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == SCCB_I2C) {
        SCCB_CLK_DISABLE();
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		
    #if (OMV_XCLK_SOURCE == OMV_XCLK_TIM)
    if (htim->Instance == DCMI_TIM) {
        /* Enable DCMI timer clock */
        DCMI_TIM_CLK_ENABLE();

        /* Timer GPIO configuration */
        
        GPIO_InitStructure.Pin       = DCMI_TIM_PIN;
        GPIO_InitStructure.Pull      = GPIO_NOPULL;
        GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
        GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Alternate = DCMI_TIM_AF;
        HAL_GPIO_Init(DCMI_TIM_PORT, &GPIO_InitStructure);
    }
    #endif // (OMV_XCLK_SOURCE == OMV_XCLK_TIM)
}

//void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
//{
//		int i;
//		GPIO_InitTypeDef  GPIO_InitStructure;
//	
//    /* DCMI clock enable */
//    __DCMI_CLK_ENABLE();

//    /* DCMI GPIOs configuration */
//    
//    GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
//    GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
//    GPIO_InitStructure.Alternate = GPIO_AF13_DCMI;

//    /* Enable VSYNC EXTI */
//    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
//    GPIO_InitStructure.Pin  = DCMI_VSYNC_PIN;
//    HAL_GPIO_Init(DCMI_VSYNC_PORT, &GPIO_InitStructure);

//    /* Configure DCMI pins */
//    GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
//    for (i=0; i<NUM_DCMI_PINS; i++) {
//        GPIO_InitStructure.Pin = dcmi_pins[i].pin;
//        HAL_GPIO_Init(dcmi_pins[i].port, &GPIO_InitStructure);
//    }
//}

void HAL_MspDeInit(void)
{

}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bsp.h"
#include "config.h"

#ifdef CONFIG_ENABLE_PCN
    #include "cc1101.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t uart_rx_buf = 0;
int     uart2_rx_error = 0;
//uint8_t uart2_rx_buf[40];

typedef union
{
  uint8_t u8[2];
  uint16_t u16;
} int2byte;

volatile int pm25_value = 0;
volatile int co2_ppm = 0;
volatile int co2_temp = 0;
volatile int co2_humi = 0;
int system_config_mode = 0;
int system_error = 0;
extern volatile int key_down_press_type;
extern volatile int key_up_press_type;

struct eerom_content_s eerom_cfg;
int menu_update_next = 0;
int diag_mode = 0;

int uart2_irq_count = 0;
unsigned int run_time_count = 0;
unsigned int last_time_count_ms = 0;

struct uart2_buffer_s
{
    #define UART2_MAX_BUFFER 40
    uint8_t ptr;
    uint8_t rx_data;
    uint8_t data[UART2_MAX_BUFFER];

} uart2_buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void u2_buffer_dump()
{
    int i = 0;
    PrintDBG("Dump:\r\n");
    for(i = 0; i < 32; i++)
    {
        PrintDBG("0x%x ", uart2_buffer.data[i]);
    }
    PrintDBG("\r\n");
}

#define USART_IT_PE ((uint16_t)0x0028)
#define USART_IT_TXE ((uint16_t)0x0727)
#define USART_IT_TC ((uint16_t)0x0626)
#define USART_IT_RXNE ((uint16_t)0x0525)
#define USART_IT_IDLE ((uint16_t)0x0424)
#define USART_IT_LBD ((uint16_t)0x0846)
#define USART_IT_CTS ((uint16_t)0x096A)
#define USART_IT_ERR ((uint16_t)0x0060)
#define USART_IT_ORE ((uint16_t)0x0360)
#define USART_IT_NE ((uint16_t)0x0260)
#define USART_IT_FE ((uint16_t)0x0160)

#define IT_Mask ((uint16_t)0x001F) /*!< USART Interrupt Mask */

#define USART_FLAG_CTS ((uint16_t)0x0200)
#define USART_FLAG_LBD ((uint16_t)0x0100)
#define USART_FLAG_TXE ((uint16_t)0x0080)
#define USART_FLAG_TC ((uint16_t)0x0040)
#define USART_FLAG_RXNE ((uint16_t)0x0020)
#define USART_FLAG_IDLE ((uint16_t)0x0010)
#define USART_FLAG_ORE ((uint16_t)0x0008)
#define USART_FLAG_NE ((uint16_t)0x0004)
#define USART_FLAG_FE ((uint16_t)0x0002)
#define USART_FLAG_PE ((uint16_t)0x0001)
#define IS_USART_FLAG(FLAG) (((FLAG) == USART_FLAG_PE) || ((FLAG) == USART_FLAG_TXE) ||   \
                             ((FLAG) == USART_FLAG_TC) || ((FLAG) == USART_FLAG_RXNE) ||  \
                             ((FLAG) == USART_FLAG_IDLE) || ((FLAG) == USART_FLAG_LBD) || \
                             ((FLAG) == USART_FLAG_CTS) || ((FLAG) == USART_FLAG_ORE) ||  \
                             ((FLAG) == USART_FLAG_NE) || ((FLAG) == USART_FLAG_FE))

#define IS_USART_CLEAR_FLAG(FLAG) ((((FLAG) & (uint16_t)0xFC9F) == 0x00) && ((FLAG) != (uint16_t)0x00))
#define IS_USART_PERIPH_FLAG(PERIPH, USART_FLAG) ((((*(uint32_t *)&(PERIPH)) != UART4_BASE) &&  \
                                                   ((*(uint32_t *)&(PERIPH)) != UART5_BASE)) || \
                                                  ((USART_FLAG) != USART_FLAG_CTS))
#define IS_USART_BAUDRATE(BAUDRATE) (((BAUDRATE) > 0) && ((BAUDRATE) < 0x0044AA21))
#define IS_USART_ADDRESS(ADDRESS) ((ADDRESS) <= 0xF)
#define IS_USART_DATA(DATA) ((DATA) <= 0x1FF)

ITStatus USART_GetITStatus(USART_TypeDef *USARTx, uint16_t USART_IT)
{
    uint32_t bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
    ITStatus bitstatus = RESET;
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(USARTx));
    assert_param(IS_USART_GET_IT(USART_IT));
    /* The CTS interrupt is not available for UART4 and UART5 */
    if (USART_IT == USART_IT_CTS)
    {
        assert_param(IS_USART_123_PERIPH(USARTx));
    }

    /* Get the USART register index */
    usartreg = (((uint8_t)USART_IT) >> 0x05);
    /* Get the interrupt position */
    itmask = USART_IT & IT_Mask;
    itmask = (uint32_t)0x01 << itmask;

    if (usartreg == 0x01) /* The IT  is in CR1 register */
    {
        itmask &= USARTx->CR1;
    }
    else if (usartreg == 0x02) /* The IT  is in CR2 register */
    {
        itmask &= USARTx->CR2;
    }
    else /* The IT  is in CR3 register */
    {
        itmask &= USARTx->CR3;
    }

    bitpos = USART_IT >> 0x08;
    bitpos = (uint32_t)0x01 << bitpos;
    bitpos &= USARTx->SR;
    if ((itmask != (uint16_t)RESET) && (bitpos != (uint16_t)RESET))
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

uint16_t USART_ReceiveData(USART_TypeDef *USARTx)
{
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(USARTx));

    /* Receive Data */
    return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}

void USART_ClearFlag(USART_TypeDef *USARTx, uint16_t USART_FLAG)
{
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(USARTx));
    assert_param(IS_USART_CLEAR_FLAG(USART_FLAG));
    /* The CTS flag is not available for UART4 and UART5 */
    if ((USART_FLAG & USART_FLAG_CTS) == USART_FLAG_CTS)
    {
        assert_param(IS_USART_123_PERIPH(USARTx));
    }

    USARTx->SR = (uint16_t)~USART_FLAG;
}


void USART_ITConfig(USART_TypeDef *USARTx, uint16_t USART_IT, FunctionalState NewState)
{
    uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
    uint32_t usartxbase = 0x00;
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(USARTx));
    assert_param(IS_USART_CONFIG_IT(USART_IT));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    /* The CTS interrupt is not available for UART4 and UART5 */
    if (USART_IT == USART_IT_CTS)
    {
        assert_param(IS_USART_123_PERIPH(USARTx));
    }

    usartxbase = (uint32_t)USARTx;

    /* Get the USART register index */
    usartreg = (((uint8_t)USART_IT) >> 0x05);

    /* Get the interrupt position */
    itpos = USART_IT & IT_Mask;
    itmask = (((uint32_t)0x01) << itpos);

    if (usartreg == 0x01) /* The IT is in CR1 register */
    {
        usartxbase += 0x0C;
    }
    else if (usartreg == 0x02) /* The IT is in CR2 register */
    {
        usartxbase += 0x10;
    }
    else /* The IT is in CR3 register */
    {
        usartxbase += 0x14;
    }
    if (NewState != DISABLE)
    {
        *(__IO uint32_t *)usartxbase |= itmask;
    }
    else
    {
        *(__IO uint32_t *)usartxbase &= ~itmask;
    }
}

#define CR1_UE_Set ((uint16_t)0x2000)   /*!< USART Enable Mask */
#define CR1_UE_Reset ((uint16_t)0xDFFF) /*!< USART Disable Mask */

void USART_Cmd(USART_TypeDef *USARTx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_USART_ALL_PERIPH(USARTx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the selected USART by setting the UE bit in the CR1 register */
        USARTx->CR1 |= CR1_UE_Set;
    }
    else
    {
        /* Disable the selected USART by clearing the UE bit in the CR1 register */
        USARTx->CR1 &= CR1_UE_Reset;
    }
}

void Process_UART2_Rx()
{
    uint8_t getdata = 0;
    uint8_t i = 0;
    uint16_t addsum = 0, addsum2 = 0;
    uint16_t value = 0;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        getdata = USART_ReceiveData(USART2);
        if (uart2_buffer.ptr == 0)
        {
            if (getdata == 0x42)
            {
                uart2_buffer.data[uart2_buffer.ptr] = getdata;
                uart2_buffer.ptr++;
            }
        }
        else if (uart2_buffer.ptr == 1)
        {
            if (getdata == 0x4d)
            {
                uart2_buffer.data[uart2_buffer.ptr] = getdata;
                uart2_buffer.ptr++;
            }
            else
            {
                uart2_buffer.ptr = 0;
            }
        }
        else if ((uart2_buffer.ptr > 1) && (uart2_buffer.ptr < 32))
        {
            uart2_buffer.data[uart2_buffer.ptr] = getdata;
            uart2_buffer.ptr++;
        }
        else if (uart2_buffer.ptr == 32)
        {
            for (i = 0; i < 30; i++)
            {
                addsum += uart2_buffer.data[i];
            }

            addsum2 = (uart2_buffer.data[30] << 8) | (uart2_buffer.data[31]);
            if (addsum != addsum2)
            {
                uart2_rx_error++;
                //if (uart2_rx_error >= 5)
                {
                    system_error |= SYS_ERROR_FLAG_PM25_EFRAME;
                    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
                    PrintERR("U2 Frame Error\r\n");
                    u2_buffer_dump();
                }
            }
            else
            {
                value = (uart2_buffer.data[12] << 8) | uart2_buffer.data[13];
                pm25_value = value;
                //uart2_rx_error = 0;

                HAL_GPIO_TogglePin(LED_G1_GPIO_Port, LED_G1_Pin);
            }

            for (i = 0; i < UART2_MAX_BUFFER; i++)
                uart2_buffer.data[i] = 0;
            uart2_buffer.ptr = 0;
        }
    }

    USART_ClearFlag(USART2, USART_FLAG_RXNE);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  //CDC_Transmit_FS((uint8_t *)&ch, 1);
  return ch;
}
#else
int fputc(int ch, FILE *stream)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    int err = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    //Read EEROM CONFIG
  err = eerom_boot_init(&eerom_cfg);
  if(err < 0)
  {
      PrintERR("ERROR:Invalid EEROM\r\n");
      PrintERR("Revert to factory\r\n");
      system_error |= SYS_ERROR_FLAG_ROM;
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  }

    //Init PM25
    PrintDBG("Init PM25\r\n");
    //PM25RESET(GPIO_PIN_SET);

    HAL_Delay(100);

    //HAL_UART_Receive(&huart2, (uint8_t *)uart2_buffer.data, 32, 1);  //flush uart
    PM25SET(GPIO_PIN_SET);
    for (int dd = 0; dd < 32; dd++)
        uart2_buffer.data[dd] = 0;
    uart2_buffer.ptr = 0;
    //HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2_buffer.rx_data, 1);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TXE);
    //USART_ClearFlag(USART2, USART_FLAG_TXE);

#ifdef CONFIG_ENALBE_CO2
    //Init CO2
    co2_init();
    HAL_Delay(100);
    co2_enable(1);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    //Init LCD
    lcd_reset();
#if 0
    lcd_switch_page(LCD_PAGE_INIT);
    lcd_update_init_page(0);
#endif
    PrintDBG("Start Loop\r\n");

    float co2=0.0, temp=0.0, hm=0.0;
    int ret = 0;
#ifdef CONFIG_ENALBE_CO2
    co2_getdata(&co2, &temp, &hm, 1);
#endif
    unsigned int cnt = 0;
    //Init page
    int loop_cnt = 0;
    int init_progress = 0;

    if (eerom_cfg.auto_bl_en == 0)
        lcd_set_fix_bl(eerom_cfg.fix_bl_val);
        
#if 1
    while(1)
    {
        cnt = HAL_GetTick();
        if ((cnt % 100) == 0)
        {
            if (init_progress >= 100)
                init_progress = 100;
            lcd_update_init_page(init_progress);
            init_progress++;
        }

        if ((cnt % 1000) == 0)
        {
            loop_cnt++;
            if (loop_cnt > 10)
                break;
#ifdef CONFIG_ENALBE_CO2
            ret = co2_getdata(&co2, &temp, &hm, 0);
            if (ret != 0)
            {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
                system_error |= SYS_ERROR_FLAG_CO2;
            }
#endif
            HAL_GPIO_TogglePin(LED_G2_GPIO_Port, LED_G2_Pin);
        }


    }
    HAL_Delay(200);
#else
    HAL_Delay(1000);
#endif
    
    lcd_switch_page(LCD_PAGE_ALL);

    loop_cnt = 0;
    unsigned int last_tick_val = -1;
    //int config_mode_entry_cnt = 0;
    while (1)
    {
        if ((key_down_press_type == KEY_PRESS_LONG))
        {
            if(0 != HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin))
            {
                key_down_press_type = KEY_PRESS_NONE;
                key_up_press_type = KEY_PRESS_NONE;
            }
            else
            {
                while(1)
                {
                    if (key_up_press_type != KEY_PRESS_NONE)
                        break;
                    HAL_Delay(1);
                }
                key_down_press_type = KEY_PRESS_NONE;
                key_up_press_type = KEY_PRESS_NONE;
                config_mode();
            }
            
        }
        
#if 1
        if((key_down_press_type != KEY_PRESS_NONE) || (key_up_press_type != KEY_PRESS_NONE))
        {
            key_down_press_type = KEY_PRESS_NONE;
            key_up_press_type = KEY_PRESS_NONE;
        }
#endif

        if(system_config_mode != 0)
            continue;

        cnt = HAL_GetTick();

        if(cnt == last_tick_val)
            continue;
        last_tick_val = cnt;

#if 0
        if ((cnt % 300) == 0)
        {
            HAL_GPIO_TogglePin(LED_G2_GPIO_Port, LED_G2_Pin);
        }
#endif

        if ((cnt % 1000) == 0)
        {
            loop_cnt++;
            if(loop_cnt == 50)
            {
                sensor_stable(1);
            }
            PrintDBG("PM25= %d ug/m3, cnt=%d\r\n", pm25_value, uart2_irq_count);
            if(system_error)
            {
                PrintERR("System Error:0x%x\r\n", system_error);
            }
            //Update LCD
            float temp_ofst = 0.0;
            int hm_ofst = 0;
            if(eerom_cfg.temp_offset == 0x0 || eerom_cfg.temp_offset == 0x80)
            {
                temp_ofst = 0.0;
            }
            else
            {
                if ((eerom_cfg.temp_offset & 0x80) != 0)
                {
                    int tmp = (-1) * (eerom_cfg.temp_offset & 0x7f);
                    temp_ofst = ((float)tmp / 10.0);
                }
                else
                {
                    int tmp = (eerom_cfg.temp_offset & 0x7f);
                    temp_ofst = ((float)tmp / 10.0);
                }
            }

            if(eerom_cfg.hm_offset == 0x0 || eerom_cfg.hm_offset == 0x80)
            {
                hm_ofst = 0;
            }
            else
            {
                if ((eerom_cfg.hm_offset & 0x80) != 0)
                {
                    hm_ofst = (-1) * (eerom_cfg.hm_offset & 0x7f);
                }
                else
                {
                    hm_ofst = (eerom_cfg.hm_offset & 0x7f);
                }
            }

            //temp += temp_ofst;
            //hm += hm_ofst;

            if(diag_mode)
            {
                lcd_show_diag_page(co2, temp, hm);
            }
            else
            {
                lcd_update_main(pm25_value, (int)(co2 + 0.5), (int)(temp + temp_ofst + 0.5), (int)(hm + 0.5 + hm_ofst));
            }

            run_time_count++;
            if(run_time_count >= 600)
            {
                eerom_write_increase_time(1);
                run_time_count = 0;
            }
        }

#ifdef CONFIG_ENALBE_CO2
        if((cnt % 2000) == 0)
        {
            ret = co2_getdata(&co2, &temp, &hm, 0);
            if (ret == 0)
            {
                PrintDBG("CO2=%.fppm  Temp=%.1f'C  HM=%.f%%\r\n", co2, temp, hm);
            }
            else
            {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
                system_error |= SYS_ERROR_FLAG_CO2;
            }
        }
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart_rx_buf, 1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
#if 0
  for(int dd = 0; dd < 32; dd++)
      uart2_rx_buf[dd] = 0;
  HAL_UART_Receive_IT(&huart2, (uint8_t *)uart2_rx_buf, 32);
#endif
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, USB_CTRL_Pin|LED_Y_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PM25_RESET_Pin|PM25_SET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_G2_Pin|LED_G1_Pin|LED_R_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PA15_GPIO_Port, PA15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_UP_Pin KEY_DOWN_Pin */
  GPIO_InitStruct.Pin = KEY_UP_Pin|KEY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_BOOT_Pin */
  GPIO_InitStruct.Pin = KEY_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_BOOT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_CTRL_Pin */
  GPIO_InitStruct.Pin = USB_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(USB_CTRL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Y_Pin */
  GPIO_InitStruct.Pin = LED_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_Y_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin PA15_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|PA15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GDO0_Pin GDO2_Pin CO2_RDY_Pin */
  GPIO_InitStruct.Pin = GDO0_Pin|GDO2_Pin|CO2_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PM25_RESET_Pin PM25_SET_Pin */
  GPIO_InitStruct.Pin = PM25_RESET_Pin|PM25_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G2_Pin LED_G1_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_G2_Pin|LED_G1_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

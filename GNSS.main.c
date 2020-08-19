/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    int tempo;
}GNSS_loc_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UBLOX_I2C_ADDR          (0x42 << 1)
#define TESEO_I2C_ADDR          (0x3A << 1)

#define SUCCESS   1
#define ERROR     0

#define UBLOX_SIZE_MESSAGE    342
//RMC - 73 bytes +-
//VTG   38 bytes
//GGA   73 bytes
//GSA   58 bytes as vezes tem uma a mais tbm
//GSV   50 bytes as vezes tem uma mensagem a mais - tentar descobrir
//GLL   50 bytes
#define TESEO_SIZE_MESSAGE    573
//GNS   74 bytes
//GGA - 74 bytes
//GSA   51 bytes
//GSV 66 + 68 + 68 bytes
//RMC - 66 bytes
//GST   49 bytes
//PSTMSBAS 30 bytes
//PSTMCPU  +- 27 bytes


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Parse_NMEAmessage(char* NMEAmessage, GNSS_loc_t* GNSSloc) {
    char NorthorSouth;
    char EastorWest;
    float Time_float;
    int lixoint[2];
    float lixofloat;
    int MintoDegree;

    //pegando dados da string
    if (sscanf(NMEAmessage, "%f,%f,%c,%f,%c,%d,%d,%f,%f", &Time_float, &GNSSloc->latitude, &NorthorSouth, &GNSSloc->longitude, &EastorWest, &lixoint[0], &lixoint[1], &lixofloat, &GNSSloc->altitude) != 9)
        return ERROR;

    // convertendo o tempo de float para int 

    GNSSloc->tempo = (int)Time_float;

    //convertendo latitude e longitude para graus

    MintoDegree = (int)(GNSSloc->latitude * 100000) % 10000000;
    GNSSloc->latitude = ((int)GNSSloc->latitude / 100) + (float)MintoDegree / 6000000;
    if (NorthorSouth == 'S')  GNSSloc->latitude = GNSSloc->latitude * (-1);
    MintoDegree = (int)(GNSSloc->longitude * 100000) % 10000000;
    GNSSloc->longitude = ((int)GNSSloc->longitude / 100) + (float)MintoDegree / 6000000;
    if (EastorWest == 'W')  GNSSloc->longitude = GNSSloc->longitude * (-1);

    //validando data
    if (GNSSloc->latitude > 90 || GNSSloc->latitude < -90) return ERROR;
    if (GNSSloc->longitude > 180 || GNSSloc->longitude < -180) return ERROR;

    return SUCCESS;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    uint8_t ReceiveFlag;
    uint16_t i;
    uint8_t GnssUblox = UBLOX_I2C_ADDR;
    uint8_t NMEAmessage[600];
    GNSS_loc_t GNSSloc;
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
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        //getting message

        HAL_I2C_Master_Receive(&hi2c1, GnssUblox, NMEAmessage, UBLOX_SIZE_MESSAGE, HAL_MAX_DELAY);
        //looking for GGA Message in the data received
        for (i = 0; i < UBLOX_SIZE_MESSAGE; i++) {
            
            if ((char)NMEAmessage[i] == 'G' && (char)NMEAmessage[i + 1] == 'G' && (char)NMEAmessage[i + 2] == 'A') {
                //Parsing data 
                ReceiveFlag = Parse_NMEAmessage( (char*)&NMEAmessage[i + 4], &GNSSloc );
                //leaving the for loop
                break;
            }
        }
        // testing if the parsing data succeeded, to leave(if succeeded) or receive more data(if failed) 
        if (ReceiveFlag) break;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
    /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
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
    hi2c1.Init.Timing = 0x00000E14;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Acquisition de donnees et commande moteur DC pour maintenance
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
    #include "semphr.h"
    #include "cmsis_os.h"
    #include "lcd_i2c.h"
}

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
 // <--- INCLUSION POUR LA COMMUNICATION USB CDC
// Redéfinition forcée des types statiques pour le compilateur C++
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
/* USER CODE END Includes */
/* USER CODE END Includes */
typedef struct {
  float tension;
  float courant;
  float temperature;
  float vib_x;
  float vib_y;
  float vib_z;
} CapteurData_t;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */
#define FLAG_ANOMALIE_CRITIQUE 0x0001U
#define V_MOTEUR_ON_SEUIL    5.0f     // au-dessus : moteur allumé
#define V_MOTEUR_OFF_SEUIL   2.0f     // en dessous : moteur éteint (hystérésis)
#define ARRET_CONFIRM_MS     3000U  // arrêt confirmé après 3 s continues < 2 V
#define PERIODE_CSV_ARRET_MS        1000U  // PuTTY : 1 ligne/s moteur éteint (0 = à chaque paquet)
#define GELER_VALEURS_APRES_ARRET   0      // 1 = après arrêt, garder les dernières valeurs en marche
                                           // 0 = afficher les mesures en direct (V=0, I=0...)
#define LCD_COLS                    20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

volatile uint32_t g_hb_acq   = 0;   // heartbeat Acquisition_Rapide
volatile uint32_t g_hb_trait = 0;   // heartbeat Traitement_Decision
#define WATCHDOG_TIMEOUT_MS  2000U  // au-delà : on considère le système figé
/* USER CODE BEGIN PV */
volatile uint32_t index_echantillon = 0;

volatile UBaseType_t diag_marge_memoire_ia = 0;
volatile float diag_meill_score = 0.0f;
   // nombre d'inférences IA effectuées
const char* volatile diag_nom_prediction = "normal";
// --- AJOUT POUR LE LCD ---
volatile float global_derniere_tension = 0.0f;
volatile float global_dernier_courant = 0.0f;

#define T_REF_DATASET   33.88f   // T° moteur à l'arrêt au début de coax_70.csv (été)
#define T_COMP_MIN      (-5.0f)
#define T_COMP_MAX      8.0f
volatile float g_t_comp = 4.0f;

volatile uint8_t g_moteur_on = 0;     // 0 = éteint, 1 = allumé
volatile uint8_t g_ia_prete  = 0;     // 1 = au moins une inférence faite depuis l'allumage
volatile float g_vib_moy_x = 0.0f, g_vib_moy_y = 0.0f, g_vib_moy_z = 0.0f;

volatile uint8_t g_a_deja_tourne = 0;   // passe à 1 dès le premier démarrage
volatile float g_aff_v = 0.0f, g_aff_i = 0.0f, g_aff_t = 0.0f;   // valeurs affichées sur le LCD
// 0.8503 (dataset) / 0.9755 (acquisition automne)
/* USER CODE BEGIN PV */
osThreadId_t Affichage_LCDHandle;
const osThreadAttr_t Affichage_LCD_attributes = {
  .name = "Affichage_LCD",
  .stack_size = 512 * 4,                      // avant : 256 * 4
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END PV */
/* USER CODE END PV */
/* USER CODE END PV */

osThreadId_t Acquisition_RapHandle;
uint32_t Acquisition_RapBuffer[ 512 ];
osStaticThreadDef_t Acquisition_RapControlBlock;
const osThreadAttr_t Acquisition_Rap_attributes = {
  .name = "Acquisition_Rap",
  .cb_mem = &Acquisition_RapControlBlock,
  .cb_size = sizeof(Acquisition_RapControlBlock),
  .stack_mem = &Acquisition_RapBuffer[0],
  .stack_size = sizeof(Acquisition_RapBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Acquisition_Len */
osThreadId_t Acquisition_LenHandle;
uint32_t Acquisition_LenBuffer[ 256 ];
osStaticThreadDef_t Acquisition_LenControlBlock;
const osThreadAttr_t Acquisition_Len_attributes = {
  .name = "Acquisition_Len",
  .cb_mem = &Acquisition_LenControlBlock,
  .cb_size = sizeof(Acquisition_LenControlBlock),
  .stack_mem = &Acquisition_LenBuffer[0],
  .stack_size = sizeof(Acquisition_LenBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Traitement_Deci */
osThreadId_t Traitement_DeciHandle;
uint32_t Traitement_DeciBuffer[ 8192 ];
osStaticThreadDef_t Traitement_DeciControlBlock;
const osThreadAttr_t Traitement_Deci_attributes = {
  .name = "Traitement_Deci",
  .cb_mem = &Traitement_DeciControlBlock,
  .cb_size = sizeof(Traitement_DeciControlBlock),
  .stack_mem = &Traitement_DeciBuffer[0],
  .stack_size = sizeof(Traitement_DeciBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Communication */
osThreadId_t CommunicationHandle;
uint32_t CommunicationBuffer[ 512 ];
osStaticThreadDef_t CommunicationControlBlock;
const osThreadAttr_t Communication_attributes = {
  .name = "Communication",
  .cb_mem = &CommunicationControlBlock,
  .cb_size = sizeof(CommunicationControlBlock),
  .stack_mem = &CommunicationBuffer[0],
  .stack_size = sizeof(CommunicationBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Watchdog */
osThreadId_t WatchdogHandle;
uint32_t WatchdogBuffer[ 128 ];
osStaticThreadDef_t WatchdogControlBlock;
const osThreadAttr_t Watchdog_attributes = {
  .name = "Watchdog",
  .cb_mem = &WatchdogControlBlock,
  .cb_size = sizeof(WatchdogControlBlock),
  .stack_mem = &WatchdogBuffer[0],
  .stack_size = sizeof(WatchdogBuffer),
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for Queue_Capteurs */
osMessageQueueId_t Queue_CapteursHandle;
osMessageQueueId_t Queue_CommHandle;
uint8_t myQueue01Buffer[ 10 * sizeof( CapteurData_t ) ];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t Queue_Capteurs_attributes = {
  .name = "Queue_Capteurs",
  .cb_mem = &myQueue01ControlBlock,
  .cb_size = sizeof(myQueue01ControlBlock),
  .mq_mem = &myQueue01Buffer,
  .mq_size = sizeof(myQueue01Buffer)
};
/* Definitions for Mutex_UART */
osMutexId_t Mutex_UARTHandle;
osStaticMutexDef_t Mutex_UARTControlBlock;
const osMutexAttr_t Mutex_UART_attributes = {
  .name = "Mutex_UART",
  .cb_mem = &Mutex_UARTControlBlock,
  .cb_size = sizeof(Mutex_UARTControlBlock),
};



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
extern "C" void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
extern "C" void Start_Acquisition_Rapide(void *argument);
extern "C" void Start_Acquisition_Lente(void *argument);
extern "C" void Start_Traitement_Decision(void *argument);
extern "C" void Start_Communication(void *argument);
extern "C" void Start_Watchdog(void *argument);
extern "C" void Start_Affichage_LCD(void *argument);


extern char uart_buf[128];
void DWT_Init(void);
void MEMS_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();

   // démarre le watchdog matériel — irréversible jusqu'au reset
  /* USER CODE BEGIN 2 */
  // 1. Initialiser le compteur DWT pour la température
  DWT_Init();

  // 2. Initialiser le capteur de vibrations SPI
  MEMS_Init();

  // 3. Sens de rotation : Avant (PE8 = HIGH, PE10 = LOW)
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);

  // 4. CONFIGURATION DU PWM SUR PE9 (ENA) A 70% DE VITESSE
  __HAL_RCC_TIM1_CLK_ENABLE(); // Active l'horloge du Timer 1

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  htim1.Init.Prescaler = 168 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_PWM_Init(&htim1);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 700; // 70% DE VITESSE (7V sur 10V utiles)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

  // Démarrage du signal PWM sur PE9
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // --- MESSAGE D'ACCUEIL ---
  sprintf(uart_buf, "--- SYSTEME INITIALISE A 70%% DE VITESSE ---\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);

  HAL_Delay(2000);

  // --- EN-TÊTE CSV ---
  sprintf(uart_buf, "Tension_V;Courant_A;Temperature_C;Vibration_X;Vibration_Y;Vibration_Z\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 1000);
  /* USER CODE END 2 */
  // --- SCANNER I2C DE DIAGNOSTIC ---
  osDelay(100);
  sprintf(uart_buf, "\r\n--- Scan I2C en cours ---\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
  uint8_t peripherique_trouve = 0;
  for (uint8_t addr = 1; addr < 128; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 2, 10) == HAL_OK) {
          sprintf(uart_buf, "Peripherique trouve : 0x%02X\r\n", addr);
          HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
          peripherique_trouve = 1;
      }
  }
  if (!peripherique_trouve) {
      sprintf(uart_buf, "AUCUN peripherique I2C detecte !\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
  }
  sprintf(uart_buf, "--- Fin du scan ---\r\n\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
  /* Init scheduler */
  osKernelInitialize();

  /* Create the mutex(es) */
  Mutex_UARTHandle = osMutexNew(&Mutex_UART_attributes);

  /* Create the queue(s) */
  Queue_CapteursHandle = osMessageQueueNew (10, sizeof(CapteurData_t), &Queue_Capteurs_attributes);
  Queue_CommHandle = osMessageQueueNew (10, sizeof(CapteurData_t), NULL);

  /* Create the thread(s) */
  Acquisition_RapHandle = osThreadNew(Start_Acquisition_Rapide, NULL, &Acquisition_Rap_attributes);
  Acquisition_LenHandle = osThreadNew(Start_Acquisition_Lente, NULL, &Acquisition_Len_attributes);
  Traitement_DeciHandle = osThreadNew(Start_Traitement_Decision, NULL, &Traitement_Deci_attributes);
  CommunicationHandle = osThreadNew(Start_Communication, NULL, &Communication_attributes);
  WatchdogHandle = osThreadNew(Start_Watchdog, NULL, &Watchdog_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    // ... vos autres créations de threads (Task_Traitement, etc.) ...

  Affichage_LCDHandle = osThreadNew(Start_Affichage_LCD, NULL, &Affichage_LCD_attributes);

    /* USER CODE END RTOS_THREADS */
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN PV */
char uart_buf[128];
float global_temperature = 0.0f;
int16_t accel_x, accel_y, accel_z;

static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

extern "C" int run_ai_inference(void) {
    signal_t signal;
    ei_impulse_result_t result = { 0 };

    int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) return err;

    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
    return (int)res;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
extern "C" void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
extern "C" void delay_us(uint32_t us);
void PE7_Set_Output(void);
void PE7_Set_Input(void);
uint8_t DS18B20_Start(void);
void DS18B20_Write(uint8_t data);
uint8_t DS18B20_Read(void);
float DS18B20_GetTemp(void);
uint32_t Read_ADC_Channel(uint32_t channel);

// Prototypes pour le capteur de vibration (Accelerometre SPI)
void MEMS_WriteReg(uint8_t reg, uint8_t data);
uint8_t MEMS_ReadReg(uint8_t reg);
void MEMS_Init(void);
void MEMS_ReadAxes(int16_t *x, int16_t *y, int16_t *z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- Variables globales pour le capteur MEMS --- */
uint8_t mems_type = 0; // 0x3F = LIS3DSH, 0x3B = LIS302DL

/* --- Fonctions de lecture / écriture SPI --- */
void MEMS_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t txData[2] = {reg & 0x7F, data}; // Bit 7 à 0 = Ecriture
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, txData, 2, 50);
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

uint8_t MEMS_ReadReg(uint8_t reg) {
    uint8_t txData[2] = {reg | 0x80, 0x00}; // Bit 7 à 1 = Lecture
    uint8_t rxData[2] = {0};
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, 50);
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET);
    return rxData[1];
}

void MEMS_Init(void) {
    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // 1. Verification du registre WHO_AM_I (0x0F)
    mems_type = MEMS_ReadReg(0x0F);

    if (mems_type == 0x3F) { // LIS3DSH
        // CTRL_REG4 (0x20) : ODR 100Hz, Activation axes X, Y, Z
        MEMS_WriteReg(0x20, 0x67);
        // CTRL_REG5 (0x24) : Echelle +/- 2g
        MEMS_WriteReg(0x24, 0x00);
    }
    else if (mems_type == 0x3B) { // LIS302DL
        // CTRL_REG1 (0x20) : Power ON, ODR 100Hz, Axes X, Y, Z
        MEMS_WriteReg(0x20, 0x47);
    }
}

void MEMS_ReadAxes_g(float *x_g, float *y_g, float *z_g) {
    if (mems_type == 0x3F) { // LIS3DSH (16 bits)
        uint8_t xl = MEMS_ReadReg(0x28);
        uint8_t xh = MEMS_ReadReg(0x29);
        uint8_t yl = MEMS_ReadReg(0x2A);
        uint8_t yh = MEMS_ReadReg(0x2B);
        uint8_t zl = MEMS_ReadReg(0x2C);
        uint8_t zh = MEMS_ReadReg(0x2D);

        int16_t raw_x = (int16_t)((xh << 8) | xl);
        int16_t raw_y = (int16_t)((yh << 8) | yl);
        int16_t raw_z = (int16_t)((zh << 8) | zl);

        // Sensibilite LIS3DSH (+/- 2g) : 0.06 mg/LSB
        *x_g = (float)raw_x * 0.06f / 1000.0f;
        *y_g = (float)raw_y * 0.06f / 1000.0f;
        *z_g = (float)raw_z * 0.06f / 1000.0f;
    }
    else if (mems_type == 0x3B) { // LIS302DL (8 bits)
        int8_t raw_x = (int8_t)MEMS_ReadReg(0x29);
        int8_t raw_y = (int8_t)MEMS_ReadReg(0x2B);
        int8_t raw_z = (int8_t)MEMS_ReadReg(0x2D);

        // Sensibilite LIS302DL (+/- 2g) : 18 mg/LSB
        *x_g = (float)raw_x * 0.018f;
        *y_g = (float)raw_y * 0.018f;
        *z_g = (float)raw_z * 0.018f;
    }
    else {
        // Si le capteur n'est pas detecte
        *x_g = 0.0f;
        *y_g = 0.0f;
        *z_g = 0.0f;
    }
}

/* --- Gestion des delais en microsecondes (pour DS18B20) --- */
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

extern "C" void delay_us(uint32_t us) {
    uint32_t startTicks = DWT->CYCCNT;
    uint32_t targetTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTicks) < targetTicks);
}

/* --- Pilote DS18B20 (Sonde de temperature sur PE7) --- */
void PE7_Set_Output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void PE7_Set_Input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint8_t DS18B20_Start(void) {
    uint8_t Response = 0;
    PE7_Set_Output();
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
    delay_us(480);
    PE7_Set_Input();
    delay_us(80);
    if (!HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)) Response = 1;
    else Response = 0;
    delay_us(400);
    return Response;
}

void DS18B20_Write(uint8_t data) {
    for (int i=0; i<8; i++) {
        PE7_Set_Output();
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        if (data & (1<<i)) {
            delay_us(1);
            PE7_Set_Input();
            delay_us(60);
        } else {
            delay_us(60);
            PE7_Set_Input();
        }
    }
}

uint8_t DS18B20_Read(void) {
    uint8_t value = 0;
    for (int i=0; i<8; i++) {
        PE7_Set_Output();
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
        delay_us(2);
        PE7_Set_Input();
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)) {
            value |= (1<<i);
        }
        delay_us(60);
    }
    return value;
}

float DS18B20_GetTemp(void) {
    if (!DS18B20_Start()) return -999.0f;
    DS18B20_Write(0xCC);
    DS18B20_Write(0x44);
    osDelay(750);  // Laisser le temps réel à la conversion 12 bits
    if (!DS18B20_Start()) return -999.0f;
    DS18B20_Write(0xCC);
    DS18B20_Write(0xBE);
    uint8_t temp_lsb = DS18B20_Read();
    uint8_t temp_msb = DS18B20_Read();
    int16_t temp_raw = (temp_msb << 8) | temp_lsb;
    return (float)temp_raw / 16.0f;
}

/* --- Lecture ADC Multicanal (PB0 pour Courant, PB1 pour Tension) --- */
uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return val;
    }
    HAL_ADC_Stop(&hadc1);
    return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
extern "C" void MX_I2C1_Init(void)
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;

  // CORRECTION ICI : On ralentit le SPI pour que l'accéléromètre puisse suivre !
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 115200;
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

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE7 PE8 PE10 */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN Header_Start_Acquisition_Rapide */
/**
  * @brief  Function implementing the Acquisition_Rap thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Acquisition_Rapide */
/* USER CODE BEGIN Header_Start_Acquisition_Rapide */
/**
  * @brief  Function implementing the Acquisition_Rap thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Acquisition_Rapide */
/* USER CODE BEGIN Header_Start_Acquisition_Rapide */
/**
  * @brief  Function implementing the Acquisition_Rap thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Acquisition_Rapide */
extern "C" void Start_Acquisition_Rapide(void *argument)
{
  for(;;)
  {
      float vib_x, vib_y, vib_z;
      MEMS_ReadAxes_g(&vib_x, &vib_y, &vib_z);

      float vib_x_calibre = vib_x + 0.001f;   // avant : vib_x - 0.010f
      float vib_y_calibre = vib_y + 0.025f;   // avant : vib_y + 0.016f
      float vib_z_calibre = vib_z + 0.004f;   // inchangé

      uint32_t adc_current_sum = 0;
      uint32_t adc_voltage_sum = 0;
      const int NUM_SAMPLES = 40;

      // Espacement de 100µs entre échantillons : 20 x 100µs = 2ms,
      // soit ~2 périodes PWM complètes couvertes, quel que soit le point de départ
      for(int i = 0; i < NUM_SAMPLES; i++) {
          adc_current_sum += Read_ADC_Channel(ADC_CHANNEL_8);
          adc_voltage_sum += Read_ADC_Channel(ADC_CHANNEL_9);
          delay_us(100);
      }

      uint32_t adc_raw_current = adc_current_sum / NUM_SAMPLES;
      uint32_t adc_raw_voltage = adc_voltage_sum / NUM_SAMPLES;

      float v_pb0 = (adc_raw_current * 3.3f) / 4095.0f;
      float courant_brut = ((v_pb0 - 2.5f) / 0.185f) / 2.0f;
      if (courant_brut < 0.0f) courant_brut = 0.0f;

      float v_pb1 = (adc_raw_voltage * 3.3f) / 4095.0f;
      float tension_brute = v_pb1 * 5.12f;

      // PAS de calibration pour l'instant — on teste la mesure brute corrigée

      CapteurData_t mes_donnees;
      mes_donnees.tension = tension_brute;
      mes_donnees.courant = courant_brut;
      mes_donnees.temperature = global_temperature;
      mes_donnees.vib_x = vib_x_calibre;
      mes_donnees.vib_y = vib_y_calibre;
      mes_donnees.vib_z = vib_z_calibre;

      global_derniere_tension = tension_brute; // ou tension_calibree selon votre code final
      global_dernier_courant = courant_brut;
      // --- Détection moteur ON/OFF avec hystérésis + confirmation d'arrêt ---
      // --- Détection moteur ON/OFF avec hystérésis + confirmation d'arrêt ---
      static bool arret_en_cours = false;
      static uint32_t t_debut_arret = 0;
      uint32_t maintenant = osKernelGetTickCount();
      if (tension_brute > V_MOTEUR_ON_SEUIL) {
          g_moteur_on = 1;
          g_a_deja_tourne = 1;
          arret_en_cours = false;
      } else if (tension_brute < V_MOTEUR_OFF_SEUIL) {
          if (!arret_en_cours) { arret_en_cours = true; t_debut_arret = maintenant; }
          else if (g_moteur_on && (maintenant - t_debut_arret) >= ARRET_CONFIRM_MS) g_moteur_on = 0;
      } else {
          arret_en_cours = false;
      }

      // --- Valeurs lissées pour le LCD (moyenne glissante exponentielle) ---
      static bool aff_init = false;
      if (!aff_init) {
          g_aff_v = tension_brute;  g_aff_i = courant_brut;  g_aff_t = global_temperature;
          g_vib_moy_x = vib_x_calibre; g_vib_moy_y = vib_y_calibre; g_vib_moy_z = vib_z_calibre;
          aff_init = true;
      }
      // Mise à jour : toujours si pas de gel, avant le 1er démarrage (valeurs initiales),
      // ou pendant que le moteur tourne réellement (on ignore les 3 s de descente vers 0 V)
      bool maj_aff = (!GELER_VALEURS_APRES_ARRET) || (!g_a_deja_tourne)
                     || (g_moteur_on && tension_brute > V_MOTEUR_ON_SEUIL);
      if (maj_aff) {
          const float A = 0.15f;   // avant : 0.6f
          g_aff_v     = g_aff_v     + A * (tension_brute   - g_aff_v);
          g_aff_i     = g_aff_i     + A * (courant_brut    - g_aff_i);
          g_vib_moy_x = g_vib_moy_x + A * (vib_x_calibre   - g_vib_moy_x);
          g_vib_moy_y = g_vib_moy_y + A * (vib_y_calibre   - g_vib_moy_y);
          g_vib_moy_z = g_vib_moy_z + A * (vib_z_calibre   - g_vib_moy_z);
          g_aff_t     = global_temperature;
      }

      osStatus_t statut = osMessageQueuePut(Queue_CapteursHandle, &mes_donnees, 0, 0);
      g_hb_acq = osKernelGetTickCount();
      osDelay(6);
  }
}
/* USER CODE BEGIN Header_Start_Acquisition_Lente */
/**
* @brief Function implementing the Acquisition_Len thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Acquisition_Lente */
/* USER CODE BEGIN Header_Start_Acquisition_Lente */
/**
* @brief Function implementing the Acquisition_Len thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Acquisition_Lente */
float derniere_temp_valide = 27.0f; // Valeur initiale logique pour l'atelier

static uint8_t n_calib = 0; static float somme_calib = 0.0f; static bool calib_ok = false;

extern "C" void Start_Acquisition_Lente(void *argument) {
  for(;;) {
    float temp_lue = DS18B20_GetTemp();
    if (temp_lue > -100.0f && fabsf(temp_lue - derniere_temp_valide) < 5.0f) {
        global_temperature = temp_lue;
        derniere_temp_valide = temp_lue;

        if (!calib_ok) {
            if (global_derniere_tension < 1.0f) {          // moteur à l'arrêt
                somme_calib += temp_lue;
                if (++n_calib >= 5) {                      // 5 mesures = ~5 s
                    float comp = T_REF_DATASET - (somme_calib / n_calib);
                    if (comp < T_COMP_MIN) comp = T_COMP_MIN;
                    if (comp > T_COMP_MAX) comp = T_COMP_MAX;
                    g_t_comp = comp; calib_ok = true;
                    char b[80];
                    snprintf(b, sizeof(b), "[CALIB] T_amb=%.2f comp=%.2f\r\n", somme_calib / n_calib, comp);
                    osMutexAcquire(Mutex_UARTHandle, osWaitForever);
                    HAL_UART_Transmit(&huart2, (uint8_t*)b, strlen(b), 100);
                    osMutexRelease(Mutex_UARTHandle);
                }
            } else calib_ok = true;   // moteur démarré trop tôt : on garde le repli +4,0
        }
    }
    osDelay(1000);
  }
}

/* USER CODE BEGIN Header_Start_Traitement_Decision */
/**
* @brief Function implementing the Traitement_Deci thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Traitement_Decision */
/* USER CODE BEGIN Header_Start_Traitement_Decision */
/**
* @brief Function implementing the Traitement_Deci thread.
* @param argument: Not used
* @retval None
*
*/
/* USER CODE END Header_Start_Traitement_Decision */
extern osThreadId_t CommunicationHandle;
extern "C" void Start_Traitement_Decision(void *argument)
{
  CapteurData_t donnees_recues;

  // 1. Tableau rendu 'static' pour préserver la RAM, qui va accumuler les 600 valeurs
  static float features_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];


  for(;;)
  {
    // 2. Réception des données brutes depuis l'acquisition
    if (osMessageQueueGet(Queue_CapteursHandle, &donnees_recues, NULL, osWaitForever) == osOK)
    {
      // 3. Remplissage progressif du grand tableau de l'IA
        // --- Marqueur d'état sur PuTTY (une seule fois à chaque changement) ---
        g_hb_trait = osKernelGetTickCount();
    	static uint8_t etat_prec = 2;
        if (g_moteur_on != etat_prec) {
            etat_prec = g_moteur_on;
            const char* m = g_moteur_on ? "\r\n[ETAT] MOTEUR ALLUME\r\n" : "\r\n[ETAT] MOTEUR ETEINT\r\n";
            osMutexAcquire(Mutex_UARTHandle, osWaitForever);
            HAL_UART_Transmit(&huart2, (uint8_t*)m, strlen(m), 100);
            osMutexRelease(Mutex_UARTHandle);
        }

        if (!g_moteur_on) {
            // Moteur éteint : on envoie les valeurs initiales vers PuTTY, mais on n'analyse rien
            static uint32_t t_csv = 0;
            uint32_t now = osKernelGetTickCount();
            if (PERIODE_CSV_ARRET_MS == 0 || (now - t_csv) >= PERIODE_CSV_ARRET_MS) {
                t_csv = now;
                osMessageQueuePut(Queue_CommHandle, &donnees_recues, 0, 0);
            }
            index_echantillon = 0;
            g_ia_prete = 0;
            diag_nom_prediction = "vide";
            diag_meill_score = 0.0f;
            continue;
        }
      if (index_echantillon + 6 <= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
      {

    	  // puis, dans le buffer de l'IA, remplacez tension et courant bruts par tension_ia et courant_ia
    	  // la température reste : donnees_recues.temperature + g_t_comp
    	features_buffer[index_echantillon++] = donnees_recues.tension * 1.0457f;
    	features_buffer[index_echantillon++] = donnees_recues.courant * 0.8717f;
    	features_buffer[index_echantillon++] = donnees_recues.temperature + g_t_comp;
        features_buffer[index_echantillon++] = donnees_recues.vib_x;
        features_buffer[index_echantillon++] = donnees_recues.vib_y;
        features_buffer[index_echantillon++] = donnees_recues.vib_z;
      }

      // Transmission à la file dédiée à la communication (conservée de votre code)
      osMessageQueuePut(Queue_CommHandle, &donnees_recues, 0, 0);

      // 4. Si on a accumulé les 100 paquets (donc 600 valeurs), on lance l'IA
      if (index_echantillon >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
      {


        signal_t signal;
        int err = numpy::signal_from_buffer(features_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

        if (err != 0) {
          // En cas d'erreur, on remet à zéro et on attend les prochaines données
          index_echantillon = 0;
          continue;
        }

        // --- ÉTAPE 5 : EXÉCUTION DE L'INFÉRENCE IA ---
        ei_impulse_result_t result = { 0 };

        // Prendre l'heure avant le calcul
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res == EI_IMPULSE_OK)
        {
          uint16_t meill_classe_idx = 0;
          diag_meill_score = 0.0f;

          // Recherche de la classe avec la plus haute probabilité
          for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (result.classification[i].value > diag_meill_score) {
              diag_meill_score = result.classification[i].value;
              meill_classe_idx = i;
            }
          }

          diag_nom_prediction = result.classification[meill_classe_idx].label;
          g_ia_prete = 1;


          // --- DIAGNOSTIC TEMPORAIRE ---
          if (strstr(diag_nom_prediction, "vide") == NULL) {
              char diag_buf[200];
              int pos = snprintf(diag_buf, sizeof(diag_buf), "[DIAG] ");
              for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                  pos += snprintf(diag_buf + pos, sizeof(diag_buf) - pos, "%s=%.2f ",
                                   result.classification[i].label, result.classification[i].value);
              }
              snprintf(diag_buf + pos, sizeof(diag_buf) - pos, "\r\n");

              osMutexAcquire(Mutex_UARTHandle, osWaitForever);
              HAL_UART_Transmit(&huart2, (uint8_t*)diag_buf, strlen(diag_buf), 100);
              osMutexRelease(Mutex_UARTHandle);
          }

          // --- CODE D'URGENCE ---
          // Si la confiance est de 80% ou plus ET que c'est une anomalie
          if (diag_meill_score >= 0.80f && strstr(diag_nom_prediction, "vide") == NULL)
          {
              osThreadFlagsSet(CommunicationHandle, FLAG_ANOMALIE_CRITIQUE);
          }

          diag_marge_memoire_ia = uxTaskGetStackHighWaterMark(NULL);
        }

        // 5. TRÈS IMPORTANT : On remet l'index à zéro pour recommencer une nouvelle fenêtre
        index_echantillon = 0;
      }
    }
  }
}

/* USER CODE BEGIN Header_Start_Communication */
/**
* @brief Function implementing the Communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Communication */
// Assurez-vous que cette ligne est bien présente en haut de votre fichier (hors de la fonction)
// extern const char* volatile diag_nom_prediction; // (Décommentez si la variable est dans un autre fichier)



// Fonction 1 : Détermine l'état général (Normal ou Anomalie)
const char* etat_affiche(const char* label_brut)
{
    if (strstr(label_brut, "vide") != NULL) return "A VIDE (fonctionnement normal)";
    return "ANOMALIE";
}

// Fonction 2 : Détermine le type précis de l'anomalie
const char* type_anomalie_affiche(const char* label_brut)
{
    if (strstr(label_brut, "surcharge") != NULL) return "Surcharge";
    if (strstr(label_brut, "tem") != NULL)        return "Surchauffage";
    if (strstr(label_brut, "OFON") != NULL)       return "Coupure inattendue (ON/OFF)";
    if (strstr(label_brut, "coax") != NULL)       return "Probleme de coaxialite de l'arbre";
    return "Type indetermine (Label non reconnu)";
}

// Votre tâche de communication
extern "C" void Start_Communication(void *argument)
{
  CapteurData_t data_a_envoyer;
  uint32_t flags;

  for(;;)
  {
    // 1. Vérification prioritaire et non-bloquante du Flag d'urgence (Timeout = 0)
    flags = osThreadFlagsWait(FLAG_ANOMALIE_CRITIQUE, osFlagsWaitAny, 0);

    // Vérification stricte : pas d'erreur ET flag présent
    if (((flags & osFlagsError) == 0) && ((flags & FLAG_ANOMALIE_CRITIQUE) != 0))
    {
      // Verrouillage de l'UART
      osMutexAcquire(Mutex_UARTHandle, osWaitForever);

      char buffer_alerte[128];

      // Si ce n'est pas "à vide" (l'alerte est justifiée)
      if (strstr(diag_nom_prediction, "vide") == NULL) {
          snprintf(buffer_alerte, sizeof(buffer_alerte),
                   "\r\n=========================================\r\n"
                   "!!! ALERTE CRITIQUE : %s !!!\r\n"
                   "=========================================\r\n",
                   type_anomalie_affiche(diag_nom_prediction));
      } else {
          // Sécurité au cas où le flag d'anomalie se déclenche par erreur sur le label normal
          snprintf(buffer_alerte, sizeof(buffer_alerte),
                   "\r\n--- ETAT : %s ---\r\n",
                   etat_affiche(diag_nom_prediction));
      }

      // Envoi sur le port série
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer_alerte, strlen(buffer_alerte), 100);

      // Libération de l'UART
      osMutexRelease(Mutex_UARTHandle);
    }

    // 2. Lecture de la Queue avec un délai d'attente limité à 100 ms
    if (osMessageQueueGet(Queue_CommHandle, &data_a_envoyer, NULL, 100) == osOK)
    {
      sprintf(uart_buf, "%.2f;%.2f;%.2f;%.3f;%.3f;%.3f\r\n",
              data_a_envoyer.tension, data_a_envoyer.courant,
              data_a_envoyer.temperature, data_a_envoyer.vib_x,
              data_a_envoyer.vib_y, data_a_envoyer.vib_z);

      osMutexAcquire(Mutex_UARTHandle, osWaitForever);
      HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
      osMutexRelease(Mutex_UARTHandle);
    }
  }
}

/* USER CODE BEGIN Header_Start_Watchdog */
/**
* @brief Function implementing the Watchdog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Watchdog */
extern "C" void Start_Watchdog(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN Header_Start_Affichage_LCD */
/**
* @brief Function implementing the Affichage_LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Affichage_LCD */

/* USER CODE BEGIN 4 */
// (Ou juste au-dessus de la fonction Start_Affichage_LCD)

static const char* type_anomalie_lcd(const char* label)
{
    if (strstr(label, "surcharge") != NULL) return "SURCHARGE";
    if (strstr(label, "tem")       != NULL) return "SURCHAUFFE";
    if (strstr(label, "OFON")      != NULL) return "COUPURE ON/OFF";
    if (strstr(label, "coax")      != NULL) return "COAXIALITE";
    return "INCONNU";
}

// Écrit exactement 16 caractères (complète avec des espaces, coupe si trop long)
static char lcd_cache[4][LCD_COLS + 1];

static void lcd_ligne(uint8_t ligne, const char* txt)
{
    char b[LCD_COLS + 1];
    snprintf(b, sizeof(b), "%-*.*s", LCD_COLS, LCD_COLS, txt);
    if (strcmp(b, lcd_cache[ligne]) == 0) return;
    LCD_SetCursor(ligne, 0);
    if (LCD_SendString(b)) {
        strcpy(lcd_cache[ligne], b);
    } else {
        memset(lcd_cache, 0, sizeof(lcd_cache));   // tout l'écran a été effacé par la récupération : on force la réécriture de TOUT
    }
}

extern "C" void Start_Affichage_LCD(void *argument)
{
  LCD_Init(&hi2c1);
  memset(lcd_cache, 0, sizeof(lcd_cache));
  uint32_t cpt = 0;
  uint8_t echecs_i2c = 0;

  for(;;)
  {
    char t[48];

    // ---- Récupération si le bus I2C est bloqué (bruit du moteur) ----
    if (hi2c1.State != HAL_I2C_STATE_READY || __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) echecs_i2c++;
    else echecs_i2c = 0;
    if (echecs_i2c >= 3) {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
        LCD_Init(&hi2c1);
        memset(lcd_cache, 0, sizeof(lcd_cache));
        echecs_i2c = 0;
    }
    // ---- Rafraîchissement complet toutes les 10 s (efface les caractères corrompus) ----
    if ((cpt % 20) == 19) memset(lcd_cache, 0, sizeof(lcd_cache));

    int score = (int)(diag_meill_score * 100.0f);

    // ---- Ligne 1 : état / alerte + score ----
    if (!g_moteur_on) {
        lcd_ligne(0, "MOTEUR ETEINT");
    } else if (!g_ia_prete) {
        lcd_ligne(0, "ANALYSE IA...");
    } else if (strstr((const char*)diag_nom_prediction, "vide") != NULL) {
        snprintf(t, sizeof(t), "A VIDE %d%%", score);
        lcd_ligne(0, t);
    } else {
        snprintf(t, sizeof(t), "! %s %d%%", type_anomalie_lcd((const char*)diag_nom_prediction), score);
        lcd_ligne(0, t);
    }

    // ---- Ligne 2 : tension + courant + température ----
    snprintf(t, sizeof(t), "%.2fV %.3fA %.1fC", g_aff_v, g_aff_i, g_aff_t);
    lcd_ligne(1, t);

    // ---- Ligne 3 : vibrations X / Y / Z en mg ----
    snprintf(t, sizeof(t), "X:%d Y:%d Z:%d mg",
             (int)lroundf(g_vib_moy_x * 1000.0f),
             (int)lroundf(g_vib_moy_y * 1000.0f),
             (int)lroundf(g_vib_moy_z * 1000.0f));
    lcd_ligne(2, t);

    // ---- Ligne 4 : diagnostic (F = fenêtres IA, I = durée d'inférence, T = uptime) ----


    cpt++;
    osDelay(900);   // avant : 500
  }
}
/* USER CODE END 4 */

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);   // LED rouge LD5 allumée = débordement de pile
  while(1) { }
}

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
#ifdef USE_FULL_ASSERT
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

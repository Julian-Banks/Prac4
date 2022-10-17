/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

int debounce_delay = 2000; // 2000/120 000 *(1000) = 16ms

// Task 2 - dont know if I'm supposed to declare these variables here or if they are already declared somewhere else and We need to assign values/
uint32_t  NS = 512; 		// There are 512 samples in each look up table.
uint32_t F_signal = 4024; // Set the frequency of our signal to 1024Hz. Well below the cutoff frequency of 5khz and a factor of 2 to make the maths better
uint32_t TIM2CLK = 48000000; //The STM has a 48 Mhz Clock
uint32_t TIM2_Ticks = 15;		// The PWM value needs to change 512 times for every wave that comes through. If we want 1024 waves to come trough in a second the PWM
							// need to change (NS)*(F_signal) = 512*1024 times per second  = 524288 hz
							// to calculate how many clock cycles need to pass between every PWM tick: (TIM2CLK/(NS*F_signal)) = 91.552734375
							// Dont know which of these we set TIM2_ticks;



//Look Up Tables

uint8_t currentLUT = 0;
//SINE WAVE
uint32_t SineLUT[512] = {
		512,
		518,
		524,
		530,
		537,
		543,
		549,
		555,
		562,
		568,
		574,
		580,
		587,
		593,
		599,
		605,
		611,
		617,
		624,
		630,
		636,
		642,
		648,
		654,
		660,
		666,
		672,
		678,
		684,
		690,
		696,
		701,
		707,
		713,
		719,
		725,
		730,
		736,
		741,
		747,
		753,
		758,
		764,
		769,
		774,
		780,
		785,
		790,
		796,
		801,
		806,
		811,
		816,
		821,
		826,
		831,
		836,
		841,
		846,
		850,
		855,
		860,
		864,
		869,
		873,
		878,
		882,
		886,
		890,
		895,
		899,
		903,
		907,
		911,
		915,
		919,
		922,
		926,
		930,
		933,
		937,
		940,
		944,
		947,
		950,
		953,
		957,
		960,
		963,
		966,
		968,
		971,
		974,
		977,
		979,
		982,
		984,
		986,
		989,
		991,
		993,
		995,
		997,
		999,
		1001,
		1003,
		1004,
		1006,
		1008,
		1009,
		1011,
		1012,
		1013,
		1014,
		1015,
		1017,
		1017,
		1018,
		1019,
		1020,
		1021,
		1021,
		1022,
		1022,
		1022,
		1023,
		1023,
		1023,
		1023,
		1023,
		1023,
		1023,
		1022,
		1022,
		1022,
		1021,
		1021,
		1020,
		1019,
		1018,
		1017,
		1017,
		1015,
		1014,
		1013,
		1012,
		1011,
		1009,
		1008,
		1006,
		1004,
		1003,
		1001,
		999,
		997,
		995,
		993,
		991,
		989,
		986,
		984,
		982,
		979,
		977,
		974,
		971,
		968,
		966,
		963,
		960,
		957,
		953,
		950,
		947,
		944,
		940,
		937,
		933,
		930,
		926,
		922,
		919,
		915,
		911,
		907,
		903,
		899,
		895,
		890,
		886,
		882,
		878,
		873,
		869,
		864,
		860,
		855,
		850,
		846,
		841,
		836,
		831,
		826,
		821,
		816,
		811,
		806,
		801,
		796,
		790,
		785,
		780,
		774,
		769,
		764,
		758,
		753,
		747,
		741,
		736,
		730,
		725,
		719,
		713,
		707,
		701,
		696,
		690,
		684,
		678,
		672,
		666,
		660,
		654,
		648,
		642,
		636,
		630,
		624,
		617,
		611,
		605,
		599,
		593,
		587,
		580,
		574,
		568,
		562,
		555,
		549,
		543,
		537,
		530,
		524,
		518,
		512,
		505,
		499,
		493,
		486,
		480,
		474,
		468,
		461,
		455,
		449,
		443,
		436,
		430,
		424,
		418,
		412,
		406,
		399,
		393,
		387,
		381,
		375,
		369,
		363,
		357,
		351,
		345,
		339,
		333,
		327,
		322,
		316,
		310,
		304,
		298,
		293,
		287,
		282,
		276,
		270,
		265,
		259,
		254,
		249,
		243,
		238,
		233,
		227,
		222,
		217,
		212,
		207,
		202,
		197,
		192,
		187,
		182,
		177,
		173,
		168,
		163,
		159,
		154,
		150,
		145,
		141,
		137,
		133,
		128,
		124,
		120,
		116,
		112,
		108,
		104,
		101,
		97,
		93,
		90,
		86,
		83,
		79,
		76,
		73,
		70,
		66,
		63,
		60,
		57,
		55,
		52,
		49,
		46,
		44,
		41,
		39,
		37,
		34,
		32,
		30,
		28,
		26,
		24,
		22,
		20,
		19,
		17,
		15,
		14,
		12,
		11,
		10,
		9,
		8,
		6,
		6,
		5,
		4,
		3,
		2,
		2,
		1,
		1,
		1,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		1,
		1,
		1,
		2,
		2,
		3,
		4,
		5,
		6,
		6,
		8,
		9,
		10,
		11,
		12,
		14,
		15,
		17,
		19,
		20,
		22,
		24,
		26,
		28,
		30,
		32,
		34,
		37,
		39,
		41,
		44,
		46,
		49,
		52,
		55,
		57,
		60,
		63,
		66,
		70,
		73,
		76,
		79,
		83,
		86,
		90,
		93,
		97,
		101,
		104,
		108,
		112,
		116,
		120,
		124,
		128,
		133,
		137,
		141,
		145,
		150,
		154,
		159,
		163,
		168,
		173,
		177,
		182,
		187,
		192,
		197,
		202,
		207,
		212,
		217,
		222,
		227,
		233,
		238,
		243,
		249,
		254,
		259,
		265,
		270,
		276,
		282,
		287,
		293,
		298,
		304,
		310,
		316,
		322,
		327,
		333,
		339,
		345,
		351,
		357,
		363,
		369,
		375,
		381,
		387,
		393,
		399,
		406,
		412,
		418,
		424,
		430,
		436,
		443,
		449,
		455,
		461,
		468,
		474,
		480,
		486,
		493,
		499,
		505,
};
//Triangle Wave

uint32_t TriangleLUT[512]={
		4,
		8,
		12,
		16,
		20,
		24,
		28,
		32,
		36,
		40,
		44,
		48,
		52,
		56,
		60,
		64,
		68,
		72,
		76,
		80,
		84,
		88,
		92,
		96,
		100,
		104,
		108,
		112,
		116,
		120,
		124,
		128,
		132,
		136,
		140,
		144,
		148,
		152,
		156,
		160,
		164,
		168,
		172,
		176,
		180,
		184,
		188,
		192,
		196,
		200,
		204,
		208,
		212,
		216,
		220,
		224,
		228,
		232,
		236,
		240,
		244,
		248,
		252,
		256,
		260,
		264,
		268,
		272,
		276,
		280,
		284,
		288,
		292,
		296,
		300,
		304,
		308,
		312,
		316,
		320,
		324,
		328,
		332,
		336,
		340,
		344,
		348,
		352,
		356,
		360,
		364,
		368,
		372,
		376,
		380,
		384,
		388,
		392,
		396,
		400,
		404,
		408,
		412,
		416,
		420,
		424,
		428,
		432,
		436,
		440,
		444,
		448,
		452,
		456,
		460,
		464,
		468,
		472,
		476,
		480,
		484,
		488,
		492,
		496,
		500,
		504,
		508,
		512,
		515,
		519,
		523,
		527,
		531,
		535,
		539,
		543,
		547,
		551,
		555,
		559,
		563,
		567,
		571,
		575,
		579,
		583,
		587,
		591,
		595,
		599,
		603,
		607,
		611,
		615,
		619,
		623,
		627,
		631,
		635,
		639,
		643,
		647,
		651,
		655,
		659,
		663,
		667,
		671,
		675,
		679,
		683,
		687,
		691,
		695,
		699,
		703,
		707,
		711,
		715,
		719,
		723,
		727,
		731,
		735,
		739,
		743,
		747,
		751,
		755,
		759,
		763,
		767,
		771,
		775,
		779,
		783,
		787,
		791,
		795,
		799,
		803,
		807,
		811,
		815,
		819,
		823,
		827,
		831,
		835,
		839,
		843,
		847,
		851,
		855,
		859,
		863,
		867,
		871,
		875,
		879,
		883,
		887,
		891,
		895,
		899,
		903,
		907,
		911,
		915,
		919,
		923,
		927,
		931,
		935,
		939,
		943,
		947,
		951,
		955,
		959,
		963,
		967,
		971,
		975,
		979,
		983,
		987,
		991,
		995,
		999,
		1003,
		1007,
		1011,
		1015,
		1019,
		1023,
		1019,
		1015,
		1011,
		1007,
		1003,
		999,
		995,
		991,
		987,
		983,
		979,
		975,
		971,
		967,
		963,
		959,
		955,
		951,
		947,
		943,
		939,
		935,
		931,
		927,
		923,
		919,
		915,
		911,
		907,
		903,
		899,
		895,
		891,
		887,
		883,
		879,
		875,
		871,
		867,
		863,
		859,
		855,
		851,
		847,
		843,
		839,
		835,
		831,
		827,
		823,
		819,
		815,
		811,
		807,
		803,
		799,
		795,
		791,
		787,
		783,
		779,
		775,
		771,
		767,
		763,
		759,
		755,
		751,
		747,
		743,
		739,
		735,
		731,
		727,
		723,
		719,
		715,
		711,
		707,
		703,
		699,
		695,
		691,
		687,
		683,
		679,
		675,
		671,
		667,
		663,
		659,
		655,
		651,
		647,
		643,
		639,
		635,
		631,
		627,
		623,
		619,
		615,
		611,
		607,
		603,
		599,
		595,
		591,
		587,
		583,
		579,
		575,
		571,
		567,
		563,
		559,
		555,
		551,
		547,
		543,
		539,
		535,
		531,
		527,
		523,
		519,
		515,
		512,
		508,
		504,
		500,
		496,
		492,
		488,
		484,
		480,
		476,
		472,
		468,
		464,
		460,
		456,
		452,
		448,
		444,
		440,
		436,
		432,
		428,
		424,
		420,
		416,
		412,
		408,
		404,
		400,
		396,
		392,
		388,
		384,
		380,
		376,
		372,
		368,
		364,
		360,
		356,
		352,
		348,
		344,
		340,
		336,
		332,
		328,
		324,
		320,
		316,
		312,
		308,
		304,
		300,
		296,
		292,
		288,
		284,
		280,
		276,
		272,
		268,
		264,
		260,
		256,
		252,
		248,
		244,
		240,
		236,
		232,
		228,
		224,
		220,
		216,
		212,
		208,
		204,
		200,
		196,
		192,
		188,
		184,
		180,
		176,
		172,
		168,
		164,
		160,
		156,
		152,
		148,
		144,
		140,
		136,
		132,
		128,
		124,
		120,
		116,
		112,
		108,
		104,
		100,
		96,
		92,
		88,
		84,
		80,
		76,
		72,
		68,
		64,
		60,
		56,
		52,
		48,
		44,
		40,
		36,
		32,
		28,
		24,
		20,
		16,
		12,
		8,
		4,
		0,
};
//SoreTooth wave

uint32_t SawtoothLUT[512] = {
		2,
		4,
		6,
		8,
		10,
		12,
		14,
		16,
		18,
		20,
		22,
		24,
		26,
		28,
		30,
		32,
		34,
		36,
		38,
		40,
		42,
		44,
		46,
		48,
		50,
		52,
		54,
		56,
		58,
		60,
		62,
		64,
		66,
		68,
		70,
		72,
		74,
		76,
		78,
		80,
		82,
		84,
		86,
		88,
		90,
		92,
		94,
		96,
		98,
		100,
		102,
		104,
		106,
		108,
		110,
		112,
		114,
		116,
		118,
		120,
		122,
		124,
		126,
		128,
		130,
		132,
		134,
		136,
		138,
		140,
		142,
		144,
		146,
		148,
		150,
		152,
		154,
		156,
		158,
		160,
		162,
		164,
		166,
		168,
		170,
		172,
		174,
		176,
		178,
		180,
		182,
		184,
		186,
		188,
		190,
		192,
		194,
		196,
		198,
		200,
		202,
		204,
		206,
		208,
		210,
		212,
		214,
		216,
		218,
		220,
		222,
		224,
		226,
		228,
		230,
		232,
		234,
		236,
		238,
		240,
		242,
		244,
		246,
		248,
		250,
		252,
		254,
		256,
		258,
		260,
		262,
		264,
		266,
		268,
		270,
		272,
		274,
		276,
		278,
		280,
		282,
		284,
		286,
		288,
		290,
		292,
		294,
		296,
		298,
		300,
		302,
		304,
		306,
		308,
		310,
		312,
		314,
		316,
		318,
		320,
		322,
		324,
		326,
		328,
		330,
		332,
		334,
		336,
		338,
		340,
		342,
		344,
		346,
		348,
		350,
		352,
		354,
		356,
		358,
		360,
		362,
		364,
		366,
		368,
		370,
		372,
		374,
		376,
		378,
		380,
		382,
		384,
		386,
		388,
		390,
		392,
		394,
		396,
		398,
		400,
		402,
		404,
		406,
		408,
		410,
		412,
		414,
		416,
		418,
		420,
		422,
		424,
		426,
		428,
		430,
		432,
		434,
		436,
		438,
		440,
		442,
		444,
		446,
		448,
		450,
		452,
		454,
		456,
		458,
		460,
		462,
		464,
		466,
		468,
		470,
		472,
		474,
		476,
		478,
		480,
		482,
		484,
		486,
		488,
		490,
		492,
		494,
		496,
		498,
		500,
		502,
		504,
		506,
		508,
		510,
		512,
		513,
		515,
		517,
		519,
		521,
		523,
		525,
		527,
		529,
		531,
		533,
		535,
		537,
		539,
		541,
		543,
		545,
		547,
		549,
		551,
		553,
		555,
		557,
		559,
		561,
		563,
		565,
		567,
		569,
		571,
		573,
		575,
		577,
		579,
		581,
		583,
		585,
		587,
		589,
		591,
		593,
		595,
		597,
		599,
		601,
		603,
		605,
		607,
		609,
		611,
		613,
		615,
		617,
		619,
		621,
		623,
		625,
		627,
		629,
		631,
		633,
		635,
		637,
		639,
		641,
		643,
		645,
		647,
		649,
		651,
		653,
		655,
		657,
		659,
		661,
		663,
		665,
		667,
		669,
		671,
		673,
		675,
		677,
		679,
		681,
		683,
		685,
		687,
		689,
		691,
		693,
		695,
		697,
		699,
		701,
		703,
		705,
		707,
		709,
		711,
		713,
		715,
		717,
		719,
		721,
		723,
		725,
		727,
		729,
		731,
		733,
		735,
		737,
		739,
		741,
		743,
		745,
		747,
		749,
		751,
		753,
		755,
		757,
		759,
		761,
		763,
		765,
		767,
		769,
		771,
		773,
		775,
		777,
		779,
		781,
		783,
		785,
		787,
		789,
		791,
		793,
		795,
		797,
		799,
		801,
		803,
		805,
		807,
		809,
		811,
		813,
		815,
		817,
		819,
		821,
		823,
		825,
		827,
		829,
		831,
		833,
		835,
		837,
		839,
		841,
		843,
		845,
		847,
		849,
		851,
		853,
		855,
		857,
		859,
		861,
		863,
		865,
		867,
		869,
		871,
		873,
		875,
		877,
		879,
		881,
		883,
		885,
		887,
		889,
		891,
		893,
		895,
		897,
		899,
		901,
		903,
		905,
		907,
		909,
		911,
		913,
		915,
		917,
		919,
		921,
		923,
		925,
		927,
		929,
		931,
		933,
		935,
		937,
		939,
		941,
		943,
		945,
		947,
		949,
		951,
		953,
		955,
		957,
		959,
		961,
		963,
		965,
		967,
		969,
		971,
		973,
		975,
		977,
		979,
		981,
		983,
		985,
		987,
		989,
		991,
		993,
		995,
		997,
		999,
		1001,
		1003,
		1005,
		1007,
		1009,
		1011,
		1013,
		1015,
		1017,
		1019,
		1021,
		1023,

};


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  //Start the PWM timer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);


  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // check counter to determine which LUT to use

  	 HAL_DMA_Start_IT(&hdma_tim2_ch1, SineLUT,&(TIM3->CCR1), 512);
  	//Enable the DMA timer
  	   __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

EXTI0_1_IRQHandler(void){

	if(currentLUT == 2){
		currentLUT=0;
	}else{
		currentLUT++;
	}



//
	for (int i =0; i< debounce_delay; i++)
			{
				// debounce delay
			}
//End the data stream from the
	HAL_DMA_Abort_IT(&hdma_tim2_ch1);

	// depending on the counter value display the relevant function.

	  if  (currentLUT ==0 ){
	  	   HAL_DMA_Start_IT(&hdma_tim2_ch1, SineLUT,&(TIM3->CCR1), 512);
	  	   }
	  	   if (currentLUT == 1){
	  	 	  HAL_DMA_Start_IT(&hdma_tim2_ch1, TriangleLUT, &(TIM3->CCR1), 512);
	  	   }
	  	   if (currentLUT == 2){
	  	 	  HAL_DMA_Start_IT(&hdma_tim2_ch1, SawtoothLUT, &(TIM3->CCR1), 512);
	  	   }

	  	   __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);


	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); // Clear interrupt flags

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

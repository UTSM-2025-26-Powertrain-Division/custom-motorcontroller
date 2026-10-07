/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== Motor-control configuration ======================================= */
#define MOTOR_PWM_ARR              11332U      /* 170 MHz / (8499 + 1) = 20 kHz */
#define MOTOR_START_DUTY_PERCENT   0U

#define PEDAL_RAW_IDLE       4095U  // 3.036 V with a 3.3-V ADC reference
#define PEDAL_RAW_FULL        532U  // 0.429 V
#define PEDAL_DEADBAND_RAW     40U
#define PEDAL_MAX_DUTY         90U

/* DRV8353RS control pins */
#define DRV_NCS_PORT       GPIOA
#define DRV_NCS_PIN        GPIO_PIN_4
#define DRV_ENABLE_PORT    GPIOA
#define DRV_ENABLE_PIN     GPIO_PIN_6
#define DRV_NFAULT_PORT    GPIOF
#define DRV_NFAULT_PIN     GPIO_PIN_1

/* 3x-PWM inputs: INHx are TIM1 PWM outputs */
/* INHA = PA8 / TIM1_CH1
 * INHB = PA9 / TIM1_CH2
 * INHC = PA10 / TIM1_CH3
 */

/* 3x-PWM phase enables / Hi-Z controls: INLx MUST be GPIO outputs */
#define DRV_INLA_PORT      GPIOA
#define DRV_INLA_PIN       GPIO_PIN_7
#define DRV_INLB_PORT      GPIOB
#define DRV_INLB_PIN       GPIO_PIN_0
#define DRV_INLC_PORT      GPIOF
#define DRV_INLC_PIN       GPIO_PIN_0

#define PHASE_FLOAT   0U
#define PHASE_LOW     1U
#define PHASE_PWM     2U

/* Hall inputs */
#define HALL_H1_PORT       GPIOB
#define HALL_H1_PIN        GPIO_PIN_6
#define HALL_H2_PORT       GPIOA
#define HALL_H2_PIN        GPIO_PIN_15
#define HALL_H3_PORT       GPIOB
#define HALL_H3_PIN        GPIO_PIN_7

/* DRV8353 register addresses */
#define DRV_REG_FAULT1     0x00U
#define DRV_REG_VGS2       0x01U
#define DRV_REG_CTRL       0x02U
#define DRV_REG_GATE_HS    0x03U
#define DRV_REG_GATE_LS    0x04U


/* Driver Control register: PWM_MODE[6:5] = 01b => 3x PWM */
#define DRV_CTRL_3X_PWM    0x0020U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint8_t  motor_enabled = 0U;
volatile uint8_t  motor_fault_latched = 0U;
volatile uint8_t  hall_state = 0U;
volatile uint8_t  drv_spi_ok = 0U;
volatile uint16_t drv_fault1 = 0U;
volatile uint16_t drv_fault2 = 0U;
volatile uint16_t drv_ctrl_readback = 0U;
volatile uint16_t drv_gate_hs_readback = 0U;
volatile uint16_t drv_gate_ls_readback = 0U;

volatile uint8_t phase_a_cmd = PHASE_FLOAT;
volatile uint8_t phase_b_cmd = PHASE_FLOAT;
volatile uint8_t phase_c_cmd = PHASE_FLOAT;

// For hall_state debugging
volatile uint32_t hall1_irq_count = 0;
volatile uint32_t hall2_irq_count = 0;
volatile uint32_t hall3_irq_count = 0;

volatile uint16_t last_exti_pin = 0;

volatile uint32_t adc1_raw = 0;
volatile uint32_t adc2_raw = 0;


volatile float adc1_voltage = 0.0f;
volatile float adc2_voltage = 0.0f;

volatile uint32_t adc1_min = 4095;
volatile uint32_t adc1_max = 0;
volatile uint32_t adc2_min = 4095;
volatile uint32_t adc2_max = 0;

static uint32_t motor_pwm_compare = 0U;

// For CAN bus RPM data
#define MOTOR_POLE_PAIRS       23U
#define HALL_EDGES_PER_REV     (6U * MOTOR_POLE_PAIRS)  // 138

volatile uint32_t hall_period_us = 0;
volatile uint32_t last_hall_time_us = 0;
volatile uint32_t last_hall_tick_ms = 0;

volatile uint32_t pedal_raw = 0U;
volatile float pedal_voltage = 0.0f;
volatile uint8_t pedal_duty = 0U;

/* 0 = none, 1 = driver nFAULT, 2 = pedal out of range */
volatile uint8_t stop_reason = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

static uint16_t Motor_GetRPM(void);
static HAL_StatusTypeDef CAN_SendRPM(uint16_t rpm);
static void Motor_UpdateThrottle(uint8_t duty_percent);
static void MotorControl_ConfigureLowSideGPIO(void);
static void Motor_AllOff(void);
static uint8_t Motor_ReadHallState(void);
static void Motor_SetDutyPercent(uint8_t duty_percent);
static void Motor_Commutate(uint8_t hall);
static void Motor_Start(void);
static void Motor_Stop(void);

volatile uint16_t drv_pre_ctrl = 0;
volatile uint16_t drv_pre_hs   = 0;
volatile uint16_t drv_pre_ls   = 0;

volatile uint8_t hall_debug = 0U;

static uint16_t DRV8353_Transfer16(uint16_t tx_word);
static void DRV8353_WriteReg(uint8_t address, uint16_t data);
static uint16_t DRV8353_ReadReg(uint8_t address);
static uint8_t DRV8353_Init3xPWM(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ------------------------------------------------------------------------- */
/* DRV8353 SPI helpers                                                       */
/* ------------------------------------------------------------------------- */

static void Motor_UpdateThrottle(uint8_t duty_percent)
{
    Motor_SetDutyPercent(duty_percent);

    /* Immediately update whichever phase is currently PWM-driven. */
    if (motor_enabled != 0U)
    {
        if (phase_a_cmd == PHASE_PWM)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm_compare);
        }

        if (phase_b_cmd == PHASE_PWM)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_pwm_compare);
        }

        if (phase_c_cmd == PHASE_PWM)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor_pwm_compare);
        }
    }
}

static uint8_t Pedal_ToDuty(uint16_t raw)
{
    const uint32_t active_start = PEDAL_RAW_IDLE - PEDAL_DEADBAND_RAW;
    const uint32_t span = active_start - PEDAL_RAW_FULL;

    /* Pedal voltage falls as it is pressed. */
    if (raw >= active_start)
    {
        return 0U;
    }

    if (raw <= PEDAL_RAW_FULL)
    {
        return PEDAL_MAX_DUTY;
    }

    return (uint8_t)(((active_start - raw) * PEDAL_MAX_DUTY + (span / 2U)) / span);
}

static void DRV8353_CS_HighDelay(void)
{
    /* DRV8353 requires nSCS high for at least 400 ns between words.
       At 170 MHz this loop is comfortably longer than that. */
    for (volatile uint32_t i = 0U; i < 100U; i++)
    {
        __NOP();
    }
}

static uint16_t DRV8353_Transfer16(uint16_t tx_word)
{
    uint16_t rx_word = 0U;

    HAL_GPIO_WritePin(DRV_NCS_PORT, DRV_NCS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive(&hspi1,
                                (uint8_t *)&tx_word,
                                (uint8_t *)&rx_word,
                                1U,
                                100U) != HAL_OK)
    {
        HAL_GPIO_WritePin(DRV_NCS_PORT, DRV_NCS_PIN, GPIO_PIN_SET);
        return 0xFFFFU;
    }

    HAL_GPIO_WritePin(DRV_NCS_PORT, DRV_NCS_PIN, GPIO_PIN_SET);
    DRV8353_CS_HighDelay();

    return rx_word;
}

static void DRV8353_WriteReg(uint8_t address, uint16_t data)
{
    /* B15 = 0 (write), B14:B11 = address, B10:B0 = data */
    uint16_t word = (((uint16_t)address & 0x0FU) << 11) |
                    (data & 0x07FFU);
    (void)DRV8353_Transfer16(word);
}

static uint16_t DRV8353_ReadReg(uint8_t address)
{
    /* B15 = 1 (read), B14:B11 = address */
    uint16_t word = 0x8000U |
                    (((uint16_t)address & 0x0FU) << 11);
    return DRV8353_Transfer16(word) & 0x07FFU;
}

/* ------------------------------------------------------------------------- */
/* Motor phase / Hall helpers                                                */
/* ------------------------------------------------------------------------- */

static void MotorControl_ConfigureLowSideGPIO(void)
{
    /* Defensive runtime configuration.
       In CubeMX, PA7/PB0/PF0 should ALSO be configured as GPIO_Output,
       not TIM1_CH1N/CH2N/CH3N. */
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = DRV_INLA_PIN;
    HAL_GPIO_Init(DRV_INLA_PORT, &gpio);

    gpio.Pin = DRV_INLB_PIN;
    HAL_GPIO_Init(DRV_INLB_PORT, &gpio);

    gpio.Pin = DRV_INLC_PIN;
    HAL_GPIO_Init(DRV_INLC_PORT, &gpio);

    HAL_GPIO_WritePin(DRV_INLA_PORT, DRV_INLA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_INLB_PORT, DRV_INLB_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_INLC_PORT, DRV_INLC_PIN, GPIO_PIN_RESET);
}

static void Motor_AllOff(void)
{
    /* INLx = 0 forces that half-bridge to Hi-Z in DRV8353 3x-PWM mode. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);

    HAL_GPIO_WritePin(DRV_INLA_PORT, DRV_INLA_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_INLB_PORT, DRV_INLB_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV_INLC_PORT, DRV_INLC_PIN, GPIO_PIN_RESET);

    phase_a_cmd = PHASE_FLOAT;
    phase_b_cmd = PHASE_FLOAT;
    phase_c_cmd = PHASE_FLOAT;

}

static uint8_t Motor_ReadHallState(void)
{
    uint8_t h1 = (HAL_GPIO_ReadPin(HALL_H1_PORT, HALL_H1_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t h2 = (HAL_GPIO_ReadPin(HALL_H2_PORT, HALL_H2_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t h3 = (HAL_GPIO_ReadPin(HALL_H3_PORT, HALL_H3_PIN) == GPIO_PIN_SET) ? 1U : 0U;

    return (uint8_t)((h1 << 2) | (h2 << 1) | h3);
}

static void Motor_SetDutyPercent(uint8_t duty_percent)
{
    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    /* Use actual TIM1 ARR so this stays correct if ARR is later changed. */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    motor_pwm_compare = ((arr + 1U) * (uint32_t)duty_percent) / 100U;

    if (motor_pwm_compare > arr)
    {
        motor_pwm_compare = arr;
    }
}

static uint16_t Motor_GetRPM(void)
{
    uint32_t period_us;
    uint32_t last_edge_ms;

    /*
     * Copy interrupt-controlled variables into local variables.
     * Prevents the interrupt from changing them while being read.
     */
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();

    period_us = hall_period_us;
    last_edge_ms = last_hall_tick_ms;

    if (!interrupt_state)
    {
        __enable_irq();
    }

    /* No valid reading yet or motor has stopped. */
    if (period_us == 0U ||
        (HAL_GetTick() - last_edge_ms) > 500U)
    {
        return 0U;
    }

    /*
     * RPM = 60,000,000 /
     *       (138 Hall edges/revolution × period in microseconds)
     */
    uint32_t rpm =
        60000000UL /
        (HALL_EDGES_PER_REV * period_us);

    if (rpm > 65535U)
    {
        rpm = 65535U;
    }

    return (uint16_t)rpm;
}

static HAL_StatusTypeDef CAN_SendRPM(uint16_t rpm)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0};

    header.Identifier           = 0x203;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = FDCAN_DLC_BYTES_2;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    /* Send RPM as an unsigned 16-bit big-endian value. */
    data[0] = (uint8_t)(rpm >> 8);
    data[1] = (uint8_t)(rpm & 0xFFU);

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &header,
        data
    );
}

static void Motor_Commutate(uint8_t hall)
{
    if ((motor_enabled == 0U) || (motor_fault_latched != 0U))
    {
        Motor_AllOff();
        return;
    }

    hall_state = hall;

    /* Break-before-make: first float all three phases. */
    Motor_AllOff();

    /*
     * IMPORTANT: This Hall-to-phase table assumes the following valid Hall
     * sequence for one direction:
     *
     *     101 -> 001 -> 011 -> 010 -> 110 -> 100 -> repeat
     *
     * If your motor's Hall/phase wiring produces a different sequence, the
     * table MUST be remapped. 000 and 111 are treated as invalid.
     *
     * DRV8353 3x-PWM behavior used here:
     *   INLx = 0            -> phase Hi-Z
     *   INLx = 1, INHx = 0  -> phase LOW
     *   INLx = 1, INHx = 1  -> phase HIGH
     *
     * The sourcing phase's INHx is PWM. The sinking phase's INHx is held 0.
     */
    switch (hall)
    {
    case 5U: /* A+ / B- / C float */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm_compare);
        HAL_GPIO_WritePin(DRV_INLA_PORT, DRV_INLA_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV_INLB_PORT, DRV_INLB_PIN, GPIO_PIN_SET);

        phase_a_cmd = PHASE_PWM;
        phase_b_cmd = PHASE_LOW;
        phase_c_cmd = PHASE_FLOAT;
        break;


    case 1U: /* A+ / C- / B float */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_pwm_compare);
        HAL_GPIO_WritePin(DRV_INLA_PORT, DRV_INLA_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV_INLC_PORT, DRV_INLC_PIN, GPIO_PIN_SET);

        phase_a_cmd = PHASE_PWM;
        phase_b_cmd = PHASE_FLOAT;
        phase_c_cmd = PHASE_LOW;
        break;


    case 3U: /* B+ / C- / A float */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_pwm_compare);
        HAL_GPIO_WritePin(DRV_INLB_PORT, DRV_INLB_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV_INLC_PORT, DRV_INLC_PIN, GPIO_PIN_SET);

        phase_a_cmd = PHASE_FLOAT;
        phase_b_cmd = PHASE_PWM;
        phase_c_cmd = PHASE_LOW;
        break;


    case 2U: /* B+ / A- / C float */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_pwm_compare);
        HAL_GPIO_WritePin(DRV_INLA_PORT, DRV_INLA_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV_INLB_PORT, DRV_INLB_PIN, GPIO_PIN_SET);

        phase_a_cmd = PHASE_LOW;
        phase_b_cmd = PHASE_PWM;
        phase_c_cmd = PHASE_FLOAT;
        break;


    case 6U: /* C+ / A- / B float */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor_pwm_compare);
        HAL_GPIO_WritePin(DRV_INLA_PORT, DRV_INLA_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV_INLC_PORT, DRV_INLC_PIN, GPIO_PIN_SET);

        phase_a_cmd = PHASE_LOW;
        phase_b_cmd = PHASE_FLOAT;
        phase_c_cmd = PHASE_PWM;
        break;


    case 4U: /* C+ / B- / A float */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor_pwm_compare);
        HAL_GPIO_WritePin(DRV_INLB_PORT, DRV_INLB_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DRV_INLC_PORT, DRV_INLC_PIN, GPIO_PIN_SET);

        phase_a_cmd = PHASE_FLOAT;
        phase_b_cmd = PHASE_LOW;
        phase_c_cmd = PHASE_PWM;
        break;

        default: /* 000 or 111 */
            Motor_AllOff();
            break;
    }
}

/* ------------------------------------------------------------------------- */
/* DRV8353 and motor startup                                                 */
/* ------------------------------------------------------------------------- */

static uint8_t DRV8353_Init3xPWM(void)
{
    Motor_AllOff();

    /* SPI chip-select must idle high. Keep driver asleep while establishing
       safe MCU output states. */
    HAL_GPIO_WritePin(DRV_NCS_PORT, DRV_NCS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DRV_ENABLE_PORT, DRV_ENABLE_PIN, GPIO_PIN_RESET);
    HAL_Delay(2U);

    /* Wake DRV8353. Datasheet specifies SPI ready within 1 ms after ENABLE. */
    HAL_GPIO_WritePin(DRV_ENABLE_PORT, DRV_ENABLE_PIN, GPIO_PIN_SET);
    HAL_Delay(5U);

    /* READ DEFAULT REGISTERS BEFORE WRITING ANYTHING */
    drv_pre_ctrl = DRV8353_ReadReg(DRV_REG_CTRL);
    drv_pre_hs   = DRV8353_ReadReg(DRV_REG_GATE_HS);
    drv_pre_ls   = DRV8353_ReadReg(DRV_REG_GATE_LS);

    HAL_GPIO_WritePin(DRV_ENABLE_PORT, DRV_ENABLE_PIN, GPIO_PIN_SET);
    HAL_Delay(5U);

    /* Configure moderate gate-drive current for initial bring-up:
       150 mA source / 300 mA sink. These values MUST ultimately be tuned for
       your MOSFET gate charge, gate resistance, bus voltage, and switching
       behavior on an oscilloscope.

       Reg 0x03: LOCK=011 (unlocked), IDRIVEP_HS=0011, IDRIVEN_HS=0011
       Reg 0x04: CBC=1, TDRIVE=11 (4 us), IDRIVEP_LS=0011, IDRIVEN_LS=0011
    */
    DRV8353_WriteReg(DRV_REG_GATE_HS, 0x0300U);
    DRV8353_WriteReg(DRV_REG_GATE_LS, 0x0700U);

    /* Reg 0x02: PWM_MODE[6:5] = 01b => 3x PWM. */
    DRV8353_WriteReg(DRV_REG_CTRL, DRV_CTRL_3X_PWM);

    /* Verify SPI communication and the selected PWM mode. */
    drv_ctrl_readback    = DRV8353_ReadReg(DRV_REG_CTRL);
    drv_gate_hs_readback = DRV8353_ReadReg(DRV_REG_GATE_HS);
    drv_gate_ls_readback = DRV8353_ReadReg(DRV_REG_GATE_LS);

    if ((drv_ctrl_readback & 0x0060U) != DRV_CTRL_3X_PWM)
    {
        drv_spi_ok = 0U;
        Motor_AllOff();
        return 0U;
    }

    drv_spi_ok = 1U;
    return 1U;
}

static void Motor_Start(void)
{
    if ((drv_spi_ok == 0U) ||
        (HAL_GPIO_ReadPin(DRV_NFAULT_PORT, DRV_NFAULT_PIN) == GPIO_PIN_RESET))
    {
        motor_enabled = 0U;
        Motor_AllOff();
        return;
    }

    Motor_SetDutyPercent(MOTOR_START_DUTY_PERCENT);

    /* Start only TIM1 CH1/CH2/CH3. In 3x-PWM mode CH1N/CH2N/CH3N are NOT used. */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

    motor_fault_latched = 0U;
    motor_enabled = 1U;

    /* Critical startup step: commutate immediately from the PRESENT Hall state.
       Do not wait for the first Hall edge, because the rotor is stationary. */
    Motor_Commutate(Motor_ReadHallState());
}

static void Motor_Stop(void)
{
    motor_enabled = 0U;
    Motor_AllOff();
}
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();


  /* USER CODE BEGIN 2 */

  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
      Error_Handler();
  }
/* The CubeMX screenshot still assigns PA7/PB0/PF0 as TIM1_CHxN.
     For 3x PWM they should be GPIO outputs. This runtime override keeps them
     correct, but also fix the .ioc so future code generation matches. */
  MotorControl_ConfigureLowSideGPIO();

  /* Put nSCS in its inactive state before waking the driver. */
  HAL_GPIO_WritePin(DRV_NCS_PORT, DRV_NCS_PIN, GPIO_PIN_SET);

  /* Configure the DRV8353RS and verify 3x-PWM mode by SPI readback. */
  if (DRV8353_Init3xPWM() != 0U)
  {
      Motor_Start();
  }
  else
  {
      Motor_Stop();
  }
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  uint32_t last_rpm_tx_ms = 0U;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/* Safety backup: poll nFAULT in case EXTI1 is not enabled yet. */

	  if ((HAL_GetTick() - last_rpm_tx_ms) >= 20U)
	  {
	      last_rpm_tx_ms = HAL_GetTick();

	      uint16_t rpm = Motor_GetRPM();
	      (void)CAN_SendRPM(rpm);
	  }

	  hall_debug = Motor_ReadHallState();

    if ((HAL_GPIO_ReadPin(DRV_NFAULT_PORT, DRV_NFAULT_PIN) == GPIO_PIN_RESET) &&
        (motor_fault_latched == 0U))
    {
        stop_reason = 1U;
        motor_fault_latched = 1U;
        Motor_Stop();

        /* Capture the fault registers for inspection in the debugger. */
        drv_fault1 = DRV8353_ReadReg(DRV_REG_FAULT1);
        drv_fault2 = DRV8353_ReadReg(DRV_REG_VGS2);
    }

	  if ((motor_enabled != 0U) &&
	      (motor_fault_latched == 0U) &&
	      (hall_debug >= 1U) &&
	      (hall_debug <= 6U) &&
	      (phase_a_cmd == PHASE_FLOAT) &&
	      (phase_b_cmd == PHASE_FLOAT) &&
	      (phase_c_cmd == PHASE_FLOAT))
	  {
	      Motor_Commutate(hall_debug);
	  }

    HAL_ADC_Start(&hadc1);

    /* Rank 1: PA0 / ADC1_IN1 */
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc1_raw = HAL_ADC_GetValue(&hadc1);

    /* Rank 2: PA1 / ADC1_IN2 */
    HAL_ADC_PollForConversion(&hadc1, 10);
    adc2_raw = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    adc1_voltage = ((float)adc1_raw * 3.3f) / 4095.0f;
    adc2_voltage = ((float)adc2_raw * 3.3f) / 4095.0f;

    if (adc1_raw < adc1_min)
        adc1_min = adc1_raw;

    if (adc1_raw > adc1_max)
        adc1_max = adc1_raw;

    if (adc2_raw < adc2_min)
        adc2_min = adc2_raw;

    if (adc2_raw > adc2_max)
        adc2_max = adc2_raw;
    HAL_ADC_Start(&hadc2);

    if (HAL_ADC_PollForConversion(&hadc2, 10U) == HAL_OK)
    {
        pedal_raw = HAL_ADC_GetValue(&hadc2);
        pedal_voltage = ((float)pedal_raw * 3.3f) / 4095.0f;
        pedal_duty = Pedal_ToDuty((uint16_t)pedal_raw);

        if ((pedal_raw < 350U) || (pedal_raw > 4095U))
        {
            stop_reason = 2U;
            motor_fault_latched = 1U;
            Motor_Stop();
        }
        else
        {
            Motor_UpdateThrottle(pedal_duty);
        }
    }

    HAL_ADC_Stop(&hadc2);


    HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_PWM_ARR;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    last_exti_pin = GPIO_Pin;

    if (GPIO_Pin == DRV_NFAULT_PIN)
    {
    	stop_reason = 1U;
    	motor_fault_latched = 1U;
    	Motor_Stop();
        return;
    }

    if (GPIO_Pin == HALL_H1_PIN)
    {
        hall1_irq_count++;
    }

    if (GPIO_Pin == HALL_H2_PIN)
    {
        hall2_irq_count++;
    }

    if (GPIO_Pin == HALL_H3_PIN)
    {
        hall3_irq_count++;
    }

    if ((GPIO_Pin == HALL_H1_PIN) ||
        (GPIO_Pin == HALL_H2_PIN) ||
        (GPIO_Pin == HALL_H3_PIN))
    {
        uint8_t new_hall_state = Motor_ReadHallState();

        /* Record the time between valid Hall transitions for RPM. */
        if ((new_hall_state >= 1U) &&
            (new_hall_state <= 6U) &&
            (new_hall_state != hall_state))
        {
            uint32_t now_us = __HAL_TIM_GET_COUNTER(&htim2);

            if (last_hall_time_us != 0U)
            {
                hall_period_us = now_us - last_hall_time_us;
            }

            last_hall_time_us = now_us;
            last_hall_tick_ms = HAL_GetTick();
            hall_state = new_hall_state;
        }

        if ((motor_enabled != 0U) &&
            (motor_fault_latched == 0U))
        {
            Motor_Commutate(new_hall_state);
        }
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/**
  * @}
  */

/**
  * @}
  */

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

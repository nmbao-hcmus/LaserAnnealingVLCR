/**
 * @file    grbl_cnc.h
 * @brief   CNC Controller v6.4.7 - NC switch + alarm escape + raw pin debug
 *
 * v6.4.0 changes (relative to v6.3.0):
 *  Shared-pin limit support:
 *    SP1  CNC_LIMIT_SHARED_PIN flag (default 1) - 3 axes on one GPIO (PA3)
 *    SP2  Single EXTI IRQ enabled when shared, three when separate
 *    SP3  Wait-for-release after PULLOFF2 in shared mode (500ms timeout->ALARM)
 *  Safety hardening:
 *    C1   EXTI always stops timer (no is_moving condition - covers races)
 *    C2   Timer ISR checks limit during homing PULLOFF (catches dir config bugs)
 *    C3   $21 hard_limit cannot be disabled when LIMIT_SWITCH=1 (force=1)
 *    C4   PULLOFF hitting limit -> ALARM (wrong direction protection)
 *  Homing reliability:
 *    H2   FEED travel = max(2*pulloff, 5mm) - covers wide switch trigger zones
 *    H6   HomeAxis/HomeAll refuse when machine is busy or in ALARM
 *    H7   Clear GOWDELAY/RECT phases when entering HOMING
 *  Acceleration / Plan correctness:
 *    A1   Junction U-turn (cos_theta>0.999) -> force MIN_SPEED (no missed step)
 *    A2   Clamp cos_theta away from +/-0.999 to avoid div-by-zero
 *    A6   Short move (distance<EPSILON) uses entry_speed, not cruise (no stall)
 *  Position correctness (LOSSLESS):
 *    P1+P5 Snap target to PHYSICAL step grid (target = pos + asteps/spm * dir)
 *          eliminates floating-point drift across many short moves
 *
 * v6.3.0 changes (relative to v6.2.0):
 *   D16  TIM4 dedicated 1kHz IRQ for accel update (no main-loop jitter)
 *   U1   DMA RX buffer 256->512 + overrun detection + [RX OVERRUN] report
 *   U2.1 GetLine overflow: discard whole line + [ERROR: Line too long]
 *   U2.2 CNC_UART_GetLine_Reset() called from RESET command
 *   U2.3 Binary noise filter (chars <0x20 except whitespace)
 *   U3   USART IDLE IRQ for prompt emergency-command response
 *   S1.6 Renamed find_value/has_command/state_char -> CNC_FindValue/HasCommand/StateChar
 *   S1.7 Renamed EXTI_Limit_Common -> CNC_EXTI_LimitCommon
 *   S2   Centralized 14 magic-number constants (CNC_EPSILON_MM, CNC_INCH_TO_MM, ...)
 *   S3.1 Unified freq caps: CNC_MAX_STEP_FREQ (500kHz) for motion,
 *        CNC_HOMING_MAX_STEP_FREQ (50kHz) for homing (was inconsistent)
 *   B12  Bresenham now int32 by default (faster ISR, ~30 cycles saved/tick)
 *
 * v6.2.0 fixes (relative to v6.1.0):
 * Critical:
 *   B6  Timer ARR update now force-reloads when CNT >= new_arr (no jitter spike)
 *   D1  Double-OK eliminated via cnc_planner_executing filter
 *   D2  GOWDELAY/GOWDELAYRECT zero-step jump now updates position correctly
 *   D3  G92 drains planner before applying offset
 *   E6  CNC_Timer_Stop pending_step access now in critical section
 * Major:
 *   B11 Completion handler honors logical_target when backlash on
 *   C11 Homing alarm clears axes_pending to prevent ghost-rehome
 *   D10 Triangle-clamp peak now >= max(entry,exit) to avoid stall
 *   D13 Planner_AddBlock rejects when state is ALARM/LIMIT_TRIGGERED
 * Minor / Perf:
 *   A6  PAUSE clears accel.needs_recalc
 *   B7  Phase-1 with !had_any safely exits without ARR confusion
 *   B10 ISR uses cached inv_steps_per_mm (no float div in ISR)
 *   B12 Bresenham can switch to int32 (compile flag)
 *   C4/C14 Homing cycle 0-1 only, documentation aligned
 *   D11 Move_NoAccel populates all accel state
 *   E2  State transition reorders (is_moving=false BEFORE state=...)
 *   F6-F12 Dead fields/functions removed
 *   F15 PutFloat skips '.' when decimals=0
 *
 * v6.1.0 changes (relative to v6.0.4):
 * - Header and .c are now synchronized (single source of truth)
 * - Step-based acceleration (no SysTick jitter)
 * - Hard-limit ISR: scans ALL axes, disables motors, sets ALARM
 * - Hard-limit EXTI: catches switch hits even when machine is IDLE
 * - Homing: pull-off uses exact $27 distance, debounce verifies release,
 *           retry counter prevents infinite loops on stuck switch
 * - CNC_Timer_Stop now commits pending-HIGH steps to position counter
 * - CNC_Limit_IsTriggered honors CNC_LIMIT_ACTIVE_HIGH
 * - Removed dead code: CNC_max, CNC_Accel_GetCurrentFreq/Speed,
 *                      fast_sqrt/fast_inv_sqrt, CNC_StepPulseDelay,
 *                      CNC_GetStepEventsRemaining/Total wrappers,
 *                      homing cycle case 2
 */

#ifndef __GRBL_647_H
#define __GRBL_647_H

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*============================================================================
 *                    BASIC CONFIGURATION
 *============================================================================*/

#define CNC_NUM_AXES                3
#define CNC_USE_ENABLE_PIN          0
#define CNC_TIMER_SELECT            2
#define CNC_UART_SELECT             1
#define CNC_UART_BAUDRATE           115200
#define CNC_SYSTEM_CLOCK            72000000UL

#define CNC_STEP_PULSE_US           3
#define CNC_BASE_TICK_US            20

/*============================================================================
 *                    STEP PULSE CONFIGURATION
 *============================================================================*/

#define CNC_STEP_DUTY_PERCENT       90
#define CNC_STEP_MIN_LOW_US         25
#define CNC_STEP_MIN_HIGH_US        10

/*============================================================================
 *                    FEATURE ENABLES
 *============================================================================*/

#define CNC_USE_DMA_UART            1
#define CNC_USE_COMMAND_QUEUE       1
#define CNC_USE_PLANNER             1
#define CNC_USE_BACKLASH            0
#define CNC_USE_ACCELERATION        1
#define CNC_USE_HOMING              1
#define CNC_USE_LIMIT_SWITCH        1
#define CNC_USE_LIMIT_EXTI          1   /* NEW: catch limits when IDLE too */
#define CNC_USE_INT32_BRESENHAM     1   /* B12: int32 for Bresenham (faster ISR, max ~2.1G steps/move) */
#define CNC_USE_ACCEL_TIMER         1   /* D16: dedicated TIM4 @ 1kHz for accel update */
#define CNC_USE_UART_IDLE_IRQ       1   /* U3: enable USART IDLE IRQ for faster RX wakeup */
#define CNC_USE_RELAY               1
#define CNC_AUTO_REPORT_ENABLE      1

/*============================================================================
 *                    DMA UART CONFIGURATION
 *============================================================================*/

#if CNC_USE_DMA_UART

/* U1 fix: enlarged DMA RX ring buffer to reduce overrun risk during
   long main-loop bursts (status reports, planner recalc). */
#define UART_DMA_BUFFER_SIZE        512

#if (CNC_UART_SELECT == 1)
    #define CNC_UART_DMA            DMA1
    #define CNC_UART_DMA_CHANNEL    DMA1_Channel5
    #define CNC_UART_DMA_IRQn       DMA1_Channel5_IRQn
    #define CNC_UART_DMA_IRQHandler DMA1_Channel5_IRQHandler
    #define CNC_UART_DMA_FLAG_TC    DMA_ISR_TCIF5
    #define CNC_UART_DMA_FLAG_HT    DMA_ISR_HTIF5
#elif (CNC_UART_SELECT == 2)
    #define CNC_UART_DMA            DMA1
    #define CNC_UART_DMA_CHANNEL    DMA1_Channel6
    #define CNC_UART_DMA_IRQn       DMA1_Channel6_IRQn
    #define CNC_UART_DMA_IRQHandler DMA1_Channel6_IRQHandler
    #define CNC_UART_DMA_FLAG_TC    DMA_ISR_TCIF6
    #define CNC_UART_DMA_FLAG_HT    DMA_ISR_HTIF6
#elif (CNC_UART_SELECT == 3)
    #define CNC_UART_DMA            DMA1
    #define CNC_UART_DMA_CHANNEL    DMA1_Channel3
    #define CNC_UART_DMA_IRQn       DMA1_Channel3_IRQn
    #define CNC_UART_DMA_IRQHandler DMA1_Channel3_IRQHandler
    #define CNC_UART_DMA_FLAG_TC    DMA_ISR_TCIF3
    #define CNC_UART_DMA_FLAG_HT    DMA_ISR_HTIF3
#endif

#endif /* CNC_USE_DMA_UART */

/*============================================================================
 *                    COMMAND QUEUE CONFIGURATION
 *============================================================================*/

#if CNC_USE_COMMAND_QUEUE
#define CMD_QUEUE_SIZE              16
#define CMD_MAX_LENGTH              80
#endif

/*============================================================================
 *                    MOTION PLANNER CONFIGURATION
 *============================================================================*/

#if CNC_USE_PLANNER
#define PLANNER_BUFFER_SIZE         16
#define PLANNER_JUNCTION_DEVIATION  0.05f
#define PLANNER_MIN_SPEED           0.1f
#endif

/*============================================================================
 *                    BACKLASH CONFIGURATION
 *============================================================================*/

#if CNC_USE_BACKLASH
#define DEFAULT_BACKLASH_X          0.0f
#define DEFAULT_BACKLASH_Y          0.0f
#define DEFAULT_BACKLASH_Z          0.0f
#define DEFAULT_BACKLASH_ENABLED    0
#endif

/*============================================================================
 *                    INVERT MASKS (bit0=X, bit1=Y, bit2=Z)
 *============================================================================*/

#define DEFAULT_STEP_INVERT_MASK        0
#define DEFAULT_DIR_INVERT_MASK         0
#define DEFAULT_LIMIT_INVERT_MASK       3   /* working machine config: invert X/Y limit logic */
#define DEFAULT_ENABLE_INVERT           1

/*============================================================================
 *                    ACCELERATION CONFIGURATION
 *============================================================================*/

#if CNC_USE_ACCELERATION

#define DEFAULT_ACCELERATION_X          100.0f
#define DEFAULT_ACCELERATION_Y          100.0f
#define DEFAULT_ACCELERATION_Z          50.0f

#define CNC_MIN_STEP_FREQ               100

/* Step-based accel: recompute every N steps */
#define CNC_ACCEL_UPDATE_STEPS          4

#if CNC_USE_ACCEL_TIMER
/* D16: TIM4 runs at 1 kHz, IRQ priority lower than step (0) and EXTI (1).
   This isolates accel re-planning from main-loop latency (UART, planner). */
#define CNC_ACCEL_TIMER                 TIM4
#define CNC_ACCEL_TIMER_IRQn            TIM4_IRQn
#define CNC_ACCEL_TIMER_IRQHandler      TIM4_IRQHandler
#define CNC_ACCEL_TIMER_RCC             RCC_APB1ENR_TIM4EN
#define CNC_ACCEL_TIMER_HZ              1000U  /* 1 kHz update */
#define CNC_ACCEL_TIMER_PRIO            2
#endif

#endif

/*============================================================================
 *                    HOMING CONFIGURATION
 *============================================================================*/

#if CNC_USE_HOMING

#define DEFAULT_HOMING_ENABLE           1
#define DEFAULT_HOMING_DIR_MASK         0x03   /* working config: home X/Y toward negative direction */
#define DEFAULT_HOMING_AXIS_MASK        0x03   /* X(bit0)+Y(bit1) only; Z has no endstop */
#define DEFAULT_HOMING_SEEK_RATE        2500.0f
#define DEFAULT_HOMING_FEED_RATE        25.0f
#define DEFAULT_HOMING_PULLOFF          10.0f
#define DEFAULT_HOMING_DEBOUNCE_MS      25
#define HOMING_MAX_RETRY                5
#define HOMING_PULLOFF_RETRY_GAIN       1.5f   /* extra pull-off on retry */
#define HOMING_PULLOFF_GRACE_STEPS      200U   /* bỏ qua check limit trong N step đầu của pulloff
                                                  (tránh alarm do switch chưa kịp nhả cơ học) */

/*
 * Homing cycle order:
 *  0 = All axes together (NOT recommended)
 *  1 = Z first, then X, then Y (recommended)
 */
#define DEFAULT_HOMING_CYCLE            1

#endif

/*============================================================================
 *                    RELAY / SPINDLE / COOLANT
 *============================================================================*/

#if CNC_USE_RELAY
#define RELAY_SPINDLE_PORT              GPIOB
#define RELAY_SPINDLE_PIN               13

#define RELAY_COOLANT_MIST_PORT         GPIOB
#define RELAY_COOLANT_MIST_PIN          14

#define RELAY_COOLANT_FLOOD_PORT        GPIOB
#define RELAY_COOLANT_FLOOD_PIN         15

#define RELAY_ACTIVE_HIGH               1
#endif

/*============================================================================
 *                    LIMIT SWITCH CONFIGURATION
 *============================================================================*/

#if CNC_USE_LIMIT_SWITCH

/*
 * Active level: 0 = switch presses pulls pin LOW (NC to GND + pull-up)
 *               1 = switch presses pulls pin HIGH (NO to VCC + pull-down)
 * Honored by CNC_Limit_IsTriggered.
 */
/* Switch wiring (independent settings):
 * CNC_LIMIT_PULL_UP    = 1: enable internal pull-up (recommended; required
 *                           for NC switches with one side to GND)
 *                      = 0: pull-DOWN (rare, only for active-HIGH external
 *                           drive with no pull resistor)
 * CNC_LIMIT_ACTIVE_HIGH= 0: triggered when pin reads LOW
 *                           (typical NO switch wired to GND)
 *                      = 1: triggered when pin reads HIGH
 *                           (typical NC switch wired to GND - pin goes HIGH
 *                            when contact opens on press)
 *
 * Common combinations:
 *   NO switch to GND:  PULL_UP=1, ACTIVE_HIGH=0
 *   NC switch to GND:  PULL_UP=1, ACTIVE_HIGH=1   <-- your case
 *   External drive HIGH on press: PULL_UP=0, ACTIVE_HIGH=1
 */
#define CNC_LIMIT_PULL_UP               1
#define CNC_LIMIT_ACTIVE_HIGH           1   /* NC switch wiring */

/* Shared-pin mode: all 3 axes wired to ONE GPIO.
 *   0 = separate pins (PA3=X, PA4=Y, PA5=Z)
 *   1 = shared pin (all axes read LIMIT_SHARED_PIN)
 *
 * Hardware note for shared mode:
 *   Wire all NC switches in series, or all NO switches in parallel,
 *   to the single LIMIT_SHARED_PIN. Firmware cannot tell WHICH axis
 *   was hit, so homing relies on "the axis currently moving must be
 *   the one whose switch was tripped" - safe for sequential homing
 *   (CYCLE=1: Z->X->Y).
 */
#define CNC_LIMIT_SHARED_PIN            0   /* X=PA3, Y=PA4 separate; Z has no endstop */
#define LIMIT_SHARED_PIN                3       /* PA3 when shared */

#define LIMIT_PORT                      GPIOA
#if CNC_LIMIT_SHARED_PIN
  /* All three axes map to the same physical pin */
  #define LIMIT_X_PIN                   LIMIT_SHARED_PIN
  #define LIMIT_Y_PIN                   LIMIT_SHARED_PIN
  #define LIMIT_Z_PIN                   LIMIT_SHARED_PIN
#else
  #define LIMIT_X_PIN                   3   /* PA3 - X endstop */
  #define LIMIT_Y_PIN                   4   /* PA4 - Y endstop */
  #define LIMIT_Z_PIN                   5   /* Z has NO endstop - IsTriggered(2) always returns false */
  #define CNC_LIMIT_Z_NO_ENDSTOP        1   /* Z axis skipped in limit logic */
#endif

/* EXTI lines & IRQs.
 * Separate-pin mode: PA3->EXTI3, PA4->EXTI4, PA5->EXTI9_5
 * Shared-pin mode:   only one line -> derived from LIMIT_SHARED_PIN
 */
#if CNC_USE_LIMIT_EXTI
  #if CNC_LIMIT_SHARED_PIN
    #define LIMIT_SHARED_EXTI_LINE      (1U << LIMIT_SHARED_PIN)
  #else
    #define LIMIT_X_EXTI_LINE           (1U << LIMIT_X_PIN)   /* PA3 EXTI3 */
    #define LIMIT_Y_EXTI_LINE           (1U << LIMIT_Y_PIN)   /* PA4 EXTI4 */
    #define LIMIT_Z_EXTI_LINE           (1U << LIMIT_Z_PIN)   /* PA5 - NOT USED (Z no endstop) */
  #endif
#endif

#endif

/*============================================================================
 *                    AUTO REPORT
 *============================================================================*/

#if CNC_AUTO_REPORT_ENABLE
#define CNC_AUTO_REPORT_INTERVAL_MS     300

/* GRBL v1.1 defaults for new settings (G1) */
#define DEFAULT_STEP_PULSE_US           3       /* $0 */
#define DEFAULT_STEP_IDLE_MS            25      /* $1 (255 = always enabled) */
#define DEFAULT_JUNCTION_DEVIATION_MM   1.0f   /* $11 (was hardcoded 0.05) */
#endif

/*============================================================================
 *                    SOFT LIMIT
 *============================================================================*/

#define CNC_SOFT_LIMIT_DEFAULT          1
#define CNC_ALLOW_NEGATIVE_COORD        1

/*============================================================================
 *                    DEFAULT MOTION PARAMETERS
 *============================================================================*/

#define DEFAULT_STEPS_PER_MM_X          4.0f
#define DEFAULT_STEPS_PER_MM_Y          4.0f
#define DEFAULT_STEPS_PER_MM_Z          80.0f

#define DEFAULT_MAX_FEEDRATE_X          10000.0f
#define DEFAULT_MAX_FEEDRATE_Y          10000.0f
#define DEFAULT_MAX_FEEDRATE_Z          500.0f
#define DEFAULT_FEEDRATE                1000.0f

#define DEFAULT_MAX_TRAVEL_X            10000.0f
#define DEFAULT_MAX_TRAVEL_Y            10000.0f
#define DEFAULT_MAX_TRAVEL_Z            20000.0f

#define MIN_STEPS_PER_MM        1.0f
#define MAX_STEPS_PER_MM        100000.0f
#define MIN_TIMER_PERIOD_US     2
#define MAX_TIMER_PERIOD_US     65535

#define UART_BUFFER_SIZE                256
#define UART_TX_BUFFER_SIZE             256   /* U-A2: non-blocking TX */
#define GCODE_LINE_SIZE                 80

/*============================================================================
 *                    NUMERIC CONSTANTS (S2: no magic numbers)
 *============================================================================*/

/* Distance thresholds (mm) */
#define CNC_EPSILON_MM              0.001f   /* "zero distance" */
#define CNC_DIR_THRESHOLD_MM        0.0001f  /* min delta to count as direction */
#define CNC_UNIT_VEC_EPS            1e-6f    /* unit-vector "zero" component */

/* Steps_per_mm safety floor */
#define CNC_SPM_EPSILON             0.01f

/* Step frequency caps (Hz) — S3.1: HARDWARE max vs HOMING max */
#define CNC_MAX_STEP_FREQ           500000UL /* hard ceiling for any motion */
#define CNC_HOMING_MAX_STEP_FREQ    50000UL  /* slow for safety during seek */
#define CNC_MIN_STEP_FREQ_RUNTIME   10UL     /* lower bound */

/* Acceleration */
#define CNC_MIN_ACCEL_MMS2          0.1f

/* Homing extents */
#define CNC_HOMING_TRAVEL_OVERSHOOT 1.5f     /* seek travel = max_travel * 1.5 */

/* Planner */
#define PLANNER_JUNCTION_COS_THRESHOLD 0.95f /* angle > ~18 deg triggers slowdown */

/* Time conversion */
#define CNC_SEC_PER_MIN             60.0f
#define CNC_US_PER_SEC              1000000UL
#define CNC_INCH_TO_MM              25.4f
#define CNC_MAX_REPORT_INTERVAL_MS  3600000UL

/* Misc */
#define CNC_STARTUP_DELAY_LOOPS     100000U

/*============================================================================
 *                    PIN DEFINITIONS
 *============================================================================*/

#define STEP_PORT_X                     GPIOA
#define STEP_X_PIN                      0
#define STEP_PORT_Y                     GPIOA
#define STEP_Y_PIN                      1
#define STEP_PORT_Z                     GPIOA
#define STEP_Z_PIN                      2

#define DIR_PORT                        GPIOB
#define DIR_X_PIN                       3
#define DIR_Y_PIN                       4
#define DIR_Z_PIN                       5

#if CNC_USE_ENABLE_PIN
    #define EN_PORT                     GPIOB
    #define EN_X_PIN                    10
    #define EN_Y_PIN                    11
    #define EN_Z_PIN                    12
#endif

/*============================================================================
 *                    TIMER AUTO CONFIG
 *============================================================================*/

#if (CNC_TIMER_SELECT == 2)
    #define CNC_TIMER                   TIM2
    #define CNC_TIMER_IRQn              TIM2_IRQn
    #define CNC_TIMER_IRQHandler        TIM2_IRQHandler
    #define CNC_TIMER_RCC               RCC_APB1ENR_TIM2EN
#elif (CNC_TIMER_SELECT == 3)
    #define CNC_TIMER                   TIM3
    #define CNC_TIMER_IRQn              TIM3_IRQn
    #define CNC_TIMER_IRQHandler        TIM3_IRQHandler
    #define CNC_TIMER_RCC               RCC_APB1ENR_TIM3EN
#elif (CNC_TIMER_SELECT == 4)
    #define CNC_TIMER                   TIM4
    #define CNC_TIMER_IRQn              TIM4_IRQn
    #define CNC_TIMER_IRQHandler        TIM4_IRQHandler
    #define CNC_TIMER_RCC               RCC_APB1ENR_TIM4EN
#endif

/*============================================================================
 *                    UART AUTO CONFIG
 *============================================================================*/

#if (CNC_UART_SELECT == 1)
    #define CNC_UART                    USART1
    #define CNC_UART_IRQn               USART1_IRQn
    #define CNC_UART_IRQHandler         USART1_IRQHandler
    #define CNC_UART_RCC                RCC_APB2ENR_USART1EN
    #define CNC_UART_PORT               GPIOA
    #define CNC_UART_APB2               1
#elif (CNC_UART_SELECT == 2)
    #define CNC_UART                    USART2
    #define CNC_UART_IRQn               USART2_IRQn
    #define CNC_UART_IRQHandler         USART2_IRQHandler
    #define CNC_UART_RCC                RCC_APB1ENR_USART2EN
    #define CNC_UART_PORT               GPIOA
    #define CNC_UART_APB2               0
#elif (CNC_UART_SELECT == 3)
    #define CNC_UART                    USART3
    #define CNC_UART_IRQn               USART3_IRQn
    #define CNC_UART_IRQHandler         USART3_IRQHandler
    #define CNC_UART_RCC                RCC_APB1ENR_USART3EN
    #define CNC_UART_PORT               GPIOB
    #define CNC_UART_APB2               0
#endif

/*============================================================================
 *                    STEP PIN MACROS
 *============================================================================*/

#define _PIN_MASK(pin)                  (1U << (pin))

#define STEP_X_HIGH()                   (STEP_PORT_X->BSRR = _PIN_MASK(STEP_X_PIN))
#define STEP_X_LOW()                    (STEP_PORT_X->BRR  = _PIN_MASK(STEP_X_PIN))
#define STEP_Y_HIGH()                   (STEP_PORT_Y->BSRR = _PIN_MASK(STEP_Y_PIN))
#define STEP_Y_LOW()                    (STEP_PORT_Y->BRR  = _PIN_MASK(STEP_Y_PIN))
#define STEP_Z_HIGH()                   (STEP_PORT_Z->BSRR = _PIN_MASK(STEP_Z_PIN))
#define STEP_Z_LOW()                    (STEP_PORT_Z->BRR  = _PIN_MASK(STEP_Z_PIN))

/*============================================================================
 *                    TYPES
 *============================================================================*/

/*============================================================================
 *                    B12: Bresenham integer type
 *  int32 saves ~30 cycles per timer ISR on Cortex-M3 (no 64-bit ALU)
 *  - int32 max ~2.1e9 steps  -> ~2km @ 1000 steps/mm  (more than enough)
 *  - int64 max ~9.2e18 steps -> safety for extreme configurations
 *============================================================================*/
#if CNC_USE_INT32_BRESENHAM
  typedef int32_t bresenham_int_t;
  #define MAX_STEPS_PER_MOVE   2000000000L   /* < 2^31, leaves headroom */
#else
  typedef int64_t bresenham_int_t;
  #define MAX_STEPS_PER_MOVE   100000000LL   /* legacy default */
#endif

typedef enum {
    CNC_STATE_IDLE = 0,
    CNC_STATE_RUN,
    CNC_STATE_HOLD,
    CNC_STATE_HOMING,
    CNC_STATE_ALARM,
    CNC_STATE_LIMIT_TRIGGERED,
    CNC_STATE_GOWDELAY,
    CNC_STATE_GOWDELAY_RECT
} CNC_State_t;

typedef enum { CNC_COORD_ABSOLUTE = 0, CNC_COORD_INCREMENTAL } CNC_CoordMode_t;
typedef enum { CNC_UNIT_MM = 0, CNC_UNIT_INCH } CNC_Unit_t;

#if CNC_USE_DMA_UART
typedef struct {
    uint8_t buffer[UART_DMA_BUFFER_SIZE];
    volatile uint16_t write_pos;
    volatile uint16_t read_pos;
} CNC_DMA_Buffer_t;
#endif

#if CNC_USE_COMMAND_QUEUE
typedef struct {
    char commands[CMD_QUEUE_SIZE][CMD_MAX_LENGTH];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} CNC_CommandQueue_t;
#endif

#if CNC_USE_PLANNER
typedef struct {
    float target[CNC_NUM_AXES];
    float feedrate;
    bool rapid;
    float distance;
    float unit_vec[CNC_NUM_AXES];
    float entry_speed_sqr;
    float max_entry_speed_sqr;
    float exit_speed_sqr;
    float max_exit_speed_sqr;
    float nominal_speed;
    float nominal_speed_sqr;
    float acceleration;
    uint8_t recalculate_flag;
    uint8_t nominal_length_flag;
} PlannerBlock_t;

typedef struct {
    PlannerBlock_t blocks[PLANNER_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
    float previous_unit_vec[CNC_NUM_AXES];
    float previous_nominal_speed;
} MotionPlanner_t;
#endif

#if CNC_USE_BACKLASH
typedef struct {
    float backlash[CNC_NUM_AXES];
    int8_t last_direction[CNC_NUM_AXES];
    bool enabled;
} BacklashComp_t;
#endif

#if CNC_USE_HOMING
typedef enum {
    HOMING_IDLE = 0,
    HOMING_SEEK,
    HOMING_PULLOFF1,
    HOMING_PULLOFF1_DEBOUNCE,
    HOMING_FEED,
    HOMING_PULLOFF2,
    HOMING_COMPLETE
} HomingPhase_t;
#endif

#if CNC_USE_ACCELERATION
typedef enum {
    ACCEL_PHASE_ACCEL = 0,
    ACCEL_PHASE_CRUISE,
    ACCEL_PHASE_DECEL
} AccelPhase_t;

typedef struct {
    AccelPhase_t phase;
    float current_speed;
    float target_speed;
    float entry_speed;
    float exit_speed;
    float accel_rate;
    uint32_t accel_steps;
    uint32_t cruise_steps;
    uint32_t decel_steps;
    uint32_t decel_start;
    uint32_t last_update_step_done;
    float mm_per_step;
    float spm_dominant;
    float inv_spm_dominant;
    volatile uint32_t precomp_high_ticks;
    volatile uint32_t precomp_low_ticks;
    volatile uint32_t precomp_period;
    volatile uint8_t  needs_recalc;
} AccelState_t;
#endif

typedef enum {
    GOWDELAY_IDLE = 0, GOWDELAY_MOVING, GOWDELAY_WAITING, GOWDELAY_COMPLETE
} GoWDelayPhase_t;

typedef struct {
    GoWDelayPhase_t phase;
    float start_pos[CNC_NUM_AXES];
    float target_pos[CNC_NUM_AXES];
    float unit_vector[CNC_NUM_AXES];
    float total_distance;
    float step_distance;
    float distance_moved;
    uint32_t delay_ms;
    float feedrate;
    uint32_t current_step;
    uint32_t total_steps;
    uint32_t delay_start_tick;
} GoWDelayState_t;

typedef enum {
    GWDRECT_IDLE = 0, GWDRECT_MOVE_X, GWDRECT_WAIT_X,
    GWDRECT_MOVE_Y, GWDRECT_WAIT_Y, GWDRECT_COMPLETE
} GoWDelayRectPhase_t;

typedef struct {
    GoWDelayRectPhase_t phase;
    float start_pos[CNC_NUM_AXES];
    float rect_size_x;
    float rect_size_y;
    float step_distance;
    uint32_t delay_ms;
    float feedrate;
    float current_x;
    float current_y;
    int8_t x_direction;
    uint32_t current_row;
    uint32_t total_rows;
    uint32_t current_step_in_row;
    uint32_t steps_per_row;
    uint32_t total_steps;
    uint32_t completed_steps;
    uint32_t delay_start_tick;
} GoWDelayRectState_t;

typedef struct {
    float steps_per_mm[CNC_NUM_AXES];
    float max_feedrate[CNC_NUM_AXES];
    float max_travel[CNC_NUM_AXES];
    float acceleration[CNC_NUM_AXES];
    uint8_t step_invert_mask;
    uint8_t dir_invert_mask;
    uint8_t limit_invert_mask;
    uint8_t enable_invert;
    bool soft_limit_enabled;
    bool allow_negative;
    bool hard_limit_enabled;
#if CNC_USE_HOMING
    bool homing_enabled;
    uint8_t homing_dir_mask;
    uint8_t homing_axis_mask;
    float homing_seek_rate;
    float homing_feed_rate;
    float homing_pulloff;
    uint8_t homing_cycle;
    uint16_t homing_debounce_ms;
#endif
#if CNC_USE_BACKLASH
    float backlash[CNC_NUM_AXES];
    bool backlash_enabled;
#endif
    uint32_t auto_report_interval;

    /* GRBL v1.1 standard settings (G1) */
    uint16_t step_pulse_us;       /* $0 - step pulse time (us) */
    uint16_t step_idle_ms;        /* $1 - step idle delay (ms, 255=keep enabled) */
    float    junction_deviation;  /* $11 - planner junction deviation (mm) */
} CNC_Config_t;

typedef struct {
    CNC_State_t state;
    float position[CNC_NUM_AXES];
    float target[CNC_NUM_AXES];
    volatile int32_t step_count[CNC_NUM_AXES];
    /* removed: steps_total (dead) */
    int8_t direction[CNC_NUM_AXES];
    float feedrate;
    CNC_CoordMode_t coord_mode;
    CNC_Unit_t unit;
    volatile bool is_moving;
    bool motors_enabled;
    bool limit_triggered[CNC_NUM_AXES];
    bool spindle_on;
    bool coolant_mist_on;
    bool coolant_flood_on;
#if CNC_USE_HOMING
    HomingPhase_t homing_phase;
    uint8_t homing_axis;
    /* removed: homing_cycle_step (dead) */
    uint8_t homing_axes_pending;
    bool homed[CNC_NUM_AXES];
    uint8_t homing_retry_count;
    uint32_t homing_debounce_tick;
    uint32_t homing_pulloff_steps_done; /* số step đã chạy trong pulloff hiện tại (grace counter) */
#endif
#if CNC_USE_ACCELERATION
    AccelState_t accel;
#endif
    GoWDelayState_t gowdelay;
    GoWDelayRectState_t gowdelay_rect;
    float logical_delta[CNC_NUM_AXES];
    /* removed: logical_steps_total (dead) */
    uint8_t dominant_axis;
    float move_distance;

    /* v6.2.0: cached inverse steps_per_mm to avoid float division in ISR */
    float inv_steps_per_mm[CNC_NUM_AXES];

    /* v6.2.0: logical target (pre-backlash) for completion handler */
    float logical_target[CNC_NUM_AXES];
} CNC_Machine_t;

typedef struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} CNC_RingBuffer_t;

/*============================================================================
 *                    GLOBALS
 *============================================================================*/

extern CNC_Machine_t cnc_machine;
extern CNC_Config_t  cnc_config;
extern CNC_RingBuffer_t cnc_uart_rx;
extern volatile uint32_t cnc_systick_ms;
extern volatile bool cnc_send_ok_flag;
extern volatile bool cnc_auto_report_flag;
extern volatile bool cnc_planner_executing;

/* Realtime command flags - set by UART RX path, polled by main loop */
extern volatile bool cnc_rt_status_req;   /* '?' - send status now */
extern volatile bool cnc_rt_feed_hold;    /* '!' - immediate hold */
extern volatile bool cnc_rt_resume;       /* '~' - resume from hold */
extern volatile bool cnc_rt_soft_reset;

/* Alarm-escape: when true, ONE next move is allowed to ignore limit hits
   so the user can jog the head off a tripped switch. Auto-cleared. */
/* cnc_unlock_active: set by CLEAR/$X.
 * Cho phép DUY NHẤT 1 lệnh move thoát khỏi endstop.
 * Sau khi lệnh đó kết thúc (hoặc vẫn chạm endstop), flag bị clear ngay.
 * Nếu move kết thúc mà switch vẫn còn tripped -> ALARM lại.
 */
extern volatile bool cnc_unlock_active;
extern volatile bool cnc_unlock_move_started; /* true sau khi move dau tien trong unlock da bat dau */

/*============================================================================
 *                    API
 *============================================================================*/

void CNC_Init(void);
void CNC_SystemClock_Config(void);
void CNC_LoadDefaultConfig(void);

void CNC_UART_PutChar(char c);
void CNC_UART_PutString(const char* str);
void CNC_UART_PutInt(int32_t num);
void CNC_UART_PutFloat(float num, uint8_t decimals);
bool CNC_UART_Available(void);
char CNC_UART_GetChar(void);
bool CNC_UART_GetLine(char* line, uint16_t max_len);
void CNC_UART_GetLine_Reset(void);   /* U2.2: flush partial line on RESET */
bool CNC_UART_DMA_HasOverrun(void);   /* U1: read & clear overrun flag */
void CNC_UART_SendOK(void);
void CNC_UART_SendError(uint8_t code);

#if CNC_USE_DMA_UART
void CNC_UART_DMA_Init(void);
uint16_t CNC_UART_DMA_Available(void);
uint8_t CNC_UART_DMA_Read(void);
void CNC_UART_DMA_Flush(void);
#endif

#if CNC_USE_COMMAND_QUEUE
void CNC_Queue_Init(void);
bool CNC_Queue_Push(const char* cmd);
bool CNC_Queue_Pop(char* cmd);
bool CNC_Queue_IsFull(void);
bool CNC_Queue_IsEmpty(void);
uint8_t CNC_Queue_Count(void);
uint8_t CNC_Queue_Available(void);
void CNC_Queue_Clear(void);
#endif

#if CNC_USE_PLANNER
void CNC_Planner_Init(void);
bool CNC_Planner_AddBlock(float* target, float feedrate, bool rapid);
PlannerBlock_t* CNC_Planner_GetCurrentBlock(void);
void CNC_Planner_DiscardCurrentBlock(void);
bool CNC_Planner_IsBufferEmpty(void);
bool CNC_Planner_IsBufferFull(void);
uint8_t CNC_Planner_BlocksAvailable(void);
uint8_t CNC_Planner_BlocksCount(void);
void CNC_Planner_Recalculate(void);
void CNC_Planner_Clear(void);
#endif

#if CNC_USE_BACKLASH
void  CNC_Backlash_Init(void);
void  CNC_Backlash_SetValue(uint8_t axis, float value);
float CNC_Backlash_GetValue(uint8_t axis);
void  CNC_Backlash_Enable(bool enable);
bool  CNC_Backlash_IsEnabled(void);
void  CNC_Backlash_Compensate(uint8_t axis, int8_t new_direction, float* delta);
void  CNC_Backlash_ResetDirections(void);
#endif

void CNC_Step_Pulse(uint8_t axis, bool active);
void CNC_Step_AllLow(void);
void CNC_SetDirection(uint8_t axis, int8_t dir);
void CNC_EnableMotors(bool enable);

void CNC_Move(float* target, float feedrate, bool rapid);
void CNC_Move_NoAccel(float* target, float feedrate);
bool CNC_IsMoving(void);
void CNC_Stop(void);
void CNC_WaitComplete(void);
void CNC_SetPosition(float* pos);
void CNC_SetAxisPosition(uint8_t axis, float value);
void CNC_GetPosition(float* pos);

#if CNC_USE_ACCELERATION
void CNC_Accel_Plan(float distance, float feedrate, float accel);
void CNC_Accel_PlanWithEntryExit(float distance, float feedrate, float accel,
                                 float entry_speed, float exit_speed);
void CNC_Accel_Update(void);
#endif

#if CNC_USE_HOMING
void CNC_HomeAll(void);
void CNC_HomeAxis(uint8_t axis);
void CNC_Homing_Process(void);
bool CNC_Homing_IsActive(void);
#endif

#if CNC_USE_LIMIT_SWITCH
void CNC_Limit_Init(void);
bool CNC_Limit_IsTriggered(uint8_t axis);
bool CNC_Limit_CheckAll(void);
void CNC_Limit_ClearAlarm(void);
#else
static inline bool CNC_Limit_IsTriggered(uint8_t axis) { (void)axis; return false; }
static inline bool CNC_Limit_CheckAll(void) { return false; }
static inline void CNC_Limit_ClearAlarm(void) {}
#endif

#if CNC_USE_RELAY
void CNC_Relay_Init(void);
void CNC_Spindle_On(void);
void CNC_Spindle_Off(void);
void CNC_Coolant_Mist_On(void);
void CNC_Coolant_Mist_Off(void);
void CNC_Coolant_Flood_On(void);
void CNC_Coolant_Flood_Off(void);
void CNC_Coolant_Off(void);

/* High-precision laser pulse handled inside firmware (STM32), not GUI.
 * Start with M33 D<ms> -> firmware turns spindle/laser ON, times the pulse,
 * then turns it OFF and emits [PULSE DONE] + OK.
 * Status report exposes:
 *   Sp = current spindle relay level (0/1)
 *   LP = pulse active flag + remaining ms
 */
bool CNC_LaserPulse_Start(uint32_t duration_ms);
void CNC_LaserPulse_Abort(void);
bool CNC_LaserPulse_IsActive(void);
uint32_t CNC_LaserPulse_RemainingMs(void);
#endif

void CNC_Config_Show(void);
void CNC_Config_Set(const char* param, float value);
void CNC_Config_SetStepsPerMM(uint8_t axis, float value);
void CNC_Config_SetMaxFeedrate(uint8_t axis, float value);
void CNC_Config_SetMaxTravel(uint8_t axis, float value);
void CNC_Config_SetAcceleration(uint8_t axis, float value);
void CNC_Config_SetSoftLimit(bool enable);
void CNC_Config_SetAutoReport(uint32_t interval_ms);

uint8_t CNC_Execute(const char* line);
void    CNC_ProcessCommand(const char* cmd);
void    CNC_SendStatus(void);
void    CNC_SendStatusCompact(void);

void CNC_GoWDelay_Start(float* target, float step_distance, uint32_t delay_ms, float feedrate);
void CNC_GoWDelay_Process(void);
void CNC_GoWDelay_Stop(void);
bool CNC_GoWDelay_IsActive(void);

void CNC_GoWDelayRect_Start(float size_x, float size_y, float step_distance,
                            uint32_t delay_ms, float feedrate);
void CNC_GoWDelayRect_Process(void);
void CNC_GoWDelayRect_Stop(void);
bool CNC_GoWDelayRect_IsActive(void);

void CNC_Process(void);
uint32_t CNC_GetTick(void);

float CNC_min(float a, float b);
float CNC_clamp(float value, float min_val, float max_val);

/* S3.1: enforce relationship between freq caps at compile time */
#if CNC_HOMING_MAX_STEP_FREQ > CNC_MAX_STEP_FREQ
  #error "CNC_HOMING_MAX_STEP_FREQ must not exceed CNC_MAX_STEP_FREQ"
#endif

#define CNC_VERSION_MAJOR   6
#define CNC_VERSION_MINOR   4
#define CNC_VERSION_PATCH   7
#define CNC_VERSION_STRING  "6.4.7"

#endif /* __GRBL_CNC_H */

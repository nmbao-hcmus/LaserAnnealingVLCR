/**
 * @file    grbl_cnc.c
 * @brief   CNC Controller v6.4.7 - NC switch support + alarm escape + raw pin debug
 *
 * Key fixes vs v6.0:
 *   L1  Hard-limit ISR now calls CNC_EnableMotors(false) immediately
 *   L2  Hard-limit ISR scans ALL axes (no per-axis step_count gating)
 *   L3  Homing inserts PULLOFF1_DEBOUNCE phase to wait for switch release;
 *       retry counter prevents infinite loop on stuck switch
 *   L4  CNC_Timer_Stop now commits pending HIGH steps to position counter
 *   L5  All accel struct fields/macros aligned with header (step-based)
 *   L6  SysTick_Handler is bare-metal (no HAL_IncTick)
 *   L7  CNC_Move busy-wait left as-is but documented; never called from ISR
 *   L8  CNC_Limit_IsTriggered now honors CNC_LIMIT_ACTIVE_HIGH
 *   L9  CNC_Limit_Init configures EXTI3/4/9_5 to catch hits when IDLE too
 *   L10 CNC_Homing_GetNextAxis: removed duplicate case 2
 *   L11 Homing rate uses CNC_SafeStepsPerMM
 *   L12 GoWDelay default state corruption now falls into ALARM, not IDLE
 *   D*  Removed CNC_max, GetCurrentFreq/Speed, fast_sqrt, StepPulseDelay,
 *       GetStepEventsRemaining/Total wrappers, homing cycle case 2
 */

#include "grbl_cnc.h"

/*============================================================================
 *                    GLOBALS
 *============================================================================*/

CNC_Machine_t cnc_machine;
CNC_Config_t  cnc_config;
CNC_RingBuffer_t cnc_uart_rx;

volatile uint32_t cnc_systick_ms = 0;
volatile bool cnc_send_ok_flag = false;
volatile bool cnc_auto_report_flag = false;
volatile bool cnc_planner_executing = false;

/* Realtime command flags - U-A7 */
volatile bool cnc_rt_status_req = false;
volatile bool cnc_rt_feed_hold  = false;
volatile bool cnc_rt_resume     = false;
volatile bool cnc_rt_soft_reset = false;

#if CNC_USE_RELAY
/* Timed laser pulse state (M33 D<ms>) */
static volatile bool     cnc_laser_pulse_active = false;
static volatile uint32_t cnc_laser_pulse_end_tick = 0;
static volatile uint32_t cnc_laser_pulse_duration_ms = 0;
static volatile bool     cnc_laser_pulse_done_flag = false;
static volatile bool     cnc_laser_pulse_aborted_flag = false;
#endif

/* Alarm-escape mode flag - see header doc */
volatile bool cnc_unlock_active       = false;
volatile bool cnc_unlock_move_started  = false; /* set khi move dau tien trong unlock bat dau */

/* TX ring buffer - U-A2 */
static volatile uint8_t  tx_buf[UART_TX_BUFFER_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint16_t tx_count = 0;

/*============================================================================
 *                    PRIVATE VARIABLES
 *============================================================================*/

/* Bresenham counters - width controlled by CNC_USE_INT32_BRESENHAM (B12) */
static volatile bresenham_int_t bresenham_total;
static volatile bresenham_int_t bresenham_counter[CNC_NUM_AXES];
static volatile bresenham_int_t bresenham_steps[CNC_NUM_AXES];
static volatile bresenham_int_t step_events_remaining;
/* MAX_STEPS_PER_MOVE is in header (B12) */

/* Parser */
static float current_feedrate = DEFAULT_FEEDRATE;
static bool  rapid_mode = false;

/* Auto report */
static volatile uint32_t auto_report_counter = 0;

#if CNC_USE_ACCELERATION
/* F7: removed current_step_period (dead) */
#endif

#if CNC_USE_HOMING
/* F8: removed homing_step_count (dead) */
static volatile bool     homing_limit_hit  = false;
#endif

#if CNC_USE_DMA_UART
static CNC_DMA_Buffer_t uart_dma_rx;
static volatile bool uart_dma_overrun = false;   /* U1 */
#endif

#if CNC_USE_COMMAND_QUEUE
static CNC_CommandQueue_t cmd_queue;
#endif

#if CNC_USE_PLANNER
static MotionPlanner_t planner;
static uint32_t planner_first_enqueue_tick = 0;
#endif

#if CNC_USE_BACKLASH
static BacklashComp_t backlash_comp;
#endif

/* Step pulse phase: 0=HIGH window, 1=LOW window */
static volatile uint8_t  step_phase = 0;
static volatile uint32_t step_high_ticks = 0;
static volatile uint32_t step_low_ticks  = 0;
static volatile bool     pending_step[CNC_NUM_AXES] = { false };

/*============================================================================
 *                    FORWARD DECLARATIONS
 *============================================================================*/

static void  CNC_GoWDelay_NextStep(void);
static void  CNC_GoWDelayRect_NextStepX(void);
static void  CNC_GoWDelayRect_NextStepY(void);
static bool  CNC_FindValue(const char* line, char letter, float* value);
static bool  CNC_HasCommand(const char* line, char letter, int code);
static uint32_t CNC_Step_CalcDuty(uint32_t period_us);
static void  CNC_MCU_SystemReset(void);
static void  CNC_Timer_SetPeriod(uint32_t period_us);
static void  CNC_Timer_Start(void);
static void  CNC_Timer_Stop(void);
static float CNC_ComputeVectorMaxSpeedMMs(const float* unit_vec);
static float CNC_ComputeVectorMaxAccelMMs2(const float* unit_vec);
static bool  CNC_IsPlannerStreamableLine(const char* line);
static bool  CNC_HandlePlannerStreamingInput(const char* line);
#if CNC_USE_COMMAND_QUEUE && CNC_USE_PLANNER
static void  CNC_ProcessQueuedStreamingCommands(void);
#endif
#if CNC_USE_ACCELERATION
static void  CNC_Accel_TimerTick(void);
#endif
#if CNC_USE_RELAY
static void  CNC_LaserPulse_TickIRQ(void);
#endif
static float    CNC_Config_ClampFloat(float value, float min_val, float max_val);
static uint8_t  CNC_Config_ClampMask3(float value);
static bool     CNC_Config_ClampBool(float value);
static uint32_t CNC_Config_ClampU32(float value, uint32_t min_val, uint32_t max_val);
static inline float CNC_SafeStepsPerMM(uint8_t axis);

/*============================================================================
 *                    UTILITY
 *============================================================================*/

float CNC_min(float a, float b)   { return (a < b) ? a : b; }
float CNC_clamp(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

static float CNC_Config_ClampFloat(float value, float min_val, float max_val) {
    if (value != value) return min_val;
    return CNC_clamp(value, min_val, max_val);
}
static uint8_t CNC_Config_ClampMask3(float value) {
    int32_t v = (int32_t)value;
    if (v < 0) v = 0;
    if (v > 7) v = 7;
    return (uint8_t)v;
}
static bool CNC_Config_ClampBool(float value) { return (value > 0.0f); }
static uint32_t CNC_Config_ClampU32(float value, uint32_t min_val, uint32_t max_val) {
    if (value != value) return min_val;
    if (value < (float)min_val) return min_val;
    if (value > (float)max_val) return max_val;
    return (uint32_t)value;
}

static inline float CNC_SafeStepsPerMM(uint8_t axis) {
    if (axis >= CNC_NUM_AXES) return 1.0f;
    float spm = cnc_config.steps_per_mm[axis];
    if (spm < CNC_SPM_EPSILON) spm = 1.0f;
    if (spm < MIN_STEPS_PER_MM) spm = MIN_STEPS_PER_MM;
    if (spm > MAX_STEPS_PER_MM) spm = MAX_STEPS_PER_MM;
    return spm;
}

static void CNC_ClearBufferedMotion(void) {
    cnc_send_ok_flag = false;
    cnc_planner_executing = false;
#if CNC_USE_COMMAND_QUEUE
    CNC_Queue_Clear();
#endif
#if CNC_USE_PLANNER
    CNC_Planner_Clear();
#endif
}

static void CNC_MCU_SystemReset(void) {
    /* Full STM32 chip reset via UART command.
       Different from RESET command, which only resets CNC state/config.
       Safety first: stop all motion/output, print a short acknowledgement,
       wait until UART physically transmits it, then trigger NVIC reset. */
    CNC_Timer_Stop();
#if CNC_USE_RELAY
    CNC_LaserPulse_Abort();
    CNC_Spindle_Off();
    CNC_Coolant_Off();
#endif
    cnc_machine.is_moving = false;
    CNC_ClearBufferedMotion();
    CNC_UART_PutString("[MCU RESET]\r\n");
    while (tx_count > 0) { __NOP(); }
    while (!(CNC_UART->SR & USART_SR_TC)) { __NOP(); }
    NVIC_SystemReset();
}

static float CNC_ComputeVectorMaxSpeedMMs(const float* unit_vec) {
    float max_speed = 0.0f; bool have_axis = false;
    if (unit_vec == NULL) return DEFAULT_FEEDRATE / CNC_SEC_PER_MIN;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        float u = fabsf(unit_vec[i]);
        if (u > 1e-6f) {
            float axis_speed = cnc_config.max_feedrate[i] / CNC_SEC_PER_MIN;
            float vec_speed  = axis_speed / u;
            if (!have_axis || vec_speed < max_speed) max_speed = vec_speed;
            have_axis = true;
        }
    }
    if (!have_axis) max_speed = DEFAULT_FEEDRATE / CNC_SEC_PER_MIN;
    if (max_speed < CNC_EPSILON_MM) max_speed = CNC_EPSILON_MM;
    return max_speed;
}
static float CNC_ComputeVectorMaxAccelMMs2(const float* unit_vec) {
    float max_accel = 0.0f; bool have_axis = false;
    if (unit_vec == NULL) return cnc_config.acceleration[0];
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        float u = fabsf(unit_vec[i]);
        if (u > 1e-6f) {
            float vec_accel = cnc_config.acceleration[i] / u;
            if (!have_axis || vec_accel < max_accel) max_accel = vec_accel;
            have_axis = true;
        }
    }
    if (!have_axis) max_accel = cnc_config.acceleration[0];
    if (max_accel < CNC_MIN_ACCEL_MMS2) max_accel = CNC_MIN_ACCEL_MMS2;
    return max_accel;
}

#if CNC_USE_RELAY
/*============================================================================
 *                    LASER PULSE (M33 D<ms>)
 *  - Precision comes from STM32 SysTick (1 ms), not Tkinter/Windows timers.
 *  - Non-blocking: main loop and safety logic continue to run while pulse is on.
 *  - On completion firmware emits [PULSE DONE] + OK.
 *  - On emergency/limit/reset firmware aborts pulse immediately.
 *============================================================================*/

bool CNC_LaserPulse_Start(uint32_t duration_ms) {
    /* D0/0ms means shortest supported pulse.  The GUI may use this as
       "minimum pulse", so do not reject it here. */
    if (duration_ms == 0U) duration_ms = 1U;
    if (cnc_machine.state != CNC_STATE_IDLE) return false;
    if (cnc_machine.is_moving) return false;
#if CNC_USE_LIMIT_SWITCH
    /* Do not fire laser while a limit switch is physically active.  Normally
       EXTI should already put the machine in LIMIT/ALARM, but this also covers
       boot-with-switch-pressed or missed-edge cases. */
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        if (CNC_Limit_IsTriggered(i)) return false;
    }
#endif
    if (cnc_laser_pulse_active) return false;
    if (cnc_machine.spindle_on) return false;   /* reject if manual M3 already ON */

    /* Critical section fixes a short-pulse race:
       If SysTick fired after active=true but before CNC_Spindle_On(), the
       ISR could finish the pulse then this function would turn the relay ON
       afterwards.  Also add one tick so the relay stays on for at least the
       requested number of milliseconds despite SysTick phase quantization
       (actual width is requested..requested+1 ms). */
    __disable_irq();
    cnc_laser_pulse_done_flag    = false;
    cnc_laser_pulse_aborted_flag = false;
    cnc_laser_pulse_duration_ms  = duration_ms;
    cnc_laser_pulse_end_tick     = cnc_systick_ms + duration_ms + 1U;
    cnc_laser_pulse_active       = true;
    CNC_Spindle_On();
    __enable_irq();
    return true;
}

void CNC_LaserPulse_Abort(void) {
    bool was_active = cnc_laser_pulse_active;
    cnc_laser_pulse_active      = false;
    cnc_laser_pulse_end_tick    = 0;
    cnc_laser_pulse_duration_ms = 0;
    cnc_laser_pulse_done_flag   = false;
    if (was_active) cnc_laser_pulse_aborted_flag = true;
    CNC_Spindle_Off();
}

bool CNC_LaserPulse_IsActive(void) {
    return cnc_laser_pulse_active;
}

uint32_t CNC_LaserPulse_RemainingMs(void) {
    if (!cnc_laser_pulse_active) return 0U;
    int32_t rem = (int32_t)(cnc_laser_pulse_end_tick - cnc_systick_ms);
    return (rem > 0) ? (uint32_t)rem : 0U;
}

static void CNC_LaserPulse_TickIRQ(void) {
    if (!cnc_laser_pulse_active) return;
    if ((int32_t)(cnc_systick_ms - cnc_laser_pulse_end_tick) >= 0) {
        cnc_laser_pulse_active      = false;
        cnc_laser_pulse_end_tick    = 0;
        cnc_laser_pulse_duration_ms = 0;
        CNC_Spindle_Off();
        cnc_laser_pulse_done_flag   = true;
    }
}
#endif

/*============================================================================
 *                    PLANNER STREAMING HELPERS
 *============================================================================*/

static bool CNC_IsPlannerStreamableLine(const char* line) {
#if !CNC_USE_PLANNER
    (void)line; return false;
#else
    if (line == NULL || line[0] == '\0') return false;
    const char* p = line; bool saw_token = false;
    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == ';' || *p == '(') break;
        char c = *p; if (c >= 'a' && c <= 'z') c -= 32;
        if (c == 'N' || c == 'X' || c == 'Y' || c == 'Z' || c == 'F' || c == 'G' || c == 'M') {
            char* end_ptr;
            float word_val = strtof(p + 1, &end_ptr);
            if (end_ptr == p + 1) return false;
            if (*end_ptr != '\0' && *end_ptr != ' ' && *end_ptr != '\t'
                && *end_ptr != ';' && *end_ptr != '(') return false;
            if (c == 'G') {
                int code = (int)word_val;
                if (!(code == 0 || code == 1 || code == 20 || code == 21 || code == 90 || code == 91)) return false;
            } else if (c == 'M') {
                int code = (int)word_val;
                /* Do not planner-stream M3/M5/M7/M8/M9.  They would execute
                   immediately while previous motion is still running, causing
                   spindle/laser/coolant timing conflicts.  Use M114/status only. */
                if (!(code == 114)) return false;
            }
            saw_token = true; p = end_ptr; continue;
        }
        return false;
    }
    return saw_token;
#endif
}

static bool CNC_HandlePlannerStreamingInput(const char* line) {
#if !CNC_USE_PLANNER
    (void)line; return false;
#else
    if (!CNC_IsPlannerStreamableLine(line)) return false;
#if CNC_USE_COMMAND_QUEUE
    /* Do not ACK commands into a secondary queue before semantic validation.
       A delayed ERROR after an earlier OK is a protocol conflict for senders.
       If planner is full, report buffer-full now; a well-behaved sender will
       wait for more OKs/status before sending more. */
    if (!CNC_Queue_IsEmpty() || CNC_Planner_IsBufferFull()) {
        CNC_UART_SendError(9);
        return true;
    }
#else
    if (CNC_Planner_IsBufferFull()) { CNC_UART_SendError(9); return true; }
#endif
    uint8_t err = CNC_Execute(line);
    if (err) CNC_UART_SendError(err); else CNC_UART_SendOK();
    return true;
#endif
}

#if CNC_USE_COMMAND_QUEUE && CNC_USE_PLANNER
static void CNC_ProcessQueuedStreamingCommands(void) {
    if (cnc_machine.state == CNC_STATE_HOMING ||
        cnc_machine.state == CNC_STATE_GOWDELAY ||
        cnc_machine.state == CNC_STATE_GOWDELAY_RECT ||
        cnc_machine.state == CNC_STATE_ALARM ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) return;
    uint8_t budget = 4;
    while (budget-- && !CNC_Queue_IsEmpty() && !CNC_Planner_IsBufferFull()) {
        char cmd[CMD_MAX_LENGTH];
        if (!CNC_Queue_Pop(cmd)) break;
        uint8_t err = CNC_Execute(cmd);
        if (err) CNC_UART_SendError(err);
    }
}
#endif

/*============================================================================
 *                    SYSTICK & CLOCK
 *============================================================================*/

void CNC_SysTick_Callback(void) {
    cnc_systick_ms++;
#if CNC_USE_RELAY
    CNC_LaserPulse_TickIRQ();
#endif
#if CNC_AUTO_REPORT_ENABLE
    if (cnc_config.auto_report_interval > 0) {
        if (++auto_report_counter >= cnc_config.auto_report_interval) {
            auto_report_counter = 0;
            cnc_auto_report_flag = true;
        }
    }
#endif
}

/* Bare-metal SysTick: NO HAL_IncTick */
void SysTick_Handler(void) { CNC_SysTick_Callback(); }

uint32_t CNC_GetTick(void) { return cnc_systick_ms; }

void CNC_SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
    SysTick_Config(CNC_SYSTEM_CLOCK / 1000);
    /* CMSIS SysTick_Config() commonly leaves SysTick at the lowest priority.
       M33 laser OFF is driven from SysTick, so give it higher priority than
       UART/TIM4 accel work, but keep the step timer (priority 0) above it. */
    NVIC_SetPriority(SysTick_IRQn, 1);
}

/*============================================================================
 *                    DEFAULT CONFIG
 *============================================================================*/

void CNC_LoadDefaultConfig(void) {
    cnc_config.steps_per_mm[0] = DEFAULT_STEPS_PER_MM_X;
    cnc_config.steps_per_mm[1] = DEFAULT_STEPS_PER_MM_Y;
    cnc_config.steps_per_mm[2] = DEFAULT_STEPS_PER_MM_Z;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        cnc_config.steps_per_mm[i] =
            CNC_Config_ClampFloat(cnc_config.steps_per_mm[i], MIN_STEPS_PER_MM, MAX_STEPS_PER_MM);
    }

    cnc_config.max_feedrate[0] = DEFAULT_MAX_FEEDRATE_X;
    cnc_config.max_feedrate[1] = DEFAULT_MAX_FEEDRATE_Y;
    cnc_config.max_feedrate[2] = DEFAULT_MAX_FEEDRATE_Z;

    cnc_config.max_travel[0] = DEFAULT_MAX_TRAVEL_X;
    cnc_config.max_travel[1] = DEFAULT_MAX_TRAVEL_Y;
    cnc_config.max_travel[2] = DEFAULT_MAX_TRAVEL_Z;

#if CNC_USE_ACCELERATION
    cnc_config.acceleration[0] = DEFAULT_ACCELERATION_X;
    cnc_config.acceleration[1] = DEFAULT_ACCELERATION_Y;
    cnc_config.acceleration[2] = DEFAULT_ACCELERATION_Z;
#endif

    cnc_config.step_invert_mask  = DEFAULT_STEP_INVERT_MASK;
    cnc_config.dir_invert_mask   = DEFAULT_DIR_INVERT_MASK;
    cnc_config.limit_invert_mask = DEFAULT_LIMIT_INVERT_MASK;
    cnc_config.enable_invert     = DEFAULT_ENABLE_INVERT;

    cnc_config.soft_limit_enabled = CNC_SOFT_LIMIT_DEFAULT;
    cnc_config.allow_negative     = CNC_ALLOW_NEGATIVE_COORD;
    /* C3: hard limit is mandatory when switch hardware is compiled in */
    cnc_config.hard_limit_enabled = CNC_USE_LIMIT_SWITCH ? true : false;

#if CNC_USE_HOMING
    cnc_config.homing_enabled     = DEFAULT_HOMING_ENABLE;
    cnc_config.homing_dir_mask    = DEFAULT_HOMING_DIR_MASK;
    cnc_config.homing_axis_mask   = DEFAULT_HOMING_AXIS_MASK;  /* X+Y only (0x03); Z has no endstop */
    cnc_config.homing_seek_rate   = DEFAULT_HOMING_SEEK_RATE;
    cnc_config.homing_feed_rate   = DEFAULT_HOMING_FEED_RATE;
    cnc_config.homing_pulloff     = DEFAULT_HOMING_PULLOFF;
    cnc_config.homing_cycle       = (DEFAULT_HOMING_CYCLE > 1) ? 1 : DEFAULT_HOMING_CYCLE;
    cnc_config.homing_debounce_ms = DEFAULT_HOMING_DEBOUNCE_MS;
#endif

#if CNC_USE_BACKLASH
    cnc_config.backlash[0] = DEFAULT_BACKLASH_X;
    cnc_config.backlash[1] = DEFAULT_BACKLASH_Y;
    cnc_config.backlash[2] = DEFAULT_BACKLASH_Z;
    cnc_config.backlash_enabled = DEFAULT_BACKLASH_ENABLED;
#endif

#if CNC_AUTO_REPORT_ENABLE
    cnc_config.auto_report_interval = CNC_AUTO_REPORT_INTERVAL_MS;
#else
    cnc_config.auto_report_interval = 0;
#endif

    /* G1: GRBL v1.1 standard defaults */
    cnc_config.step_pulse_us      = DEFAULT_STEP_PULSE_US;
    cnc_config.step_idle_ms       = DEFAULT_STEP_IDLE_MS;
    cnc_config.junction_deviation = DEFAULT_JUNCTION_DEVIATION_MM;
}

/*============================================================================
 *                    GPIO INIT
 *============================================================================*/

static void CNC_GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    AFIO->MAPR  |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    /* STEP outputs PA0..PA2, 50 MHz push-pull */
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); GPIOA->CRL |= GPIO_CRL_MODE0;
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1); GPIOA->CRL |= GPIO_CRL_MODE1;
    GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2); GPIOA->CRL |= GPIO_CRL_MODE2;
    CNC_Step_AllLow();

    /* DIR outputs PB3..PB5, 10 MHz push-pull */
    GPIOB->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3); GPIOB->CRL |= GPIO_CRL_MODE3_0;
    GPIOB->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4); GPIOB->CRL |= GPIO_CRL_MODE4_0;
    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5); GPIOB->CRL |= GPIO_CRL_MODE5_0;
    DIR_PORT->BRR = (1U << DIR_X_PIN) | (1U << DIR_Y_PIN) | (1U << DIR_Z_PIN);

#if CNC_USE_ENABLE_PIN
    GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10); GPIOB->CRH |= GPIO_CRH_MODE10_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11); GPIOB->CRH |= GPIO_CRH_MODE11_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12); GPIOB->CRH |= GPIO_CRH_MODE12_0;
    CNC_EnableMotors(false);
#endif
}

/*============================================================================
 *                    STEP / DIR / ENABLE
 *============================================================================*/

void CNC_Step_Pulse(uint8_t axis, bool active) {
    bool inverted = (cnc_config.step_invert_mask >> axis) & 1U;
    bool output = active ^ inverted;
    switch (axis) {
        case 0: if (output) STEP_X_HIGH(); else STEP_X_LOW(); break;
        case 1: if (output) STEP_Y_HIGH(); else STEP_Y_LOW(); break;
        case 2: if (output) STEP_Z_HIGH(); else STEP_Z_LOW(); break;
        default: break;
    }
}
void CNC_Step_AllLow(void) {
    for (uint8_t i = 0; i < CNC_NUM_AXES; i++) CNC_Step_Pulse(i, false);
}
void CNC_SetDirection(uint8_t axis, int8_t dir) {
    if (axis >= CNC_NUM_AXES) return;
    cnc_machine.direction[axis] = dir;
    bool inverted = (cnc_config.dir_invert_mask >> axis) & 1U;
    bool output = (dir > 0) ^ inverted;
    uint16_t pin = 0;
    switch (axis) {
        case 0: pin = (1U << DIR_X_PIN); break;
        case 1: pin = (1U << DIR_Y_PIN); break;
        case 2: pin = (1U << DIR_Z_PIN); break;
    }
    if (output) DIR_PORT->BSRR = pin; else DIR_PORT->BRR = pin;
}
void CNC_EnableMotors(bool enable) {
#if CNC_USE_ENABLE_PIN
    cnc_machine.motors_enabled = enable;
    uint32_t pins = (1U << EN_X_PIN) | (1U << EN_Y_PIN) | (1U << EN_Z_PIN);
    bool output = enable ^ cnc_config.enable_invert;
    if (output) EN_PORT->BRR = pins; else EN_PORT->BSRR = pins;
#else
    cnc_machine.motors_enabled = enable;
#endif
}

/*============================================================================
 *                    TIMER
 *============================================================================*/

static void CNC_Timer_Init(void) {
    RCC->APB1ENR |= CNC_TIMER_RCC;
    CNC_TIMER->CR1 = 0; CNC_TIMER->CR2 = 0;
    CNC_TIMER->PSC = 72 - 1;
    CNC_TIMER->ARR = CNC_BASE_TICK_US - 1;
    CNC_TIMER->CCMR1 = 0; CNC_TIMER->CCMR2 = 0; CNC_TIMER->CCER = 0;
    CNC_TIMER->CR1 |= TIM_CR1_ARPE;
    CNC_TIMER->DIER |= TIM_DIER_UIE;
    CNC_TIMER->EGR  |= TIM_EGR_UG;
    CNC_TIMER->SR    = 0;
    NVIC_SetPriority(CNC_TIMER_IRQn, 0);
    NVIC_EnableIRQ(CNC_TIMER_IRQn);
}

#if CNC_USE_ACCELERATION && CNC_USE_ACCEL_TIMER
/*============================================================================
 *                    D16: ACCEL TIMER (TIM4 @ 1 kHz)
 *  Runs CNC_Accel_Update() at a fixed cadence so main-loop latency
 *  (UART prints, planner recalculation) cannot stall the speed curve.
 *  Priority is LOWER than step timer (0) and limit EXTI (1).
 *============================================================================*/
static void CNC_AccelTimer_Init(void) {
    RCC->APB1ENR |= CNC_ACCEL_TIMER_RCC;
    CNC_ACCEL_TIMER->CR1 = 0;
    CNC_ACCEL_TIMER->CR2 = 0;
    /* APB1 clock = 36 MHz; timer clock = 72 MHz when prescaler != 1 (TIM4 on APB1) */
    /* Use PSC = 7200-1 -> 10 kHz tick, ARR = 10-1 -> 1 kHz period */
    CNC_ACCEL_TIMER->PSC = (CNC_SYSTEM_CLOCK / 10000U) - 1U;
    CNC_ACCEL_TIMER->ARR = (10000U / CNC_ACCEL_TIMER_HZ) - 1U;
    CNC_ACCEL_TIMER->CCMR1 = 0;
    CNC_ACCEL_TIMER->CCMR2 = 0;
    CNC_ACCEL_TIMER->CCER  = 0;
    CNC_ACCEL_TIMER->CR1  |= TIM_CR1_ARPE;
    CNC_ACCEL_TIMER->DIER |= TIM_DIER_UIE;
    CNC_ACCEL_TIMER->EGR  |= TIM_EGR_UG;
    CNC_ACCEL_TIMER->SR    = 0;
    NVIC_SetPriority(CNC_ACCEL_TIMER_IRQn, CNC_ACCEL_TIMER_PRIO);
    NVIC_EnableIRQ(CNC_ACCEL_TIMER_IRQn);
    CNC_ACCEL_TIMER->CR1 |= TIM_CR1_CEN;
}

void CNC_ACCEL_TIMER_IRQHandler(void) {
    if (!(CNC_ACCEL_TIMER->SR & TIM_SR_UIF)) return;
    CNC_ACCEL_TIMER->SR = ~TIM_SR_UIF;
    /* Only do real work when actively executing a planned RUN move */
    if (cnc_machine.is_moving && cnc_machine.state == CNC_STATE_RUN) {
        CNC_Accel_Update();
    }
}
#endif /* CNC_USE_ACCELERATION && CNC_USE_ACCEL_TIMER */

static uint32_t CNC_Step_CalcDuty(uint32_t period_us) {
    uint32_t min_period = CNC_STEP_MIN_LOW_US + CNC_STEP_MIN_HIGH_US;
    if (period_us < min_period) period_us = min_period;
    uint32_t low_us  = period_us * (100U - (uint32_t)CNC_STEP_DUTY_PERCENT) / 100U;
    uint32_t high_us = period_us - low_us;
    if (low_us  < CNC_STEP_MIN_LOW_US)  { low_us  = CNC_STEP_MIN_LOW_US;  high_us = period_us - low_us; }
    if (high_us < CNC_STEP_MIN_HIGH_US) { high_us = CNC_STEP_MIN_HIGH_US; low_us  = period_us - high_us; }
    if (low_us  < CNC_STEP_MIN_LOW_US)    low_us  = CNC_STEP_MIN_LOW_US;
    if (high_us < CNC_STEP_MIN_HIGH_US)   high_us = CNC_STEP_MIN_HIGH_US;
    step_high_ticks = high_us;
    step_low_ticks  = low_us;
    return (high_us + low_us);
}

static void CNC_Timer_SetPeriod(uint32_t period_us) {
    if (period_us < MIN_TIMER_PERIOD_US) period_us = MIN_TIMER_PERIOD_US;
    if (period_us > MAX_TIMER_PERIOD_US) period_us = MAX_TIMER_PERIOD_US;
    (void)CNC_Step_CalcDuty(period_us);
    CNC_TIMER->ARR = (step_phase == 0 ? step_high_ticks : step_low_ticks) - 1;
}

static void CNC_Timer_Start(void) {
    if (step_high_ticks == 0 || step_low_ticks == 0) {
        (void)CNC_Step_CalcDuty(CNC_BASE_TICK_US);
    }
    step_phase = 0;
    CNC_TIMER->ARR = step_high_ticks - 1;
    CNC_TIMER->CNT = 0;
    CNC_TIMER->SR  = 0;
    CNC_TIMER->CR1 |= TIM_CR1_CEN;
}

/* L4+E6 fix: commit pending HIGH steps inside critical section (atomic vs ISR) */
static void CNC_Timer_Stop(void) {
    __disable_irq();
    /* Stop the timer immediately so ISR cannot append more pending steps */
    CNC_TIMER->CR1 &= ~TIM_CR1_CEN;
    for (uint8_t i = 0; i < CNC_NUM_AXES; i++) {
        if (pending_step[i]) {
            cnc_machine.position[i] +=
                cnc_machine.inv_steps_per_mm[i] * (float)cnc_machine.direction[i];
            pending_step[i] = false;
            CNC_Step_Pulse(i, false);
        }
    }
    CNC_Step_AllLow();
    CNC_TIMER->CNT = 0;
    step_phase     = 0;
    __enable_irq();
}

/*============================================================================
 *                    ACCELERATION (step-based)
 *============================================================================*/

#if CNC_USE_ACCELERATION

void CNC_Accel_Update(void) {
    AccelState_t* a = &cnc_machine.accel;
    if (!cnc_machine.is_moving || cnc_machine.state != CNC_STATE_RUN) return;

    bresenham_int_t total_snap, remaining_snap;
    __disable_irq();
    total_snap     = bresenham_total;
    remaining_snap = step_events_remaining;
    __enable_irq();

    if (total_snap <= 0) return;
    uint32_t total_steps = (uint32_t)total_snap;
    uint32_t steps_rem   = (remaining_snap < 0) ? 0 : (uint32_t)remaining_snap;
    uint32_t steps_done  = total_steps - steps_rem;

    if (steps_done < a->last_update_step_done + CNC_ACCEL_UPDATE_STEPS) return;
    if (a->last_update_step_done == steps_done) return;
    a->last_update_step_done = steps_done;

    if (a->mm_per_step <= 0.0f) {
        if (cnc_machine.move_distance > CNC_EPSILON_MM)
            a->mm_per_step = cnc_machine.move_distance / (float)total_steps;
        else { a->mm_per_step = 0.0f; return; }
    }

    if (steps_rem == 0) {
        a->phase = ACCEL_PHASE_DECEL;
        a->current_speed = a->exit_speed;
        goto compute_timing;
    } else if (steps_done >= a->decel_start) {
        a->phase = ACCEL_PHASE_DECEL;
    } else if (steps_done >= a->accel_steps) {
        a->phase = ACCEL_PHASE_CRUISE;
    } else {
        a->phase = ACCEL_PHASE_ACCEL;
    }

    float mm = a->mm_per_step;
    float speed = 0.0f;

    switch (a->phase) {
        case ACCEL_PHASE_ACCEL: {
            float s = (float)steps_done * mm;
            float v_sqr = a->entry_speed * a->entry_speed + 2.0f * a->accel_rate * s;
            if (v_sqr < 0.0f) v_sqr = 0.0f;
            speed = sqrtf(v_sqr);
            if (speed > a->target_speed) speed = a->target_speed;
            if (speed < a->entry_speed)  speed = a->entry_speed;
            break;
        }
        case ACCEL_PHASE_CRUISE:
            speed = a->target_speed; break;
        case ACCEL_PHASE_DECEL: {
            float s_rem = (float)steps_rem * mm;
            float v_sqr = a->exit_speed * a->exit_speed + 2.0f * a->accel_rate * s_rem;
            if (v_sqr < 0.0f) v_sqr = 0.0f;
            speed = sqrtf(v_sqr);
            if (speed > a->target_speed) speed = a->target_speed;
            if (speed < a->exit_speed)   speed = a->exit_speed;
            break;
        }
    }

    /* Safety envelope */
    float dist_rem = (float)steps_rem * mm;
    float safe_sqr = a->exit_speed * a->exit_speed + 2.0f * a->accel_rate * dist_rem;
    if (safe_sqr < 0.0f) safe_sqr = 0.0f;
    float max_safe = sqrtf(safe_sqr);
    if (max_safe < a->exit_speed)   max_safe = a->exit_speed;
    if (max_safe > a->target_speed) max_safe = a->target_speed;
    if (speed > max_safe) speed = max_safe;

    a->current_speed = speed;

compute_timing:
    {
        float spm = a->spm_dominant;
        if (spm < CNC_SPM_EPSILON) spm = 1.0f;
        float s = a->current_speed; if (s < 0.0f) s = 0.0f;

        float min_speed = (float)CNC_MIN_STEP_FREQ * a->inv_spm_dominant;
        if (min_speed < CNC_EPSILON_MM) min_speed = CNC_EPSILON_MM;
        if (s < min_speed) s = min_speed;

        float freq = s * spm;
        if (freq < (float)CNC_MIN_STEP_FREQ) freq = (float)CNC_MIN_STEP_FREQ;
        if (freq > (float)CNC_MAX_STEP_FREQ) freq = (float)CNC_MAX_STEP_FREQ;

        uint32_t new_period = (uint32_t)((float)CNC_US_PER_SEC / freq);
        if (new_period < MIN_TIMER_PERIOD_US) new_period = MIN_TIMER_PERIOD_US;
        if (new_period > MAX_TIMER_PERIOD_US) new_period = MAX_TIMER_PERIOD_US;

        uint32_t min_p = CNC_STEP_MIN_LOW_US + CNC_STEP_MIN_HIGH_US;
        if (new_period < min_p) new_period = min_p;

        uint32_t low_us  = new_period * (100U - (uint32_t)CNC_STEP_DUTY_PERCENT) / 100U;
        uint32_t high_us = new_period - low_us;
        if (low_us  < CNC_STEP_MIN_LOW_US)  { low_us  = CNC_STEP_MIN_LOW_US;  high_us = new_period - low_us; }
        if (high_us < CNC_STEP_MIN_HIGH_US) { high_us = CNC_STEP_MIN_HIGH_US; low_us  = new_period - high_us; }
        if (low_us  < CNC_STEP_MIN_LOW_US)    low_us  = CNC_STEP_MIN_LOW_US;
        if (high_us < CNC_STEP_MIN_HIGH_US)   high_us = CNC_STEP_MIN_HIGH_US;
        uint32_t eff_period = high_us + low_us;

        __disable_irq();
        a->precomp_high_ticks = high_us;
        a->precomp_low_ticks  = low_us;
        a->precomp_period     = eff_period;
        a->needs_recalc       = 1;
        __enable_irq();
    }
}

/* ISR-side */
static void CNC_Accel_TimerTick(void) {
    AccelState_t* a = &cnc_machine.accel;
    if (!a->needs_recalc) return;
    a->needs_recalc = 0;
    uint32_t new_high   = a->precomp_high_ticks;
    uint32_t new_low    = a->precomp_low_ticks;
    uint32_t new_period = a->precomp_period;
    if (new_high < CNC_STEP_MIN_HIGH_US) new_high = CNC_STEP_MIN_HIGH_US;
    if (new_low  < CNC_STEP_MIN_LOW_US)  new_low  = CNC_STEP_MIN_LOW_US;
    if (new_period < (CNC_STEP_MIN_HIGH_US + CNC_STEP_MIN_LOW_US))
        new_period = CNC_STEP_MIN_HIGH_US + CNC_STEP_MIN_LOW_US;
    step_high_ticks     = new_high;
    step_low_ticks      = new_low;
    (void)new_period;  /* period was used by removed current_step_period (F7) */
}

void CNC_Accel_Plan(float distance, float feedrate, float accel) {
    uint8_t dom = cnc_machine.dominant_axis;
    float spm = CNC_SafeStepsPerMM(dom);
    float min_speed = (float)CNC_MIN_STEP_FREQ / spm;
    CNC_Accel_PlanWithEntryExit(distance, feedrate, accel, min_speed, min_speed);
}

void CNC_Accel_PlanWithEntryExit(float distance, float feedrate, float accel,
                                 float entry_speed, float exit_speed) {
    AccelState_t* a = &cnc_machine.accel;
    uint8_t  dom = cnc_machine.dominant_axis;
    float    spm = CNC_SafeStepsPerMM(dom);

    a->last_update_step_done = 0;
    a->mm_per_step = 0.0f;
    a->needs_recalc = 0;
    a->spm_dominant     = spm;
    a->inv_spm_dominant = 1.0f / spm;

    float cruise_speed = feedrate / CNC_SEC_PER_MIN;
    if (cruise_speed < 0.0f) cruise_speed = 0.0f;

    float min_speed = (float)CNC_MIN_STEP_FREQ / spm;
    if (min_speed < CNC_EPSILON_MM) min_speed = CNC_EPSILON_MM;
    if (entry_speed  < min_speed) entry_speed  = min_speed;
    if (exit_speed   < min_speed) exit_speed   = min_speed;
    if (cruise_speed < min_speed) cruise_speed = min_speed;
    if (accel < CNC_MIN_ACCEL_MMS2) accel = CNC_MIN_ACCEL_MMS2;

    if (distance < CNC_EPSILON_MM) {
        /* A6 fix: super-short move (< 1um). Don't jump straight to cruise speed,
           the driver may stall. Use entry_speed (minimum-safe speed). */
        a->phase = ACCEL_PHASE_CRUISE;
        a->current_speed = a->target_speed = entry_speed;
        a->entry_speed = a->exit_speed = entry_speed;
        a->accel_rate = accel;
        a->accel_steps = a->cruise_steps = a->decel_steps = a->decel_start = 0;
        return;
    }

    a->target_speed = cruise_speed;
    a->entry_speed  = entry_speed;
    a->exit_speed   = exit_speed;
    a->accel_rate   = accel;
    a->current_speed = entry_speed;

    float accel_dist = (cruise_speed * cruise_speed - entry_speed * entry_speed) / (2.0f * accel);
    float decel_dist = (cruise_speed * cruise_speed - exit_speed  * exit_speed)  / (2.0f * accel);
    if (accel_dist < 0.0f) accel_dist = 0.0f;
    if (decel_dist < 0.0f) decel_dist = 0.0f;

    if (accel_dist + decel_dist > distance) {
        float peak_sqr = accel * distance + 0.5f * (entry_speed * entry_speed + exit_speed * exit_speed);
        if (peak_sqr < 0.0f) peak_sqr = 0.0f;
        float peak = sqrtf(peak_sqr);
        if (peak < min_speed) peak = min_speed;
        /* D10 fix: never drive peak below either endpoint - avoids stall */
        float endpoint_max = (entry_speed > exit_speed) ? entry_speed : exit_speed;
        if (peak < endpoint_max) peak = endpoint_max;
        accel_dist = (peak * peak - entry_speed * entry_speed) / (2.0f * accel);
        if (accel_dist < 0.0f) accel_dist = 0.0f;
        if (accel_dist > distance) accel_dist = distance;
        decel_dist = distance - accel_dist;
        if (decel_dist < 0.0f) decel_dist = 0.0f;
        a->target_speed = peak;
    }
    float cruise_dist = distance - accel_dist - decel_dist;
    if (cruise_dist < 0.0f) cruise_dist = 0.0f;

    bresenham_int_t bt;
    __disable_irq();
    bt = bresenham_total;
    __enable_irq();

#if CNC_USE_INT32_BRESENHAM
    /* int32 inherently bounded by MAX_STEPS_PER_MOVE; just check <=0 */
    if (bt <= 0) {
#else
    if (bt <= 0 || bt > (int64_t)0xFFFFFFFFLL) {
#endif
        a->phase = ACCEL_PHASE_CRUISE;
        a->current_speed = cruise_speed;
        a->accel_steps = a->cruise_steps = a->decel_steps = a->decel_start = 0;
        return;
    }
    uint32_t total_steps = (uint32_t)bt;
    if (total_steps == 0) {
        a->phase = ACCEL_PHASE_CRUISE;
        a->current_speed = cruise_speed;
        return;
    }
    a->mm_per_step = distance / (float)total_steps;
    a->accel_steps  = (uint32_t)((accel_dist  / distance) * (float)total_steps);
    a->cruise_steps = (uint32_t)((cruise_dist / distance) * (float)total_steps);
    if (a->accel_steps > total_steps) { a->accel_steps = total_steps; a->cruise_steps = 0; }
    if (a->accel_steps + a->cruise_steps > total_steps)
        a->cruise_steps = total_steps - a->accel_steps;
    a->decel_steps = total_steps - a->accel_steps - a->cruise_steps;
    a->decel_start = a->accel_steps + a->cruise_steps;
    if (a->decel_start >= total_steps)
        a->decel_start = (total_steps > 1) ? (total_steps - 1) : 0;

    if (a->accel_steps == 0) {
        a->phase = (a->cruise_steps == 0) ? ACCEL_PHASE_DECEL : ACCEL_PHASE_CRUISE;
        a->current_speed = a->target_speed;
    } else {
        a->phase = ACCEL_PHASE_ACCEL;
        a->current_speed = entry_speed;
    }

    /* First timing */
    float freq = entry_speed * spm;
    if (freq < (float)CNC_MIN_STEP_FREQ) freq = (float)CNC_MIN_STEP_FREQ;
    if (freq > (float)CNC_MAX_STEP_FREQ) freq = (float)CNC_MAX_STEP_FREQ;
    uint32_t period = (uint32_t)((float)CNC_US_PER_SEC / freq);
    if (period < MIN_TIMER_PERIOD_US) period = MIN_TIMER_PERIOD_US;
    if (period > MAX_TIMER_PERIOD_US) period = MAX_TIMER_PERIOD_US;
    uint32_t eff = CNC_Step_CalcDuty(period);  /* populates step_high/low_ticks */
    a->precomp_high_ticks = step_high_ticks;
    a->precomp_low_ticks  = step_low_ticks;
    a->precomp_period     = eff;
    a->needs_recalc       = 0;
}

#endif /* CNC_USE_ACCELERATION */

/*============================================================================
 *                    TIMER IRQ HANDLER
 *  - Phase 0: emit step HIGH pulse on axes whose Bresenham counter wraps
 *  - Phase 1: clear step pulse LOW, commit position increment
 *
 *  Hard-limit handling (FIXES L1+L2):
 *    - Scans ALL axes (no per-axis step gating)
 *    - On any trigger: stop timer, disable motors, ALARM state
 *============================================================================*/

void CNC_TIMER_IRQHandler(void) {
    if (!(CNC_TIMER->SR & TIM_SR_UIF)) return;
    CNC_TIMER->SR = ~TIM_SR_UIF;

    if (!cnc_machine.is_moving) return;

    /*--- PHASE 1: end of HIGH window, drop pulses low + commit ---*/
    if (step_phase == 1) {
        bool had_any = false;
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            if (pending_step[i]) {
                CNC_Step_Pulse(i, false);
                pending_step[i] = false;
                had_any = true;
                /* B10: cached inverse to avoid float division in ISR */
                cnc_machine.position[i] +=
                    cnc_machine.inv_steps_per_mm[i] * (float)cnc_machine.direction[i];
            }
        }
        if (had_any) {
            step_events_remaining--;
#if CNC_USE_ACCELERATION
            if (cnc_machine.state == CNC_STATE_RUN) CNC_Accel_TimerTick();
#endif
#if CNC_USE_HOMING
#endif
        }
        step_phase = 0;
        if (step_high_ticks == 0) step_high_ticks = CNC_STEP_MIN_HIGH_US;
        {
            uint32_t arr_new = step_high_ticks - 1;
            CNC_TIMER->ARR = arr_new;
            /* B6: if CNT already passed new ARR, force update to avoid 65535 wrap */
            if (CNC_TIMER->CNT >= arr_new) {
                CNC_TIMER->CNT = 0;
                CNC_TIMER->EGR = TIM_EGR_UG;
                CNC_TIMER->SR &= ~TIM_SR_UIF;
            }
        }
        return;
    }

    /*--- PHASE 0: hard-limit check (L1+L2: scan ALL axes) ---*/
#if CNC_USE_LIMIT_SWITCH
    if (cnc_config.hard_limit_enabled && !cnc_unlock_active &&
        (cnc_machine.state == CNC_STATE_RUN ||
         cnc_machine.state == CNC_STATE_GOWDELAY ||
         cnc_machine.state == CNC_STATE_GOWDELAY_RECT)) {
        bool tripped = false;
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            if (CNC_Limit_IsTriggered(i)) {
                cnc_machine.limit_triggered[i] = true;
                tripped = true;
            }
        }
        if (tripped) {
            CNC_Timer_Stop();
            cnc_machine.is_moving = false;
            CNC_EnableMotors(false);   /* L1: HARD CUT */
#if CNC_USE_RELAY
            CNC_LaserPulse_Abort();
#endif
            CNC_ClearBufferedMotion();
            cnc_machine.state = CNC_STATE_LIMIT_TRIGGERED;
            return;
        }
    }
    /* Trong unlock mode: ISR khong tu clear flag.
       CNC_Process se xu ly sau khi move ket thuc. */
#endif

    /*--- Homing limit check ---*/
#if CNC_USE_HOMING
    if (cnc_machine.state == CNC_STATE_HOMING) {
        bool limit_now = CNC_Limit_IsTriggered(cnc_machine.homing_axis);
        if (cnc_machine.homing_phase == HOMING_SEEK ||
            cnc_machine.homing_phase == HOMING_FEED) {
            if (limit_now) {
                /* Expected hit - signal main state machine */
                homing_limit_hit = true;
                CNC_Timer_Stop();
                cnc_machine.is_moving = false;
                return;
            }
        } else if (cnc_machine.homing_phase == HOMING_PULLOFF1 ||
                   cnc_machine.homing_phase == HOMING_PULLOFF2) {
            /* C4: Pulloff đang di chuyển RA KHỎI switch.
               Bỏ qua HOMING_PULLOFF_GRACE_STEPS step đầu để switch cơ học kịp nhả.
               Sau grace steps nếu vẫn tripped -> sai hướng -> ALARM. */
            cnc_machine.homing_pulloff_steps_done++;
            if (limit_now && cnc_machine.homing_pulloff_steps_done > HOMING_PULLOFF_GRACE_STEPS) {
                /* Qua grace steps mà switch vẫn còn -> sai hướng -> ALARM */
                CNC_Timer_Stop();
                cnc_machine.is_moving = false;
                CNC_EnableMotors(false);
#if CNC_USE_RELAY
                CNC_LaserPulse_Abort();
#endif
                CNC_ClearBufferedMotion();
                cnc_machine.homing_axes_pending = 0;
                cnc_machine.homing_phase = HOMING_IDLE;
                cnc_machine.state = CNC_STATE_ALARM;
                return;
            }
        }
    }
#endif

    /*--- Completion ---*/
    if (step_events_remaining <= 0) {
        CNC_Timer_Stop();
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            /* B11+P1: both target[] and logical_target[] are now PHYSICAL-grid
               snapped (P1). They coincide when backlash is OFF; differ only by
               backlash compensation distance when ON. */
            float final_pos =
#if CNC_USE_BACKLASH
                cnc_config.backlash_enabled
                    ? cnc_machine.logical_target[i]
                    : cnc_machine.target[i];
#else
                cnc_machine.target[i];
#endif
            cnc_machine.position[i]   = final_pos;
            cnc_machine.step_count[i] = 0;
        }
        /* E2 fix: clear is_moving FIRST so main loop never sees IDLE && moving */
        cnc_machine.is_moving = false;
        if (cnc_machine.state == CNC_STATE_RUN) {
            cnc_machine.state = CNC_STATE_IDLE;
            cnc_send_ok_flag = true;
        }
        return;
    }

    /*--- Bresenham step ---*/
    bool any_step = false;
    for (int axis = 0; axis < CNC_NUM_AXES; axis++) {
        if (bresenham_steps[axis] == 0) continue;
        bresenham_counter[axis] += bresenham_steps[axis];
        if (bresenham_counter[axis] >= bresenham_total) {
            bresenham_counter[axis] -= bresenham_total;
            CNC_Step_Pulse(axis, true);
            pending_step[axis] = true;
            any_step = true;
        }
    }

    if (any_step) {
        step_phase = 1;
        if (step_low_ticks == 0) step_low_ticks = CNC_STEP_MIN_LOW_US;
        uint32_t arr_new = step_low_ticks - 1;
        CNC_TIMER->ARR = arr_new;
        /* B6: force reload if CNT overshot */
        if (CNC_TIMER->CNT >= arr_new) {
            CNC_TIMER->CNT = 0;
            CNC_TIMER->EGR = TIM_EGR_UG;
            CNC_TIMER->SR &= ~TIM_SR_UIF;
        }
    } else {
        /* B7 fix: no step pending - stay in HIGH but don't disrupt ARR */
        if (step_high_ticks > 0) {
            uint32_t arr_new = step_high_ticks - 1;
            CNC_TIMER->ARR = arr_new;
            if (CNC_TIMER->CNT >= arr_new) {
                CNC_TIMER->CNT = 0;
                CNC_TIMER->EGR = TIM_EGR_UG;
                CNC_TIMER->SR &= ~TIM_SR_UIF;
            }
        }
    }
}

/*============================================================================
 *                    LIMIT SWITCH
 *============================================================================*/

#if CNC_USE_LIMIT_SWITCH

void CNC_Limit_Init(void) {
    /*
     * Hardware mapping:
     *   X endstop -> PA3 (EXTI3, pull-up, NC wiring -> ACTIVE_HIGH)
     *   Y endstop -> PA4 (EXTI4, pull-up, NC wiring -> ACTIVE_HIGH)
     *   Z axis    -> NO endstop; CNC_Limit_IsTriggered(2) always returns false
     *
     * CNF=10 (input pull-up/down), MODE=00 (input)
     */
#if CNC_LIMIT_SHARED_PIN
    /* Shared-pin mode (not used in this config, kept for compile safety) */
    #if LIMIT_SHARED_PIN == 3
        GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3); GPIOA->CRL |= GPIO_CRL_CNF3_1;
    #elif LIMIT_SHARED_PIN == 4
        GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4); GPIOA->CRL |= GPIO_CRL_CNF4_1;
    #elif LIMIT_SHARED_PIN == 5
        GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5); GPIOA->CRL |= GPIO_CRL_CNF5_1;
    #else
        #error "Unsupported LIMIT_SHARED_PIN - add CRL bits"
    #endif
    #if CNC_LIMIT_PULL_UP
        GPIOA->BSRR = (1U << LIMIT_SHARED_PIN);
    #else
        GPIOA->BRR  = (1U << LIMIT_SHARED_PIN);
    #endif
#else
    /* PA3 -> X endstop input */
    GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
    GPIOA->CRL |= GPIO_CRL_CNF3_1;   /* CNF=10: input pull-up/down */

    /* PA4 -> Y endstop input */
    GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
    GPIOA->CRL |= GPIO_CRL_CNF4_1;   /* CNF=10: input pull-up/down */

    /* PA5 -> NOT configured (Z has no endstop) */

    /* Set pull direction for PA3 and PA4 only */
    #if CNC_LIMIT_PULL_UP
        GPIOA->BSRR = (1U << LIMIT_X_PIN) | (1U << LIMIT_Y_PIN);   /* pull-UP X, Y */
    #else
        GPIOA->BRR  = (1U << LIMIT_X_PIN) | (1U << LIMIT_Y_PIN);   /* pull-DOWN X, Y */
    #endif
    /* PA5 (Z) intentionally left as default (floating/analog safe) */
#endif

    for (int i = 0; i < CNC_NUM_AXES; i++) cnc_machine.limit_triggered[i] = false;

#if CNC_USE_LIMIT_EXTI
    /* Route PORTA pins to EXTI lines (EXTICR default = 0x0 = PORTA) */
    AFIO->EXTICR[0] &= ~(AFIO_EXTICR1_EXTI3);   /* EXTI3 <- PA3 (X) */
    AFIO->EXTICR[1] &= ~(AFIO_EXTICR2_EXTI4);   /* EXTI4 <- PA4 (Y) */
    /* EXTI5 (PA5) NOT configured - Z has no endstop */

#if CNC_LIMIT_SHARED_PIN
    uint32_t mask = LIMIT_SHARED_EXTI_LINE;
#else
    /* Z excluded: only X (PA3/EXTI3) and Y (PA4/EXTI4) */
    uint32_t mask = LIMIT_X_EXTI_LINE | LIMIT_Y_EXTI_LINE;
#endif

    EXTI->IMR  |= mask;

#if CNC_LIMIT_ACTIVE_HIGH
    /* Rising edge = switch pressed (NC opens, pin goes HIGH) */
    EXTI->RTSR |= mask;
    EXTI->FTSR &= ~mask;
#else
    /* Falling edge = switch pressed (NO closes to GND) */
    EXTI->FTSR |= mask;
    EXTI->RTSR &= ~mask;
#endif

    EXTI->PR = mask;   /* clear any pending flags before enabling IRQ */

    /* Enable NVIC for X (EXTI3) and Y (EXTI4) only - Z not needed */
#if CNC_LIMIT_SHARED_PIN
    #if LIMIT_SHARED_PIN == 3
        NVIC_SetPriority(EXTI3_IRQn, 1);   NVIC_EnableIRQ(EXTI3_IRQn);
    #elif LIMIT_SHARED_PIN == 4
        NVIC_SetPriority(EXTI4_IRQn, 1);   NVIC_EnableIRQ(EXTI4_IRQn);
    #elif LIMIT_SHARED_PIN >= 5 && LIMIT_SHARED_PIN <= 9
        NVIC_SetPriority(EXTI9_5_IRQn, 1); NVIC_EnableIRQ(EXTI9_5_IRQn);
    #endif
#else
    NVIC_SetPriority(EXTI3_IRQn, 1); NVIC_EnableIRQ(EXTI3_IRQn);   /* X: PA3 */
    NVIC_SetPriority(EXTI4_IRQn, 1); NVIC_EnableIRQ(EXTI4_IRQn);   /* Y: PA4 */
    /* EXTI9_5_IRQn NOT enabled - Z has no endstop */
#endif
#endif /* CNC_USE_LIMIT_EXTI */
}

bool CNC_Limit_IsTriggered(uint8_t axis) {
    /*
     * Endstop mapping:
     *   axis 0 (X) -> PA3, EXTI3, pull-up, NC wiring (ACTIVE_HIGH)
     *   axis 1 (Y) -> PA4, EXTI4, pull-up, NC wiring (ACTIVE_HIGH)
     *   axis 2 (Z) -> NO ENDSTOP -> always returns false
     */
#ifdef CNC_LIMIT_Z_NO_ENDSTOP
    if (axis == 2) return false;   /* Z: no physical endstop */
#endif

    uint16_t pin = 0;
    switch (axis) {
        case 0: pin = (1U << LIMIT_X_PIN); break;   /* PA3 */
        case 1: pin = (1U << LIMIT_Y_PIN); break;   /* PA4 */
        default: return false;
    }

    bool level_high = (LIMIT_PORT->IDR & pin) != 0;

    /* "raw triggered" logic:
     *   ACTIVE_HIGH=1 (NC switch, pull-up): triggered when pin reads HIGH
     *   ACTIVE_HIGH=0 (NO switch, pull-up): triggered when pin reads LOW  */
#if CNC_LIMIT_ACTIVE_HIGH
    bool raw = level_high;
#else
    bool raw = !level_high;
#endif

    /* Optional per-axis software invert via $5 setting */
    bool inverted = (cnc_config.limit_invert_mask >> axis) & 1U;
    return raw ^ inverted;
}

bool CNC_Limit_CheckAll(void) {
    if (!cnc_config.hard_limit_enabled) return false;
    if (cnc_unlock_active) return false;   /* escape mode bypass */
    bool any = false;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        if (CNC_Limit_IsTriggered(i)) {
            cnc_machine.limit_triggered[i] = true;
            any = true;
        }
    }
    if (any && cnc_machine.is_moving &&
        (cnc_machine.state == CNC_STATE_RUN ||
         cnc_machine.state == CNC_STATE_GOWDELAY ||
         cnc_machine.state == CNC_STATE_GOWDELAY_RECT)) {
        CNC_Stop();
        CNC_EnableMotors(false);
        cnc_machine.state = CNC_STATE_LIMIT_TRIGGERED;
        CNC_UART_PutString("[LIMIT:");
        if (cnc_machine.limit_triggered[0]) CNC_UART_PutString(" X");
        if (cnc_machine.limit_triggered[1]) CNC_UART_PutString(" Y");
        if (cnc_machine.limit_triggered[2]) CNC_UART_PutString(" Z");
        CNC_UART_PutString("]\r\n");
    }
    return any;
}

void CNC_Limit_ClearAlarm(void) {
    bool limit_now = false;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        cnc_machine.limit_triggered[i] = false;
        if (CNC_Limit_IsTriggered(i)) limit_now = true;
    }
    if (cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED ||
        cnc_machine.state == CNC_STATE_ALARM) {
        cnc_machine.state = CNC_STATE_IDLE;
        CNC_EnableMotors(true);

        if (limit_now) {
            /* Allow exactly ONE escape move only when a limit is physically
               still active.  Do not bypass hard limits after a non-limit alarm
               or after the switch has already been released. */
            cnc_unlock_active       = true;
            cnc_unlock_move_started = false;
            CNC_UART_PutString("[ALARM CLEARED - send 1 move to escape endstop]\r\n");
        } else {
            cnc_unlock_active       = false;
            cnc_unlock_move_started = false;
            CNC_UART_PutString("[ALARM CLEARED]\r\n");
        }
    }
}

#if CNC_USE_LIMIT_EXTI
/* Common path: any limit line triggered → halt + alarm */
static void CNC_EXTI_LimitCommon(uint32_t pr_mask) {
    EXTI->PR = pr_mask;
    if (!cnc_config.hard_limit_enabled) return;
    /* Unlock/escape mode: user explicitly wants to move off the switch.
       Just snapshot which pins are tripped, do NOT alarm. */
    if (cnc_unlock_active) {
        for (int i = 0; i < CNC_NUM_AXES; i++)
            if (CNC_Limit_IsTriggered(i)) cnc_machine.limit_triggered[i] = true;
        return;
    }
    /* Homing handles its own limit semantics in the Timer ISR. */
    if (cnc_machine.state == CNC_STATE_HOMING) {
        for (int i = 0; i < CNC_NUM_AXES; i++)
            if (CNC_Limit_IsTriggered(i)) cnc_machine.limit_triggered[i] = true;
        return;
    }
    bool any = false;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        if (CNC_Limit_IsTriggered(i)) {
            cnc_machine.limit_triggered[i] = true;
            any = true;
        }
    }
    if (!any) return;
    /* C1: ALWAYS stop the timer in EXTI, even if is_moving==false. */
    CNC_Timer_Stop();
    cnc_machine.is_moving = false;
    CNC_EnableMotors(false);
#if CNC_USE_RELAY
    CNC_LaserPulse_Abort();
#endif
    CNC_ClearBufferedMotion();
    cnc_machine.state = CNC_STATE_LIMIT_TRIGGERED;
}

#if CNC_LIMIT_SHARED_PIN
    #if LIMIT_SHARED_PIN == 3
        void EXTI3_IRQHandler(void) {
            if (EXTI->PR & LIMIT_SHARED_EXTI_LINE)
                CNC_EXTI_LimitCommon(LIMIT_SHARED_EXTI_LINE);
        }
    #elif LIMIT_SHARED_PIN == 4
        void EXTI4_IRQHandler(void) {
            if (EXTI->PR & LIMIT_SHARED_EXTI_LINE)
                CNC_EXTI_LimitCommon(LIMIT_SHARED_EXTI_LINE);
        }
    #elif LIMIT_SHARED_PIN >= 5 && LIMIT_SHARED_PIN <= 9
        void EXTI9_5_IRQHandler(void) {
            if (EXTI->PR & LIMIT_SHARED_EXTI_LINE)
                CNC_EXTI_LimitCommon(LIMIT_SHARED_EXTI_LINE);
        }
    #endif
#else
    void EXTI3_IRQHandler(void) {
        /* PA3 -> X endstop */
        if (EXTI->PR & LIMIT_X_EXTI_LINE) CNC_EXTI_LimitCommon(LIMIT_X_EXTI_LINE);
    }
    void EXTI4_IRQHandler(void) {
        /* PA4 -> Y endstop */
        if (EXTI->PR & LIMIT_Y_EXTI_LINE) CNC_EXTI_LimitCommon(LIMIT_Y_EXTI_LINE);
    }
    /* EXTI9_5_IRQHandler NOT defined - Z axis has no endstop */
#endif
#endif /* CNC_USE_LIMIT_EXTI */

#endif /* CNC_USE_LIMIT_SWITCH */

/*============================================================================
 *                    RELAY
 *============================================================================*/

#if CNC_USE_RELAY
void CNC_Relay_Init(void) {
    GPIOB->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); GPIOB->CRH |= GPIO_CRH_MODE13_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14); GPIOB->CRH |= GPIO_CRH_MODE14_0;
    GPIOB->CRH &= ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15); GPIOB->CRH |= GPIO_CRH_MODE15_0;
    CNC_Spindle_Off(); CNC_Coolant_Off();
    cnc_laser_pulse_done_flag = false;
    cnc_laser_pulse_aborted_flag = false;
}
#if RELAY_ACTIVE_HIGH
  #define _RLY_ON(port,pin)  ((port)->BSRR = (1U << (pin)))
  #define _RLY_OFF(port,pin) ((port)->BRR  = (1U << (pin)))
#else
  #define _RLY_ON(port,pin)  ((port)->BRR  = (1U << (pin)))
  #define _RLY_OFF(port,pin) ((port)->BSRR = (1U << (pin)))
#endif
void CNC_Spindle_On(void)        { _RLY_ON(RELAY_SPINDLE_PORT, RELAY_SPINDLE_PIN);             cnc_machine.spindle_on = true; }
void CNC_Spindle_Off(void)       {
    _RLY_OFF(RELAY_SPINDLE_PORT, RELAY_SPINDLE_PIN);
    cnc_machine.spindle_on = false;
    cnc_laser_pulse_active = false;
    cnc_laser_pulse_end_tick = 0;
    cnc_laser_pulse_duration_ms = 0;
}
void CNC_Coolant_Mist_On(void)   { _RLY_ON(RELAY_COOLANT_MIST_PORT, RELAY_COOLANT_MIST_PIN);   cnc_machine.coolant_mist_on = true; }
void CNC_Coolant_Mist_Off(void)  { _RLY_OFF(RELAY_COOLANT_MIST_PORT, RELAY_COOLANT_MIST_PIN);  cnc_machine.coolant_mist_on = false; }
void CNC_Coolant_Flood_On(void)  { _RLY_ON(RELAY_COOLANT_FLOOD_PORT, RELAY_COOLANT_FLOOD_PIN); cnc_machine.coolant_flood_on = true; }
void CNC_Coolant_Flood_Off(void) { _RLY_OFF(RELAY_COOLANT_FLOOD_PORT, RELAY_COOLANT_FLOOD_PIN);cnc_machine.coolant_flood_on = false; }
void CNC_Coolant_Off(void)       { CNC_Coolant_Mist_Off(); CNC_Coolant_Flood_Off(); }
#endif

/*============================================================================
 *                    HOMING
 *============================================================================*/

#if CNC_USE_HOMING

/* Move a single axis a signed mm distance at given mm/min feedrate */
static void CNC_Homing_MoveAxis(uint8_t axis, float distance, float feedrate) {
    if (axis >= CNC_NUM_AXES) return;
    if (fabsf(distance) < CNC_DIR_THRESHOLD_MM) return;

    float spm = CNC_SafeStepsPerMM(axis);
    bresenham_int_t steps = (bresenham_int_t)roundf(fabsf(distance) * spm);
    if (steps == 0) return;
    if (steps > MAX_STEPS_PER_MOVE) return;

    /* Set target only for this axis */
    cnc_machine.target[axis] = cnc_machine.position[axis] + distance;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        bresenham_steps[i] = (i == axis) ? steps : 0;
        cnc_machine.step_count[i] = (i == axis) ? (int32_t)steps : 0;
    }

    /* B10 fix: cache inv_steps_per_mm and logical_target before ISR uses them */
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        cnc_machine.inv_steps_per_mm[i] = 1.0f / CNC_SafeStepsPerMM(i);
        cnc_machine.logical_target[i]   = cnc_machine.target[i];
    }
    __disable_irq();
    bresenham_total = steps;
    step_events_remaining = steps;
    __enable_irq();
    for (int i = 0; i < CNC_NUM_AXES; i++) bresenham_counter[i] = bresenham_total / 2;

    CNC_SetDirection(axis, (distance > 0) ? 1 : -1);

    /* L11: clamp step_freq using safe spm */
    float freq_f = (feedrate / CNC_SEC_PER_MIN) * spm;
    if (freq_f < (float)CNC_MIN_STEP_FREQ_RUNTIME) freq_f = (float)CNC_MIN_STEP_FREQ_RUNTIME;
    if (freq_f > (float)CNC_HOMING_MAX_STEP_FREQ) freq_f = (float)CNC_HOMING_MAX_STEP_FREQ;
    uint32_t period = (uint32_t)((float)CNC_US_PER_SEC / freq_f);
    if (period < MIN_TIMER_PERIOD_US) period = MIN_TIMER_PERIOD_US;
    if (period > MAX_TIMER_PERIOD_US) period = MAX_TIMER_PERIOD_US;
    CNC_Timer_SetPeriod(period);
    homing_limit_hit  = false;
    /* Reset grace step counter cho pulloff mới */
    cnc_machine.homing_pulloff_steps_done = 0;
    cnc_machine.is_moving = true;
    if (cnc_unlock_active) cnc_unlock_move_started = true;
    CNC_Timer_Start();
}

/* L10: no case 2; cycle 0=any-order, 1=Z then X then Y */
static int8_t CNC_Homing_GetNextAxis(void) {
    uint8_t pending = cnc_machine.homing_axes_pending;
    if (pending == 0) return -1;
    if (cnc_config.homing_cycle == 1) {
        if (pending & 0x04) return 2;
        if (pending & 0x01) return 0;
        if (pending & 0x02) return 1;
    } else {
        for (int i = 0; i < CNC_NUM_AXES; i++)
            if (pending & (1U << i)) return i;
    }
    return -1;
}

void CNC_HomeAxis(uint8_t axis) {
    if (axis >= CNC_NUM_AXES) return;
#ifdef CNC_LIMIT_Z_NO_ENDSTOP
    if (axis == 2) { CNC_UART_PutString("[AXIS Z HAS NO ENDSTOP]\r\n"); return; }
#endif
    if (!cnc_config.homing_enabled) {
        CNC_UART_PutString("[HOMING DISABLED]\r\n"); return;
    }
    if (!(cnc_config.homing_axis_mask & (1U << axis))) {
        CNC_UART_PutString("[AXIS "); CNC_UART_PutChar('X' + axis);
        CNC_UART_PutString(" NOT IN HOMING MASK]\r\n"); return;
    }
    /* H6: refuse if machine is mid-motion or in alarm. */
    if (cnc_machine.is_moving || cnc_machine.state == CNC_STATE_RUN ||
        cnc_machine.state == CNC_STATE_HOLD ||
        cnc_machine.state == CNC_STATE_GOWDELAY ||
        cnc_machine.state == CNC_STATE_GOWDELAY_RECT) {
        CNC_UART_PutString("[ERROR: Machine busy - cannot home]\r\n"); return;
    }
    if (cnc_machine.state == CNC_STATE_ALARM ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) {
        CNC_UART_PutString("[ERROR: Clear alarm first]\r\n"); return;
    }
#if CNC_USE_LIMIT_SWITCH
    /* H7: clear leftover phases */
    cnc_machine.gowdelay.phase      = GOWDELAY_IDLE;
    cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
    cnc_machine.state             = CNC_STATE_HOMING;
    cnc_machine.homing_axis       = axis;
    cnc_machine.homing_phase      = HOMING_SEEK;
    cnc_machine.homing_axes_pending = (1U << axis);
    cnc_machine.homing_retry_count = 0;
    cnc_machine.homed[axis]       = false;
    CNC_EnableMotors(true);

    float direction = ((cnc_config.homing_dir_mask >> axis) & 1U) ? -1.0f : 1.0f;
    float max_travel = cnc_config.max_travel[axis] * CNC_HOMING_TRAVEL_OVERSHOOT;

    CNC_UART_PutString("[HOMING "); CNC_UART_PutChar('X' + axis);
    CNC_UART_PutString(direction < 0 ? " DIR:- SEEK]\r\n" : " DIR:+ SEEK]\r\n");

    CNC_Homing_MoveAxis(axis, direction * max_travel, cnc_config.homing_seek_rate);
#else
    cnc_machine.position[axis] = 0;
    cnc_machine.homed[axis] = true;
    CNC_UART_PutString("["); CNC_UART_PutChar('X' + axis);
    CNC_UART_PutString(" RESET - NO SWITCH]\r\n");
#endif
}

void CNC_HomeAll(void) {
    if (!cnc_config.homing_enabled) {
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            cnc_machine.position[i] = 0;
            cnc_machine.homed[i] = false;
        }
        CNC_Limit_ClearAlarm();
        CNC_UART_PutString("[POSITION RESET - HOMING DISABLED]\r\n");
        return;
    }
    uint8_t home_mask = cnc_config.homing_axis_mask;
#ifdef CNC_LIMIT_Z_NO_ENDSTOP
    home_mask &= (uint8_t)~0x04U;  /* Z has no physical switch in this build */
#endif
    if (home_mask == 0) {
        CNC_UART_PutString("[NO AXES IN HOMING MASK]\r\n"); return;
    }
    /* H6: refuse if machine is mid-motion */
    if (cnc_machine.is_moving || cnc_machine.state == CNC_STATE_RUN ||
        cnc_machine.state == CNC_STATE_HOLD ||
        cnc_machine.state == CNC_STATE_GOWDELAY ||
        cnc_machine.state == CNC_STATE_GOWDELAY_RECT) {
        CNC_UART_PutString("[ERROR: Machine busy - cannot home]\r\n"); return;
    }
    if (cnc_machine.state == CNC_STATE_ALARM ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) {
        CNC_UART_PutString("[ERROR: Clear alarm first]\r\n"); return;
    }
    /* H7: clear leftover GOWDELAY/RECT state so post-homing IDLE is clean */
    cnc_machine.gowdelay.phase      = GOWDELAY_IDLE;
    cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
#if CNC_USE_LIMIT_SWITCH
    cnc_machine.state               = CNC_STATE_HOMING;
    cnc_machine.homing_axes_pending = home_mask;
    cnc_machine.homing_retry_count  = 0;
    for (int i = 0; i < CNC_NUM_AXES; i++)
        if (home_mask & (1U << i)) cnc_machine.homed[i] = false;
    CNC_EnableMotors(true);

    CNC_UART_PutString("[HOMING START AXES=");
    if (home_mask & 1) CNC_UART_PutChar('X');
    if (home_mask & 2) CNC_UART_PutChar('Y');
    if (home_mask & 4) CNC_UART_PutChar('Z');
    CNC_UART_PutString(" CYCLE=");
    CNC_UART_PutInt(cnc_config.homing_cycle);
    CNC_UART_PutString("]\r\n");

    int8_t a = CNC_Homing_GetNextAxis();
    if (a >= 0) {
        cnc_machine.homing_axis  = a;
        cnc_machine.homing_phase = HOMING_SEEK;
        float dir = ((cnc_config.homing_dir_mask >> a) & 1U) ? -1.0f : 1.0f;
        float max_travel = cnc_config.max_travel[a] * CNC_HOMING_TRAVEL_OVERSHOOT;
        CNC_UART_PutString("[HOMING "); CNC_UART_PutChar('X' + a);
        CNC_UART_PutString(dir < 0 ? " DIR:- SEEK]\r\n" : " DIR:+ SEEK]\r\n");
        CNC_Homing_MoveAxis(a, dir * max_travel, cnc_config.homing_seek_rate);
    } else {
        cnc_machine.state = CNC_STATE_IDLE;
        CNC_UART_PutString("[NO AXES TO HOME]\r\n");
    }
#else
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        if (cnc_config.homing_axis_mask & (1U << i)) {
            cnc_machine.position[i] = 0;
            cnc_machine.homed[i] = true;
        }
    }
    CNC_UART_PutString("[HOMED - NO SWITCHES]\r\n");
#endif
}

/* L3: full debounced state machine with retry on stuck switch */
void CNC_Homing_Process(void) {
    if (cnc_machine.state != CNC_STATE_HOMING) return;
    if (cnc_machine.is_moving) return;

    uint8_t axis = cnc_machine.homing_axis;
    float   dir  = ((cnc_config.homing_dir_mask >> axis) & 1U) ? -1.0f : 1.0f;

    switch (cnc_machine.homing_phase) {

    case HOMING_SEEK:
        if (homing_limit_hit) {
            /* Move OFF the switch by pulloff distance (exact, not *3) */
            cnc_machine.homing_phase = HOMING_PULLOFF1;
            CNC_UART_PutString("["); CNC_UART_PutChar('X' + axis);
            CNC_UART_PutString(" PULLOFF1]\r\n");
            float pulloff = cnc_config.homing_pulloff;
            if (cnc_machine.homing_retry_count > 0) {
                pulloff *= 1.0f + HOMING_PULLOFF_RETRY_GAIN *
                           (float)cnc_machine.homing_retry_count;
            }
            CNC_Homing_MoveAxis(axis, -dir * pulloff, cnc_config.homing_seek_rate);
        } else {
            cnc_machine.homing_axes_pending = 0;
            cnc_machine.homing_phase = HOMING_IDLE;
            cnc_machine.state = CNC_STATE_ALARM;
            CNC_EnableMotors(false);
            CNC_UART_PutString("[HOMING FAIL ");
            CNC_UART_PutChar('X' + axis);
            CNC_UART_PutString(" - NO SWITCH]\r\n");
        }
        break;

    case HOMING_PULLOFF1:
        /* Wait for switch to release; if still pressed → retry pull-off */
        cnc_machine.homing_phase = HOMING_PULLOFF1_DEBOUNCE;
        cnc_machine.homing_debounce_tick = CNC_GetTick();
        break;

    case HOMING_PULLOFF1_DEBOUNCE: {
        uint32_t elapsed = CNC_GetTick() - cnc_machine.homing_debounce_tick;
        if (elapsed < cnc_config.homing_debounce_ms) return; /* wait */

        if (CNC_Limit_IsTriggered(axis)) {
            /* Switch still pressed - retry pull-off (more travel) */
            if (++cnc_machine.homing_retry_count > HOMING_MAX_RETRY) {
                cnc_machine.homing_axes_pending = 0;
                cnc_machine.homing_phase = HOMING_IDLE;
                cnc_machine.state = CNC_STATE_ALARM;
                CNC_EnableMotors(false);
                CNC_UART_PutString("[HOMING FAIL ");
                CNC_UART_PutChar('X' + axis);
                CNC_UART_PutString(" - SWITCH STUCK]\r\n");
                return;
            }
            cnc_machine.homing_phase = HOMING_SEEK; /* will re-pulloff with bigger distance */
            homing_limit_hit = true;                /* simulate just-hit */
            CNC_UART_PutString("["); CNC_UART_PutChar('X' + axis);
            CNC_UART_PutString(" RETRY PULLOFF]\r\n");
            return;
        }
        /* Switch released → slow FEED toward switch for precise edge */
        cnc_machine.homing_phase = HOMING_FEED;
        CNC_UART_PutString("["); CNC_UART_PutChar('X' + axis);
        CNC_UART_PutString(" FEED]\r\n");
        /* H2 fix: FEED travel = max(2*pulloff, 5mm) so we never under-shoot
           a wide switch trigger zone. Without this, small $27 (e.g. 0.5mm)
           causes FEED to stop short of the switch boundary. */
        float feed_travel = cnc_config.homing_pulloff * 2.0f;
        if (feed_travel < 5.0f) feed_travel = 5.0f;
        CNC_Homing_MoveAxis(axis, dir * feed_travel, cnc_config.homing_feed_rate);
        break;
    }

    case HOMING_FEED:
        if (homing_limit_hit) {
            cnc_machine.homing_phase = HOMING_PULLOFF2;
            CNC_UART_PutString("["); CNC_UART_PutChar('X' + axis);
            CNC_UART_PutString(" PULLOFF2]\r\n");
            /* Pull off EXACTLY pulloff distance — this is the rest position */
            CNC_Homing_MoveAxis(axis, -dir * cnc_config.homing_pulloff,
                                cnc_config.homing_feed_rate);
        } else {
            /* C11 fix: clear pending so next HOME does not auto-rehome failed axis */
            cnc_machine.homing_axes_pending = 0;
            cnc_machine.homing_phase = HOMING_IDLE;
            cnc_machine.state = CNC_STATE_ALARM;
            CNC_EnableMotors(false);
            CNC_UART_PutString("[HOMING FAIL ");
            CNC_UART_PutChar('X' + axis);
            CNC_UART_PutString(" - FEED NO HIT]\r\n");
        }
        break;

    case HOMING_PULLOFF2: {
        /* This is the home reference position */
        cnc_machine.position[axis] = 0;
        cnc_machine.homed[axis]    = true;
        cnc_machine.homing_axes_pending &= ~(1U << axis);
        cnc_machine.homing_retry_count   = 0;
        CNC_UART_PutString("["); CNC_UART_PutChar('X' + axis);
        CNC_UART_PutString(" HOMED]\r\n");

#if CNC_LIMIT_SHARED_PIN
        /* Shared-pin: the shared line must be RELEASED before starting next
           axis, otherwise next axis SEEK reads "already hit". The PULLOFF2
           distance ($27) should have cleared the switch; verify or alarm. */
        {
            uint32_t t0 = CNC_GetTick();
            while (CNC_Limit_IsTriggered(axis)) {
                if ((CNC_GetTick() - t0) > 500U) {   /* 500 ms timeout */
                    cnc_machine.homing_axes_pending = 0;
                    cnc_machine.homing_phase = HOMING_IDLE;
                    cnc_machine.state = CNC_STATE_ALARM;
                    CNC_EnableMotors(false);
                    CNC_UART_PutString("[HOMING FAIL - SHARED PIN STILL ACTIVE]\r\n");
                    return;
                }
            }
        }
#endif

        int8_t next = CNC_Homing_GetNextAxis();
        if (next >= 0) {
            cnc_machine.homing_axis  = next;
            cnc_machine.homing_phase = HOMING_SEEK;
            float ndir = ((cnc_config.homing_dir_mask >> next) & 1U) ? -1.0f : 1.0f;
            float mt   = cnc_config.max_travel[next] * CNC_HOMING_TRAVEL_OVERSHOOT;
            CNC_UART_PutString("[HOMING ");
            CNC_UART_PutChar('X' + next);
            CNC_UART_PutString(ndir < 0 ? " DIR:- SEEK]\r\n" : " DIR:+ SEEK]\r\n");
            CNC_Homing_MoveAxis(next, ndir * mt, cnc_config.homing_seek_rate);
        } else {
            cnc_machine.homing_phase = HOMING_COMPLETE;
            cnc_machine.state        = CNC_STATE_IDLE;
            CNC_UART_PutString("[HOMING COMPLETE - ");
            if (cnc_machine.homed[0]) CNC_UART_PutChar('X');
            if (cnc_machine.homed[1]) CNC_UART_PutChar('Y');
            if (cnc_machine.homed[2]) CNC_UART_PutChar('Z');
            CNC_UART_PutString(" HOMED]\r\n");
        }
        break;
    }

    case HOMING_COMPLETE:
    case HOMING_IDLE:
    default:
        /* Should not get here — fail-safe to IDLE */
        cnc_machine.state = CNC_STATE_IDLE;
        break;
    }
}

bool CNC_Homing_IsActive(void) { return (cnc_machine.state == CNC_STATE_HOMING); }

#endif /* CNC_USE_HOMING */

/*============================================================================
 *                    BACKLASH
 *============================================================================*/

#if CNC_USE_BACKLASH
void CNC_Backlash_Init(void) {
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        backlash_comp.backlash[i] = cnc_config.backlash[i];
        backlash_comp.last_direction[i] = 0;
    }
    backlash_comp.enabled = cnc_config.backlash_enabled;
}
void CNC_Backlash_SetValue(uint8_t axis, float value) {
    if (axis < CNC_NUM_AXES) {
        backlash_comp.backlash[axis] = fabsf(value);
        cnc_config.backlash[axis] = fabsf(value);
    }
}
float CNC_Backlash_GetValue(uint8_t axis) {
    return (axis < CNC_NUM_AXES) ? backlash_comp.backlash[axis] : 0.0f;
}
void CNC_Backlash_Enable(bool enable) {
    backlash_comp.enabled = enable; cnc_config.backlash_enabled = enable;
}
bool CNC_Backlash_IsEnabled(void) { return backlash_comp.enabled; }
void CNC_Backlash_Compensate(uint8_t axis, int8_t new_direction, float* delta) {
    if (!backlash_comp.enabled || axis >= CNC_NUM_AXES) return;
    if (backlash_comp.backlash[axis] < CNC_EPSILON_MM) return;
    if (new_direction == 0) return;
    int8_t last_dir = backlash_comp.last_direction[axis];
    if (last_dir != 0 && last_dir != new_direction) {
        *delta += (float)new_direction * backlash_comp.backlash[axis];
    }
    backlash_comp.last_direction[axis] = new_direction;
}
void CNC_Backlash_ResetDirections(void) {
    for (int i = 0; i < CNC_NUM_AXES; i++) backlash_comp.last_direction[i] = 0;
}
#endif

/*============================================================================
 *                    DMA UART
 *============================================================================*/

#if CNC_USE_DMA_UART
void CNC_UART_DMA_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    uart_dma_rx.write_pos = 0;
    uart_dma_rx.read_pos  = 0;
    uart_dma_overrun      = false;
    CNC_UART_DMA_CHANNEL->CCR = 0;
    DMA1->IFCR = CNC_UART_DMA_FLAG_TC | CNC_UART_DMA_FLAG_HT;
    CNC_UART_DMA_CHANNEL->CPAR  = (uint32_t)&(CNC_UART->DR);
    CNC_UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_dma_rx.buffer;
    CNC_UART_DMA_CHANNEL->CNDTR = UART_DMA_BUFFER_SIZE;
    CNC_UART_DMA_CHANNEL->CCR   = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_0;
    CNC_UART->CR3 |= USART_CR3_DMAR;
    CNC_UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;

#if CNC_USE_UART_IDLE_IRQ
    /* U3: enable USART IDLE detection so emergency commands (STOP/!) are
       picked up promptly even if main loop is busy printing. */
    CNC_UART->CR1 |= USART_CR1_IDLEIE;
    NVIC_SetPriority(CNC_UART_IRQn, 2);
    NVIC_EnableIRQ(CNC_UART_IRQn);
#endif
}
uint16_t CNC_UART_DMA_Available(void) {
    /* U1 fix: detect DMA circular wrap that crossed read_pos (data loss). */
    uint16_t dma_counter = CNC_UART_DMA_CHANNEL->CNDTR;
    uint16_t new_write   = UART_DMA_BUFFER_SIZE - dma_counter;
    uint16_t old_write   = uart_dma_rx.write_pos;
    uart_dma_rx.write_pos = new_write;

    /* A wrap happened iff new_write < old_write. If after the wrap the
       writer is now >= read_pos, the data between old_read_pos and
       new_write_pos has been overwritten in-place. Mark and recover. */
    if (new_write < old_write) {
        if (new_write >= uart_dma_rx.read_pos) {
            uart_dma_overrun = true;
            /* Recover: drop everything; keep newest 'new_write' bytes
               starting at offset 0. read_pos = new_write means "empty". */
            uart_dma_rx.read_pos = new_write;
            return 0;
        }
    }

    if (new_write >= uart_dma_rx.read_pos)
        return (uint16_t)(new_write - uart_dma_rx.read_pos);
    return (uint16_t)(UART_DMA_BUFFER_SIZE - uart_dma_rx.read_pos + new_write);
}

bool CNC_UART_DMA_HasOverrun(void) {
    bool v = uart_dma_overrun;
    uart_dma_overrun = false;
    return v;
}
uint8_t CNC_UART_DMA_Read(void) {
    if (CNC_UART_DMA_Available() == 0) return 0;
    uint8_t data = uart_dma_rx.buffer[uart_dma_rx.read_pos];
    uart_dma_rx.read_pos = (uart_dma_rx.read_pos + 1) % UART_DMA_BUFFER_SIZE;
    return data;
}
void CNC_UART_DMA_Flush(void) {
    uint16_t dma_counter = CNC_UART_DMA_CHANNEL->CNDTR;
    uart_dma_rx.write_pos = UART_DMA_BUFFER_SIZE - dma_counter;
    uart_dma_rx.read_pos  = uart_dma_rx.write_pos;
}
#endif

/*============================================================================
 *                    COMMAND QUEUE
 *============================================================================*/

#if CNC_USE_COMMAND_QUEUE
void CNC_Queue_Init(void) {
    cmd_queue.head = 0; cmd_queue.tail = 0; cmd_queue.count = 0;
    for (int i = 0; i < CMD_QUEUE_SIZE; i++) cmd_queue.commands[i][0] = '\0';
}
bool CNC_Queue_Push(const char* cmd) {
    if (CNC_Queue_IsFull()) return false;
    /* Use bounded copy that always null-terminates */
    size_t len = strlen(cmd);
    if (len >= CMD_MAX_LENGTH) len = CMD_MAX_LENGTH - 1;
    memcpy(cmd_queue.commands[cmd_queue.head], cmd, len);
    cmd_queue.commands[cmd_queue.head][len] = '\0';
    cmd_queue.head = (cmd_queue.head + 1) % CMD_QUEUE_SIZE;
    cmd_queue.count++;
    return true;
}
bool CNC_Queue_Pop(char* cmd) {
    if (CNC_Queue_IsEmpty()) return false;
    /* Source is already null-terminated (pushed via memcpy + '\0'). */
    size_t len = strlen(cmd_queue.commands[cmd_queue.tail]);
    if (len >= CMD_MAX_LENGTH) len = CMD_MAX_LENGTH - 1;
    memcpy(cmd, cmd_queue.commands[cmd_queue.tail], len);
    cmd[len] = '\0';
    cmd_queue.tail = (cmd_queue.tail + 1) % CMD_QUEUE_SIZE;
    cmd_queue.count--;
    return true;
}
bool CNC_Queue_IsFull(void)   { return (cmd_queue.count >= CMD_QUEUE_SIZE); }
bool CNC_Queue_IsEmpty(void)  { return (cmd_queue.count == 0); }
uint8_t CNC_Queue_Count(void) { return cmd_queue.count; }
uint8_t CNC_Queue_Available(void) { return CMD_QUEUE_SIZE - cmd_queue.count; }
void CNC_Queue_Clear(void)    { cmd_queue.head = cmd_queue.tail = cmd_queue.count = 0; }
#endif

/*============================================================================
 *                    MOTION PLANNER
 *============================================================================*/

#if CNC_USE_PLANNER
void CNC_Planner_Init(void) {
    planner.head = planner.tail = planner.count = 0;
    planner_first_enqueue_tick = 0;
    for (int i = 0; i < CNC_NUM_AXES; i++) planner.previous_unit_vec[i] = 0.0f;
    planner.previous_nominal_speed = 0.0f;
}

static void CNC_Planner_GetPlanningPosition(float* pos) {
    if (pos == NULL) return;
    if (planner.count > 0) {
        uint8_t last = (planner.head == 0) ? (PLANNER_BUFFER_SIZE - 1) : (planner.head - 1);
        for (int i = 0; i < CNC_NUM_AXES; i++) pos[i] = planner.blocks[last].target[i];
    } else if (cnc_machine.is_moving && cnc_machine.state == CNC_STATE_RUN) {
        /* Current executing block has already been discarded from planner.
           Plan subsequent blocks from its final physical-grid target. */
        for (int i = 0; i < CNC_NUM_AXES; i++) pos[i] = cnc_machine.logical_target[i];
    } else {
        CNC_GetPosition(pos);
    }
}

bool CNC_Planner_AddBlock(float* target, float feedrate, bool rapid) {
    /* D13 fix: refuse new blocks while in alarm or limit-triggered */
    if (cnc_machine.state == CNC_STATE_ALARM ||
        cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) return false;
#if CNC_USE_RELAY
    if (CNC_LaserPulse_IsActive()) return false; /* timed pulse owns the laser window */
#endif
    if (CNC_Planner_IsBufferFull()) return false;
    PlannerBlock_t* block = &planner.blocks[planner.head];

    float current_pos[CNC_NUM_AXES];
    CNC_Planner_GetPlanningPosition(current_pos);

    /* Snap planner targets to the physical step grid at ENQUEUE time.
       This keeps planner math, reported position, and actual step output on the
       same path for long jobs.  Without this, queued blocks are planned from raw
       float endpoints while execution later snaps to step grid, causing slow
       software/physical drift and wrong junction distances. */
    float snapped_target[CNC_NUM_AXES];
    float delta[CNC_NUM_AXES];
    bresenham_int_t max_axis_steps = 0;
    block->distance = 0.0f;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        float raw_delta = target[i] - current_pos[i];
        float spm_i = CNC_SafeStepsPerMM(i);
        bresenham_int_t asteps = (bresenham_int_t)roundf(fabsf(raw_delta * spm_i));
        if (asteps > MAX_STEPS_PER_MOVE) return false;
        int8_t mvdir = 0;
        if (raw_delta > CNC_DIR_THRESHOLD_MM) mvdir = 1;
        else if (raw_delta < -CNC_DIR_THRESHOLD_MM) mvdir = -1;
        float quantized_delta = (float)mvdir * (float)asteps / spm_i;
        snapped_target[i] = current_pos[i] + quantized_delta;
        delta[i] = snapped_target[i] - current_pos[i];
        block->distance += delta[i] * delta[i];
        if (asteps > max_axis_steps) max_axis_steps = asteps;
    }
    block->distance = sqrtf(block->distance);
    if (block->distance < CNC_EPSILON_MM || max_axis_steps == 0) {
        /* No physical step would be generated. Treat as successful no-op so
           tiny/sub-step commands do not become protocol ERROR:9. */
        return true;
    }
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        block->target[i] = snapped_target[i];
        block->unit_vec[i] = delta[i] / block->distance;
    }
    block->feedrate = feedrate;
    block->rapid    = rapid;

    block->acceleration = CNC_ComputeVectorMaxAccelMMs2(block->unit_vec);
    if (block->acceleration < CNC_MIN_ACCEL_MMS2) block->acceleration = CNC_MIN_ACCEL_MMS2;

    float v_max = CNC_ComputeVectorMaxSpeedMMs(block->unit_vec);
    if (rapid) {
        block->nominal_speed = v_max;
    } else {
        float req = feedrate / CNC_SEC_PER_MIN;
        if (req < PLANNER_MIN_SPEED) req = PLANNER_MIN_SPEED;
        block->nominal_speed = CNC_min(req, v_max);
    }
    if (block->nominal_speed < PLANNER_MIN_SPEED) block->nominal_speed = PLANNER_MIN_SPEED;
    block->nominal_speed_sqr  = block->nominal_speed * block->nominal_speed;
    block->max_entry_speed_sqr = block->nominal_speed_sqr;
    block->max_exit_speed_sqr  = block->nominal_speed_sqr;

    if (planner.count > 0) {
        uint8_t prev_idx = (planner.head == 0) ? (PLANNER_BUFFER_SIZE - 1) : (planner.head - 1);
        PlannerBlock_t* prev = &planner.blocks[prev_idx];
        /* A1 fix: junction_cos_theta = -dot(prev_dir, curr_dir).
           Interpretation:
             cos_theta = -1 -> straight line (no turn)         -> keep nominal
             cos_theta =  0 -> 90 degree turn                  -> compute slowdown
             cos_theta = +1 -> U-turn (180 deg reversal)       -> force MIN speed */
        float cos_theta = 0.0f;
        for (int i = 0; i < CNC_NUM_AXES; i++)
            cos_theta -= prev->unit_vec[i] * block->unit_vec[i];

        if (cos_theta > 0.999f) {
            /* A1: detected U-turn / sharp acute junction. Stepper must
               nearly stop before reversing direction. */
            block->entry_speed_sqr     = PLANNER_MIN_SPEED * PLANNER_MIN_SPEED;
            block->max_entry_speed_sqr = block->entry_speed_sqr;
        } else {
            /* A2: clamp cos_theta away from -1 to avoid sin_half->1 (div by 0)
               and away from +1 (already handled above). */
            if (cos_theta < -0.999f) cos_theta = -0.999f;
            float sin_half = sqrtf(0.5f * (1.0f - cos_theta));
            float junction = CNC_min(block->nominal_speed, prev->nominal_speed);
            float denom    = 1.0f - sin_half;
            if (denom > CNC_EPSILON_MM) {
                float v_j = sqrtf(block->acceleration * cnc_config.junction_deviation
                                  * sin_half / denom);
                if (v_j < junction) junction = v_j;
            }
            if (junction < PLANNER_MIN_SPEED) junction = PLANNER_MIN_SPEED;
            block->entry_speed_sqr     = junction * junction;
            block->max_entry_speed_sqr = block->entry_speed_sqr;
        }
    } else {
        block->entry_speed_sqr = PLANNER_MIN_SPEED * PLANNER_MIN_SPEED;
    }
    block->exit_speed_sqr = PLANNER_MIN_SPEED * PLANNER_MIN_SPEED;
    block->recalculate_flag = 1;
    block->nominal_length_flag = 0;

    if (planner.count == 0) planner_first_enqueue_tick = CNC_GetTick();
    planner.head = (planner.head + 1) % PLANNER_BUFFER_SIZE;
    planner.count++;
    CNC_Planner_Recalculate();
    return true;
}

void CNC_Planner_Recalculate(void) {
    if (planner.count < 2) return;
    uint8_t idx = planner.head;
    for (uint8_t i = 0; i < planner.count; i++) {
        idx = (idx == 0) ? (PLANNER_BUFFER_SIZE - 1) : (idx - 1);
        PlannerBlock_t* cur = &planner.blocks[idx];
        if (i == 0) cur->exit_speed_sqr = PLANNER_MIN_SPEED * PLANNER_MIN_SPEED;
        float max_entry = cur->exit_speed_sqr + 2.0f * cur->acceleration * cur->distance;
        if (max_entry < cur->entry_speed_sqr) {
            cur->entry_speed_sqr = max_entry;
            cur->recalculate_flag = 1;
        }
        if (i < planner.count - 1) {
            uint8_t prev_idx = (idx == 0) ? (PLANNER_BUFFER_SIZE - 1) : (idx - 1);
            PlannerBlock_t* prev = &planner.blocks[prev_idx];
            if (cur->entry_speed_sqr < prev->exit_speed_sqr) {
                prev->exit_speed_sqr = cur->entry_speed_sqr;
                prev->recalculate_flag = 1;
            }
        }
    }
    idx = planner.tail;
    for (uint8_t i = 0; i < planner.count; i++) {
        PlannerBlock_t* cur = &planner.blocks[idx];
        float max_exit = cur->entry_speed_sqr + 2.0f * cur->acceleration * cur->distance;
        if (max_exit < cur->exit_speed_sqr) cur->exit_speed_sqr = max_exit;
        if (cur->exit_speed_sqr > cur->nominal_speed_sqr) {
            cur->exit_speed_sqr = cur->nominal_speed_sqr;
            cur->nominal_length_flag = 1;
        }
        idx = (idx + 1) % PLANNER_BUFFER_SIZE;
    }
}

PlannerBlock_t* CNC_Planner_GetCurrentBlock(void) {
    return CNC_Planner_IsBufferEmpty() ? NULL : &planner.blocks[planner.tail];
}
void CNC_Planner_DiscardCurrentBlock(void) {
    if (CNC_Planner_IsBufferEmpty()) return;
    PlannerBlock_t* b = &planner.blocks[planner.tail];
    for (int i = 0; i < CNC_NUM_AXES; i++) planner.previous_unit_vec[i] = b->unit_vec[i];
    planner.previous_nominal_speed = b->nominal_speed;
    planner.tail = (planner.tail + 1) % PLANNER_BUFFER_SIZE;
    planner.count--;
}
bool CNC_Planner_IsBufferEmpty(void) { return (planner.count == 0); }
bool CNC_Planner_IsBufferFull(void)  { return (planner.count >= PLANNER_BUFFER_SIZE); }
uint8_t CNC_Planner_BlocksAvailable(void) { return PLANNER_BUFFER_SIZE - planner.count; }
uint8_t CNC_Planner_BlocksCount(void)     { return planner.count; }
void CNC_Planner_Clear(void) {
    planner.head = planner.tail = planner.count = 0;
    planner_first_enqueue_tick = 0;
    for (int i = 0; i < CNC_NUM_AXES; i++) planner.previous_unit_vec[i] = 0.0f;
    planner.previous_nominal_speed = 0.0f;
}
#endif /* CNC_USE_PLANNER */

/*============================================================================
 *                    UART BASIC
 *============================================================================*/

static void CNC_UART_Init(void) {
#if CNC_UART_APB2
    RCC->APB2ENR |= CNC_UART_RCC;
#else
    RCC->APB1ENR |= CNC_UART_RCC;
#endif
#if (CNC_UART_SELECT == 1)
    GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);
    GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;
#elif (CNC_UART_SELECT == 2)
    GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
    GPIOA->CRL |= (GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2);
    GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
    GPIOA->CRL |= GPIO_CRL_CNF3_0;
#elif (CNC_UART_SELECT == 3)
    GPIOB->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
    GPIOB->CRH |= (GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10);
    GPIOB->CRH &= ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11);
    GPIOB->CRH |= GPIO_CRH_CNF11_0;
#endif
    CNC_UART->CR1 = 0; CNC_UART->CR2 = 0; CNC_UART->CR3 = 0;
#if CNC_UART_APB2
    CNC_UART->BRR = CNC_SYSTEM_CLOCK / CNC_UART_BAUDRATE;
#else
    CNC_UART->BRR = (CNC_SYSTEM_CLOCK / 2) / CNC_UART_BAUDRATE;
#endif
    CNC_UART->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
#if !CNC_USE_DMA_UART
    CNC_UART->CR1 |= USART_CR1_RXNEIE;
#endif
    /* U-A2: enable USART IRQ in BOTH modes (TXE drain + IDLE) */
    NVIC_SetPriority(CNC_UART_IRQn, 2);
    NVIC_EnableIRQ(CNC_UART_IRQn);
    cnc_uart_rx.head = cnc_uart_rx.tail = cnc_uart_rx.count = 0;
    tx_head = tx_tail = tx_count = 0;
}

void CNC_UART_PutChar(char c) {
    __disable_irq();
    if (tx_count < UART_TX_BUFFER_SIZE) {
        tx_buf[tx_head] = (uint8_t)c;
        tx_head = (tx_head + 1) % UART_TX_BUFFER_SIZE;
        tx_count++;
        CNC_UART->CR1 |= USART_CR1_TXEIE;
        __enable_irq();
        return;
    }
    __enable_irq();
    while (!(CNC_UART->SR & USART_SR_TXE)) {}
    CNC_UART->DR = (uint8_t)c;
}
void CNC_UART_PutString(const char* str) { while (*str) CNC_UART_PutChar(*str++); }
void CNC_UART_PutInt(int32_t num) {
    char buf[12]; int i = 0; bool neg = false;
    if (num < 0) { neg = true; num = -num; }
    if (num == 0) { CNC_UART_PutChar('0'); return; }
    while (num > 0) { buf[i++] = '0' + (num % 10); num /= 10; }
    if (neg) CNC_UART_PutChar('-');
    while (i > 0) CNC_UART_PutChar(buf[--i]);
}
void CNC_UART_PutFloat(float num, uint8_t decimals) {
    if (num < 0) { CNC_UART_PutChar('-'); num = -num; }
    int32_t int_part = (int32_t)num;
    CNC_UART_PutInt(int_part);
    if (decimals > 0) {
        CNC_UART_PutChar('.');
        float frac = num - (float)int_part;
        for (uint8_t i = 0; i < decimals; i++) {
            frac *= 10.0f;
            int digit = (int)frac % 10;
            CNC_UART_PutChar('0' + digit);
        }
    }
}
bool CNC_UART_Available(void) {
#if CNC_USE_DMA_UART
    return (CNC_UART_DMA_Available() > 0);
#else
    return (cnc_uart_rx.count > 0);
#endif
}
static char CNC_UART_GetCharRaw(void) {
#if CNC_USE_DMA_UART
    return CNC_UART_DMA_Read();
#else
    if (cnc_uart_rx.count == 0) return 0;
    char c = cnc_uart_rx.buffer[cnc_uart_rx.tail];
    cnc_uart_rx.tail = (cnc_uart_rx.tail + 1) % UART_BUFFER_SIZE;
    cnc_uart_rx.count--;
    return c;
#endif
}

static bool CNC_UART_RealtimeFilter(char c) {
    switch (c) {
        case '?':  cnc_rt_status_req = true; return true;
        case '!':  cnc_rt_feed_hold  = true; return true;
        case '~':  cnc_rt_resume     = true; return true;
        case 0x18: cnc_rt_soft_reset = true; return true;
        default:   return false;
    }
}

char CNC_UART_GetChar(void) {
    while (1) {
        char c = CNC_UART_GetCharRaw();
        if (c == 0) return 0;
        if (!CNC_UART_RealtimeFilter(c)) return c;
    }
}
/* U2.1/U2.2/U2.3 fix: bounded line accumulator with overflow + noise filter
   and a public reset hook used by the RESET command. */
static char     getline_buf[GCODE_LINE_SIZE];
static uint16_t getline_idx      = 0;
static bool     getline_overflow = false;

void CNC_UART_GetLine_Reset(void) {
    getline_idx      = 0;
    getline_overflow = false;
}

bool CNC_UART_GetLine(char* line, uint16_t max_len) {
    /* U1: emit warning once when DMA overrun was detected since last call. */
    if (CNC_UART_DMA_HasOverrun()) {
        CNC_UART_PutString("[RX OVERRUN]\r\n");
        getline_idx      = 0;
        getline_overflow = true;   /* current half-line is unreliable */
    }
    while (CNC_UART_Available()) {
        char c = CNC_UART_GetChar();
        /* U2.3: filter binary noise (control chars except \r \n \t) */
        if ((unsigned char)c < 0x20 && c != '\n' && c != '\r' && c != '\t') continue;
        if (c == '\r') continue;
        if (c == '\n') {
            if (getline_overflow) {
                /* U2.1: line too long was silently truncated before;
                   now we discard the whole line and report ERROR. */
                CNC_UART_PutString("[ERROR: Line too long, discarded]\r\n");
                getline_overflow = false;
                getline_idx      = 0;
                continue;
            }
            getline_buf[getline_idx] = '\0';
            strncpy(line, getline_buf, max_len);
            line[max_len - 1] = '\0';
            getline_idx = 0;
            return true;
        }
        if (getline_idx < GCODE_LINE_SIZE - 1) {
            if (c >= 'a' && c <= 'z') c -= 32;
            getline_buf[getline_idx++] = c;
        } else {
            getline_overflow = true;   /* keep consuming until newline */
        }
    }
    return false;
}
void CNC_UART_SendOK(void)    { CNC_UART_PutString("OK\r\n"); }
void CNC_UART_SendError(uint8_t code) {
    CNC_UART_PutString("ERROR:"); CNC_UART_PutInt(code); CNC_UART_PutString("\r\n");
}

void CNC_UART_IRQHandler(void) {
    uint32_t sr = CNC_UART->SR;
#if !CNC_USE_DMA_UART
    if (sr & USART_SR_RXNE) {
        uint8_t data = (uint8_t)CNC_UART->DR;
        if (cnc_uart_rx.count < UART_BUFFER_SIZE) {
            cnc_uart_rx.buffer[cnc_uart_rx.head] = data;
            cnc_uart_rx.head = (cnc_uart_rx.head + 1) % UART_BUFFER_SIZE;
            cnc_uart_rx.count++;
        }
    }
#endif
#if CNC_USE_DMA_UART && CNC_USE_UART_IDLE_IRQ
    /* U3: IDLE detection — clear by reading SR then DR. */
    if (sr & USART_SR_IDLE) {
        volatile uint32_t tmp = CNC_UART->SR; tmp = CNC_UART->DR; (void)tmp;
    }
#endif
    /* U-A2: TX ring buffer drain via TXE IRQ */
    if ((sr & USART_SR_TXE) && (CNC_UART->CR1 & USART_CR1_TXEIE)) {
        if (tx_count > 0) {
            CNC_UART->DR = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % UART_TX_BUFFER_SIZE;
            tx_count--;
        }
        if (tx_count == 0) {
            CNC_UART->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    if (sr & USART_SR_ORE) {
        volatile uint32_t tmp = CNC_UART->SR; tmp = CNC_UART->DR; (void)tmp;
    }
}

/*============================================================================
 *                    MOTION CONTROL
 *============================================================================*/

void CNC_Move(float* target, float feedrate, bool rapid) {
    /* Note: called from main context only (CNC_Process / planner-execute) */
    while (cnc_machine.is_moving) __NOP();
#if CNC_USE_RELAY
    if (CNC_LaserPulse_IsActive()) {
        CNC_UART_PutString("[BUSY - LASER PULSE ACTIVE]\r\n");
        return;
    }
#endif
    /* Permit moves during unlock-escape (state would be IDLE already after CLEAR
       but be defensive: also allow during the brief LIMIT/ALARM->IDLE transition) */
    if ((cnc_machine.state == CNC_STATE_ALARM ||
         cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) && !cnc_unlock_active) {
        CNC_UART_PutString("[ALARM - CLEAR FIRST]\r\n");
        return;
    }
    if (!cnc_machine.motors_enabled) CNC_EnableMotors(true);

    float delta[CNC_NUM_AXES], comp[CNC_NUM_AXES], uvec[CNC_NUM_AXES] = {0};
    bresenham_int_t new_total = 0;
    int dom = 0;
    float motion_dist = 0.0f;

    for (int i = 0; i < CNC_NUM_AXES; i++) {
        delta[i] = target[i] - cnc_machine.position[i];
        comp[i]  = delta[i];
#if CNC_USE_BACKLASH
        int8_t nd = 0;
        if (delta[i] > CNC_DIR_THRESHOLD_MM) nd = 1; else if (delta[i] < -CNC_DIR_THRESHOLD_MM) nd = -1;
        if (cnc_config.backlash_enabled && nd != 0)
            CNC_Backlash_Compensate(i, nd, &comp[i]);
#endif
        cnc_machine.logical_delta[i]       = delta[i];

        float spm_i = CNC_SafeStepsPerMM(i);
        bresenham_int_t asteps = (bresenham_int_t)roundf(fabsf(comp[i] * spm_i));
        if (asteps > MAX_STEPS_PER_MOVE) {
            CNC_UART_PutString("[ERROR: Move too long]\r\n"); return;
        }
        bresenham_steps[i]        = asteps;
        cnc_machine.step_count[i] = (int32_t)asteps;
        if (asteps > new_total) { new_total = asteps; dom = i; }

        int8_t mvdir = 0;
        if (comp[i] > CNC_DIR_THRESHOLD_MM)      { CNC_SetDirection(i, 1);  mvdir =  1; }
        else if (comp[i] < -CNC_DIR_THRESHOLD_MM){ CNC_SetDirection(i, -1); mvdir = -1; }
        else                                      cnc_machine.direction[i] = 0;

        /* P1+P5 fix: snap target to PHYSICAL position the steps will produce.
           Eliminates float drift across many short moves. */
        float quantized_delta = (float)mvdir * (float)asteps / spm_i;
        cnc_machine.target[i] = cnc_machine.position[i] + quantized_delta;

        motion_dist += comp[i] * comp[i];
    }
    motion_dist = sqrtf(motion_dist);
    if (new_total == 0) {
        /* All requested deltas rounded to 0 physical steps.  Do NOT jump the
           reported position to the raw target: no motor step occurred, so doing
           so would create software/physical drift after many sub-step moves.
           Keep position on the physical step grid and just report completion. */
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            cnc_machine.target[i] = cnc_machine.position[i];
            cnc_machine.logical_target[i] = cnc_machine.position[i];
            cnc_machine.step_count[i] = 0;
        }
        cnc_machine.is_moving = false;
        return;
    }
    if (motion_dist > CNC_DIR_THRESHOLD_MM) {
        for (int i = 0; i < CNC_NUM_AXES; i++) uvec[i] = comp[i] / motion_dist;
    }

    for (int i = 0; i < CNC_NUM_AXES; i++) bresenham_counter[i] = new_total / 2;
    /* B10 fix: cache inv_steps_per_mm and logical_target before ISR uses them.
       P1 fix: logical_target is the SNAPPED target (cnc_machine.target),
       not the raw input, so completion handler lands on physical grid. */
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        cnc_machine.inv_steps_per_mm[i] = 1.0f / CNC_SafeStepsPerMM(i);
        cnc_machine.logical_target[i]   = cnc_machine.target[i];
    }
    __disable_irq();
    bresenham_total = new_total;
    step_events_remaining = new_total;
    __enable_irq();

    float vec_max_feed = CNC_ComputeVectorMaxSpeedMMs(uvec) * CNC_SEC_PER_MIN;
    float eff = rapid ? vec_max_feed : CNC_min(feedrate, vec_max_feed);
    if (eff < 1.0f) eff = 1.0f;

    cnc_machine.dominant_axis = dom;
    cnc_machine.move_distance = motion_dist;

#if CNC_USE_ACCELERATION
    float a = CNC_ComputeVectorMaxAccelMMs2(uvec);
    CNC_Accel_Plan(motion_dist, eff, a);
#else
    float spm = CNC_SafeStepsPerMM(dom);
    uint32_t freq = (uint32_t)((eff / CNC_SEC_PER_MIN) * spm);
    if (freq < CNC_MIN_STEP_FREQ_RUNTIME) freq = CNC_MIN_STEP_FREQ_RUNTIME;
    if (freq > CNC_MAX_STEP_FREQ) freq = CNC_MAX_STEP_FREQ;
    CNC_Timer_SetPeriod(CNC_US_PER_SEC / freq);
#endif

    cnc_machine.is_moving = true;
    if (cnc_machine.state == CNC_STATE_IDLE) cnc_machine.state = CNC_STATE_RUN;
    if (cnc_unlock_active) cnc_unlock_move_started = true; /* unlock: ghi nhan move da chay */
    CNC_Timer_Start();
}

void CNC_Move_NoAccel(float* target, float feedrate) {
    while (cnc_machine.is_moving) __NOP();
#if CNC_USE_RELAY
    if (CNC_LaserPulse_IsActive()) {
        CNC_UART_PutString("[BUSY - LASER PULSE ACTIVE]\r\n");
        return;
    }
#endif
    if ((cnc_machine.state == CNC_STATE_ALARM ||
         cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) && !cnc_unlock_active) {
        CNC_UART_PutString("[ALARM - CLEAR FIRST]\r\n"); return;
    }
    if (!cnc_machine.motors_enabled) CNC_EnableMotors(true);

    float delta[CNC_NUM_AXES], comp[CNC_NUM_AXES], uvec[CNC_NUM_AXES] = {0};
    bresenham_int_t new_total = 0; int dom = 0; float md = 0.0f;

    for (int i = 0; i < CNC_NUM_AXES; i++) {
        delta[i] = target[i] - cnc_machine.position[i];
        comp[i]  = delta[i];
#if CNC_USE_BACKLASH
        int8_t nd = 0;
        if (delta[i] > CNC_DIR_THRESHOLD_MM) nd = 1; else if (delta[i] < -CNC_DIR_THRESHOLD_MM) nd = -1;
        if (cnc_config.backlash_enabled && nd != 0)
            CNC_Backlash_Compensate(i, nd, &comp[i]);
#endif
        cnc_machine.logical_delta[i]       = delta[i];

        float spm_i = CNC_SafeStepsPerMM(i);
        bresenham_int_t asteps = (bresenham_int_t)roundf(fabsf(comp[i] * spm_i));
        if (asteps > MAX_STEPS_PER_MOVE) {
            CNC_UART_PutString("[ERROR: Move too long]\r\n"); return;
        }
        bresenham_steps[i]        = asteps;
        cnc_machine.step_count[i] = (int32_t)asteps;
        if (asteps > new_total) { new_total = asteps; dom = i; }

        int8_t mvdir = 0;
        if (comp[i] > CNC_DIR_THRESHOLD_MM)      { CNC_SetDirection(i, 1);  mvdir =  1; }
        else if (comp[i] < -CNC_DIR_THRESHOLD_MM){ CNC_SetDirection(i, -1); mvdir = -1; }
        else                                      cnc_machine.direction[i] = 0;

        /* P1+P5: snap target to quantized physical position */
        float quantized_delta = (float)mvdir * (float)asteps / spm_i;
        cnc_machine.target[i] = cnc_machine.position[i] + quantized_delta;

        md += comp[i] * comp[i];
    }
    md = sqrtf(md);
    if (new_total == 0) {
        /* All requested deltas rounded to 0 physical steps.  Keep reported
           position equal to actual physical position; callers still advance
           because is_moving is false. */
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            cnc_machine.target[i] = cnc_machine.position[i];
            cnc_machine.logical_target[i] = cnc_machine.position[i];
            cnc_machine.step_count[i] = 0;
        }
        cnc_machine.is_moving = false;
        return;
    }
    if (md > CNC_DIR_THRESHOLD_MM) for (int i = 0; i < CNC_NUM_AXES; i++) uvec[i] = comp[i] / md;

    for (int i = 0; i < CNC_NUM_AXES; i++) bresenham_counter[i] = new_total / 2;
    /* B10+P1: cache snapped target so completion uses physical grid */
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        cnc_machine.inv_steps_per_mm[i] = 1.0f / CNC_SafeStepsPerMM(i);
        cnc_machine.logical_target[i]   = cnc_machine.target[i];
    }
    __disable_irq();
    bresenham_total = new_total;
    step_events_remaining = new_total;
    __enable_irq();

    float vec_max = CNC_ComputeVectorMaxSpeedMMs(uvec) * CNC_SEC_PER_MIN;
    float eff = CNC_min(feedrate, vec_max);
    if (eff < 1.0f) eff = 1.0f;

    cnc_machine.dominant_axis = dom;
    cnc_machine.move_distance = md;

    float spm = CNC_SafeStepsPerMM(dom);
    uint32_t freq = (uint32_t)((eff / CNC_SEC_PER_MIN) * spm);
    if (freq < CNC_MIN_STEP_FREQ_RUNTIME) freq = CNC_MIN_STEP_FREQ_RUNTIME;
    if (freq > CNC_MAX_STEP_FREQ) freq = CNC_MAX_STEP_FREQ;
    CNC_Timer_SetPeriod(CNC_US_PER_SEC / freq);

#if CNC_USE_ACCELERATION
    /* D11 fix: fully initialize accel state so ISR-side helpers stay safe */
    cnc_machine.accel.phase            = ACCEL_PHASE_CRUISE;
    cnc_machine.accel.current_speed    = eff / CNC_SEC_PER_MIN;
    cnc_machine.accel.target_speed     = eff / CNC_SEC_PER_MIN;
    cnc_machine.accel.entry_speed      = eff / CNC_SEC_PER_MIN;
    cnc_machine.accel.exit_speed       = eff / CNC_SEC_PER_MIN;
    cnc_machine.accel.accel_rate       = 1.0f;
    cnc_machine.accel.spm_dominant     = spm;
    cnc_machine.accel.inv_spm_dominant = (spm > 0.01f) ? (1.0f / spm) : 1.0f;
    cnc_machine.accel.mm_per_step      = (md > CNC_DIR_THRESHOLD_MM && new_total > 0)
                                         ? md / (float)new_total : 0.0f;
    cnc_machine.accel.needs_recalc     = 0;
#endif

    cnc_machine.is_moving = true;
    if (cnc_unlock_active) cnc_unlock_move_started = true;
    CNC_Timer_Start();
}

bool CNC_IsMoving(void) { return cnc_machine.is_moving; }

void CNC_Stop(void) {
    CNC_Timer_Stop();
#if CNC_USE_RELAY
    CNC_LaserPulse_Abort();
    CNC_Spindle_Off();
#endif
    cnc_machine.is_moving = false;
    cnc_send_ok_flag = false;
    cnc_planner_executing = false;
    __disable_irq();
    step_events_remaining = 0;
    bresenham_total = 0;
    __enable_irq();
    for (int i = 0; i < CNC_NUM_AXES; i++) cnc_machine.step_count[i] = 0;
#if CNC_USE_ACCELERATION
    cnc_machine.accel.mm_per_step = 0.0f;
    cnc_machine.accel.needs_recalc = 0;
    cnc_machine.accel.precomp_period = 0;
    cnc_machine.accel.last_update_step_done = 0;
#endif
    if (cnc_machine.state == CNC_STATE_GOWDELAY)      cnc_machine.gowdelay.phase = GOWDELAY_IDLE;
    if (cnc_machine.state == CNC_STATE_GOWDELAY_RECT) cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
    cnc_unlock_active      = false;   /* STOP huy ca unlock */
    cnc_unlock_move_started = false;
    if (cnc_machine.state == CNC_STATE_RUN ||
        cnc_machine.state == CNC_STATE_HOMING ||
        cnc_machine.state == CNC_STATE_GOWDELAY ||
        cnc_machine.state == CNC_STATE_GOWDELAY_RECT) {
        cnc_machine.state = CNC_STATE_IDLE;
    }
    CNC_ClearBufferedMotion();
}

void CNC_WaitComplete(void) {
    while (cnc_machine.is_moving) {
#if CNC_USE_LIMIT_SWITCH
        CNC_Limit_CheckAll();
#endif
        __NOP();
    }
}

void CNC_SetPosition(float* pos) {
    for (int i = 0; i < CNC_NUM_AXES; i++) cnc_machine.position[i] = pos[i];
#if CNC_USE_BACKLASH
    CNC_Backlash_ResetDirections();
#endif
}
void CNC_SetAxisPosition(uint8_t axis, float value) {
    if (axis < CNC_NUM_AXES) {
        cnc_machine.position[axis] = value;
#if CNC_USE_BACKLASH
        backlash_comp.last_direction[axis] = 0;
#endif
    }
}
void CNC_GetPosition(float* pos) {
    for (int i = 0; i < CNC_NUM_AXES; i++) pos[i] = cnc_machine.position[i];
}

/*============================================================================
 *                    GOWDELAY (line stepped)
 *============================================================================*/

void CNC_GoWDelay_Start(float* target, float step_distance, uint32_t delay_ms, float feedrate) {
    if (step_distance <= CNC_EPSILON_MM) { CNC_UART_PutString("[ERROR: D must be > 0]\r\n"); return; }
    if (cnc_machine.state != CNC_STATE_IDLE) { CNC_UART_PutString("[ERROR: Machine busy]\r\n"); return; }

    GoWDelayState_t* g = &cnc_machine.gowdelay;
    CNC_GetPosition(g->start_pos);
    for (int i = 0; i < CNC_NUM_AXES; i++) g->target_pos[i] = target[i];

    float delta[CNC_NUM_AXES]; g->total_distance = 0;
    for (int i = 0; i < CNC_NUM_AXES; i++) {
        delta[i] = target[i] - g->start_pos[i];
        g->total_distance += delta[i] * delta[i];
    }
    g->total_distance = sqrtf(g->total_distance);
    if (g->total_distance < CNC_EPSILON_MM) { CNC_UART_PutString("[NO MOVEMENT]\r\n"); return; }
    for (int i = 0; i < CNC_NUM_AXES; i++) g->unit_vector[i] = delta[i] / g->total_distance;

    uint32_t full = (uint32_t)(g->total_distance / step_distance);
    float rem = g->total_distance - (float)full * step_distance;
    g->total_steps = (rem > CNC_EPSILON_MM) ? (full + 1) : (full == 0 ? 1 : full);

    g->step_distance     = step_distance;
    g->delay_ms          = delay_ms;
    g->feedrate          = (feedrate > 0) ? feedrate : current_feedrate;
    g->current_step      = 0;
    g->distance_moved    = 0;
    g->delay_start_tick  = 0;

    cnc_machine.state = CNC_STATE_GOWDELAY;
    g->phase = GOWDELAY_MOVING;

    CNC_UART_PutString("[GOWDELAY: ");
    CNC_UART_PutInt(g->total_steps); CNC_UART_PutString(" steps]\r\n");
    CNC_EnableMotors(true);

    float next[CNC_NUM_AXES];
    float sd = CNC_min(g->step_distance, g->total_distance);
    g->distance_moved = sd;
    for (int i = 0; i < CNC_NUM_AXES; i++)
        next[i] = g->start_pos[i] + g->unit_vector[i] * g->distance_moved;
    CNC_Move_NoAccel(next, g->feedrate);
}

static void CNC_GoWDelay_NextStep(void) {
    GoWDelayState_t* g = &cnc_machine.gowdelay;
    float next[CNC_NUM_AXES];
    float remaining = g->total_distance - g->distance_moved;
    if (g->current_step >= g->total_steps - 1 || remaining <= g->step_distance + CNC_EPSILON_MM) {
        for (int i = 0; i < CNC_NUM_AXES; i++) next[i] = g->target_pos[i];
        g->distance_moved = g->total_distance;
    } else {
        float sd = CNC_min(g->step_distance, remaining);
        g->distance_moved += sd;
        for (int i = 0; i < CNC_NUM_AXES; i++)
            next[i] = g->start_pos[i] + g->unit_vector[i] * g->distance_moved;
    }
    g->phase = GOWDELAY_MOVING;
    CNC_Move_NoAccel(next, g->feedrate);
}

void CNC_GoWDelay_Process(void) {
    if (cnc_machine.state != CNC_STATE_GOWDELAY) return;
    GoWDelayState_t* g = &cnc_machine.gowdelay;
    switch (g->phase) {
    case GOWDELAY_MOVING:
        if (!cnc_machine.is_moving) {
            g->current_step++;
            if (g->current_step >= g->total_steps) {
                g->phase = GOWDELAY_COMPLETE;
                cnc_machine.state = CNC_STATE_IDLE;
                CNC_UART_PutString("[GOWDELAY COMPLETE]\r\n");
                return;
            }
            if (g->delay_ms > 0) {
                g->phase = GOWDELAY_WAITING;
                g->delay_start_tick = CNC_GetTick();
            } else CNC_GoWDelay_NextStep();
        }
        break;
    case GOWDELAY_WAITING:
        if ((CNC_GetTick() - g->delay_start_tick) >= g->delay_ms) CNC_GoWDelay_NextStep();
        break;
    case GOWDELAY_COMPLETE:
    case GOWDELAY_IDLE:
    default:
        /* L12: phase corrupt — go to ALARM, not silent IDLE */
        CNC_Stop();
        cnc_machine.state = CNC_STATE_ALARM;
        CNC_UART_PutString("[GOWDELAY PHASE ERROR -> ALARM]\r\n");
        break;
    }
}

void CNC_GoWDelay_Stop(void) {
    if (cnc_machine.state == CNC_STATE_GOWDELAY) {
        CNC_Stop();
        cnc_machine.gowdelay.phase = GOWDELAY_IDLE;
        cnc_machine.state = CNC_STATE_IDLE;
        CNC_UART_PutString("[GOWDELAY ABORTED]\r\n");
    }
}
bool CNC_GoWDelay_IsActive(void) { return (cnc_machine.state == CNC_STATE_GOWDELAY); }

/*============================================================================
 *                    GOWDELAY RECT (raster)
 *============================================================================*/

void CNC_GoWDelayRect_Start(float size_x, float size_y, float step_distance,
                            uint32_t delay_ms, float feedrate) {
    if (step_distance <= CNC_EPSILON_MM) { CNC_UART_PutString("[ERROR: D must be > 0]\r\n"); return; }
    if (fabsf(size_x) < CNC_EPSILON_MM || fabsf(size_y) < CNC_EPSILON_MM) {
        CNC_UART_PutString("[ERROR: X and Y must be != 0]\r\n"); return;
    }
    if (cnc_machine.state != CNC_STATE_IDLE) {
        CNC_UART_PutString("[ERROR: Machine busy]\r\n"); return;
    }
    GoWDelayRectState_t* r = &cnc_machine.gowdelay_rect;
    CNC_GetPosition(r->start_pos);
    r->rect_size_x = size_x; r->rect_size_y = size_y;
    r->step_distance = step_distance;
    r->delay_ms = delay_ms;
    r->feedrate = (feedrate > 0) ? feedrate : current_feedrate;

    float ax = fabsf(size_x), ay = fabsf(size_y);
    uint32_t fr = (uint32_t)(ay / step_distance);
    float yr = ay - (float)fr * step_distance;
    r->total_rows = (yr > CNC_EPSILON_MM) ? (fr + 1) : (fr == 0 ? 1 : fr);

    uint32_t fx = (uint32_t)(ax / step_distance);
    float xr = ax - (float)fx * step_distance;
    r->steps_per_row = (xr > CNC_EPSILON_MM) ? (fx + 1) : (fx == 0 ? 1 : fx);
    r->total_steps = r->steps_per_row * r->total_rows;

    r->current_x = 0; r->current_y = 0;
    r->x_direction = (size_x >= 0) ? 1 : -1;
    r->current_row = 0;
    r->current_step_in_row = 0;
    r->completed_steps = 0;
    r->delay_start_tick = 0;

    cnc_machine.state = CNC_STATE_GOWDELAY_RECT;
    r->phase = GWDRECT_MOVE_X;
    CNC_UART_PutString("[GOWDELAYRECT START]\r\n");
    CNC_EnableMotors(true);
    CNC_GoWDelayRect_NextStepX();
}

static void CNC_GoWDelayRect_NextStepX(void) {
    GoWDelayRectState_t* r = &cnc_machine.gowdelay_rect;
    float next[CNC_NUM_AXES];
    float ax = fabsf(r->rect_size_x);
    if (r->rect_size_x >= 0) {
        if (r->x_direction > 0) {
            r->current_x += r->step_distance;
            if (r->current_x > ax) r->current_x = ax;
        } else {
            r->current_x -= r->step_distance;
            if (r->current_x < 0) r->current_x = 0;
        }
    } else {
        if (r->x_direction < 0) {
            r->current_x -= r->step_distance;
            if (r->current_x < -ax) r->current_x = -ax;
        } else {
            r->current_x += r->step_distance;
            if (r->current_x > 0) r->current_x = 0;
        }
    }
    next[0] = r->start_pos[0] + r->current_x;
    next[1] = r->start_pos[1] + r->current_y;
    next[2] = r->start_pos[2];
    r->phase = GWDRECT_MOVE_X;
    CNC_Move_NoAccel(next, r->feedrate);
}

static void CNC_GoWDelayRect_NextStepY(void) {
    GoWDelayRectState_t* r = &cnc_machine.gowdelay_rect;
    float next[CNC_NUM_AXES];
    float ay = fabsf(r->rect_size_y);
    if (r->rect_size_y >= 0) {
        float rem = ay - r->current_y;
        float sy = CNC_min(r->step_distance, rem);
        r->current_y += sy;
        if (r->current_y > ay) r->current_y = ay;
    } else {
        float rem = ay + r->current_y;
        float sy = CNC_min(r->step_distance, rem);
        r->current_y -= sy;
        if (r->current_y < -ay) r->current_y = -ay;
    }
    next[0] = r->start_pos[0] + r->current_x;
    next[1] = r->start_pos[1] + r->current_y;
    next[2] = r->start_pos[2];
    r->phase = GWDRECT_MOVE_Y;
    CNC_Move_NoAccel(next, r->feedrate);
}

void CNC_GoWDelayRect_Process(void) {
    if (cnc_machine.state != CNC_STATE_GOWDELAY_RECT) return;
    GoWDelayRectState_t* r = &cnc_machine.gowdelay_rect;
    float ax = fabsf(r->rect_size_x), ay = fabsf(r->rect_size_y);

    switch (r->phase) {
    case GWDRECT_MOVE_X:
        if (!cnc_machine.is_moving) {
            r->current_step_in_row++; r->completed_steps++;
            bool row_done = false;
            if (r->rect_size_x >= 0) {
                if (r->x_direction > 0) row_done = (r->current_x >= ax - CNC_EPSILON_MM);
                else                    row_done = (r->current_x <=  CNC_EPSILON_MM);
            } else {
                if (r->x_direction < 0) row_done = (r->current_x <= -ax + CNC_EPSILON_MM);
                else                    row_done = (r->current_x >= -CNC_EPSILON_MM);
            }
            if (row_done) {
                bool all_done;
                if (r->rect_size_y >= 0) all_done = (r->current_y >= ay - CNC_EPSILON_MM);
                else                     all_done = (r->current_y <= -ay + CNC_EPSILON_MM);
                if (all_done) {
                    r->phase = GWDRECT_COMPLETE;
                    cnc_machine.state = CNC_STATE_IDLE;
                    CNC_UART_PutString("[GOWDELAYRECT COMPLETE]\r\n");
                    return;
                }
                r->x_direction = -r->x_direction;
                r->current_row++;
                r->current_step_in_row = 0;
                if (r->delay_ms > 0) {
                    r->phase = GWDRECT_WAIT_X;
                    r->delay_start_tick = CNC_GetTick();
                } else CNC_GoWDelayRect_NextStepY();
            } else {
                if (r->delay_ms > 0) {
                    r->phase = GWDRECT_WAIT_X;
                    r->delay_start_tick = CNC_GetTick();
                } else CNC_GoWDelayRect_NextStepX();
            }
        }
        break;
    case GWDRECT_WAIT_X:
        if ((CNC_GetTick() - r->delay_start_tick) >= r->delay_ms) {
            if (r->current_step_in_row == 0) CNC_GoWDelayRect_NextStepY();
            else                              CNC_GoWDelayRect_NextStepX();
        }
        break;
    case GWDRECT_MOVE_Y:
        if (!cnc_machine.is_moving) {
            if (r->delay_ms > 0) {
                r->phase = GWDRECT_WAIT_Y;
                r->delay_start_tick = CNC_GetTick();
            } else CNC_GoWDelayRect_NextStepX();
        }
        break;
    case GWDRECT_WAIT_Y:
        if ((CNC_GetTick() - r->delay_start_tick) >= r->delay_ms) CNC_GoWDelayRect_NextStepX();
        break;
    case GWDRECT_COMPLETE:
    case GWDRECT_IDLE:
    default:
        /* L12: phase corrupt — ALARM */
        CNC_Stop();
        cnc_machine.state = CNC_STATE_ALARM;
        CNC_UART_PutString("[GWDRECT PHASE ERROR -> ALARM]\r\n");
        break;
    }
}

void CNC_GoWDelayRect_Stop(void) {
    if (cnc_machine.state == CNC_STATE_GOWDELAY_RECT) {
        CNC_Stop();
        cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
        cnc_machine.state = CNC_STATE_IDLE;
        CNC_UART_PutString("[GOWDELAYRECT ABORTED]\r\n");
    }
}
bool CNC_GoWDelayRect_IsActive(void) { return (cnc_machine.state == CNC_STATE_GOWDELAY_RECT); }

/*============================================================================
 *                    CONFIG SHOW/SET
 *============================================================================*/

/* G4: GRBL v1.1 standard $$ output - one setting per line, no decoration */
static void CNC_PrintSetting(int idx, float value, uint8_t decimals) {
    CNC_UART_PutChar('$');
    CNC_UART_PutInt(idx);
    CNC_UART_PutChar('=');
    CNC_UART_PutFloat(value, decimals);
    CNC_UART_PutString("\r\n");
}

void CNC_Config_Show(void) {
    /* Order strictly follows GRBL v1.1 reference output */
    CNC_PrintSetting(0,  (float)cnc_config.step_pulse_us, 0);
    CNC_PrintSetting(1,  (float)cnc_config.step_idle_ms,  0);
    CNC_PrintSetting(2,  (float)cnc_config.step_invert_mask,  0);
    CNC_PrintSetting(3,  (float)cnc_config.dir_invert_mask,   0);
    CNC_PrintSetting(4,  (float)cnc_config.enable_invert,     0);
    CNC_PrintSetting(5,  (float)cnc_config.limit_invert_mask, 0);
    CNC_PrintSetting(11, cnc_config.junction_deviation, 3);
    CNC_PrintSetting(20, cnc_config.soft_limit_enabled ? 1.0f : 0.0f, 0);
    CNC_PrintSetting(21, cnc_config.hard_limit_enabled ? 1.0f : 0.0f, 0);
#if CNC_USE_HOMING
    CNC_PrintSetting(22, cnc_config.homing_enabled ? 1.0f : 0.0f, 0);
    CNC_PrintSetting(23, (float)cnc_config.homing_dir_mask,    0);
    CNC_PrintSetting(24, cnc_config.homing_feed_rate, 1);
    CNC_PrintSetting(25, cnc_config.homing_seek_rate, 1);
    CNC_PrintSetting(26, (float)cnc_config.homing_debounce_ms, 0);
    CNC_PrintSetting(27, cnc_config.homing_pulloff,   3);
    CNC_PrintSetting(28, (float)cnc_config.homing_cycle, 0);
    CNC_PrintSetting(29, (float)cnc_config.homing_axis_mask, 0);    /* custom */
#endif
    CNC_PrintSetting(40, (float)cnc_config.auto_report_interval, 0); /* custom */
    /* Per-axis settings: $100..102, $110..112, $120..122, $130..132 */
    for (int i = 0; i < CNC_NUM_AXES; i++)
        CNC_PrintSetting(100 + i, cnc_config.steps_per_mm[i], 3);
    for (int i = 0; i < CNC_NUM_AXES; i++)
        CNC_PrintSetting(110 + i, cnc_config.max_feedrate[i], 3);
#if CNC_USE_ACCELERATION
    for (int i = 0; i < CNC_NUM_AXES; i++)
        CNC_PrintSetting(120 + i, cnc_config.acceleration[i], 3);
#endif
    for (int i = 0; i < CNC_NUM_AXES; i++)
        CNC_PrintSetting(130 + i, cnc_config.max_travel[i], 3);
#if CNC_USE_BACKLASH
    for (int i = 0; i < CNC_NUM_AXES; i++)
        CNC_PrintSetting(140 + i, cnc_config.backlash[i], 3);
    CNC_PrintSetting(143, cnc_config.backlash_enabled ? 1.0f : 0.0f, 0);
#endif
}

void CNC_Config_Set(const char* param, float value) {
    if (param == NULL || param[0] != '$') return;
    int idx = atoi(&param[1]);
    float applied = 0.0f;
    switch (idx) {
    /* G1: GRBL v1.1 standard settings */
    case 0: cnc_config.step_pulse_us = (uint16_t)CNC_Config_ClampU32(value, 1, 1000);
            applied = (float)cnc_config.step_pulse_us; break;
    case 1: cnc_config.step_idle_ms  = (uint16_t)CNC_Config_ClampU32(value, 0, 255);
            applied = (float)cnc_config.step_idle_ms; break;
    case 2: cnc_config.step_invert_mask  = CNC_Config_ClampMask3(value); applied = cnc_config.step_invert_mask;  break;
    case 3: cnc_config.dir_invert_mask   = CNC_Config_ClampMask3(value); applied = cnc_config.dir_invert_mask;   break;
    case 4: cnc_config.enable_invert     = CNC_Config_ClampBool(value);  applied = cnc_config.enable_invert;     break;
    case 5: cnc_config.limit_invert_mask = CNC_Config_ClampMask3(value); applied = cnc_config.limit_invert_mask; break;
    case 11: cnc_config.junction_deviation = CNC_Config_ClampFloat(value, 0.001f, 1.0f);
             applied = cnc_config.junction_deviation; break;
    case 20: cnc_config.soft_limit_enabled = CNC_Config_ClampBool(value); applied = cnc_config.soft_limit_enabled ? 1.0f : 0.0f; break;
    case 21:
#if CNC_USE_LIMIT_SWITCH
        /* C3 safety: hard limit cannot be disabled when switch hardware exists.
           Silently force back to 1 and warn. */
        cnc_config.hard_limit_enabled = true;
        applied = 1.0f;
        if (!CNC_Config_ClampBool(value)) {
            CNC_UART_PutString("[WARN: $21 forced to 1 for safety]\r\n");
        }
#else
        cnc_config.hard_limit_enabled = CNC_Config_ClampBool(value);
        applied = cnc_config.hard_limit_enabled ? 1.0f : 0.0f;
#endif
        break;
#if CNC_USE_HOMING
    case 22: cnc_config.homing_enabled   = CNC_Config_ClampBool(value);  applied = cnc_config.homing_enabled ? 1.0f : 0.0f; break;
    case 23: cnc_config.homing_dir_mask  = CNC_Config_ClampMask3(value); applied = cnc_config.homing_dir_mask;  break;
    case 24: cnc_config.homing_feed_rate = CNC_Config_ClampFloat(value, 1.0f, 1000000.0f); applied = cnc_config.homing_feed_rate; break;
    case 25: cnc_config.homing_seek_rate = CNC_Config_ClampFloat(value, 1.0f, 1000000.0f); applied = cnc_config.homing_seek_rate; break;
    /* GRBL chuẩn: $26 = debounce */
    case 26: cnc_config.homing_debounce_ms = (uint16_t)CNC_Config_ClampU32(value, 0, 1000); applied = cnc_config.homing_debounce_ms; break;
    case 27: cnc_config.homing_pulloff = CNC_Config_ClampFloat(value, 0.0f, 1000.0f); applied = cnc_config.homing_pulloff; break;
    case 28: { uint32_t v = CNC_Config_ClampU32(value, 0, 1);
               cnc_config.homing_cycle = (uint8_t)v; applied = cnc_config.homing_cycle; break; }
    /* Custom extension (not in standard GRBL): $29 = axis mask */
    case 29:
        cnc_config.homing_axis_mask = CNC_Config_ClampMask3(value);
#ifdef CNC_LIMIT_Z_NO_ENDSTOP
        if (cnc_config.homing_axis_mask & 0x04U) {
            cnc_config.homing_axis_mask &= (uint8_t)~0x04U;
            CNC_UART_PutString("[WARN: Z removed from $29 - no endstop]\r\n");
        }
#endif
        applied = cnc_config.homing_axis_mask; break;
#endif
    /* G3: $32 in GRBL = laser mode. We don't have laser mode; reject or ignore. */
    case 32:
        /* Silently accept-and-ignore to keep UGS happy. */
        applied = 0.0f;
        break;
    /* Custom: $40 = auto report interval (was $32 in older versions) */
    case 40: cnc_config.auto_report_interval = CNC_Config_ClampU32(value, 0, CNC_MAX_REPORT_INTERVAL_MS);
             auto_report_counter = 0; applied = (float)cnc_config.auto_report_interval; break;
    case 100: case 101: case 102:
        cnc_config.steps_per_mm[idx-100] = CNC_Config_ClampFloat(value, MIN_STEPS_PER_MM, MAX_STEPS_PER_MM);
        applied = cnc_config.steps_per_mm[idx-100]; break;
    case 110: case 111: case 112:
        cnc_config.max_feedrate[idx-110] = CNC_Config_ClampFloat(value, 1.0f, 1000000.0f);
        applied = cnc_config.max_feedrate[idx-110]; break;
#if CNC_USE_ACCELERATION
    case 120: case 121: case 122:
        cnc_config.acceleration[idx-120] = CNC_Config_ClampFloat(value, 0.1f, 1000000.0f);
        applied = cnc_config.acceleration[idx-120]; break;
#endif
    case 130: case 131: case 132:
        cnc_config.max_travel[idx-130] = CNC_Config_ClampFloat(value, 0.0f, 1000000.0f);
        applied = cnc_config.max_travel[idx-130]; break;
#if CNC_USE_BACKLASH
    case 140: case 141: case 142:
        cnc_config.backlash[idx-140] = CNC_Config_ClampFloat(fabsf(value), 0.0f, 1000.0f);
        CNC_Backlash_SetValue(idx-140, cnc_config.backlash[idx-140]);
        applied = cnc_config.backlash[idx-140]; break;
    case 143: cnc_config.backlash_enabled = CNC_Config_ClampBool(value);
              CNC_Backlash_Enable(cnc_config.backlash_enabled);
              applied = cnc_config.backlash_enabled ? 1.0f : 0.0f; break;
#endif
    default: CNC_UART_PutString("[UNKNOWN PARAM]\r\n"); return;
    }
    CNC_UART_PutString("[SET $"); CNC_UART_PutInt(idx); CNC_UART_PutChar('=');
    CNC_UART_PutFloat(applied, 3); CNC_UART_PutString("]\r\n");
}

void CNC_Config_SetStepsPerMM(uint8_t axis, float value)  { if (axis<CNC_NUM_AXES) cnc_config.steps_per_mm[axis] = CNC_Config_ClampFloat(value, MIN_STEPS_PER_MM, MAX_STEPS_PER_MM); }
void CNC_Config_SetMaxFeedrate(uint8_t axis, float value) { if (axis<CNC_NUM_AXES) cnc_config.max_feedrate[axis] = CNC_Config_ClampFloat(value, 1.0f, 1000000.0f); }
void CNC_Config_SetMaxTravel(uint8_t axis, float value)   { if (axis<CNC_NUM_AXES) cnc_config.max_travel[axis]   = CNC_Config_ClampFloat(value, 0.0f, 1000000.0f); }
void CNC_Config_SetAcceleration(uint8_t axis, float value){
#if CNC_USE_ACCELERATION
    if (axis<CNC_NUM_AXES) cnc_config.acceleration[axis] = CNC_Config_ClampFloat(value, 0.1f, 1000000.0f);
#else
    (void)axis; (void)value;
#endif
}
void CNC_Config_SetSoftLimit(bool enable) { cnc_config.soft_limit_enabled = enable; }
void CNC_Config_SetAutoReport(uint32_t interval_ms) {
    if (interval_ms > CNC_MAX_REPORT_INTERVAL_MS) interval_ms = CNC_MAX_REPORT_INTERVAL_MS;
    cnc_config.auto_report_interval = interval_ms;
    auto_report_counter = 0;
}

/*============================================================================
 *                    G-CODE PARSER
 *============================================================================*/

static bool CNC_FindValue(const char* line, char letter, float* value) {
    if (line == NULL || value == NULL) return false;
    const char* p = line;
    char upper = (letter >= 'a' && letter <= 'z') ? letter - 32 : letter;
    char lower = (letter >= 'A' && letter <= 'Z') ? letter + 32 : letter;
    while (*p) {
        if (*p == upper || *p == lower) {
            bool at_start    = (p == line);
            bool after_space = (p > line) && (*(p-1) == ' ' || *(p-1) == '\t');
            if (at_start || after_space) {
                const char* ns = p + 1;
                while (*ns == ' ' || *ns == '\t') ns++;
                char fc = *ns;
                if (fc == '-' || fc == '+' || fc == '.' || (fc >= '0' && fc <= '9')) {
                    char* end_ptr;
                    float v = strtof(ns, &end_ptr);
                    if (end_ptr != ns) { *value = v; return true; }
                }
            }
        }
        p++;
    }
    return false;
}
static bool CNC_HasCommand(const char* line, char letter, int code) {
    float v;
    return CNC_FindValue(line, letter, &v) && ((int)v == code);
}

uint8_t CNC_Execute(const char* line) {
    float value;
    if (line[0] == '\0' || line[0] == ';' || line[0] == '(') return 0;

    if (CNC_HasCommand(line, 'G', 0)) rapid_mode = true;
    if (CNC_HasCommand(line, 'G', 1)) rapid_mode = false;
    if (CNC_HasCommand(line, 'G', 20)) cnc_machine.unit = CNC_UNIT_INCH;
    if (CNC_HasCommand(line, 'G', 21)) cnc_machine.unit = CNC_UNIT_MM;

    if (CNC_HasCommand(line, 'G', 28)) {
#if CNC_USE_HOMING
        CNC_HomeAll();
#else
        for (int i = 0; i < CNC_NUM_AXES; i++) cnc_machine.position[i] = 0;
        CNC_UART_PutString("[POSITION RESET]\r\n");
#endif
        return 0;
    }
    if (CNC_HasCommand(line, 'G', 90)) cnc_machine.coord_mode = CNC_COORD_ABSOLUTE;
    if (CNC_HasCommand(line, 'G', 91)) cnc_machine.coord_mode = CNC_COORD_INCREMENTAL;
    if (CNC_HasCommand(line, 'G', 92)) {
        /* D3 fix: G92 must drain planner so queued blocks (which used
           old position) finish before the coordinate system shifts. */
#if CNC_USE_PLANNER
        CNC_WaitComplete();
        while (!CNC_Planner_IsBufferEmpty()) {
            CNC_Process();           /* keep feeding/draining planner */
            if (cnc_machine.state == CNC_STATE_ALARM ||
                cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) break;
        }
        CNC_WaitComplete();
#endif
        float pos[CNC_NUM_AXES]; CNC_GetPosition(pos);
        if (CNC_FindValue(line, 'X', &value)) pos[0] = value;
        if (CNC_FindValue(line, 'Y', &value)) pos[1] = value;
        if (CNC_FindValue(line, 'Z', &value)) pos[2] = value;
        CNC_SetPosition(pos);
        return 0;
    }

    if (CNC_HasCommand(line, 'M', 17)) CNC_EnableMotors(true);
    if (CNC_HasCommand(line, 'M', 18) || CNC_HasCommand(line, 'M', 84)) CNC_EnableMotors(false);
#if CNC_USE_RELAY
    if (CNC_HasCommand(line, 'M', 33)) {
        float pulse_ms_f = 0.0f;
        /* D0 is accepted and clamped to the shortest supported pulse (1 ms). */
        if (!CNC_FindValue(line, 'D', &pulse_ms_f) || pulse_ms_f < 0.0f) return 2;
        uint32_t pulse_ms = CNC_Config_ClampU32(pulse_ms_f, 1U, 60000U);
        if (!CNC_LaserPulse_Start(pulse_ms)) return 9;
        return 0;
    }
    if (CNC_HasCommand(line, 'M', 3)) CNC_Spindle_On();
    if (CNC_HasCommand(line, 'M', 5)) {
        /* If M5 is sent during M33, make it an explicit pulse abort so the
           host receives [PULSE ABORTED] instead of silently losing state. */
        if (CNC_LaserPulse_IsActive()) CNC_LaserPulse_Abort();
        else CNC_Spindle_Off();
    }
    if (CNC_HasCommand(line, 'M', 7)) CNC_Coolant_Mist_On();
    if (CNC_HasCommand(line, 'M', 8)) CNC_Coolant_Flood_On();
    if (CNC_HasCommand(line, 'M', 9)) CNC_Coolant_Off();
#endif
    if (CNC_HasCommand(line, 'M', 114)) { CNC_SendStatus(); return 0; }
    if (CNC_HasCommand(line, 'M', 0) || CNC_HasCommand(line, 'M', 2) || CNC_HasCommand(line, 'M', 30)) {
        CNC_Stop();
#if CNC_USE_RELAY
        CNC_Spindle_Off(); CNC_Coolant_Off();
#endif
        return 0;
    }

    float target[CNC_NUM_AXES];
#if CNC_USE_PLANNER
    CNC_Planner_GetPlanningPosition(target);
#else
    CNC_GetPosition(target);
#endif
    bool has_motion = false;
    float feedrate = current_feedrate;

    if (CNC_FindValue(line, 'X', &value)) {
        if (cnc_machine.unit == CNC_UNIT_INCH) value *= CNC_INCH_TO_MM;
        target[0] = (cnc_machine.coord_mode == CNC_COORD_ABSOLUTE) ? value : target[0] + value;
        has_motion = true;
    }
    if (CNC_FindValue(line, 'Y', &value)) {
        if (cnc_machine.unit == CNC_UNIT_INCH) value *= CNC_INCH_TO_MM;
        target[1] = (cnc_machine.coord_mode == CNC_COORD_ABSOLUTE) ? value : target[1] + value;
        has_motion = true;
    }
    if (CNC_FindValue(line, 'Z', &value)) {
        if (cnc_machine.unit == CNC_UNIT_INCH) value *= CNC_INCH_TO_MM;
        target[2] = (cnc_machine.coord_mode == CNC_COORD_ABSOLUTE) ? value : target[2] + value;
        has_motion = true;
    }
    if (CNC_FindValue(line, 'F', &value)) {
        if (cnc_machine.unit == CNC_UNIT_INCH) value *= CNC_INCH_TO_MM;
        feedrate = value;
        current_feedrate = feedrate;
    }

    if (has_motion) {
        if (cnc_config.soft_limit_enabled) {
            for (int i = 0; i < CNC_NUM_AXES; i++) {
                float lo = cnc_config.allow_negative ? -cnc_config.max_travel[i] : 0.0f;
                if (target[i] < lo || target[i] > cnc_config.max_travel[i]) return 6;
            }
        }
#if CNC_USE_PLANNER
        if (!CNC_Planner_AddBlock(target, feedrate, rapid_mode)) return 9;
#else
        CNC_Move(target, feedrate, rapid_mode);
#endif
    }
    return 0;
}

/*============================================================================
 *                    COMMAND PROCESSING (text commands)
 *============================================================================*/

void CNC_ProcessCommand(const char* cmd) {
    if (strncmp(cmd, "GOWDELAYRECT", 12) == 0) {
        const char* p = cmd + 12; while (*p == ' ') p++;
        float sx=0, sy=0, sd=0, fr=0; uint32_t wm=0;
        bool hx=false, hy=false, hd=false, hw=false; float v;
        if (CNC_FindValue(p,'X',&v)){sx=v;hx=true;}
        if (CNC_FindValue(p,'Y',&v)){sy=v;hy=true;}
        if (CNC_FindValue(p,'D',&v)){sd=v;hd=true;}
        if (CNC_FindValue(p,'W',&v)){wm=(uint32_t)v;hw=true;}
        if (CNC_FindValue(p,'F',&v)){fr=v;}
        if (!hx||!hy){ CNC_UART_PutString("[ERROR: X,Y required]\r\n"); return; }
        if (!hd){     CNC_UART_PutString("[ERROR: D required]\r\n");   return; }
        if (!hw){     CNC_UART_PutString("[ERROR: W required]\r\n");   return; }
        if (sd<=0){   CNC_UART_PutString("[ERROR: D>0]\r\n");          return; }
        if (cnc_machine.unit == CNC_UNIT_INCH) { sx*=CNC_INCH_TO_MM; sy*=CNC_INCH_TO_MM; sd*=CNC_INCH_TO_MM; if (fr>0) fr*=CNC_INCH_TO_MM; }
        CNC_GoWDelayRect_Start(sx,sy,sd,wm,fr); return;
    }
    if (strncmp(cmd, "GOWDELAY", 8) == 0) {
        const char* p = cmd + 8; while (*p == ' ') p++;
        float target[CNC_NUM_AXES]; CNC_GetPosition(target);
        float sd=0, fr=0; uint32_t wm=0;
        bool hm=false, hd=false, hw=false; float v;
        if (CNC_FindValue(p,'X',&v)){ if (cnc_machine.unit == CNC_UNIT_INCH) v*=CNC_INCH_TO_MM; target[0]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[0]+v; hm=true; }
        if (CNC_FindValue(p,'Y',&v)){ if (cnc_machine.unit == CNC_UNIT_INCH) v*=CNC_INCH_TO_MM; target[1]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[1]+v; hm=true; }
        if (CNC_FindValue(p,'Z',&v)){ if (cnc_machine.unit == CNC_UNIT_INCH) v*=CNC_INCH_TO_MM; target[2]=(cnc_machine.coord_mode==CNC_COORD_ABSOLUTE)?v:target[2]+v; hm=true; }
        if (CNC_FindValue(p,'D',&v)){ if (cnc_machine.unit == CNC_UNIT_INCH) v*=CNC_INCH_TO_MM; sd=v; hd=true; }
        if (CNC_FindValue(p,'W',&v)){ wm=(uint32_t)v; hw=true; }
        if (CNC_FindValue(p,'F',&v)){ if (cnc_machine.unit == CNC_UNIT_INCH) v*=CNC_INCH_TO_MM; fr=v; }
        if (!hm){ CNC_UART_PutString("[ERROR: No axis]\r\n"); return; }
        if (!hd){ CNC_UART_PutString("[ERROR: D required]\r\n"); return; }
        if (!hw){ CNC_UART_PutString("[ERROR: W required]\r\n"); return; }
        if (sd<=0){ CNC_UART_PutString("[ERROR: D>0]\r\n"); return; }
        CNC_GoWDelay_Start(target, sd, wm, fr); return;
    }
    if (strncmp(cmd, "SET ", 4) == 0) {
        char ax = cmd[4]; float v = (float)atof(&cmd[5]);
        switch (ax) {
        case 'X': CNC_SetAxisPosition(0,v); break;
        case 'Y': CNC_SetAxisPosition(1,v); break;
        case 'Z': CNC_SetAxisPosition(2,v); break;
        default: CNC_UART_PutString("[INVALID]\r\n"); return;
        }
        CNC_UART_PutString("[SET ");CNC_UART_PutChar(ax);CNC_UART_PutChar('=');
        CNC_UART_PutFloat(v,2);CNC_UART_PutString("]\r\n"); return;
    }
    if (cmd[0]=='$' && strchr(cmd,'=')!=NULL) {
        char* eq = strchr(cmd,'='); float v = (float)atof(eq+1);
        char p[10]; int len = eq - cmd; if (len>9) len=9;
        strncpy(p, cmd, len); p[len]='\0';
        CNC_Config_Set(p, v); return;
    }
    if (strcmp(cmd,"$$")==0 || strcmp(cmd,"CONFIG")==0) { CNC_Config_Show(); return; }
    if (strcmp(cmd,"MCURESET")==0 || strcmp(cmd,"REBOOT")==0) { CNC_MCU_SystemReset(); return; }

    if (strcmp(cmd,"STOP")==0 || strcmp(cmd,"!")==0) {
        if (cnc_machine.state == CNC_STATE_GOWDELAY)      CNC_GoWDelay_Stop();
        else if (cnc_machine.state == CNC_STATE_GOWDELAY_RECT) CNC_GoWDelayRect_Stop();
        else { CNC_Stop(); CNC_UART_PutString("[STOPPED]\r\n"); }
        return;
    }
    if (strcmp(cmd,"RESET")==0) {
        CNC_UART_GetLine_Reset();   /* U2.2 */
        CNC_Stop(); CNC_EnableMotors(false);
#if CNC_USE_RELAY
        CNC_Spindle_Off(); CNC_Coolant_Off();
#endif
        CNC_LoadDefaultConfig();
        for (int i=0;i<CNC_NUM_AXES;i++) {
            cnc_machine.position[i] = 0;
#if CNC_USE_HOMING
            cnc_machine.homed[i] = false;
#endif
        }
        cnc_machine.state = CNC_STATE_IDLE;
        cnc_machine.gowdelay.phase = GOWDELAY_IDLE;
        cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
        current_feedrate = DEFAULT_FEEDRATE; rapid_mode = false;
#if CNC_USE_BACKLASH
        CNC_Backlash_Init();
#endif
#if CNC_USE_COMMAND_QUEUE
        CNC_Queue_Clear();
#endif
#if CNC_USE_PLANNER
        CNC_Planner_Clear();
#endif
        CNC_UART_PutString("[RESET OK]\r\n"); return;
    }
    if (strcmp(cmd,"STATUS")==0 || strcmp(cmd,"?")==0) { CNC_SendStatus(); return; }
    if (strcmp(cmd,"PAUSE")==0) {
        if (cnc_machine.state == CNC_STATE_RUN) {
            CNC_TIMER->CR1 &= ~TIM_CR1_CEN;
#if CNC_USE_ACCELERATION
            cnc_machine.accel.needs_recalc = 0;  /* A6 fix */
#endif
            cnc_machine.state = CNC_STATE_HOLD;
            CNC_UART_PutString("[PAUSED]\r\n");
        } return;
    }
    if (strcmp(cmd,"RESUME")==0) {
        if (cnc_machine.state == CNC_STATE_HOLD) {
            CNC_TIMER->CR1 |= TIM_CR1_CEN;
            cnc_machine.state = CNC_STATE_RUN;
            CNC_UART_PutString("[RESUMED]\r\n");
        } return;
    }
#if CNC_USE_HOMING
    if (strcmp(cmd,"HOME")==0)  { CNC_HomeAll(); return; }
    if (strcmp(cmd,"HOMEX")==0) { CNC_HomeAxis(0); return; }
    if (strcmp(cmd,"HOMEY")==0) { CNC_HomeAxis(1); return; }
    if (strcmp(cmd,"HOMEZ")==0) { CNC_HomeAxis(2); return; }
#endif
    if (strcmp(cmd,"ENABLE")==0)  { CNC_EnableMotors(true);  CNC_UART_PutString("[MOTORS ON]\r\n");  return; }
    if (strcmp(cmd,"DISABLE")==0) { CNC_EnableMotors(false); CNC_UART_PutString("[MOTORS OFF]\r\n"); return; }
    if (strcmp(cmd,"LIMITON")==0) { cnc_config.soft_limit_enabled = true;  CNC_UART_PutString("[SOFT LIMIT ON]\r\n");  return; }
    if (strcmp(cmd,"LIMITOFF")==0){ cnc_config.soft_limit_enabled = false; CNC_UART_PutString("[SOFT LIMIT OFF]\r\n"); return; }
    /* CLEAR (custom) and $X (GRBL standard) both unlock alarm */
    if (strcmp(cmd,"CLEAR")==0 || strcmp(cmd,"$X")==0) {
#if CNC_USE_LIMIT_SWITCH
        CNC_Limit_ClearAlarm();
#else
        cnc_machine.state = CNC_STATE_IDLE;
        cnc_unlock_active = false;
        CNC_UART_PutString("[ALARM CLEARED]\r\n");
#endif
        return;
    }
#if CNC_USE_RELAY
    if (strcmp(cmd,"SPINON")==0){ CNC_Spindle_On(); CNC_UART_PutString("[SPINDLE ON]\r\n"); return; }
    if (strcmp(cmd,"SPINOFF")==0){CNC_Spindle_Off();CNC_UART_PutString("[SPINDLE OFF]\r\n");return; }
    if (strcmp(cmd,"MISTON")==0){ CNC_Coolant_Mist_On();  return; }
    if (strcmp(cmd,"MISTOFF")==0){CNC_Coolant_Mist_Off(); return; }
    if (strcmp(cmd,"FLOODON")==0){CNC_Coolant_Flood_On(); return; }
    if (strcmp(cmd,"FLOODOFF")==0){CNC_Coolant_Flood_Off();return;}
    if (strcmp(cmd,"COOLOFF")==0){CNC_Coolant_Off();      return; }
#endif
#if CNC_USE_COMMAND_QUEUE
    if (strcmp(cmd,"QUEUE")==0)      { CNC_UART_PutString("[Q="); CNC_UART_PutInt(CNC_Queue_Count()); CNC_UART_PutString("]\r\n"); return; }
    if (strcmp(cmd,"CLEARQUEUE")==0) { CNC_Queue_Clear(); CNC_UART_PutString("[Q CLEARED]\r\n"); return; }
#endif
#if CNC_USE_PLANNER
    if (strcmp(cmd,"PLANNER")==0)      { CNC_UART_PutString("[P="); CNC_UART_PutInt(CNC_Planner_BlocksCount()); CNC_UART_PutString("]\r\n"); return; }
    if (strcmp(cmd,"CLEARPLANNER")==0) { CNC_Planner_Clear(); CNC_UART_PutString("[P CLEARED]\r\n"); return; }
#endif
    if (strncmp(cmd,"REPORT ",7)==0) {
        uint32_t v = atoi(&cmd[7]); CNC_Config_SetAutoReport(v);
        CNC_UART_PutString("[REPORT="); CNC_UART_PutInt(v); CNC_UART_PutString("ms]\r\n"); return;
    }
    if (strcmp(cmd,"REPORT")==0) { CNC_Config_SetAutoReport(0); CNC_UART_PutString("[REPORT OFF]\r\n"); return; }
#if CNC_USE_LIMIT_SWITCH
    if (strcmp(cmd,"LIMITS")==0) {
        /* Debug-rich output: show raw pin state + config + triggered logic */
        uint32_t idr = LIMIT_PORT->IDR;
        uint8_t raw_x = (idr >> LIMIT_X_PIN) & 1U;
        uint8_t raw_y = (idr >> LIMIT_Y_PIN) & 1U;
        uint8_t raw_z = (idr >> LIMIT_Z_PIN) & 1U;
        CNC_UART_PutString("[LIMITS triggered X=");
        CNC_UART_PutInt(CNC_Limit_IsTriggered(0));
        CNC_UART_PutString(" Y="); CNC_UART_PutInt(CNC_Limit_IsTriggered(1));
        CNC_UART_PutString(" Z="); CNC_UART_PutInt(CNC_Limit_IsTriggered(2));
        CNC_UART_PutString(" | raw pin X="); CNC_UART_PutInt(raw_x);
        CNC_UART_PutString(" Y="); CNC_UART_PutInt(raw_y);
        CNC_UART_PutString(" Z="); CNC_UART_PutInt(raw_z);
        CNC_UART_PutString(" | active_high=");
#if CNC_LIMIT_ACTIVE_HIGH
        CNC_UART_PutInt(1);
#else
        CNC_UART_PutInt(0);
#endif
        CNC_UART_PutString(" pull_up=");
#if CNC_LIMIT_PULL_UP
        CNC_UART_PutInt(1);
#else
        CNC_UART_PutInt(0);
#endif
        CNC_UART_PutString(" shared=");
#if CNC_LIMIT_SHARED_PIN
        CNC_UART_PutInt(1);
        CNC_UART_PutString(" shared_pin=PA");
        CNC_UART_PutInt(LIMIT_SHARED_PIN);
#else
        CNC_UART_PutInt(0);
#endif
        CNC_UART_PutString(" invert_mask=");
        CNC_UART_PutInt(cnc_config.limit_invert_mask);
        CNC_UART_PutString(" hard_enabled=");
        CNC_UART_PutInt(cnc_config.hard_limit_enabled);
        CNC_UART_PutString("]\r\n");
        return;
    }
#endif
    if (strcmp(cmd,"HELP")==0) {
        CNC_UART_PutString("Commands: HOME HOMEX HOMEY HOMEZ STOP RESET MCURESET REBOOT STATUS PAUSE RESUME\r\n");
        CNC_UART_PutString(" ENABLE DISABLE LIMITON LIMITOFF CLEAR LIMITS CONFIG ($$) HELP\r\n");
        CNC_UART_PutString(" M33 D.. (timed laser pulse in ms)\r\n");
        CNC_UART_PutString(" GOWDELAY X.. D.. W.. [F..]\r\n");
        CNC_UART_PutString(" GOWDELAYRECT X.. Y.. D.. W.. [F..]\r\n");
        return;
    }
    CNC_UART_PutString("[UNKNOWN: "); CNC_UART_PutString(cmd); CNC_UART_PutString("]\r\n");
}

/*============================================================================
 *                    STATUS REPORT
 *============================================================================*/

static char CNC_StateChar(CNC_State_t s) {
    switch (s) {
    case CNC_STATE_IDLE:  return 'I';
    case CNC_STATE_RUN:   return 'R';
    case CNC_STATE_HOLD:  return 'H';
    case CNC_STATE_HOMING:return 'G';
    case CNC_STATE_ALARM: return 'A';
    case CNC_STATE_LIMIT_TRIGGERED: return 'L';
    case CNC_STATE_GOWDELAY:        return 'D';
    case CNC_STATE_GOWDELAY_RECT:   return 'E';
    default: return '?';
    }
}

void CNC_SendStatusCompact(void) {
    CNC_UART_PutChar('<'); CNC_UART_PutChar(CNC_StateChar(cnc_machine.state));
    CNC_UART_PutChar('|');
    CNC_UART_PutFloat(cnc_machine.position[0], 3); CNC_UART_PutChar(',');
    CNC_UART_PutFloat(cnc_machine.position[1], 3); CNC_UART_PutChar(',');
    CNC_UART_PutFloat(cnc_machine.position[2], 3);
#if CNC_USE_RELAY
    CNC_UART_PutString("|Sp:");
    CNC_UART_PutInt(cnc_machine.spindle_on ? 1 : 0);
    CNC_UART_PutString("|LP:");
    CNC_UART_PutInt(CNC_LaserPulse_IsActive() ? 1 : 0);
    CNC_UART_PutChar(',');
    CNC_UART_PutInt((int32_t)CNC_LaserPulse_RemainingMs());
#endif
#if CNC_USE_LIMIT_SWITCH
    CNC_UART_PutString("|L");
    CNC_UART_PutInt(CNC_Limit_IsTriggered(0));
    CNC_UART_PutInt(CNC_Limit_IsTriggered(1));
    CNC_UART_PutInt(CNC_Limit_IsTriggered(2));
    /* Raw pin level (HIGH=1, LOW=0) for hardware debug.
       If R-bits never change when user presses switch -> wiring problem. */
    uint32_t idr = LIMIT_PORT->IDR;
    CNC_UART_PutString("|R");
    CNC_UART_PutInt((idr >> LIMIT_X_PIN) & 1U);
    CNC_UART_PutInt((idr >> LIMIT_Y_PIN) & 1U);
    CNC_UART_PutInt((idr >> LIMIT_Z_PIN) & 1U);
#endif
    CNC_UART_PutString(">\r\n");
}

/* G5: GRBL v1.1 standard status report
   Format: <State|MPos:x,y,z|FS:feed,spindle|Bf:planner,rx|Pn:XYZ|H:111>
   UGS/Candle expect EXACTLY this format. */
void CNC_SendStatus(void) {
    CNC_UART_PutChar('<');
    /* State field - GRBL v1.1 spec uses these literal strings */
    switch (cnc_machine.state) {
    case CNC_STATE_IDLE:            CNC_UART_PutString("Idle"); break;
    case CNC_STATE_RUN:             CNC_UART_PutString("Run");  break;
    case CNC_STATE_HOLD:            CNC_UART_PutString("Hold:0"); break;  /* Hold:0=complete */
    case CNC_STATE_HOMING:          CNC_UART_PutString("Home"); break;
    case CNC_STATE_ALARM:           CNC_UART_PutString("Alarm");break;
    case CNC_STATE_LIMIT_TRIGGERED: CNC_UART_PutString("Alarm");break; /* GRBL-equivalent */
    case CNC_STATE_GOWDELAY:        CNC_UART_PutString("Run");  break; /* hide custom states */
    case CNC_STATE_GOWDELAY_RECT:   CNC_UART_PutString("Run");  break;
    default: CNC_UART_PutChar('?');
    }
    /* MPos: machine position (mandatory, GRBL v1.1) */
    CNC_UART_PutString("|MPos:");
    CNC_UART_PutFloat(cnc_machine.position[0],3); CNC_UART_PutChar(',');
    CNC_UART_PutFloat(cnc_machine.position[1],3); CNC_UART_PutChar(',');
    CNC_UART_PutFloat(cnc_machine.position[2],3);
    /* FS: feedrate, spindle_rpm (GRBL v1.1 mandatory if requested by $10) */
    CNC_UART_PutString("|FS:");
    CNC_UART_PutInt((int32_t)current_feedrate);
    CNC_UART_PutString(",0");   /* no spindle PWM, always 0 */
#if CNC_USE_RELAY
    /* Custom fields for GUI safety/state sync */
    CNC_UART_PutString("|Sp:");
    CNC_UART_PutInt(cnc_machine.spindle_on ? 1 : 0);
    CNC_UART_PutString("|LP:");
    CNC_UART_PutInt(CNC_LaserPulse_IsActive() ? 1 : 0);
    CNC_UART_PutChar(',');
    CNC_UART_PutInt((int32_t)CNC_LaserPulse_RemainingMs());
#endif
    /* Bf: planner blocks free, rx bytes free (optional) */
#if CNC_USE_PLANNER
    CNC_UART_PutString("|Bf:");
    CNC_UART_PutInt(CNC_Planner_BlocksAvailable());
    CNC_UART_PutChar(',');
#if CNC_USE_DMA_UART
    CNC_UART_PutInt(UART_DMA_BUFFER_SIZE - CNC_UART_DMA_Available());
#else
    CNC_UART_PutInt(UART_BUFFER_SIZE - cnc_uart_rx.count);
#endif
#endif
    /* Pn: triggered input pins (GRBL convention: X,Y,Z=limits, P=probe, S=safety) */
#if CNC_USE_LIMIT_SWITCH
    bool lx = CNC_Limit_IsTriggered(0);
    bool ly = CNC_Limit_IsTriggered(1);
    bool lz = CNC_Limit_IsTriggered(2);
    if (lx || ly || lz) {
        CNC_UART_PutString("|Pn:");
        if (lx) CNC_UART_PutChar('X');
        if (ly) CNC_UART_PutChar('Y');
        if (lz) CNC_UART_PutChar('Z');
    }
    /* R: RAW pin level for hardware debug.
       If R<bits> never changes when you press the switch -> wiring problem. */
    {
        uint32_t idr = LIMIT_PORT->IDR;
        CNC_UART_PutString("|R");
        CNC_UART_PutInt((idr >> LIMIT_X_PIN) & 1U);
        CNC_UART_PutInt((idr >> LIMIT_Y_PIN) & 1U);
        CNC_UART_PutInt((idr >> LIMIT_Z_PIN) & 1U);
    }
#endif
    /* H: homed status (custom extension; UGS ignores unknown fields) */
#if CNC_USE_HOMING
    CNC_UART_PutString("|H:");
    CNC_UART_PutInt(cnc_machine.homed[0]);
    CNC_UART_PutInt(cnc_machine.homed[1]);
    CNC_UART_PutInt(cnc_machine.homed[2]);
#endif
    CNC_UART_PutString(">\r\n");
}

/*============================================================================
 *                    INIT
 *============================================================================*/

void CNC_Init(void) {
    CNC_SystemClock_Config();
    CNC_LoadDefaultConfig();
    CNC_GPIO_Init();
    CNC_Timer_Init();
#if CNC_USE_ACCELERATION && CNC_USE_ACCEL_TIMER
    CNC_AccelTimer_Init();   /* D16 */
#endif
    CNC_UART_Init();
#if CNC_USE_DMA_UART
    CNC_UART_DMA_Init();
#endif
#if CNC_USE_LIMIT_SWITCH
    CNC_Limit_Init();
#endif
#if CNC_USE_RELAY
    CNC_Relay_Init();
#endif
#if CNC_USE_COMMAND_QUEUE
    CNC_Queue_Init();
#endif
#if CNC_USE_PLANNER
    CNC_Planner_Init();
#endif
#if CNC_USE_BACKLASH
    CNC_Backlash_Init();
#endif

    cnc_machine.state          = CNC_STATE_IDLE;
    cnc_machine.coord_mode     = CNC_COORD_ABSOLUTE;
    cnc_machine.unit           = CNC_UNIT_MM;
    cnc_machine.is_moving      = false;
    cnc_machine.motors_enabled = false;
    cnc_machine.feedrate       = DEFAULT_FEEDRATE;

    for (int i = 0; i < CNC_NUM_AXES; i++) {
        cnc_machine.position[i] = 0;
        cnc_machine.target[i]   = 0;
        cnc_machine.step_count[i] = 0;
        cnc_machine.direction[i]  = 0;
        cnc_machine.limit_triggered[i] = false;
        cnc_machine.logical_delta[i] = 0.0f;
        cnc_machine.logical_target[i] = 0.0f;
        cnc_machine.inv_steps_per_mm[i] = 1.0f / CNC_SafeStepsPerMM(i);
#if CNC_USE_HOMING
        cnc_machine.homed[i] = false;
#endif
    }
#if CNC_USE_HOMING
    cnc_machine.homing_phase        = HOMING_IDLE;
    cnc_machine.homing_axis         = 0;
    cnc_machine.homing_axes_pending = 0;
    cnc_machine.homing_retry_count  = 0;
    cnc_machine.homing_debounce_tick       = 0;
    cnc_machine.homing_pulloff_steps_done  = 0;
#endif

#if CNC_USE_ACCELERATION
    memset(&cnc_machine.accel, 0, sizeof(cnc_machine.accel));
    cnc_machine.accel.spm_dominant     = 1.0f;
    cnc_machine.accel.inv_spm_dominant = 1.0f;
#endif

    cnc_machine.gowdelay.phase = GOWDELAY_IDLE;
    cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
    cnc_machine.gowdelay_rect.x_direction = 1;

#if CNC_USE_RELAY
    cnc_laser_pulse_active = false;
    cnc_laser_pulse_end_tick = 0;
    cnc_laser_pulse_duration_ms = 0;
    cnc_laser_pulse_done_flag = false;
    cnc_laser_pulse_aborted_flag = false;
#endif

    bresenham_total = 0;
    step_events_remaining = 0;

    for (volatile uint32_t i = 0; i < CNC_STARTUP_DELAY_LOOPS; i++) ;

    /* G6: GRBL v1.1 welcome string - UGS/Candle detect this to confirm Grbl device */
    CNC_UART_PutString("\r\nGrbl 1.1f ['$' for help]\r\n");
}

/*============================================================================
 *                    MAIN LOOP
 *============================================================================*/

void CNC_Process(void) {
    static char line[GCODE_LINE_SIZE];

    /*===== Realtime commands - U-A7/C-A1/C-A4 =====*/
    if (cnc_rt_status_req) {
        cnc_rt_status_req = false;
        CNC_SendStatus();
    }
    if (cnc_rt_feed_hold) {
        cnc_rt_feed_hold = false;
        if (cnc_machine.state == CNC_STATE_RUN) {
            CNC_TIMER->CR1 &= ~TIM_CR1_CEN;
#if CNC_USE_ACCELERATION
            cnc_machine.accel.needs_recalc = 0;
#endif
            cnc_machine.state = CNC_STATE_HOLD;
            CNC_UART_PutString("[HOLD]\r\n");
        } else if (cnc_machine.state == CNC_STATE_GOWDELAY) {
            CNC_GoWDelay_Stop();
        } else if (cnc_machine.state == CNC_STATE_GOWDELAY_RECT) {
            CNC_GoWDelayRect_Stop();
        }
    }
    if (cnc_rt_resume) {
        cnc_rt_resume = false;
        if (cnc_machine.state == CNC_STATE_HOLD) {
            CNC_TIMER->CR1 |= TIM_CR1_CEN;
            cnc_machine.state = CNC_STATE_RUN;
            CNC_UART_PutString("[RESUMED]\r\n");
        }
    }
    if (cnc_rt_soft_reset) {
        cnc_rt_soft_reset = false;
        CNC_UART_GetLine_Reset();
        CNC_Stop();
        CNC_UART_PutString("[SOFT RESET]\r\n");
    }

#if CNC_AUTO_REPORT_ENABLE
    if (cnc_auto_report_flag) {
        cnc_auto_report_flag = false;
        if (CNC_UART->SR & USART_SR_TXE) CNC_SendStatusCompact();
    }
#endif

#if CNC_USE_ACCELERATION && !CNC_USE_ACCEL_TIMER
    /* Fallback: called from main loop if dedicated timer disabled (D16). */
    CNC_Accel_Update();
#endif

#if CNC_USE_HOMING
    CNC_Homing_Process();
#endif

    CNC_GoWDelay_Process();
    CNC_GoWDelayRect_Process();

#if CNC_USE_LIMIT_SWITCH
    /* Kiem tra sau moi lenh move trong unlock mode.
     * unlock_active cho phep duy nhat 1 lenh. Khi lenh do xong:
     *   - Switch van con tripped -> ALARM lai
     *   - Switch da nha          -> clear unlock, bao ve phuc hoi
     * Neu machine dang move thi chua xu ly (cho het buoc). */
    /* Sau khi move trong unlock-mode KET THUC moi kiem tra switch.
     * Dieu kien: unlock_active=true VA move da tung chay (move_started=true)
     * VA hien tai khong con dang move nua.
     * => Tranh truong hop CLEAR xong chua gui lenh gi ma da bao alarm lai. */
    if (cnc_unlock_active && cnc_unlock_move_started && !cnc_machine.is_moving) {
        cnc_unlock_active      = false;
        cnc_unlock_move_started = false;
        bool still_tripped = false;
        for (int i = 0; i < CNC_NUM_AXES; i++) {
            if (CNC_Limit_IsTriggered(i)) { still_tripped = true; break; }
        }
        if (still_tripped) {
            /* Move xong ma switch van con -> ALARM lai, yeu cau CLEAR tiep */
            CNC_Timer_Stop();
            CNC_EnableMotors(false);
            CNC_ClearBufferedMotion();
            for (int i = 0; i < CNC_NUM_AXES; i++) {
                if (CNC_Limit_IsTriggered(i)) cnc_machine.limit_triggered[i] = true;
            }
            cnc_machine.state = CNC_STATE_LIMIT_TRIGGERED;
            CNC_UART_PutString("[ALARM: still on endstop - CLEAR + move away]\r\n");
        } else {
            CNC_UART_PutString("[Endstop protection restored]\r\n");
        }
    }

    /* Polling backup (EXTI is primary) */
    if (cnc_config.hard_limit_enabled && cnc_machine.is_moving &&
        (cnc_machine.state == CNC_STATE_RUN ||
         cnc_machine.state == CNC_STATE_GOWDELAY ||
         cnc_machine.state == CNC_STATE_GOWDELAY_RECT)) {
        if (CNC_Limit_CheckAll()) {
            if (cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) {
                if (cnc_machine.gowdelay.phase != GOWDELAY_IDLE)
                    cnc_machine.gowdelay.phase = GOWDELAY_IDLE;
                if (cnc_machine.gowdelay_rect.phase != GWDRECT_IDLE)
                    cnc_machine.gowdelay_rect.phase = GWDRECT_IDLE;
            }
        }
    }
#endif

#if CNC_USE_RELAY
    if (cnc_laser_pulse_done_flag) {
        cnc_laser_pulse_done_flag = false;
        CNC_UART_PutString("[PULSE DONE]\r\n");
        CNC_UART_SendOK();
    }
    if (cnc_laser_pulse_aborted_flag) {
        cnc_laser_pulse_aborted_flag = false;
        CNC_UART_PutString("[PULSE ABORTED]\r\n");
    }
#endif

    if (cnc_send_ok_flag) {
        cnc_send_ok_flag = false;
        /* D1/E1/F13 fix: do NOT send OK from completion ISR when the block
           was streamed via the planner - the streaming layer already ACKed
           on enqueue. Avoids double-OK that breaks GRBL protocol senders. */
        if (!cnc_planner_executing &&
            cnc_machine.state != CNC_STATE_GOWDELAY &&
            cnc_machine.state != CNC_STATE_GOWDELAY_RECT) {
            CNC_UART_SendOK();
        }
    }

#if CNC_USE_COMMAND_QUEUE && CNC_USE_PLANNER
    CNC_ProcessQueuedStreamingCommands();
#endif

#if CNC_USE_PLANNER
    bool planner_wait_for_lookahead = false;
    if (!CNC_Planner_IsBufferEmpty()) {
#if CNC_USE_COMMAND_QUEUE
        if (!CNC_Queue_IsEmpty()) planner_wait_for_lookahead = true;
#endif
        if (CNC_Planner_BlocksCount() == 1) {
            uint32_t age = CNC_GetTick() - planner_first_enqueue_tick;
            /* Give the command reader a short chance to enqueue the next block
               before starting motion.  Without this, the main loop starts the
               first block before reading the second line, so look-ahead rarely
               works.  Bounded to 10 ms normally, or 50 ms while UART has
               pending input, to reduce STOP/command-vs-next-block races without
               stalling forever on partial UART lines. */
            if (age < 10U || (CNC_UART_Available() && age < 50U)) planner_wait_for_lookahead = true;
        }
    }
    if (!planner_wait_for_lookahead &&
        !cnc_machine.is_moving && cnc_machine.state == CNC_STATE_IDLE &&
#if CNC_USE_RELAY
        !CNC_LaserPulse_IsActive() &&
#endif
        !CNC_Planner_IsBufferEmpty()) {
        PlannerBlock_t* block = CNC_Planner_GetCurrentBlock();
        if (block) {
            float bt[CNC_NUM_AXES];
            float bd = block->distance;
            float bn = block->nominal_speed;
            float ba = block->acceleration;
            float be = sqrtf(block->entry_speed_sqr);
            float bx = sqrtf(block->exit_speed_sqr);
            for (int i = 0; i < CNC_NUM_AXES; i++) bt[i] = block->target[i];
            CNC_Planner_DiscardCurrentBlock();

            cnc_machine.state = CNC_STATE_RUN;

            float delta[CNC_NUM_AXES], comp[CNC_NUM_AXES];
            bresenham_int_t new_total = 0; int dom = 0;
            for (int i = 0; i < CNC_NUM_AXES; i++) {
                delta[i] = bt[i] - cnc_machine.position[i];
                comp[i]  = delta[i];
#if CNC_USE_BACKLASH
                int8_t nd = 0;
                if (delta[i] > CNC_DIR_THRESHOLD_MM) nd = 1; else if (delta[i] < -CNC_DIR_THRESHOLD_MM) nd = -1;
                if (cnc_config.backlash_enabled && nd != 0)
                    CNC_Backlash_Compensate(i, nd, &comp[i]);
#endif
                cnc_machine.logical_delta[i]       = delta[i];
                float spm_i = CNC_SafeStepsPerMM(i);
                bresenham_int_t asteps = (bresenham_int_t)roundf(fabsf(comp[i] * spm_i));
                if (asteps > MAX_STEPS_PER_MOVE) {
                    CNC_UART_PutString("[ERROR: Planner move too long]\r\n");
                    cnc_machine.state = CNC_STATE_IDLE;
                    return;
                }
                bresenham_steps[i]        = asteps;
                cnc_machine.step_count[i] = (int32_t)asteps;
                if (asteps > new_total) { new_total = asteps; dom = i; }

                int8_t mvdir = 0;
                if (comp[i] > CNC_DIR_THRESHOLD_MM)       { CNC_SetDirection(i,  1); mvdir =  1; }
                else if (comp[i] < -CNC_DIR_THRESHOLD_MM) { CNC_SetDirection(i, -1); mvdir = -1; }
                else                                       cnc_machine.direction[i] = 0;

                /* P1+P5: snap target to quantized physical position */
                float quantized_delta = (float)mvdir * (float)asteps / spm_i;
                cnc_machine.target[i] = cnc_machine.position[i] + quantized_delta;
            }
            if (new_total > 0) {
                for (int i = 0; i < CNC_NUM_AXES; i++) bresenham_counter[i] = new_total / 2;
                /* B10+P1: cache snapped target so completion uses physical grid */
                for (int i = 0; i < CNC_NUM_AXES; i++) {
                    cnc_machine.inv_steps_per_mm[i] = 1.0f / CNC_SafeStepsPerMM(i);
                    cnc_machine.logical_target[i]   = cnc_machine.target[i];
                }
                __disable_irq();
                bresenham_total = new_total;
                step_events_remaining = new_total;
                cnc_send_ok_flag = false;   /* F14 fix: avoid stale OK */
                __enable_irq();
                if (!cnc_machine.motors_enabled) CNC_EnableMotors(true);
                cnc_machine.dominant_axis = dom;
                cnc_machine.move_distance = bd;
#if CNC_USE_ACCELERATION
                CNC_Accel_PlanWithEntryExit(bd, bn * CNC_SEC_PER_MIN, ba, be, bx);
#else
                float spm = CNC_SafeStepsPerMM(dom);
                uint32_t freq = (uint32_t)(bn * spm);
                if (freq < CNC_MIN_STEP_FREQ_RUNTIME) freq = CNC_MIN_STEP_FREQ_RUNTIME;
    if (freq > CNC_MAX_STEP_FREQ) freq = CNC_MAX_STEP_FREQ;
                CNC_Timer_SetPeriod(CNC_US_PER_SEC / freq);
#endif
                cnc_machine.is_moving = true;
                cnc_planner_executing = true;
                if (cnc_unlock_active) cnc_unlock_move_started = true;
                CNC_Timer_Start();
            } else {
                /* Planner accepted a geometrically non-zero block, but every
                   axis rounded to 0 physical steps.  Do not leave state stuck
                   in RUN and do not fake a physical move in reported position. */
                for (int i = 0; i < CNC_NUM_AXES; i++) {
                    cnc_machine.target[i] = cnc_machine.position[i];
                    cnc_machine.logical_target[i] = cnc_machine.position[i];
                    cnc_machine.step_count[i] = 0;
                }
                cnc_machine.is_moving = false;
                cnc_machine.state = CNC_STATE_IDLE;
                cnc_planner_executing = false;
            }
        }
    }
    if (!cnc_machine.is_moving) cnc_planner_executing = false;
#endif /* CNC_USE_PLANNER */

    /*--- Command input ---*/
    if (CNC_UART_GetLine(line, sizeof(line))) {
        /* Full chip reset must be accepted from any state, including RUN/HOLD. */
        if (strcmp(line,"MCURESET")==0 || strcmp(line,"REBOOT")==0) {
            CNC_MCU_SystemReset();
            return;
        }
        if (strcmp(line,"STOP")==0 || strcmp(line,"!")==0) {
            if (cnc_machine.state == CNC_STATE_GOWDELAY)      CNC_GoWDelay_Stop();
            else if (cnc_machine.state == CNC_STATE_GOWDELAY_RECT) CNC_GoWDelayRect_Stop();
            else { CNC_Stop(); CNC_UART_PutString("[EMERGENCY STOP]\r\n"); }
            return;
        }
        if (strcmp(line,"STATUS")==0 || strcmp(line,"?")==0) { CNC_SendStatus(); return; }
        if (strcmp(line,"PAUSE")==0) {
            if (cnc_machine.state == CNC_STATE_RUN) {
                CNC_TIMER->CR1 &= ~TIM_CR1_CEN;
#if CNC_USE_ACCELERATION
                cnc_machine.accel.needs_recalc = 0;  /* A6 fix */
#endif
                cnc_machine.state = CNC_STATE_HOLD;
                CNC_UART_PutString("[PAUSED]\r\n");
            }
            return;
        }

#if CNC_USE_PLANNER
        if (cnc_machine.state == CNC_STATE_IDLE ||
            cnc_machine.state == CNC_STATE_HOLD ||
            cnc_machine.state == CNC_STATE_RUN) {
            if (CNC_HandlePlannerStreamingInput(line)) return;
        }
#endif

        if (cnc_machine.state == CNC_STATE_IDLE ||
            cnc_machine.state == CNC_STATE_HOLD ||
            cnc_machine.state == CNC_STATE_ALARM ||
            cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) {

            /* Allowed text commands when not RUN */
            if (strncmp(line,"GOWDELAYRECT",12)==0 ||
                strncmp(line,"GOWDELAY",8)==0 ||
                strncmp(line,"SET ",4)==0 ||
                strncmp(line,"REPORT ",7)==0 ||
                line[0]=='$' ||
                strcmp(line,"RESET")==0 || strcmp(line,"MCURESET")==0 ||
                strcmp(line,"REBOOT")==0 || strcmp(line,"RESUME")==0 ||
                strcmp(line,"HOME")==0  || strcmp(line,"HOMEX")==0 ||
                strcmp(line,"HOMEY")==0 || strcmp(line,"HOMEZ")==0 ||
                strcmp(line,"ENABLE")==0|| strcmp(line,"DISABLE")==0 ||
                strcmp(line,"LIMITON")==0|| strcmp(line,"LIMITOFF")==0 ||
                strcmp(line,"CLEAR")==0 || strcmp(line,"REPORT")==0 ||
                strcmp(line,"LIMITS")==0|| strcmp(line,"CONFIG")==0 ||
                strcmp(line,"HELP")==0  ||
                strcmp(line,"SPINON")==0|| strcmp(line,"SPINOFF")==0 ||
                strcmp(line,"MISTON")==0|| strcmp(line,"MISTOFF")==0 ||
                strcmp(line,"FLOODON")==0||strcmp(line,"FLOODOFF")==0 ||
                strcmp(line,"COOLOFF")==0||
                strcmp(line,"QUEUE")==0 || strcmp(line,"CLEARQUEUE")==0 ||
                strcmp(line,"PLANNER")==0||strcmp(line,"CLEARPLANNER")==0) {
                CNC_ProcessCommand(line);
            } else {
                if (cnc_machine.state == CNC_STATE_ALARM ||
                    cnc_machine.state == CNC_STATE_LIMIT_TRIGGERED) {
                    CNC_UART_PutString("[ALARM - USE CLEAR]\r\n");
                } else {
                    uint8_t err = CNC_Execute(line);
                    if (err) CNC_UART_SendError(err);
                    else if (!cnc_machine.is_moving
#if CNC_USE_RELAY
                             && !CNC_LaserPulse_IsActive()
#endif
                    ) {
#if CNC_USE_PLANNER
                        if (CNC_Planner_IsBufferEmpty()) CNC_UART_SendOK();
#else
                        CNC_UART_SendOK();
#endif
                    }
                }
            }
        } else if (cnc_machine.state == CNC_STATE_RUN) {
            CNC_UART_PutString("[BUSY - UNSAFE COMMAND]\r\n");
        } else if (cnc_machine.state == CNC_STATE_HOMING) {
            CNC_UART_PutString("[BUSY]\r\n");
        } else if (cnc_machine.state == CNC_STATE_GOWDELAY ||
                   cnc_machine.state == CNC_STATE_GOWDELAY_RECT) {
            CNC_UART_PutString("[GOWDELAY RUNNING - USE STOP]\r\n");
        }
    }
}

/* End of grbl_cnc.c */

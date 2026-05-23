/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "PCA9685.h"

#include "W25X0XGV.h"

#include "rfm95.h"

#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ============================================================
 *  Inter-chip protocol — NAV → MPU state packet.
 *  This struct MUST be byte-identical to the NAV side.
 *  See ALBERT_FlightComp/Core/Src/main.c for the canonical layout.
 * ============================================================ */
typedef struct __attribute__((packed)) {
    uint8_t  sync1;        /*  0   LINK_SYNC1          */
    uint8_t  sync2;        /*  1   LINK_SYNC2          */
    uint8_t  type;         /*  2   LINK_TYPE_STATE     */
    uint8_t  seq;          /*  3   rolling counter     */
    uint8_t  flags;        /*  4   bit0 = att_init, bit1 = mag_ref */
    uint8_t  reserved;     /*  5   keep total at 32    */
    float    phi;          /*  6   rad                 */
    float    theta;        /* 10   rad                 */
    float    psi;          /* 14   rad                 */
    float    p;            /* 18   rad/s (bias-corrected) */
    float    q;            /* 22                       */
    float    r;            /* 26                       */
    uint16_t crc;          /* 30   CRC-16/CCITT-FALSE  */
} link_state_t;
/* Compile-time size check (C89-portable; fails to compile with a
 * "negative size array" error if the struct is ever the wrong size). */
typedef char link_state_size_check[sizeof(link_state_t) == 32 ? 1 : -1];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define W25N01GV_STORAGE_PAGE_SIZ	W25X0XGV_MAX_COLUMN	// 2048 bytes per page

#define LINK_PACKET_LEN   32        /* must equal sizeof(link_state_t)        */
#define LINK_SYNC1        0xA1
#define LINK_SYNC2        0xB5
#define LINK_TYPE_STATE   0x01

#define LINK_FLAG_ATT_INIT   (1u << 0)
#define LINK_FLAG_MAG_REF    (1u << 1)

/* If no fresh state packet arrives within this many ms, the control law
 * should disengage and command neutral.  100 Hz nominal → 50 ms = miss
 * up to 5 packets before we consider the NAV silent. */
#define LINK_STATE_STALE_MS  50

#define USB_BUFLEN  160

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
PCA9685 servos;

/* =====================================================================
 *  Inter-chip UART link (MPU side)
 * =====================================================================
 *  MPU --INT1_MPU(out)--> INT1_NAV(EXTI rising) :  MPU is sending
 *  NAV --INT2_NAV(out)--> INT2_MPU(EXTI rising) :  NAV is sending
 *
 *  32-byte state packet (link_state_t).  Defined in PTD.
 * ===================================================================== */

uint8_t  link_rx_buf[LINK_PACKET_LEN];
volatile uint8_t nav_rx_ready  = 0;     /* set by HAL_UART_RxCpltCallback */
volatile uint8_t link_tx_busy  = 0;     /* gates mpu_send_packet re-entry */

/* Latest validated NAV → MPU state.  Updated by app_handle_link_rx()
 * after sync/CRC pass.  Read by app_run_control(). */
link_state_t  latest_state;
uint32_t      latest_state_ms     = 0;  /* HAL_GetTick() when populated   */
uint32_t      link_rx_pkts_ok     = 0;  /* good packets received          */
uint32_t      link_rx_pkts_drop   = 0;  /* sync or CRC failures           */
uint8_t       link_state_fresh    = 0;  /* set on every successful update */

/* USB telemetry buffer (mirror of NAV) */
uint8_t  usbTxBuf[USB_BUFLEN];
uint16_t usbTxBufLen;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* Main-loop helpers */
static void app_init_payload(void);
static void app_servo_show(void);
static void app_handle_link_rx(void);
static void app_run_control(void);
static void app_emit_telemetry(void);

/* Public TX entry — pulses INT1_MPU then starts a non-blocking UART TX */
HAL_StatusTypeDef mpu_send_packet(const uint8_t *buf, uint16_t n);

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t tx_buff[]={0,1,2,3,4,5,6,7,8,9};


/* ---------------------------------------------------------------------
 *  app_init_payload
 *  Brings up the servo driver, the on-board NAND flash, and (later)
 *  the RFM95 LoRa modem.  Kept separate from main() so the boot
 *  sequence is one line: app_init_payload().
 * ------------------------------------------------------------------ */
static void app_init_payload(void)
{
    /* PWM driver — neutral pulse on all four channels */
    PCA9685_Init(&servos, &hi2c2, 330);
    for (uint8_t ch = 0; ch < 4; ch++) PCA9685_SetMicros(&servos, ch, 0);
    HAL_Delay(1000);

    /* Arm motors (1500 µs centre + 500 µs idle on the others)         */
    PCA9685_SetMicros(&servos, 0, 1500);
    PCA9685_SetMicros(&servos, 1, 1500);
    PCA9685_SetMicros(&servos, 2, 1500);
    PCA9685_SetMicros(&servos, 3, 1500);
    HAL_Delay(1000);

    /* NAND flash — erase block holding page 0 so logging starts clean */
    W25X0XGV_begin();
    W25X0XGV_block_erase(0);

    /* Pyro 2 default state (kept from original boot sequence)         */
    HAL_GPIO_WritePin(Pyro_1_MPU_GPIO_Port, Pyro_1_MPU_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Pyro_2_MPU_GPIO_Port, Pyro_2_MPU_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Pyro_3_MPU_GPIO_Port, Pyro_3_MPU_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Pyro_4_MPU_GPIO_Port, Pyro_4_MPU_Pin, GPIO_PIN_SET);

    /* RFM95 — disabled until the radio is on the to-do list.  The
     * struct is left visible to the rest of the file via PV section
     * if needed; nothing in the main loop depends on it today. */
}


/* ---------------------------------------------------------------------
 *  Servo helpers used by the boot-time self-test "show".
 *  Smooth ramps via small µs steps + short HAL_Delay slices so the
 *  fins move at a visible, deliberate rate (not a snap).
 * ------------------------------------------------------------------ */
static void servo_ramp_one(uint8_t ch, int from_us, int to_us)
{
    const int step = (to_us > from_us) ? 10 : -10;
    int us = from_us;
    while ((step > 0 && us <= to_us) || (step < 0 && us >= to_us)) {
        PCA9685_SetMicros(&servos, ch, (uint16_t)us);
        HAL_Delay(5);                 /* 50 steps × 5 ms = 250 ms per 500 µs leg */
        us += step;
    }
    PCA9685_SetMicros(&servos, ch, (uint16_t)to_us);
}

static void servo_ramp_pair(uint8_t ch_pos, uint8_t ch_neg,
                            int from_us, int to_us)
{
    /* ch_pos sweeps from_us → to_us;  ch_neg mirrors about 1500 µs. */
    const int step = (to_us > from_us) ? 10 : -10;
    int us = from_us;
    while ((step > 0 && us <= to_us) || (step < 0 && us >= to_us)) {
        PCA9685_SetMicros(&servos, ch_pos, (uint16_t)us);
        PCA9685_SetMicros(&servos, ch_neg, (uint16_t)(3000 - us)); /* mirror */
        HAL_Delay(5);
        us += step;
    }
    PCA9685_SetMicros(&servos, ch_pos, (uint16_t)to_us);
    PCA9685_SetMicros(&servos, ch_neg, (uint16_t)(3000 - to_us));
}

static void servos_all_neutral(void)
{
    for (uint8_t ch = 0; ch < 4; ch++)
        PCA9685_SetMicros(&servos, ch, 1500);
}


/* ---------------------------------------------------------------------
 *  app_servo_show
 *  One-shot boot-time fin exercise.  Roughly 12 seconds total.
 *
 *    Act 1  (≈6 s) — each channel ramps neutral → max → min → neutral,
 *                    in turn.  Visual confirmation of wiring per fin.
 *    Act 2  (≈2 s) — pitch pair (0 ↑, 2 ↓) full sweep then back.
 *    Act 3  (≈2 s) — yaw   pair (1 ↑, 3 ↓) full sweep then back.
 *    Act 4  (≈2 s) — "wave": each fin in sequence flicks +max then back.
 *    Finale          — all four to neutral, brief hold.
 *
 *  Blocking by design.  Called once from main() before the loop starts.
 *  The NAV's INT2_MPU edges that arrive during the show will still
 *  rearm RX via the EXTI ISR, so no inbound packets are lost.
 * ------------------------------------------------------------------ */
static void app_servo_show(void)
{
    servos_all_neutral();
    HAL_Delay(500);

    /* ---- Act 1: each channel, alone ---------------------------------- */
    for (uint8_t ch = 0; ch < 4; ch++) {
        servo_ramp_one(ch, 1500, 2000);
        servo_ramp_one(ch, 2000, 1000);
        servo_ramp_one(ch, 1000, 1500);
        HAL_Delay(150);
    }

    /* ---- Act 2: pitch pair (mirrored) -------------------------------- */
    servo_ramp_pair(0, 2, 1500, 2000);
    servo_ramp_pair(0, 2, 2000, 1000);
    servo_ramp_pair(0, 2, 1000, 1500);
    HAL_Delay(250);

    /* ---- Act 3: yaw pair (mirrored) ---------------------------------- */
    servo_ramp_pair(1, 3, 1500, 2000);
    servo_ramp_pair(1, 3, 2000, 1000);
    servo_ramp_pair(1, 3, 1000, 1500);
    HAL_Delay(250);

    /* ---- Act 4: a quick "wave" round the cluster --------------------- */
    for (uint8_t ch = 0; ch < 4; ch++) {
        servo_ramp_one(ch, 1500, 1900);
        servo_ramp_one(ch, 1900, 1500);
    }

    /* ---- Finale: lock everything at neutral and hold ----------------- */
    servos_all_neutral();
    HAL_Delay(500);
}


/* ---------------------------------------------------------------------
 *  mpu_send_packet
 *  Pulses INT1_MPU and starts an interrupt-driven UART1 transmit.
 *  NAV's EXTI on INT1_NAV arms its RX before the first byte lands.
 * ------------------------------------------------------------------ */
HAL_StatusTypeDef mpu_send_packet(const uint8_t *buf, uint16_t n)
{
    if (link_tx_busy)                        return HAL_BUSY;
    if (n != LINK_PACKET_LEN || buf == NULL) return HAL_ERROR;

    link_tx_busy = 1;

    HAL_GPIO_WritePin(INT1_MPU_GPIO_Port, INT1_MPU_Pin, GPIO_PIN_SET);
    /* ~1 µs settle — ISR latency on NAV is well under one UART byte time */
    for (volatile int i = 0; i < 32; i++) __NOP();
    HAL_GPIO_WritePin(INT1_MPU_GPIO_Port, INT1_MPU_Pin, GPIO_PIN_RESET);

    /* HAL_UART_Transmit_IT takes a non-const pointer — cast away const.
     * Safe: HAL only reads from the buffer during the transfer. */
    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart1,
                                                (uint8_t *)buf, n);
    if (st != HAL_OK) link_tx_busy = 0;
    return st;
}


/* ---------------------------------------------------------------------
 *  CRC-16 / CCITT-FALSE  —  must match the NAV-side implementation.
 * ------------------------------------------------------------------ */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}


/* ---------------------------------------------------------------------
 *  app_handle_link_rx
 *  Drain a completed NAV→MPU packet, validate sync + CRC, and copy
 *  the payload into latest_state.  A garbled packet bumps the drop
 *  counter; a valid one bumps the ok counter and refreshes the timer.
 * ------------------------------------------------------------------ */
static void app_handle_link_rx(void)
{
    if (!nav_rx_ready) return;
    nav_rx_ready = 0;

    /* Re-arm RX first so we never miss the next packet while parsing. */
    if (huart1.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&huart1, link_rx_buf, LINK_PACKET_LEN);
    }

    /* Sync check — cheap reject before the CRC math. */
    if (link_rx_buf[0] != LINK_SYNC1 ||
        link_rx_buf[1] != LINK_SYNC2 ||
        link_rx_buf[2] != LINK_TYPE_STATE)
    {
        link_rx_pkts_drop++;
        return;
    }

    /* CRC over everything except the trailing two CRC bytes themselves */
    const uint16_t crc_calc = crc16_ccitt(link_rx_buf, LINK_PACKET_LEN - 2);
    const uint16_t crc_rx   = (uint16_t)link_rx_buf[LINK_PACKET_LEN - 2]
                            | ((uint16_t)link_rx_buf[LINK_PACKET_LEN - 1] << 8);
    if (crc_calc != crc_rx) {
        link_rx_pkts_drop++;
        return;
    }

    /* Looks good — overlay the struct on the buffer. */
    const link_state_t *pkt = (const link_state_t *)link_rx_buf;
    latest_state      = *pkt;
    latest_state_ms   = HAL_GetTick();
    link_state_fresh  = 1;
    link_rx_pkts_ok++;
}


/* ---------------------------------------------------------------------
 *  Control-law gains and limits (tune on the bench, NOT in flight)
 *  --------------------------------------------------------------------
 *  Target: pitch (theta) and yaw (psi) both → 0.  Phi (roll about the
 *  long axis) is intentionally not controlled here — it's either left
 *  passively damped by the fins, or handled by a separate roll loop
 *  later (delta_r on differential fin deflection).
 *
 *  Sign convention (from RocketModel.c):
 *     Cm  = ... + Cm_delta * delta_eta   → δ_η ↑ pitches up   (θ↑)
 *     Cn  = ... + Cn_delta * delta_zeta  → δ_ζ ↑ yaws right   (ψ↑)
 *  So to bring θ back to 0 when θ>0, we need δ_η<0 → gain is negative. */
#define CTRL_KP_THETA          (-3.0f)   /* δ_η  per rad of θ  [rad/rad] */
#define CTRL_KP_PSI            (-3.0f)   /* δ_ζ  per rad of ψ  [rad/rad] */

/* Mechanical fin deflection limit, radians.  ±20° ≈ ±0.35 rad. */
#define CTRL_DELTA_MAX_RAD     (0.35f)

/* Servo mapping: 1500 µs = neutral.  500 µs ↔ CTRL_DELTA_MAX_RAD. */
#define CTRL_SERVO_NEUTRAL_US  (1500)
#define CTRL_SERVO_US_PER_RAD  (500.0f / CTRL_DELTA_MAX_RAD)   /* ≈1428 */
#define CTRL_SERVO_MIN_US      (1000)
#define CTRL_SERVO_MAX_US      (2000)


static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint16_t fin_us(float delta_rad)
{
    delta_rad = clampf(delta_rad, -CTRL_DELTA_MAX_RAD, CTRL_DELTA_MAX_RAD);
    float us = (float)CTRL_SERVO_NEUTRAL_US + delta_rad * CTRL_SERVO_US_PER_RAD;
    if (us < CTRL_SERVO_MIN_US) us = CTRL_SERVO_MIN_US;
    if (us > CTRL_SERVO_MAX_US) us = CTRL_SERVO_MAX_US;
    return (uint16_t)us;
}


/* ---------------------------------------------------------------------
 *  app_run_control
 *  Pitch + yaw P-controller.  Reads theta, psi from latest_state;
 *  drives the four fins via the PCA9685.  Disengages and commands
 *  neutral if the NAV link is stale or the EKF hasn't initialised.
 *
 *  Fin assignment (assumes "+" config — adjust signs if your fins
 *  are mounted in an "X" arrangement):
 *     channel 0 →  +δ_η   (pitch fin, top)
 *     channel 2 →  −δ_η   (pitch fin, bottom)
 *     channel 1 →  +δ_ζ   (yaw   fin, right)
 *     channel 3 →  −δ_ζ   (yaw   fin, left)
 * ------------------------------------------------------------------ */
static void app_run_control(void)
{
    const uint8_t link_alive =
        link_rx_pkts_ok > 0 &&
        (HAL_GetTick() - latest_state_ms) < LINK_STATE_STALE_MS;

    const uint8_t att_ready =
        (latest_state.flags & LINK_FLAG_ATT_INIT) != 0;

    if (!link_alive || !att_ready) {
        /* Failsafe — neutral pulse on all fin channels */
        PCA9685_SetMicros(&servos, 0, CTRL_SERVO_NEUTRAL_US);
        PCA9685_SetMicros(&servos, 1, CTRL_SERVO_NEUTRAL_US);
        PCA9685_SetMicros(&servos, 2, CTRL_SERVO_NEUTRAL_US);
        PCA9685_SetMicros(&servos, 3, CTRL_SERVO_NEUTRAL_US);
        return;
    }

    /* --- error: target attitude is θ = 0, ψ = 0 ---------------------- */
    const float err_theta = 0.0f - latest_state.theta;
    const float err_psi   = 0.0f - latest_state.psi;

    /* --- P-control on each angle ------------------------------------- */
    const float delta_eta  = CTRL_KP_THETA * err_theta;   /* pitch fin command */
    const float delta_zeta = CTRL_KP_PSI   * err_psi;     /* yaw   fin command */

    /* --- Map to four-fin "+" output (mirrored opposing pair) --------- */
    PCA9685_SetMicros(&servos, 0, fin_us(+delta_eta));
    PCA9685_SetMicros(&servos, 2, fin_us(-delta_eta));
    PCA9685_SetMicros(&servos, 1, fin_us(+delta_zeta));
    PCA9685_SetMicros(&servos, 3, fin_us(-delta_zeta));

    link_state_fresh = 0;   /* consumed */
}


/* ---------------------------------------------------------------------
 *  Float-format helpers (no float printf required)
 *  --------------------------------------------------------------------
 *  This project builds without "-u _printf_float", so %f is unavailable.
 *  These helpers reconstruct a "%.2f"-style string from a value scaled
 *  ×1000 (milli-units), e.g. 12345 → "12.35" (rounded half-up away
 *  from zero, matching what printf would produce).
 *
 *  Wrap-to-180 helper folds an angle expressed in milli-degrees into
 *  the (−180000, +180000] range — same range a human expects from a
 *  compass / attitude readout.
 * ------------------------------------------------------------------ */
static long wrap_mdeg_180(long mdeg)
{
    while (mdeg >   180000L) mdeg -= 360000L;
    while (mdeg <= -180000L) mdeg += 360000L;
    return mdeg;
}

static int fmt_milli_2dp(char *out, int outsz, long milli)
{
    /* round to nearest centi-unit (×100) */
    long centi;
    if (milli >= 0) centi = (milli + 5) / 10;
    else            centi = (milli - 5) / 10;

    const long abs_c = centi < 0 ? -centi : centi;
    const long whole = abs_c / 100;
    const long frac  = abs_c % 100;
    const char *sign = (centi < 0) ? "-" : "";

    return snprintf(out, outsz, "%s%ld.%02ld", sign, whole, frac);
}


/* ---------------------------------------------------------------------
 *  app_emit_telemetry  —  10 Hz USB-CDC dump of the last NAV packet.
 *
 *  Format mirrors the NAV's "$ALB" frame: degrees / deg-per-sec with
 *  two decimal places.  Angles wrap to (−180°, +180°].
 *
 *    $MPU,phi,theta,psi,p,q,r,seq,flags,age_ms,ok,drop\r\n
 * ------------------------------------------------------------------ */
static void app_emit_telemetry(void)
{
    static uint32_t last_ms = 0;
    if (HAL_GetTick() - last_ms < 100) return;
    last_ms = HAL_GetTick();

    /* rad → milli-degrees: 180/pi * 1000 = 57295.7795 */
    const float k = 57295.7795f;

    const long phi_mdeg   = wrap_mdeg_180((long)(latest_state.phi   * k));
    const long theta_mdeg = wrap_mdeg_180((long)(latest_state.theta * k));
    const long psi_mdeg   = wrap_mdeg_180((long)(latest_state.psi   * k));
    const long p_mdps     = (long)(latest_state.p * k);   /* rates aren't wrapped */
    const long q_mdps     = (long)(latest_state.q * k);
    const long r_mdps     = (long)(latest_state.r * k);

    char s_phi[12], s_theta[12], s_psi[12], s_p[12], s_q[12], s_r[12];
    fmt_milli_2dp(s_phi,   sizeof(s_phi),   phi_mdeg);
    fmt_milli_2dp(s_theta, sizeof(s_theta), theta_mdeg);
    fmt_milli_2dp(s_psi,   sizeof(s_psi),   psi_mdeg);
    fmt_milli_2dp(s_p,     sizeof(s_p),     p_mdps);
    fmt_milli_2dp(s_q,     sizeof(s_q),     q_mdps);
    fmt_milli_2dp(s_r,     sizeof(s_r),     r_mdps);

    const uint32_t age_ms = HAL_GetTick() - latest_state_ms;

    usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
        "$MPU,%s,%s,%s,%s,%s,%s,%u,0x%02X,%lu,%lu,%lu\r\n",
        s_phi, s_theta, s_psi,
        s_p,   s_q,     s_r,
        (unsigned)latest_state.seq,
        (unsigned)latest_state.flags,
        (unsigned long)age_ms,
        (unsigned long)link_rx_pkts_ok,
        (unsigned long)link_rx_pkts_drop);
    CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI4_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  app_init_payload();        /* PCA9685 servos, NAND flash, Pyro defaults */
  app_servo_show();          /* ~12 s boot-time fin exercise               */

  /* Defensive initial RX arm — even before the very first INT2_MPU edge
   * the MPU is ready to catch a NAV-initiated packet. */
  HAL_UART_Receive_IT(&huart1, link_rx_buf, LINK_PACKET_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      app_handle_link_rx();     /* validate + decode NAV→MPU state packet */
      app_run_control();        /* fin control law (placeholder)          */
      app_emit_telemetry();     /* 10 Hz USB telemetry: $MPU frame        */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00909BEB;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00909BEB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_MPU_GPIO_Port, SPI1_CS_MPU_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RST_MPU_GPIO_Port, LORA_RST_MPU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO1_MPU_Pin|GPIO2_MPU_Pin|GPIO3_MPU_Pin|GPIO4_MPU_Pin
                          |GPIO5_MPU_Pin|GPIO6_MPU_Pin|GPIO7_MPU_Pin|GPIO8_MPU_Pin
                          |GPIO9_MPU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, INT1_MPU_Pin|Pyro_4_MPU_Pin|Pyro_3_MPU_Pin|Pyro_2_MPU_Pin
                          |Pyro_1_MPU_Pin|LED1_MPU_Pin|LED2_MPU_Pin|LED3_MPU_Pin
                          |LED4_MPU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO10_MPU_Pin|GPIO11_MPU_Pin|GPIO12_MPU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_MPU_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_MPU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_DIO0_MPU_Pin SPI2_DIO2_MPU_Pin */
  GPIO_InitStruct.Pin = SPI2_DIO0_MPU_Pin|SPI2_DIO2_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RST_MPU_Pin */
  GPIO_InitStruct.Pin = LORA_RST_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_MPU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO1_MPU_Pin GPIO2_MPU_Pin GPIO3_MPU_Pin GPIO4_MPU_Pin
                           GPIO5_MPU_Pin GPIO6_MPU_Pin GPIO7_MPU_Pin GPIO8_MPU_Pin
                           GPIO9_MPU_Pin */
  GPIO_InitStruct.Pin = GPIO1_MPU_Pin|GPIO2_MPU_Pin|GPIO3_MPU_Pin|GPIO4_MPU_Pin
                          |GPIO5_MPU_Pin|GPIO6_MPU_Pin|GPIO7_MPU_Pin|GPIO8_MPU_Pin
                          |GPIO9_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_MPU_Pin Pyro_4_MPU_Pin Pyro_3_MPU_Pin Pyro_2_MPU_Pin
                           Pyro_1_MPU_Pin LED1_MPU_Pin LED2_MPU_Pin LED3_MPU_Pin
                           LED4_MPU_Pin */
  GPIO_InitStruct.Pin = INT1_MPU_Pin|Pyro_4_MPU_Pin|Pyro_3_MPU_Pin|Pyro_2_MPU_Pin
                          |Pyro_1_MPU_Pin|LED1_MPU_Pin|LED2_MPU_Pin|LED3_MPU_Pin
                          |LED4_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : INT2_MPU_Pin */
  GPIO_InitStruct.Pin = INT2_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT2_MPU_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO10_MPU_Pin GPIO11_MPU_Pin GPIO12_MPU_Pin */
  GPIO_InitStruct.Pin = GPIO10_MPU_Pin|GPIO11_MPU_Pin|GPIO12_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DET_MPU_Pin */
  GPIO_InitStruct.Pin = SD_DET_MPU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DET_MPU_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* EXTI15_10 covers INT2_MPU (PD11) — fires when NAV pulses INT2_NAV.
   * No CubeMX-generated NVIC enable lives here, so we add it ourselves. */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ  (EXTI15_10_IRQn);

  /* USART1 IRQ — required for HAL_UART_Receive_IT / _Transmit_IT */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ  (USART1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---------------------------------------------------------------------
 *  HAL EXTI dispatcher — only handles INT2_MPU (the NAV→MPU "I'm
 *  sending" line).  Other EXTI sources on this board don't have any
 *  attached behaviour yet.
 * ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT2_MPU_Pin) {
        if (huart1.RxState == HAL_UART_STATE_READY) {
            HAL_UART_Receive_IT(&huart1, link_rx_buf, LINK_PACKET_LEN);
        }
    }
}


/* HAL UART completion callbacks — both raise main-loop flags only. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1) {
        nav_rx_ready = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1) {
        link_tx_busy = 0;
    }
}


/* ---------------------------------------------------------------------
 *  EXTI15_10_IRQHandler — PD11 (INT2_MPU) sits on EXTI11.  The
 *  CubeMX-generated stm32h7xx_it.c does not contain a handler for it,
 *  so we define a strong override here and forward to the HAL.
 * ------------------------------------------------------------------ */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT2_MPU_Pin);
}


/* ---------------------------------------------------------------------
 *  USART1_IRQHandler — required for the HAL's _IT and _DMA UART paths.
 * ------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

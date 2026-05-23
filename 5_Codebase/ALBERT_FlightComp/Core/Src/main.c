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

#include "usbd_cdc_if.h"
#include <math.h>

#include "LSM6DSLTR.h"
#include "BMM150.h"
#include "BMP390.h"
#include "BMI088.h"

#include "Kalman.h"
#include "RocketModel.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ============================================================
 *  Inter-chip protocol — NAV ↔ MPU
 *  This struct MUST be byte-identical on both boards.
 *  Layout (32 bytes, little-endian floats):
 *    [0]  sync1 = 0xA1     [16..19] phi   (rad)
 *    [1]  sync2 = 0xB5     [20..23] theta (rad)
 *    [2]  type  = 0x01     [24..27] psi   (rad)
 *    [3]  seq   (rolling)  [28..31] p     (rad/s, bias-corrected)
 *    [4]  flags            ──── waits, that doesn't fit 32 bytes.
 *  Real layout below:
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
    uint16_t crc;          /* 30   CRC-16/CCITT-FALSE over bytes [0..29] */
} link_state_t;
/* Compile-time size check (C89-portable; fails to compile with a
 * "negative size array" error if the struct is ever the wrong size). */
typedef char link_state_size_check[sizeof(link_state_t) == 32 ? 1 : -1];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_BUFLEN        128

#define LINK_PACKET_LEN   32        /* must equal sizeof(link_state_t)        */
#define LINK_SYNC1        0xA1
#define LINK_SYNC2        0xB5
#define LINK_TYPE_STATE   0x01

#define LINK_FLAG_ATT_INIT   (1u << 0)
#define LINK_FLAG_MAG_REF    (1u << 1)
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* =====================================================================
 *  Inter-chip UART link (NAV side)
 * =====================================================================
 *  Handshake:
 *    MPU --INT1_MPU(out)--> INT1_NAV(EXTI rising) :  "I'm sending"
 *    NAV --INT2_NAV(out)--> INT2_MPU(EXTI rising) :  "I'm sending"
 *
 *  On EXTI rising on INT1_NAV the NAV arms HAL_UART_Receive_IT for one
 *  fixed-size packet; HAL_UART_RxCpltCallback raises mpu_rx_ready for
 *  the main loop to consume.
 *
 *  TX direction (NAV -> MPU) carries the rocket state packet at 100 Hz
 *  via nav_send_state().
 * ===================================================================== */

uint8_t  link_rx_buf[LINK_PACKET_LEN];
uint8_t  link_tx_buf[LINK_PACKET_LEN];
volatile uint8_t mpu_rx_ready = 0;       /* set by HAL_UART_RxCpltCallback */
volatile uint8_t link_tx_busy = 0;       /* gates nav_send_packet re-entry */


BMI088 imu1;
LSM6DSLTR imu2;
BMM150 magneto;
BMP390 prs;

Kalman      ekf;            //  6-DOF strapdown EKF (primary state estimator)
RocketModel rmdl;           //  9-DOF dynamic model, runs in parallel for validation

/* Process noise (diagonal).  u,v,w [m/s per sqrt(s)]^2  ;  phi,theta,psi [rad/s per sqrt(s)]^2 */
float Q_ekf[6] = {
    1.0e-2f, 1.0e-2f, 1.0e-2f,   /* velocity: smaller Q fights drift when obs are sparse */
    1.0e-3f, 1.0e-3f, 1.0e-3f
};

/* Measurement noise (tune against static bench data) */
float R_accel[3] = { 0.5f, 0.5f, 0.5f };   /* (m/s^2)^2   */
float R_mag      = 0.1f;                    /* rad^2       */
float R_baro     = 1.0f;                    /* (m/s)^2     */
float R_gps[3]   = { 0.25f, 0.25f, 0.25f }; /* (m/s)^2     */
float R_zupt     = 0.01f;                   /* tight: believe v = 0 when static */

/* Placeholders — hook up real servo commands and a thrust-time curve later */
float cmd_delta_eta  = 0.0f;
float cmd_delta_zeta = 0.0f;
float cmd_delta_r    = 0.0f;
float cmd_thrust_N   = 0.0f;

/* Sensor bias estimates — updated in-place during static detection on the pad.
 * gyr_bias absorbs the ~0.1-0.5 deg/s BMI088 offset that otherwise walks phi.
 * acc_bias absorbs accel DC offset so Kalman_UpdateAccel sees a true +g vector. */
float gyr_bias[3] = {0.0f, 0.0f, 0.0f};
float acc_bias[3] = {0.0f, 0.0f, 0.0f};

/* Nav-frame magnetic reference, captured once on pad during static hold.
 * Fixes the "phi=0" heading to whatever orientation the rocket sits in at t0. */
float            mag_nav[3]                = {0.0f, 0.0f, 0.0f};
uint8_t          mag_reference_captured    = 0;
volatile uint8_t static_now                = 0;   /* set by gyro handler each tick */

/* Tilt-compensated phi from magnetometer.
 *   body mag  =  R_x(phi) R_y(theta) R_z(psi) * mag_nav
 * With A = -sy*mnx + cy*mny,  B = st*cy*mnx + st*sy*mny + ct*mnz :
 *   m_by = A*cp + B*sp,   m_bz = B*cp - A*sp
 * → phi = atan2(m_by*B - m_bz*A,  m_by*A + m_bz*B)
 * Returns 0 when the horizontal-field basis degenerates (|A|,|B| both small). */
static inline int phi_from_mag(const float m_b[3], const float mn[3],
                               float theta, float psi, float *phi_out)
{
    const float st = sinf(theta), ct = cosf(theta);
    const float sy = sinf(psi),   cy = cosf(psi);
    const float A = -sy*mn[0] + cy*mn[1];
    const float B =  st*cy*mn[0] + st*sy*mn[1] + ct*mn[2];
    if (A*A + B*B < 1e-6f) return 0;
    *phi_out = atan2f(m_b[1]*B - m_b[2]*A,
                      m_b[1]*A + m_b[2]*B);
    return 1;
}


/* Interrupt Flags and timers*/
volatile uint8_t imu1_acc_drdy = 0;
volatile uint8_t imu1_gyr_drdy = 0;
volatile uint8_t bmp390_drdy   = 0;

volatile uint32_t acc_t = 0,    gyr_t = 0,    bmp_t = 0,	last_gyr_t = 0;

uint8_t vehiclestate = 0;
// Test counters
//volatile uint32_t imu1_acc_irq_cnt = 0;
//volatile uint32_t imu1_gyr_irq_cnt = 0;
//volatile uint32_t bmp390_irq_cnt   = 0;



/* USB data buffer */
uint8_t usbTxBuf[USB_BUFLEN];
uint16_t usbTxBufLen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* App layer — main-loop helpers */
static void app_init_sensors(void);
static void app_arm_uart_rx(void);
static void app_handle_imu_accel(void);
static void app_handle_imu_gyro(void);
static void app_handle_baro(void);
static void app_handle_mag(void);
static void app_handle_link_rx(void);
static void app_emit_telemetry(void);
static void app_send_state(void);

/* Public TX entry (call when NAV wants to push a packet up to the MPU) */
HAL_StatusTypeDef nav_send_packet(const uint8_t *buf, uint16_t n);

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no xor-out) */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Shared loop state previously living inline in main() */
static uint8_t  ekf_attitude_initialized = 0;
static uint32_t lastPrint                = 0;

/* Latest bias-corrected gyro reading [rad/s] — updated in
 * app_handle_imu_gyro(), consumed in app_send_state(). */
static float gyr_corrected[3] = {0.0f, 0.0f, 0.0f};


/* ---------------------------------------------------------------------
 *  CRC-16 / CCITT-FALSE
 *      poly  = 0x1021
 *      init  = 0xFFFF
 *      refin = false, refout = false, xorout = 0x0000
 *  Matches Boost / pycrc / online calculators set to "CCITT-FALSE".
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
 *  app_init_sensors
 *  Powers up all on-board peripherals and seeds the filter state.
 *  Called once from main() right after the CubeMX MX_*_Init() calls.
 * ------------------------------------------------------------------ */
static void app_init_sensors(void)
{
    BMM150_Init   (&magneto, &hi2c2);
    LSM6DSLTR_Init(&imu2,    &hi2c2);
    BMI088_Init   (&imu1,    &hspi1);
    BMP390_Init   (&prs,     &hspi2);

    Kalman_Init     (&ekf, 0.5f, Q_ekf, R_accel, R_mag, R_baro, R_gps);
    RocketModel_Init(&rmdl, NULL);

    last_gyr_t = gyr_t;                 /* avoids huge first dt */
}


/* ---------------------------------------------------------------------
 *  app_arm_uart_rx
 *  Idempotently re-arms the UART1 RX interrupt for a fixed-length
 *  packet.  Used both at boot (defensive: even before the first
 *  INT1_NAV edge) and from the INT1_NAV EXTI handler.
 * ------------------------------------------------------------------ */
static void app_arm_uart_rx(void)
{
    if (huart1.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&huart1, link_rx_buf, LINK_PACKET_LEN);
    }
}


/* ---------------------------------------------------------------------
 *  Accelerometer DRDY processing — one-shot attitude seed + EKF accel
 *  update.  Velocity drift on the pad gets murdered if attitude is
 *  initialised from the first reliable g-reading instead of starting
 *  at 0° while the rocket is actually tilted.
 * ------------------------------------------------------------------ */
static void app_handle_imu_accel(void)
{
    if (!imu1_acc_drdy) return;
    imu1_acc_drdy = 0;

    BMI088_ReadAcceleration(&imu1);

    const float ax = imu1.acc_mps2[0] - acc_bias[0];
    const float ay = imu1.acc_mps2[1] - acc_bias[1];
    const float az = imu1.acc_mps2[2] - acc_bias[2];
    const float acc_c[3] = { ax, ay, az };

    if (!ekf_attitude_initialized) {
        const float an = sqrtf(ax*ax + ay*ay + az*az);
        if (an > 0.5f * 9.80665f && an < 1.5f * 9.80665f) {
            ekf.x[4] = atan2f(az, ax);
            ekf.x[3] = atan2f(ay, sqrtf(ax*ax + az*az));
            ekf.x[5] = 0.0f;
            ekf_attitude_initialized = 1;
        }
    }

    Kalman_UpdateAccel(&ekf, acc_c);    /* internally gated to |a| ~ g */
}


/* ---------------------------------------------------------------------
 *  Gyro DRDY processing — drives Kalman_Predict, the parallel rocket
 *  model, ZUPT, and the running gyro/accel bias EMA while static.
 * ------------------------------------------------------------------ */
static void app_handle_imu_gyro(void)
{
    if (!imu1_gyr_drdy) return;
    imu1_gyr_drdy = 0;

    BMI088_ReadAngularRate(&imu1);

    const float dt = (float)(gyr_t - last_gyr_t) / (float)SystemCoreClock;
    last_gyr_t = gyr_t;

    /* Publish the bias-corrected gyro so app_send_state() can grab it
     * without reaching into the EKF internals. */
    gyr_corrected[0] = imu1.gyr_rdps[0] - gyr_bias[0];
    gyr_corrected[1] = imu1.gyr_rdps[1] - gyr_bias[1];
    gyr_corrected[2] = imu1.gyr_rdps[2] - gyr_bias[2];

    const float acc_c[3] = {
        imu1.acc_mps2[0] - acc_bias[0],
        imu1.acc_mps2[1] - acc_bias[1],
        imu1.acc_mps2[2] - acc_bias[2]
    };

    Kalman_Predict (&ekf,  acc_c, gyr_corrected, dt);
    RocketModel_Step(&rmdl,
                     cmd_delta_eta, cmd_delta_zeta, cmd_delta_r,
                     cmd_thrust_N, dt);

    /* Static detection on the RAW readings */
    const float gx = imu1.gyr_rdps[0];
    const float gy = imu1.gyr_rdps[1];
    const float gz = imu1.gyr_rdps[2];
    const float axr = imu1.acc_mps2[0];
    const float ayr = imu1.acc_mps2[1];
    const float azr = imu1.acc_mps2[2];
    const float gyr2 = gx*gx + gy*gy + gz*gz;
    const float an   = sqrtf(axr*axr + ayr*ayr + azr*azr);
    static_now = (gyr2 < (0.1f*0.1f)) && (fabsf(an - 9.80665f) < 0.5f);

    if (!static_now) return;

    Kalman_UpdateZUPT(&ekf, R_zupt);

    /* Gyro bias EMA: while static, truth = 0  →  bias ≈ raw reading */
    const float alpha_g = 0.005f;
    for (int i = 0; i < 3; i++)
        gyr_bias[i] += alpha_g * (imu1.gyr_rdps[i] - gyr_bias[i]);

    /* Accel bias EMA: expected static reading = R_b<-n * [+g, 0, 0] */
    if (ekf_attitude_initialized) {
        const float phi   = ekf.x[3];
        const float theta = ekf.x[4];
        const float psi   = ekf.x[5];
        const float sp=sinf(phi),   cp=cosf(phi);
        const float st=sinf(theta), ct=cosf(theta);
        const float sy=sinf(psi),   cy=cosf(psi);
        const float a_exp_x = 9.80665f *  ct*cy;
        const float a_exp_y = 9.80665f * (sp*st*cy - cp*sy);
        const float a_exp_z = 9.80665f * (sp*sy + cp*st*cy);
        const float alpha_a = 0.002f;
        acc_bias[0] += alpha_a * ((imu1.acc_mps2[0] - a_exp_x) - acc_bias[0]);
        acc_bias[1] += alpha_a * ((imu1.acc_mps2[1] - a_exp_y) - acc_bias[1]);
        acc_bias[2] += alpha_a * ((imu1.acc_mps2[2] - a_exp_z) - acc_bias[2]);
    }
}


/* ---------------------------------------------------------------------
 *  Barometer DRDY processing — pressure read only for now; altitude
 *  derivative → Kalman_UpdateBaro is left as a TODO.
 * ------------------------------------------------------------------ */
static void app_handle_baro(void)
{
    if (!bmp390_drdy) return;
    bmp390_drdy = 0;

    BMP390_ReadPressure(&prs);
    /* TODO: pressure -> altitude -> filtered dh/dt -> Kalman_UpdateBaro */
}


/* ---------------------------------------------------------------------
 *  Magnetometer @ 50 Hz — captures a phi=0 reference once static, then
 *  feeds tilt-compensated phi into the EKF.  Phi is unobservable from
 *  the accelerometer alone, so this is what keeps the heading bounded.
 * ------------------------------------------------------------------ */
static void app_handle_mag(void)
{
    static uint32_t last_mag_ms = 0;
    if (HAL_GetTick() - last_mag_ms < 20) return;
    last_mag_ms = HAL_GetTick();

    if (BMM150_ReadMagneticField(&magneto) != HAL_OK) return;

    /* PCB axis remap: BMM150 Y axis is along rocket body-X.
     * Right-handed cyclic map: body(x,y,z) = sensor(y,z,x). */
    const float m_body[3] = {
        magneto.mag_uT[1],
        magneto.mag_uT[2],
        magneto.mag_uT[0]
    };

    if (!mag_reference_captured && ekf_attitude_initialized && static_now) {
        const float theta = ekf.x[4];
        const float psi   = ekf.x[5];
        const float st=sinf(theta), ct=cosf(theta);
        const float sy=sinf(psi),   cy=cosf(psi);
        const float mx=m_body[0], my=m_body[1], mz=m_body[2];
        mag_nav[0] = ct*cy*mx + (-sy)*my + st*cy*mz;
        mag_nav[1] = ct*sy*mx + ( cy)*my + st*sy*mz;
        mag_nav[2] = -st  *mx + 0.0f*my + ct   *mz;
        mag_reference_captured = 1;
    }

    if (mag_reference_captured) {
        float phi_meas;
        if (phi_from_mag(m_body, mag_nav, ekf.x[4], ekf.x[5], &phi_meas)) {
            Kalman_UpdateMag(&ekf, phi_meas);
        }
    }
}


/* ---------------------------------------------------------------------
 *  Inter-chip link: drain a completed MPU→NAV packet.
 *  Edit the inside of this function to interpret the protocol once
 *  the MPU side has something useful to say.
 * ------------------------------------------------------------------ */
static void app_handle_link_rx(void)
{
    if (!mpu_rx_ready) return;
    mpu_rx_ready = 0;

    /* TODO: decode link_rx_buf[0..LINK_PACKET_LEN-1].
     * Today the MPU just streams {0..9} as a heartbeat. */

    /* Re-arm so the next INT1_NAV edge isn't strictly required to receive */
    app_arm_uart_rx();
}


/* ---------------------------------------------------------------------
 *  USB telemetry @ 10 Hz.  Two NMEA-style frames:
 *    $ALB  — EKF state + raw accel
 *    $MAG  — raw body-frame magnetometer (for hard-iron diagnostics)
 * ------------------------------------------------------------------ */
static void app_emit_telemetry(void)
{
    if (HAL_GetTick() - lastPrint <= 100) return;
    lastPrint = HAL_GetTick();

    usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
        "$ALB,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        imu1.acc_mps2[0], imu1.acc_mps2[1], imu1.acc_mps2[2],
        ekf.x[0], ekf.x[1], ekf.x[2],
        ekf.x[3] * 57.2957795f,
        ekf.x[4] * 57.2957795f,
        ekf.x[5] * 57.2957795f);
    CDC_Transmit_FS(usbTxBuf, usbTxBufLen);

    usbTxBufLen = snprintf((char*)usbTxBuf, USB_BUFLEN,
        "$MAG,%.2f,%.2f,%.2f\r\n",
        magneto.mag_uT[0], magneto.mag_uT[1], magneto.mag_uT[2]);
    CDC_Transmit_FS(usbTxBuf, usbTxBufLen);
}


/* ---------------------------------------------------------------------
 *  app_send_state — 100 Hz NAV → MPU state packet.
 *  Fills a link_state_t from the current EKF state + last bias-corrected
 *  gyro reading, computes the CRC, and hands it to nav_send_packet().
 *  Skips this tick if the previous TX hasn't drained — the next tick
 *  will pick up the freshest state, which is what a control loop wants.
 * ------------------------------------------------------------------ */
static void app_send_state(void)
{
    static uint32_t last_ms = 0;
    static uint8_t  seq     = 0;
    if (HAL_GetTick() - last_ms < 10) return;       /* ~100 Hz */
    last_ms = HAL_GetTick();

    link_state_t pkt;
    pkt.sync1    = LINK_SYNC1;
    pkt.sync2    = LINK_SYNC2;
    pkt.type     = LINK_TYPE_STATE;
    pkt.seq      = seq++;
    pkt.flags    = (uint8_t)
                 ( (ekf_attitude_initialized ? LINK_FLAG_ATT_INIT : 0u)
                 | (mag_reference_captured   ? LINK_FLAG_MAG_REF  : 0u) );
    pkt.reserved = 0;
    pkt.phi      = ekf.x[3];
    pkt.theta    = ekf.x[4];
    pkt.psi      = ekf.x[5];
    pkt.p        = gyr_corrected[0];
    pkt.q        = gyr_corrected[1];
    pkt.r        = gyr_corrected[2];
    pkt.crc      = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt) - 2);

    nav_send_packet((const uint8_t *)&pkt, sizeof(pkt));
}


/* ---------------------------------------------------------------------
 *  nav_send_packet — pulse INT2_NAV and start an interrupt-driven TX.
 *  The MPU's INT2_MPU EXTI rising edge arms its UART RX so the packet
 *  lands in its rx buffer.
 *
 *  Returns whatever HAL_UART_Transmit_IT returns; HAL_BUSY if a TX is
 *  already in flight.
 * ------------------------------------------------------------------ */
HAL_StatusTypeDef nav_send_packet(const uint8_t *buf, uint16_t n)
{
    if (link_tx_busy)                        return HAL_BUSY;
    if (n != LINK_PACKET_LEN || buf == NULL) return HAL_ERROR;

    for (uint16_t i = 0; i < n; i++) link_tx_buf[i] = buf[i];
    link_tx_busy = 1;

    /* Brief rising pulse on INT2_NAV → MPU EXTI arms its RX */
    HAL_GPIO_WritePin(INT2_NAV_GPIO_Port, INT2_NAV_Pin, GPIO_PIN_SET);
    /* ~1 µs settle; ISR latency on the other side is well under
     * one UART byte time (87 µs @ 115200) so the actual delay isn't
     * critical, but the explicit nop sequence keeps it deterministic. */
    for (volatile int i = 0; i < 32; i++) __NOP();
    HAL_GPIO_WritePin(INT2_NAV_GPIO_Port, INT2_NAV_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef st = HAL_UART_Transmit_IT(&huart1, link_tx_buf, n);
    if (st != HAL_OK) link_tx_busy = 0;
    return st;
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

  // Enable DWT cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable trace
  DWT->CYCCNT = 0;                                // reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // start counter

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  app_init_sensors();    /* sensors + EKF + parallel rocket model */
  app_arm_uart_rx();     /* listen for the first MPU packet */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      app_handle_imu_accel();   /* accel DRDY → EKF accel update + one-shot attitude seed */
      app_handle_imu_gyro();    /* gyro  DRDY → EKF predict, ZUPT, bias EMA, parallel model */
      app_handle_baro();        /* bmp390 DRDY                                              */
      app_handle_mag();         /* 50 Hz mag poll → tilt-comp phi → Kalman_UpdateMag         */
      app_handle_link_rx();     /* drain MPU→NAV UART packet if RxCplt set the flag         */
      app_send_state();         /* 100 Hz NAV→MPU state packet for the control loop         */
      app_emit_telemetry();     /* 10 Hz USB telemetry                                      */

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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, INT2_NAV_Pin|SPI1_CS_ACCEL_NAV_Pin|SPI1_CS_GYRO_NAV_Pin|SPI2_CS_NAV_Pin
                          |GPIO6_NAV_Pin|GPIO5_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO4_NAV_GPIO_Port, GPIO4_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO3_NAV_Pin|GPIO2_NAV_Pin|GPIO1_NAV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INT1_NAV_Pin SPI1_INT_ACCEL_NAV_Pin SPI1_INT_GYRO_NAV_Pin SPI2_INT_NAV_Pin */
  GPIO_InitStruct.Pin = INT1_NAV_Pin|SPI1_INT_ACCEL_NAV_Pin|SPI1_INT_GYRO_NAV_Pin|SPI2_INT_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_NAV_Pin SPI1_CS_ACCEL_NAV_Pin SPI1_CS_GYRO_NAV_Pin GPIO6_NAV_Pin
                           GPIO5_NAV_Pin */
  GPIO_InitStruct.Pin = INT2_NAV_Pin|SPI1_CS_ACCEL_NAV_Pin|SPI1_CS_GYRO_NAV_Pin|GPIO6_NAV_Pin
                          |GPIO5_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_NAV_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_NAV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPS_TP_NAV_Pin GPS_LNA_NAV_Pin */
  GPIO_InitStruct.Pin = GPS_TP_NAV_Pin|GPS_LNA_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO4_NAV_Pin */
  GPIO_InitStruct.Pin = GPIO4_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO4_NAV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO3_NAV_Pin GPIO2_NAV_Pin GPIO1_NAV_Pin */
  GPIO_InitStruct.Pin = GPIO3_NAV_Pin|GPIO2_NAV_Pin|GPIO1_NAV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* EXTI15_10 covers INT1_NAV (PC13) — fires when MPU pulses INT1_MPU.
   * Lower priority than the IMU EXTIs so a packet edge can't preempt
   * the gyro/accel timestamp capture. */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ  (EXTI15_10_IRQn);

  /* USART1 IRQ is required for HAL_UART_Receive_IT / _Transmit_IT */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ  (USART1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*Use DWT instead of halgettick because of higher resolution*/
	uint32_t t = DWT->CYCCNT;

    if (GPIO_Pin == SPI1_INT_ACCEL_NAV_Pin) {
        imu1_acc_drdy = 1;
        acc_t = t;
        //imu1_acc_irq_cnt++;
    }
    else if (GPIO_Pin == SPI1_INT_GYRO_NAV_Pin) {
        imu1_gyr_drdy = 1;
        gyr_t = t;
        //imu1_gyr_irq_cnt++;

    }
    else if (GPIO_Pin == SPI2_INT_NAV_Pin) {
        bmp390_drdy = 1;
        bmp_t = t;
        //bmp390_irq_cnt++;

    }
    else if (GPIO_Pin == INT1_NAV_Pin) {
        /* MPU about to send.  Arm RX so the bytes land in link_rx_buf.
         * Idempotent — if RX is already armed (e.g. from a previous
         * heartbeat), HAL returns HAL_BUSY and we just keep listening. */
        if (huart1.RxState == HAL_UART_STATE_READY) {
            HAL_UART_Receive_IT(&huart1, link_rx_buf, LINK_PACKET_LEN);
        }
    }
}


/* ---------------------------------------------------------------------
 *  HAL UART completion callbacks — both raise main-loop flags only.
 * ------------------------------------------------------------------ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1) {
        mpu_rx_ready = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1) {
        link_tx_busy = 0;
    }
}


/* ---------------------------------------------------------------------
 *  EXTI15_10_IRQHandler — INT1_NAV (PC13) lives on EXTI13, which the
 *  CubeMX-generated stm32f4xx_it.c does NOT contain a handler for.
 *  Defining one here as a strong symbol overrides the startup file's
 *  weak default and routes the line into HAL_GPIO_EXTI_Callback above.
 * ------------------------------------------------------------------ */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT1_NAV_Pin);
}


/* ---------------------------------------------------------------------
 *  USART1_IRQHandler — same story.  HAL_UART_*_IT only works if this
 *  handler routes the interrupt into the HAL.
 * ------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
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

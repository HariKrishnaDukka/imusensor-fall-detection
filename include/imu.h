#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

/*------------------------------------------------------------------
 * ISM330DHCX Hardware Register Addresses for Embedded Engines
 *-----------------------------------------------------------------*/
#define ISM_REG_FIFO_CTRL1          0x07U  /* FIFO Watermark threshold low bits  */
#define ISM_REG_FIFO_CTRL2          0x08U  /* FIFO Watermark threshold high bits */
#define ISM_REG_FIFO_CTRL3          0x09U  /* Gyro & Accel FIFO batch data rates */
#define ISM_REG_FIFO_CTRL4          0x0AU  /* FIFO operational mode control      */
#define ISM_REG_WHO_AM_I            0x0FU
#define ISM_REG_STATUS_REG          0x1EU
#define ISM_REG_WAKE_UP_SRC         0x1BU  /* Hardware event status register     */
#define ISM_REG_FIFO_STATUS1        0x3AU  /* Remaining unread FIFO words count  */
#define ISM_REG_FIFO_STATUS2        0x3BU  /* FIFO status flags (watermark, ovf) */
#define ISM_REG_TAP_CFG0            0x56U  /* Interrupt latch configuration      */
#define ISM_REG_TAP_CFG2            0x58U  /* Master interrupt enablement        */
#define ISM_REG_WAKE_UP_THS         0x5BU  /* Wake-up & Inactivity threshold     */
#define ISM_REG_WAKE_UP_DUR         0x5CU  /* Wake-up & Inactivity duration      */
#define ISM_REG_FREE_FALL           0x5DU  /* Free-fall threshold & duration     */
#define ISM_REG_MD1_CFG             0x5EU  /* Route hardware events to INT1       */
#define ISM_REG_FIFO_DATA_OUT_TAG   0x78U  /* Identifies type of data in FIFO     */
#define ISM_REG_FIFO_DATA_OUT_X_L   0x79U  /* FIFO data output start address      */

/*------------------------------------------------------------------
 * Hardware Event Source Status Bitmasks
 *-----------------------------------------------------------------*/
#define ISM_WAKE_UP_SRC_FF_IA       0x20U  /* Bit 5: Free-Fall event active   */
#define ISM_WAKE_UP_SRC_SLEEP_IA    0x10U  /* Bit 4: Inactivity state active  */
#define ISM_WAKE_UP_SRC_WU_IA       0x08U  /* Bit 3: Motion Wake-Up active    */

/*------------------------------------------------------------------
 * Hardware FIFO Constants & Tag Identifiers
 *-----------------------------------------------------------------*/
#define ISM_FIFO_TAG_GYRO           0x01U  /* FIFO data belongs to Gyroscope     */
#define ISM_FIFO_TAG_ACCEL          0x02U  /* FIFO data belongs to Accelerometer */

/* FIFO Operational Modes (FIFO_CTRL4)
 * 0x00 = Bypass mode (FIFO turned off)
 * 0x06 = Continuous mode (New data overwrites old data if full) */
#define ISM_FIFO_MODE_BYPASS        0x00U
#define ISM_FIFO_MODE_CONTINUOUS    0x06U

/*------------------------------------------------------------------
 * Custom Legacy Configuration Mappings (±4g Full Scale @ 416Hz)
 *-----------------------------------------------------------------*/

/* FALL CONFIGURATION (Register 0x5D)
 * Bits [7:3] = FF_DUR[4:0] (1 LSB = 1 / ODR = 2.403 ms)
 * 0x06U = 6 samples * 2.4 ms = ~14.4 ms minimal duration requirement.
 * Bits [2:0] = FF_THS[2:0] (100b sets the threshold to exactly 344 mg / ~0.35g) */
#define ISM_FF_THS_344MG            0x04U  
#define ISM_FF_DUR_VAL              (0x06U << 3) 

/* NO-FALL / INACTIVITY CONFIGURATIONS (Registers 0x5B & 0x5C)
 * WAKE_UP_THS (0x5B):
 * Bit [6]   = SLEEP_ON_INACT (1 = enable power saving/stationary tracking)
 * Bits[5:0] = WK_THS (Inactivity threshold: 1 LSB = FS / 64 = 4g / 64 = 62.5 mg)
 * 0x02U = 2 * 62.5 mg = 125 mg stillness baseline window.
 * WAKE_UP_DUR (0x5C):
 * Bits [3:0] = SLEEP_DUR (Stillness duration: 1 LSB = 512 / ODR = 1.23 seconds)
 * 0x02U = 2 * 1.23s = ~2.46 seconds of verified rest required. */
#define ISM_SLEEP_ON_INACT_ENABLE   0x40U
#define ISM_NO_FALL_THS_VAL         0x02U  
#define ISM_NO_FALL_DUR_VAL         0x02U  

/* MD1_CFG (0x5E) Physical Routing Flags
 * Bit 5 = INT1_WU (Route wake-up / motion transitions to INT1)
 * Bit 4 = INT1_FF (Route free-fall state transitions to INT1)
 * Bit 3 = INT1_FIFO_TH (Route FIFO Watermark Threshold to INT1) */
#define ISM_INT1_WU_ENABLE          0x20U
#define ISM_INT1_FF_ENABLE          0x10U
#define ISM_INT1_FIFO_TH_ENABLE     0x08U

/*------------------------------------------------------------------
 * Unified Data and Event Handling Enums / Structures
 *-----------------------------------------------------------------*/
struct imu_data
{
    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;
};

typedef enum
{
    IMU_HW_EVENT_NONE        = 0,
    IMU_HW_EVENT_FALL        = 1,  /* Built-in engine matched Free-Fall parameters */
    IMU_HW_EVENT_NO_FALL     = 2,  /* Built-in engine matched Stillness parameters */
    IMU_HW_EVENT_MOTION      = 3   /* Built-in engine matched Wake-Up parameters    */
} imu_hw_event_t;

/*------------------------------------------------------------------
 * Driver API Declarations
 *-----------------------------------------------------------------*/
int imu_init(void);
int imu_read_register(uint8_t reg, uint8_t *value);
int imu_write_register(uint8_t reg, uint8_t value);
int imu_get_status(uint8_t *status);

int imu_read_accel(float *ax, float *ay, float *az);
int imu_read_gyro(float *gx, float *gy, float *gz);
int imu_read_all(struct imu_data *data);

/* Hardware Feature Engine & Hardware FIFO Configurations */
int imu_configure_hw_features(void);
int imu_clear_fifo(void);
int imu_get_fifo_sample_count(uint16_t *count);
int imu_read_fifo_packet(struct imu_data *data, bool *accel_ready, bool *gyro_ready);

/* Event processor evaluating WAKE_UP_SRC flags and unlatching the INT pin */
int imu_process_hw_events(imu_hw_event_t *event);

int imu_int_init(void);
int imu_int_level(void);
struct k_sem *imu_get_hw_event_sem(void);

#endif /* IMU_H */
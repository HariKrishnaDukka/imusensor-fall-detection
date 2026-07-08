#include "imu.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

#define CTRL1_XL           0x10U
#define CTRL2_G            0x11U
#define CTRL3_C            0x12U

#define OUTX_L_G           0x22U
#define OUTX_L_A           0x28U

#define WHO_AM_I_VALUE     0x6BU
#define IMU_ADDR_0         0x6AU
#define IMU_ADDR_1         0x6BU

#define XL_FS_4G           (0x02U << 2)
#define G_FS_500DPS        (0x04U << 2)

#define ACC_SENSITIVITY_G     0.000122f
#define GYRO_SENSITIVITY_DPS  0.01750f

static const struct device *i2c_dev;
static uint8_t imu_addr;

K_SEM_DEFINE(hw_event_sem, 0, 1);
static struct gpio_callback imu_cb;

static const struct gpio_dt_spec imu_int =
    GPIO_DT_SPEC_GET(DT_NODELABEL(imu_int1_pin), gpios);

/*------------------------------------------------------------------
 * Low-Level Communication Blocks
 *-----------------------------------------------------------------*/
static int reg_read(uint8_t reg, uint8_t *value)
{
    return i2c_write_read(i2c_dev, imu_addr, &reg, 1, value, 1);
}

static int reg_write(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    return i2c_write(i2c_dev, tx, sizeof(tx), imu_addr);
}

static int burst_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    return i2c_write_read(i2c_dev, imu_addr, &reg, 1, data, len);
}

int imu_read_register(uint8_t reg, uint8_t *value)
{
    return reg_read(reg, value);
}

int imu_write_register(uint8_t reg, uint8_t value)
{
    return reg_write(reg, value);
}

static int detect_sensor(void)
{
    uint8_t whoami = 0;

    imu_addr = IMU_ADDR_0;
    if ((reg_read(ISM_REG_WHO_AM_I, &whoami) == 0) && (whoami == WHO_AM_I_VALUE))
    {
        LOG_INF("ISM330DHCX recognized at address 0x6A");
        return 0;
    }

    imu_addr = IMU_ADDR_1;
    if ((reg_read(ISM_REG_WHO_AM_I, &whoami) == 0) && (whoami == WHO_AM_I_VALUE))
    {
        LOG_INF("ISM330DHCX recognized at address 0x6B");
        return 0;
    }

    return -ENODEV;
}

/*------------------------------------------------------------------
 * Physical Interrupt Service Routine
 *-----------------------------------------------------------------*/
static void imu_gpio_callback(const struct device *port,
                              struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_sem_give(&hw_event_sem);
}

/*------------------------------------------------------------------
 * Base Core Initializations
 *-----------------------------------------------------------------*/
int imu_init(void)
{
    int ret;

    i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(i2c_dev))
    {
        LOG_ERR("Physical I2C Master Line Not Ready");
        return -ENODEV;
    }

    ret = detect_sensor();
    if (ret) return ret;

    /* BDU=1, IF_INC=1 for robust multi-byte reading configurations */
    ret = reg_write(CTRL3_C, 0x44);
    if (ret) return ret;

    /* Base sampling configured at 416Hz to drive internal silicon blocks */
    ret = reg_write(CTRL1_XL, ((0x06 << 4) | XL_FS_4G));
    if (ret) return ret;

    ret = reg_write(CTRL2_G, ((0x06 << 4) | G_FS_500DPS));
    if (ret) return ret;

    k_msleep(20);
    return 0;
}

/*------------------------------------------------------------------
 * Advanced Hardware Feature Engine & FIFO Configurations
 *-----------------------------------------------------------------*/
int imu_configure_hw_features(void)
{
    int ret;

    /* 1. LIR=1: Latch the interrupt line until WAKE_UP_SRC is explicitly read */
    ret = reg_write(ISM_REG_TAP_CFG0, 0x01U);
    if (ret) return ret;

    /* 2. Enable physical interrupts globally */
    ret = reg_write(ISM_REG_TAP_CFG2, 0x80U); 
    if (ret) return ret;

    /* 3. Program Built-In Fall properties (~0.35g threshold limit) */
    ret = reg_write(ISM_REG_FREE_FALL, (ISM_FF_DUR_VAL | ISM_FF_THS_344MG));
    if (ret) return ret;

    /* 4. Program Built-In Stillness Threshold (125mg baseline limit) */
    ret = reg_write(ISM_REG_WAKE_UP_THS, (ISM_SLEEP_ON_INACT_ENABLE | ISM_NO_FALL_THS_VAL));
    if (ret) return ret;

    /* 5. Program Built-In Stillness Verification Duration Requirement (~2.46 seconds) */
    ret = reg_write(ISM_REG_WAKE_UP_DUR, ISM_NO_FALL_DUR_VAL);
    if (ret) return ret;

    /* 6. Configure Hardware FIFO Batching 
     * Batch both Accel and Gyro data at the current ODR rate (416Hz) */
    ret = reg_write(ISM_REG_FIFO_CTRL3, 0x66U);
    if (ret) return ret;

    /* 7. Arm Hardware FIFO into Continuous Mode 
     * Stream continuously loops inside the silicon buffer until emptied */
    ret = reg_write(ISM_REG_FIFO_CTRL4, ISM_FIFO_MODE_CONTINUOUS);
    if (ret) return ret;

    /* 8. Route internal event evaluation engine outputs to hardware pin INT1 */
    ret = reg_write(ISM_REG_MD1_CFG, (ISM_INT1_WU_ENABLE | ISM_INT1_FF_ENABLE));
    if (ret) return ret;

    LOG_INF("Inbuilt Fall, No-Fall, and FIFO Hardware Cores Configured Successfully");
    return 0;
}

/*------------------------------------------------------------------
 * Hardware FIFO Management Helper Utilities
 *-----------------------------------------------------------------*/
int imu_clear_fifo(void)
{
    int ret;
    /* Put FIFO into bypass mode to reset pointers and wipe data */
    ret = reg_write(ISM_REG_FIFO_CTRL4, ISM_FIFO_MODE_BYPASS);
    if (ret) return ret;

    /* Re-arm FIFO into continuous mode */
    return reg_write(ISM_REG_FIFO_CTRL4, ISM_FIFO_MODE_CONTINUOUS);
}

int imu_get_fifo_sample_count(uint16_t *count)
{
    uint8_t status1 = 0;
    uint8_t status2 = 0;
    int ret;

    if (count == NULL) return -EINVAL;

    ret = reg_read(ISM_REG_FIFO_STATUS1, &status1);
    if (ret) return -EIO;

    ret = reg_read(ISM_REG_FIFO_STATUS2, &status2);
    if (ret) return -EIO;

    /* Combine upper and lower bits to parse unread words count */
    *count = (uint16_t)status1 | (((uint16_t)status2 & 0x03U) << 8);
    return 0;
}

int imu_read_fifo_packet(struct imu_data *data, bool *accel_ready, bool *gyro_ready)
{
    uint8_t raw_packet[7]; /* 1 Tag Byte + 6 Data Bytes */
    int16_t raw_x, raw_y, raw_z;
    int ret;

    if ((data == NULL) || (accel_ready == NULL) || (gyro_ready == NULL)) return -EINVAL;

    *accel_ready = false;
    *gyro_ready = false;

    /* Read the data block starting directly from the Tag identifier register */
    ret = burst_read(ISM_REG_FIFO_DATA_OUT_TAG, raw_packet, sizeof(raw_packet));
    if (ret) return -EIO;

    uint8_t tag = (raw_packet[0] >> 3) & 0x1FU; /* Extract bits [7:3] for the data tag */

    raw_x = (int16_t)sys_get_le16(&raw_packet[1]);
    raw_y = (int16_t)sys_get_le16(&raw_packet[3]);
    raw_z = (int16_t)sys_get_le16(&raw_packet[5]);

    if (tag == ISM_FIFO_TAG_ACCEL)
    {
        data->ax_g = (float)raw_x * ACC_SENSITIVITY_G;
        data->ay_g = (float)raw_y * ACC_SENSITIVITY_G;
        data->az_g = (float)raw_z * ACC_SENSITIVITY_G;
        *accel_ready = true;
    }
    else if (tag == ISM_FIFO_TAG_GYRO)
    {
        data->gx_dps = (float)raw_x * GYRO_SENSITIVITY_DPS;
        data->gy_dps = (float)raw_y * GYRO_SENSITIVITY_DPS;
        data->gz_dps = (float)raw_z * GYRO_SENSITIVITY_DPS;
        *gyro_ready = true;
    }

    return 0;
}

/*------------------------------------------------------------------
 * Hardware Interrupt Event Interpreter
 *-----------------------------------------------------------------*/
int imu_process_hw_events(imu_hw_event_t *event)
{
    uint8_t src_reg = 0;
    int ret;

    if (event == NULL) return -EINVAL;
    *event = IMU_HW_EVENT_NONE;

    /* Reading this register automatically drops the physical INT1 line back low */
    ret = reg_read(ISM_REG_WAKE_UP_SRC, &src_reg);
    if (ret) return -EIO;

    if ((src_reg & ISM_WAKE_UP_SRC_FF_IA) != 0U)
    {
        *event = IMU_HW_EVENT_FALL;
    }
    else if ((src_reg & ISM_WAKE_UP_SRC_SLEEP_IA) != 0U)
    {
        *event = IMU_HW_EVENT_NO_FALL;
    }
    else if ((src_reg & ISM_WAKE_UP_SRC_WU_IA) != 0U)
    {
        *event = IMU_HW_EVENT_MOTION;
    }

    return 0;
}

/*------------------------------------------------------------------
 * Polling Standard Fallbacks
 *-----------------------------------------------------------------*/
int imu_read_accel(float *ax, float *ay, float *az)
{
    uint8_t raw[6];
    if ((ax == NULL) || (ay == NULL) || (az == NULL)) return -EINVAL;
    if (burst_read(OUTX_L_A, raw, 6)) return -EIO;

    *ax = (float)((int16_t)sys_get_le16(&raw[0])) * ACC_SENSITIVITY_G;
    *ay = (float)((int16_t)sys_get_le16(&raw[2])) * ACC_SENSITIVITY_G;
    *az = (float)((int16_t)sys_get_le16(&raw[4])) * ACC_SENSITIVITY_G;
    return 0;
}

int imu_read_gyro(float *gx, float *gy, float *gz)
{
    uint8_t raw[6];
    if ((gx == NULL) || (gy == NULL) || (gz == NULL)) return -EINVAL;
    if (burst_read(OUTX_L_G, raw, 6)) return -EIO;

    *gx = (float)((int16_t)sys_get_le16(&raw[0])) * GYRO_SENSITIVITY_DPS;
    *gy = (float)((int16_t)sys_get_le16(&raw[2])) * GYRO_SENSITIVITY_DPS;
    *gz = (float)((int16_t)sys_get_le16(&raw[4])) * GYRO_SENSITIVITY_DPS;
    return 0;
}

int imu_read_all(struct imu_data *data)
{
    uint8_t raw[12];
    if (data == NULL) return -EINVAL;
    if (burst_read(OUTX_L_G, raw, sizeof(raw))) return -EIO;

    data->gx_dps = (float)((int16_t)sys_get_le16(&raw[0])) * GYRO_SENSITIVITY_DPS;
    data->gy_dps = (float)((int16_t)sys_get_le16(&raw[2])) * GYRO_SENSITIVITY_DPS;
    data->gz_dps = (float)((int16_t)sys_get_le16(&raw[4])) * GYRO_SENSITIVITY_DPS;

    data->ax_g = (float)((int16_t)sys_get_le16(&raw[6])) * ACC_SENSITIVITY_G;
    data->ay_g = (float)((int16_t)sys_get_le16(&raw[8])) * ACC_SENSITIVITY_G;
    data->az_g = (float)((int16_t)sys_get_le16(&raw[10])) * ACC_SENSITIVITY_G;
    return 0;
}

int imu_get_status(uint8_t *status)
{
    if (status == NULL) return -EINVAL;
    return reg_read(ISM_REG_STATUS_REG, status);
}

int imu_int_init(void)
{
    int ret;
    if (!gpio_is_ready_dt(&imu_int)) return -ENODEV;

    ret = gpio_pin_configure_dt(&imu_int, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret) return ret;

    gpio_init_callback(&imu_cb, imu_gpio_callback, BIT(imu_int.pin));
    ret = gpio_add_callback(imu_int.port, &imu_cb);
    if (ret) return ret;

    return gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_EDGE_TO_ACTIVE);
}

int imu_int_level(void)
{
    return gpio_pin_get_dt(&imu_int);
}

struct k_sem *imu_get_hw_event_sem(void)
{
    return &hw_event_sem;
}
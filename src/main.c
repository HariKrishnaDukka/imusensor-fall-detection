#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>  /* Added for onboard LED interaction */

#include "imu.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static uint8_t i2c_error_count = 0U;

/* Devicetree node fetching for the standard onboard nRF LED */
static const struct gpio_dt_spec onboard_led = 
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* Non-blocking timestamp trackers for the 5-second LED event */
static int64_t blink_end_timestamp = 0;
static int64_t next_toggle_timestamp = 0;
static bool led_is_active = false;

/* Frames to align asynchronous Accel/Gyro FIFO packets */
static struct imu_data current_frame;
static bool has_accel = false;
static bool has_gyro = false;

/*------------------------------------------------------------------
 * Continuous FIFO Flushing
 * Always active. Empties raw hardware frames as they fill up.
 *-----------------------------------------------------------------*/
static void process_fifo_stream(void)
{
    uint16_t words_in_fifo = 0;

    if (imu_get_fifo_sample_count(&words_in_fifo) != 0 || words_in_fifo == 0)
    {
        return;
    }

    struct imu_data packet;
    bool accel_ready;
    bool gyro_ready;

    for (uint16_t i = 0; i < words_in_fifo; i++)
    {
        if (imu_read_fifo_packet(&packet, &accel_ready, &gyro_ready) == 0)
        {
            if (accel_ready)
            {
                current_frame.ax_g = packet.ax_g;
                current_frame.ay_g = packet.ay_g;
                current_frame.az_g = packet.az_g;
                has_accel = true;
            }
            if (gyro_ready)
            {
                current_frame.gx_dps = packet.gx_dps;
                current_frame.gy_dps = packet.gy_dps;
                current_frame.gz_dps = packet.gz_dps;
                has_gyro = true;
            }

            if (has_accel && has_gyro)
            {
                // printk("DATA,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                //        (double)current_frame.ax_g,
                //        (double)current_frame.ay_g,
                //        (double)current_frame.az_g,
                //        (double)current_frame.gx_dps,
                //        (double)current_frame.gy_dps,
                //        (double)current_frame.gz_dps);
                
                has_accel = false;
                has_gyro = false;
            }
        }
    }
}

/*------------------------------------------------------------------
 * Flat Event Parsing 
 * Instantly outputs matching register state transitions
 *-----------------------------------------------------------------*/
static void parse_hardware_event(imu_hw_event_t event)
{
    switch (event)
    {
        case IMU_HW_EVENT_FALL:
            printk("EVENT,FALL_DETECTED\n");

            printk("DATA,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",

                        (double)current_frame.ax_g,
                       (double)current_frame.ay_g,
                        (double)current_frame.az_g,
                        (double)current_frame.gx_dps,
                        (double)current_frame.gy_dps,
                        (double)current_frame.gz_dps);
            
            /* Arm the non-blocking LED state tracking machine
             * 5000ms total window, toggling every 200ms for a rapid alert look */
            blink_end_timestamp = k_uptime_get() + 5000;
            next_toggle_timestamp = k_uptime_get();
            break;

        case IMU_HW_EVENT_NO_FALL:
            printk("EVENT,NO_FALL_STILLNESS\n");
            break;

        case IMU_HW_EVENT_MOTION:
            printk("EVENT,MOTION_ALERT\n");

            printk("DATA,0,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        (double)current_frame.ax_g,
                       (double)current_frame.ay_g,
                       (double)current_frame.az_g,
                       (double)current_frame.gx_dps,
                       (double)current_frame.gy_dps,
                       (double)current_frame.gz_dps);
            break;

        default:
            break;
    }
}

/*------------------------------------------------------------------
 * Non-Blocking LED Coordinator
 * Evaluates timing milestones continuously without halting execution
 *-----------------------------------------------------------------*/
static void update_led_state(void)
{
    int64_t current_time = k_uptime_get();

    if (current_time < blink_end_timestamp)
    {
        /* If the 5-second window is active, check if it's time to toggle the pin */
        if (current_time >= next_toggle_timestamp)
        {
            led_is_active = !led_is_active;
            gpio_pin_set_dt(&onboard_led, led_is_active ? 1 : 0);
            
            /* Schedule next flash edge in 200 milliseconds */
            next_toggle_timestamp = current_time + 200;
        }
    }
    else
    {
        /* 5-second window has elapsed, guarantee the LED turns off completely */
        if (led_is_active)
        {
            led_is_active = false;
            gpio_pin_set_dt(&onboard_led, 0);
        }
    }
}

int main(void)
{
    int ret;

    LOG_INF("Starting Flat Continuous Streaming & Event Detection Firmware");

    /* Initialize the onboard LED GPIO pin */
    if (!gpio_is_ready_dt(&onboard_led))
    {
        LOG_ERR("Onboard nRF LED hardware node not ready");
        return -1;
    }
    
    ret = gpio_pin_configure_dt(&onboard_led, GPIO_OUTPUT_INACTIVE);
    if (ret)
    {
        LOG_ERR("Failed to configure LED pin directional output (%d)", ret);
        return -1;
    }

    /* Initialize baseline sensor communication configurations */
    ret = imu_init();
    if (ret)
    {
        LOG_ERR("IMU registration failed (%d)", ret);
        return -1;
    }

    /* Inject legacy hardware parameters (Free-fall and Stillness registers) */
    ret = imu_configure_hw_features();
    if (ret)
    {
        LOG_ERR("Register feature configuration failed (%d)", ret);
        return -1;
    }

    /* Setup hardware interrupt channels */
    ret = imu_int_init();
    if (ret)
    {
        LOG_ERR("Interrupt registration failed (%d)", ret);
        return -1;
    }

    /* Initial latch clearance check on startup */
    if (imu_int_level() == 1)
    {
        imu_hw_event_t boot_event;
        imu_process_hw_events(&boot_event);
        parse_hardware_event(boot_event);
    }

    struct k_sem *hw_sem = imu_get_hw_event_sem();

    while (1)
    {
        /* Check for an interrupt event with a non-blocking 5ms window */
        if (k_sem_take(hw_sem, K_MSEC(5)) == 0)
        {
            imu_hw_event_t registered_event = IMU_HW_EVENT_NONE;

            if (imu_process_hw_events(&registered_event) != 0)
            {
                i2c_error_count++;
                if (i2c_error_count >= 3U)
                {
                    uint8_t whoami = 0;
                    if (imu_read_register(ISM_REG_WHO_AM_I, &whoami) == 0 && whoami == 0x6BU)
                    {
                        imu_configure_hw_features(); 
                    }
                    i2c_error_count = 0U;
                }
                continue;
            }

            i2c_error_count = 0U;

            if (registered_event != IMU_HW_EVENT_NONE)
            {
                parse_hardware_event(registered_event);
            }
        }

        /* Continually evaluate the asynchronous LED blink timer */
        update_led_state();

        /* Always run the streaming engine unconditionally every loop pass */
        process_fifo_stream();
    }

    return 0;
}

/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-08     Jackistang   the first version
 */
#include "../inc/max30102.h"
#include "../inc/CircularBuffer.h"
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_LEVEL   DBG_LOG
#include <rtdbg.h>
#define LOG_TAG                "sensor.max30102"

#ifndef MIN
    #define MIN(a, b)   ((a) < (b) ? (a) : (b))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) \
        ((unsigned long)(sizeof(array) / sizeof((array)[0])))
#endif

//register addresses
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PILOT_PA        0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR       0x1F
#define REG_TEMP_FRAC       0x20
#define REG_TEMP_CONFIG     0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID          0xFE
#define REG_PART_ID         0xFF


#define MAX30102_ADDR 0x57 // 7-bit version of the above

static struct rt_i2c_bus_device *i2c_bus;
static rt_uint32_t g_heartrate;

static void max30102_thread_entry(void *args);

/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
static rt_bool_t maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
    uint8_t buf[2];
    struct rt_i2c_msg msg;
    rt_size_t err;

    buf[0] = uch_addr;
    buf[1] = uch_data;

    msg.addr = MAX30102_ADDR;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = ARRAY_SIZE(buf);

    if ((err = rt_i2c_transfer(i2c_bus, &msg, 1)) != 1)
    {
        LOG_E("I2c write failed (err: %d).", err);
        return RT_FALSE;
    }

    return RT_TRUE;
}

/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data   - pointer that stores the register data
*
* \retval       true on success
*/
static rt_bool_t maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *data, uint16_t len)
{
    RT_ASSERT(data);

    struct rt_i2c_msg msg_buf[2];
    rt_size_t err;

    msg_buf[0].addr  = MAX30102_ADDR;
    msg_buf[0].flags = RT_I2C_WR;
    msg_buf[0].buf   = &uch_addr;
    msg_buf[0].len   = 1;

    msg_buf[1].addr  = MAX30102_ADDR;
    msg_buf[1].flags = RT_I2C_RD;
    msg_buf[1].buf   = data;
    msg_buf[1].len   = len;

    if ((err = rt_i2c_transfer(i2c_bus, msg_buf, 2)) != 2)
    {
        LOG_E("I2c read failed (err: %d).", err);
        return RT_FALSE;
    }

    return RT_TRUE;
}

/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
static rt_bool_t maxim_max30102_init()
{
    if (!maxim_max30102_write_reg(REG_INTR_ENABLE_1, 0xc0)) // INTR setting
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_INTR_ENABLE_2, 0x00))
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_FIFO_WR_PTR, 0x00)) //FIFO_WR_PTR[4:0]
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_OVF_COUNTER, 0x00)) //OVF_COUNTER[4:0]
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_FIFO_RD_PTR, 0x00)) //FIFO_RD_PTR[4:0]
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_FIFO_CONFIG, 0x4f)) //sample avg = 4, fifo rollover=RT_FALSE, fifo almost full = 17
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x03))  //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_SPO2_CONFIG, 0x27)) // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (411uS)
        return RT_FALSE;

    if (!maxim_max30102_write_reg(REG_LED1_PA, 0x24))  //Choose value for ~ 7mA for LED1
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_LED2_PA, 0x24))  // Choose value for ~ 7mA for LED2
        return RT_FALSE;
    if (!maxim_max30102_write_reg(REG_PILOT_PA, 0x7f))  // Choose value for ~ 25mA for Pilot LED
        return RT_FALSE;

    return RT_TRUE;
}

/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
static rt_bool_t maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
    uint8_t uch_temp;
    uint8_t buf[6];

    *pun_ir_led = 0;
    *pun_red_led = 0;

    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp, sizeof(uch_temp));
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp, sizeof(uch_temp));

    if (!maxim_max30102_read_reg(REG_FIFO_DATA, buf, ARRAY_SIZE(buf)))
    {
        LOG_E("Max30102 read fifo failed.");
        return RT_FALSE;
    }

    *pun_red_led = (((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2]);
    *pun_ir_led  = (((uint32_t)buf[3] << 16) + ((uint32_t)buf[4] << 8) + buf[5]);

    *pun_red_led &= 0x03FFFF;   //Mask MSB [23:18]
    *pun_ir_led  &= 0x03FFFF;   //Mask MSB [23:18]

    return RT_TRUE;
}

/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
static rt_bool_t maxim_max30102_reset()
{
    if (!maxim_max30102_write_reg(REG_MODE_CONFIG, 0x40))
    {
        LOG_E("Max30102 reset failed.");
        return RT_FALSE;
    }

    return RT_TRUE;
}


static RT_SIZE_TYPE max30102_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(sensor);
    RT_ASSERT(buf);
    RT_ASSERT(len == sizeof(struct rt_sensor_data));

    struct rt_sensor_data *data = buf;
    data->timestamp = rt_tick_get();
    data->type = RT_SENSOR_UNIT_BPM;
    data->data.hr = g_heartrate;

    return len;
}

static rt_err_t max30102_control(struct rt_sensor_device *sensor, int cmd, void *arg)
{
    RT_ASSERT(sensor);
    RT_ASSERT(arg);

    if (cmd == RT_SENSOR_CTRL_SET_MODE || cmd == RT_SENSOR_CTRL_SET_POWER)
        return RT_EOK;

    LOG_E("Now support command: (%d).", cmd);
    return RT_ERROR;
}

static struct rt_sensor_ops max30102_ops = {
    .fetch_data = max30102_fetch_data,
    .control = max30102_control,
};


int rt_hw_max30102_init(struct rt_sensor_config *cfg)
{
    RT_ASSERT(cfg);
    RT_ASSERT(cfg->mode == RT_SENSOR_MODE_POLLING); // Only support polling.

    char *err_msg = RT_NULL;
    rt_thread_t tid;
    rt_sensor_t sensor = RT_NULL;

    do {
        i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(cfg->intf.dev_name);
        if (i2c_bus == RT_NULL)
        {
            err_msg = "Not find i2c bus (cfg->intf.dev_name).";
            break;
        }

        sensor = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor == RT_NULL) {
            err_msg = "rt_calloc error.";
            break;
        }

        struct rt_sensor_info info = {
            .type = RT_SENSOR_CLASS_HR,
            .vendor = RT_SENSOR_VENDOR_MAXIM,
            .model = "max30102",
            .unit = RT_SENSOR_UNIT_BPM,
            .intf_type = RT_SENSOR_INTF_I2C,
            .range_max = 200,       // 200 bpm
            .range_min = 30,        // 30 bpm
            .period_min = 1,        // 1 ms
        };
        rt_memcpy(&sensor->info, &info, sizeof(struct rt_sensor_info));
        rt_memcpy(&sensor->config, cfg, sizeof(struct rt_sensor_config));
        sensor->ops = &max30102_ops;

        if (rt_hw_sensor_register(sensor, "max30102", RT_DEVICE_FLAG_RDONLY, RT_NULL) != RT_EOK) {
            err_msg = "Register max30102 sensor device error.";
            break;
        }

        // MAX30102 Init.
        uint8_t uch_dummy;
        maxim_max30102_reset(); //resets the MAX30102
        rt_thread_mdelay(1000);

        maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_dummy, sizeof(uch_dummy));  //Reads/clears the interrupt status register
        maxim_max30102_init();  //initialize the MAX30102

        tid = rt_thread_create("max30102", max30102_thread_entry, sensor, 
                        MAX30102_STACK_SIZE, MAX30102_PRIORITY, MAX30102_TICKS);
        if (tid == RT_NULL) {
            err_msg = "Create max30102 thread error.";
            break;
        }
        rt_thread_startup(tid);

    }while (0);

    if (err_msg) {
        if (sensor) rt_free(sensor);
        if (tid)    rt_thread_delete(tid);

        LOG_E(err_msg);
        return RT_ERROR;
    }
    return RT_EOK;
}


/* Heart rate algorithm */

#define FS  100

/*******************************************************************************
* Reference: https://blog.csdn.net/qq_29874741/article/details/80007763
* Function Name  : calc_average
* Description    : 计算平均值 公式: avg=last_avg*((num-1)/num)+next_val*1/num        
* Input          : last_avg:上一次平均值 next_val:加入计算的值 num:到现在加入计算的总个数
* Output         : None
* Return         : 平均值
*******************************************************************************/
static float calc_average(float last_avg, float next_val, uint64_t num)
{
    float avg_val = 0;
    if (num <= 1)
    {
        avg_val = next_val;
    }
    else
    {
        avg_val = (last_avg * ((float)(num - 1) / (float)num) + next_val * (1 / (float)num)); /*必须强转float*/
    }
    return avg_val;
}

static float get_interval_average_tick(CircularBuffer cBuf)
{
    size_t size = CircularBufferGetDataSize(cBuf) / sizeof(rt_tick_t);
		if (0 == size)
			return 0;
		
    rt_tick_t peek_buf[size];
    int count = 0;
    float avg = 0;

    CircularBufferRead(cBuf, size * sizeof(rt_tick_t), peek_buf);
    for (int i = 0; i < size-1; i++) {
        count++;
        avg = calc_average(avg, peek_buf[i+1] - peek_buf[i], count);
    }

    return avg / 1000.0f;    //ms -> s
}

static void max30102_thread_entry(void *args)
{
    rt_sensor_t sensor = args;
    uint32_t pun_red_led, pun_ir_led;
    uint32_t last_red, last_if;

    int16_t red, ir;

    int16_t threshold = -40;        // lab data
    const int16_t peek_min = -3000;
    const int16_t peek_max = -100;
    int16_t min = 0;
    uint64_t count = 0;
    float last_peek_avg = 0;

    rt_tick_t tick = rt_tick_get();
    rt_tick_t sample_tick = rt_tick_get();
    rt_tick_t init_tick = rt_tick_get();

    CircularBuffer cirbuf = CircularBufferCreate(30 * sizeof(rt_tick_t));
    int init_flag = 0;

    pun_red_led = last_red = 0;

    rt_pin_mode(sensor->config.irq_pin.pin, PIN_MODE_INPUT_PULLUP);

    while (1)
    {
        tick = rt_tick_get();

        if (tick - sample_tick >= 1000 / FS)      //FS Hz sample rate
        {
            if (rt_pin_read(sensor->config.irq_pin.pin) == 0)
            {
                maxim_max30102_read_fifo(&pun_red_led, &pun_ir_led);

                red = pun_red_led - last_red;
                ir = pun_ir_led - last_if;

                //update threshold
                if (red < threshold)
                {
                    min = MIN(min, red);
                }
                else if (min < 0)
                {
                    if (min > peek_min && min < peek_max)
                    {
                        CircularBufferPush(cirbuf, &tick, sizeof(tick));
                        count++;
                        last_peek_avg = calc_average(last_peek_avg, min, count);
                        threshold = last_peek_avg * 0.4f;
                    }

                    min = 0;
                }

//                rt_kprintf("<max30102>: %d, %d, %d, %d\n", red, ir, threshold, g_heartrate);

                last_red = pun_red_led;
                last_if = pun_ir_led;

                if (init_flag)
                {
                    float interval_average_tick = get_interval_average_tick(cirbuf);
									
                    if (0 == interval_average_tick)
                        g_heartrate = 999;
                    else
                        g_heartrate = 60.0f / interval_average_tick;

                    rt_tick_t data;
                    while (CircularBufferRead(cirbuf, sizeof(data), &data) == sizeof(data))
                    {
                        if (tick - data > 8000ul)
                            CircularBufferPop(cirbuf, sizeof(data), &data);
                        else
                            break;
                    }
                }
            }

            sample_tick = tick;
        }

        if (tick - init_tick >= 8000ul)     // 8s
        {
            init_flag = 1;
            init_tick = tick;
        }

        rt_thread_mdelay(1);
    }
}

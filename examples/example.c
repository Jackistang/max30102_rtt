#include <rtthread.h>
#include <rtdevice.h>

#define DBG_LEVEL   DBG_LOG
#include <rtdbg.h>
#define LOG_TAG                "example.hr"

void heart_rate_example(void)
{
    rt_device_t dev = rt_device_find("hr_max30102");
    if (dev == RT_NULL) {
        LOG_E("Find max30102 error");
        return ;
    }

    rt_device_open(dev, RT_DEVICE_FLAG_RDONLY);

    struct rt_sensor_data data;
    for (int i = 0; i < 10; i++) {
        if (rt_device_read(dev, 0, &data, sizeof(data)) == sizeof(data)) {
            LOG_D("heart rate: %d", data.data.hr);
        }
				rt_thread_mdelay(1000);
    }

    rt_device_close(dev);
}
MSH_CMD_EXPORT(heart_rate_example, heart rate example);

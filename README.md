# max30102_rtt
## 介绍

MAX30102 传感器的软件包，已对接 sensor 框架，目前仅支持测量**心率**，血氧饱和度还不支持。

心率波形图如下，软件为 [VOFA+](https://www.vofa.plus/) ，右上角 I0 为红光强度采样值，I2 为动态阈值，I3 为心率值。

![心率](docs/images/心率.gif)

### 目录结构

| 名称     | 说明         |
| -------- | ------------ |
| docs     | 文档目录     |
| examples | 使用例程目录 |
| inc      | 头文件目录   |
| src      | 源代码目录   |
| port     | 移植代码目录 |

### 许可证

MAX30102 package 遵循 Apache-2.0 License 许可，详见 `LICENSE` 文件。

### 依赖

- RT-Thread 4.0
- RT-Thread I2C 设备框架
- RT-Thread sensor 框架

## 如何打开 MAX30102

使用 MAX30102 package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    peripherals packages --->
        [*] sensors package --->
           [*] max30102
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 使用 MAX30102

利用 `rt_device_find()` 找到传感器设备，然后用 `rt_device_open()` 打开设备，最后利用 `rt_device_read()` 读取设备数据即可。

```C
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
    }

    rt_device_close(dev);
}
```

## 注意事项

- `RT_TICK_PER_SECOND` 的值需要为 1000 （默认值）。

## 联系方式 & 感谢

- 维护：Jackistang
- 主页：[https://github.com/Jackistang](https://github.com/Jackistang)
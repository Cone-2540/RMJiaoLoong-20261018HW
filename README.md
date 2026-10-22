# BMI088 IMU 作业

基于已调通的 C 板 SPI 工程，完成原始数据读取、陀螺仪零偏校准、roll/pitch/yaw、互补滤波，并加入 Mahony 和 DMA 中断采样。

## 代码入口

| 文件 | 职责 |
| --- | --- |
| `UserCode/bmi088.c/.h` | 保留原来的寄存器访问与初始化；读取、解析原始数据，完成单位换算和 DMA 传输 |
| `UserCode/imu.cpp/.hpp` | 纯 C++ `Imu` 类；校准、互补滤波和 Mahony，不引用 HAL、SPI、GPIO |
| `UserCode/imu_app.cpp` | 定时调度、驱动与算法连接、Scope 变量；安装方向变换也应放在这里 |
| `Core/Src/main.c` | 初始化后调用 `imu_app_init()`，主循环调用 `imu_app_poll()` |
| `tests/imu_test.cpp` | 在电脑上验证校准、组合旋转、yaw 跨界和滤波反馈 |

原 `imu.c` 更名为 `bmi088.c`，`imu.h` 保留兼容入口。此前的 `acc_calculate()`、`gyro_calculate()` 仍可用于讲义练习，应用采样使用一次性读出的同一组原始数据，避免重复读寄存器。`imu_monitor.cpp` 是原有参考文件，不参与编译。

## 编译与烧录

1. Keil 打开 `MDK-ARM/20261018.uvprojx`，选择 `20261018`，使用 ARM Compiler 6。
2. Build，应为 **0 Error、0 Warning**。项目已启用 C++11，并加入 BMI088、IMU 类与应用三个源文件。
3. Scope 若已连接，先 Disconnect 并停止 OpenOCD；Keil 退出 Debug 后再 Download。ST-Link 同时由一个调试工具使用。
4. 烧录完成后复位，**板子静止约 2 秒**。`imu_calibrated=1` 后开始旋转。

启动 Scope 后需要再次烧录时，也按第 3 步释放探针。使用 SWD，关闭 SWO Trace：PB3 是 SPI1 时钟引脚。

可选 CMake 构建（STM32CubeCLT 的 GCC、CMake、Ninja 已加入 PATH）：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

算法测试（MinGW g++）：

```powershell
New-Item -ItemType Directory -Force build/host
g++ -std=c++11 -Wall -Wextra -Werror -O2 -IUserCode tests/imu_test.cpp UserCode/imu.cpp -o build/host/imu_test.exe
./build/host/imu_test.exe
```

## 初始化与采样

- STM32F407，168 MHz；SPI1 Mode 3，软件片选，分频 256。
- 加速度计 CS 为 **PA4**，陀螺仪 CS 为 **PB0**。ID 分别为 `0x1E`、`0x0F`。
- 加速度计：±6 g、800 Hz；陀螺仪：±2000 °/s、1000 Hz、116 Hz 带宽。
- 原始数据按低字节在前组成 `int16_t`。加速度转为 m/s²，角速度转为 rad/s。
- TIM6：84 MHz / (83+1) / (1999+1) = **500 Hz**，每 2 ms 发出采样节拍。中断只计数；主循环启动 SPI、处理数据。
- SPI1 RX：DMA2 Stream0 Channel3；TX：DMA2 Stream3 Channel3，Normal、Byte、Memory increment。启用两个 DMA IRQ、SPI1 IRQ、TIM6 IRQ。设置已同步到 `.ioc`。
- 加速度计 DMA 包为「地址 `0x92` + dummy + 6 字节数据」；陀螺仪包为「地址 `0x82` + 6 字节数据」。完成回调释放片选并启动下一段，主循环取走整组数据后才能再次启动。
- DMA 缓冲区为静态 SRAM 数组。正常采样不等待 SPI；上电初始化使用阻塞读写，异常超时后在主循环中恢复 SPI。
- DWT 周期计数器测量相邻采样起点的真实 `dt`，无符号减法处理计数器回绕。调试暂停超过 50 ms 后重新校准，避免把停顿积分进去。

## 校准与姿态

上电累计 1000 个静止样本，计算三轴陀螺仪均值作为零偏。加速度模长偏离 1 g 超过 15%、角速度模长超过 0.15 rad/s，或窗口方差过大时，重新累计。校准后每个样本先减零偏，再进入两种滤波器。校准时应保持板子静止。

算法使用右手坐标系，机体坐标沿用 BMI088 芯片的 X/Y/Z。本次连接的板子元件面朝上时，`acc_z` 约为 +9.8 m/s²。讲义中的 `diag(-1,+1,-1)` 是另一种安装方向示例，当前没有使用，以便平放时 roll、pitch 接近 0。真实桌面倾斜及传感器误差会留下小角度，本次实测 roll 约 -1.17°、pitch 约 0.17°。

互补滤波先用四元数积分陀螺仪，再用加速度估计的倾角修正 roll/pitch，时间常数 0.5 s，`alpha=tau/(tau+dt)`。Mahony 用测量与预测的重力方向叉乘修正角速度，`Kp=2.0`、`Ki=0.05`。加速度模长不在 0.85～1.15 g 时暂停重力修正，减少明显平动的影响。

角度按 Z-Y-X 欧拉角输出：roll、yaw 范围 ±180°，pitch 范围 ±90°。互补滤波在 pitch 接近 ±90° 时暂停欧拉倾角修正，四元数仍然预测姿态。录制时推荐各轴缓慢转动约 ±45°，清楚展示对应变化。

**yaw 是校准开始方向的相对航向**。BMI088 没有磁力计，重力无法修正绕竖直轴的航向，长时间会有漂移；两种六轴滤波都具有这个特性。重新校准会把 yaw 重置为 0。

## Scope

使用桌面的 `Scope-v0.4.5`，通过 ST-Link / SWD 读取 RAM，**无需串口线或串口发送代码**。

1. 退出 Keil Debug，双击工程根目录 **`start_scope.cmd`**。
2. 脚本启动 Scope，配置 OpenOCD，并加载本次 Keil 的 `MDK-ARM/20261018/20261018.axf`。浏览器打开 `http://127.0.0.1:8001`。
3. 默认预设为 `roll_deg`、`pitch_deg`、`yaw_deg`，采集频率 **50 Hz**，点击 **Start** 开始画图。板上算法仍以 500 Hz 更新。
4. 更换观测组：先停止采样并关闭旧 Scope 页面，打开 [预设选择页](http://127.0.0.1:8001/assets/imu-setup.html)，选择姿态、两种滤波对比、校准前后、传感器数据，再点击 Start。旧页面关闭后不会把旧配置写回。

预设页由 `scripts/setup_scope.py` 写入 Scope 的静态资源目录，只保存本工程的浏览器配置。重新执行启动脚本可恢复默认姿态预设。脚本要求 Python 和 OpenOCD 在 PATH；本次电脑已配置好。

| 变量 | 含义 / 正常状态 |
| --- | --- |
| `roll_deg / pitch_deg / yaw_deg` | 当前选用算法输出，单位 °；默认互补滤波 |
| `comp_*_deg / mahony_*_deg` | 两种算法同时计算的结果，便于对比 |
| `acc_raw[3] / gyro_raw[3]` | 芯片原始整数，Keil Watch 可展开三轴 |
| `acc_x / acc_y / acc_z` | 加速度，m/s² |
| `gyro_uncal_*_dps / gyro_*_dps` | 校准前 / 后角速度，°/s |
| `bias_*_dps` | 静止校准得到的三轴零偏，°/s |
| `imu_init_ok / imu_calibrated` | 均应为 1 |
| `imu_calibration_count` | 校准进度，完成后为 1000 |
| `imu_acc_id / imu_gyro_id` | 十进制 30 / 15，即 `0x1E / 0x0F` |
| `imu_sample_hz / imu_dt_ms` | 约 500 / 2 |
| `imu_sample_count / bmi088_dma_count` | 采样与 DMA 完成次数持续增加 |
| `bmi088_spi_fail_count / imu_skipped_ticks` | 应保持 0 |
| `imu_algorithm` | 可写：0 互补滤波、1 Mahony；只切换显示，两个算法始终更新 |
| `imu_recalibrate` | 板子静止时写 1，启动重新校准，程序随后自动清零 |
| `imu_use_dma` | 可写：1 DMA、0 阻塞读取；默认 1 |

写变量使用 Scope 的变量列表操作。未勾选绘图的诊断变量可手动刷新读取；需要连续观察时临时勾选。原始数组也可以在 Keil Debug 的 Watch 中查看：先断开 Scope，避免抢占 ST-Link。

Scope 加载的 AXF 必须与板上固件匹配。修改代码后重新 Build、烧录，再执行启动脚本，地址会从新 AXF 重新解析。

## 验收视频（约 2 分钟）

最方便的方式是**手机同时拍到 C 板和 Scope 屏幕**。保持 ST-Link 连接，画面中能看清你怎么转板、曲线怎么变化；需要画面更清楚时再使用屏幕录制加手机画中画。

1. **0～15 秒：准备与校准。** 展示板子、项目名称、连接状态。复位后静置，刷新确认 `imu_calibrated=1`、`imu_sample_hz≈500`，然后 Start。平放几秒展示稳定曲线。
2. **15～45 秒：roll、pitch。** 绕 X 轴慢慢转到约 +45°，停 2 秒，再反向到约 -45°，回到平放；绕 Y 轴重复。说明对应的是 roll、pitch。动作以转动为主，避免猛甩或大幅平移。
3. **45～65 秒：yaw。** 保持板面水平，绕竖直 Z 轴转约 +90°、停顿、再转回。说明 yaw 为相对角度，开始方向为 0°。
4. **65～90 秒：零偏校准。** 切换“校准前 / 后”预设，静放展示校准后角速度在 0 附近波动，说明三轴 `bias_*_dps` 为启动时的平均零偏。可写 `imu_recalibrate=1` 演示进度从 0 到 1000。零偏本来很小时，重点展示均值被扣除，曲线仍有正常噪声。
5. **90～120 秒：Extra。** 切换滤波对比预设，缓慢转板，展示互补与 Mahony 曲线同时变化。最后刷新 `bmi088_dma_count` 两次，说明计数增加、SPI 错误为 0。停止采样，结束录像。

只验收基础部分时，前三段加零偏展示即可。预设切换间隙可以剪掉，保留真实转板和曲线变化。视频中不需要刻意把平放时的小倾角调到精确 0。

实板测试记录见 [docs/validation.md](docs/validation.md)。

# esp32-freertos-task-queue · 姿态监测系统多任务改造

> 把原本「一个 `while(1)` 顺序跑完」的姿态监测程序，重构成
> **采集 / 解算 / UI 三个 FreeRTOS 任务 + 两个队列**的架构。
> 基于真实项目 ESP32-S3 + QMI8658 六轴传感器改造，解算公式与原版完全一致。

## 改造前后对比

**改造前（v1）**：所有事情挤在一个循环里

```c
while (1) {
    qmi8658_fetch_angleFromAcc(&value);   // 读 I2C（慢）+ 算角度
    lv_label_set_text(a, buf);            // 刷屏（更慢）
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

问题：**I2C 读取和 LVGL 刷屏互相阻塞**。LVGL 刷新一次要几毫秒到几十毫秒，
这段时间传感器根本没被读取，采样点直接漏掉——姿态曲线出现台阶，快速转动时尤其明显。

**改造后（v2）**：

```
[采集任务 prio=3] --q_raw(长度5)--> [解算任务 prio=2] --q_attitude(长度3)--> [UI任务 prio=1]
   每 10ms 读 I2C                      收到就解算                            每 50ms 刷屏
```

## 三个任务的职责边界

| 任务 | 优先级 | 周期 | 只做 | 绝不碰 |
| --- | --- | --- | --- | --- |
| `task_sample` | 3（最高） | 10ms | I2C 读取 | 不做计算、不碰屏幕 |
| `task_process` | 2 | 事件驱动 | 角度解算 | 不读硬件、不碰屏幕 |
| `task_ui` | 1（最低） | 50ms | LVGL 刷新 | 不读硬件、不算角度 |

**职责单一**是这次改造的核心收益：每个任务只干一件事，
任何一环变慢都不会拖垮其他环。

## 编译与烧录

需要 ESP-IDF v5.x 环境（本项目是纯应用层代码，依赖 `esp_driver_i2c`、`lvgl` 组件）：

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

> ⚠️ 本项目由本地开发环境整理后开源，**编译验证需在装有 ESP-IDF 的机器上完成**。
> 传感器驱动（`components/QMI`）与解算算法沿用原项目，接口为
> `Qmi8658_Read_Acc()` / `IIC_Init()` / `Qmi8658_Init()`。

## 关键设计决策

| 决策 | 原因 |
| --- | --- |
| 采集优先级最高 | 传感器数据有时效性，晚读就是丢采样点 |
| 采集用 `xQueueSend(..., 0)` 非阻塞 | 队列满时丢旧帧，**绝不能反过来阻塞 I2C 时序** |
| 队列长度只要 5 和 3 | 太长了传的是「过期旧值」，缓冲突变毫无意义 |
| UI 优先级最低 | 屏幕刷新最慢，不能影响数据采集；人眼 20Hz 已足够流畅 |
| 栈给 4096 word（约 16KB） | LVGL 相关调用栈较深，给少了会栈溢出重启 |
| 解算任务用 `portMAX_DELAY` 阻塞 | 没数据就挂起，不空转占 CPU |

## 目录结构

```
esp32-freertos-task-queue/
├── main/
│   ├── main.c              # 三任务 + 双队列架构
│   └── CMakeLists.txt
├── CMakeLists.txt
├── docs/讲解文档.md         # 改造思路、代码走读、面试问答
└── README.md
```

## 配套仓库

- [esp32-qmi8658-attitude-monitor](https://github.com/swdscsc/esp32-qmi8658-attitude-monitor)
  —— 原始单任务版本 + QMI8658 驱动 + 《遇到的问题与解决》文档
- [esp32-freertos-peripheral-basics](https://github.com/swdscsc/esp32-freertos-peripheral-basics)
  —— GPIO / 按键中断 / FreeRTOS 多任务基础实验

## 后续可做

- [ ] 用 `xQueueOverwrite()` 替代 UI 队列：只保留最新一帧
- [ ] 加二值信号量做「数据就绪」中断通知，替代固定 10ms 轮询
- [ ] 加互补滤波融合陀螺仪数据，解决纯加速度解算的动态抖动

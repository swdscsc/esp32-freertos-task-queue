#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "led.h"
#include "key.h"
#include "lcd.h"
#include "qmi.h"
#include "pic.h"

/* ============================================================================
 * 姿态监测系统 —— FreeRTOS 多任务 + 队列版
 *
 * 改造前（v1）：单个 while(1) 里顺序执行 读传感器 → 算角度 → 刷屏幕
 *   问题：I2C 读取（慢）和 LVGL 刷屏（更慢）互相阻塞，
 *         屏幕一忙，传感器采样就漏点，姿态曲线出现台阶。
 *
 * 改造后（v2）：拆成三个任务，用两个队列传递数据
 *
 *   [采集任务 prio=3] --q_raw--> [解算任务 prio=2] --q_attitude--> [UI任务 prio=1]
 *     每 10ms 读 I2C              收到就解算角度                   每 50ms 刷屏
 *
 * 为什么这样设计：
 *   1. 采集优先级最高 —— 传感器数据有时效性，晚读就是丢采样点
 *   2. 采集任务绝不阻塞 —— 队列满了丢旧帧保新帧，不能反过来卡住 I2C 时序
 *   3. UI 优先级最低 —— 屏幕刷新慢不影响数据采集，人眼看 20Hz 已经很流畅
 * ==========================================================================*/

static const char *TAG = "attitude";

/* -------- 任务间传递的数据结构 -------- */

/** 采集 → 解算：传感器原始值 */
typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
} raw_data_t;

/** 解算 → UI：解算后的角度 */
typedef struct {
    float roll;     /* X 轴倾角 */
    float pitch;    /* Y 轴倾角 */
    float yaw;      /* Z 轴倾角 */
} attitude_t;

/* -------- 队列句柄（全局，三个任务都要用）-------- */
static QueueHandle_t q_raw;       /* 采集 → 解算 */
static QueueHandle_t q_attitude;  /* 解算 → UI   */

/* -------- LVGL 标签句柄（UI 任务专用，其他任务不碰屏幕）-------- */
static lv_obj_t *label_x;
static lv_obj_t *label_y;
static lv_obj_t *label_z;

#define RAD_TO_DEG 57.29578f   /* 180/π */

/**
 * 由加速度原始值解算三轴倾角。
 * 公式与原版 qmi8658_fetch_angleFromAcc() 完全一致，只是把「读寄存器」和「算角度」分开了：
 * 本函数只做纯计算，不碰硬件。
 */
static void attitude_from_acc(const raw_data_t *raw, attitude_t *att)
{
    float temp;

    temp = (float)raw->acc_x /
           sqrtf((float)raw->acc_y * raw->acc_y + (float)raw->acc_z * raw->acc_z);
    att->roll = atanf(temp) * RAD_TO_DEG;

    temp = (float)raw->acc_y /
           sqrtf((float)raw->acc_x * raw->acc_x + (float)raw->acc_z * raw->acc_z);
    att->pitch = atanf(temp) * RAD_TO_DEG;

    temp = sqrtf((float)raw->acc_x * raw->acc_x + (float)raw->acc_y * raw->acc_y) /
           (float)raw->acc_z;
    att->yaw = atanf(temp) * RAD_TO_DEG;
}

/* ========================== 任务 1：采集 ==========================
 * 职责：只做 I2C 读取，不做计算、不碰屏幕
 * 优先级 3（最高）—— 传感器数据有时效性
 */
static void task_sample(void *arg)
{
    t_sQMI8658 sensor;
    raw_data_t raw;

    while (1) {
        Qmi8658_Read_Acc(&sensor);          /* 读加速度 + 陀螺仪寄存器 */

        raw.acc_x = sensor.acc_x;
        raw.acc_y = sensor.acc_y;
        raw.acc_z = sensor.acc_z;
        raw.gyr_x = sensor.gyr_x;
        raw.gyr_y = sensor.gyr_y;
        raw.gyr_z = sensor.gyr_z;

        /* 关键：阻塞时间为 0。
         * 队列满时直接丢弃这一帧，绝不让采集任务等待 ——
         * 一旦采集被阻塞，I2C 时序就乱了，采样点会漏。宁可丢旧数据，不能停采集。 */
        if (xQueueSend(q_raw, &raw, 0) != pdTRUE) {
            ESP_LOGW(TAG, "raw queue full, frame dropped");
        }

        vTaskDelay(pdMS_TO_TICKS(10));      /* 100Hz 采样 */
    }
}

/* ========================== 任务 2：解算 ==========================
 * 职责：从队列取原始值，算角度，再丢给 UI
 * 优先级 2 —— 比采集低（数据来了就算），比 UI 高（不能让显示拖慢解算）
 */
static void task_process(void *arg)
{
    raw_data_t raw;
    attitude_t att;

    while (1) {
        /* 阻塞等待：没数据就挂起，不占 CPU。这是队列相比全局变量轮询的优势 */
        if (xQueueReceive(q_raw, &raw, portMAX_DELAY) == pdTRUE) {
            attitude_from_acc(&raw, &att);

            if (xQueueSend(q_attitude, &att, 0) != pdTRUE) {
                /* UI 消费慢是正常的，丢帧可接受：人眼看不出 50ms 和 60ms 的区别 */
                ESP_LOGD(TAG, "attitude queue full, frame dropped");
            }
        }
    }
}

/* ========================== 任务 3：UI ==========================
 * 职责：只做 LVGL 刷新
 * 优先级 1（最低）—— 屏幕刷新是最慢的一环，绝不能影响采集
 */
static void task_ui(void *arg)
{
    attitude_t att;
    char buf[16];

    while (1) {
        /* 等待 50ms：有数据就刷新，没数据也到期返回，避免屏幕长时间不更新 */
        if (xQueueReceive(q_attitude, &att, pdMS_TO_TICKS(50)) == pdTRUE) {
            snprintf(buf, sizeof(buf), "%0.2f", att.roll);
            lv_label_set_text(label_x, buf);

            snprintf(buf, sizeof(buf), "%0.2f", att.pitch);
            lv_label_set_text(label_y, buf);

            snprintf(buf, sizeof(buf), "%0.2f", att.yaw);
            lv_label_set_text(label_z, buf);
        }
    }
}

/* ========================== 初始化 ========================== */
void app_main(void)
{
    led_init();
    key_init();
    bsp_lvgl_start();          /* LCD + LVGL 初始化 */
    IIC_Init();                /* I2C 初始化 */
    Qmi8658_Init();            /* QMI8658 姿态传感器配置 */

    lcd_show_pic(0, 0, 320, 240, gImage_pic);

    /* 三个角度标签（沿用 v1 的布局） */
    label_x = lv_label_create(lv_scr_act());
    lv_label_set_text(label_x, "0");
    lv_obj_set_pos(label_x, 40, 110);
    lv_obj_set_style_text_color(label_x, lv_color_hex(0xff0000), 0);
    lv_obj_set_style_text_font(label_x, &lv_font_montserrat_24, 0);

    label_y = lv_label_create(lv_scr_act());
    lv_label_set_text(label_y, "0");
    lv_obj_set_pos(label_y, 140, 110);
    lv_obj_set_style_text_color(label_y, lv_color_hex(0x00ff00), 0);
    lv_obj_set_style_text_font(label_y, &lv_font_montserrat_24, 0);

    label_z = lv_label_create(lv_scr_act());
    lv_label_set_text(label_z, "0");
    lv_obj_set_pos(label_z, 240, 110);
    lv_obj_set_style_text_color(label_z, lv_color_hex(0x0000ff), 0);
    lv_obj_set_style_text_font(label_z, &lv_font_montserrat_24, 0);

    /* --- 创建队列 ---
     * 长度 5 和 3：够缓冲几帧就行。太长了数据会是「过期的旧值」，反而不好。 */
    q_raw      = xQueueCreate(5, sizeof(raw_data_t));
    q_attitude = xQueueCreate(3, sizeof(attitude_t));

    if (q_raw == NULL || q_attitude == NULL) {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }

    /* --- 创建三个任务 ---
     * 栈 4096（单位 word，ESP32 上 1 word = 4 字节，即约 16KB）
     * 优先级：采集 3 > 解算 2 > UI 1 */
    xTaskCreate(task_sample,  "sample",  4096, NULL, 3, NULL);
    xTaskCreate(task_process, "process", 4096, NULL, 2, NULL);
    xTaskCreate(task_ui,      "ui",      4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "attitude monitor started: 3 tasks + 2 queues");

    /* app_main 返回即可，三个任务在后台跑 */
}

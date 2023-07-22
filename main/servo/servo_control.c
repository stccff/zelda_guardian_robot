/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "servo_control.h"
#include "hid_host_gamepad.h"
#include <math.h>
#include "driver/gpio.h"
#include <string.h>

// #define MIN(a, b)   ((a < b) ? a : b)
// #define MAX(a, b)   ((a > b) ? a : b)

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

#define PWM_GROUP   (1)

// 舵机
#define SERVO_NUMS (3)
#define SERVO_STOP_TIMEOUT (1000)   // ms

// 摇杆
#define JOY_MID_VAL (XBOX_JOYSTICK_MAX / 2)
#define JOY_DEAD_ZONE (0.10)
#define SERVO_MAX_SPEED (60.0 / 300.0) // usually 200ms/60degree
#define PROCESS_PERIOD (10)            // 50ms

/* 结构体定义 */
typedef struct
{
    int pulseWidthUsMin;    // 硬件脉宽规格
    int pulseWidthUsMax;
    int angleSpecMIn;       // 硬件角度旋转范围
    int angleSpecMax;
    int angleLimitMin;      // 角度限制（软件配置）
    int angleLimitMax;
    int initAngle;          // 初始状态角度
    int gpioPin;            // gpio引脚
} ServoMotorCfgItemSt;

typedef struct
{   
    /* 配置信息 */
    const ServoMotorCfgItemSt cfg;
    /* 运行态信息 */
    mcpwm_cmpr_handle_t cmp;
    mcpwm_timer_handle_t timer;
    double currentAngle;
    uint64_t stopTimer;
    bool status;
} ServoMotorObjSt;

/* 全局变量 */
static const char *TAG = "servo";
static ServoMotorObjSt g_servo[SERVO_NUMS] = {
    {   /* 头部转动舵机 x轴 */
        .cfg = {
            .pulseWidthUsMin = 500,
            .pulseWidthUsMax = 2500,
            .angleSpecMIn = -90,
            .angleSpecMax = 90,
            .angleLimitMin = -90,
            .angleLimitMax = 90,
            .initAngle = 0,
            .gpioPin = GPIO_NUM_11,
        },
        .currentAngle = 0,
        .stopTimer = 0,
        .status = false,
    },
    {   /* 眼部激光舵机 y轴 */
        .cfg = {
            .pulseWidthUsMin = 500,
            .pulseWidthUsMax = 2500,
            .angleSpecMIn = -90,
            .angleSpecMax = 90,
            .angleLimitMin = -17,   // 上
            .angleLimitMax = 18,    // 下
            .initAngle = 0,
            .gpioPin = GPIO_NUM_12,
        },
        .currentAngle = 0,
        .stopTimer = 0,
        .status = false,
    },
    {   /* 肩部圆环舵机 */
        .cfg = {
            .pulseWidthUsMin = 500,
            .pulseWidthUsMax = 2500,
            .angleSpecMIn = -135,
            .angleSpecMax = 135,
            .angleLimitMin = -135,
            .angleLimitMax = 135,
            .initAngle = 0,
            .gpioPin = GPIO_NUM_10,
        },
        .currentAngle = 0,
        .stopTimer = 0,
        .status = false,
    },
};

/**
 * @brief 角度换算成比较器的值
 *
 * @param angle 角度
 * @return uint32_t 比较器值
 */
static uint32_t angle_to_compare(const ServoMotorCfgItemSt *cfg, double angle)
{
    return (angle - cfg->angleSpecMIn) / (cfg->angleSpecMax - cfg->angleSpecMIn) *
           (cfg->pulseWidthUsMax - cfg->pulseWidthUsMin) + cfg->pulseWidthUsMin;
}

/**
 * @brief 创建一个pwm发生器
 *
 * @param groupId 定时器/操作器id
 * @param gpioNum gpio管脚
 * @return mcpwm_cmpr_handle_t 比较器handler
 */
static mcpwm_cmpr_handle_t create_pwm_gen(int groupId, ServoMotorObjSt *servoObj)
{
    ESP_LOGI(TAG, "Create timer and operator");
    // mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = groupId,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &servoObj->timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = groupId, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, servoObj->timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = servoObj->cfg.gpioPin, // GPIO connects to the PWM signal line
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(&servoObj->cfg, servoObj->cfg.initAngle)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generator,
                                                               MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                                                               MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generator,
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW),
                                                                 MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(servoObj->timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(servoObj->timer, MCPWM_TIMER_START_NO_STOP));
    servoObj->status = true;

    return comparator;
}

/**
 * @brief Get the step angle
 *
 * @param joyVal
 * @return double
 */
static double get_step_angle(uint16_t joyVal)
{

    if (joyVal == 0xffff)
    {
        joyVal -= 1;
    }
    int16_t val = joyVal - JOY_MID_VAL;
    int8_t sign = (val < 0) ? -1 : 1;

    double percent = (double)val / JOY_MID_VAL;
    if (fabs(percent) < JOY_DEAD_ZONE)
    { // 死区处理
        return 0;
    }

    double maxAngle = SERVO_MAX_SPEED * PROCESS_PERIOD;
    double anglePercent = (fabs(percent) - JOY_DEAD_ZONE) / (1 - JOY_DEAD_ZONE);
    anglePercent = pow(anglePercent, 5); // 摇杆小角度时缓慢变化，大角度时快速变化
    double step = anglePercent * maxAngle;

    return step * sign;
}


/**
 * @brief 
 *
 * @param joyRx
 */
static void servo_relative_location_control(ServoMotorObjSt *servo, uint16_t speed)
{
    double stepAngle = get_step_angle(speed);   // 获取相对旋转角度
    
    if (stepAngle != 0) {
        servo->stopTimer = 0;
        if (servo->status == false) {
            // ESP_ERROR_CHECK(mcpwm_timer_enable(servo->timer));
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(servo->timer, MCPWM_TIMER_START_NO_STOP));   // 使能pwm
            servo->status = true;
        }
        servo->currentAngle += stepAngle;
        if (servo->currentAngle > servo->cfg.angleLimitMax) {
            servo->currentAngle = servo->cfg.angleLimitMax;
        }
        if (servo->currentAngle < servo->cfg.angleLimitMin) {
            servo->currentAngle = servo->cfg.angleLimitMin;
        }
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo->cmp, angle_to_compare(&servo->cfg, servo->currentAngle)));    // 更新
    } else {
        servo->stopTimer += PROCESS_PERIOD;
        if (servo->stopTimer > SERVO_STOP_TIMEOUT) {
            if (servo->status != false) {
                // ESP_ERROR_CHECK(mcpwm_timer_disable(servo->timer));
                ESP_ERROR_CHECK(mcpwm_timer_start_stop(servo->timer, MCPWM_TIMER_STOP_EMPTY));  // 关闭pwm，防止因堵转造成大电流
                // ESP_LOGI(TAG, "servo pwm disable");
                servo->status = false;
            }
        }
    }
    
    // printf("angle = %.2f\n", servo->currentAngle);
}


/**
 * @brief servo控制任务主函数
 *
 * @param arg
 */
static void servo_motor_task(void *arg)
{
    const struct XboxData *xbox = get_xbox_pad_data();
    while (1)
    {
        servo_relative_location_control(&g_servo[0], xbox->joyRx);
        servo_relative_location_control(&g_servo[1], xbox->joyRy);
        servo_relative_location_control(&g_servo[2], xbox->joyLx);
        vTaskDelay(pdMS_TO_TICKS(PROCESS_PERIOD));
    }
}

/**
 * @brief servo控制初始化函数
 *
 */
void servo_init(void)
{
    for (int i = 0; i < SERVO_NUMS; i++)
    {
        g_servo[i].cmp = create_pwm_gen(0, &g_servo[i]);    // 初始化各个舵机
    }

    xTaskCreate(servo_motor_task, "servo_motor_task", 1024 * 10, NULL, 3, NULL);
}

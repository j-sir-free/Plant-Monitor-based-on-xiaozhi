/**
 ****************************************************************************************************
 * @file        dht11.c
 * @author      普中科技
 * @version     V1.0
 * @date        2024-03-01
 * @brief       DHT11驱动代码
 * @license     Copyright (c) 2024-2034, 深圳市普中科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:普中科技 ESP32-S3 开发板
 * 在线视频:https://space.bilibili.com/2146492485
 * 技术论坛:www.prechin.net
 * 公司网址:www.prechin.cn
 * 购买地址:
 *
 ****************************************************************************************************
 */
 
#include "dht11.h"
 
 
/**
 * @brief       复位DHT11
 * @param       data: 要写入的数据
 * @retval      无
 */
void dht11_reset(void)
{
    DHT11_DQ_OUT(0);        /* 拉低DQ */
    vTaskDelay(25);         /* 拉低至少18ms */
    DHT11_DQ_OUT(1);        /* DQ=1 */
    esp_rom_delay_us(30);   /* 主机拉高10~35us */
}
 
/**
 * @brief       等待DHT11的回应
 * @param       无
 * @retval      0, DHT11正常
 *              1, DHT11异常/不存在
 */
uint8_t dht11_check(void)
{
    uint8_t retry = 0;
    uint8_t rval = 0;
 
    while (DHT11_DQ_IN && retry < 100)      /* DHT11会拉低40~80us */
    {
        retry++;
        esp_rom_delay_us(1);
    }
 
    if (retry >= 100)
    {
        rval = 1;
    }
    else
    {
        retry = 0;
 
        while (!DHT11_DQ_IN && retry < 100) /* DHT11拉低后会再次拉高40~80us */
        {
            retry++;
            esp_rom_delay_us(1);
        }
        
        if (retry >= 100)
        {
            rval = 1;
        }
    }
    
    return rval;
}
 
/**
 * @brief       从DHT11读取一个位
 * @param       无
 * @retval      读取到的位值: 0 / 1
 */
uint8_t dht11_read_bit(void)
{
    uint8_t retry = 0;
 
    while (DHT11_DQ_IN && retry < 100)  /* 等待变为低电平 */
    {
        retry++;
        esp_rom_delay_us(1);
    }
 
    retry = 0;
 
    while (!DHT11_DQ_IN && retry < 100) /* 等待变高电平 */
    {
        retry++;
        esp_rom_delay_us(1);
    }
 
    esp_rom_delay_us(40);               /* 等待40us */
 
    if (DHT11_DQ_IN)                    /* 根据引脚状态返回 bit */
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}
 
/**
 * @brief       从DHT11读取一个字节
 * @param       无
 * @retval      data：读到的数据
 */
static uint8_t dht11_read_byte(void)
{
    uint8_t i, data = 0;
 
    for (i = 0; i < 8; i++)         /* 循环读取8位数据 */
    {
        data <<= 1;                 /* 高位数据先输出, 先左移一位 */
        data |= dht11_read_bit();   /* 读取1bit数据 */
    }
 
    return data;
}
 
/**
 * @brief       从DHT11读取一次数据
 * @param       temp: 温度值(范围:-20~50°)
 * @param       humi: 湿度值(范围:5%~95%)
 * @retval      0, 正常.
 *              1, 失败
 */
uint8_t dht11_read_data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5];
    uint8_t i;
 
    dht11_reset();
 
    if (dht11_check() == 0)
    {
        for (i = 0; i < 5; i++) /* 读取40位数据 */
        {
            buf[i] = dht11_read_byte();
        }
 
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            *humi = buf[0];
            *temp = buf[2];
        }
    }
    else
    {
        return 1;
    }
    
    return 0;
}
 
/**
 * @brief       初始化DHT11
 * @param       无
 * @retval      0, 正常
 *              1, 不存在/不正常
 */
uint8_t dht11_init(void)
{
    gpio_config_t gpio_init_struct;
 
    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;             /* 失能引脚中断 */
    gpio_init_struct.mode = GPIO_MODE_INPUT_OUTPUT_OD;          /* 开漏模式的输入和输出 */
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;           /* 使能上拉 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;      /* 失能下拉 */
    gpio_init_struct.pin_bit_mask = 1ull << DHT11_DQ_GPIO_PIN;  /* 设置的引脚的位掩码 */
    gpio_config(&gpio_init_struct);                             /* 配置DHT11引脚 */
 
    dht11_reset();
    return dht11_check();
}
 

 /**
 ****************************************************************************************************
 * @file        dht11.h
 * @author      普中科技
 * @version     V1.0
 * @date        2024-03-01
 * @brief       DHT11驱动代码
 * @license     Copyright (c) 2024-2034, 深圳市普中科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:普中科技 ESP32-S3 开发板
 * 在线视频:https://space.bilibili.com/2146492485
 * 技术论坛:www.prechin.net
 * 公司网址:www.prechin.cn
 * 购买地址:
 *
 ****************************************************************************************************
 */
 
#ifndef __DHT11_H
#define __DHT11_H
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h" 
 
/* 引脚定义 */
#define DHT11_DQ_GPIO_PIN       GPIO_NUM_13
 
/* DHT11引脚高低电平枚举 */
typedef enum 
{
    DHT11_PIN_RESET = 0u,
    DHT11_PIN_SET
}DHT11_GPIO_PinState;
 
/* IO操作 */
#define DHT11_DQ_IN     gpio_get_level(DHT11_DQ_GPIO_PIN)   /* 数据端口输入 */
 
/* DHT11端口定义 */
#define DHT11_DQ_OUT(x) do{ x ?                                                 \
                            gpio_set_level(DHT11_DQ_GPIO_PIN, DHT11_PIN_SET) :  \
                            gpio_set_level(DHT11_DQ_GPIO_PIN, DHT11_PIN_RESET); \
                        }while(0)
 
/* 函数声明 */
void dht11_reset(void);                                 /* 复位DHT11 */
uint8_t dht11_init(void);                               /* 初始化DHT11 */
uint8_t dht11_check(void);                              /* 等待DHT11的回应 */
uint8_t dht11_read_data(uint8_t *temp,uint8_t *humi);   /* 读取温湿度 */
 
#endif

/**
 ****************************************************************************************************
 * @file        main.c
 * @author      普中科技
 * @version     V1.0
 * @date        2024-03-01
 * @brief       
 * @license     Copyright (c) 2024-2034, 深圳市普中科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:普中科技 ESP32-S3 开发板
 * 在线视频:https://space.bilibili.com/2146492485
 * 技术论坛:www.prechin.net
 * 公司网址:www.prechin.cn
 * 购买地址:
 *
 ****************************************************************************************************
 * 实验名称：DHT11温湿度传感器实验
 * 
 * 接线说明：DHT11温湿度传感器模块-->ESP32 IO
             (VCC)-->(3V3)
             (DATA)-->(13)
             (GND)-->(GND)
 * 
 * 实验现象：程序下载成功后，软件串口控制台间隔1S输出DHT11温湿度传感器采集的温度和湿度
 * 
 * 注意事项：
 * 
 *
 ****************************************************************************************************
 */
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "uart.h"
#include "led.h"
#include "dht11.h"
 
 
 
/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;
    uint8_t t = 0;
    uint8_t temperature;
    uint8_t humidity;
 
    ret = nvs_flash_init(); /* 初始化NVS */
 
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
 
	led_init();	/* 初始化LED */
    uart_init(115200);   /* 初始化串口 */
    /* 初始化DHT11数字温湿度传感器 */
    while(dht11_init())
	{
		printf("DHT11检测失败，请插好!\r\n");
		vTaskDelay(500);
	}
	printf("DHT11检测成功!\r\n");
     
    while (1) 
    {
        t++;
 
        if ((t % 100) == 0)              /* 每1s更新一次显示数据 */
        {
            dht11_read_data(&temperature, &humidity);   /* 读取温湿度值 */
			printf("\r\n温度：%d°C\r\n",temperature);
            printf("湿度：%d%%RH\r\n",humidity);
        }
 
        if ((t % 20) == 0)
        {
            LED_TOGGLE();               /* 每200ms,翻转一次LED */
        }
 
        vTaskDelay(10);  
    }
}
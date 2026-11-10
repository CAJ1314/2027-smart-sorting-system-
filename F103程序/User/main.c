#include "stm32f10x.h"
#include "Delay.h"
#include "Servo.h"
#include "BoardUART.h"
#include <string.h>

#define GATE_CLOSED_DEG 80U
#define GATE_RELEASE_DEG 162U /* 释放机构打开角度；上电待机和RELEASE释放都使用此角度 */
#define GATE_HOLD_MS 2000UL
#define GATE_SETTLE_MS 500UL
#define SENSOR_PERIOD_MS 10UL

/* Sensor 1..6: PB3, PB4, PB5, PB6, PB7, PB8; low = blocked. Pull-up input. */
static uint8_t ReadSensors(void)
{
    uint8_t mask = 0U;

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == Bit_RESET) mask |= 0x01U; /* 1 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET) mask |= 0x02U; /* 2 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_RESET) mask |= 0x04U; /* 3 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == Bit_RESET) mask |= 0x08U; /* 4 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == Bit_RESET) mask |= 0x10U; /* 5 */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == Bit_RESET) mask |= 0x20U; /* 6 */

    return mask;
}

static void Sensors_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    /* PB3/PB4 are JTAG pins; release them for GPIO while keeping SWD. */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                    GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
}

static void SendSensorMask(uint8_t mask)
{
    static const char hex[] = "0123456789ABCDEF";
    char line[] = "SENSOR:00\r\n";

    line[7] = hex[(mask >> 4) & 0x0FU];
    line[8] = hex[mask & 0x0FU];
    BoardUART_Send(line);
}

int main(void)
{
    char line[32];
    uint8_t sensor_mask;

    SystemCoreClockUpdate();
    Servo_Init();
    Servo_SetAngle(GATE_RELEASE_DEG);
    Sensors_Init();
    BoardUART_Init();

    while (1)
    {
        sensor_mask = ReadSensors();
        SendSensorMask(sensor_mask);

        if (BoardUART_ReadLine(line))
        {
            if (strcmp(line, "RELEASE") == 0)
            {
                BoardUART_Send("ACK\r\n");
                Servo_SetAngle(GATE_RELEASE_DEG);
                Delay_ms(GATE_HOLD_MS);
                Servo_SetAngle(GATE_CLOSED_DEG);
                Delay_ms(GATE_SETTLE_MS);
                /* Discard commands received while the gate was moving. */
                while (BoardUART_ReadLine(line)) {}
                BoardUART_Send("DONE\r\n");
            }
            else if (strcmp(line, "RESET") == 0)
            {
                BoardUART_Send("ACK\r\n");
                Servo_SetAngle(GATE_CLOSED_DEG);
                Delay_ms(GATE_SETTLE_MS);
                while (BoardUART_ReadLine(line)) {}
                BoardUART_Send("DONE\r\n");
            }
            else
            {
                BoardUART_Send("ERR\r\n");
            }
        }

        Delay_ms(SENSOR_PERIOD_MS);
    }
}



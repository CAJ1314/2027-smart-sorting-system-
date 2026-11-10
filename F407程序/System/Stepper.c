#include "Stepper.h"
#include "Delay.h"
#include "Sensor.h"

/* ================================================================
   TB6600步进驱动 — GPIO位带脉冲 + __disable_irq 保护时序
   ----------------------------------------------------------------
   • EN:  LOW  = 芯片使能(正常工作)
          HIGH = 芯片失能(电机脱机, 可自由转动)
   • PUL: 上升沿 → 电机走一步
   • DIR: LOW=CW顺时针  HIGH=CCW逆时针
   ----------------------------------------------------------------
   频率范围: 50Hz ~ 50000Hz
   ================================================================ */

/* ================================================================
   底层: 发指定数量脉冲 (不控制DIR, 调用者提前设好方向)
   ================================================================ */
   
            //最核心: 循环发高低电平
static void Stepper_Pulse(GPIO_TypeDef* PUL_PORT, uint16_t PUL_PIN,
                          uint32_t steps, uint16_t hz)
{
    uint16_t half;   /* 半周期 (us) */

    if (hz < 50)    hz = 50;
    if (hz > 50000) hz = 50000;

    half = 500000UL / hz;   /* T/2 = 500ms/hz = 500000us/hz */
    if (half < 10) half = 10;

    __disable_irq();
    for (uint32_t i = 0; i < steps; i++)
    {
        GPIO_SetBits(PUL_PORT, PUL_PIN);    /* PUL上升沿 */
        Delay_us(half);
        GPIO_ResetBits(PUL_PORT, PUL_PIN);  /* PUL下降沿 */
        Delay_us(half);
    }
    __enable_irq();
}

/* ================================================================
   通用: 按步数 + 方向运行
   steps > 0 → CW, steps < 0 → CCW
   ================================================================ */


            //设定方向 + 调发脉冲
static void Stepper_Run(GPIO_TypeDef* PUL_PORT, uint16_t PUL_PIN,
                        GPIO_TypeDef* DIR_PORT, uint16_t DIR_PIN,
                        int32_t steps, uint16_t hz)
{
    if (steps >= 0)
        GPIO_ResetBits(DIR_PORT, DIR_PIN);   /* CW */
    else {
        GPIO_SetBits(DIR_PORT, DIR_PIN);     /* CCW */
        steps = -steps;
    }

    /* DIR建立时间: DIR改变后先等待1ms再发STEP,
       这里给1ms确保稳定 */
    Delay_ms(1);

    Stepper_Pulse(PUL_PORT, PUL_PIN, (uint32_t)steps, hz);
}

/* ================================================================
   通用: 初始化一组 TB6600 引脚
   ================================================================ */


            //把引脚配置成输出
static void Stepper_InitPins(GPIO_TypeDef* PUL_PORT, uint16_t PUL_PIN,
                             GPIO_TypeDef* DIR_PORT, uint16_t DIR_PIN,
                             GPIO_TypeDef* EN_PORT,  uint16_t EN_PIN)
{
    GPIO_InitTypeDef g;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    /* PUL 引脚 */
    g.GPIO_Pin = PUL_PIN;
    GPIO_Init(PUL_PORT, &g);

    /* DIR 引脚 (可能在不同端口) */
    g.GPIO_Pin = DIR_PIN;
    GPIO_Init(DIR_PORT, &g);

    /* EN 引脚 (可能在不同端口) */
    g.GPIO_Pin = EN_PIN;
    GPIO_Init(EN_PORT, &g);

    /* 初始状态: PUL=L, DIR=L(CW), EN=L(使能) */
    GPIO_ResetBits(PUL_PORT, PUL_PIN);
    GPIO_ResetBits(DIR_PORT, DIR_PIN);
    GPIO_ResetBits(EN_PORT,  EN_PIN);    /* EN=L → 芯片使能 */
}

/* ############################################################## */
/*                        下层转盘 (盒子输送)                       */
/* ############################################################## */

/**
  * @brief  初始化下层转盘 TB6600: PB0=PUL PE0=DIR PB2=EN
  */
void Stepper_Lower_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE, ENABLE);

    Stepper_InitPins(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN,
                     Stepper_LOWER_DIR_PORT, Stepper_LOWER_DIR_PIN,
                     Stepper_LOWER_EN_PORT,  Stepper_LOWER_EN_PIN);
}

/**
  * @brief  下层转盘转动指定步数
  * @param  steps  正数=CW(顺时针), 负数=CCW
  * @param  hz     脉冲频率 (Hz), 50~50000
  */
void Stepper_Lower_Run(int32_t steps, uint16_t hz)
{
    Stepper_Run(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN,
                Stepper_LOWER_DIR_PORT, Stepper_LOWER_DIR_PIN,
                steps, hz);
}

/**
  * @brief  下层转盘顺时针转一整圈
  * @param  hz 脉冲频率, 如 6680=约5秒一圈(需按实际步数算)
  */
void Stepper_Lower_RunOneRev(uint16_t hz)
{
    Stepper_Lower_Run(Stepper_STEPS_PER_REV, hz);
}

/**
  * @brief  下层转盘正转(CW), 直到指定传感器被遮挡时停止
  * @param  sensorIndex 1~6, 对应Sensor_Get()的编号
  * @param  hz         脉冲频率 (Hz)
  * @retval 1=传感器触发停止, 0=未触发(不会发生,除非传感器故障)
  */
uint8_t Stepper_Lower_RunUntilSensor(uint8_t sensorIndex, uint16_t hz)
{
    uint16_t half;

    if (hz < 50)    hz = 50;
    if (hz > 50000) hz = 50000;

    half = 500000UL / hz;
    if (half < 10) half = 10;

    GPIO_ResetBits(Stepper_LOWER_DIR_PORT, Stepper_LOWER_DIR_PIN); /* CW */
    Delay_ms(1);

    __disable_irq();
    while (Sensor_Get(sensorIndex) != 0)
    {
        GPIO_SetBits(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN);
        Delay_us(half);
        GPIO_ResetBits(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN);
        Delay_us(half);
    }
    uint8_t sensorStopped = (Sensor_Get(sensorIndex) == 0);
    __enable_irq();

    return (uint8_t)(sensorStopped);
}

/**
  * @brief  下层转盘正转(CW), 直到传感器触发或超时
  * @param  sensorIndex 1~6
  * @param  hz         脉冲频率 (Hz)
  * @param  maxSteps   最大步数 (超时保护, 防传感器故障死转)
  * @retval 1=传感器触发停止, 0=超时
  */
uint8_t Stepper_Lower_RunUntilSensorTimeout(uint8_t sensorIndex, uint16_t hz, uint32_t maxSteps)
{
    uint32_t step = 0;
    uint16_t half;

    if (hz < 50)    hz = 50;
    if (hz > 50000) hz = 50000;

    half = 500000UL / hz;
    if (half < 10) half = 10;

    GPIO_ResetBits(Stepper_LOWER_DIR_PORT, Stepper_LOWER_DIR_PIN); /* CW */
    Delay_ms(1);

    __disable_irq();
    while (Sensor_Get(sensorIndex) != 0 && step < maxSteps)
    {
        GPIO_SetBits(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN);
        Delay_us(half);
        GPIO_ResetBits(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN);
        Delay_us(half);
        step++;
    }
    uint8_t sensorStopped = (Sensor_Get(sensorIndex) == 0);
    __enable_irq();

    return (uint8_t)(sensorStopped);
}

/**
  * @brief  停止下层转盘 (PUL=L)
  */
void Stepper_Lower_Stop(void)
{
    GPIO_ResetBits(Stepper_LOWER_PUL_PORT, Stepper_LOWER_PUL_PIN);
}


/**
  * @brief  下层转盘复位：顺时针运行，直到1号复位传感器被遮挡
  * @param  hz       脉冲频率
  * @param  maxSteps 最大步数，防止传感器故障时持续转动
  * @retval 1=复位传感器触发，0=达到最大步数
  */
uint8_t Stepper_Lower_Reset(uint16_t hz, uint32_t maxSteps)
{
    return Stepper_Lower_RunUntilSensorTimeout(1, hz, maxSteps);
}/* ############################################################## */
/*                      上层转盘 (存料区送料)                       */
/* ############################################################## */

/**
  * @brief  初始化上层转盘 TB6600: PB5=PUL PB6=DIR PB7=EN
  */
void Stepper_Upper_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    Stepper_InitPins(Stepper_UPPER_PUL_PORT, Stepper_UPPER_PUL_PIN,
                     Stepper_UPPER_DIR_PORT, Stepper_UPPER_DIR_PIN,
                     Stepper_UPPER_EN_PORT,  Stepper_UPPER_EN_PIN);
}

/**
  * @brief  上层转盘转动指定步数
  */
void Stepper_Upper_Run(int32_t steps, uint16_t hz)
{
    Stepper_Run(Stepper_UPPER_PUL_PORT, Stepper_UPPER_PUL_PIN,
                Stepper_UPPER_DIR_PORT, Stepper_UPPER_DIR_PIN,
                steps, hz);
}

/**
  * @brief  上层转盘按角度转动
  * @param  degrees  角度 (0~360)
  * @param  hz       脉冲频率
  */
void Stepper_Upper_RotateDeg(uint16_t degrees, uint16_t hz)
{
    int32_t steps = (int32_t)Stepper_STEPS_PER_REV * degrees / 360;
    if (steps > 0)
        Stepper_Upper_Run(steps, hz);
}

/**
  * @brief  停止上层转盘
  */
void Stepper_Upper_Stop(void)
{
    GPIO_ResetBits(Stepper_UPPER_PUL_PORT, Stepper_UPPER_PUL_PIN);
}






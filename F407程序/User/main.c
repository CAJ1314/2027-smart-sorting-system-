#include "stm32f4xx.h"
#include "Delay.h"
#include "TB6600.h"
#include "ObjectUART.h"
#include "VisionUART.h"
#include "ZStepper.h"
#include "GripperServo.h"
#include "BoardUART.h"
#include "ScreenTable.h"

#define Z_START_HEIGHT_RAISE_STEPS 427L /* 夹爪起始高度上调1cm；按2347步≈5.5cm换算，实际需试运行确认 */
#define Z_START_HEIGHT_RAISE_MS 1281UL /* 起点上调对应的下降时间：427步×3ms/步 */
#define Z_CUBE_DOWN_STEPS (640L + Z_START_HEIGHT_RAISE_STEPS) /* 正方体下探到原抓取高度：约2.5cm */
#define Z_OTHER_DOWN_STEPS (1707L + Z_START_HEIGHT_RAISE_STEPS) /* 其他方块下探到原抓取高度：约5cm */
#define Z_OTHER_STAGE1_DOWN_STEPS 490L /* 其他物块先下降约0.7cm */
#define Z_OTHER_STAGE2_DOWN_STEPS (Z_OTHER_DOWN_STEPS - Z_OTHER_STAGE1_DOWN_STEPS) /* 再下降到原有最终深度，约4.3cm */
#define Z_OTHER_AFTER_OPEN_DOWN_STEPS 490L /* 张开后先下降7mm，再开始末段夹取 */
#define Z_OTHER_FINAL_DESCENT_STEPS (Z_OTHER_STAGE2_DOWN_STEPS - Z_OTHER_AFTER_OPEN_DOWN_STEPS)
#define Z_CYLINDER_DOWN_STEPS (1280L + Z_START_HEIGHT_RAISE_STEPS) /* 圆柱从新起点下探约4cm */
#define Z_CUBE_LIFT_STEPS 2134L /* 正方体抓取后上升5cm；起点上调后约在新起点上方2.5cm */
#define Z_CUBE_RETURN_STEPS 1067L /* 正方体释放后下降约2.5cm，回到新起始高度 */
#define Z_OTHER_LIFT_STEPS 2000L /* 其他方块抓取后最多上升5cm，回到新起始高度 */
#define Z_CYLINDER_LIFT_STEPS 2134L /* 圆柱抓取后最多上升5cm，至新起点上方约1cm */
#define Z_CYLINDER_RETURN_STEPS 427L /* 圆柱下探4cm、上升5cm后，释放后下降约1cm回到新起点 */
#define Z_OTHER_RETURN_STEPS 0L /* 其他方块已升回新起始高度，无需再下降 */
#define Z_STEP_INTERVAL_US 3000U /* 恢复原Z轴下降速度：3ms/步 */
#define Z_LIFT_INTERVAL_US 1000U /* 恢复原Z轴上升速度：1ms/步 */
#define Z_RETURN_INTERVAL_US 1500U /* 正方体/圆柱上升5cm后的Z轴回位速度提高2倍 */
#define Z_REVERSE_PAUSE_MS 200U /* 夹爪闭合完成后、Z轴上升前的停顿(ms) */
#define Z_CUBE_CLOSE_TIME_MS    300UL /* 正方体夹爪闭合到最终角度所用时间(ms) */
#define Z_CUBE_PRE_MOVE_STEPS 641L /* 正方体预动作上下1.5cm，按427步/cm取整 */
#define Z_CUBE_PRE_CLOSE_ANGLE_DEG (GRIPPER_OPEN_ANGLE_DEG - 17U) /* 正方体预夹紧17° */
#define Z_CYLINDER_LIFT_PRE_CLOSE_ANGLE_DEG (Z_CUBE_PRE_CLOSE_ANGLE_DEG + 4U) /* 圆柱上升1.5cm后比正方体预收拢角度增加4° */
#define Z_CUBE_PRE_CLOSE_TIME_MS 100UL /* 正方体预夹紧/张开动作约100ms */
#define Z_CYLINDER_PRE_CLOSE_ANGLE_DEG (GRIPPER_OPEN_ANGLE_DEG - 3U) /* 圆柱下探前先从170°收拢到167° */
#define Z_CYLINDER_PRE_CLOSE_TIME_MS 300UL /* 圆柱下探前预收拢时间(ms) */
#define Z_CYLINDER_CLOSE_TIME_MS 300UL /* 圆柱下探4cm后闭合到最终角度所用时间(ms) */
#define Z_OTHER_STAGE1_CLOSE_ANGLE_DEG (GRIPPER_OPEN_ANGLE_DEG - 35U) /* 其他物块立即从170°闭合到135° */
#define Z_OTHER_STAGE1_CLOSE_TIME_MS 300UL /* 快速闭合35°所用时间 */
#define Z_OTHER_STAGE2_OPEN_ANGLE_DEG (Z_OTHER_STAGE1_CLOSE_ANGLE_DEG + 35U) /* 下降0.7cm后立即打开35°回到170° */
#define Z_OTHER_STAGE2_OPEN_TIME_MS 300UL /* 快速打开35°所用时间 */
#define Z_OTHER_FINAL_CLOSE_REFERENCE_INTERVAL_MS 3UL /* 夹爪末段闭合保持原速度基准(ms/步) */
#define Z_OTHER_FINAL_CLOSE_TIME_MS 4100UL /* 恢复提速前的末段闭合用时 */

static uint8_t zCurrentPickIsCube;
static uint8_t zCurrentPickIsCylinder;

static volatile uint8_t lowerPreturnActive;
static volatile uint8_t lowerPreturnError;
static volatile uint8_t lowerPreturnConfirming;
static uint8_t lowerPreturnTarget;
static uint8_t lowerPreturnSensorMask;
static uint8_t lowerPreturnConfirmMs;
static uint16_t lowerPreturnNoStateMs;
static uint32_t lowerPreturnTotalSteps;
static uint32_t lowerPreturnLastSteps;
static uint8_t *lowerPreturnGate;

#define HOME_SPEED_HZ       14000U /* XY回零请求脉冲频率(Hz)；底层限幅12000Hz */
#define HOME_TIMEOUT_MS     180000UL /* XY回零超时(ms)；180000ms=180秒 */
#define MOVE_SPEED_HZ       13000U /* XY移动请求脉冲频率(Hz)；底层限幅14000Hz */
#define STEPS_PER_MM_X_NUM  15686L /* X轴每毫米脉冲数的分子；与下面分母配合，当前156.86脉冲/mm */
#define STEPS_PER_MM_X_DEN  100L /* X轴步距换算分母；每毫米脉冲数=分子/分母 */
#define STEPS_PER_MM_Y_NUM  15686L /* Y轴每毫米脉冲数的分子；与下面分母配合，当前156.86脉冲/mm */
#define STEPS_PER_MM_Y_DEN  100L /* Y轴步距换算分母；每毫米脉冲数=分子/分母 */
#define LOWER_HOME_HALF_US         93U /* 下层转盘复位/分类脉冲半周期(us)；恢复此前速度 */
#define LOWER_HOME_MAX_STEPS        70000UL /* 下层转盘复位最多输出的脉冲数；防止找不到传感器时持续转动 */
#define LOWER_HOME_CONFIRM_MS       30UL /* 目标光电门被遮挡后的稳定确认时间(ms) */
#define LOWER_HOME_STARTUP_TIMEOUT_MS 2000UL /* 启动复位时等待F103有效传感器状态的最长时间(ms) */
#define LOWER_HOME_STATE_TIMEOUT_MS   500UL /* 转盘运行中允许无有效传感器更新的最长时间(ms) */
/* Lower turntable startup mode:
 *   LOWER_MOTOR_TEST_ONE_REV: temporary one-revolution test.
 *   LOWER_SENSOR_SEQUENCE_TEST: optional 1..6 calibration sequence.
 *   Set both to 0 for normal power-on homing to sensor 1.
 */
#define LOWER_MOTOR_TEST_ONE_REV 0U /* 单圈电机测试开关：0关闭、1启用；正常分类保持0 */
#define LOWER_MOTOR_TEST_STEPS   6400UL /* 单圈测试脉冲数；1.8度电机32细分时6400脉冲/圈，不含减速比 */
#define LOWER_MOTOR_TEST_HALF_US 500UL /* 单圈测试脉冲半周期(us)；500us约为1000Hz */
#define LOWER_MOTOR_TEST_DIR_LEVEL 1U /* 单圈测试方向电平：0或1；只影响测试模式 */
#define LOWER_SENSOR_SEQUENCE_TEST 0U /* 光电门1至6依次停靠测试：0关闭、1启用；正常分类保持0 */
#define LOWER_SEQUENCE_DIR_LEVEL TB6600_LOWER_HOME_DIR_LEVEL /* 光电门顺序测试的转动方向；沿用底层复位方向定义 */
#define LOWER_SEQUENCE_STOP_MS 1000UL /* 顺序测试到达每个光电门后的停留时间(ms) */
#define LOWER_SEQUENCE_MAX_STEPS_PER_TARGET 20000UL /* 顺序测试寻找单个目标光电门的最大脉冲数 */
#define PLACE_X_CENTI_MM 0L /* 物料释放点X绝对坐标，单位0.01mm；16500=165mm */
#define PLACE_Y_CENTI_MM 12000L /* 物料释放点Y绝对坐标，单位0.01mm；10140=101.40mm */
#define STANDBY_X_CENTI_MM 6500L /* 识别等待待机点X坐标：距零点6.5cm */
#define STANDBY_Y_CENTI_MM 0L /* 识别等待待机点Y坐标：回到Y零点 */
#define VISION_SHAPE_TIMEOUT_MS 10000UL /* 第二摄像头识别结果等待超时(ms) */
#define VISION_START_AFTER_RELEASE_MS 500UL /* 夹爪完全松开后延迟多久请求第二摄像头识别(ms)，可与移动到待机点重叠 */
#define LOWER_CLASSIFY_MAX_STEPS 70000UL /* 分类寻找目标盒子时允许的最大转盘脉冲数 */
#define SHAPE_GATE_COUNT 6U /* 分类盒子数量；当前固定6类，勿只改单值，需同步数组和屏幕设计 */
static int32_t MmToSteps(int32_t centi_mm, int32_t numerator, int32_t denominator)
{
    int32_t steps;
    steps = ((int64_t)centi_mm * numerator + denominator * 50L) / (denominator * 100L);
    return steps;
}

/* A category is (color, shape): color * 8 + shape. */
static uint8_t Shape_Gate[SHAPE_GATE_COUNT] = {0U, 0U, 0U, 0U, 0U, 0U};

static uint8_t VisionCategory(void)
{
    return (uint8_t)(Vision_Color * 8U + Vision_Shape);
}

static uint8_t CategoryToTargetGate(uint8_t category)
{
    uint8_t i;

    for (i = 0U; i < SHAPE_GATE_COUNT; ++i)
    {
        if (Shape_Gate[i] == category) return (uint8_t)(i + 1U);
    }

    for (i = 0U; i < SHAPE_GATE_COUNT; ++i)
    {
        if (Shape_Gate[i] == 0U)
        {
            Shape_Gate[i] = category;
            return (uint8_t)(i + 1U);
        }
    }

    return 0U;
}

#if !LOWER_MOTOR_TEST_ONE_REV && !LOWER_SENSOR_SEQUENCE_TEST
/*
 * Return values:
 *   0 = sensor 1 reached and stable
 *   1 = step limit reached
 *   2 = no valid F103 sensor status
 */
static uint8_t LowerTurntable_Home(void)
{
    uint8_t sensor_mask = 0U;
    uint8_t sensor_seen = 0U;
    uint8_t confirm_ok;
    uint32_t steps = 0U;
    uint32_t startup_ms;
    uint32_t no_state_ms = 0U;
    uint32_t confirm_ms;
    char line[32];

    TB6600_LowerInit();
    TB6600_LowerSetDirection(TB6600_LOWER_HOME_DIR_LEVEL);
    TB6600_LowerStop();
    while (BoardUART_ReadLine(line)) {}

    /* Common-cathode wiring is enabled by a high level on EN by default. */
    TB6600_LowerEnable(1U);
    Delay_ms(100);

    /* Allow the F103 to finish booting before requiring sensor telemetry. */
    for (startup_ms = 0U; startup_ms < LOWER_HOME_STARTUP_TIMEOUT_MS; ++startup_ms)
    {
        if (BoardUART_PollSensor(&sensor_mask))
        {
            sensor_seen = 1U;
            break;
        }
        Delay_ms(1);
    }

    if (!sensor_seen)
    {
        TB6600_LowerStop();
        return 2U;
    }

    no_state_ms = 0U;
    while (steps < LOWER_HOME_MAX_STEPS)
    {
        /* Poll UART independently from the step pulse so homing runs at 1 kHz. */
        if (BoardUART_PollSensor(&sensor_mask))
        {
            no_state_ms = 0U;
        }
        else if (++no_state_ms >= LOWER_HOME_STATE_TIMEOUT_MS)
        {
            TB6600_LowerStop();
            return 2U;
        }

        if ((sensor_mask & 0x01U) != 0U)
        {
            TB6600_LowerStop();
            confirm_ok = 1U;
            for (confirm_ms = 0U; confirm_ms < LOWER_HOME_CONFIRM_MS; ++confirm_ms)
            {
                if (BoardUART_PollSensor(&sensor_mask))
                {
                    no_state_ms = 0U;
                    if ((sensor_mask & 0x01U) == 0U)
                    {
                        confirm_ok = 0U;
                        break;
                    }
                }
                else if (++no_state_ms >= LOWER_HOME_STATE_TIMEOUT_MS)
                {
                    TB6600_LowerStop();
                    return 2U;
                }
                Delay_ms(1);
            }
            if (confirm_ok)
            {
                TB6600_LowerStop();
                return 0U;
            }
            no_state_ms = 0U;
            continue;
        }

        TB6600_LowerStep(LOWER_HOME_HALF_US);
        ++steps;
    }

    TB6600_LowerStop();
    return 1U;
}
#endif

#if LOWER_MOTOR_TEST_ONE_REV
static void LowerTurntable_TestOneRev(void)
{
    uint32_t step;

    TB6600_LowerInit();
    TB6600_LowerSetDirection(LOWER_MOTOR_TEST_DIR_LEVEL);
    TB6600_LowerStop();
    TB6600_LowerEnable(1U);
    Delay_ms(100);

    for (step = 0U; step < LOWER_MOTOR_TEST_STEPS; ++step)
    {
        TB6600_LowerStep(LOWER_MOTOR_TEST_HALF_US);
    }

    TB6600_LowerStop();
    TB6600_LowerEnable(0U);
}
#endif

static uint8_t LowerTurntable_FindSensor(uint8_t sensor_index, uint32_t max_steps,
                                         uint8_t *sensor_mask)
{
    uint8_t confirm_ok;
    uint16_t target_bit;
    uint32_t steps = 0U;
    uint32_t no_state_ms = 0U;
    uint32_t confirm_ms;

    if (sensor_index >= 6U || sensor_mask == 0) return 1U;
    target_bit = (uint16_t)(1U << sensor_index);

    while (steps < max_steps)
    {
        if (BoardUART_PollSensor(sensor_mask))
        {
            no_state_ms = 0U;
        }
        else if (++no_state_ms >= LOWER_HOME_STATE_TIMEOUT_MS)
        {
            TB6600_LowerStop();
            return 2U;
        }

        if ((*sensor_mask & target_bit) != 0U)
        {
            TB6600_LowerStop();
            confirm_ok = 1U;
            for (confirm_ms = 0U; confirm_ms < LOWER_HOME_CONFIRM_MS; ++confirm_ms)
            {
                if (BoardUART_PollSensor(sensor_mask))
                {
                    no_state_ms = 0U;
                    if ((*sensor_mask & target_bit) == 0U)
                    {
                        confirm_ok = 0U;
                        break;
                    }
                }
                else if (++no_state_ms >= LOWER_HOME_STATE_TIMEOUT_MS)
                {
                    TB6600_LowerStop();
                    return 2U;
                }
                Delay_ms(1);
            }

            if (confirm_ok)
            {
                TB6600_LowerStop();
                return 0U;
            }
            no_state_ms = 0U;
            continue;
        }

        TB6600_LowerStep(LOWER_HOME_HALF_US);
        ++steps;
    }

    TB6600_LowerStop();
    return 1U;
}

static void LowerTurntable_Service1ms(void)
{
    uint8_t target_bit;
    uint32_t steps;
    uint8_t got_sensor;

    if (lowerPreturnActive == 0U) return;
    if (!TB6600_RunPressed())
    {
        TB6600_LowerAsyncStop();
        lowerPreturnError = 3U;
        lowerPreturnActive = 0U;
        return;
    }

    got_sensor = BoardUART_PollSensor(&lowerPreturnSensorMask);
    if (got_sensor) lowerPreturnNoStateMs = 0U;
    else if (++lowerPreturnNoStateMs >= LOWER_HOME_STATE_TIMEOUT_MS)
    {
        TB6600_LowerAsyncStop();
        lowerPreturnError = 2U;
        lowerPreturnActive = 0U;
        return;
    }

    steps = TB6600_LowerAsyncSteps();
    lowerPreturnTotalSteps += steps - lowerPreturnLastSteps;
    lowerPreturnLastSteps = steps;
    if (lowerPreturnTotalSteps >= LOWER_CLASSIFY_MAX_STEPS)
    {
        TB6600_LowerAsyncStop();
        lowerPreturnError = 1U;
        lowerPreturnActive = 0U;
        return;
    }

    target_bit = (uint8_t)(1U << (lowerPreturnTarget - 1U));
    if ((lowerPreturnSensorMask & target_bit) != 0U)
    {
        if (lowerPreturnConfirming == 0U)
        {
            TB6600_LowerAsyncStop();
            lowerPreturnLastSteps = 0U;
            lowerPreturnConfirming = 1U;
            lowerPreturnConfirmMs = 0U;
        }
        if (++lowerPreturnConfirmMs >= LOWER_HOME_CONFIRM_MS)
        {
            if (lowerPreturnGate != 0) *lowerPreturnGate = lowerPreturnTarget;
            lowerPreturnError = 0U;
            lowerPreturnActive = 0U;
        }
    }
    else if (lowerPreturnConfirming != 0U)
    {
        lowerPreturnConfirming = 0U;
        lowerPreturnConfirmMs = 0U;
        lowerPreturnLastSteps = 0U;
        TB6600_LowerAsyncStart((uint8_t)!TB6600_LOWER_HOME_DIR_LEVEL,
                               (uint16_t)LOWER_HOME_HALF_US);
    }
}

static uint8_t LowerTurntable_WaitPreturn(void)
{
    uint8_t result;
    while (lowerPreturnActive != 0U)
    {
        LowerTurntable_Service1ms();
        Delay_ms(1);
    }
    result = lowerPreturnError;
    lowerPreturnError = 0U;
    return result;
}

static uint8_t LowerTurntable_MoveToSensor(uint8_t target, uint8_t *gate)
{
    uint8_t mask = 0U, direction;
    uint8_t forward, reverse;
    uint32_t wait;
    uint8_t current;
    uint8_t pending_error = LowerTurntable_WaitPreturn();
    if (pending_error != 0U) return pending_error;
    if (gate == 0) return 1U;
    current = *gate;
    if(target < 1U || target > 6U || current < 1U || current > 6U) return 1U;
    for(wait=0U; wait<LOWER_HOME_STATE_TIMEOUT_MS; ++wait)
    { if(BoardUART_PollSensor(&mask)) break; Delay_ms(1); }
    if(wait>=LOWER_HOME_STATE_TIMEOUT_MS) return 2U;
    if(target==current) return 0U;
    /* Sequence-test order shows home direction steps 6,5,...,1. */
    forward=(uint8_t)((target+6U-current)%6U);
    reverse=(uint8_t)((current+6U-target)%6U);
    direction=(forward<=reverse) ? TB6600_LOWER_HOME_DIR_LEVEL : (uint8_t)!TB6600_LOWER_HOME_DIR_LEVEL;
    TB6600_LowerStop();
    TB6600_LowerSetDirection(direction);
    TB6600_LowerEnable(1U);
    Delay_ms(1);
    pending_error = LowerTurntable_FindSensor((uint8_t)(target-1U), LOWER_CLASSIFY_MAX_STEPS, &mask);
    if (pending_error == 0U) *gate = target;
    return pending_error;
}

static uint8_t LowerTurntable_PreturnNext(uint8_t *gate)
{
    uint8_t next, forward, reverse, direction;
    uint8_t pending_error;
    if (gate == 0) return 1U;
    pending_error = LowerTurntable_WaitPreturn();
    if (pending_error != 0U) return pending_error;
    if (*gate < 1U || *gate > 6U) return 1U;
    next = (*gate >= 6U) ? 1U : (uint8_t)(*gate + 1U);
    forward = (uint8_t)((next + 6U - *gate) % 6U);
    reverse = (uint8_t)((*gate + 6U - next) % 6U);
    direction = (forward <= reverse) ? TB6600_LOWER_HOME_DIR_LEVEL
                                     : (uint8_t)!TB6600_LOWER_HOME_DIR_LEVEL;
    lowerPreturnTarget = next;
    lowerPreturnGate = gate;
    lowerPreturnSensorMask = 0U;
    lowerPreturnConfirming = 0U;
    lowerPreturnConfirmMs = 0U;
    lowerPreturnNoStateMs = 0U;
    lowerPreturnTotalSteps = 0U;
    lowerPreturnLastSteps = 0U;
    lowerPreturnError = 0U;
    lowerPreturnActive = 1U;
    TB6600_LowerAsyncStart(direction, (uint16_t)LOWER_HOME_HALF_US);
    return 0U;
}

static void LowerTurntable_ServicePump(void)
{
    LowerTurntable_Service1ms();
}

static void LowerTurntable_ServiceZ(uint32_t elapsed_ms)
{
    (void)elapsed_ms;
    LowerTurntable_Service1ms();
}

#if LOWER_SENSOR_SEQUENCE_TEST
static uint8_t LowerTurntable_Sequence1To6(void)
{
    uint8_t target, error;
    uint8_t sensor_mask = 0U;
    uint32_t startup_ms;
    char line[32];

    TB6600_LowerInit();
    TB6600_LowerSetDirection(LOWER_SEQUENCE_DIR_LEVEL);
    TB6600_LowerStop();
    while (BoardUART_ReadLine(line)) {}

    TB6600_LowerEnable(1U);
    Delay_ms(100);

    /* Wait for the first valid F103 sensor report after F103 power-up. */
    for (startup_ms = 0U; startup_ms < LOWER_HOME_STARTUP_TIMEOUT_MS; ++startup_ms)
    {
        if (BoardUART_PollSensor(&sensor_mask)) break;
        Delay_ms(1);
    }
    if (startup_ms >= LOWER_HOME_STARTUP_TIMEOUT_MS)
    {
        TB6600_LowerStop();
        return 2U;
    }

    /* Keep known-good direction; logical 1..6 maps to sensor bits 6..1. */
    for (target = 0U; target < 6U; ++target)
    {
        error = LowerTurntable_FindSensor((uint8_t)(5U - target),
                                          LOWER_SEQUENCE_MAX_STEPS_PER_TARGET, &sensor_mask);
        if (error != 0U) return error;
        Delay_ms(LOWER_SEQUENCE_STOP_MS);
    }

    /* Return to sensor 1 so the following normal flow still starts from home. */
    error = LowerTurntable_FindSensor(0U, LOWER_SEQUENCE_MAX_STEPS_PER_TARGET, &sensor_mask);
    return error;
}
#endif

/* Start the second camera 3 s after release. The standby travel and the
 * standby wait both call this service about once per millisecond; the same
 * callback also advances the asynchronous Z return without changing its path.
 */
static uint8_t visionPumpStarted;
static uint8_t visionPumpDone;
static uint8_t visionPumpStatus;
static uint32_t visionPumpElapsedMs;
static uint32_t visionPumpWaitMs;

static void VisionPumpDuringHome(void)
{
    ZStepper_Service1ms();
    LowerTurntable_Service1ms();
    if (visionPumpDone) return;

    if (!visionPumpStarted)
    {
        ++visionPumpElapsedMs;
        if (visionPumpElapsedMs < VISION_START_AFTER_RELEASE_MS) return;

        visionPumpStatus = VisionUART_BeginRequest();
        if (visionPumpStatus == VISION_UART_OK)
        {
            visionPumpStarted = 1U;
            visionPumpWaitMs = 0U;
        }
        else
        {
            visionPumpDone = 1U;
        }
        return;
    }

    visionPumpStatus = VisionUART_Poll();
    if (visionPumpStatus != 0U)
    {
        visionPumpDone = 1U;
    }
    else if (++visionPumpWaitMs >= VISION_SHAPE_TIMEOUT_MS)
    {
        visionPumpStatus = VISION_UART_TIMEOUT;
        visionPumpDone = 1U;
    }
}
int main(void)
{
    uint8_t homeError, moveError;
    uint8_t visionTarget, visionCategory, classifyError, visionStatus;
    uint8_t firstRelease, releaseError;
    uint8_t currentGate;
    uint8_t i;
#if LOWER_SENSOR_SEQUENCE_TEST
    uint8_t lowerSequenceError;
#elif !LOWER_MOTOR_TEST_ONE_REV
    uint8_t lowerHomeError;
#endif
    int32_t x_steps, y_steps;

    /* The lower turntable is homed to sensor 1 at power-on, so box 1 is active. */
    currentGate = 1U;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000000U);
    TB6600_Init();
    TB6600_StopBoth();
    ObjectUART_Init();
    VisionUART_Init();
    ZStepper_Init();
    GripperServo_Init();
    BoardUART_Init();
    ScreenTable_Init();
    ScreenTable_DrawFrame();

#if LOWER_MOTOR_TEST_ONE_REV
    /* Temporary test: run the lower motor exactly one revolution at power-on. */
    LowerTurntable_TestOneRev();
#elif LOWER_SENSOR_SEQUENCE_TEST
    /* Optional test/calibration: stop at sensors 1..6, then return to sensor 1. */
    lowerSequenceError = LowerTurntable_Sequence1To6();
    if (lowerSequenceError != 0U)
    {
        TB6600_LowerStop();
        TB6600_LowerEnable(0U);
        TB6600_StopBoth();
        while (1) { Delay_ms(1000); }
    }
#endif

#if !LOWER_MOTOR_TEST_ONE_REV && !LOWER_SENSOR_SEQUENCE_TEST
    /* Power-on step: home the lower turntable to sensor 1 before any key press. */
    lowerHomeError = LowerTurntable_Home();
    if (lowerHomeError != 0U)
    {
        TB6600_LowerStop();
        TB6600_LowerEnable(0U);
        TB6600_StopBoth();
        while (1) { Delay_ms(1000); }
    }
#endif

    /* Home XY once at power-on so a start press begins recognition directly. */
    homeError = TB6600_HomeBoth(HOME_SPEED_HZ, HOME_TIMEOUT_MS);
    if (homeError != 0U)
    {
        TB6600_StopBoth();
        while (1) { Delay_ms(1000); }
    }

    /* PC7: NO to GND. Released=high, pressed=low. */
    while (TB6600_RunPressed()) { Delay_ms(1); }
    while (1)
    {
        while (!TB6600_RunPressed()) { Delay_ms(1); }
        Delay_ms(20);
        if (!TB6600_RunPressed()) { continue; }
        /* Reset the release gate after the start key is accepted. */
        if (BoardUART_RequestGateReset(3000UL) != 1U)
        {
            TB6600_StopBoth();
            while (TB6600_RunPressed()) { Delay_ms(1); }
            continue;
        }

        if (TB6600_RunPressed())
        {
            for (i = 0U; i < SHAPE_GATE_COUNT; ++i)
            {
                Shape_Gate[i] = 0U;
            }
            firstRelease = 1U;
            ScreenTable_ResetData();

            /* Batch loop: stop when the first camera has no more material. */
            while (TB6600_RunPressed())
            {
                if (ObjectUART_RequestPump(15000UL, LowerTurntable_ServicePump) != OBJECT_UART_OK ||
                    !ObjectUART_IsValid() || !TB6600_RunPressed())
                {
                    break;
                }

                /* First camera position: move to the object and pick it. */
                x_steps = MmToSteps(Object_X_centi_mm, STEPS_PER_MM_X_NUM, STEPS_PER_MM_X_DEN);
                y_steps = MmToSteps(Object_Y_centi_mm, STEPS_PER_MM_Y_NUM, STEPS_PER_MM_Y_DEN);
                moveError = TB6600_MovePump(x_steps, y_steps, MOVE_SPEED_HZ, MOVE_SPEED_HZ,
                                            LowerTurntable_ServicePump);
                if (moveError != 0U) break;

                /* Descend the selected travel; other shapes use the staged grip sequence. */
                TB6600_StopBoth();
                zCurrentPickIsCube = Object_IsCube; /* 按本次第一摄像头识别形状选择抓取方案 */
                zCurrentPickIsCylinder = Object_IsCylinder; /* 圆柱使用独立抓取参数 */
                if (zCurrentPickIsCube)
                {
                    /* 正方体预动作：抬高1.5cm、夹紧17°、下降回原高度并张开。 */
                    ZStepper_RunWithService(-Z_CUBE_PRE_MOVE_STEPS, Z_LIFT_INTERVAL_US, LowerTurntable_ServiceZ);
                    GripperServo_MoveTimedPump(Z_CUBE_PRE_CLOSE_ANGLE_DEG, Z_CUBE_PRE_CLOSE_TIME_MS, LowerTurntable_ServicePump);
                    ZStepper_RunWithService(Z_CUBE_PRE_MOVE_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    GripperServo_MoveTimedPump(GRIPPER_OPEN_ANGLE_DEG, Z_CUBE_PRE_CLOSE_TIME_MS, LowerTurntable_ServicePump);
                    ZStepper_RunWithService(Z_CUBE_DOWN_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    /* 正方体：下降到原设定位置后闭合。 */
                    GripperServo_StartMove(GRIPPER_CLOSE_ANGLE_DEG, Z_CUBE_CLOSE_TIME_MS);
                }
                else if (zCurrentPickIsCylinder)
                {
                    /* Run the cube-style lift, partial close, return, and reopen before the existing cylinder grip. */
                    ZStepper_RunWithService(-Z_CUBE_PRE_MOVE_STEPS, Z_LIFT_INTERVAL_US, LowerTurntable_ServiceZ);
                    GripperServo_MoveTimedPump(Z_CYLINDER_LIFT_PRE_CLOSE_ANGLE_DEG, Z_CUBE_PRE_CLOSE_TIME_MS, LowerTurntable_ServicePump);
                    ZStepper_RunWithService(Z_CUBE_PRE_MOVE_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    GripperServo_MoveTimedPump(GRIPPER_OPEN_ANGLE_DEG, Z_CUBE_PRE_CLOSE_TIME_MS, LowerTurntable_ServicePump);
                    GripperServo_MoveTimedPump(Z_CYLINDER_PRE_CLOSE_ANGLE_DEG, Z_CYLINDER_PRE_CLOSE_TIME_MS, LowerTurntable_ServicePump);
                    ZStepper_RunWithService(Z_CYLINDER_DOWN_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    /* 圆柱：先预收拢3°，下探约4cm后再夹紧到最终角度。 */
                    GripperServo_StartMove(GRIPPER_CLOSE_ANGLE_DEG, Z_CYLINDER_CLOSE_TIME_MS);
                }
                else
                {
                    /* 其他物块：先快闭35°，下降0.7cm，再快开35°；再空降7mm后边下降边闭合。 */
                    GripperServo_MoveTimedPump(Z_OTHER_STAGE1_CLOSE_ANGLE_DEG, Z_OTHER_STAGE1_CLOSE_TIME_MS, LowerTurntable_ServicePump);
                    ZStepper_RunWithService(Z_OTHER_STAGE1_DOWN_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    GripperServo_MoveTimedPump(Z_OTHER_STAGE2_OPEN_ANGLE_DEG, Z_OTHER_STAGE2_OPEN_TIME_MS, LowerTurntable_ServicePump);
                    ZStepper_RunWithService(Z_OTHER_AFTER_OPEN_DOWN_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    GripperServo_StartMove(GRIPPER_CLOSE_ANGLE_DEG,
                                           Z_OTHER_FINAL_CLOSE_TIME_MS);
                    ZStepper_RunWithService(Z_OTHER_FINAL_DESCENT_STEPS, Z_STEP_INTERVAL_US, LowerTurntable_ServiceZ);
                    while (GripperServo_IsBusy()) { LowerTurntable_Service1ms(); Delay_ms(1); }
                }
                /* Wait only for the commanded gripper movement to finish. */
                while (GripperServo_IsBusy()) { LowerTurntable_Service1ms(); Delay_ms(1); }
                if (zCurrentPickIsCube)
                {
                    ZStepper_RunWithService(-Z_CUBE_LIFT_STEPS, Z_LIFT_INTERVAL_US, LowerTurntable_ServiceZ);
                }
                else if (zCurrentPickIsCylinder)
                {
                    ZStepper_RunWithService(-Z_CYLINDER_LIFT_STEPS, Z_LIFT_INTERVAL_US, LowerTurntable_ServiceZ);
                }
                else
                {
                    Delay_ms(Z_REVERSE_PAUSE_MS);
                    ZStepper_RunWithService(-Z_OTHER_LIFT_STEPS, Z_LIFT_INTERVAL_US, LowerTurntable_ServiceZ);
                }
                /* Lift finished: move to the absolute release point, then open gripper. */
                x_steps = MmToSteps(PLACE_X_CENTI_MM, STEPS_PER_MM_X_NUM, STEPS_PER_MM_X_DEN)
                          - TB6600_X_Position;
                y_steps = MmToSteps(PLACE_Y_CENTI_MM, STEPS_PER_MM_Y_NUM, STEPS_PER_MM_Y_DEN)
                          - TB6600_Y_Position;
                moveError = TB6600_MovePump(x_steps, y_steps, MOVE_SPEED_HZ, MOVE_SPEED_HZ,
                                            LowerTurntable_ServicePump);
                TB6600_StopBoth();
                if (moveError != 0U)
                {
                    /* The release point is X=0. Accept an early X-home-limit
                       stop only when both reported positions equal the exact
                       release target; never ignore PC7 abort or a Y-axis error. */
                    if (moveError != TB6600_HOME_ERR_X || !TB6600_RunPressed() ||
                        TB6600_X_Position != MmToSteps(PLACE_X_CENTI_MM,
                                                       STEPS_PER_MM_X_NUM, STEPS_PER_MM_X_DEN) ||
                        TB6600_Y_Position != MmToSteps(PLACE_Y_CENTI_MM,
                                                       STEPS_PER_MM_Y_NUM, STEPS_PER_MM_Y_DEN))
                    {
                        break;
                    }
                }

                /* Leave the object in the release device. */
                GripperServo_MoveTimedPump(GRIPPER_OPEN_ANGLE_DEG, GRIPPER_MOVE_TIME_MS, LowerTurntable_ServicePump);

                if (zCurrentPickIsCube)
                {
                    /* Return from about 2.5 cm above the new Z start while XY homes. */
                    ZStepper_StartRun(Z_CUBE_RETURN_STEPS, Z_RETURN_INTERVAL_US);
                }
                else if (zCurrentPickIsCylinder)
                {
                    /* 圆柱上升5cm后，在XY前往待机点途中下降约2cm复位。 */
                    ZStepper_StartRun(Z_CYLINDER_RETURN_STEPS, Z_RETURN_INTERVAL_US);
                }
                else
                {
                    /* Other shapes are already at the new Z start after the 5 cm lift. */
                    ZStepper_StartRun(Z_OTHER_RETURN_STEPS, Z_RETURN_INTERVAL_US);
                }

                /* Move to the standby point, start camera recognition 3 s after
                   release during travel, wait there for the result, then home XY. */
                visionPumpStarted = 0U;
                visionPumpDone = 0U;
                visionPumpStatus = 0U;
                visionPumpElapsedMs = 0U;
                visionPumpWaitMs = 0U;

                x_steps = MmToSteps(STANDBY_X_CENTI_MM,
                                    STEPS_PER_MM_X_NUM, STEPS_PER_MM_X_DEN) - TB6600_X_Position;
                y_steps = MmToSteps(STANDBY_Y_CENTI_MM,
                                    STEPS_PER_MM_Y_NUM, STEPS_PER_MM_Y_DEN) - TB6600_Y_Position;
                moveError = TB6600_MovePump(x_steps, y_steps, MOVE_SPEED_HZ, MOVE_SPEED_HZ,
                                            VisionPumpDuringHome);
                while (ZStepper_IsBusy())
                {
                    ZStepper_Service1ms();
                    LowerTurntable_Service1ms();
                    Delay_ms(1);
                }
                if ((moveError != 0U &&
                     !(moveError == TB6600_HOME_ERR_Y && TB6600_Y_Position == 0 &&
                       TB6600_X_Position == MmToSteps(STANDBY_X_CENTI_MM,
                                                      STEPS_PER_MM_X_NUM, STEPS_PER_MM_X_DEN))) ||
                    !TB6600_RunPressed()) break;

                /* Remain at (X=6.5 cm, Y=0) until the second camera is done. */
                while (!visionPumpDone && TB6600_RunPressed())
                {
                    VisionPumpDuringHome();
                    Delay_ms(1);
                }
                while (ZStepper_IsBusy())
                {
                    ZStepper_Service1ms();
                    LowerTurntable_Service1ms();
                    Delay_ms(1);
                }
                if (!TB6600_RunPressed() || !visionPumpDone) break;

                visionStatus = visionPumpStatus;

                if (visionStatus != VISION_UART_OK)
                {
                    /* No second-camera match: release to box 4, then preposition one gate ahead. */
                    classifyError = LowerTurntable_MoveToSensor(4U, &currentGate);
                    if (classifyError != 0U) break;

                    releaseError = BoardUART_RequestRelease(3000UL);
                    if (releaseError != 1U) break;

                    /* Release as soon as box 4 is aligned; home XY afterward. */
                    homeError = TB6600_HomeBothPump(HOME_SPEED_HZ, HOME_TIMEOUT_MS, LowerTurntable_ServicePump);
                    if (homeError != 0U || !TB6600_RunPressed()) break;

                    classifyError = LowerTurntable_PreturnNext(&currentGate);
                    if (classifyError != 0U) break;

                    continue;
                }

                visionCategory = VisionCategory();
                if (Vision_Color == 4U)
                {
                    /* Normalize dark-blue detections to cuboid for routing and display. */
                    Vision_Shape = 7U;
                    visionCategory = VisionCategory();
                }
                if (firstRelease)
                {
                    /* First recognized category uses box 1, even after unknown-item prepositioning. */
                    classifyError = LowerTurntable_MoveToSensor(1U, &currentGate);
                    if (classifyError != 0U) break;
                    Shape_Gate[0] = visionCategory;
                    firstRelease = 0U;
                }
                else
                {
                    visionTarget = CategoryToTargetGate(visionCategory);
                    if (visionTarget == 0U) break;

                    classifyError = LowerTurntable_MoveToSensor(visionTarget, &currentGate);
                    if (classifyError != 0U) break;
                }

                releaseError = BoardUART_RequestRelease(3000UL);
                if (releaseError != 1U) break;

                /* Release as soon as the destination box is aligned; home XY afterward. */
                homeError = TB6600_HomeBothPump(HOME_SPEED_HZ, HOME_TIMEOUT_MS, LowerTurntable_ServicePump);
                if (homeError != 0U || !TB6600_RunPressed()) break;

                /* Count and display only after F103 confirms release DONE. */
                ScreenTable_UpdateItem(Vision_Shape, visionCategory);

                /* Advance one gate now; the next class can route back by shortest path. */
                classifyError = LowerTurntable_PreturnNext(&currentGate);
                if (classifyError != 0U) break;

            }
        }

        while (lowerPreturnActive != 0U)
        {
            LowerTurntable_Service1ms();
            Delay_ms(1);
        }
        ZStepper_Stop();
        TB6600_StopBoth();
        /* Finish this batch before accepting another PC7 press. */
        while (TB6600_RunPressed()) { Delay_ms(1); }
    }
}







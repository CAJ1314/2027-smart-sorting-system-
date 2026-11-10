#include "ScreenTable.h"
#include "TJCDisplay.h"
#include "Delay.h"

#define SCREEN_ROW_COUNT       6U
#define TJC_PICTURE_BASE_ID    1U

static const char *const NameComponent[SCREEN_ROW_COUNT] = {
    "tName0", "tName1", "tName2", "tName3", "tName4", "tName5"
};
static const char *const PictureComponent[SCREEN_ROW_COUNT] = {
    "pItem0", "pItem1", "pItem2", "pItem3", "pItem4", "pItem5"
};
static const char *const CountComponent[SCREEN_ROW_COUNT] = {
    "nCount0", "nCount1", "nCount2", "nCount3", "nCount4", "nCount5"
};

/* GB2312 byte strings. The TJC text components must use a GB2312 font. */
static const char ShapeCube[]       = "\xD5\xFD\xB7\xBD\xCC\xE5";             /* 正方体 */
static const char ShapeHexPrism[]   = "\xC1\xF9\xC0\xE2\xD6\xF9";             /* 六棱柱 */
static const char ShapeTriPrism[]   = "\xC8\xFD\xC0\xE2\xD6\xF9";             /* 三棱柱 */
static const char ShapeSphere[]      = "\xC7\xF2";                         /* 球 */
static const char ShapePentPrism[]  = "\xCE\xE5\xC0\xE2\xD6\xF9";             /* 五棱柱 */
static const char ShapeCylinder[]   = "\xD4\xB2\xD6\xF9";                         /* 圆柱 */
static const char ShapeCuboid[]     = "\xB3\xA4\xB7\xBD\xCC\xE5";             /* 长方体 */

static uint8_t ScreenCategory[SCREEN_ROW_COUNT];
static uint16_t ScreenCount[SCREEN_ROW_COUNT];

static const char *ShapeName(uint8_t shape)
{
    switch (shape)
    {
        case 1U: return ShapeCube;
        case 2U: return ShapeHexPrism;
        case 3U: return ShapeTriPrism;
        case 4U: return ShapeSphere;
        case 5U: return ShapePentPrism;
        case 6U: return ShapeCylinder;
        case 7U: return ShapeCuboid;
        default: return "";
    }
}

/* Resource order is color-major, then shape-major, matching BlockPicture.h. */
static uint16_t CategoryPictureId(uint8_t category)
{
    uint8_t color = (uint8_t)(category / 8U);
    uint8_t shape = (uint8_t)(category % 8U);

    if (shape < 1U || shape > 7U) return TJC_PICTURE_BASE_ID;
    if (color < 1U || color > 7U) color = 1U;

    return (uint16_t)(TJC_PICTURE_BASE_ID +
        (uint16_t)(color - 1U) * 7U + (uint16_t)(shape - 1U));
}

void ScreenTable_Init(void)
{
    TJCDisplay_Init();
    /* Give the serial display time to finish its power-on initialization. */
    Delay_ms(1200U);
}

void ScreenTable_DrawFrame(void)
{
    TJCDisplay_Page("main");
    Delay_ms(80U);
    ScreenTable_ResetData();
}

void ScreenTable_ResetData(void)
{
    uint8_t i;

    TJCDisplay_SendCommand("ref_stop");
    for (i = 0U; i < SCREEN_ROW_COUNT; ++i)
    {
        ScreenCategory[i] = 0U;
        ScreenCount[i] = 0U;
        TJCDisplay_SetText(NameComponent[i], "");
        TJCDisplay_SetValue(CountComponent[i], 0U);
        TJCDisplay_SetVisible(PictureComponent[i], 0U);
    }
    TJCDisplay_SendCommand("ref_star");
}

void ScreenTable_UpdateItem(uint8_t shape, uint8_t category)
{
    uint8_t i;
    uint8_t slot = SCREEN_ROW_COUNT;

    if (shape < 1U || shape > 7U || category == 0U) return;

    for (i = 0U; i < SCREEN_ROW_COUNT; ++i)
    {
        if (ScreenCategory[i] == category)
        {
            slot = i;
            break;
        }
    }

    if (slot == SCREEN_ROW_COUNT)
    {
        for (i = 0U; i < SCREEN_ROW_COUNT; ++i)
        {
            if (ScreenCategory[i] == 0U)
            {
                slot = i;
                ScreenCategory[i] = category;
                break;
            }
        }
    }

    if (slot == SCREEN_ROW_COUNT) return;

    if (ScreenCount[slot] < 65535U) ++ScreenCount[slot];

    TJCDisplay_SendCommand("ref_stop");
    TJCDisplay_SetText(NameComponent[slot], ShapeName(shape));
    TJCDisplay_SetPicture(PictureComponent[slot], CategoryPictureId(category));
    TJCDisplay_SetVisible(PictureComponent[slot], 1U);
    TJCDisplay_SetValue(CountComponent[slot], ScreenCount[slot]);
    TJCDisplay_SendCommand("ref_star");
}

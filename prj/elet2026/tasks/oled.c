/**
  ******************************************************************************
  * @file    oled.c
  * @brief   0.96 寸 128x64 OLED：SSD1306/SSD1315 初始化、显存与 I2C DMA 刷新。
  ******************************************************************************
  * @note    只由 displayTask 调用；显存和 DMA 缓冲均为静态变量。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "i2c.h"
#include "cmsis_os.h"
#include "oled.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define OLED_CTRL_CMD             0x00U
#define OLED_CTRL_DATA            0x40U

/* 5x7 字体，覆盖 ASCII 32~90；小写字母在显示函数中折叠为大写 */
#define OLED_FONT_FIRST_CHAR      32U
#define OLED_FONT_LAST_CHAR       90U
#define OLED_FONT_CHAR_COUNT      (OLED_FONT_LAST_CHAR - OLED_FONT_FIRST_CHAR + 1U)

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t s_fb[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t s_i2c_addr = OLED_I2C_ADDR_DEFAULT;
static uint8_t s_initialized = 0U;
static volatile uint8_t s_refresh_result = 0U;
static volatile uint32_t s_error_count = 0U;
static osSemaphoreId_t s_refresh_sem = NULL;

/* 前置声明，字体表放在文件末尾，节省阅读驱动流程的时间 */
static const uint8_t s_font5x7[OLED_FONT_CHAR_COUNT][5];

/* 私有函数声明 --------------------------------------------------------------*/
static uint8_t OLED_ProbeAddress(void);

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：探测 OLED 的 8 位 I2C 地址（0x78 或 0x7A）
 * 输入：无
 * 输出：1 = 找到设备；0 = 都无应答
 */
static uint8_t OLED_ProbeAddress(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR_DEFAULT, 3U, 100U) == HAL_OK)
  {
    s_i2c_addr = OLED_I2C_ADDR_DEFAULT;
    return 1U;
  }

  if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_I2C_ADDR_ALT, 3U, 100U) == HAL_OK)
  {
    s_i2c_addr = OLED_I2C_ADDR_ALT;
    return 1U;
  }

  return 0U;
}

/*
 * 功能：初始化 OLED
 * 输入：无
 * 输出：1 = 成功；0 = 失败
 */
uint8_t OLED_Init(void)
{
  static const uint8_t init_cmds[] =
  {
    0xAEU,              /* display off */
    0xD5U, 0x80U,       /* clock divide / oscillator */
    0xA8U, 0x3FU,       /* multiplex ratio 1/64 */
    0xD3U, 0x00U,       /* display offset 0 */
    0x40U,              /* start line 0 */
    0x8DU, 0x14U,       /* charge pump enable */
    0xA1U,              /* segment remap */
    0xC8U,              /* COM scan direction */
    0xDAU, 0x12U,       /* COM pins */
    0x81U, 0xCFU,       /* contrast */
    0xD9U, 0xF1U,       /* pre-charge */
    0xDBU, 0x40U,       /* VCOMH */
    0x20U, 0x00U,       /* horizontal addressing mode */
    0x21U, 0x00U, 0x7FU,/* column 0..127 */
    0x22U, 0x00U, 0x07U,/* page 0..7 */
    0xA4U,              /* display follows RAM */
    0xA6U,              /* normal display */
    0xAFU               /* display on */
  };

  if (s_refresh_sem == NULL)
  {
    s_refresh_sem = osSemaphoreNew(1U, 0U, NULL);
    if (s_refresh_sem == NULL)
    {
      s_error_count++;
      return 0U;
    }
  }

  /* 4 针模块没有 RESET 脚，依赖上电复位，先等电源和内部电荷泵稳定 */
  HAL_Delay(100U);

  if (OLED_ProbeAddress() == 0U)
  {
    return 0U;
  }

  if (HAL_I2C_Mem_Write(&hi2c1,
                        s_i2c_addr,
                        OLED_CTRL_CMD,
                        I2C_MEMADD_SIZE_8BIT,
                        (uint8_t *)init_cmds,
                        (uint16_t)sizeof(init_cmds),
                        100U) != HAL_OK)
  {
    s_error_count++;
    return 0U;
  }

  HAL_Delay(100U);

  s_initialized = 1U;
  OLED_Clear();
  if (OLED_Refresh_DMA() == 0U)
  {
    s_initialized = 0U;
    return 0U;
  }
  if (OLED_WaitRefresh(100U) == 0U)
  {
    s_initialized = 0U;
    return 0U;
  }

  return 1U;
}

/*
 * 功能：清空 MCU 侧显存
 * 输入：无
 * 输出：无
 */
void OLED_Clear(void)
{
  (void)memset(s_fb, 0, sizeof(s_fb));
}

/*
 * 功能：在指定页显示 ASCII 字符串
 * 输入：x —— 起始列；page —— 页号 0~7；text —— 字符串
 * 输出：无
 */
void OLED_ShowString(uint8_t x, uint8_t page, const char *text)
{
  const uint8_t *glyph;
  uint8_t ch;
  uint8_t i;

  if ((text == NULL) || (page >= OLED_PAGE_COUNT))
  {
    return;
  }

  while ((*text != '\0') && (x < OLED_WIDTH))
  {
    ch = (uint8_t)*text;

    if ((ch >= (uint8_t)'a') && (ch <= (uint8_t)'z'))
    {
      ch = (uint8_t)(ch - 32U);
    }
    if ((ch < OLED_FONT_FIRST_CHAR) || (ch > OLED_FONT_LAST_CHAR))
    {
      ch = (uint8_t)'?';
    }

    glyph = &s_font5x7[ch - OLED_FONT_FIRST_CHAR][0];

    for (i = 0U; i < 5U; i++)
    {
      if ((uint16_t)x + i < OLED_WIDTH)
      {
        s_fb[page][x + i] = glyph[i];
      }
    }
    if ((uint16_t)x + 5U < OLED_WIDTH)
    {
      s_fb[page][x + 5U] = 0U;
    }

    x = (uint8_t)(x + 6U);
    text++;
  }
}

/*
 * 功能：启动一次整屏 DMA 刷新
 * 输入：无
 * 输出：1 = 启动成功；0 = 失败
 */
uint8_t OLED_Refresh_DMA(void)
{
  if ((s_initialized == 0U) || (s_refresh_sem == NULL))
  {
    return 0U;
  }

  /* 丢弃上一次遗留的完成信号 */
  (void)osSemaphoreAcquire(s_refresh_sem, 0U);
  s_refresh_result = 0U;

  if (HAL_I2C_Mem_Write_DMA(&hi2c1,
                            s_i2c_addr,
                            OLED_CTRL_DATA,
                            I2C_MEMADD_SIZE_8BIT,
                            (uint8_t *)s_fb,
                            (uint16_t)sizeof(s_fb)) != HAL_OK)
  {
    s_error_count++;
    return 0U;
  }

  return 1U;
}

/*
 * 功能：等待上一次 DMA 刷新完成
 * 输入：timeout_ms —— 超时时间
 * 输出：1 = 成功；0 = 超时/错误
 */
uint8_t OLED_WaitRefresh(uint32_t timeout_ms)
{
  if (s_refresh_sem == NULL)
  {
    return 0U;
  }

  if (osSemaphoreAcquire(s_refresh_sem, pdMS_TO_TICKS(timeout_ms)) != osOK)
  {
    s_error_count++;
    return 0U;
  }

  return s_refresh_result;
}

/*
 * 功能：读取 OLED I2C 错误计数
 * 输入：无
 * 输出：错误计数
 */
uint32_t OLED_GetErrorCount(void)
{
  return s_error_count;
}

/*
 * 功能：HAL I2C memory write DMA 完成回调
 * 输入：hi2c —— I2C 句柄
 * 输出：无
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c != NULL) && (hi2c->Instance == I2C1))
  {
    s_refresh_result = 1U;
    if (s_refresh_sem != NULL)
    {
      (void)osSemaphoreRelease(s_refresh_sem);
    }
  }
}

/*
 * 功能：HAL I2C 错误回调
 * 输入：hi2c —— I2C 句柄
 * 输出：无
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if ((hi2c != NULL) && (hi2c->Instance == I2C1))
  {
    s_refresh_result = 0U;
    s_error_count++;
    if (s_refresh_sem != NULL)
    {
      (void)osSemaphoreRelease(s_refresh_sem);
    }
  }
}

/* ========================================================================== */
/*                              5x7 ASCII 字体                                */
/* ========================================================================== */
/* 每字符 5 列，列内 bit0 = 最上面一行；字符串显示时列间隔 1 像素。 */
static const uint8_t s_font5x7[OLED_FONT_CHAR_COUNT][5] =
{
  0x00U, 0x00U, 0x00U, 0x00U, 0x00U, /* 32 space */
  0x00U, 0x00U, 0x5FU, 0x00U, 0x00U, /* 33 ! */
  0x00U, 0x07U, 0x00U, 0x07U, 0x00U, /* 34 " */
  0x14U, 0x7FU, 0x14U, 0x7FU, 0x14U, /* 35 # */
  0x24U, 0x2AU, 0x7FU, 0x2AU, 0x12U, /* 36 $ */
  0x23U, 0x13U, 0x08U, 0x64U, 0x62U, /* 37 % */
  0x36U, 0x49U, 0x55U, 0x22U, 0x50U, /* 38 & */
  0x00U, 0x05U, 0x03U, 0x00U, 0x00U, /* 39 ' */
  0x00U, 0x1CU, 0x22U, 0x41U, 0x00U, /* 40 ( */
  0x00U, 0x41U, 0x22U, 0x1CU, 0x00U, /* 41 ) */
  0x14U, 0x08U, 0x3EU, 0x08U, 0x14U, /* 42 * */
  0x08U, 0x08U, 0x3EU, 0x08U, 0x08U, /* 43 + */
  0x00U, 0x50U, 0x30U, 0x00U, 0x00U, /* 44 , */
  0x08U, 0x08U, 0x08U, 0x08U, 0x08U, /* 45 - */
  0x00U, 0x60U, 0x60U, 0x00U, 0x00U, /* 46 . */
  0x20U, 0x10U, 0x08U, 0x04U, 0x02U, /* 47 / */
  0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU, /* 48 0 */
  0x00U, 0x42U, 0x7FU, 0x40U, 0x00U, /* 49 1 */
  0x42U, 0x61U, 0x51U, 0x49U, 0x46U, /* 50 2 */
  0x21U, 0x41U, 0x45U, 0x4BU, 0x31U, /* 51 3 */
  0x18U, 0x14U, 0x12U, 0x7FU, 0x10U, /* 52 4 */
  0x27U, 0x45U, 0x45U, 0x45U, 0x39U, /* 53 5 */
  0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U, /* 54 6 */
  0x01U, 0x71U, 0x09U, 0x05U, 0x03U, /* 55 7 */
  0x36U, 0x49U, 0x49U, 0x49U, 0x36U, /* 56 8 */
  0x06U, 0x49U, 0x49U, 0x29U, 0x1EU, /* 57 9 */
  0x00U, 0x36U, 0x36U, 0x00U, 0x00U, /* 58 : */
  0x00U, 0x56U, 0x36U, 0x00U, 0x00U, /* 59 ; */
  0x00U, 0x08U, 0x14U, 0x22U, 0x41U, /* 60 < */
  0x14U, 0x14U, 0x14U, 0x14U, 0x14U, /* 61 = */
  0x41U, 0x22U, 0x14U, 0x08U, 0x00U, /* 62 > */
  0x02U, 0x01U, 0x51U, 0x09U, 0x06U, /* 63 ? */
  0x32U, 0x49U, 0x79U, 0x41U, 0x3EU, /* 64 @ */
  0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU, /* 65 A */
  0x7FU, 0x49U, 0x49U, 0x49U, 0x36U, /* 66 B */
  0x3EU, 0x41U, 0x41U, 0x41U, 0x22U, /* 67 C */
  0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU, /* 68 D */
  0x7FU, 0x49U, 0x49U, 0x49U, 0x41U, /* 69 E */
  0x7FU, 0x09U, 0x09U, 0x09U, 0x01U, /* 70 F */
  0x3EU, 0x41U, 0x41U, 0x51U, 0x32U, /* 71 G */
  0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU, /* 72 H */
  0x00U, 0x41U, 0x7FU, 0x41U, 0x00U, /* 73 I */
  0x20U, 0x40U, 0x41U, 0x3FU, 0x01U, /* 74 J */
  0x7FU, 0x08U, 0x14U, 0x22U, 0x41U, /* 75 K */
  0x7FU, 0x40U, 0x40U, 0x40U, 0x40U, /* 76 L */
  0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU, /* 77 M */
  0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU, /* 78 N */
  0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU, /* 79 O */
  0x7FU, 0x09U, 0x09U, 0x09U, 0x06U, /* 80 P */
  0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU, /* 81 Q */
  0x7FU, 0x09U, 0x19U, 0x29U, 0x46U, /* 82 R */
  0x46U, 0x49U, 0x49U, 0x49U, 0x31U, /* 83 S */
  0x01U, 0x01U, 0x7FU, 0x01U, 0x01U, /* 84 T */
  0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU, /* 85 U */
  0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU, /* 86 V */
  0x7FU, 0x20U, 0x18U, 0x20U, 0x7FU, /* 87 W */
  0x63U, 0x14U, 0x08U, 0x14U, 0x63U, /* 88 X */
  0x03U, 0x04U, 0x78U, 0x04U, 0x03U, /* 89 Y */
  0x61U, 0x51U, 0x49U, 0x45U, 0x43U  /* 90 Z */
};

/************************ (C) COPYRIGHT *****END OF FILE***********************/


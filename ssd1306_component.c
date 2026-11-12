#include "ssd1306_component.h"
#include "string.h"
#include "esp_log.h"
#include <stdio.h>


static const char *TAG = "ssd1306";

/* ── 5×8 ASCII font (printable range 0x20 … 0x7E) ──────────────────────── */
/* Each character is 5 bytes wide; each byte is a column of 8 pixels,
   LSB = top row.                                                            */
static const uint8_t font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 0x20  space  */
    {0x00,0x00,0x5F,0x00,0x00}, /* 0x21  !      */
    {0x00,0x07,0x00,0x07,0x00}, /* 0x22  "      */
    {0x14,0x7F,0x14,0x7F,0x14}, /* 0x23  #      */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* 0x24  $      */
    {0x23,0x13,0x08,0x64,0x62}, /* 0x25  %      */
    {0x36,0x49,0x55,0x22,0x50}, /* 0x26  &      */
    {0x00,0x05,0x03,0x00,0x00}, /* 0x27  '      */
    {0x00,0x1C,0x22,0x41,0x00}, /* 0x28  (      */
    {0x00,0x41,0x22,0x1C,0x00}, /* 0x29  )      */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* 0x2A  *      */
    {0x08,0x08,0x3E,0x08,0x08}, /* 0x2B  +      */
    {0x00,0x50,0x30,0x00,0x00}, /* 0x2C  ,      */
    {0x08,0x08,0x08,0x08,0x08}, /* 0x2D  -      */
    {0x00,0x60,0x60,0x00,0x00}, /* 0x2E  .      */
    {0x20,0x10,0x08,0x04,0x02}, /* 0x2F  /      */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0x30  0      */
    {0x00,0x42,0x7F,0x40,0x00}, /* 0x31  1      */
    {0x42,0x61,0x51,0x49,0x46}, /* 0x32  2      */
    {0x21,0x41,0x45,0x4B,0x31}, /* 0x33  3      */
    {0x18,0x14,0x12,0x7F,0x10}, /* 0x34  4      */
    {0x27,0x45,0x45,0x45,0x39}, /* 0x35  5      */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 0x36  6      */
    {0x01,0x71,0x09,0x05,0x03}, /* 0x37  7      */
    {0x36,0x49,0x49,0x49,0x36}, /* 0x38  8      */
    {0x06,0x49,0x49,0x29,0x1E}, /* 0x39  9      */
    {0x00,0x36,0x36,0x00,0x00}, /* 0x3A  :      */
    {0x00,0x56,0x36,0x00,0x00}, /* 0x3B  ;      */
    {0x08,0x14,0x22,0x41,0x00}, /* 0x3C  <      */
    {0x14,0x14,0x14,0x14,0x14}, /* 0x3D  =      */
    {0x00,0x41,0x22,0x14,0x08}, /* 0x3E  >      */
    {0x02,0x01,0x51,0x09,0x06}, /* 0x3F  ?      */
    {0x32,0x49,0x79,0x41,0x3E}, /* 0x40  @      */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 0x41  A      */
    {0x7F,0x49,0x49,0x49,0x36}, /* 0x42  B      */
    {0x3E,0x41,0x41,0x41,0x22}, /* 0x43  C      */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 0x44  D      */
    {0x7F,0x49,0x49,0x49,0x41}, /* 0x45  E      */
    {0x7F,0x09,0x09,0x09,0x01}, /* 0x46  F      */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 0x47  G      */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 0x48  H      */
    {0x00,0x41,0x7F,0x41,0x00}, /* 0x49  I      */
    {0x20,0x40,0x41,0x3F,0x01}, /* 0x4A  J      */
    {0x7F,0x08,0x14,0x22,0x41}, /* 0x4B  K      */
    {0x7F,0x40,0x40,0x40,0x40}, /* 0x4C  L      */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 0x4D  M      */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 0x4E  N      */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 0x4F  O      */
    {0x7F,0x09,0x09,0x09,0x06}, /* 0x50  P      */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 0x51  Q      */
    {0x7F,0x09,0x19,0x29,0x46}, /* 0x52  R      */
    {0x46,0x49,0x49,0x49,0x31}, /* 0x53  S      */
    {0x01,0x01,0x7F,0x01,0x01}, /* 0x54  T      */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 0x55  U      */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 0x56  V      */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 0x57  W      */
    {0x63,0x14,0x08,0x14,0x63}, /* 0x58  X      */
    {0x07,0x08,0x70,0x08,0x07}, /* 0x59  Y      */
    {0x61,0x51,0x49,0x45,0x43}, /* 0x5A  Z      */
    {0x00,0x7F,0x41,0x41,0x00}, /* 0x5B  [      */
    {0x02,0x04,0x08,0x10,0x20}, /* 0x5C  \      */
    {0x00,0x41,0x41,0x7F,0x00}, /* 0x5D  ]      */
    {0x04,0x02,0x01,0x02,0x04}, /* 0x5E  ^      */
    {0x40,0x40,0x40,0x40,0x40}, /* 0x5F  _      */
    {0x00,0x01,0x02,0x04,0x00}, /* 0x60  `      */
    {0x20,0x54,0x54,0x54,0x78}, /* 0x61  a      */
    {0x7F,0x48,0x44,0x44,0x38}, /* 0x62  b      */
    {0x38,0x44,0x44,0x44,0x20}, /* 0x63  c      */
    {0x38,0x44,0x44,0x48,0x7F}, /* 0x64  d      */
    {0x38,0x54,0x54,0x54,0x18}, /* 0x65  e      */
    {0x08,0x7E,0x09,0x01,0x02}, /* 0x66  f      */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 0x67  g      */
    {0x7F,0x08,0x04,0x04,0x78}, /* 0x68  h      */
    {0x00,0x44,0x7D,0x40,0x00}, /* 0x69  i      */
    {0x20,0x40,0x44,0x3D,0x00}, /* 0x6A  j      */
    {0x7F,0x10,0x28,0x44,0x00}, /* 0x6B  k      */
    {0x00,0x41,0x7F,0x40,0x00}, /* 0x6C  l      */
    {0x7C,0x04,0x18,0x04,0x78}, /* 0x6D  m      */
    {0x7C,0x08,0x04,0x04,0x78}, /* 0x6E  n      */
    {0x38,0x44,0x44,0x44,0x38}, /* 0x6F  o      */
    {0x7C,0x14,0x14,0x14,0x08}, /* 0x70  p      */
    {0x08,0x14,0x14,0x18,0x7C}, /* 0x71  q      */
    {0x7C,0x08,0x04,0x04,0x08}, /* 0x72  r      */
    {0x48,0x54,0x54,0x54,0x20}, /* 0x73  s      */
    {0x04,0x3F,0x44,0x40,0x20}, /* 0x74  t      */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 0x75  u      */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 0x76  v      */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 0x77  w      */
    {0x44,0x28,0x10,0x28,0x44}, /* 0x78  x      */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 0x79  y      */
    {0x44,0x64,0x54,0x4C,0x44}, /* 0x7A  z      */
    {0x00,0x08,0x36,0x41,0x00}, /* 0x7B  {      */
    {0x00,0x00,0x7F,0x00,0x00}, /* 0x7C  |      */
    {0x00,0x41,0x36,0x08,0x00}, /* 0x7D  }      */
    {0x08,0x08,0x2A,0x1C,0x08}, /* 0x7E  ~      */
};

/* ── Internal helpers ───────────────────────────────────────────────────── */

static esp_err_t write_cmd(ssd1306_t *dev, uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CTRL_CMD, cmd };
    return i2c_dev_write(&dev->dev, NULL, 0, buf, sizeof(buf));
}

static esp_err_t write_cmd2(ssd1306_t *dev, uint8_t cmd, uint8_t arg)
{
    uint8_t buf[3] = { SSD1306_CTRL_CMD, cmd, arg };
    return i2c_dev_write(&dev->dev, NULL, 0, buf, sizeof(buf));
}

/* ── Lifecycle ──────────────────────────────────────────────────────────── */

esp_err_t ssd1306_init(ssd1306_t *dev,
                       i2c_port_t port,
                       gpio_num_t sda_gpio,
                       gpio_num_t scl_gpio,
                       uint8_t    addr)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    memset(dev, 0, sizeof(*dev));

    /* Configure i2cdev device descriptor */
    dev->dev.port    = port;
    dev->dev.addr    = addr;
    dev->dev.cfg.sda_io_num = sda_gpio;
    dev->dev.cfg.scl_io_num = scl_gpio;
    dev->dev.cfg.master.clk_speed = SSD1306_I2C_FREQ_HZ;

    esp_err_t ret = i2c_dev_create_mutex(&dev->dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_dev_create_mutex: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialisation sequence (datasheet §8.5 Application Example) */
    const struct { uint8_t cmd; uint8_t has_arg; uint8_t arg; } seq[] = {
        { SSD1306_CMD_DISPLAY_OFF,          0, 0x00 },
        { SSD1306_CMD_SET_DISPLAY_CLK,      1, 0x80 }, /* ratio/freq default */
        { SSD1306_CMD_SET_MUX_RATIO,        1, SSD1306_HEIGHT - 1 },
        { SSD1306_CMD_SET_DISPLAY_OFFSET,   1, 0x00 },
        { SSD1306_CMD_SET_DISP_START_LINE,  0, 0x00 },
        { SSD1306_CMD_SET_CHARGE_PUMP,      1, SSD1306_CHARGE_PUMP_ENABLE },
        { SSD1306_CMD_SET_MEM_MODE,         1, SSD1306_MEM_MODE_HORIZONTAL },
        { SSD1306_CMD_SET_SEG_REMAP_1,      0, 0x00 }, /* col 127 → SEG0 */
        { SSD1306_CMD_SET_COM_SCAN_DEC,     0, 0x00 }, /* scan N-1 → 0   */
        { SSD1306_CMD_SET_COM_PIN_CFG,      1, 0x12 }, /* alt COM, no remap */
        { SSD1306_CMD_SET_CONTRAST,         1, 0xCF },
        { SSD1306_CMD_SET_PRECHARGE,        1, 0xF1 },
        { SSD1306_CMD_SET_VCOM_DESELECT,    1, 0x40 },
        { SSD1306_CMD_ENTIRE_DISPLAY_OFF,   0, 0x00 }, /* use RAM          */
        { SSD1306_CMD_NORMAL_DISPLAY,       0, 0x00 },
        { SSD1306_CMD_DISPLAY_ON,           0, 0x00 },
    };

    for (size_t i = 0; i < sizeof(seq)/sizeof(seq[0]); i++) {
        ret = seq[i].has_arg
              ? write_cmd2(dev, seq[i].cmd, seq[i].arg)
              : write_cmd(dev, seq[i].cmd);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "init seq[%zu] failed: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }

    dev->display_on = true;

    /* Clear display RAM */
    ssd1306_clear(dev);
    ret = ssd1306_flush(dev);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "SSD1306 initialised at 0x%02X on I2C%d", addr, port);
    return ESP_OK;
}

esp_err_t ssd1306_free(ssd1306_t *dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    return i2c_dev_delete_mutex(&dev->dev);
}

/* ── Display control ────────────────────────────────────────────────────── */

esp_err_t ssd1306_display_on(ssd1306_t *dev, bool on)
{
    esp_err_t ret = write_cmd(dev, on ? SSD1306_CMD_DISPLAY_ON
                                      : SSD1306_CMD_DISPLAY_OFF);
    if (ret == ESP_OK) dev->display_on = on;
    return ret;
}

esp_err_t ssd1306_set_contrast(ssd1306_t *dev, uint8_t contrast)
{
    return write_cmd2(dev, SSD1306_CMD_SET_CONTRAST, contrast);
}

esp_err_t ssd1306_invert(ssd1306_t *dev, bool invert)
{
    esp_err_t ret = write_cmd(dev, invert ? SSD1306_CMD_INVERT_DISPLAY
                                          : SSD1306_CMD_NORMAL_DISPLAY);
    if (ret == ESP_OK) dev->inverted = invert;
    return ret;
}

/* ── Framebuffer helpers ────────────────────────────────────────────────── */

void ssd1306_clear(ssd1306_t *dev)
{
    memset(dev->framebuf, 0, sizeof(dev->framebuf));
}

void ssd1306_draw_pixel(ssd1306_t *dev, uint8_t x, uint8_t y, bool color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    if (color)
        dev->framebuf[page][x] |=  (1u << bit);
    else
        dev->framebuf[page][x] &= ~(1u << bit);
}

void ssd1306_draw_hline(ssd1306_t *dev, uint8_t x, uint8_t y,
                        uint8_t len, bool color)
{
    for (uint8_t i = 0; i < len; i++)
        ssd1306_draw_pixel(dev, x + i, y, color);
}

void ssd1306_draw_vline(ssd1306_t *dev, uint8_t x, uint8_t y,
                        uint8_t len, bool color)
{
    for (uint8_t i = 0; i < len; i++)
        ssd1306_draw_pixel(dev, x, y + i, color);
}

void ssd1306_draw_rect(ssd1306_t *dev, uint8_t x, uint8_t y,
                       uint8_t w, uint8_t h, bool color)
{
    ssd1306_draw_hline(dev, x,         y,         w, color);
    ssd1306_draw_hline(dev, x,         y + h - 1, w, color);
    ssd1306_draw_vline(dev, x,         y,         h, color);
    ssd1306_draw_vline(dev, x + w - 1, y,         h, color);
}

void ssd1306_fill_rect(ssd1306_t *dev, uint8_t x, uint8_t y,
                       uint8_t w, uint8_t h, bool color)
{
    for (uint8_t row = 0; row < h; row++)
        ssd1306_draw_hline(dev, x, y + row, w, color);
}

void ssd1306_draw_char(ssd1306_t *dev, uint8_t x, uint8_t y, char c)
{
    if (c < 0x20 || c > 0x7E) c = '?';
    const uint8_t *glyph = font5x8[(uint8_t)(c - 0x20)];
    uint8_t page = y / 8;
    if (page >= SSD1306_PAGES) return;

    for (uint8_t col = 0; col < 5; col++) {
        if (x + col >= SSD1306_WIDTH) break;
        dev->framebuf[page][x + col] = glyph[col];
    }
    /* one column gap */
    if (x + 5 < SSD1306_WIDTH)
        dev->framebuf[page][x + 5] = 0x00;
}

void ssd1306_draw_string(ssd1306_t *dev, uint8_t x, uint8_t y,
                         const char *str)
{
    if (!str) return;
    uint8_t cx = x, cy = y;
    while (*str) {
        if (cx + 6 > SSD1306_WIDTH) {   /* wrap */
            cx  = 0;
            cy += 8;
        }
        if (cy + 8 > SSD1306_HEIGHT) break; /* no more rows */
        ssd1306_draw_char(dev, cx, cy, *str++);
        cx += 6;
    }
}

/* ── Flush ──────────────────────────────────────────────────────────────── */

esp_err_t ssd1306_flush(ssd1306_t *dev)
{
    /* Set full-screen address window */
    esp_err_t ret;
    uint8_t addr_cmds[] = {
        SSD1306_CTRL_CMD,
        SSD1306_CMD_SET_COL_ADDR,  0, SSD1306_WIDTH - 1,
        SSD1306_CMD_SET_PAGE_ADDR, 0, SSD1306_PAGES - 1,
    };
    ret = i2c_dev_write(&dev->dev, NULL, 0, addr_cmds, sizeof(addr_cmds));
    if (ret != ESP_OK) return ret;

    /* Stream all pages row-by-row.
       Each I²C write: 1 control byte + SSD1306_WIDTH data bytes.          */
    uint8_t rowbuf[1 + SSD1306_WIDTH];
    rowbuf[0] = SSD1306_CTRL_DATA;

    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        memcpy(&rowbuf[1], dev->framebuf[page], SSD1306_WIDTH);
        ret = i2c_dev_write(&dev->dev, NULL, 0, rowbuf, sizeof(rowbuf));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "flush page %u: %s", page, esp_err_to_name(ret));
            return ret;
        }
    }
    return ESP_OK;
}
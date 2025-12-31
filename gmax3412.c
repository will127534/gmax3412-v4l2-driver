// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony GMAX3412 camera.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* --------------------------------------------------------------------------
 * Registers / limits
 * --------------------------------------------------------------------------
 */
#define GMAX3412_STREAM_DELAY_US          25000
#define GMAX3412_STREAM_DELAY_RANGE_US    1000

/* Initialisation delay between XCLR low->high and the moment sensor is ready */
#define GMAX3412_XCLR_MIN_DELAY_US        10000
#define GMAX3412_XCLR_DELAY_RANGE_US      1000


/* Exposure control (lines) */
/* The driver is set to use external exposure, it will still report the control but it actuall does nothing. */
#define GMAX3412_EXPOSURE_MIN             1000
#define GMAX3412_EXPOSURE_STEP            1000
#define GMAX3412_EXPOSURE_DEFAULT         1000
#define GMAX3412_EXPOSURE_MAX             1000

/* Black level control */
#define GMAX3412_REG_BLKLEVEL             CCI_REG16_LE(0x305B)
#define GMAX3412_BLKLEVEL_DEFAULT         50

/* Analog gain control */
#define GMAX3412_REG_ANALOG_GAIN          CCI_REG8(0x2EAE)
#define GMAX3412_ANA_GAIN_MIN             0
#define GMAX3412_ANA_GAIN_MAX             57-18
#define GMAX3412_ANA_GAIN_STEP            1
#define GMAX3412_ANA_GAIN_DEFAULT         0

/* Vertical Flip */
#define GMAX3412_REG_FLIP_H            CCI_REG8(0x3500)
#define GMAX3412_REG_FLIP_V            CCI_REG8(0x2E05)

/* Pixel rate helper (sensor line clock proxy used below) */
#define GMAX3412_PIXEL_RATE               60000000U
#define GMAX3412_LINK_FREQ               600000000U

static const s64 gmax3412_link_freq_menu[] = {
    GMAX3412_LINK_FREQ,
};

/* Technically the native width and height is 2080x1216 but the frame output is 2048Ux1218 */
#define GMAX3412_NATIVE_WIDTH       4096U
#define GMAX3412_NATIVE_HEIGHT      3072U
#define GMAX3412_PIXEL_ARRAY_LEFT      0U
#define GMAX3412_PIXEL_ARRAY_TOP       0U
#define GMAX3412_PIXEL_ARRAY_WIDTH  4096U
#define GMAX3412_PIXEL_ARRAY_HEIGHT 3072U


static const struct cci_reg_sequence mode_common_regs[] = {
	{CCI_REG8(0x2E00),0x00},
	{CCI_REG8(0x2E01),0x00},
	{CCI_REG8(0x2E02),0x00},
	{CCI_REG8(0x2E03),0x00}, //0: External exposure
	{CCI_REG8(0x2E04),0x00},
	{CCI_REG8(0x2E05),0x08},
	{CCI_REG8(0x2E06),0x00},
	{CCI_REG8(0x2E07),0x01},
	{CCI_REG8(0x2E08),0x00},
	{CCI_REG8(0x2E09),0x00},
	{CCI_REG8(0x2E0A),0x00},
	{CCI_REG8(0x2E0B),0x00},
	{CCI_REG8(0x2E0C),0x00},
	{CCI_REG8(0x2E0D),0x00},
	{CCI_REG8(0x2E0E),0x00},
	{CCI_REG8(0x2E0F),0x00},
	{CCI_REG8(0x2E10),0x01},
	{CCI_REG8(0x2E11),0x00},
	{CCI_REG8(0x2E12),0x0C},
	{CCI_REG8(0x2E13),0x0C},
	{CCI_REG8(0x2E14),0x01},
	{CCI_REG8(0x2E15),0x00},
	{CCI_REG8(0x2E16),0x00},
	{CCI_REG8(0x2E17),0x00},
	{CCI_REG8(0x2E18),0x00},
	{CCI_REG8(0x2E19),0x0C},
	{CCI_REG8(0x2E1A),0x0C},
	{CCI_REG8(0x2E1B),0x00},
	{CCI_REG8(0x2E1C),0x00},
	{CCI_REG8(0x2E1D),0x08},
	{CCI_REG8(0x2E1E),0x00},
	{CCI_REG8(0x2E1F),0x00},
	{CCI_REG8(0x2E20),0x0C},
	{CCI_REG8(0x2E21),0x00},
	{CCI_REG8(0x2E22),0x00},
	{CCI_REG8(0x2E23),0x00},
	{CCI_REG8(0x2E24),0x00},
	{CCI_REG8(0x2E25),0x00},
	{CCI_REG8(0x2E26),0x00},
	{CCI_REG8(0x2E27),0x00},
	{CCI_REG8(0x2E28),0x00},
	{CCI_REG8(0x2E29),0x00},
	{CCI_REG8(0x2E2A),0x00},
	{CCI_REG8(0x2E2B),0x00},
	{CCI_REG8(0x2E2C),0x00},
	{CCI_REG8(0x2E2D),0x00},
	{CCI_REG8(0x2E2E),0x00},
	{CCI_REG8(0x2E2F),0x00},
	{CCI_REG8(0x2E30),0x00},
	{CCI_REG8(0x2E31),0x00},
	{CCI_REG8(0x2E32),0x00},
	{CCI_REG8(0x2E33),0x00},
	{CCI_REG8(0x2E34),0x00},
	{CCI_REG8(0x2E35),0x00},
	{CCI_REG8(0x2E36),0x00},
	{CCI_REG8(0x2E37),0x00},
	{CCI_REG8(0x2E38),0x00},
	{CCI_REG8(0x2E39),0x00},
	{CCI_REG8(0x2E3A),0x00},
	{CCI_REG8(0x2E3B),0x00},
	{CCI_REG8(0x2E3C),0x00},
	{CCI_REG8(0x2E3D),0x05},
	{CCI_REG8(0x2E3E),0x06},
	{CCI_REG8(0x2E3F),0x00},
	{CCI_REG8(0x2E40),0x02},
	{CCI_REG8(0x2E41),0x03},
	{CCI_REG8(0x2E42),0x2A},
	{CCI_REG8(0x2E43),0x04},
	{CCI_REG8(0x2E44),0x04},
	{CCI_REG8(0x2E45),0x04},
	{CCI_REG8(0x2E46),0x04},
	{CCI_REG8(0x2E47),0x06}, //6: FOT
	{CCI_REG8(0x2E48),0x1E},
	{CCI_REG8(0x2E49),0x06},
	{CCI_REG8(0x2E4A),0x00},
	{CCI_REG8(0x2E4B),0x07},
	{CCI_REG8(0x2E4C),0x00},
	{CCI_REG8(0x2E4D),0xA4},
	{CCI_REG8(0x2E4E),0x84},
	{CCI_REG8(0x2E4F),0x62},
	{CCI_REG8(0x2E50),0x02},
	{CCI_REG8(0x2E51),0x50},
	{CCI_REG8(0x2E52),0x9E},
	{CCI_REG8(0x2E53),0x05},
	{CCI_REG8(0x2E54),0x37},
	{CCI_REG8(0x2E55),0xFF},
	{CCI_REG8(0x2E56),0xFF},
	{CCI_REG8(0x2E57),0xFF},
	{CCI_REG8(0x2E58),0xFF},
	{CCI_REG8(0x2E59),0x85},
	{CCI_REG8(0x2E5A),0x94},
	{CCI_REG8(0x2E5B),0x19},
	{CCI_REG8(0x2E5C),0xFF},
	{CCI_REG8(0x2E5D),0xFF},
	{CCI_REG8(0x2E5E),0xFF},
	{CCI_REG8(0x2E5F),0x01},
	{CCI_REG8(0x2E60),0xFD},
	{CCI_REG8(0x2E61),0x01},
	{CCI_REG8(0x2E62),0xFE},
	{CCI_REG8(0x2E63),0xFF},
	{CCI_REG8(0x2E64),0xFF},
	{CCI_REG8(0x2E65),0x05},
	{CCI_REG8(0x2E66),0x3E},
	{CCI_REG8(0x2E67),0x2A},
	{CCI_REG8(0x2E68),0xFE},
	{CCI_REG8(0x2E69),0x05},
	{CCI_REG8(0x2E6A),0x7E},
	{CCI_REG8(0x2E6B),0xBE},
	{CCI_REG8(0x2E6C),0xFA},
	{CCI_REG8(0x2E6D),0x71},
	{CCI_REG8(0x2E6E),0x80},
	{CCI_REG8(0x2E6F),0xC2},
	{CCI_REG8(0x2E70),0xFC},
	{CCI_REG8(0x2E71),0x2A},
	{CCI_REG8(0x2E72),0xFF},
	{CCI_REG8(0x2E73),0xFF},
	{CCI_REG8(0x2E74),0xFF},
	{CCI_REG8(0x2E75),0x2B},
	{CCI_REG8(0x2E76),0xFF},
	{CCI_REG8(0x2E77),0xFF},
	{CCI_REG8(0x2E78),0xFF},
	{CCI_REG8(0x2E79),0x72},
	{CCI_REG8(0x2E7A),0x80},
	{CCI_REG8(0x2E7B),0xC3},
	{CCI_REG8(0x2E7C),0xFC},
	{CCI_REG8(0x2E7D),0x6F},
	{CCI_REG8(0x2E7E),0x70},
	{CCI_REG8(0x2E7F),0xC0},
	{CCI_REG8(0x2E80),0xC1},
	{CCI_REG8(0x2E81),0x72},
	{CCI_REG8(0x2E82),0x80},
	{CCI_REG8(0x2E83),0xC3},
	{CCI_REG8(0x2E84),0xFC},
	{CCI_REG8(0x2E85),0x7E},
	{CCI_REG8(0x2E86),0x80},
	{CCI_REG8(0x2E87),0xFA},
	{CCI_REG8(0x2E88),0xFC},
	{CCI_REG8(0x2E89),0x80},
	{CCI_REG8(0x2E8A),0x8A},
	{CCI_REG8(0x2E8B),0x84},
	{CCI_REG8(0x2E8C),0xFF},
	{CCI_REG8(0x2E8D),0xFF},
	{CCI_REG8(0x2E8E),0xFF},
	{CCI_REG8(0x2E8F),0xFF},
	{CCI_REG8(0x2E90),0xFF},
	{CCI_REG8(0x2E91),0x01},
	{CCI_REG8(0x2E92),0x02},
	{CCI_REG8(0x2E93),0xFF},
	{CCI_REG8(0x2E94),0xFF},
	{CCI_REG8(0x2E95),0xFF},
	{CCI_REG8(0x2E96),0xFF},
	{CCI_REG8(0x2E97),0xFF},
	{CCI_REG8(0x2E98),0xFF},
	{CCI_REG8(0x2E99),0xFF},
	{CCI_REG8(0x2E9A),0xFF},
	{CCI_REG8(0x2E9B),0x24},
	{CCI_REG8(0x2E9C),0x1C},
	{CCI_REG8(0x2E9D),0x1E},
	{CCI_REG8(0x2E9E),0x0C},
	{CCI_REG8(0x2E9F),0x03},
	{CCI_REG8(0x2EA0),0x00},
	{CCI_REG8(0x2EA1),0x01},
	{CCI_REG8(0x2EA2),0x00},
	{CCI_REG8(0x2EA3),0x01},
	{CCI_REG8(0x2EA4),0x03},
	{CCI_REG8(0x2EA5),0x01},
	{CCI_REG8(0x2EA6),0x01},
	{CCI_REG8(0x2EA7),0x00},
	{CCI_REG8(0x2EA8),0x01},
	{CCI_REG8(0x2EA9),0x1E},
	{CCI_REG8(0x2EAA),0x0C},
	{CCI_REG8(0x2EAB),0x00},
	{CCI_REG8(0x2EAC),0x00},
	{CCI_REG8(0x2EAD),0x01},
	{CCI_REG8(0x2EAE),0x12},
	{CCI_REG8(0x2EAF),0x02},
	{CCI_REG8(0x2EB0),0x00},
	{CCI_REG8(0x2EB1),0x00},
	{CCI_REG8(0x2EB2),0x00},
	{CCI_REG8(0x2EB3),0x00},
	{CCI_REG8(0x2EB4),0x00},
	{CCI_REG8(0x2EB5),0x00},
	{CCI_REG8(0x2EB6),0x00},
	{CCI_REG8(0x2EB7),0x00},
	{CCI_REG8(0x2EB8),0x00},
	{CCI_REG8(0x2EB9),0x05},
	{CCI_REG8(0x3000),0x01},
	{CCI_REG8(0x3001),0x02},
	{CCI_REG8(0x3002),0x02},
	{CCI_REG8(0x3003),0x00},
	{CCI_REG8(0x3004),0x03},
	{CCI_REG8(0x3005),0x00},
	{CCI_REG8(0x3006),0x00},
	{CCI_REG8(0x3007),0x00},
	{CCI_REG8(0x3008),0x00},
	{CCI_REG8(0x3009),0x08},
	{CCI_REG8(0x300A),0x00},
	{CCI_REG8(0x300B),0x01},
	{CCI_REG8(0x300C),0x01},
	{CCI_REG8(0x300D),0x0F},
	{CCI_REG8(0x300E),0x00},
	{CCI_REG8(0x300F),0x11},
	{CCI_REG8(0x3010),0x01},
	{CCI_REG8(0x3011),0x01},
	{CCI_REG8(0x3012),0x00},
	{CCI_REG8(0x3013),0x00},
	{CCI_REG8(0x3014),0x65},
	{CCI_REG8(0x3015),0x01},
	{CCI_REG8(0x3016),0x04},
	{CCI_REG8(0x3017),0x00},
	{CCI_REG8(0x3018),0x08},
	{CCI_REG8(0x3019),0x07},
	{CCI_REG8(0x301A),0x10},
	{CCI_REG8(0x301B),0x07},
	{CCI_REG8(0x301C),0x27},
	{CCI_REG8(0x301D),0x00},
	{CCI_REG8(0x301E),0x0B},
	{CCI_REG8(0x301F),0x09},
	{CCI_REG8(0x3020),0x05},
	{CCI_REG8(0x3021),0x06},
	{CCI_REG8(0x3022),0x96},
	{CCI_REG8(0x3023),0xF8},
	{CCI_REG8(0x3024),0x14},
	{CCI_REG8(0x3025),0x00},
	{CCI_REG8(0x3026),0x00},
	{CCI_REG8(0x3027),0x00},
	{CCI_REG8(0x3028),0x00},
	{CCI_REG8(0x3029),0x00},
	{CCI_REG8(0x302A),0x04},
	{CCI_REG8(0x302B),0x04},
	{CCI_REG8(0x302C),0x04},
	{CCI_REG8(0x302D),0x04},
	{CCI_REG8(0x302E),0x00},
	{CCI_REG8(0x302F),0x00},
	{CCI_REG8(0x3030),0x00},
	{CCI_REG8(0x3031),0x00},
	{CCI_REG8(0x3200),0x20},
	{CCI_REG8(0x3201),0x00},
	{CCI_REG8(0x3202),0x04},
	{CCI_REG8(0x3203),0x03},
	{CCI_REG8(0x3204),0x20},
	{CCI_REG8(0x3205),0x03},
	{CCI_REG8(0x3206),0x03},
	{CCI_REG8(0x3207),0x03},
	{CCI_REG8(0x3208),0x16},
	{CCI_REG8(0x3209),0x04},
	{CCI_REG8(0x320A),0x00},
	{CCI_REG8(0x320B),0x16},
	{CCI_REG8(0x320C),0x04},
	{CCI_REG8(0x320D),0x16},
	{CCI_REG8(0x320E),0x0F},
	{CCI_REG8(0x320F),0x00},
	{CCI_REG8(0x3210),0x11},
	{CCI_REG8(0x3211),0x07},
	{CCI_REG8(0x3212),0x00},
	{CCI_REG8(0x3213),0x0E},
	{CCI_REG8(0x3214),0x1B},
	{CCI_REG8(0x3215),0x03},
	{CCI_REG8(0x3216),0x21},
	{CCI_REG8(0x3217),0x04},
	{CCI_REG8(0x3218),0x07},
	{CCI_REG8(0x3219),0x00},
	{CCI_REG8(0x321A),0x29},
	{CCI_REG8(0x321B),0x07},
	{CCI_REG8(0x321C),0x00},
	{CCI_REG8(0x321D),0x04},
	{CCI_REG8(0x321E),0x29},
	{CCI_REG8(0x321F),0x04},
	{CCI_REG8(0x3220),0x07},
	{CCI_REG8(0x3221),0x00},
	{CCI_REG8(0x3222),0x0F},
	{CCI_REG8(0x3223),0x03},
	{CCI_REG8(0x3224),0x00},
	{CCI_REG8(0x3225),0x00},
	{CCI_REG8(0x3226),0x29},
	{CCI_REG8(0x3227),0x03},
	{CCI_REG8(0x3228),0x00},
	{CCI_REG8(0x3229),0x00},
	{CCI_REG8(0x322A),0x11},
	{CCI_REG8(0x322B),0x03},
	{CCI_REG8(0x322C),0x1B},
	{CCI_REG8(0x322D),0x00},
	{CCI_REG8(0x322E),0x07},
	{CCI_REG8(0x322F),0x0F},
	{CCI_REG8(0x3230),0x0E},
	{CCI_REG8(0x3231),0x61},
	{CCI_REG8(0x3232),0x53},
	{CCI_REG8(0x3233),0x53},
	{CCI_REG8(0x3234),0x46},
	{CCI_REG8(0x3235),0x49},
	{CCI_REG8(0x3236),0x49},
	{CCI_REG8(0x3237),0x47},
	{CCI_REG8(0x3238),0x4F},
	{CCI_REG8(0x3239),0x47},
	{CCI_REG8(0x323A),0x53},
	{CCI_REG8(0x323B),0x53},
	{CCI_REG8(0x323C),0x49},
	{CCI_REG8(0x323D),0x49},
	{CCI_REG8(0x323E),0x68},
	{CCI_REG8(0x323F),0x64},
	{CCI_REG8(0x3240),0x00},
	{CCI_REG8(0x3241),0x1A},
	{CCI_REG8(0x3242),0x1C},
	{CCI_REG8(0x3243),0x04},
	{CCI_REG8(0x3244),0x13},
	{CCI_REG8(0x3245),0x13},
	{CCI_REG8(0x3246),0x32},
	{CCI_REG8(0x3247),0x08},
	{CCI_REG8(0x3248),0x7D},
	{CCI_REG8(0x3249),0x20},
	{CCI_REG8(0x324A),0x0F},
	{CCI_REG8(0x324B),0x53},
	{CCI_REG8(0x324C),0x84},
	{CCI_REG8(0x324D),0x1C},
	{CCI_REG8(0x324E),0x28},
	{CCI_REG8(0x324F),0x07},
	{CCI_REG8(0x3250),0x04},
	{CCI_REG8(0x3251),0x00},
	{CCI_REG8(0x3252),0x0F},
	{CCI_REG8(0x3253),0x00},
	{CCI_REG8(0x3254),0x00},
	{CCI_REG8(0x3255),0x00},
	{CCI_REG8(0x3256),0x00},
	{CCI_REG8(0x3300),0x00},
	{CCI_REG8(0x3301),0x00},
	{CCI_REG8(0x3302),0x00},
	{CCI_REG8(0x3303),0x00},
	{CCI_REG8(0x3304),0x00},
	{CCI_REG8(0x3305),0x00},
	{CCI_REG8(0x3306),0x00},
	{CCI_REG8(0x3307),0x01},
	{CCI_REG8(0x3308),0x00},
	{CCI_REG8(0x3309),0x01},
	{CCI_REG8(0x330A),0x00},
	{CCI_REG8(0x330B),0x01},
	{CCI_REG8(0x330C),0x00},
	{CCI_REG8(0x330D),0x01},
	{CCI_REG8(0x330E),0x00},
	{CCI_REG8(0x330F),0x01},
	{CCI_REG8(0x3310),0x00},
	{CCI_REG8(0x3311),0x01},
	{CCI_REG8(0x3312),0x00},
	{CCI_REG8(0x3313),0x2C},
	{CCI_REG8(0x3314),0x01},
	{CCI_REG8(0x3315),0x01},
	{CCI_REG8(0x3316),0x00},
	{CCI_REG8(0x3317),0xE2},
	{CCI_REG8(0x3318),0x04},
	{CCI_REG8(0x3319),0xE2},
	{CCI_REG8(0x331A),0x04},
	{CCI_REG8(0x331B),0xE2},
	{CCI_REG8(0x331C),0x04},
	{CCI_REG8(0x331D),0xE2},
	{CCI_REG8(0x331E),0x04},
	{CCI_REG8(0x331F),0xE2},
	{CCI_REG8(0x3320),0x04},
	{CCI_REG8(0x3321),0xE2},
	{CCI_REG8(0x3322),0x04},
	{CCI_REG8(0x3323),0xEE},
	{CCI_REG8(0x3324),0x02},
	{CCI_REG8(0x3325),0xEE},
	{CCI_REG8(0x3326),0x02},
	{CCI_REG8(0x3327),0x01},
	{CCI_REG8(0x3328),0x00},
	{CCI_REG8(0x3329),0x01},
	{CCI_REG8(0x332A),0x00},
	{CCI_REG8(0x332B),0x01},
	{CCI_REG8(0x332C),0x00},
	{CCI_REG8(0x332D),0x01},
	{CCI_REG8(0x332E),0x00},
	{CCI_REG8(0x332F),0x77},
	{CCI_REG8(0x3330),0x01},
	{CCI_REG8(0x3331),0x01},
	{CCI_REG8(0x3332),0x00},
	{CCI_REG8(0x3333),0xE2},
	{CCI_REG8(0x3334),0x04},
	{CCI_REG8(0x3335),0xE2},
	{CCI_REG8(0x3336),0x04},
	{CCI_REG8(0x3337),0xE2},
	{CCI_REG8(0x3338),0x04},
	{CCI_REG8(0x3339),0xE2},
	{CCI_REG8(0x333A),0x04},
	{CCI_REG8(0x333B),0xE2},
	{CCI_REG8(0x333C),0x04},
	{CCI_REG8(0x333D),0xE2},
	{CCI_REG8(0x333E),0x04},
	{CCI_REG8(0x333F),0x01},
	{CCI_REG8(0x3340),0x00},
	{CCI_REG8(0x3341),0x01},
	{CCI_REG8(0x3342),0x00},
	{CCI_REG8(0x3343),0x01},
	{CCI_REG8(0x3344),0x00},
	{CCI_REG8(0x3345),0x00},
	{CCI_REG8(0x3346),0x00},
	{CCI_REG8(0x3347),0x00},
	{CCI_REG8(0x3348),0x00},
	{CCI_REG8(0x3349),0x00},
	{CCI_REG8(0x334A),0x00},
	{CCI_REG8(0x334B),0x00},
	{CCI_REG8(0x334C),0x00},
	{CCI_REG8(0x334D),0x10},
	{CCI_REG8(0x334E),0x32},
	{CCI_REG8(0x334F),0x54},
	{CCI_REG8(0x3350),0x86},
	{CCI_REG8(0x3351),0xA9},
	{CCI_REG8(0x3352),0xCB},
	{CCI_REG8(0x3353),0xED},
	{CCI_REG8(0x3354),0x7F},
	{CCI_REG8(0x3355),0x10},
	{CCI_REG8(0x3356),0x32},
	{CCI_REG8(0x3357),0x54},
	{CCI_REG8(0x3358),0x87},
	{CCI_REG8(0x3359),0xA9},
	{CCI_REG8(0x335A),0xCB},
	{CCI_REG8(0x335B),0xED},
	{CCI_REG8(0x335C),0x5F},
	{CCI_REG8(0x335D),0x00},
	{CCI_REG8(0x335E),0x00},
	{CCI_REG8(0x335F),0x00},
	{CCI_REG8(0x3360),0x00},
	{CCI_REG8(0x3361),0x01},
	{CCI_REG8(0x3362),0x00},
	{CCI_REG8(0x3363),0x76},
	{CCI_REG8(0x3364),0xE8},
	{CCI_REG8(0x3365),0x03},
	{CCI_REG8(0x3366),0xE8},
	{CCI_REG8(0x3367),0x03},
	{CCI_REG8(0x3368),0x88},
	{CCI_REG8(0x3369),0x13},
	{CCI_REG8(0x336A),0x88},
	{CCI_REG8(0x336B),0x13},
	{CCI_REG8(0x336C),0x00},
	{CCI_REG8(0x3400),0x00},
	{CCI_REG8(0x3401),0x12},
	{CCI_REG8(0x3402),0x00},
	{CCI_REG8(0x3403),0x01},
	{CCI_REG8(0x3404),0x64},
	{CCI_REG8(0x3405),0x02},
	{CCI_REG8(0x3500),0x10},
	{CCI_REG8(0x3501),0x86},
	{CCI_REG8(0x3502),0x00},
	{CCI_REG8(0x3503),0x00},
	{CCI_REG8(0x3504),0x00},
	{CCI_REG8(0x3505),0x00},
	{CCI_REG8(0x3506),0x00},
	{CCI_REG8(0x3507),0x00},
	{CCI_REG8(0x3508),0x00},
	{CCI_REG8(0x3509),0x00},
	{CCI_REG8(0x350A),0x00},
	{CCI_REG8(0x350B),0x00},
	{CCI_REG8(0x350C),0x00},
	{CCI_REG8(0x350D),0x00},
	{CCI_REG8(0x350E),0x00},
	{CCI_REG8(0x350F),0x00},
	{CCI_REG8(0x3510),0x00},
	{CCI_REG8(0x3511),0x00},
	{CCI_REG8(0x3512),0x10},
	{CCI_REG8(0x3513),0x00},
	{CCI_REG8(0x3514),0x00},
	{CCI_REG8(0x3515),0x00},
	{CCI_REG8(0x3516),0x00},
	{CCI_REG8(0x3517),0x00},
	{CCI_REG8(0x3518),0x00},
	{CCI_REG8(0x3519),0x00},
	{CCI_REG8(0x351A),0x00},
	{CCI_REG8(0x351B),0x00},
	{CCI_REG8(0x351C),0x00},
	{CCI_REG8(0x351D),0x00},
	{CCI_REG8(0x351E),0x00},
	{CCI_REG8(0x351F),0x00},
	{CCI_REG8(0x3520),0x00},
	{CCI_REG8(0x3521),0x00},
	{CCI_REG8(0x3522),0x10},
	{CCI_REG8(0x3523),0x52},
	{CCI_REG8(0x3524),0x04},
	{CCI_REG8(0x3525),0x00},
	{CCI_REG8(0x3526),0x00},
	{CCI_REG8(0x3527),0x00},
	{CCI_REG8(0x3528),0x00},
	{CCI_REG8(0x3700),0xE0}, //Sub-OB Calibration = off
	{CCI_REG8(0x3701),0x08},
	{CCI_REG8(0x3702),0x6E},
	{CCI_REG8(0x3703),0x0D},
	{CCI_REG8(0x3704),0x00},
	{CCI_REG8(0x3705),0x80},
	{CCI_REG8(0x3706),0x00},
	{CCI_REG8(0x3707),0xC8},
	{CCI_REG8(0x3708),0x00},
	{CCI_REG8(0x3709),0xF9},
	{CCI_REG8(0x370A),0x32},
	{CCI_REG8(0x370B),0x00},
	{CCI_REG8(0x370C),0x02},
	{CCI_REG8(0x370D),0x24},
	{CCI_REG8(0x370E),0x3F},
	{CCI_REG8(0x370F),0x0F},
	{CCI_REG8(0x3710),0x64},
	{CCI_REG8(0x3711),0x00},
	{CCI_REG8(0x3712),0x00},
	{CCI_REG8(0x3713),0x3A},
	{CCI_REG8(0x3800),0xE0}, //Sub-OB Calibration = off
	{CCI_REG8(0x3801),0x08},
	{CCI_REG8(0x3802),0x6E},
	{CCI_REG8(0x3803),0x0D},
	{CCI_REG8(0x3804),0x00},
	{CCI_REG8(0x3805),0x80},
	{CCI_REG8(0x3806),0x00},
	{CCI_REG8(0x3807),0xC8},
	{CCI_REG8(0x3808),0x00},
	{CCI_REG8(0x3809),0xF9},
	{CCI_REG8(0x380A),0x32},
	{CCI_REG8(0x380B),0x00},
	{CCI_REG8(0x380C),0x02},
	{CCI_REG8(0x380D),0x24},
	{CCI_REG8(0x380E),0x3F},
	{CCI_REG8(0x380F),0x0F},
	{CCI_REG8(0x3810),0x64},
	{CCI_REG8(0x3811),0x00},
	{CCI_REG8(0x3812),0x00},
	{CCI_REG8(0x3813),0x3A}
};


/* Mode description */
struct gmax3412_mode {
    unsigned int width;
    unsigned int height;
    struct v4l2_rect crop;
    struct {
        unsigned int num_of_regs;
        const struct cci_reg_sequence *regs;
    } reg_list;
};

static struct gmax3412_mode supported_modes_12bit[] = {
    {
        .width = GMAX3412_NATIVE_WIDTH,
        .height = GMAX3412_NATIVE_HEIGHT,
        .crop = {
            .left = GMAX3412_PIXEL_ARRAY_LEFT,
            .top = GMAX3412_PIXEL_ARRAY_TOP,
            .width = GMAX3412_PIXEL_ARRAY_WIDTH,
            .height = GMAX3412_PIXEL_ARRAY_HEIGHT,
        },
    },
};

/* Formats exposed per mode/bit depth */
static const u32 codes[] = {
    /* 10-bit modes. */
    MEDIA_BUS_FMT_SGBRG12_1X12,
    MEDIA_BUS_FMT_SBGGR12_1X12,
    MEDIA_BUS_FMT_SRGGB12_1X12,
    MEDIA_BUS_FMT_SGRBG12_1X12,

};

static const u32 mono_codes[] = {
	MEDIA_BUS_FMT_Y12_1X12,
};

/* Regulators */
static const char * const gmax3412_supply_name[] = {
    "vana", /* 3.3V analog */
    "vdig", /* 1.1V core   */
    "vddl", /* 1.8V I/O    */
};

#define GMAX3412_NUM_SUPPLIES ARRAY_SIZE(gmax3412_supply_name)

/* --------------------------------------------------------------------------
 * State
 * --------------------------------------------------------------------------
 */

struct gmax3412 {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct device *dev;
    struct regmap *regmap;

    struct clk *xclk;

    struct gpio_desc *reset_gpio;
    struct regulator_bulk_data supplies[GMAX3412_NUM_SUPPLIES];

    struct v4l2_ctrl_handler ctrl_handler;

    /* Controls */
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *gain;
    struct v4l2_ctrl *vflip;
    struct v4l2_ctrl *hflip;
    struct v4l2_ctrl *vblank;
    struct v4l2_ctrl *hblank;

    bool streaming;
};

/* Helpers */

static inline struct gmax3412 *to_gmax3412(struct v4l2_subdev *sd)
{
    return container_of(sd, struct gmax3412, sd);
}

static inline void get_mode_table(struct gmax3412 *gmax3412, unsigned int code,
                  const struct gmax3412_mode **mode_list,
                  unsigned int *num_modes)
{
    switch (code) {
    case MEDIA_BUS_FMT_SRGGB12_1X12:
    case MEDIA_BUS_FMT_SGRBG12_1X12:
    case MEDIA_BUS_FMT_SGBRG12_1X12:
    case MEDIA_BUS_FMT_SBGGR12_1X12:
        *mode_list = supported_modes_12bit;
        *num_modes = ARRAY_SIZE(supported_modes_12bit);
        break;
    default:
        *mode_list = NULL;
        *num_modes = 0;
    }
}

static u32 gmax3412_get_format_code(struct gmax3412 *gmax3412, u32 code)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(codes); i++)
        if (codes[i] == code)
            return codes[i];
    return codes[0];
}

/* --------------------------------------------------------------------------
 * Controls
 * --------------------------------------------------------------------------
 */

static int gmax3412_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct gmax3412 *gmax3412 = container_of(ctrl->handler, struct gmax3412, ctrl_handler);
    const struct gmax3412_mode *mode, *mode_list;
    struct v4l2_subdev_state *state;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int num_modes;
    int ret = 0;

    /* Apply control only when powered (runtime active). */
    if (!pm_runtime_get_if_active(gmax3412->dev))
        return 0;

    /* Look, I told you this is a bare minimum driver with external exposure contorl
     * The only control that works is analog gain, VFLIP, HFLIP.
     */
    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE: {
        //dev_info(gmax3412->dev, "EXPOSURE=%u -> SHR=%u (VMAX=%u HMAX=%u)\n", ctrl->val, 0, 0, 0);
    }
    case V4L2_CID_ANALOGUE_GAIN:
        dev_info(gmax3412->dev, "ANALOG_GAIN=%u\n", ctrl->val);
        ret = cci_write(gmax3412->regmap, GMAX3412_REG_ANALOG_GAIN, ctrl->val+18, NULL);
        if (ret)
            dev_err_ratelimited(gmax3412->dev, "Gain write failed (%d)\n", ret);
        break;
    case V4L2_CID_VBLANK: {
        //dev_info(gmax3412->dev, "VBLANK=%u -> VMAX=%u\n", ctrl->val, 0);
        break;
    }
    case V4L2_CID_HBLANK: {
        //dev_info(gmax3412->dev, "HBLANK=%u -> HMAX=%u\n", ctrl->val, 0);
        break;
    }
    case V4L2_CID_VFLIP:
    	dev_info(gmax3412->dev, "V4L2_CID_VFLIP=%u\n", ctrl->val);
        cci_update_bits(gmax3412->regmap, GMAX3412_REG_FLIP_V, BIT(0), ctrl->val, &ret);
        break;
    case V4L2_CID_HFLIP:
    	dev_info(gmax3412->dev, "V4L2_CID_HFLIP=%u\n", ctrl->val);
        cci_update_bits(gmax3412->regmap, GMAX3412_REG_FLIP_H, BIT(0), ctrl->val, &ret);
        break;
    case V4L2_CID_BRIGHTNESS: {
        //u16 blacklevel = min_t(u32, ctrl->val, 1023);
        //ret = cci_write(gmax3412->regmap, GMAX3412_REG_BLKLEVEL, blacklevel, NULL);
        break;
    }
    default:
        dev_info(gmax3412->dev, "Unhandled ctrl %s: id=0x%x, val=0x%x\n",
            ctrl->name, ctrl->id, ctrl->val);
        break;
    }

    pm_runtime_put(gmax3412->dev);
    return ret;
}

static const struct v4l2_ctrl_ops gmax3412_ctrl_ops = {
    .s_ctrl = gmax3412_set_ctrl,
};


static int gmax3412_init_controls(struct gmax3412 *gmax3412)
{
    struct v4l2_ctrl_handler *hdl = &gmax3412->ctrl_handler;
    struct v4l2_fwnode_device_properties props;
    int ret;

    ret = v4l2_ctrl_handler_init(hdl, 8);

    /* Read-only, updated per mode */
    gmax3412->pixel_rate = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops,
                           V4L2_CID_PIXEL_RATE,
                           1, GMAX3412_PIXEL_RATE, 1, 1);
    gmax3412->link_freq =
        v4l2_ctrl_new_int_menu(hdl, &gmax3412_ctrl_ops, V4L2_CID_LINK_FREQ,
                       0, 0, gmax3412_link_freq_menu);
    if (gmax3412->link_freq)
        gmax3412->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    gmax3412->vblank = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops,
                       V4L2_CID_VBLANK, 0, 0xFFFFF, 1, 0);
    gmax3412->hblank = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops,
                       V4L2_CID_HBLANK, 0, 0xFFFF, 1, 0);

    gmax3412->exposure = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops,
                         V4L2_CID_EXPOSURE,
                         GMAX3412_EXPOSURE_MIN, GMAX3412_EXPOSURE_MAX,
                         GMAX3412_EXPOSURE_STEP, GMAX3412_EXPOSURE_DEFAULT);

    gmax3412->gain = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                     GMAX3412_ANA_GAIN_MIN, GMAX3412_ANA_GAIN_MAX,
                     GMAX3412_ANA_GAIN_STEP, GMAX3412_ANA_GAIN_DEFAULT);

    gmax3412->vflip = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops,
                      V4L2_CID_VFLIP, 0, 1, 1, 0);

    gmax3412->hflip = v4l2_ctrl_new_std(hdl, &gmax3412_ctrl_ops,
                      V4L2_CID_HFLIP, 0, 1, 1, 0);
    if (hdl->error) {
        ret = hdl->error;
        dev_err(gmax3412->dev, "control init failed (%d)\n", ret);
        goto err_free;
    }

    ret = v4l2_fwnode_device_parse(gmax3412->dev, &props);
    if (ret)
        goto err_free;

    ret = v4l2_ctrl_new_fwnode_properties(hdl, &gmax3412_ctrl_ops, &props);
    if (ret)
        goto err_free;

    gmax3412->sd.ctrl_handler = hdl;
    return 0;

err_free:
    v4l2_ctrl_handler_free(hdl);
    return ret;
}

static void gmax3412_free_controls(struct gmax3412 *gmax3412)
{
    v4l2_ctrl_handler_free(gmax3412->sd.ctrl_handler);
}

/* --------------------------------------------------------------------------
 * Pad ops / formats
 * --------------------------------------------------------------------------
 */

static int gmax3412_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    unsigned int entries;
    const u32 *tbl;

    tbl = codes;
    entries = ARRAY_SIZE(codes) / 4;

    if (code->index >= entries)
        return -EINVAL;

    code->code = gmax3412_get_format_code(gmax3412, tbl[code->index * 4]);
    return 0;
}

static int gmax3412_enum_frame_size(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *sd_state,
                  struct v4l2_subdev_frame_size_enum *fse)
{
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    const struct gmax3412_mode *mode_list;
    unsigned int num_modes;

    get_mode_table(gmax3412, fse->code, &mode_list, &num_modes);
    if (fse->index >= num_modes)
        return -EINVAL;
    if (fse->code != gmax3412_get_format_code(gmax3412, fse->code))
        return -EINVAL;

    fse->min_width  = mode_list[fse->index].width;
    fse->max_width  = fse->min_width;
    fse->min_height = mode_list[fse->index].height;
    fse->max_height = fse->min_height;

    return 0;
}

static int gmax3412_set_pad_format(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *sd_state,
                 struct v4l2_subdev_format *fmt)
{
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    const struct gmax3412_mode *mode_list, *mode;
    unsigned int num_modes;
    struct v4l2_mbus_framefmt *format;
    struct v4l2_rect *crop;

    /* Normalize requested code to what we really support */
    fmt->format.code = gmax3412_get_format_code(gmax3412, fmt->format.code);

    get_mode_table(gmax3412, fmt->format.code, &mode_list, &num_modes);
    mode = v4l2_find_nearest_size(mode_list, num_modes, width, height,
                                  fmt->format.width, fmt->format.height);

    fmt->format.width        = mode->width;
    fmt->format.height       = mode->height;
    fmt->format.field        = V4L2_FIELD_NONE;
    fmt->format.colorspace   = V4L2_COLORSPACE_RAW;
    fmt->format.ycbcr_enc    = V4L2_YCBCR_ENC_601;
    fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    fmt->format.xfer_func    = V4L2_XFER_FUNC_NONE;

    /* Update TRY/ACTIVE format kept by the framework */
    format = v4l2_subdev_state_get_format(sd_state, 0);
    *format = fmt->format;

    /* Keep the crop in sync with the selected mode */
    crop = v4l2_subdev_state_get_crop(sd_state, 0);
    *crop = mode->crop;

    return 0;
}



/* --------------------------------------------------------------------------
 * Stream on/off
 * --------------------------------------------------------------------------
 */

static int gmax3412_enable_streams(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state, u32 pad,
                 u64 streams_mask)
{
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    const struct gmax3412_mode *mode_list, *mode;
    struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;
    int ret;

    ret = pm_runtime_get_sync(gmax3412->dev);
    if (ret < 0) {
        pm_runtime_put_noidle(gmax3412->dev);
        return ret;
    }

    ret = cci_multi_reg_write(gmax3412->regmap, mode_common_regs,
                  ARRAY_SIZE(mode_common_regs), NULL);
    if (ret) {
        dev_err(gmax3412->dev, "Failed to write common settings\n");
        goto err_rpm_put;
    }

    usleep_range(10000,12000);
    //CLK_STABLE_EN = 1
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x2E00), BIT(0), BIT(0), &ret);
    usleep_range(10000,12000);
    //PWR_UP_EN = 1
    cci_write(gmax3412->regmap, CCI_REG8(0x3301), 0x01, &ret);
    usleep_range(10000,12000);

    //CH_CLK_EN =1
    cci_write(gmax3412->regmap, CCI_REG8(0x3005), 0x01, &ret);
    //PHY CAL
    /* D<0x3024>_Bit<5>=0 */
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x3024), BIT(5), 0, &ret);
    /* D<0x3023>_Bit<7:4>=b'1111 */
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x3023), GENMASK(7, 4),FIELD_PREP(GENMASK(7, 4), 0xF), &ret);
    /* D<0x3023>_Bit<0>=1 */
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x3023), BIT(0), BIT(0), &ret);
    /* D<0x3023>_Bit<2:1>=2'b11 */
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x3023), GENMASK(2, 1),FIELD_PREP(GENMASK(2, 1), 0x3), &ret);
    /* D<0x3024>_Bit<5>=1 */
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x3024), BIT(5), BIT(5), &ret);
    /* D<0x3023>_Bit<2>=0 */
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x3023), BIT(2), 0, &ret);

    //STREAM_EN = 1
    cci_update_bits(gmax3412->regmap, CCI_REG8(0x2E00), BIT(1), BIT(1), &ret);


    /* Apply user controls after writing the base tables */
    ret = __v4l2_ctrl_handler_setup(gmax3412->sd.ctrl_handler);
    if (ret) {
        dev_err(gmax3412->dev, "Control handler setup failed\n");
        goto err_rpm_put;
    }

    dev_info(gmax3412->dev, "Streaming started\n");
    usleep_range(GMAX3412_STREAM_DELAY_US,
             GMAX3412_STREAM_DELAY_US + GMAX3412_STREAM_DELAY_RANGE_US);

    /* vflip cannot change during streaming */
    __v4l2_ctrl_grab(gmax3412->vflip, true);
    __v4l2_ctrl_grab(gmax3412->hflip, true);

    return 0;

err_rpm_put:
    pm_runtime_put_autosuspend(gmax3412->dev);
    return ret;
}

static int gmax3412_disable_streams(struct v4l2_subdev *sd,
                  struct v4l2_subdev_state *state, u32 pad,
                  u64 streams_mask)
{
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    int ret;

    __v4l2_ctrl_grab(gmax3412->vflip, false);
    __v4l2_ctrl_grab(gmax3412->hflip, false);

    pm_runtime_put_autosuspend(gmax3412->dev);

    return ret;
}

/* --------------------------------------------------------------------------
 * Power / runtime PM
 * --------------------------------------------------------------------------
 */

static int gmax3412_power_on(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    int ret;

    dev_info(gmax3412->dev, "power_on\n");

    ret = regulator_bulk_enable(GMAX3412_NUM_SUPPLIES, gmax3412->supplies);
    if (ret) {
        dev_err(gmax3412->dev, "Failed to enable regulators\n");
        return ret;
    }

    ret = clk_prepare_enable(gmax3412->xclk);
    if (ret) {
        dev_err(gmax3412->dev, "Failed to enable clock\n");
        goto reg_off;
    }

    gpiod_set_value_cansleep(gmax3412->reset_gpio, 1);
    usleep_range(GMAX3412_XCLR_MIN_DELAY_US,
             GMAX3412_XCLR_MIN_DELAY_US + GMAX3412_XCLR_DELAY_RANGE_US);
    return 0;

reg_off:
    regulator_bulk_disable(GMAX3412_NUM_SUPPLIES, gmax3412->supplies);
    return ret;
}

static int gmax3412_power_off(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct gmax3412 *gmax3412 = to_gmax3412(sd);

    dev_info(gmax3412->dev, "power_off\n");

    gpiod_set_value_cansleep(gmax3412->reset_gpio, 0);
    regulator_bulk_disable(GMAX3412_NUM_SUPPLIES, gmax3412->supplies);
    clk_disable_unprepare(gmax3412->xclk);

    return 0;
}

/* --------------------------------------------------------------------------
 * Selection / state
 * --------------------------------------------------------------------------
 */

static int gmax3412_get_selection(struct v4l2_subdev *sd,
                struct v4l2_subdev_state *sd_state,
                struct v4l2_subdev_selection *sel)
{
    struct gmax3412 *gmax3412 = to_gmax3412(sd);
    const struct gmax3412_mode *mode_list, *mode;
    const struct v4l2_mbus_framefmt *fmt;
    unsigned int n_modes;

    fmt = v4l2_subdev_state_get_format(sd_state, 0);
    get_mode_table(gmax3412, fmt->code, &mode_list, &n_modes);
    mode = v4l2_find_nearest_size(mode_list, n_modes, width, height,
                                  fmt->width, fmt->height);

    switch (sel->target) {


    case V4L2_SEL_TGT_NATIVE_SIZE:
        sel->r.left   = 0;
        sel->r.top    = 0;
        sel->r.width  = GMAX3412_NATIVE_WIDTH;   /* pixel array (no blanking) */
        sel->r.height = GMAX3412_NATIVE_HEIGHT;
        return 0;
    case V4L2_SEL_TGT_CROP_BOUNDS:
    case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = GMAX3412_PIXEL_ARRAY_TOP;
		sel->r.left = GMAX3412_PIXEL_ARRAY_LEFT;
		sel->r.width = GMAX3412_PIXEL_ARRAY_WIDTH;
		sel->r.height = GMAX3412_PIXEL_ARRAY_HEIGHT;
        return 0;

    case V4L2_SEL_TGT_CROP:
        sel->r = *v4l2_subdev_state_get_crop(sd_state, 0);
        return 0;

    default:
        return -EINVAL;
    }
}

static int gmax3412_init_state(struct v4l2_subdev *sd,
                 struct v4l2_subdev_state *state)
{
    struct v4l2_rect *crop;
    struct v4l2_subdev_format fmt = {
        .which  = V4L2_SUBDEV_FORMAT_TRY,
        .pad    = 0,
        .format = {
            .code   = MEDIA_BUS_FMT_SGBRG12_1X12,
            .width  = GMAX3412_NATIVE_WIDTH,
            .height = GMAX3412_NATIVE_HEIGHT,
        },
    };

    gmax3412_set_pad_format(sd, state, &fmt);

    crop = v4l2_subdev_state_get_crop(state, 0);
    *crop = supported_modes_12bit[0].crop;

    return 0;
}

/* --------------------------------------------------------------------------
 * Subdev ops
 * --------------------------------------------------------------------------
 */

static const struct v4l2_subdev_video_ops gmax3412_video_ops = {
    .s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops gmax3412_pad_ops = {
    .enum_mbus_code = gmax3412_enum_mbus_code,
    .get_fmt        = v4l2_subdev_get_fmt,
    .set_fmt        = gmax3412_set_pad_format,
    .get_selection  = gmax3412_get_selection,
    .enum_frame_size = gmax3412_enum_frame_size,
    .enable_streams  = gmax3412_enable_streams,
    .disable_streams = gmax3412_disable_streams,
};

static const struct v4l2_subdev_internal_ops gmax3412_internal_ops = {
    .init_state = gmax3412_init_state,
};

static const struct v4l2_subdev_ops gmax3412_subdev_ops = {
    .video = &gmax3412_video_ops,
    .pad   = &gmax3412_pad_ops,
};

/* --------------------------------------------------------------------------
 * Probe / remove
 * --------------------------------------------------------------------------
 */

static int gmax3412_check_hwcfg(struct device *dev, struct gmax3412 *gmax3412)
{
    struct fwnode_handle *endpoint;
    struct v4l2_fwnode_endpoint ep = {
        .bus_type = V4L2_MBUS_CSI2_DPHY,
    };
    int ret = -EINVAL;

    endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!endpoint) {
        dev_err(dev, "endpoint node not found\n");
        return -EINVAL;
    }

    if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep)) {
        dev_err(dev, "could not parse endpoint\n");
        goto out_put;
    }

    if (ep.bus.mipi_csi2.num_data_lanes != 4) {
        dev_err(dev, "only 4 data lanes supported\n");
        goto out_free;
    }

    ret = 0;

out_free:
    v4l2_fwnode_endpoint_free(&ep);
out_put:
    fwnode_handle_put(endpoint);
    return ret;
}

static int gmax3412_get_regulators(struct gmax3412 *gmax3412)
{
    unsigned int i;

    for (i = 0; i < GMAX3412_NUM_SUPPLIES; i++)
        gmax3412->supplies[i].supply = gmax3412_supply_name[i];

    return devm_regulator_bulk_get(gmax3412->dev,
                       GMAX3412_NUM_SUPPLIES, gmax3412->supplies);
}

static int gmax3412_check_module_exists(struct gmax3412 *gmax3412)
{
    int ret;
    u64 val;

    /* No chip-id register; read a known register as a presence test */
    ret = cci_read(gmax3412->regmap, GMAX3412_REG_BLKLEVEL, &val, NULL);
    if (ret) {
        dev_err(gmax3412->dev, "register read failed (%d)\n", ret);
        return ret;
    }

    dev_info(gmax3412->dev, "Sensor detected\n");
    return 0;
}

static int gmax3412_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct gmax3412 *gmax3412;
    unsigned int xclk_freq;
    int ret, i;
    const char *sync_mode;

    gmax3412 = devm_kzalloc(dev, sizeof(*gmax3412), GFP_KERNEL);
    if (!gmax3412)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&gmax3412->sd, client, &gmax3412_subdev_ops);
    gmax3412->dev = dev;

    ret = gmax3412_check_hwcfg(dev, gmax3412);
    if (ret)
        return ret;

    gmax3412->regmap = devm_cci_regmap_init_i2c(client, 16);
    if (IS_ERR(gmax3412->regmap))
        return dev_err_probe(dev, PTR_ERR(gmax3412->regmap), "CCI init failed\n");

    gmax3412->xclk = devm_clk_get(dev, NULL);
    if (IS_ERR(gmax3412->xclk))
        return dev_err_probe(dev, PTR_ERR(gmax3412->xclk), "xclk missing\n");

    xclk_freq = clk_get_rate(gmax3412->xclk);

    ret = gmax3412_get_regulators(gmax3412);
    if (ret)
        return dev_err_probe(dev, ret, "regulators\n");

    gmax3412->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);

    /* Power on to probe the device */
    ret = gmax3412_power_on(dev);
    if (ret)
        return ret;

    ret = gmax3412_check_module_exists(gmax3412);
    if (ret)
        goto err_power_off;

    pm_runtime_set_active(dev);
    pm_runtime_get_noresume(dev);
    pm_runtime_enable(dev);
    pm_runtime_set_autosuspend_delay(dev, 1000);
    pm_runtime_use_autosuspend(dev);

    ret = gmax3412_init_controls(gmax3412);
    if (ret)
        goto err_pm;

    gmax3412->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    gmax3412->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    gmax3412->sd.internal_ops = &gmax3412_internal_ops;

    gmax3412->pad.flags = MEDIA_PAD_FL_SOURCE;

    ret = media_entity_pads_init(&gmax3412->sd.entity, 1, &gmax3412->pad);
    if (ret) {
        dev_err(dev, "entity pads init failed: %d\n", ret);
        goto err_ctrls;
    }

    gmax3412->sd.state_lock = gmax3412->ctrl_handler.lock;
    ret = v4l2_subdev_init_finalize(&gmax3412->sd);
    if (ret) {
        dev_err_probe(dev, ret, "subdev init\n");
        goto err_entity;
    }

    ret = v4l2_async_register_subdev_sensor(&gmax3412->sd);
    if (ret) {
        dev_err(dev, "sensor subdev register failed: %d\n", ret);
        goto err_entity;
    }

    pm_runtime_mark_last_busy(dev);
    pm_runtime_put_autosuspend(dev);
    return 0;

err_entity:
    media_entity_cleanup(&gmax3412->sd.entity);
err_ctrls:
    gmax3412_free_controls(gmax3412);
err_pm:
    pm_runtime_disable(dev);
    pm_runtime_set_suspended(dev);
err_power_off:
    gmax3412_power_off(dev);
    return ret;
}

static void gmax3412_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct gmax3412 *gmax3412 = to_gmax3412(sd);

    v4l2_async_unregister_subdev(sd);
    v4l2_subdev_cleanup(sd);
    media_entity_cleanup(&sd->entity);
    gmax3412_free_controls(gmax3412);

    pm_runtime_disable(gmax3412->dev);
    if (!pm_runtime_status_suspended(gmax3412->dev))
        gmax3412_power_off(gmax3412->dev);
    pm_runtime_set_suspended(gmax3412->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(gmax3412_pm_ops, gmax3412_power_off,
                 gmax3412_power_on, NULL);

static const struct of_device_id gmax3412_of_match[] = {
    { .compatible = "gpixel,gmax3412" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gmax3412_of_match);

static struct i2c_driver gmax3412_i2c_driver = {
    .driver = {
        .name  = "gmax3412",
        .pm    = pm_ptr(&gmax3412_pm_ops),
        .of_match_table = gmax3412_of_match,
    },
    .probe  = gmax3412_probe,
    .remove = gmax3412_remove,
};
module_i2c_driver(gmax3412_i2c_driver);

MODULE_AUTHOR("Will Whang <will@willwhang.com>");
MODULE_DESCRIPTION("Gpixel GMAX3412 sensor driver");
MODULE_LICENSE("GPL");

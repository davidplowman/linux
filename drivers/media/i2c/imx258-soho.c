// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX258 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define IMX258_REG_VALUE_08BIT		1
#define IMX258_REG_VALUE_16BIT		2

/* Chip ID */
#define IMX258_REG_CHIP_ID		0x0016
#define IMX258_CHIP_ID			0x0258

#define IMX258_REG_MODE_SELECT		0x0100
#define IMX258_MODE_STANDBY		0x00
#define IMX258_MODE_STREAMING		0x01

#define IMX258_REG_ORIENTATION		0x101

#define IMX258_XCLK_FREQ		24000000

#define IMX258_DEFAULT_LINK_FREQ	450000000

/* Pixel rate is fixed at 840MHz for all the modes */
#define IMX258_PIXEL_RATE		(518400000/2)

/* V_TIMING internal */
#define IMX258_REG_FRAME_LENGTH		0x0340
#define IMX258_FRAME_LENGTH_MAX		0xffdc

/* Long exposure multiplier */
#define IMX258_LONG_EXP_SHIFT_MAX	7
#define IMX258_LONG_EXP_SHIFT_REG	0x3100

/* Exposure control */
#define IMX258_REG_EXPOSURE		0x0202
#define IMX258_EXPOSURE_OFFSET		22
#define IMX258_EXPOSURE_MIN		20
#define IMX258_EXPOSURE_STEP		1
#define IMX258_EXPOSURE_DEFAULT		0x640
#define IMX258_EXPOSURE_MAX		(IMX258_FRAME_LENGTH_MAX - \
					 IMX258_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX258_REG_ANALOG_GAIN		0x0204
#define IMX258_ANA_GAIN_MIN		0
#define IMX258_ANA_GAIN_MAX		978
#define IMX258_ANA_GAIN_STEP		1
#define IMX258_ANA_GAIN_DEFAULT		0x0

/* Digital gain control */
#define IMX258_REG_GR_DIGITAL_GAIN	0x020e
#define IMX258_REG_R_DIGITAL_GAIN	0x0210
#define IMX258_REG_B_DIGITAL_GAIN	0x0212
#define IMX258_REG_GB_DIGITAL_GAIN	0x0214
#define IMX258_DGTL_GAIN_MIN		0x0100
#define IMX258_DGTL_GAIN_MAX		4096
#define IMX258_DGTL_GAIN_DEFAULT	1024
#define IMX258_DGTL_GAIN_STEP		1

/* Test Pattern Control */
#define IMX258_REG_TEST_PATTERN		0x0600
#define IMX258_TEST_PATTERN_DISABLE	0
#define IMX258_TEST_PATTERN_SOLID_COLOR	1
#define IMX258_TEST_PATTERN_COLOR_BARS	2
#define IMX258_TEST_PATTERN_GREY_COLOR	3
#define IMX258_TEST_PATTERN_PN9		4

/* Test pattern colour components */
#define IMX258_REG_TEST_PATTERN_R	0x0602
#define IMX258_REG_TEST_PATTERN_GR	0x0604
#define IMX258_REG_TEST_PATTERN_B	0x0606
#define IMX258_REG_TEST_PATTERN_GB	0x0608
#define IMX258_TEST_PATTERN_COLOUR_MIN	0
#define IMX258_TEST_PATTERN_COLOUR_MAX	0x0fff
#define IMX258_TEST_PATTERN_COLOUR_STEP	1
#define IMX258_TEST_PATTERN_R_DEFAULT	IMX258_TEST_PATTERN_COLOUR_MAX
#define IMX258_TEST_PATTERN_GR_DEFAULT	0
#define IMX258_TEST_PATTERN_B_DEFAULT	0
#define IMX258_TEST_PATTERN_GB_DEFAULT	0

/* Orientation copied from original driver */
#define REG_MIRROR_FLIP_CONTROL         0x0101
#define REG_CONFIG_MIRROR_FLIP          0x03
#define REG_CONFIG_FLIP_TEST_PATTERN    0x02

/* Embedded metadata stream structure */
#define IMX258_EMBEDDED_LINE_WIDTH 16384
#define IMX258_NUM_EMBEDDED_LINES 1

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

/* IMX258 native and active pixel array size. */
#define IMX258_NATIVE_WIDTH		4208U
#define IMX258_NATIVE_HEIGHT		3120U
#define IMX258_PIXEL_ARRAY_LEFT		0U
#define IMX258_PIXEL_ARRAY_TOP		0U
#define IMX258_PIXEL_ARRAY_WIDTH	4208U
#define IMX258_PIXEL_ARRAY_HEIGHT	3120U

struct imx258_reg {
	u16 address;
	u8 val;
};

struct imx258_reg_list {
	unsigned int num_of_regs;
	const struct imx258_reg *regs;
};

/* Mode : resolution and related config&values */
struct imx258_mode {
	/* Frame width */
	unsigned int width;

	/* Frame height */
	unsigned int height;

	/* H-timing in pixels */
	unsigned int line_length_pix;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Highest possible framerate. */
	struct v4l2_fract timeperframe_min;

	/* Default framerate. */
	struct v4l2_fract timeperframe_default;

	/* Default register values */
	struct imx258_reg_list reg_list;
};

static const struct imx258_reg mode_common_regs[] = {
// Common setting
//      External Clock Setting
//      Address         value
{        0x0136,          0x18},
{        0x0137,          0x00},


//      Global Setting
//      Address         value
{        0x3051,          0x00},
{        0x6B11,          0xCF},
{        0x7FF0,          0x08},
{        0x7FF1,          0x0F},
{        0x7FF2,          0x08},
{        0x7FF3,          0x1B},
{        0x7FF4,          0x23},
{        0x7FF5,          0x60},
{        0x7FF6,          0x00},
{        0x7FF7,          0x01},
{        0x7FF8,          0x00},
{        0x7FF9,          0x78},
{        0x7FFA,          0x01},
{        0x7FFB,          0x00},
{        0x7FFC,          0x00},
{        0x7FFD,          0x00},
{        0x7FFE,          0x00},
{        0x7FFF,          0x03},
{        0x7F76,          0x03},
{        0x7F77,          0xFE},
{        0x7FA8,          0x03},
{        0x7FA9,          0xFE},
{        0x7B24,          0x81},
{        0x7B25,          0x01},
{        0x6564,          0x07},
{        0x6B0D,          0x41},
{        0x653D,          0x04},
{        0x6B05,          0x8C},
{        0x6B06,          0xF9},
{        0x6B08,          0x65},
{        0x6B09,          0xFC},
{        0x6B0A,          0xCF},
{        0x6B0B,          0xD2},
{        0x6700,          0x0E},
{        0x6707,          0x0E},
{        0x5F04,          0x00},
{        0x5F05,          0xED},
// End of Common setting
};

/* 12 mpix 15fps */
static const struct imx258_reg mode_4208x3120_regs[] = {
//      Output format Settin
//      Address         value
{        0x0112,          0x0A},
{        0x0113,          0x0A},
{        0x0114,          0x01},

//      Clock Setting
//      Address         value
{        0x0301,          0x05 },   // IVTPXCK_DIV
{        0x0303,          0x04 },   // IVTSYCK_DIV
{        0x0305,          0x04 },
{        0x0306,          0x00 },   // PLL_IVT_MPY
{        0x0307,          216 },    // # 152 for 912MHz PLL_IVT_MPY 216 for 1,296MHz
{        0x0309,          0x0A },
{        0x030B,          0x01 },
{        0x030D,          0x02 },
{        0x030E,          0x00 },  // # PLL_IOP_MPY
{        0x030F,          216 },    // PLL_IOP_MPY
{        0x0310,          0x00 },

{        0x0820,          0x0A },
{        0x0821,          0x20 },
{        0x0822,          0x00 },
{        0x0823,          0x00 },

//      Clock Adjustment Setting
//      Address         value
{        0x4648,          0x7F },
//      0x7420          0x01 // removed
//      0x7421          0x00 // removed
//      0x7422          0x03 // removed
//      0x7423          0x00 // removed
{        0x9104,          0x04 }, // modified to 0x00

//      Line Length Setting
//      Address         value
{        0x0342,          0x14 },    // 5352
{        0x0343,          0xE8 },    // 5352

//      Frame Length Setting
//      Address         value
//{        0x0340,          0x0C },    // 3224
//{        0x0341,          0x98 },    // 3224

//      ROI Setting
//      Address         valu
{        0x0344,          0x00 },
{        0x0345,          0x00 },
{        0x0346,          0x00 },
{        0x0347,          0x00 },
{        0x0348,          0x10 },    // 4207
{        0x0349,          0x6F },    // 4207
{        0x034A,          0x0C },    // 3119
{        0x034B,          0x2F },    // 3119

//      Analog Image Size Setting
//      Address         value
{        0x0381,          0x01 },
{        0x0383,          0x01 },
{        0x0385,          0x01 },
{        0x0387,          0x01 },
{        0x0900,          0x00 },
{        0x0901,          0x11 },

//      Digital Image Size Setting
//      Address         value
{        0x0401,          0x00 },
{        0x0404,          0x00 },
{        0x0405,          0x10 },
{        0x0408,          0x00 },
{        0x0409,          0x00 },
{        0x040A,          0x00 },
{        0x040B,          0x00 },
{        0x040C,          0x10 },
{        0x040D,          0x70 },
{        0x040E,          0x0C },
{        0x040F,          0x30 },
{        0x3038,          0x00 },
{        0x303A,          0x00 },
{        0x303B,          0x10 },
{        0x300D,          0x00 },

//      Output Size Setting
//      Address         value
{        0x034C,          0x10 },
{        0x034D,          0x70 },
{        0x034E,          0x0C },
{        0x034F,          0x30 },

//      Integration Time Setting
//      Address         value
// {        0x0202,          0x0C },
// {        0x0203,          0x8E },

//      Gain Setting
//      Address         value
//{        0x0204,          0x00 },
//{        0x0205,          0x00 },
{        0x020E,          0x01 },
{        0x020F,          0x00 },
{        0x0210,          0x01 },
{        0x0211,          0x00 },
{        0x0212,          0x01 },
{        0x0213,          0x00 },
{        0x0214,          0x01 },
{        0x0215,          0x00 },

//      Added Setting(AF)
//      Address         value
{        0x7BCD,          0x00 },

//      Added Setting(IQ)
//      Address         value
{        0x94DC,          0x20 },
{        0x94DD,          0x20 },
{        0x94DE,          0x20 },
{        0x95DC,          0x20 },
{        0x95DD,          0x20 },
{        0x95DE,          0x20 },
{        0x7FB0,          0x00 },
{        0x9010,          0x3E },
{        0x9419,          0x50 },
{        0x941B,          0x50 },
{        0x9519,          0x50 },
{        0x951B,          0x50 },

//      Added Setting(mode)
//      Address         value
{        0x3030,          0x01 },
{        0x3032,          0x01 },
{        0x0220,          0x00 },
};

/* 2x2 binned. 30fps */
static const struct imx258_reg mode_2048x1560_regs[] = {
//      Output format Setting
//      Address         value
{        0x0112,          0x0A },
{        0x0113,          0x0A },
{        0x0114,          0x01 },

//      Clock Setting
//      Address         value
{        0x0301,          0x05 },
{        0x0303,          0x02 },
{        0x0305,          0x04 },
{        0x0306,          0x00 },
{        0x0307,          0xD8 },
{        0x0309,          0x0A },
{        0x030B,          0x01 },
{        0x030D,          0x02 },
{        0x030E,          0x00 },
{        0x030F,          0xD8 },
{        0x0310,          0x00 },
{        0x0820,          0x0A },
{        0x0821,          0x20 },
{        0x0822,          0x00 },
{        0x0823,          0x00 },


//      Clock Adjustment Setting
//      Address         value
{        0x4648,          0x7F },
//      0x7420          0x01 // removed
//      0x7421          0x00 // removed
//      0x7422          0x03 // removed
//      0x7423          0x00 // removed
{        0x9104,          0x00 }, //


//      Line Length Setting
//      Address         value
{        0x0342,          0x14 },
{        0x0343,          0xE8 },

//      Frame Length Setting
//      Address         value
//{        0x0340,          0x0C },
//{        0x0341,          0x98 },


//      ROI Setting
//      Address         value
{        0x0344,          0x00 },
{        0x0345,          0x00 },
{        0x0346,          0x00 },
{        0x0347,          0x00 },
{        0x0348,          0x10 },
{        0x0349,          0x6F },
{        0x034A,          0x0C },
{        0x034B,          0x2F },


//      Analog Image Size Setting
//      Address         value
{        0x0381,          0x01 },
{        0x0383,          0x01 },
{        0x0385,          0x01 },
{        0x0387,          0x01 },
{        0x0900,          0x01 },
{        0x0901,          0x12 },


//      Digital Image Size Setting
//      Address         value
{        0x0401,          0x01 },
{        0x0404,          0x00 },
{        0x0405,          0x20 },
{        0x0408,          0x00 },
{        0x0409,          0x02 },
{        0x040A,          0x00 },
{        0x040B,          0x00 },
{        0x040C,          0x10 },
{        0x040D,          0x68 },
{        0x040E,          0x06 },
{        0x040F,          0x18 },
{        0x3038,          0x00 },
{        0x303A,          0x00 },
{        0x303B,          0x10 },
{        0x300D,          0x00 },

//      Output Size Setting
//      Address         value
{        0x034C,          0x08 },
{        0x034D,          0x34 },
{        0x034E,          0x06 },
{        0x034F,          0x18 },

//      Integration Time Setting
//      Address         value
// {        0x0202,          0x0C },
// {        0x0203,          0x8E },

//      Gain Setting
//      Address         value
//{        0x0204,          0x00 },
//{        0x0205,          0x00 },
{        0x020E,          0x01 },
{        0x020F,          0x00 },
{        0x0210,          0x01 },
{        0x0211,          0x00 },
{        0x0212,          0x01 },
{        0x0213,          0x00 },
{        0x0214,          0x01 },
{        0x0215,          0x00 },

//      Added Setting(AF)
//      Address         valu
{        0x7BCD,          0x01 },

//      Added Setting(IQ)
//      Address         value
{        0x94DC,          0x20 },
{        0x94DD,          0x20 },
{        0x94DE,          0x20 },
{        0x95DC,          0x20 },
{        0x95DD,          0x20 },
{        0x95DE,          0x20 },
{        0x7FB0,          0x00 },
{        0x9010,          0x3E },
{        0x9419,          0x50 },
{        0x941B,          0x50 },
{        0x9519,          0x50 },
{        0x951B,          0x50 },

//      Added Setting(mode)
//      Address         value
{        0x3030,          0x00 },
{        0x3032,          0x00 },
{        0x0220,          0x00 },
};

/* 1080p cropped mode */
static const struct imx258_reg mode_1920x1080_regs[] = {
//      Output format Setting
//      Address         value
{        0x0112,          0x0A },
{        0x0113,          0x0A },
{        0x0114,          0x01 },
 
//      Clock Setting
//      Address         value
{        0x0301,          0x05 },
{        0x0303,          0x02 },
{        0x0305,          0x04 },
{        0x0306,          0x00 },
{        0x0307,          0xD8 },
{        0x0309,          0x0A },
{        0x030B,          0x01 },
{        0x030D,          0x02 },
{        0x030E,          0x00 },
{        0x030F,          0xD8 },
{        0x0310,          0x00 },
{        0x0820,          0x0A },
{        0x0821,          0x20 },
{        0x0822,          0x00 },
{        0x0823,          0x00 },


//      Clock Adjustment Setting
//      Address         value
{        0x4648,          0x7F },
//      0x7420          0x01 // removed
//      0x7421          0x00 // removed
//      0x7422          0x03 // removed
//      0x7423          0x00 // removed
{        0x9104,          0x00 }, //


//      Line Length Setting
//      Address         value
{        0x0342,          0x14 },
{        0x0343,          0xE8 },


//      Frame Length Setting
//      Address         value
//{        0x0340,          0x0C },
//{        0x0341,          0x98 },

//      ROI Setting
//      Address         value
{        0x0344,          0x00 },
{        0x0345,          0x00 },
{        0x0346,          0x00 },
{        0x0347,          0x00 },
{        0x0348,          0x10 },
{        0x0349,          0x6F },
{        0x034A,          0x0C },
{        0x034B,          0x2F },


//      Analog Image Size Setting
//      Address         value
{        0x0381,          0x01 },
{        0x0383,          0x01 },
{        0x0385,          0x01 },
{        0x0387,          0x01 },
{        0x0900,          0x01 },
{        0x0901,          0x12 },
 

//      Digital Image Size Setting
//      Address         value
{        0x0401,          0x01 },
{        0x0404,          0x00 },
{        0x0405,          0x20 },
{        0x0408,          0x00 },
{        0x0409,          92 },
{        0x040A,          0x00 },
{        0x040B,          240 },
{        0x040C,          0x0F }, // width 3,840
{        0x040D,          0x00 },
{        0x040E,          0x04 }, // height 1,080
{        0x040F,          0x38 },
{        0x3038,          0x00 },
{        0x303A,          0x00 },
{        0x303B,          0x10 },
{        0x300D,          0x00 },

//      Output Size Setting
//      Address         value
{        0x034C,          0x07 }, // x out size
{        0x034D,          0x80 }, // x out size
{        0x034E,          0x04 }, // y out size
{        0x034F,          0x38 }, // y out size

//      Integration Time Setting
//      Address         value
// {        0x0202,          0x0C },
// {        0x0203,          0x8E },

//      Gain Setting
//      Address         value
//{        0x0204,          0x03 },
//{        0x0205,          0x00 },
{        0x020E,          0x01 },
{        0x020F,          0xf0 },
{        0x0210,          0x01 },
{        0x0211,          0xf0 },
{        0x0212,          0x01 },
{        0x0213,          0xf0 },
{        0x0214,          0x01 },
{        0x0215,          0xf0 },

//      Added Setting(AF)
//      Address         valu
{        0x7BCD,          0x01 },

//      Added Setting(IQ)
//      Address         value
{        0x94DC,          0x20 },
{        0x94DD,          0x20 },
{        0x94DE,          0x20 },
{        0x95DC,          0x20 },
{        0x95DD,          0x20 },
{        0x95DE,          0x20 },
{        0x7FB0,          0x00 },
{        0x9010,          0x3E },
{        0x9419,          0x50 },
{        0x941B,          0x50 },
{        0x9519,          0x50 },
{        0x951B,          0x50 },

//      Added Setting(mode)
//      Address         value
{        0x3030,          0x00 },
{        0x3032,          0x00 },
{        0x0220,          0x00 },

};

/* Mode configs */
static const struct imx258_mode supported_modes_10bit[] = {
	{
		/* 12MPix 15fps mode */
		.width = 4208,
		.height = 3120,
		.line_length_pix = 5352,
		.crop = {
			.left = IMX258_PIXEL_ARRAY_LEFT,
			.top = IMX258_PIXEL_ARRAY_TOP,
			.width = 4096,
			.height = 3120,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 1000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_4208x3120_regs),
			.regs = mode_4208x3120_regs,
		},
	},
	{
		/* 2x2 binned 30fps mode */
		.width = 2048,
		.height = 1560,
		.line_length_pix = 5352,
		.crop = {
			.left = IMX258_PIXEL_ARRAY_LEFT,
			.top = IMX258_PIXEL_ARRAY_TOP,
			.width = 2048,
			.height = 1560,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 4000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_2048x1560_regs),
			.regs = mode_2048x1560_regs,
		},
	},
	{
		/* 1080p 30fps cropped mode */
		.width = 1920,
		.height = 1080,
		.line_length_pix = 5352,
		.crop = {
			.left = IMX258_PIXEL_ARRAY_LEFT,
			.top = IMX258_PIXEL_ARRAY_TOP + 440,
			.width = 1920,
			.height = 1080,
		},
		.timeperframe_min = {
			.numerator = 100,
			.denominator = 4000
		},
		.timeperframe_default = {
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1920x1080_regs),
			.regs = mode_1920x1080_regs,
		},
	}
};


/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 12-bit modes. */
/*
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,
*/
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
};

static const char * const imx258_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Solid Color",
	"Grey Color Bars",
	"PN9"
};

static const int imx258_test_pattern_val[] = {
	IMX258_TEST_PATTERN_DISABLE,
	IMX258_TEST_PATTERN_COLOR_BARS,
	IMX258_TEST_PATTERN_SOLID_COLOR,
	IMX258_TEST_PATTERN_GREY_COLOR,
	IMX258_TEST_PATTERN_PN9,
};

/* regulator supplies */
static const char * const imx258_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.05V) supply */
	"VDDL",  /* IF (1.8V) supply */
};

#define IMX258_NUM_SUPPLIES ARRAY_SIZE(imx258_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX258_XCLR_MIN_DELAY_US	8000
#define IMX258_XCLR_DELAY_RANGE_US	1000

struct imx258 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	unsigned int fmt_code;

	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[IMX258_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;

	/* Current mode */
	const struct imx258_mode *mode;

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* Rewrite common registers on stream on? */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK */
	unsigned int long_exp_shift;
};

static inline struct imx258 *to_imx258(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx258, sd);
}

static inline void get_mode_table(unsigned int code,
				  const struct imx258_mode **mode_list,
				  unsigned int *num_modes)
{
	switch (code) {
	/* 12-bit */
/*
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;
		*num_modes = ARRAY_SIZE(supported_modes_12bit);
		break;
*/
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = supported_modes_10bit;
		*num_modes = ARRAY_SIZE(supported_modes_10bit);
		break;
	default:
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
static int imx258_read_reg(struct imx258 *imx258, u16 reg, u32 len, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };
	u8 data_buf[4] = { 0, };
	int ret;

	if (len > 4)
		return -EINVAL;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

/* Write registers up to 2 at a time */
static int imx258_write_reg(struct imx258 *imx258, u16 reg, u32 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Write a list of registers */
static int imx258_write_regs(struct imx258 *imx258,
			     const struct imx258_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = imx258_write_reg(imx258, regs[i].address, 1, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		}
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 imx258_get_format_code(struct imx258 *imx258, u32 code)
{
	unsigned int i;

	lockdep_assert_held(&imx258->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
		i = 0;

	i = (i & ~3) | (imx258->vflip->val ? 2 : 0) |
	    (imx258->hflip->val ? 1 : 0);

	return codes[i];
}

static void imx258_set_default_format(struct imx258 *imx258)
{
	/* Set default mode to max resolution */
	/* Only support 10bit mode */
	// imx258->mode = &supported_modes_12bit[0];
	//imx258->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;
	imx258->mode = &supported_modes_10bit[0];
	imx258->fmt_code = MEDIA_BUS_FMT_SRGGB10_1X10;
}

static int imx258_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx258 *imx258 = to_imx258(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->pad, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->pad, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&imx258->mutex);

	/* Initialize try_fmt for the image pad */
	//try_fmt_img->width = supported_modes_12bit[0].width;
	//try_fmt_img->height = supported_modes_12bit[0].height;
	//try_fmt_img->code = imx258_get_format_code(imx258,
	try_fmt_img->width = supported_modes_10bit[0].width;
	try_fmt_img->height = supported_modes_10bit[0].height;
	try_fmt_img->code = imx258_get_format_code(imx258,
						   MEDIA_BUS_FMT_SRGGB10_1X10);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = IMX258_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = IMX258_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->pad, IMAGE_PAD);
	try_crop->left = IMX258_PIXEL_ARRAY_LEFT;
	try_crop->top = IMX258_PIXEL_ARRAY_TOP;
	try_crop->width = IMX258_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX258_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx258->mutex);

	return 0;
}

static void imx258_adjust_exposure_range(struct imx258 *imx258)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = imx258->mode->height + imx258->vblank->val -
		       (IMX258_EXPOSURE_OFFSET << imx258->long_exp_shift);
	exposure_def = min(exposure_max, imx258->exposure->val);
	__v4l2_ctrl_modify_range(imx258->exposure, imx258->exposure->minimum,
				 exposure_max, imx258->exposure->step,
				 exposure_def);
}

static int imx258_set_frame_length(struct imx258 *imx258, unsigned int val)
{
	int ret = 0;

	imx258->long_exp_shift = 0;

	while (val > IMX258_FRAME_LENGTH_MAX) {
		imx258->long_exp_shift++;
		val >>= 1;
	}

	ret = imx258_write_reg(imx258, IMX258_REG_FRAME_LENGTH,
			       IMX258_REG_VALUE_16BIT, val);
	if (ret)
		return ret;
	return imx258_write_reg(imx258, IMX258_LONG_EXP_SHIFT_REG,
				IMX258_REG_VALUE_08BIT, imx258->long_exp_shift);
}

static int imx258_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx258 *imx258 =
		container_of(ctrl->handler, struct imx258, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	int ret = 0;

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		imx258_adjust_exposure_range(imx258);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx258_write_reg(imx258, IMX258_REG_ANALOG_GAIN,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx258_write_reg(imx258, IMX258_REG_EXPOSURE,
				       IMX258_REG_VALUE_16BIT, ctrl->val >>
							imx258->long_exp_shift);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx258_write_reg(imx258, IMX258_REG_GR_DIGITAL_GAIN,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		ret = imx258_write_reg(imx258, IMX258_REG_R_DIGITAL_GAIN,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		ret = imx258_write_reg(imx258, IMX258_REG_B_DIGITAL_GAIN,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		ret = imx258_write_reg(imx258, IMX258_REG_GB_DIGITAL_GAIN,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx258_write_reg(imx258, IMX258_REG_TEST_PATTERN,
				       IMX258_REG_VALUE_16BIT,
				       imx258_test_pattern_val[ctrl->val]);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx258_write_reg(imx258, IMX258_REG_TEST_PATTERN_R,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx258_write_reg(imx258, IMX258_REG_TEST_PATTERN_GR,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx258_write_reg(imx258, IMX258_REG_TEST_PATTERN_B,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx258_write_reg(imx258, IMX258_REG_TEST_PATTERN_GB,
				       IMX258_REG_VALUE_16BIT, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx258_write_reg(imx258, IMX258_REG_ORIENTATION, 1,
				       imx258->hflip->val |
				       imx258->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		ret = imx258_set_frame_length(imx258,
					      imx258->mode->height + ctrl->val);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx258_ctrl_ops = {
	.s_ctrl = imx258_set_ctrl,
};

static int imx258_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx258 *imx258 = to_imx258(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;

		code->code = imx258_get_format_code(imx258,
						    codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int imx258_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx258 *imx258 = to_imx258(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		const struct imx258_mode *mode_list;
		unsigned int num_modes;

		get_mode_table(fse->code, &mode_list, &num_modes);

		if (fse->index >= num_modes)
			return -EINVAL;

		if (fse->code != imx258_get_format_code(imx258, fse->code))
			return -EINVAL;

		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = IMX258_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX258_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void imx258_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void imx258_update_image_pad_format(struct imx258 *imx258,
					   const struct imx258_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	imx258_reset_colorspace(&fmt->format);
}

static void imx258_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = IMX258_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX258_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int imx258_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct imx258 *imx258 = to_imx258(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx258->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx258->sd, cfg, fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx258_get_format_code(imx258, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			imx258_update_image_pad_format(imx258, imx258->mode,
						       fmt);
			fmt->format.code =
			       imx258_get_format_code(imx258, imx258->fmt_code);
		} else {
			imx258_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx258->mutex);
	return 0;
}

static
unsigned int imx258_get_frame_length(const struct imx258_mode *mode,
				     const struct v4l2_fract *timeperframe)
{
	u64 frame_length;

	frame_length = (u64)timeperframe->numerator * IMX258_PIXEL_RATE;
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	if (WARN_ON(frame_length > IMX258_FRAME_LENGTH_MAX))
		frame_length = IMX258_FRAME_LENGTH_MAX;

	return max_t(unsigned int, frame_length, mode->height);
}

static void imx258_set_framing_limits(struct imx258 *imx258)
{
	unsigned int frm_length_min, frm_length_default, hblank;
	const struct imx258_mode *mode = imx258->mode;

	frm_length_min = imx258_get_frame_length(mode, &mode->timeperframe_min);
	frm_length_default =
		     imx258_get_frame_length(mode, &mode->timeperframe_default);

	/* Default to no long exposure multiplier. */
	imx258->long_exp_shift = 0;

	/* Update limits and set FPS to default */
/* ************************************************************************
	__v4l2_ctrl_modify_range(imx258->vblank, frm_length_min - mode->height,
				 ((1 << IMX258_LONG_EXP_SHIFT_MAX) *
					IMX258_FRAME_LENGTH_MAX) - mode->height,
				 1, frm_length_default - mode->height);
************************************************************************* */

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx258->vblank, frm_length_default - mode->height);

	/*
	 * Currently PPL is fixed to the mode specified value, so hblank
	 * depends on mode->width only, and is not changeable in any
	 * way other than changing the mode.
	 */
	hblank = mode->line_length_pix - mode->width;
	__v4l2_ctrl_modify_range(imx258->hblank, hblank, hblank, 1, hblank);
}

static int imx258_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx258_mode *mode;
	struct imx258 *imx258 = to_imx258(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&imx258->mutex);

	if (fmt->pad == IMAGE_PAD) {
		const struct imx258_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		fmt->format.code = imx258_get_format_code(imx258,
							  fmt->format.code);

		get_mode_table(fmt->format.code, &mode_list, &num_modes);

		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		imx258_update_image_pad_format(imx258, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, cfg,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			imx258->mode = mode;
			imx258->fmt_code = fmt->format.code;
			imx258_set_framing_limits(imx258);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, cfg,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			imx258_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&imx258->mutex);

	return 0;
}

static const struct v4l2_rect *
__imx258_get_pad_crop(struct imx258 *imx258, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx258->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx258->mode->crop;
	}

	return NULL;
}

static int imx258_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct imx258 *imx258 = to_imx258(sd);

		mutex_lock(&imx258->mutex);
		sel->r = *__imx258_get_pad_crop(imx258, cfg, sel->pad,
						sel->which);
		mutex_unlock(&imx258->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX258_NATIVE_WIDTH;
		sel->r.height = IMX258_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX258_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX258_PIXEL_ARRAY_TOP;
		sel->r.width = IMX258_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX258_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

/* Start streaming */
static int imx258_start_streaming(struct imx258 *imx258)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	const struct imx258_reg_list *reg_list;
	int ret;

	if (!imx258->common_regs_written) {
		ret = imx258_write_regs(imx258, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		imx258->common_regs_written = true;
	}

	/* Apply default values of current mode */
	reg_list = &imx258->mode->reg_list;
	ret = imx258_write_regs(imx258, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(imx258->sd.ctrl_handler);
	if (ret)
		return ret;

	/* set stream on register */
	return imx258_write_reg(imx258, IMX258_REG_MODE_SELECT,
				IMX258_REG_VALUE_08BIT, IMX258_MODE_STREAMING);
}

/* Stop streaming */
static void imx258_stop_streaming(struct imx258 *imx258)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	int ret;

	/* set stream off register */
	ret = imx258_write_reg(imx258, IMX258_REG_MODE_SELECT,
			       IMX258_REG_VALUE_08BIT, IMX258_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);
}

static int imx258_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx258 *imx258 = to_imx258(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&imx258->mutex);
	if (imx258->streaming == enable) {
		mutex_unlock(&imx258->mutex);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = imx258_start_streaming(imx258);
		if (ret)
			goto err_rpm_put;
	} else {
		imx258_stop_streaming(imx258);
		pm_runtime_put(&client->dev);
	}

	imx258->streaming = enable;

	/* vflip and hflip cannot change during streaming */
	__v4l2_ctrl_grab(imx258->vflip, enable);
	__v4l2_ctrl_grab(imx258->hflip, enable);

	mutex_unlock(&imx258->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&imx258->mutex);

	return ret;
}

/* Power/clock management functions */
static int imx258_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx258 *imx258 = to_imx258(sd);
	int ret;

	ret = regulator_bulk_enable(IMX258_NUM_SUPPLIES,
				    imx258->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(imx258->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(imx258->reset_gpio, 1);
	usleep_range(IMX258_XCLR_MIN_DELAY_US,
		     IMX258_XCLR_MIN_DELAY_US + IMX258_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(IMX258_NUM_SUPPLIES, imx258->supplies);
	return ret;
}

static int imx258_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx258 *imx258 = to_imx258(sd);

	gpiod_set_value_cansleep(imx258->reset_gpio, 0);
	regulator_bulk_disable(IMX258_NUM_SUPPLIES, imx258->supplies);
	clk_disable_unprepare(imx258->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	imx258->common_regs_written = false;

	return 0;
}

static int __maybe_unused imx258_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx258 *imx258 = to_imx258(sd);

	if (imx258->streaming)
		imx258_stop_streaming(imx258);

	return 0;
}

static int __maybe_unused imx258_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx258 *imx258 = to_imx258(sd);
	int ret;

	if (imx258->streaming) {
		ret = imx258_start_streaming(imx258);
		if (ret)
			goto error;
	}

	return 0;

error:
	imx258_stop_streaming(imx258);
	imx258->streaming = 0;
	return ret;
}

static int imx258_get_regulators(struct imx258 *imx258)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	unsigned int i;

	for (i = 0; i < IMX258_NUM_SUPPLIES; i++)
		imx258->supplies[i].supply = imx258_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       IMX258_NUM_SUPPLIES,
				       imx258->supplies);
}

/* Verify chip ID */
static int imx258_identify_module(struct imx258 *imx258)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	int ret;
	u32 val;

	ret = imx258_read_reg(imx258, IMX258_REG_CHIP_ID,
			      IMX258_REG_VALUE_16BIT, &val);
	if (ret) {
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			IMX258_CHIP_ID, ret);
		return ret;
	}

	if (val != IMX258_CHIP_ID) {
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			IMX258_CHIP_ID, val);
		return -EIO;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops imx258_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops imx258_video_ops = {
	.s_stream = imx258_set_stream,
};

static const struct v4l2_subdev_pad_ops imx258_pad_ops = {
	.enum_mbus_code = imx258_enum_mbus_code,
	.get_fmt = imx258_get_pad_format,
	.set_fmt = imx258_set_pad_format,
	.get_selection = imx258_get_selection,
	.enum_frame_size = imx258_enum_frame_size,
};

static const struct v4l2_subdev_ops imx258_subdev_ops = {
	.core = &imx258_core_ops,
	.video = &imx258_video_ops,
	.pad = &imx258_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx258_internal_ops = {
	.open = imx258_open,
};

/* Initialize control handlers */
static int imx258_init_controls(struct imx258 *imx258)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx258->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	ctrl_hdlr = &imx258->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&imx258->mutex);
	ctrl_hdlr->lock = &imx258->mutex;

	/* By default, PIXEL_RATE is read only */
	imx258->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       IMX258_PIXEL_RATE,
					       IMX258_PIXEL_RATE, 1,
					       IMX258_PIXEL_RATE);

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx258_set_framing_limits() call below.
	 */
	imx258->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	imx258->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	/* HBLANK is read-only for now, but does change with mode. */
	if (imx258->hblank)
		imx258->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx258->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX258_EXPOSURE_MIN,
					     IMX258_EXPOSURE_MAX,
					     IMX258_EXPOSURE_STEP,
					     IMX258_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX258_ANA_GAIN_MIN, IMX258_ANA_GAIN_MAX,
			  IMX258_ANA_GAIN_STEP, IMX258_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX258_DGTL_GAIN_MIN, IMX258_DGTL_GAIN_MAX,
			  IMX258_DGTL_GAIN_STEP, IMX258_DGTL_GAIN_DEFAULT);

	imx258->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx258->hflip)
		imx258->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx258->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx258->vflip)
		imx258->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx258_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx258_test_pattern_menu) - 1,
				     0, 0, imx258_test_pattern_menu);
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx258_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX258_TEST_PATTERN_COLOUR_MIN,
				  IMX258_TEST_PATTERN_COLOUR_MAX,
				  IMX258_TEST_PATTERN_COLOUR_STEP,
				  IMX258_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx258_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx258->sd.ctrl_handler = ctrl_hdlr;

	/* Setup exposure and frame/line length limits. */
	imx258_set_framing_limits(imx258);

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx258->mutex);

	return ret;
}

static void imx258_free_controls(struct imx258 *imx258)
{
	v4l2_ctrl_handler_free(imx258->sd.ctrl_handler);
	mutex_destroy(&imx258->mutex);
}

static int imx258_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != IMX258_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int imx258_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx258 *imx258;
	int ret;

	imx258 = devm_kzalloc(&client->dev, sizeof(*imx258), GFP_KERNEL);
	if (!imx258)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx258->sd, client, &imx258_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (imx258_check_hwcfg(dev))
		return -EINVAL;

	/* Get system clock (xclk) */
	imx258->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx258->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx258->xclk);
	}

	imx258->xclk_freq = clk_get_rate(imx258->xclk);
	if (imx258->xclk_freq != IMX258_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx258->xclk_freq);
		return -EINVAL;
	}

	ret = imx258_get_regulators(imx258);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	imx258->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx258_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = imx258_power_on(dev);
	if (ret)
		return ret;

	ret = imx258_identify_module(imx258);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx258_set_default_format(imx258);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	/* This needs the pm runtime to be registered. */
	ret = imx258_init_controls(imx258);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx258->sd.internal_ops = &imx258_internal_ops;
	imx258->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	imx258->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	imx258->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx258->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx258->sd.entity, NUM_PADS, imx258->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	ret = v4l2_async_register_subdev_sensor_common(&imx258->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx258->sd.entity);

error_handler_free:
	imx258_free_controls(imx258);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx258_power_off(&client->dev);

	return ret;
}

static int imx258_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx258 *imx258 = to_imx258(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx258_free_controls(imx258);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx258_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

static const struct of_device_id imx258_dt_ids[] = {
	{ .compatible = "sony,imx258" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx258_dt_ids);

static const struct dev_pm_ops imx258_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx258_suspend, imx258_resume)
	SET_RUNTIME_PM_OPS(imx258_power_off, imx258_power_on, NULL)
};

static struct i2c_driver imx258_i2c_driver = {
	.driver = {
		.name = "imx258",
		.of_match_table	= imx258_dt_ids,
		.pm = &imx258_pm_ops,
	},
	.probe_new = imx258_probe,
	.remove = imx258_remove,
};

module_i2c_driver(imx258_i2c_driver);

MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_DESCRIPTION("Sony IMX258 sensor driver");
MODULE_LICENSE("GPL v2");

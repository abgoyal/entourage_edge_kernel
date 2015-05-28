/*
 * wm8960.c  --  WM8960 ALSA SoC Audio driver
 *
 * Author: Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include "autopower.h"

#include "wm8960.h"

#define AUDIO_NAME "wm8960"
#define WM8960_VERSION "0.1"

/*
 * Debug
 */

#define WM8960_DEBUG 0

#ifdef WM8960_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

struct snd_soc_codec_device soc_codec_dev_wm8960;

/*
 * wm8960 register cache
 * We can't read the WM8960 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8960_reg[WM8960_CACHEREGNUM] = {
	0x0097, 0x0097, 0x0000, 0x0000,
	0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x007b, 0x0100, 0x0032,
	0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0100, 0x0100, 0x0050, 0x0050,
	0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000,
	0x0000, 0x0050, 0x0050, 0x0000,
	0x0002, 0x0037, 0x004d, 0x0080,
	0x0008, 0x0031, 0x0026, 0x00e9,
};

struct wm8960_priv {
	u16 reg_cache[WM8960_CACHEREGNUM];
	struct snd_soc_codec codec;
};

/*
 * read wm8960 register cache
 */
static inline unsigned int wm8960_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8960_RESET)
		return 0;
	if (reg >= WM8960_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8960 register cache
 */
static inline void wm8960_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8960_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8960 register space
 */
static int wm8960_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8960 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8960_write_reg_cache(codec, reg, value);
	if (codec->hw_write(codec->control_data, data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8960_reset(c)	wm8960_write(c, WM8960_RESET, 0)

/* enumerated controls */
static const char *wm8960_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static const char *wm8960_polarity[] = {"No Inversion", "Left Inverted",
	"Right Inverted", "Stereo Inversion"};
static const char *wm8960_3d_upper_cutoff[] = {"High", "Low"};
static const char *wm8960_3d_lower_cutoff[] = {"Low", "High"};
static const char *wm8960_alcfunc[] = {"Off", "Right", "Left", "Stereo"};
static const char *wm8960_alcmode[] = {"ALC", "Limiter"};

static const struct soc_enum wm8960_enum[] = {
	SOC_ENUM_SINGLE(WM8960_DACCTL1, 1, 4, wm8960_deemph),
	SOC_ENUM_SINGLE(WM8960_DACCTL1, 5, 4, wm8960_polarity),
	SOC_ENUM_SINGLE(WM8960_DACCTL2, 5, 4, wm8960_polarity),
	SOC_ENUM_SINGLE(WM8960_3D, 6, 2, wm8960_3d_upper_cutoff),
	SOC_ENUM_SINGLE(WM8960_3D, 5, 2, wm8960_3d_lower_cutoff),
	SOC_ENUM_SINGLE(WM8960_ALC1, 7, 4, wm8960_alcfunc),
	SOC_ENUM_SINGLE(WM8960_ALC3, 8, 2, wm8960_alcmode),
};

/* to complete */
static const struct snd_kcontrol_new wm8960_snd_controls[] = {
SOC_DOUBLE_R("Capture Volume", WM8960_LINVOL, WM8960_RINVOL, 0, 63, 0),
SOC_DOUBLE_R("Capture Volume Update Now", WM8960_LINVOL, WM8960_RINVOL, 8, 1, 0),
SOC_DOUBLE_R("Capture Volume ZC Enable", WM8960_LINVOL, WM8960_RINVOL, 6, 1, 0),
SOC_DOUBLE_R("Capture Disable", WM8960_LINVOL, WM8960_RINVOL,	7, 1, 0),

SOC_DOUBLE_R("Headphone Volume Update Now", WM8960_LOUT1, WM8960_ROUT1, 8, 1, 0),
SOC_DOUBLE_R("Headphone Volume Control", WM8960_LOUT1, WM8960_ROUT1, 0, 127, 0),
SOC_DOUBLE_R("Headphone Playback ZC Enable", WM8960_LOUT1, WM8960_ROUT1,	7, 1, 0),

SOC_SINGLE("Speaker Enable", WM8960_CLASSD1 , 6, 3, 0),
SOC_DOUBLE_R("Speaker Volume Control", WM8960_LOUT2, WM8960_ROUT2, 0, 127, 0),
SOC_DOUBLE_R("Speaker Volume Update Now", WM8960_LOUT2, WM8960_ROUT2, 8, 1, 0),
SOC_DOUBLE_R("Speaker ZC Enable", WM8960_LOUT2, WM8960_ROUT2,	7, 1, 0),

SOC_SINGLE("DAC Attenuate -6dB Enable", WM8960_DACCTL1, 7, 1, 0),
SOC_ENUM("DAC De-emphasis", wm8960_enum[0]),
SOC_ENUM("DAC Polarity Control", wm8960_enum[2]),
SOC_DOUBLE_R("DAC Digital Volume Control", WM8960_LDAC, WM8960_RDAC, 0, 127, 0),
SOC_SINGLE("DAC Soft Mute", WM8960_DACCTL1, 3, 1, 0),

SOC_SINGLE("ADC High Pass Filter Enable", WM8960_DACCTL1, 0, 1, 0),
SOC_ENUM("ADC Polarity Control", wm8960_enum[1]),

SOC_ENUM("3D Filter Upper Cut-Off", wm8960_enum[3]),
SOC_ENUM("3D Filter Lower Cut-Off", wm8960_enum[4]),
SOC_SINGLE("3D Volume", WM8960_3D, 1, 15, 0),
SOC_SINGLE("3D Enable", WM8960_3D, 0, 1, 0),

SOC_ENUM("ALC Function", wm8960_enum[5]),
SOC_SINGLE("ALC Max Gain", WM8960_ALC1, 4, 7, 0),
SOC_SINGLE("ALC Target", WM8960_ALC1, 0, 15, 1),
SOC_SINGLE("ALC Min Gain", WM8960_ALC2, 4, 7, 0),
SOC_SINGLE("ALC Hold Time", WM8960_ALC2, 0, 15, 0),
SOC_ENUM("ALC Mode", wm8960_enum[6]),
SOC_SINGLE("ALC Decay", WM8960_ALC3, 4, 15, 0),
SOC_SINGLE("ALC Attack", WM8960_ALC3, 0, 15, 0),

SOC_SINGLE("Noise Gate Threshold", WM8960_NOISEG, 3, 31, 0),
SOC_SINGLE("Noise Gate Switch", WM8960_NOISEG, 0, 1, 0),


/* SOC_DOUBLE_R("ADC PCM Capture Volume", WM8960_LINPATH, WM8960_RINPATH, 0, 127, 0), */
};

/* Left Output Mixer */
static const struct snd_kcontrol_new wm8960_loutput_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8960_LOUTMIX, 8, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wm8960_routput_mixer_controls[] = {
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8960_ROUTMIX, 8, 1, 0),
};

static const struct snd_soc_dapm_widget wm8960_dapm_widgets[] = {
SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
	&wm8960_loutput_mixer_controls[0],
	ARRAY_SIZE(wm8960_loutput_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
	&wm8960_loutput_mixer_controls[0],
	ARRAY_SIZE(wm8960_routput_mixer_controls)),
};

static int wm8960_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, wm8960_dapm_widgets,
				  ARRAY_SIZE(wm8960_dapm_widgets));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int wm8960_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	wm8960_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static int wm8960_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	u16 iface = wm8960_read_reg_cache(codec, WM8960_IFACE1) & 0xfff3;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	}

	/* set iface */
	wm8960_write(codec, WM8960_IFACE1, iface);
	return 0;
}

static int wm8960_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8960_read_reg_cache(codec, WM8960_DACCTL1) & 0xfff7;

	if (mute)
		wm8960_write(codec, WM8960_DACCTL1, mute_reg | 0x8);
	else
		wm8960_write(codec, WM8960_DACCTL1, mute_reg);
	return 0;
}

static int wm8960_dapm_event(struct snd_soc_codec *codec, enum snd_soc_bias_level level)
{
#if 0
	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* vref/mid, osc on, dac unmute */

		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* everything off except vref/vmid, */
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* everything off, dac mute, inactive */
		break;
	}
	SND_SOC_BIAS_ON,
	SND_SOC_BIAS_PREPARE,
	SND_SOC_BIAS_STANDBY,
	SND_SOC_BIAS_OFF,
#endif

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		wm8960_write(codec, WM8960_POWER1, 0xfffe);
		wm8960_write(codec, WM8960_POWER2, 0xffff);
		wm8960_write(codec, WM8960_POWER3, 0xffff);
		break;
	case SND_SOC_BIAS_OFF:
		wm8960_write(codec, WM8960_POWER1, 0x1);
		wm8960_write(codec, WM8960_POWER2, 0x0);
		wm8960_write(codec, WM8960_POWER3, 0x0);
		break;
	default:
		break;
	}
	codec->bias_level = level ;
	return 0;
}

/* PLL divisors */
struct _pll_div {
	u32 pre_div:1;
	u32 n:4;
	u32 k:24;
};

static struct _pll_div pll_div;

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static void pll_factors(unsigned int target, unsigned int source)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div.pre_div = 1;
		Ndiv = target / source;
	} else
		pll_div.pre_div = 0;

	if ((Ndiv < 6) || (Ndiv > 12))
		printk(KERN_WARNING
			"WM8960 N value outwith recommended range! N = %d\n", Ndiv);

	pll_div.n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div.k = K;
}

static int wm8960_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;
	int found = 0;
#if 0
	if (freq_in == 0 || freq_out == 0) {
		/* disable the pll */
		/* turn PLL power off */
	}
#endif

	pll_factors(freq_out * 8, freq_in);

	if (!found)
		return -EINVAL;

	reg = wm8960_read_reg_cache(codec, WM8960_PLLN) & 0x1e0;
	wm8960_write(codec, WM8960_PLLN, reg | (1<<5) | (pll_div.pre_div << 4)
		| pll_div.n);
	wm8960_write(codec, WM8960_PLLK1, pll_div.k >> 16);
	wm8960_write(codec, WM8960_PLLK2, (pll_div.k >> 8) & 0xff);
	wm8960_write(codec, WM8960_PLLK3, pll_div.k & 0xff);
	wm8960_write(codec, WM8960_CLOCK1, 4);

	return 0;
}

static int wm8960_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8960_SYSCLKSEL:
		reg = wm8960_read_reg_cache(codec, WM8960_CLOCK1) & 0x1fe;
		wm8960_write(codec, WM8960_CLOCK1, reg | div);
		break;
	case WM8960_SYSCLKDIV:
		reg = wm8960_read_reg_cache(codec, WM8960_CLOCK1) & 0x1f9;
		wm8960_write(codec, WM8960_CLOCK1, reg | div);
		break;
	case WM8960_DACDIV:
		reg = wm8960_read_reg_cache(codec, WM8960_CLOCK1) & 0x1c7;
		wm8960_write(codec, WM8960_CLOCK1, reg | div);
		break;
	case WM8960_OPCLKDIV:
		reg = wm8960_read_reg_cache(codec, WM8960_PLLN) & 0x03f;
		wm8960_write(codec, WM8960_PLLN, reg | div);
		break;
	case WM8960_DCLKDIV:
		reg = wm8960_read_reg_cache(codec, WM8960_CLOCK2) & 0x03f;
		wm8960_write(codec, WM8960_CLOCK2, reg | div);
		break;
	case WM8960_TOCLKSEL:
		reg = wm8960_read_reg_cache(codec, WM8960_ADDCTL1) & 0x1fd;
		wm8960_write(codec, WM8960_ADDCTL1, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define WM8960_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define WM8960_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops wm8960_dai_ops = {
	.hw_params = wm8960_hw_params,
	.digital_mute = wm8960_mute,
	.set_fmt = wm8960_set_dai_fmt,
	.set_clkdiv = wm8960_set_dai_clkdiv,
	.set_pll = wm8960_set_dai_pll,
};

struct snd_soc_dai wm8960_dai = {
	.name = "WM8960",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = WM8960_RATES,
		.formats = WM8960_FORMATS,},
	.ops = &wm8960_dai_ops,
	.symmetric_rates = 1,
};
EXPORT_SYMBOL_GPL(wm8960_dai);

static int wm8960_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	wm8960_dapm_event(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static void restore_reg(struct snd_soc_codec *codec , unsigned int reg)
{
	u8 data[2];
	u16 *cache = codec->reg_cache;

	data[0] = (reg << 1) | ((cache[reg] >> 8) & 0x0001);
	data[1] = cache[reg] & 0x00ff ;
	codec->hw_write(codec->control_data, data, 2);

}
static int wm8960_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;
	wm8960_reset(codec);
	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8960_reg); i++) {
		restore_reg(codec , i);
	}

	/* just work around for wm8960 */
	restore_reg(codec , WM8960_LOUT1);
	restore_reg(codec , WM8960_ROUT1);
	restore_reg(codec , WM8960_LOUT2);
	restore_reg(codec , WM8960_ROUT2);

	wm8960_dapm_event(codec, SND_SOC_BIAS_STANDBY);
	wm8960_dapm_event(codec, codec->suspend_bias_level);
	return 0;
}

static struct platform_device *gpdev;

void wm8960_powerdown()
{
	pm_message_t pm_message;
	pm_message.event = 0;
	if (gpdev)
		wm8960_suspend(gpdev , pm_message);
}

void wm8960_powerup()
{
	if (gpdev)
		wm8960_resume(gpdev);
}

static struct snd_soc_codec *wm8960_codec;

static int wm8960_register(struct wm8960_priv *wm8960)
{
	struct snd_soc_codec *codec = &wm8960->codec;
	int reg, ret = 0;

	if (wm8960_codec) {
		dev_err(codec->dev, "Another WM8960 is registered\n");
		return -EINVAL;
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->private_data = wm8960;
	codec->name = "WM8960";
	codec->owner = THIS_MODULE;
	codec->read = wm8960_read_reg_cache;
	codec->write = wm8960_write;
	codec->set_bias_level = wm8960_dapm_event;
	codec->dai = &wm8960_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(wm8960_reg);
	codec->reg_cache = &wm8960->reg_cache;

	memcpy(codec->reg_cache, wm8960_reg, sizeof(wm8960_reg));

	if (codec->reg_cache == NULL)
		return -ENOMEM;
	wm8960_reset(codec);

	wm8960_dai.dev = codec->dev;
	/* power on device */
	wm8960_dapm_event(codec, SND_SOC_BIAS_STANDBY);

	wm8960_write(codec, WM8960_RESET, 0x0); /* R15 */

	wm8960_write(codec, WM8960_APOP1, 0xf5); /* R28 */
	msleep(400);
	wm8960_write(codec, WM8960_POWER2, 0x60); /* R26 */

	msleep(50);

	wm8960_write(codec, WM8960_APOP1, 0x94); /* R28 */
	wm8960_write(codec, WM8960_POWER1, 0x80); /* R25 */
	msleep(100);

	wm8960_write(codec, WM8960_POWER1, 0xfc); /* R25 */
	wm8960_write(codec, WM8960_POWER2, 0x1f8); /* 26 */
	wm8960_write(codec, WM8960_POWER3, 0x3c); /* R47 */

	wm8960_write(codec, WM8960_CLASSD1, 0xf7);
	wm8960_write(codec, WM8960_CLASSD3, 0x9b); /* R51	3.6db */

	wm8960_write(codec, WM8960_LOUTMIX, 0x100); /* R34	Left DAC->Left Output Mixer */
	wm8960_write(codec, WM8960_ROUTMIX, 0x100); /* R37	Right DAC->Right Output Mixer */

	wm8960_write(codec, WM8960_DACCTL1,  0x0) ; /* R5 */

	wm8960_write(codec, WM8960_LADC, 0x1c3);
	wm8960_write(codec, WM8960_RADC, 0x1c3);

	wm8960_write(codec, WM8960_LINPATH , 0x148);

	/* setting for HP autodetection */
	wm8960_write(codec, WM8960_ADDCTL2 , 0x40);
	wm8960_write(codec, WM8960_ADDCTL4 , 0x8);
	wm8960_write(codec, WM8960_ADDCTL1 , 0x3);

	/* setting for RINPUT1 */
	/* wm8960_write(codec , WM8960_RINVOL , 0x13f); */
	wm8960_write(codec, WM8960_RINPATH , 0x138);
	wm8960_write(codec, WM8960_RADC , 0x1c3);

	wm8960_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		return ret;
	}

	ret = snd_soc_register_dai(&wm8960_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(codec);
		return ret;
	}
	autopower_init();
	return 0;
}


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static int wm8960_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct wm8960_priv *wm8960;
	struct snd_soc_codec *codec;

	wm8960 = kzalloc(sizeof(struct wm8960_priv), GFP_KERNEL);
	if (wm8960 == NULL)
		return -ENOMEM;

	codec = &wm8960->codec;
	codec->hw_write = (hw_write_t)i2c_master_send;

	i2c_set_clientdata(i2c, wm8960);
	codec->control_data = i2c;

	codec->dev = &i2c->dev;

	return wm8960_register(wm8960);
}

static int wm8960_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	kfree(codec->reg_cache);
	return 0;
}

static struct i2c_device_id wm8960_idtable[] =
{
	{"WM8960", 0},
	{},
};

/* corgi i2c codec control layer */
static struct i2c_driver wm8960_i2c_driver = {
	.driver = {
		.name = "WM8960",
	},
       .id_table = wm8960_idtable,
	.probe			= &wm8960_i2c_probe,
	.remove			= &wm8960_i2c_remove,
};

#endif

static int wm8960_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	if (wm8960_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = wm8960_codec;
	codec = wm8960_codec;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	snd_soc_add_controls(codec, wm8960_snd_controls,
				 ARRAY_SIZE(wm8960_snd_controls));
	wm8960_add_widgets(codec);
	ret = snd_soc_init_card(socdev);
	if (ret < 0) {
		dev_err(codec->dev, "failed to register card: %d\n", ret);
		goto card_err;
	}
	gpdev = pdev;
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	return ret;
}


/* power down chip */
static int wm8960_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		wm8960_dapm_event(codec, SND_SOC_BIAS_OFF);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&wm8960_i2c_driver);
#endif
	kfree(codec);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_wm8960 = {
	.probe = 	wm8960_probe,
	.remove = 	wm8960_remove,
	.suspend = 	wm8960_suspend,
	.resume =	wm8960_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_wm8960);

static int __init wm8960_modinit(void)
{
	int ret;

	ret = i2c_add_driver(&wm8960_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register WM8960 I2C driver: %d\n",
		       ret);
	}

	return ret;
}
module_init(wm8960_modinit);

static void __exit wm8960_exit(void)
{
	i2c_del_driver(&wm8960_i2c_driver);
}
module_exit(wm8960_exit);

MODULE_DESCRIPTION("ASoC WM8960 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");

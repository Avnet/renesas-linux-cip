/*
 * Driver for generic lite codec that only has DAC/ADC
 * Copyright 2022 David Fu <david.fu@avnet.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static const struct snd_soc_dapm_widget  lite_codec_widgets[] = {
	SND_SOC_DAPM_INPUT("RX"),
	SND_SOC_DAPM_OUTPUT("TX"),
};

static const struct snd_soc_dapm_route  lite_codec_routes[] = {
	{ "Capture", NULL, "RX" },
	{ "TX", NULL, "Playback" },
};

static int lite_codec_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir){
    printk("%s: clk_id=%d, freq=%d.\n", __func__, clk_id, freq);
    return 0;
}

static int lite_codec_set_pll(struct snd_soc_dai *dai, int pll_id, int source, unsigned int freq_in, unsigned int freq_out){
    printk("%s: pll_id=%d, freq_in=%d, freq_out=%d.\n", __func__, pll_id, freq_in, freq_out);
    return 0;
}

static int lite_codec_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div){
    printk("%s: div_id=%d, div=%d.\n", __func__, div_id, div);
    return 0;
}

static int lite_codec_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai){
    printk("%s .\n", __func__);
    return 0;
}

static int lite_codec_hw_free(struct snd_pcm_substream *substream, struct snd_soc_dai *dai){
    printk("%s .\n", __func__);
    return 0;
}

static int lite_codec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt){
    printk("%s: fmt=%d.\n", __func__, fmt);
    return 0;
}


static const struct snd_soc_dai_ops lite_codec_dai_ops = {
    .set_sysclk = lite_codec_set_sysclk,
    .set_pll = lite_codec_set_pll,
    .set_clkdiv = lite_codec_set_clkdiv,
    .hw_params = lite_codec_hw_params,
    .hw_free = lite_codec_hw_free,
    .set_fmt = lite_codec_set_fmt,
};

static struct snd_soc_dai_driver lite_codec_dai[] = {
	{
		.name = "avt-lite-codec-pcm",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			 .stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &lite_codec_dai_ops,
	},

	{
		.name = "avt-lite-codec-pcm-wb",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			 .stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &lite_codec_dai_ops,
	},
};

static const struct snd_soc_component_driver soc_component_dev_lite_codec = {
	.dapm_widgets		=  lite_codec_widgets,
	.num_dapm_widgets	= ARRAY_SIZE( lite_codec_widgets),
	.dapm_routes		=  lite_codec_routes,
	.num_dapm_routes	= ARRAY_SIZE( lite_codec_routes),
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static int lite_codec_probe(struct platform_device *pdev)
{
	int ret;

	ret = devm_snd_soc_register_component(&pdev->dev,
				      &soc_component_dev_lite_codec,
				      lite_codec_dai, ARRAY_SIZE(lite_codec_dai));

	printk("###ret=%d %s-%d\n",ret, __func__,__LINE__);
	return ret;
}

static int lite_codec_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id lite_codec_driver_ids[] = {
	{
		.name		= "avnet-lite-codec",
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lite_codec_driver_ids);

#if defined(CONFIG_OF)
static const struct of_device_id lite_codec_codec_of_match[] = {
	{ .compatible = "avt,lite-codec-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, lite_codec_codec_of_match);
#endif

static struct platform_driver lite_codec_driver = {
	.driver = {
		.name = "lite-codec",
		.of_match_table = of_match_ptr(lite_codec_codec_of_match),
	},
	.probe =lite_codec_probe,
	.remove = lite_codec_remove,
	.id_table = lite_codec_driver_ids,
};

module_platform_driver(lite_codec_driver);

MODULE_AUTHOR("David Fu <david.fu@avnet.com>");
MODULE_DESCRIPTION("ASoC generic lite codec driver");
MODULE_LICENSE("GPL");

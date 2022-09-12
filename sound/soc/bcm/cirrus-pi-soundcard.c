/*
 * ASoC driver for the guitar system based on CS5343/CS5344 ADC
 * connected to a Raspberry Pi
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>

// starting with support for 48KHz
static const unsigned int cirrus_pi_soundcard_rates[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list cirrus_pi_soundcard_constraints = {
	.list = cirrus_pi_soundcard_rates,
	.count = ARRAY_SIZE(cirrus_pi_soundcard_rates),
};

static int snd_cirrus_pi_soundcard_startup(struct snd_pcm_substream
					 *substream)
{
	/* Setup constraints, because the sampling rate is fixed */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &cirrus_pi_soundcard_constraints);
	return 0;
}

static int snd_cirrus_pi_soundcard_init(struct snd_soc_pcm_runtime *rtd)
{
	return 0;
}

static int snd_cirrus_pi_soundcard_hw_params(struct snd_pcm_substream
					   *substream,
					   struct snd_pcm_hw_params
					   *params)
{
	return 0;
}

/* stream operations */
static struct snd_soc_ops snd_cirrus_pi_soundcard_ops = {
	.startup = snd_cirrus_pi_soundcard_startup,
	.hw_params = snd_cirrus_pi_soundcard_hw_params,
};

static struct snd_soc_dai_link snd_cirrus_pi_soundcard_dai[] = {
    {
        .name = "cirrus-pi-soundcard-cs5343",
        .stream_name = "cs5343 HiFi",
        .dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
                SND_SOC_DAIFMT_CBM_CFM,
        .ops = &snd_cirrus_pi_soundcard_ops,
        .init = snd_cirrus_pi_soundcard_init,
    },
    {
        .name = "cirrus-pi-soundcard-cs5344",
        .stream_name = "cs5344 HiFi",
        .dai_fmt = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF |
                SND_SOC_DAIFMT_CBM_CFM,
        .ops = &snd_cirrus_pi_soundcard_ops,
        .init = snd_cirrus_pi_soundcard_init,
    },
};

/* audio driver */
static struct snd_soc_card snd_cirrus_pi_soundcard = {
	.name = "snd-cirrus-pi-soundcard",
	.dai_link = snd_cirrus_pi_soundcard_dai,
	.num_links = ARRAY_SIZE(snd_cirrus_pi_soundcard_dai),
};

static int snd_cirrus_pi_soundcard_probe(struct platform_device *pdev)
{
	int ret = 0;

	snd_cirrus_pi_soundcard.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_cirrus_pi_soundcard);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
	}

	return ret;
}

static int snd_cirrus_pi_soundcard_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_cirrus_pi_soundcard);
}

static struct platform_driver snd_cirrus_pi_soundcard_driver = {
	.driver = {
		   .name = "snd-cirrus-pi-soundcard",
		   .owner = THIS_MODULE,
		   },
	.probe = snd_cirrus_pi_soundcard_probe,
	.remove = snd_cirrus_pi_soundcard_remove,
};

module_platform_driver(snd_cirrus_pi_soundcard_driver);

MODULE_AUTHOR("George Vigelette <gvigelet@duvitech.com>");
MODULE_DESCRIPTION
    ("ASoC Driver for a system with CS534x & Raspberry Pi");
MODULE_LICENSE("GPL");

/*
 * sound/soc/codecs/cs534x.c
 * simple, strap-pin configured 24bit 2ch ADC
 *
 */


#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>

static struct snd_soc_dai_driver cs534x_dai = {
	.name = "cs534x-hifi",
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 2,
		    .channels_max = 2,
		    /* 
		     * The sampling rate is defined at powerup by hardware
		     * basing on pull-up and pull-down resistors.
		     * Therefore the machine driver should constrain
		     * range of available rates to the one selected     
		     * in the hardware. 
		     */
		    .rates = SNDRV_PCM_RATE_32000 |
		             SNDRV_PCM_RATE_44100 |
		             SNDRV_PCM_RATE_48000 |
		             SNDRV_PCM_RATE_88200 |
                             SNDRV_PCM_RATE_96000,
		    .formats = SNDRV_PCM_FMTBIT_S32_LE,
		    },
};

static struct snd_soc_codec_driver soc_codec_dev_cs534x;

static int cs534x_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev,
				      &soc_codec_dev_cs534x, &cs534x_dai,
				      1);
}

static int cs534x_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver cs534x_codec_driver = {
	.driver = {
		   .name = "cs534x-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = cs534x_probe,
	.remove = cs534x_remove,
};

module_platform_driver(cs534x_codec_driver);

MODULE_DESCRIPTION("ASoC cs534x driver");
MODULE_AUTHOR("George Vigelette <gvigelet@duvitech.com>");
MODULE_LICENSE("GPL");

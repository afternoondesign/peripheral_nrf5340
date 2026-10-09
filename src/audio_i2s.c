/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "audio_i2s.h"

LOG_MODULE_REGISTER(audio_i2s, LOG_LEVEL_INF);

#define I2S_SAMPLE_RATE_HZ       48000U
#define I2S_FRAME_DURATION_US    10000U
#define I2S_SAMPLES_PER_FRAME    ((I2S_SAMPLE_RATE_HZ * I2S_FRAME_DURATION_US) / 1000000U)
#define I2S_CHANNEL_COUNT        2U
#define I2S_TX_BLOCK_SIZE        (I2S_SAMPLES_PER_FRAME * I2S_CHANNEL_COUNT * sizeof(int16_t))
#define I2S_TX_BLOCK_COUNT       4U
#define PCM_QUEUE_DEPTH          6U
#define I2S_SD_MODE_PIN          13U
#define I2S_WORKER_STACK_SIZE    2048U
#define I2S_WORKER_PRIORITY      5

struct audio_pcm_frame {
	bool stop;
	int16_t samples[I2S_SAMPLES_PER_FRAME];
};

K_MEM_SLAB_DEFINE(i2s_tx_slab, I2S_TX_BLOCK_SIZE, I2S_TX_BLOCK_COUNT, 4);
K_MSGQ_DEFINE(pcm_queue, sizeof(struct audio_pcm_frame), PCM_QUEUE_DEPTH, 4);
K_THREAD_STACK_DEFINE(i2s_worker_stack, I2S_WORKER_STACK_SIZE);

static struct k_thread i2s_worker_thread;
static const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
static const struct device *const gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static bool initialized;

static void i2s_worker(void *arg1, void *arg2, void *arg3)
{
	bool tx_started = false;
	uint8_t startup_blocks_queued = 0U;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct audio_pcm_frame frame;
		void *tx_block;
		int err;

		err = k_msgq_get(&pcm_queue, &frame, K_FOREVER);
		if (err != 0) {
			continue;
		}

		if (frame.stop) {
			if (tx_started) {
				err = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
				if (err != 0) {
					LOG_WRN("I2S drain failed (%d); dropping queued audio", err);
					(void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
				}
				tx_started = false;
			}
			startup_blocks_queued = 0U;
			continue;
		}

		err = k_mem_slab_alloc(&i2s_tx_slab, &tx_block, K_FOREVER);
		if (err != 0) {
			LOG_ERR("Could not allocate I2S TX block (%d)", err);
			continue;
		}

		int16_t *stereo = tx_block;
		for (size_t i = 0U; i < I2S_SAMPLES_PER_FRAME; i++) {
			/* The current single ASE is mono; duplicate it to both I2S slots. */
			stereo[(i * I2S_CHANNEL_COUNT)] = frame.samples[i];
			stereo[(i * I2S_CHANNEL_COUNT) + 1U] = frame.samples[i];
		}

		err = i2s_write(i2s_dev, tx_block, I2S_TX_BLOCK_SIZE);
		if (err != 0) {
			LOG_WRN("I2S write failed (%d); preparing stream for restart", err);
			k_mem_slab_free(&i2s_tx_slab, tx_block);
			if (i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE) != 0) {
				LOG_ERR("Could not prepare I2S TX after underrun");
			}
			tx_started = false;
			startup_blocks_queued = 0U;
			continue;
		}

		if (!tx_started) {
			startup_blocks_queued++;
			if (startup_blocks_queued < 2U) {
				continue;
			}

			err = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
			if (err != 0) {
				LOG_ERR("Could not start I2S TX (%d)", err);
				(void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
				startup_blocks_queued = 0U;
				continue;
			}
			tx_started = true;
		}
	}
}

int audio_i2s_init(void)
{
	struct i2s_config cfg = {0};
	int err;

	if (initialized) {
		return 0;
	}
	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device is not ready");
		return -ENODEV;
	}
	if (!device_is_ready(gpio0_dev)) {
		LOG_ERR("GPIO0 device is not ready");
		return -ENODEV;
	}

	/* MAX98357A SD_MODE must be high to enable the amplifier. */
	err = gpio_pin_configure(gpio0_dev, I2S_SD_MODE_PIN, GPIO_OUTPUT_ACTIVE);
	if (err != 0) {
		LOG_ERR("Could not configure P0.%u for MAX98357A SD_MODE (%d)",
			I2S_SD_MODE_PIN, err);
		return err;
	}

	cfg.word_size = 16U;
	cfg.channels = I2S_CHANNEL_COUNT;
	cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	cfg.frame_clk_freq = I2S_SAMPLE_RATE_HZ;
	cfg.block_size = I2S_TX_BLOCK_SIZE;
	cfg.timeout = 1000U;
	cfg.options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER;
	cfg.mem_slab = &i2s_tx_slab;

	err = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
	if (err != 0) {
		LOG_ERR("I2S configuration failed (%d)", err);
		return err;
	}

	k_thread_create(&i2s_worker_thread, i2s_worker_stack,
			K_THREAD_STACK_SIZEOF(i2s_worker_stack), i2s_worker,
			NULL, NULL, NULL, I2S_WORKER_PRIORITY, 0, K_NO_WAIT);
	initialized = true;
	LOG_INF("I2S TX ready: %u Hz, 16-bit stereo", I2S_SAMPLE_RATE_HZ);

	return 0;
}

int audio_i2s_submit(const int16_t *mono_samples, size_t sample_count)
{
	struct audio_pcm_frame frame = {0};

	if (!initialized || mono_samples == NULL || sample_count != I2S_SAMPLES_PER_FRAME) {
		return -EINVAL;
	}

	memcpy(frame.samples, mono_samples, sizeof(frame.samples));
	return k_msgq_put(&pcm_queue, &frame, K_NO_WAIT);
}

void audio_i2s_stop(void)
{
	struct audio_pcm_frame frame = {.stop = true};

	if (!initialized) {
		return;
	}

	/* Discard pending PCM on stream stop; the worker drains the active I2S block. */
	k_msgq_purge(&pcm_queue);
	if (k_msgq_put(&pcm_queue, &frame, K_NO_WAIT) != 0) {
		LOG_WRN("Could not queue I2S stop request");
	}
}

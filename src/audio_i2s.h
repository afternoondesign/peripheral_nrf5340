/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AUDIO_I2S_H
#define AUDIO_I2S_H

#include <stddef.h>
#include <stdint.h>

int audio_i2s_init(void);
int audio_i2s_submit(const int16_t *mono_samples, size_t sample_count);
void audio_i2s_stop(void);

#endif /* AUDIO_I2S_H */

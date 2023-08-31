/* 
 *  Squeezelite for esp32
 *
 *  (c) Philippe G. 2020, philippe_44@outlook.com
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */
 
#pragma once

void equalizer_init(void);
void equalizer_open(u32_t sample_rate);
void equalizer_close(void);
void equalizer_update(s8_t *gain);
void equalizer_process(u8_t *buf, u32_t bytes, u32_t sample_rate);
void equalizer_set_loudness(unsigned int left, unsigned int right);
void equalizer_apply_loudness();
// void equalizer_two_way_update(void);
// void biquad_i16(u8_t *buf, frames_t count, float *coef, float *w, int channel);
void filter_biquad_i16(struct buffer *outputbuf, frames_t count, float *coef, float *w, int channel);
void filters_biquad_i16(struct buffer *outputbuf, frames_t count, int channel);
void equlizer_biquad_i16(struct buffer *outputbuf, frames_t count, int channel);
void apply_biquad(struct buffer *outputbuf, frames_t count);
void apply_biquad_eq(struct buffer *outputbuf, frames_t count);
void equlizer_biquad_gen_peakingEQ_f32(float *coeffs, float f, float Fs, float qFactor, float gain);
void equalizer_calc_real_gains();
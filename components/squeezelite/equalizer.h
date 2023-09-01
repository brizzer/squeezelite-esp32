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

typedef enum {LEFT = 0, RIGHT = 1, BOTH = 2} channel_t;

typedef struct {
	char type[3];
	float frequency;
	float gain;
	float q;
	float biquad_coeffs[5];
	float biquad_w[2];  
	channel_t channel;
	// bool coeffs_calculated;
} filter_config_t;

// void equalizer_get_loudness_factor();
// s8_t* equalizer_get_config(void);
// void channel_filter_get_config();
// void arb_filters_get_config(char *config_name);
// void filters_reset(filter_config_t *filter, u8_t *n_filt, u8_t max_filters);
// void filters_add(filter_config_t *filter, u8_t *n_filt, char *type, float freq, float gain, float q, channel_t channel);
// void filters_calc_coeff(filter_config_t *filter, u8_t *n_filt, u32_t sample_rate);
// int equalizer_get_config_value_int(char *config_name);

void equalizer_init(void);
void equalizer_open(u32_t sample_rate);
void equalizer_close(void);
void equalizer_update(s8_t *gain);
void equalizer_process(u8_t *buf, u32_t bytes, u32_t sample_rate);
void equalizer_set_loudness(unsigned int left, unsigned int right);
void equalizer_apply_loudness();
// void equalizer_two_way_update(void);
// void biquad_i16(u8_t *buf, frames_t count, float *coef, float *w, int channel);
// void filter_biquad_i16(struct buffer *outputbuf, frames_t count, float *coef, float *w, int channel);
// void filters_biquad_i16(struct buffer *outputbuf, frames_t count, int channel);
void filters_reset(filter_config_t *filter, u8_t *n_filt, u8_t max_filters);
void filters_add(filter_config_t *filter, u8_t *n_filt, char *type, float freq, float gain, float q, channel_t channel);
int equalizer_get_config_value_int(char *config_name);

// void equlizer_biquad_i16(struct buffer *outputbuf, frames_t count, int channel);
// void apply_biquad(struct buffer *outputbuf, frames_t count);
// void apply_biquad_eq(struct buffer *outputbuf, frames_t count);
void apply_filters(struct buffer *outputbuf, frames_t count);
void filter_all(struct buffer *outputbuf, frames_t count, u8_t channel);
void filter_biquad_i16(float *val, float *coef, float *w);
void equlizer_biquad_gen_peakingEQ_f32(float *coeffs, float f, float Fs, float qFactor, float gain);
void equalizer_calc_real_gains();

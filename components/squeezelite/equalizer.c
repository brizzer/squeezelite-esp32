/*
 *  Squeezelite for esp32
 *
 *  (c) Philippe G. 2020, philippe_44@outlook.com
 *
 *  This software is released under the MIT License.
 *  https://opensource.org/licenses/MIT
 *
 */

#include "math.h"
#include "platform_config.h"
#include "squeezelite.h"
#include "equalizer.h"
// #include "esp_equalizer.h"
// #include "dsps_biquad.h"
#include "dsps_biquad_gen.h"

#pragma GCC push_options
#pragma GCC optimize ("-O2")

#define EQ_BANDS 10
#define MAX_ARB_FILTERS 10
#define MAX_CHANNEL_FILERS 5
// #define MAX_FILTERS EQ_BANDS+MAX_ARB_FILTERS+CHANNEL_FILERS
#define N_GAIN_CALC_LOOPS 5 //10
#define EQ_Q 1.0
#define CHANNEL_DELAY_BUFFER_SIZE 100 // 100us at 96kHz

static log_level loglevel = lDEBUG; //lINFO;lWARN;
float loudness_factor = 0;
float adjusted_gain = 0;
float biquad_d0;
float temp_val;

#define POLYNOME_COUNT 6
static const float loudness_envelope_coefficients[EQ_BANDS][POLYNOME_COUNT] =
{ {5.5169301499257067e+001, 6.3671410796029004e-001,
  -4.2663226432095233e-002, 8.1063072336581246e-004,
  -7.3621858933917722e-006, 2.5349489594339575e-008},
 {3.7716143859944118e+001, 1.2355293276538579e+000,
  -6.6435374582217863e-002, 1.2976763440259382e-003,
  -1.1978732496353172e-005, 4.1664114634622593e-008},
 {2.5103632377146837e+001, 1.3259150615414637e+000,
  -6.6332442135695099e-002, 1.2845279812261677e-003,
  -1.1799885217545631e-005, 4.0925911584040685e-008},
 {1.3159168212144563e+001, 8.8149357628440639e-001,
  -4.0384121097225931e-002, 7.3843501027501322e-004,
  -6.5508794453097008e-006, 2.2221997141120518e-008},
 {5.1337853800151700e+000, 4.0817077967582394e-001,
  -1.4107826528626457e-002, 1.5251066311713760e-004,
  -3.6689819583740298e-007, -2.0390798774727989e-009},
 {3.1432364156464315e-001, 9.1260548140023004e-002,
  -3.5012124633183438e-004, -8.6023911664606992e-005,
  1.6785606828245921e-006, -8.8269731094371646e-009},
 {-4.0965062397075833e+000, 1.3667010948271402e-001,
  2.4775896786988390e-004, -9.6620399661858641e-005,
  1.7733690952379155e-006, -9.1583104942496635e-009},
 {-9.0275786029994176e+000, 2.6226938845184250e-001,
  -6.5777547972402156e-003, 1.0045957188977551e-004,
  -7.8851000325128971e-007, 2.4639885209682384e-009},
 {-4.4275018199195815e+000, 4.5399572638241725e-001,
  -2.4034902766833462e-002, 5.9828953622534668e-004,
  -6.2893971217140864e-006, 2.3133296592719627e-008},
 {1.4243299202697818e+001, 3.6984458807056630e-001,
  -3.0413994109395680e-002, 7.6700105080386904e-004,
  -8.2777185209388079e-006, 3.1352890650784970e-008} };

// float biquad_eq_coefficients[2][EQ_BANDS][5];
// float biquad_eq_coefficients[EQ_BANDS][5];
// float biquad_eq_w[2][EQ_BANDS][2] = {0};

// float biquad_lpf_coeffs[5];
// float biquad_lpf_w[2] = {0};
bool biquad_lpf_process = false;

// float biquad_hpf_coeffs[5];
// float biquad_hpf_w[2] = {0};
bool biquad_hpf_process = false;

float eq_taps[EQ_BANDS] = {31,62,125,250,500,1000,2000,4000,8000,16000};

static struct {
	void *handle;
	float gain[EQ_BANDS];
	float real_gain[EQ_BANDS];
	bool update;
	// float two_way_gain_left[EQ_BANDS];
	// float two_way_gain_right[EQ_BANDS];
	float low_pass_filter_freq;
	float high_pass_filter_freq;
	u32_t sample_rate;
} equalizer = { .update = true, .sample_rate = 44100 };


u16_t channel_delay = 0;
channel_t channel_delay_channel = BOTH;
ISAMPLE_T channel_delay_buffer[CHANNEL_DELAY_BUFFER_SIZE] = {0};
u16_t channel_delay_buffer_pos = 0;

float channel_gain[2];

filter_config_t arb_filters[MAX_ARB_FILTERS];
filter_config_t eq_filters[EQ_BANDS];
filter_config_t channel_filters[MAX_CHANNEL_FILERS];
// u8_t n_filters = 0;
u8_t n_arb_filters = 0;
u8_t n_eq_filters = 0;
u8_t n_channel_filters = 0;

#define N_CLOCK_CYCLES_AVERAGE 100
u8_t n_clock_cycles_run = 0;
float total_clock_cycles = 0.0;
unsigned int n_clock_cycles_samples = 0;



// bool equalizer_active = false;

// static const char * TAG = "audio controls";

/****************************************************************************************
 * Get the loudness value from config and calculate the loudness level factor
 */
void equalizer_get_loudness_factor() {
	char* config = config_alloc_get_default(NVS_TYPE_STR, "loudness", "0", 0);
	if (!config) {
		LOG_WARN("Equalizer Config not found");
	}
	else {
		int loudness_level = atoi(config);
		loudness_factor =
			(loudness_level == 0) ? 0 : pow((loudness_level / 100.0), 2);
		free(config);
	}
}
/****************************************************************************************
 * get the equalizer config
 */
s8_t* equalizer_get_config(void) {
	s8_t* pGains = malloc(sizeof(s8_t) * EQ_BANDS);
	memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
	uint8_t num_entries = 0;
	char* config = config_alloc_get(NVS_TYPE_STR, "equalizer");
	if (!config) {
		LOG_WARN("Equalizer Config not found");
	}
	else {
		char* p = strtok(config, ", !");
		for (int i = 0; p && i < EQ_BANDS; i++) {
			pGains[i] = atoi(p);
			num_entries++;
			p = strtok(NULL, ", :");
		}
		if (num_entries < EQ_BANDS) {
			LOG_ERROR("Invalid equalizer settings. Resetting it");
			memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
		}
		free(config);
	}
	return pGains;
}

/****************************************************************************************
 * get the equalizer config
 */
// s8_t* equalizer_get_config(char *config_name) {
// 	s8_t* pGains = malloc(sizeof(s8_t) * EQ_BANDS);
// 	memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
// 	uint8_t num_entries = 0;
// 	char* config = config_alloc_get(NVS_TYPE_STR, config_name);
	
// 	if (!config) {
// 		LOG_WARN("%s Config not found", config_name);
// 	}
// 	else {
// 		LOG_INFO("equalizer_get_config %s: %s", config_name, config);
// 		char* p = strtok(config, ", !");
// 		for (int i = 0; p && i < EQ_BANDS; i++) {
// 			pGains[i] = atoi(p);
// 			num_entries++;
// 			p = strtok(NULL, ", :");
// 		}
// 		if (num_entries < EQ_BANDS) {
// 			LOG_ERROR("Invalid %s settings. Resetting it", config_name);
// 			memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
// 		}
// 		free(config);
// 	}
// 	return pGains;
// }

/****************************************************************************************
 * Get channel filter config
 */
// void channel_filter_get_config() {

// 	filters_reset(channel_filters, &n_channel_filters, MAX_CHANNEL_FILERS);
// 	float lpf_freq = equalizer_get_config_value_int("eq_lpf_freq");
// 	if (lpf_freq > 0) {
// 		biquad_lpf_process = true;
// 		filters_add(channel_filters, &n_channel_filters, "LP", lpf_freq, 0.0, 0.7, RIGHT);
// 	}

// 	float hpf_freq = equalizer_get_config_value_int("eq_hpf_freq");
// 	if (hpf_freq > 0) {
// 		filters_add(channel_filters, &n_channel_filters, "LP", hpf_freq, 0.0, 0.7, LEFT);
// 	}

// 	channel_gain[LEFT] = equalizer_get_config_value_float("gain_left");
// 	channel_gain[RIGHT] = equalizer_get_config_value_float("gain_right");
// }

/****************************************************************************************
 * Get channel filter config
 */
void delay_get_config(u32_t sample_rate) {

	u16_t delay_left = equalizer_get_config_value_int("delay_left");
	u16_t delay_right = equalizer_get_config_value_float("delay_right");

	channel_delay_channel = BOTH;
	channel_delay = 0;

	if (delay_left > delay_right) {
		channel_delay = (u16_t)((float)(delay_left - delay_right) * (float)sample_rate / 1e6);
		channel_delay_channel = LEFT;
	} 
	if (delay_right > delay_left) {
		channel_delay = (u16_t)((float)(delay_right - delay_left) * (float)sample_rate / 1e6);
		channel_delay_channel = RIGHT;
	} 

	memset(channel_delay_buffer, 0x00, CHANNEL_DELAY_BUFFER_SIZE * sizeof(s16_t));

}

/****************************************************************************************
 * Get arbitrary filter config
 */
void filters_get_config(char *config_name, filter_config_t *filter, u8_t *n_filt, u8_t max_filters, channel_t channel, bool reset) {
	uint8_t num_entries = 0;
	char* config = config_alloc_get(NVS_TYPE_STR, config_name);
	LOG_INFO("filter_get_config %s: %s", config_name, config);
	if (reset) filters_reset(filter, &(*n_filt), max_filters);
	
	if (!config) {
		LOG_WARN("%s Config not found", config_name);
	}
	else {

		char* p = strtok(config, ", !");
		while( p != NULL ) {
			num_entries++;
			p = strtok(NULL, ", :");
		}
		LOG_INFO("num_entries %d", num_entries);
		if (((num_entries % 4) != 0) || (num_entries > max_filters*4)) {
			LOG_ERROR("Invalid %s settings. Resetting it", config_name);
		} else {

			// *n_filt = 0;
			config = config_alloc_get(NVS_TYPE_STR, config_name);
			p = strtok(config, ", !");
			while( p != NULL ) {
				// [Type],[Frequency],[Gain],[Q]
				strncpy(filter[*n_filt].type, p, 2);
				// LOG_INFO("filter_get_config ok %s", p);
				p = strtok(NULL, ", :");
				// LOG_INFO("filter_get_config ok %s", p);
				filter[*n_filt].frequency = (float)atoff(p);
				p = strtok(NULL, ", :");
				// LOG_INFO("filter_get_config ok %s", p);
				filter[*n_filt].gain = (float)atoff(p);
				p = strtok(NULL, ", :");
				// LOG_INFO("filter_get_config ok %s", p);
				filter[*n_filt].q = (float)atoff(p);
				p = strtok(NULL, ", :");
				filter[*n_filt].channel = channel;
				(*n_filt)++;
			}
			
			for (int i = 0; i < *n_filt; i++)
			{
				LOG_INFO("%s %d, type: %s, f %f, g %f, q %f", config_name, i, arb_filters[i].type, arb_filters[i].frequency, arb_filters[i].gain, arb_filters[i].q);
			}
		}
		free(config);
		free(p);
	}
}

void filters_reset(filter_config_t *filter, u8_t *n_filt, u8_t max_filters) {
	for (int i = 0; i < max_filters; i++)
	{
		filter[i].frequency = 0.0;
		filter[i].gain = 0.0;
		filter[i].q = 0.0;
		strcpy(filter[i].type, "");
		filter[i].biquad_w[0][0] = 0;
		filter[i].biquad_w[0][1] = 0;
		filter[i].biquad_w[1][0] = 0;
		filter[i].biquad_w[1][1] = 0;
		
		filter[i].channel = BOTH;
		filter[i].biquad_coeffs[0] = 1;
		filter[i].biquad_coeffs[1] = -2;
		filter[i].biquad_coeffs[2] = 1;
		filter[i].biquad_coeffs[3] = -2;
		filter[i].biquad_coeffs[4] = 1;
		// filter[i].process = false;
	}
	*n_filt = 0;
}

void filters_add(filter_config_t *filter, u8_t *n_filt, char *type, float freq, float gain, float q, channel_t channel) {
	filter[*n_filt].frequency = freq;
	filter[*n_filt].gain = gain;
	filter[*n_filt].q = q;
	strncpy(filter[*n_filt].type, type, 2);
	filter[*n_filt].biquad_w[0][0] = 0;
	filter[*n_filt].biquad_w[0][1] = 0;
	filter[*n_filt].biquad_w[1][0] = 0;
	filter[*n_filt].biquad_w[1][1] = 0;
	filter[*n_filt].channel = channel;
	// filter[*n_filt].coeffs_calculated = false;
	(*n_filt)++;
}

void filters_set(filter_config_t *filter, u8_t *n_filt, char *type, float freq, float gain, float q, channel_t channel) {
	filter[*n_filt].frequency = freq;
	filter[*n_filt].gain = gain;
	filter[*n_filt].q = q;
	strncpy(filter[*n_filt].type, type, 2);
	filter[*n_filt].biquad_w[0][0] = 0;
	filter[*n_filt].biquad_w[0][1] = 0;
	filter[*n_filt].biquad_w[1][0] = 0;
	filter[*n_filt].biquad_w[1][1] = 0;
	filter[*n_filt].channel = channel;
	// filter[*n_filt].coeffs_calculated = false;
	// (*n_filt)++;
}

__attribute__((optimize("O2"))) void filters_calc_coeff(filter_config_t *filter, u8_t *n_filt, u32_t sample_rate) {
	LOG_DEBUG("Calculating filters");
	for (int i = 0; i < *n_filt; i++)  // PK, LP, HP, LS, HS, NO, AP
	{
		// LOG_DEBUG("filter type: %s, %d", filter[i].type, strcmp(filter[i].type, "PK"));
		// filter[i].process = true;
		if (strcmp(filter[i].type, "PK") == 0) // peaking EQ
		{
			equlizer_biquad_gen_peakingEQ_f32(filter[i].biquad_coeffs, filter[i].frequency, (float)sample_rate, filter[i].q, filter[i].gain);
		} 
		else if (strcmp(filter[i].type, "LP") == 0) // Low pass
		{
			dsps_biquad_gen_lpf_f32(filter[i].biquad_coeffs, filter[i].frequency/(float)sample_rate, filter[i].q);
		} 
		else if (strcmp(filter[i].type, "HP") == 0) // High pass
		{
			dsps_biquad_gen_hpf_f32(filter[i].biquad_coeffs, filter[i].frequency/(float)sample_rate, filter[i].q);
		}
		else if (strcmp(filter[i].type, "L1") == 0) // Low pass, 1st order
		{
			equlizer_biquad_gen_lpf1_f32(filter[i].biquad_coeffs, filter[i].frequency, (float)sample_rate);
		} 
		else if (strcmp(filter[i].type, "H1") == 0) // High pass, 1st order
		{
			equlizer_biquad_gen_hpf1_f32(filter[i].biquad_coeffs, filter[i].frequency, (float)sample_rate);
		}
		else if (strcmp(filter[i].type, "LS") == 0) // Low shelf
		{
			dsps_biquad_gen_lowShelf_f32(filter[i].biquad_coeffs, filter[i].frequency/(float)sample_rate, filter[i].gain, filter[i].q);
		}
		else if (strcmp(filter[i].type, "HS") == 0) // High shelf
		{
			dsps_biquad_gen_highShelf_f32(filter[i].biquad_coeffs, filter[i].frequency/(float)sample_rate, filter[i].gain, filter[i].q);
		}
		else if (strcmp(filter[i].type, "NO") == 0) // Notch
		{
			dsps_biquad_gen_notch_f32(filter[i].biquad_coeffs, filter[i].frequency/(float)sample_rate, filter[i].gain, filter[i].q);
		}
		else if (strcmp(filter[i].type, "AP") == 0) // All pass
		{
			dsps_biquad_gen_allpass360_f32(filter[i].biquad_coeffs, filter[i].frequency/(float)sample_rate, filter[i].q);
		}
		else if (strcmp(filter[i].type, "GA") == 0) // Gain only
		{
			float gain_linear = powf(10, filter[i].gain/20);
			filter[i].biquad_coeffs[0] = gain_linear;
			filter[i].biquad_coeffs[1] = 0;
			filter[i].biquad_coeffs[2] = 0;
			filter[i].biquad_coeffs[3] = 0;
			filter[i].biquad_coeffs[4] = 0;

		}
		else /* Set to flat: */
		{
			filter[i].biquad_coeffs[0] = 1;
			filter[i].biquad_coeffs[1] = 0;
			filter[i].biquad_coeffs[2] = 0;
			filter[i].biquad_coeffs[3] = 0;
			filter[i].biquad_coeffs[4] = 0;
			LOG_WARN("Flat filter");
		}
		// TODO: negate the last two coeffs
	}
}

/****************************************************************************************
 * 
 */
int equalizer_get_config_value_int(char *config_name) {
	int config_val = 0;
	char* config = config_alloc_get_default(NVS_TYPE_STR, config_name, "0", 0);
	if (!config) {
		LOG_WARN("%s Config not found", config_name);
	}
	else {
		
		config_val = atoi(config);
		LOG_INFO("equalizer_get_config_value_int %s: %d", config_name, config_val);
		free(config);
	}
	return config_val;
}

/****************************************************************************************
 * 
 */
int equalizer_get_config_value_float(char *config_name) {
	float config_val = 0;
	char* config = config_alloc_get_default(NVS_TYPE_STR, config_name, "0.0", 0);
	if (!config) {
		LOG_WARN("%s Config not found", config_name);
	}
	else {
		
		config_val = atoff(config);
		LOG_INFO("equalizer_get_config_value_float %s: %d", config_name, config_val);
		free(config);
	}
	return config_val;
}

/****************************************************************************************
 * update equalizer gain
 */
void equalizer_update(s8_t* gain) {
	char config[EQ_BANDS * 4 + 1] = {};
	int n = 0;
	for (int i = 0; i < EQ_BANDS; i++) {
		equalizer.gain[i] = gain[i];
		n += sprintf(config + n, "%d,", gain[i]);
	}
	config[n - 1] = '\0';
	config_set_value(NVS_TYPE_STR, "equalizer", config);
	equalizer_apply_loudness();
	equalizer_calc_real_gains();

	filters_reset(eq_filters, &n_eq_filters, EQ_BANDS);
	for (int i = 0; i < EQ_BANDS; i++) {
		if (equalizer.real_gain[i] != 0.0) {
			filters_add(eq_filters, &n_eq_filters, "PK", eq_taps[i], equalizer.real_gain[i], EQ_Q, BOTH);
			// filters_set(eq_filters, i, "PK", eq_taps[i], equalizer.real_gain[i], EQ_Q, BOTH);
			// eq_filters[i].frequency = eq_taps[i];
			// eq_filters[i].gain = equalizer.real_gain[i];
			// eq_filters[i].q = EQ_Q;
			// strncpy(eq_filters[i].type, "PK", 3);
			// eq_filters[i].channel = BOTH;
		}
	}
	// equalizer.update = true;
	filters_calc_coeff(eq_filters, &n_eq_filters, equalizer.sample_rate);
}

// void biquad_update(u32_t sample_rate) {
// 	float lpf_freq = equalizer_get_config_value_int("eq_lpf_freq");
//     dsps_biquad_gen_lpf_f32(biquad_lpf_coeffs, lpf_freq/(float)sample_rate, 0.7);

	
// 	if (lpf_freq > 0) {
// 		biquad_lpf_process = true;
// 	} else {
// 		biquad_lpf_process = false;
// 	}

// 	float hpf_freq = equalizer_get_config_value_int("eq_hpf_freq");
//     dsps_biquad_gen_hpf_f32(biquad_hpf_coeffs, hpf_freq/(float)sample_rate, 0.7);

	
// 	if (lpf_freq > 0) {
// 		biquad_hpf_process = true;
// 	} else {
// 		biquad_hpf_process = false;
// 	}

// }
/****************************************************************************************
 * initialize equalizer
 */
void equalizer_init(void) {
	s8_t* pGains = equalizer_get_config();
	filters_reset(eq_filters, &n_eq_filters, EQ_BANDS);
	n_eq_filters = EQ_BANDS;

	equalizer_update(pGains);
	// arb_filters_get_config("filter_json");
	filters_get_config("filter_json", arb_filters, &n_arb_filters, MAX_ARB_FILTERS, BOTH, true);
	filters_get_config("filters_left", channel_filters, &n_channel_filters, MAX_CHANNEL_FILERS, LEFT, true);
	filters_get_config("filters_right", channel_filters, &n_channel_filters, MAX_CHANNEL_FILERS, RIGHT, false);
	// channel_filter_get_config();
	delay_get_config(44100);
	// equalizer_two_way_update();
	
	// filters_calc_coeff(eq_filters, &n_eq_filters, 48000);
	// filters_calc_coeff(arb_filters, &n_arb_filters, 48000);
	// filters_calc_coeff(channel_filters, &n_channel_filters, 48000);

	// biquad_update(48000.0);
	// filters_update(48000.0);
	
	LOG_INFO("initializing equalizer, loudness %s", loudness_factor > 0 ? "ENABLED" : "DISABLED");
	free(pGains);
}

/****************************************************************************************
 * open equalizer
 * 
 * Called when the sample rate is changed
 */
void equalizer_open(u32_t sample_rate) {
	// in any case, need to clear update flag
	equalizer.update = false;
	equalizer.sample_rate = sample_rate;

	if (sample_rate != 11025 && sample_rate != 22050 && sample_rate != 44100 && sample_rate != 48000) {
		LOG_WARN("equalizer only supports 11025, 22050, 44100 and 48000 sample rates, not %u", sample_rate);
		// return;
	}

	// equalizer.handle = esp_equalizer_init(2, sample_rate, EQ_BANDS, 0);
	// biquad_update(sample_rate);
	// filters_update(sample_rate);
	filters_calc_coeff(eq_filters, &n_eq_filters, sample_rate);
	filters_calc_coeff(arb_filters, &n_arb_filters, sample_rate);
	filters_calc_coeff(channel_filters, &n_channel_filters, sample_rate);
	delay_get_config(sample_rate);
	// if (equalizer.handle) {
		// bool active = false;

	// 	for (int i = 0; i < EQ_BANDS; i++) {
	// // 		esp_equalizer_set_band_value(equalizer.handle, equalizer.gain[i] + equalizer.two_way_gain_left[i], i, 0);
	// // 		esp_equalizer_set_band_value(equalizer.handle, equalizer.gain[i] + equalizer.two_way_gain_right[i], i, 1);
	// 		// active |= (equalizer.gain[i] != 0) || (equalizer.two_way_gain_left[i] != 0) || (equalizer.two_way_gain_right[i] != 0); 
	// 		equalizer_active |= (equalizer.gain[i] != 0); 
	// // 		LOG_INFO("Left band %f", equalizer.gain[i] + equalizer.two_way_gain_left[i]);
	// // 		LOG_INFO("Right band %f", equalizer.gain[i] + equalizer.two_way_gain_right[i]);
	// 	}

	// 	// do not activate equalizer if all gain are 0
	// 	if (!active) equalizer_close();

	// 	LOG_INFO("equalizer initialized %u", active);
	// } else {
	// 	LOG_WARN("can't init equalizer");
	// }
}

/****************************************************************************************
 * close equalizer
 */
void equalizer_close(void) {
	// if (equalizer.handle) {
	// 	esp_equalizer_uninit(equalizer.handle);
	// 	equalizer.handle = NULL;
	// }
}

/****************************************************************************************
 * Prints the equalizer settings to the console
 */
void equalizer_print_bands(const char* message, float* values, uint8_t count) {
	assert(count > 0);
	char* bands = malloc(strlen(message) + count * 8 + 1);
	int n = 0;
	assert(values);
	n += sprintf(bands + n, "%s", message);
	for (int i = 0; i < count; i++) {
		n += sprintf(bands + n, "%0.2f,", values[i]);
	}
	bands[n - 1] = '\0';
	LOG_DEBUG("%s", bands);
	free(bands);
}
/****************************************************************************************
 * Calculates loudness values for each band at a given volume
 */
float* calculate_loudness_curve(float volume_level) {
	LOG_DEBUG("Calculating loudness curve for volume level %.3f", volume_level);
	float* loudness_curve = malloc(EQ_BANDS * sizeof(float));
	memset(loudness_curve, 0x00, EQ_BANDS * sizeof(float));
	equalizer_get_loudness_factor();
	for (int i = 0; i < EQ_BANDS && loudness_factor > 0; i++) {
		for (int j = 0; j < POLYNOME_COUNT; j++) {
			loudness_curve[i] +=
				loudness_envelope_coefficients[i][j] * pow(volume_level, j);
		}
		loudness_curve[i] *= loudness_factor;
	}
	if (loglevel >= lDEBUG) {
	equalizer_print_bands("calculated Loudness: ", loudness_curve, EQ_BANDS);
	}
	
	return loudness_curve;
}

/****************************************************************************************
 * Combine Loudness and user EQ settings and apply them
 */
void equalizer_apply_loudness() {
	// s8_t* pGains = equalizer_get_config();
	// filter_get_config("filter_json");
	// equalizer_two_way_update();
	float* loudness_curve = calculate_loudness_curve(adjusted_gain);
	for (int i = 0; i < EQ_BANDS; i++) {
		equalizer.gain[i] = (loudness_curve[i] + equalizer.gain[i]);
	}
	equalizer_print_bands("Combined Loudness: ", equalizer.gain, EQ_BANDS);
	
	free(loudness_curve);
	// free(pGains);
	// equalizer.update = true;
}

/****************************************************************************************
 * process equalizer
 */
void equalizer_process(u8_t *buf, u32_t bytes, u32_t sample_rate) {
	// don't want to process with output locked, so take the small risk to miss one parametric update
	if (equalizer.update) {
		// equalizer_close();
		equalizer_open(sample_rate);
	}
	equalizer.sample_rate = sample_rate;

	// if (equalizer.handle) {
	// 	esp_equalizer_process(equalizer.handle, buf, bytes, sample_rate, 2);
	// }

	// if (biquad_lpf_process) {
	// 	biquad_i16(buf, bytes, biquad_lpf_coeffs, biquad_lpf_w, 0);
	// }
}

/****************************************************************************************
 * Solver to calculate the real gains to match the set values
 */
__attribute__((optimize("O2"))) void equalizer_calc_real_gains(){
	float factor1 = 0.31;
	float factor2 = 0.06;
	
	for (u8_t i = 0; i < EQ_BANDS; i++)
	{
		equalizer.real_gain[i] = equalizer.gain[i];
	}
	
	for (u8_t n = 0; n < N_GAIN_CALC_LOOPS; n++)
	{
		equalizer.real_gain[0] = equalizer.gain[0] -  equalizer.real_gain[1] * factor1 - equalizer.real_gain[2] * factor2;
		equalizer.real_gain[1] = equalizer.gain[1] -  (equalizer.real_gain[0] + equalizer.real_gain[2]) * factor1 - equalizer.real_gain[3] * factor2;
		for (u8_t band = 2; band < (EQ_BANDS-2); band++)
		{
			equalizer.real_gain[band] = equalizer.gain[band] -  (equalizer.real_gain[band-1] + equalizer.real_gain[band+1]) * factor1 - (equalizer.real_gain[band-2] + equalizer.real_gain[band+2]) * factor2;
		}
		equalizer.real_gain[EQ_BANDS-2] = equalizer.gain[EQ_BANDS-2] -  (equalizer.real_gain[EQ_BANDS-3] + equalizer.real_gain[EQ_BANDS-1]) * factor1 - equalizer.real_gain[EQ_BANDS-4] * factor2;
		equalizer.real_gain[EQ_BANDS-1] = equalizer.gain[EQ_BANDS-1] -  equalizer.real_gain[EQ_BANDS-2] * factor1 - equalizer.real_gain[EQ_BANDS-3] * factor2;
	}
}

/****************************************************************************************
 * Updates the loudness EQ curve based on a new volume level
 */
void equalizer_set_loudness(unsigned int left, unsigned int right) {
	LOG_DEBUG("Setting loudness for volume %d/%d", left, right);
	// Calculate the average gain
	unsigned int average_gain = (left + right) / 2;
	// Convert the average gain to a logarithmic format (range -60 to 0)
	float log_gain = average_gain > 0
		? log2((float)average_gain / (1 << 16)) * 6.0206
		: -60; // Convert to dB
	adjusted_gain = (log_gain + 60.0) / 60.0 * 100.0;
	equalizer_apply_loudness();
}

__attribute__((optimize("O2"))) void apply_filters(struct buffer *outputbuf, frames_t count) {
	// #if loglevel >= lDEBUG
	unsigned int start_cc = xthal_get_ccount();
	// #endif
	filter_all(outputbuf, count, 0);
	filter_all(outputbuf, count, 1);
	// #if loglevel >= lDEBUG
	unsigned int end_cc = xthal_get_ccount();
	n_clock_cycles_run++;
	total_clock_cycles += end_cc - start_cc;
	n_clock_cycles_samples += count;
	if (n_clock_cycles_run > N_CLOCK_CYCLES_AVERAGE) {
		//LOG_DEBUG("Apply filter cycles per sample:,%f,len,%d,n_eq,%d,n_arb,%d,n_channel,%d", total_clock_cycles/n_clock_cycles_samples, n_clock_cycles_samples, n_eq_filters, n_arb_filters, n_channel_filters);
		LOG_DEBUG("Apply filter cycles per sample, len, n_eq, n_arb, n_channel, %f, %d, %d, %d, %d", total_clock_cycles/n_clock_cycles_samples, n_clock_cycles_samples, n_eq_filters, n_arb_filters, n_channel_filters);
		total_clock_cycles = 0.0;
		n_clock_cycles_run = 0;
		n_clock_cycles_samples = 0;
	}
}

// void apply_biquad(struct buffer *outputbuf, frames_t count) {
// 	if (biquad_lpf_process) {
// 		filter_biquad_i16(outputbuf, count, biquad_lpf_coeffs, biquad_lpf_w, 1); // Right (low) channel
// 	}
// 	if (biquad_hpf_process) {
// 		filter_biquad_i16(outputbuf, count, biquad_hpf_coeffs, biquad_hpf_w, 0); // Left (high) channel
// 	}
// }

// void apply_biquad_eq(struct buffer *outputbuf, frames_t count) {
// 	if (equalizer_active) {
// 		equlizer_biquad_i16(outputbuf, count, 0);
// 		equlizer_biquad_i16(outputbuf, count, 1);
// 	}
// 	if (n_filters > 0) {
// 		filters_biquad_i16(outputbuf, count, 0);
// 		filters_biquad_i16(outputbuf, count, 1);
// 	}
// }


// __attribute__((optimize("O2"))) void biquad_i16(struct buffer *outputbuf, frames_t count, float *coef, float *w, int channel) 

// __attribute__((optimize("O2"))) void biquad_i16(u8_t *buf, frames_t count, float *coef, float *w, int channel) 
// {
// 	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)buf + channel;
// 	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;

// 	while (count--) {
// 		biquad_d0 = (float)((ISAMPLE_T)(*ptr  + (*(ptr + 2)  << 8))); // de-interleavee
//         biquad_d0 = (float)*ptr - coef[3] * w[0] - coef[4] * w[1];
//         *ptr = (ISAMPLE_T)(coef[0] * biquad_d0 + coef[1] * w[0] + coef[2] * w[1]);
//         w[1] = w[0];
//         w[0] = biquad_d0;
// 		ptr += 2;
//     }
// }

__attribute__((optimize("O2"))) void filter_all(struct buffer *outputbuf, frames_t count, u8_t channel)
{

	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;

	
	while (count--) {
		// Apply delay
		if (channel == channel_delay_channel) {
			channel_delay_buffer[channel_delay_buffer_pos] = *ptr;
			channel_delay_buffer_pos++;
			if (channel_delay_buffer_pos > channel_delay) {
				channel_delay_buffer_pos = 0;
			}
			*ptr = channel_delay_buffer[(channel_delay_buffer_pos+1) % channel_delay];
		}

		// Apply filters
		temp_val = (float)*ptr;
		for (int i = 0; i < n_eq_filters; i++)
		{
			filter_biquad_i16(&temp_val, eq_filters[i].biquad_coeffs, eq_filters[i].biquad_w[channel]);
			// equalizer_biquad_f32(&temp_val, eq_filters[i].biquad_coeffs, eq_filters[i].biquad_w[channel]);
		}
		for (int i = 0; i < n_arb_filters; i++)
		{
			filter_biquad_i16(&temp_val, arb_filters[i].biquad_coeffs, arb_filters[i].biquad_w[channel]);
			// equalizer_biquad_f32(&temp_val, arb_filters[i].biquad_coeffs, arb_filters[i].biquad_w[channel]);
		}
		for (int i = 0; i < n_channel_filters; i++)
		{
			if (channel_filters[i].channel == channel) {
				filter_biquad_i16(&temp_val, channel_filters[i].biquad_coeffs, channel_filters[i].biquad_w[channel]);
				// equalizer_biquad_f32(&temp_val, channel_filters[i].biquad_coeffs, channel_filters[i].biquad_w[channel]);
			}
			
		}

		if (temp_val > 2147483646.0) temp_val = 2147483646.0;
		if (temp_val < -2147483646.0) temp_val = -2147483646.0;
		*ptr = (ISAMPLE_T)temp_val;
		ptr += 2;
    }
}

__attribute__((optimize("O2"))) inline void filter_biquad_i16(float *val, float *coef, float *w) 
{
        biquad_d0 = *val - coef[3] * w[0] - coef[4] * w[1];
		*val = coef[0] * biquad_d0 + coef[1] * w[0] + coef[2] * w[1];
        w[1] = w[0];
        w[0] = biquad_d0;
}

// __attribute__((optimize("O2"))) void filter_biquad_i16(struct buffer *outputbuf, frames_t count, float *coef, float *w, int channel) 
// {
// 	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;
// 	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;

// 	while (count--) {
//         biquad_d0 = (float)*ptr - coef[3] * w[0] - coef[4] * w[1];
//         *ptr = (ISAMPLE_T)(coef[0] * biquad_d0 + coef[1] * w[0] + coef[2] * w[1]);
//         w[1] = w[0];
//         w[0] = biquad_d0;
// 		ptr += 2;
//     }
// }

// __attribute__((optimize("O2"))) void equlizer_biquad_i16(struct buffer *outputbuf, frames_t count, int channel) 
// {
// 	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;
// 	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
// 	float temp;
// 	while (count--) {
// 		temp = (float)*ptr;
// 		for (int i = 0; i < EQ_BANDS; i++)
// 		{
// 			// biquad_d0 = temp - biquad_eq_coefficients[channel][i][3] * biquad_eq_w[channel][i][0] - biquad_eq_coefficients[channel][i][4] * biquad_eq_w[channel][i][1];
// 			// temp = (biquad_eq_coefficients[channel][i][0] * biquad_d0 + biquad_eq_coefficients[channel][i][1] * biquad_eq_w[channel][i][0] + biquad_eq_coefficients[channel][i][2] * biquad_eq_w[channel][i][1]);
// 			biquad_d0 = temp - biquad_eq_coefficients[i][3] * biquad_eq_w[channel][i][0] - biquad_eq_coefficients[i][4] * biquad_eq_w[channel][i][1];
// 			temp = (biquad_eq_coefficients[i][0] * biquad_d0 + biquad_eq_coefficients[i][1] * biquad_eq_w[channel][i][0] + biquad_eq_coefficients[i][2] * biquad_eq_w[channel][i][1]);
// 			biquad_eq_w[channel][i][1] = biquad_eq_w[channel][i][0];
// 			biquad_eq_w[channel][i][0] = biquad_d0;
// 		}
// 		*ptr = (ISAMPLE_T)temp;
// 		ptr += 2;
//     }
// }

// __attribute__((optimize("O2"))) void filters_biquad_i16(struct buffer *outputbuf, frames_t count, int channel) 
// {
// 	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;
// 	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
// 	float temp;
// 	while (count--) {
// 		temp = (float)*ptr;
// 		for (int i = 0; i < n_filters; i++)
// 		{
// 			// biquad_d0 = temp - biquad_eq_coefficients[channel][i][3] * biquad_eq_w[channel][i][0] - biquad_eq_coefficients[channel][i][4] * biquad_eq_w[channel][i][1];
// 			// temp = (biquad_eq_coefficients[channel][i][0] * biquad_d0 + biquad_eq_coefficients[channel][i][1] * biquad_eq_w[channel][i][0] + biquad_eq_coefficients[channel][i][2] * biquad_eq_w[channel][i][1]);
// 			biquad_d0 = temp - filters[i].biquad_coeffs[3] * filters[i].biquad_w[0] - filters[i].biquad_coeffs[4] * filters[i].biquad_w[1];
// 			temp = (filters[i].biquad_coeffs[0] * biquad_d0 + filters[i].biquad_coeffs[1] * filters[i].biquad_w[0] + filters[i].biquad_coeffs[2] * filters[i].biquad_w[1]);
// 			filters[i].biquad_w[1] = filters[i].biquad_w[0];
// 			filters[i].biquad_w[0] = biquad_d0;
// 		}
// 		*ptr = (ISAMPLE_T)temp;
// 		ptr += 2;
//     }
// }

__attribute__((optimize("O2"))) void equlizer_biquad_gen_peakingEQ_f32(float *coeffs, float f, float Fs, float qFactor, float gain)
{
    if (qFactor <= 0.0001) {
        qFactor = 0.0001;
    }
	float A = powf(10, gain/40);
    // float Fs = 1;

    float w0 = 2 * M_PI * f / Fs;
    float c = cosf(w0);
    float s = sinf(w0);
    float alpha = s / (2 * qFactor);
	float alpha_div_A = alpha/A;

    float b0 = 1 + alpha*A;
    float b1 = -2 * c;
    float b2 = 1 - alpha*A;
    float a0 = 1 + alpha_div_A;
    float a1 = b1;
    float a2 = 1 - alpha_div_A;

    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}


/****************************************************************************************
 * First order low pass filter, matching Virtuix CAD 2
 */
__attribute__((optimize("O2"))) void equlizer_biquad_gen_lpf1_f32(float *coeffs, float f, float Fs)
{
    float w0 = 2 * M_PI * f / Fs;
	float K = tanf(w0);
	float alpha = 1 + K;

    float b0 = K / alpha;
    float b1 = b0;
    float b2 = 0;
    float a0 = 1;
    float a1 = -(1 - K) / alpha;
    float a2 = 0;

    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}

/****************************************************************************************
 * First order high pass filter, matching Virtuix CAD 2
 */
__attribute__((optimize("O2"))) void equlizer_biquad_gen_hpf1_f32(float *coeffs, float f, float Fs)
{
    float w0 = 2 * M_PI * f / Fs;
	float K = tanf(w0);
	float alpha = 1 + K;

    float b0 = 1 - K / alpha;
    float b1 = -b0;
    float b2 = 0;
    float a0 = 1;
    float a1 = -(1 - K) / alpha;
    float a2 = 0;

    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}


#pragma GCC pop_options

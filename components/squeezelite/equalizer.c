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

#define EQ_BANDS 10
#define MAX_FILTERS 10
#define CHANNEL_FILERS 2
#define N_GAIN_CALC_LOOPS 10

static log_level loglevel = lWARN; //lDEBUG; //lINFO;
float loudness_factor = 0;
float adjusted_gain = 0;
float biquad_d0;

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
float biquad_eq_coefficients[EQ_BANDS][5];
float biquad_eq_w[2][EQ_BANDS][2] = {0};

float biquad_lpf_coeffs[5];
float biquad_lpf_w[2] = {0};
bool biquad_lpf_process = false;

float biquad_hpf_coeffs[5];
float biquad_hpf_w[2] = {0};
bool biquad_hpf_process = false;

float biquad_eq_taps[EQ_BANDS] = {31,62,125,250,500,1000,2000,4000,8000,16000};

typedef enum {LEFT = 0, RIGHT = 1, BOTH = 2} channel_t;

static struct {
	void *handle;
	float gain[EQ_BANDS];
	float real_gain[EQ_BANDS];
	bool update;
	// float two_way_gain_left[EQ_BANDS];
	// float two_way_gain_right[EQ_BANDS];
	float low_pass_filter_freq;
	float high_pass_filter_freq;
} equalizer = { .update = true };

typedef struct {
	char type[3];
	float frequency;
	float gain;
	float q;
	float biquad_coeffs[5];
	float biquad_w[2];  
	channel_t channel;
	bool process;
} filter_config_t;

filter_config_t filters[MAX_FILTERS];
filter_config_t eq_filters[EQ_BANDS];
filter_config_t filters[CHANNEL_FILERS];
u8_t n_filters = 0;
u8_t n_eq_filters = 0;
u8_t n_cannel_filters = 0;


bool equalizer_active = false;

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
// s8_t* equalizer_get_config(void) {
// 	s8_t* pGains = malloc(sizeof(s8_t) * EQ_BANDS);
// 	memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
// 	uint8_t num_entries = 0;
// 	char* config = config_alloc_get(NVS_TYPE_STR, "equalizer");
// 	if (!config) {
// 		LOG_WARN("Equalizer Config not found");
// 	}
// 	else {
// 		char* p = strtok(config, ", !");
// 		for (int i = 0; p && i < EQ_BANDS; i++) {
// 			pGains[i] = atoi(p);
// 			num_entries++;
// 			p = strtok(NULL, ", :");
// 		}
// 		if (num_entries < EQ_BANDS) {
// 			LOG_ERROR("Invalid equalizer settings. Resetting it");
// 			memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
// 		}
// 		free(config);
// 	}
// 	return pGains;
// }

/****************************************************************************************
 * get the equalizer config
 */
s8_t* equalizer_get_config(char *config_name) {
	s8_t* pGains = malloc(sizeof(s8_t) * EQ_BANDS);
	memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
	uint8_t num_entries = 0;
	char* config = config_alloc_get(NVS_TYPE_STR, config_name);
	
	if (!config) {
		LOG_WARN("%s Config not found", config_name);
	}
	else {
		LOG_INFO("equalizer_get_config %s: %s", config_name, config);
		char* p = strtok(config, ", !");
		for (int i = 0; p && i < EQ_BANDS; i++) {
			pGains[i] = atoi(p);
			num_entries++;
			p = strtok(NULL, ", :");
		}
		if (num_entries < EQ_BANDS) {
			LOG_ERROR("Invalid %s settings. Resetting it", config_name);
			memset(pGains, 0x00, sizeof(s8_t) * EQ_BANDS);
		}
		free(config);
	}
	return pGains;
}

void filters_reset(filter_config_t *filter, u8_t *n_filt) {
	for (int i = 0; i < MAX_FILTERS; i++)
	{
		filter[i].frequency = 0.0;
		filter[i].gain = 0.0;
		filter[i].q = 0.0;
		strcpy(filter[i].type, "");
		filter[i].biquad_w[0] = 0;
		filter[i].biquad_w[1] = 0;
		filter[i].channel = BOTH;
		filter[i].process = false;
	}
	n_filt = 0;
}

/****************************************************************************************
 * Get filter config
 */
void filters_get_config(char *config_name) {
	uint8_t num_entries = 0;
	char* config = config_alloc_get(NVS_TYPE_STR, config_name);
	// LOG_INFO("filter_get_config %s: %s", config_name, config);
	filters_reset(filters, n_filters);
	
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
		if (((num_entries % 4) != 0) || (num_entries > MAX_FILTERS*4)) {
			LOG_ERROR("Invalid %s settings. Resetting it", config_name);
		} else {
			uint8_t n_filter = 0;
			config = config_alloc_get(NVS_TYPE_STR, config_name);
			p = strtok(config, ", !");
			while( p != NULL ) {
				// [Type],[Frequency],[Gain],[Q]
				strncpy(filters[n_filter].type, p, 2);
				// LOG_INFO("filter_get_config ok %s", p);
				p = strtok(NULL, ", :");
				// LOG_INFO("filter_get_config ok %s", p);
				filters[n_filter].frequency = (float)atof(p);
				p = strtok(NULL, ", :");
				// LOG_INFO("filter_get_config ok %s", p);
				filters[n_filter].gain = (float)atof(p);
				p = strtok(NULL, ", :");
				// LOG_INFO("filter_get_config ok %s", p);
				filters[n_filter].q = (float)atof(p);
				p = strtok(NULL, ", :");
				n_filter++;
				
			}
			n_filters = n_filter;
			for (int i = 0; i < n_filters; i++)
			{
				LOG_INFO("Filter %d, type: %s, f %f, g %f, q %f", i, filters[i].type, filters[i].frequency, filters[i].gain, filters[i].q);
				// filters[i].frequency = 0.0;
				// filters[i].gain = 0.0;
				// filters[i].q = 0.0;
				// strcpy(filters[i].type, "");
			}
		}
		free(config);
		free(p);
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
	filters_reset(eq_filters, n_eq_filters);
	for (int i = 0; i < EQ_BANDS; i++) {
		
	}
}

void filters_update(u32_t sample_rate){
	LOG_DEBUG("Calculating filters");
	for (int i = 0; i < n_filters; i++)  // PK, LP, HP, LS, HS, NO, AP
	{
		LOG_DEBUG("filter type: %s, %d", filters[i].type, strcmp(filters[i].type, "PK"));
		if (strcmp(filters[i].type, "PK") == 0) // peaking EQ
		{
			equlizer_biquad_gen_peakingEQ_f32(filters[i].biquad_coeffs, filters[i].frequency, (float)sample_rate, filters[i].q, filters[i].gain);
			for (int j = 0; j < 5; j++)
			{
				LOG_INFO("PK filters coeffs %d, %f", i, filters[i].biquad_coeffs[j]);
			}
		} 
		else if (strcmp(filters[i].type, "LP") == 0) // Low pass
		{
			dsps_biquad_gen_lpf_f32(filters[i].biquad_coeffs, filters[i].frequency/(float)sample_rate, filters[i].q);
		} 
		else if (strcmp(filters[i].type, "HP") == 0) // High pass
		{
			dsps_biquad_gen_hpf_f32(filters[i].biquad_coeffs, filters[i].frequency/(float)sample_rate, filters[i].q);
		}
		else if (strcmp(filters[i].type, "LS") == 0) // Low shelf
		{
			dsps_biquad_gen_lowShelf_f32(filters[i].biquad_coeffs, filters[i].frequency/(float)sample_rate, filters[i].gain, filters[i].q);
		}
		else if (strcmp(filters[i].type, "HS") == 0) // High shelf
		{
			dsps_biquad_gen_highShelf_f32(filters[i].biquad_coeffs, filters[i].frequency/(float)sample_rate, filters[i].gain, filters[i].q);
		}
		else if (strcmp(filters[i].type, "NO") == 0) // Notch
		{
			dsps_biquad_gen_notch_f32(filters[i].biquad_coeffs, filters[i].frequency/(float)sample_rate, filters[i].gain, filters[i].q);
		}
		else if (strcmp(filters[i].type, "AP") == 0) // All pass
		{
			dsps_biquad_gen_allpass360_f32(filters[i].biquad_coeffs, filters[i].frequency/(float)sample_rate, filters[i].q);
		}
		else /* Set to flat: */
		{
			filters[i].biquad_coeffs[0] = 1;
			filters[i].biquad_coeffs[1] = -2;
			filters[i].biquad_coeffs[2] = 1;
			filters[i].biquad_coeffs[3] = -2;
			filters[i].biquad_coeffs[4] = 1;
			for (int j = 0; j < 5; j++)
			{
				LOG_INFO("PK filters coeffs %d, %f", i, filters[i].biquad_coeffs[j]);
			}
		}
	}
}

// void equalizer_two_way_update(void) {

// 	// s8_t* two_way_gains_left = equalizer_get_config("eq_left");
// 	// s8_t* two_way_gains_right = equalizer_get_config("eq_right");
// 	// for (int i = 0; i < EQ_BANDS; i++) {
// 	// 	equalizer.two_way_gain_left[i] = two_way_gains_left[i];
// 	// 	equalizer.two_way_gain_right[i] = two_way_gains_right[i];
// 	// }
// }

void biquad_update(u32_t sample_rate) {
	float lpf_freq = equalizer_get_config_value_int("eq_lpf_freq");
    dsps_biquad_gen_lpf_f32(biquad_lpf_coeffs, lpf_freq/(float)sample_rate, 0.7);
	// for (int i = 0; i < 5; i++)
	// {
	// 	LOG_INFO("biquad_lpf_coeffs %f", biquad_lpf_coeffs[i]);
	// }
	
	if (lpf_freq > 0) {
		biquad_lpf_process = true;
	} else {
		biquad_lpf_process = false;
	}

	float hpf_freq = equalizer_get_config_value_int("eq_hpf_freq");
    dsps_biquad_gen_hpf_f32(biquad_hpf_coeffs, hpf_freq/(float)sample_rate, 0.7);
	// for (int i = 0; i < 5; i++)
	// {
	// 	LOG_INFO("biquad_hpf_coeffs %f", biquad_hpf_coeffs[i]);
	// }
	
	if (lpf_freq > 0) {
		biquad_hpf_process = true;
	} else {
		biquad_hpf_process = false;
	}

	for (int i = 0; i < EQ_BANDS; i++){
		// LOG_INFO("equalizer.gain %f, sample_rate %d", equalizer.gain[i], sample_rate);

		// equlizer_biquad_gen_peakingEQ_f32(biquad_eq_coefficients[i], biquad_eq_taps[i], sample_rate, 0.7, equalizer.gain[i]);
		equlizer_biquad_gen_peakingEQ_f32(biquad_eq_coefficients[i], biquad_eq_taps[i], sample_rate, 1.0, equalizer.real_gain[i]);
		// equlizer_biquad_gen_peakingEQ_f32(biquad_eq_coefficients[1][i], biquad_eq_taps[i], sample_rate, 0.7, equalizer.gain[i]);
	}

	// for (int ch = 0; ch < 2; ch++)	{
	// 	for (int i = 0; i < EQ_BANDS; i++){
	// 		for (int j = 0; j < 5; j++)
	// 		{
	// 			LOG_INFO("biquad_eq_coefficients: ch: %d, band: %d, tap %f, coeff: %f", ch, i, biquad_eq_taps[i], biquad_eq_coefficients[ch][i][j]);
	// 		}
	// 	}
	// }
}
/****************************************************************************************
 * initialize equalizer
 */
void equalizer_init(void) {
	s8_t* pGains = equalizer_get_config("equalizer");
	filters_get_config("filter_json");
	filters_update(48000.0);
	// equalizer_two_way_update();
	equalizer_update(pGains);
	biquad_update(48000.0);
	
	LOG_INFO("initializing equalizer, loudness %s", loudness_factor > 0 ? "ENABLED" : "DISABLED");
	free(pGains);
}

/****************************************************************************************
 * open equalizer
 */
void equalizer_open(u32_t sample_rate) {
	// in any case, need to clear update flag
	equalizer.update = false;

	if (sample_rate != 11025 && sample_rate != 22050 && sample_rate != 44100 && sample_rate != 48000) {
		LOG_WARN("equalizer only supports 11025, 22050, 44100 and 48000 sample rates, not %u", sample_rate);
		// return;
	}

	// equalizer.handle = esp_equalizer_init(2, sample_rate, EQ_BANDS, 0);
	biquad_update(sample_rate);
	filters_update(sample_rate);

	// if (equalizer.handle) {
		// bool active = false;

		for (int i = 0; i < EQ_BANDS; i++) {
	// 		esp_equalizer_set_band_value(equalizer.handle, equalizer.gain[i] + equalizer.two_way_gain_left[i], i, 0);
	// 		esp_equalizer_set_band_value(equalizer.handle, equalizer.gain[i] + equalizer.two_way_gain_right[i], i, 1);
			// active |= (equalizer.gain[i] != 0) || (equalizer.two_way_gain_left[i] != 0) || (equalizer.two_way_gain_right[i] != 0); 
			equalizer_active |= (equalizer.gain[i] != 0); 
	// 		LOG_INFO("Left band %f", equalizer.gain[i] + equalizer.two_way_gain_left[i]);
	// 		LOG_INFO("Right band %f", equalizer.gain[i] + equalizer.two_way_gain_right[i]);
		}

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
	equalizer_print_bands("calculated Loudness: ", loudness_curve, EQ_BANDS);
	return loudness_curve;
}

/****************************************************************************************
 * Combine Loudness and user EQ settings and apply them
 */
void equalizer_apply_loudness() {
	s8_t* pGains = equalizer_get_config("equalizer");
	// filter_get_config("filter_json");
	// equalizer_two_way_update();
	float* loudness_curve = calculate_loudness_curve(adjusted_gain);
	for (int i = 0; i < EQ_BANDS; i++) {
		equalizer.gain[i] = (float)(loudness_curve[i] + (float)pGains[i]);
	}
	equalizer_print_bands("Combined Loudness: ", equalizer.gain, EQ_BANDS);
	free(loudness_curve);
	free(pGains);
	equalizer.update = true;
}

/****************************************************************************************
 * process equalizer
 */
void equalizer_process(u8_t *buf, u32_t bytes, u32_t sample_rate) {
	// don't want to process with output locked, so take the small risk to miss one parametric update
	if (equalizer.update) {
		equalizer_close();
		equalizer_open(sample_rate);
	}

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

void apply_biquad(struct buffer *outputbuf, frames_t count) {
	if (biquad_lpf_process) {
		filter_biquad_i16(outputbuf, count, biquad_lpf_coeffs, biquad_lpf_w, 1); // Right (low) channel
	}
	if (biquad_hpf_process) {
		filter_biquad_i16(outputbuf, count, biquad_hpf_coeffs, biquad_hpf_w, 0); // Left (high) channel
	}
}

void apply_biquad_eq(struct buffer *outputbuf, frames_t count) {
	if (equalizer_active) {
		equlizer_biquad_i16(outputbuf, count, 0);
		equlizer_biquad_i16(outputbuf, count, 1);
	}
	if (n_filters > 0) {
		filters_biquad_i16(outputbuf, count, 0);
		filters_biquad_i16(outputbuf, count, 1);
	}
}


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

__attribute__((optimize("O2"))) void filter_all_biquad_i16(struct buffer *outputbuf, frames_t count, int channel)
{


}

__attribute__((optimize("O2"))) void filter_biquad_i16(struct buffer *outputbuf, frames_t count, float *coef, float *w, int channel) 
{
	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;
	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;

	while (count--) {
        biquad_d0 = (float)*ptr - coef[3] * w[0] - coef[4] * w[1];
        *ptr = (ISAMPLE_T)(coef[0] * biquad_d0 + coef[1] * w[0] + coef[2] * w[1]);
        w[1] = w[0];
        w[0] = biquad_d0;
		ptr += 2;
    }
}

__attribute__((optimize("O2"))) void equlizer_biquad_i16(struct buffer *outputbuf, frames_t count, int channel) 
{
	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;
	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
	float temp;
	while (count--) {
		temp = (float)*ptr;
		for (int i = 0; i < EQ_BANDS; i++)
		{
			// biquad_d0 = temp - biquad_eq_coefficients[channel][i][3] * biquad_eq_w[channel][i][0] - biquad_eq_coefficients[channel][i][4] * biquad_eq_w[channel][i][1];
			// temp = (biquad_eq_coefficients[channel][i][0] * biquad_d0 + biquad_eq_coefficients[channel][i][1] * biquad_eq_w[channel][i][0] + biquad_eq_coefficients[channel][i][2] * biquad_eq_w[channel][i][1]);
			biquad_d0 = temp - biquad_eq_coefficients[i][3] * biquad_eq_w[channel][i][0] - biquad_eq_coefficients[i][4] * biquad_eq_w[channel][i][1];
			temp = (biquad_eq_coefficients[i][0] * biquad_d0 + biquad_eq_coefficients[i][1] * biquad_eq_w[channel][i][0] + biquad_eq_coefficients[i][2] * biquad_eq_w[channel][i][1]);
			biquad_eq_w[channel][i][1] = biquad_eq_w[channel][i][0];
			biquad_eq_w[channel][i][0] = biquad_d0;
		}
		*ptr = (ISAMPLE_T)temp;
		ptr += 2;
    }
}

__attribute__((optimize("O2"))) void filters_biquad_i16(struct buffer *outputbuf, frames_t count, int channel) 
{
	ISAMPLE_T *ptr = (ISAMPLE_T *)(void *)outputbuf->readp + channel;
	// ISAMPLE_T *ptrR = (ISAMPLE_T *)(void *)outputbuf->readp + 1;
	float temp;
	while (count--) {
		temp = (float)*ptr;
		for (int i = 0; i < n_filters; i++)
		{
			// biquad_d0 = temp - biquad_eq_coefficients[channel][i][3] * biquad_eq_w[channel][i][0] - biquad_eq_coefficients[channel][i][4] * biquad_eq_w[channel][i][1];
			// temp = (biquad_eq_coefficients[channel][i][0] * biquad_d0 + biquad_eq_coefficients[channel][i][1] * biquad_eq_w[channel][i][0] + biquad_eq_coefficients[channel][i][2] * biquad_eq_w[channel][i][1]);
			biquad_d0 = temp - filters[i].biquad_coeffs[3] * filters[i].biquad_w[0] - filters[i].biquad_coeffs[4] * filters[i].biquad_w[1];
			temp = (filters[i].biquad_coeffs[0] * biquad_d0 + filters[i].biquad_coeffs[1] * filters[i].biquad_w[0] + filters[i].biquad_coeffs[2] * filters[i].biquad_w[1]);
			filters[i].biquad_w[1] = filters[i].biquad_w[0];
			filters[i].biquad_w[0] = biquad_d0;
		}
		*ptr = (ISAMPLE_T)temp;
		ptr += 2;
    }
}

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

    float b0 = 1 + alpha*A;
    float b1 = -2 * c;
    float b2 = 1 - alpha*A;
    float a0 = 1 + alpha/A;
    float a1 = b1;
    float a2 = 1 - alpha/A;

    coeffs[0] = b0 / a0;
    coeffs[1] = b1 / a0;
    coeffs[2] = b2 / a0;
    coeffs[3] = a1 / a0;
    coeffs[4] = a2 / a0;
}

#ifndef APP_INC_GRID_H_
#define APP_INC_GRID_H_

#include "main.h"

void grid_task(void);
void zero_cross_detect(zero_cross_detect_t *zc_detect, float voltage);
static float phase_matching_calculation(uint8_t phase_idx, uint8_t ct_idx, float total_current_rms);
float calculate_rms(float *buffer, uint16_t count);
float calculate_rms_ring(float *buffer, uint16_t total_size, uint16_t start_index, uint16_t count);
float calculate_active_power(float *voltage, float *current, uint16_t count);
void phase_identify_process(phase_identify_t *phase_id);
void phase_identify_init(phase_identify_t *phase_id);


extern float ua_voltage_buffer[TOTAL_SAMPLES];
extern float last_ua_voltage_buffer[TOTAL_SAMPLES];
extern float current1_buffer[TOTAL_SAMPLES];
extern float current2_buffer[TOTAL_SAMPLES];
extern float current3_buffer[TOTAL_SAMPLES];

#endif /* APP_INC_GRID_H_ */

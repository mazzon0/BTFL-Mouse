#include "tmx_processing.h"

static uint32_t s_raw_data[TMX_M][TMX_N]; //Raw touch data
static uint32_t s_filtered_data[TMX_M][TMX_N]; //first filetring using EMWA, fast variation
static uint32_t s_adaptive_baseline[TMX_M][TMX_N];//low variation for adaptive baseline
static uint32_t s_delta_signal[TMX_M][TMX_N]; //delta signal = filtered - baseline
static tmx_touch_t s_current_frame[MAX_NUM_TOUCHES]; //curfrent detected blobs

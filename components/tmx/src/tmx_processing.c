#include "tmx_processing.h"

static uint32_t s_raw_data[TMX_M][TMX_N]; //Raw touch data
static uint32_t s_filtered_data[TMX_M][TMX_N]; //first filetring using EMWA, fast variation
static uint32_t s_adaptive_baseline[TMX_M][TMX_N];//low variation for adaptive baseline
static uint32_t s_delta_signal[TMX_M][TMX_N]; //delta signal = filtered - baseline
static uint32_t s_oversampled_delta[OVERSAMPLED_M][OVERSAMPLED_N]; //oversampled delta signal for blob detection
static bool visited[OVERSAMPLED_M][OVERSAMPLED_N];
static tmx_touch_t s_current_frame[MAX_NUM_TOUCHES]; //curfrent detected blobs
static CircularBuffer_t s_frame_history;

void tmx_processing_raw_read(void)
{
    //Read raw data from TMX driver
    tmx_driver_read_raw((uint32_t*)s_raw_data);
}

esp_err_t tmx_processing_init(void)
{
    //Initialize circular buffer for frame history
    static tmx_touch_t frame_history_buffer[MAX_NUM_FRAMES * MAX_NUM_TOUCHES];
    cb_init(&s_frame_history, frame_history_buffer, sizeof(tmx_touch_t), MAX_NUM_FRAMES * MAX_NUM_TOUCHES);
    tmx_driver_init();
    vTaskDelay(100 / portTICK_PERIOD_MS); //Wait for touchpad to stabilize
    tmx_processing_raw_read();
    memcpy(s_filtered_data, s_raw_data, sizeof(s_raw_data));
    memcpy(s_adaptive_baseline, s_raw_data, sizeof(s_raw_data));
    memset(s_delta_signal, 0, sizeof(s_raw_data));
    return ESP_OK;
}

void tmx_processing_filtering(void){
    for(int i = 0; i < TMX_M; i++){
        for(int j = 0; j < TMX_N; j++){
            //First stage: Fast EMWA filtering
            const float alpha_fast = 0.5f;
            s_filtered_data[i][j] = (uint32_t)(alpha_fast * s_raw_data[i][j] + (1 - alpha_fast) * s_filtered_data[i][j]);

            //Second stage: Adaptive baseline with slower EMWA
            const float alpha_slow = 0.00f;
            s_adaptive_baseline[i][j] = (uint32_t)(alpha_slow * s_filtered_data[i][j] + (1 - alpha_slow) * s_adaptive_baseline[i][j]);

            //Compute delta signal
            if(s_filtered_data[i][j] > s_adaptive_baseline[i][j]){
                s_delta_signal[i][j] = s_filtered_data[i][j] - s_adaptive_baseline[i][j];
            } else {
                s_delta_signal[i][j] = 0;
            }
        }
    }
}

void tmx_processing_oversampling(void){
    for(int vr = 0; vr < OVERSAMPLED_M; vr++){
        for(int vc = 0; vc < OVERSAMPLED_N; vc++){


            float fr = (float)vr / (float)OVERSAMPLING_FACTOR; 
            float fc = (float)vc / (float)OVERSAMPLING_FACTOR; 

            int r0 = (int)fr; 
            int c0 = (int)fc; 
            
           
            float y = fr - (float)r0; 
            float x = fc - (float)c0; 

           
            int r1 = (r0 + 1 < TMX_M) ? r0 + 1 : r0; 
            int c1 = (c0 + 1 < TMX_N) ? c0 + 1 : c0;
            
            uint32_t P00 = s_delta_signal[r0][c0]; 
            uint32_t P10 = s_delta_signal[r0][c1]; 
            uint32_t P01 = s_delta_signal[r1][c0]; 
            uint32_t P11 = s_delta_signal[r1][c1]; 

            // Bilinear Interpolation: V = P00*(1-x)(1-y) + P10*x(1-y) + P01*(1-x)y + P11*xy
            float v_interp = 
                (P00 * (1.0f - x) * (1.0f - y)) + 
                (P10 * x * (1.0f - y)) +          
                (P01 * (1.0f - x) * y) +          
                (P11 * x * y);                    

            
            s_oversampled_delta[vr][vc] = (uint32_t)v_interp;
        }
    }
}

void tmx_processing_print(void){
    while(1){
        tmx_processing_raw_read();
        tmx_processing_filtering();
        tmx_processing_oversampling();

        for(int i = 0; i < OVERSAMPLED_M; i++){
            for(int j = 0; j < OVERSAMPLED_N; j++){
                printf("%" PRIu32 ",", s_oversampled_delta[i][j]);
            }
        }
        printf("\n");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void tmx_processing_blob_detection(void){
    memset(visited, 0, sizeof(visited));
    memset(s_current_frame, 0, sizeof(s_current_frame));
    int touch_count = 0;
    for(int i = 0; i < OVERSAMPLED_M; i++){
        for(int j = 0; j < OVERSAMPLED_N; j++){
            if(s_oversampled_delta[i][j] > TMX_DELTA_THRESHOLD && !visited[i][j]){
                tmx_processing_flood_fill(i, j, &visited, &s_current_frame[touch_count]);
                touch_count++;
                if(touch_count >= MAX_NUM_TOUCHES){
                    return;
                }
            }
        }
    }
}

void tmx_processing_flood_fill(int i; int j; bool visited[OVERSAMPLED_M][OVERSAMPLED_N]; tmx_touch_t* touch){
    //TO BE IMPLEMENTED
}

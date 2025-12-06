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
    
    // exec pipeline
    tmx_processing_raw_read();
    tmx_processing_filtering();
    tmx_processing_oversampling();
    tmx_processing_blob_detection();
    finger_rejection_filtering();

    // print results in CSV format
    for(int k = 0; k < MAX_NUM_TOUCHES; k++){
        if(s_current_frame[k].is_active){
            tmx_touch_t *touch = &s_current_frame[k];
            
            // Stampa CSV: Index,ID,X,Y,Area
            printf("%d,%d,%.2f,%.2f,%" PRIu32 "\n",
                k,                         // Slot Index (0 o 1)
                touch->ID,        // ID di tracciamento
                touch->centroid_x,         // Centroide X (virtuale)
                touch->centroid_y,         // Centroide Y (virtuale)
                touch->area                // Area (Intensità)
            );
        }
    }
    
    // Frame marker
    printf("FRAME_END\n");
}

void tmx_processing_blob_detection(void){
    memset(visited, 0, sizeof(visited));
    memset(s_current_frame, 0, sizeof(s_current_frame));
    int touch_count = 0;

    for(int i = 0; i < OVERSAMPLED_M; i++){
        for(int j = 0; j < OVERSAMPLED_N; j++){

            if(s_oversampled_delta[i][j] > TMX_DELTA_THRESHOLD && !visited[i][j] && touch_count < MAX_NUM_TOUCHES){

                tmx_processing_flood_fill(i, j, &s_current_frame[touch_count]);
                s_current_frame[touch_count].is_active = true;
                touch_count++;

            }
        }
    }

    for(int t = 0; t < touch_count; t++){
        if(s_current_frame[t].is_active){
            //normalize centroid
            s_current_frame[t].centroid_x /= (float)s_current_frame[t].area;
            s_current_frame[t].centroid_y /= (float)s_current_frame[t].area;
        }
    }
}

void tmx_processing_flood_fill(int r, int c, tmx_touch_t* touch){
    if(r < 0 || r >= OVERSAMPLED_M || c < 0 || c >= OVERSAMPLED_N){
        return;
    }

    if(visited[r][c]){
        return;
    }

    if(s_oversampled_delta[r][c] < TMX_DELTA_THRESHOLD){
        return;
    }

    visited[r][c] = true;
    touch->area += s_oversampled_delta[r][c];
    touch->centroid_x += (float)c * (float)s_oversampled_delta[r][c];
    touch->centroid_y += (float)r * (float)s_oversampled_delta[r][c];

    //kernel 8-neighbors
    static const int dr[] = {-1,-1,-1,0,0,1,1,1};
    static const int dc[] = {-1,0,1,-1,1,-1,0,1};

    for(int k=0; k<8; k++){
        tmx_processing_flood_fill(r + dr[k], c + dc[k], touch);
    }
}

void finger_rejection_filtering(void){
    for(int t = 0; t < MAX_NUM_TOUCHES; t++){
        if(s_current_frame[t].is_active){
            if(s_current_frame[t].area > REJECTION_AREA_THRESHOLD){
                s_current_frame[t].is_active = false;
            }
        }
    }
}

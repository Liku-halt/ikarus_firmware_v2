#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/i2c_master.h"
#include "esp_mac.h"
#include "math.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gptimer.h"
// custom lib
#include "atan_lut.h"

// i2c
#define i2c_num I2C_NUM_0
#define i2c_sda 6
#define i2c_scl 7
#define i2c_freq 1000000
#define mpu_addr 0x68

// mpu_reg
#define power_reg 0x6B
#define rate_reg 0x19
#define filter_reg 0x1A
#define gyro_range_reg 0x1B
#define aclo_renge_reg 0x1C

// mpu_reg_val
#define power_val 0x00
#define rate_val 0x00   // 1khz
#define filter_val 0x02 // 98hz
#define gyro_range_val 0x10
#define aclo_range_val 0x08

// mpu_data_handleing
uint8_t handle_data[14];
uint8_t acc_data = 0x3b;

// uart
#define UART_PORT_NUM UART_NUM_0
#define UART_TX_PIN 1
#define UART_RX_PIN 3
#define UART_BAUD_RATE 115200
static const char *TAG = "APP_MAIN";

// other declarations  for mpu
i2c_master_dev_handle_t mpu_handle;
i2c_master_bus_handle_t bus;
#define time_out 2 //i have changed it 200ms time out to 2ms time out ...so that the calculations wont stop
#define time_out_ticks pdMS_TO_TICKS(time_out)

// motors
#define mot1 4
#define mot2 5
#define mot3 8
#define mot4 10

// assigning motor channels
#define mot1_cha LEDC_CHANNEL_0
#define mot2_cha LEDC_CHANNEL_1
#define mot3_cha LEDC_CHANNEL_2
#define mot4_cha LEDC_CHANNEL_3

// timer and ledc cofig constants
#define timer_speed LEDC_LOW_SPEED_MODE
#define LEDC_timer LEDC_TIMER_0
#define pwm_freq 32000
#define pwm_resol LEDC_TIMER_10_BIT  //i changed it from 8 bit to 10 bit 
#define LEDC_speed_mode LEDC_LOW_SPEED_MODE
#define pwm_max_duty 1022

//time declaratons for task
#define time_tick pdMS_TO_TICKS(1)

// floating point macros if needed
#define q_bit 16 // Q16.16 format
#define q_bit_shift (1 << q_bit)
#define div_64 6
#define q_90 90 * q_bit_shift
#define q_180 180 * q_bit_shift
#define q_360 360 * q_bit_shift
#define one_khz (int32_t)llroundf(0.001f*(float)q_bit_shift)
#define low_pas_95 (int32_t)llroundf(0.98f*(float)q_bit_shift)
#define low_pas_05 (int32_t)llroundf(0.02f*(float)q_bit_shift)

//lowpass filters variables
const int32_t a_term[6]={5644,5644,5644,25296,25296,25296};
int32_t filtered[6]={0};//i should remember that const makes it read only variable
int32_t final_angles[3];

//bias estimatation or gyro rate bias estimation by intigral term 
int32_t bias[3]={0,0,0};
int32_t error[3]={0};


// adc constants
adc_bits_width_t anal_res = 12;
adc1_channel_t adc_gpio = 0;
adc_atten_t atten_val = 3;

// pid_var
typedef struct
{
    int32_t error[3]; // 0 for roll 1 pitch 2 for yaw
    int32_t actual_angl[6];//acclero are 0 1 2 and gyro are 3 4 5 
    int32_t intigrator[3];
    int32_t prev_error[3];
    int32_t pid[3];
    int32_t mot_th[4];
    int32_t final_tn1[3];
} calc;

calc c; // for acro mode
calc w; // for angle mode

// new construct for the esp now protocall
typedef struct
{
    int16_t command[4]; // the first three are roll pitch and yaw the last one is the thrust
    float p;
    float i;
    float d;
    char arm;
} data_construct;
data_construct data; // this is the now variables

// the imu variables
int16_t raw_val[6];
int16_t offsets[6];
int32_t q16val[6];
int32_t q16_const[6] = {8, 8, 8, 1998, 1998, 1998};

int i = 0;
float itteration_time = 0.001;

// voltage variable and battry var
int16_t bat_vol;

// pid gains
float kp = 0;
float ki = 0;
float kd = 0;
float y_ki, y_kp, y_kd; // this one is only for yaw

// time variables
unsigned long time_stamp[2];
const float dt = 0.001; // loop time
unsigned long esp_now_time[2];
int64_t ti=0;
int64_t tn=0;
int64_t diff= 0;



// arm dis arm logic variables
const char arm_ = 'a';
const char dis_arm = 'd';

// communication flags
volatile bool new_data_flag; // if new data has arrived or not this is volatile

// testing variables
int32_t index_ = 0;

//flight variables 
int32_t hover_ = 0;
#define max_hov 120
int32_t emg_thrust = max_hov;

void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB};
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0);
}

// making a fxn to stup the clock for pwm
void clock_config(void)
{
    // creating a native struct for the timer
    ledc_timer_config_t clock_par = {
        .speed_mode = timer_speed,
        .timer_num = LEDC_timer,
        .duty_resolution = pwm_resol,
        .freq_hz = pwm_freq,
        .clk_cfg = LEDC_AUTO_CLK};
    // parrsed the construct into the native fxn and i am storing the error if it happens
    ledc_timer_config(&clock_par);
}

// making a fxn to initialise the channels with their gpio
void channel_config(void)
{
    // making structs for every channels
    ledc_channel_config_t mot1_configs = {
        .gpio_num = mot1,
        .speed_mode = timer_speed,
        .channel = mot1_cha,
        .timer_sel = LEDC_timer,
        .duty = 0,
        .hpoint = 0};
    // parsing this to the native api
    ledc_channel_config(&mot1_configs);

    // for mot 2
    ledc_channel_config_t mot2_configs = {
        .gpio_num = mot2,
        .speed_mode = timer_speed,
        .channel = mot2_cha,
        .timer_sel = LEDC_timer,
        .duty = 0,
        .hpoint = 0};
    // parsing this to the native api
    ledc_channel_config(&mot2_configs);

    // for mot 3
    ledc_channel_config_t mot3_configs = {
        .gpio_num = mot3,
        .speed_mode = timer_speed,
        .channel = mot3_cha,
        .timer_sel = LEDC_timer,
        .duty = 0,
        .hpoint = 0};
    // parsing this to the native api
    ledc_channel_config(&mot3_configs);

    // for mot 4
    ledc_channel_config_t mot4_configs = {
        .gpio_num = mot4,
        .speed_mode = timer_speed,
        .channel = mot4_cha,
        .timer_sel = LEDC_timer,
        .duty = 0,
        .hpoint = 0};
    // parsing this to the native api
    ledc_channel_config(&mot4_configs);
}

// making an fxn to update the pwm
static IRAM_ATTR inline void pwm_update(void)
{
    ledc_set_duty(timer_speed, mot1_cha, w.mot_th[0]);
    ledc_set_duty(timer_speed, mot2_cha, w.mot_th[1]);
    ledc_set_duty(timer_speed, mot3_cha, w.mot_th[2]);
    ledc_set_duty(timer_speed, mot4_cha, w.mot_th[3]);

    ledc_update_duty(timer_speed, mot1_cha);
    ledc_update_duty(timer_speed, mot2_cha);
    ledc_update_duty(timer_speed, mot3_cha);
    ledc_update_duty(timer_speed, mot4_cha);
}

// call back fxn
// the most important fxn here it handles the esp now callback things
void IRAM_ATTR on_callback(const esp_now_recv_info_t *recv_info, const uint8_t *data_in, int len)
{
    // const uint8_t *mac_addr = recv_info->src_addr;
    memcpy(&data, data_in, sizeof(data));
    // printf("ESP-NOW packet received! len=%d\n", len);
    // esp_now_send(broadcastAddress,(uint8_t *)&send_data, sizeof(send_data));
    new_data_flag = true;
    // Serial.println(data.message);
    // TickType_t clock= xTaskGetTickCountFromISR();
}

// now making the esp-now initialisation fxn
void esp_now_init_fxn(void)
{
    wifi_init_config_t dflt = WIFI_INIT_CONFIG_DEFAULT();
    // all the per initialisations
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_wifi_init(&dflt);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();                               // always remember u first start the wifi then set the channel
    esp_wifi_set_channel(4, WIFI_SECOND_CHAN_NONE); // god this took a whole day to figure out
    esp_now_init();
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_callback));

    esp_now_peer_info_t peer_info = {0};
    uint8_t mac[6] = {0xFC, 0xF5, 0xC4, 0x01, 0x55, 0x7C};
    memcpy(peer_info.peer_addr, mac, 6);
    peer_info.channel = 4;
    peer_info.encrypt = false;
    // adding the peer
    /* if (esp_now_add_peer(&peer_info) != ESP_OK) {
     return;
   }*/
}

static void i2c_init(void)
{
    i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_num,
        .scl_io_num = i2c_scl,
        .sda_io_num = i2c_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true};

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus));

    i2c_device_config_t i2c_device = {
        .device_address = mpu_addr,
        .scl_speed_hz = i2c_freq,
        .scl_wait_us = 0};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &i2c_device, &mpu_handle));
}

// register write fxn
void register_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_transmit(mpu_handle, data, 2, 10)); // tthe last parmeter is timeout im ms
}

void mpu_6050_init(void)
{
    register_write(power_reg, power_val);
    vTaskDelay(pdMS_TO_TICKS(1));
    register_write(rate_reg, rate_val);
    vTaskDelay(pdMS_TO_TICKS(1));
    register_write(filter_reg, filter_val);
    vTaskDelay(pdMS_TO_TICKS(1));
    register_write(gyro_range_reg, gyro_range_val);
    vTaskDelay(pdMS_TO_TICKS(1));
    register_write(aclo_renge_reg, aclo_range_val);
}
static IRAM_ATTR inline void get_raw_mpu(void)
{
    i2c_master_transmit_receive(
        mpu_handle,
        &acc_data, 1,
        handle_data, 14,
        time_out_ticks);

    // now the bit comb part
    raw_val[0] = handle_data[0] << 8 | handle_data[1];
    raw_val[1] = handle_data[2] << 8 | handle_data[3];
    raw_val[2] = handle_data[4] << 8 | handle_data[5];
    raw_val[3] = handle_data[8] << 8 | handle_data[9];
    raw_val[4] = handle_data[10] << 8 | handle_data[11];
    raw_val[5] = handle_data[12] << 8 | handle_data[13];

    // multiplying the raw val with the constants
    for (int i = 0; i < 6; i++)
    {
        raw_val[i] = (raw_val[i]-offsets[i] );
        q16val[i] = ((int32_t)raw_val[i])*q16_const[i];
    }
    
} 


// gyro_offset calculate
void offset_cal(int total_sample)
{
    
    int64_t sum[6] = {0,0,0,0,0,0};
    int i = 0;
    while (i < total_sample)
    {
        

        i2c_master_transmit_receive(
        mpu_handle,
        &acc_data, 1,
        handle_data, 14,
        time_out_ticks);

        // now the bit comb part
         raw_val[0] = handle_data[0] << 8 | handle_data[1];
         raw_val[1] = handle_data[2] << 8 | handle_data[3];
         raw_val[2] = handle_data[4] << 8 | handle_data[5];
         raw_val[3] = handle_data[8] << 8 | handle_data[9];
         raw_val[4] = handle_data[10] << 8 | handle_data[11];
         raw_val[5] = handle_data[12] << 8 | handle_data[13];

        for (int j = 0; j < 6; j++)
        {
            sum[j] += raw_val[j];
        }
        i++;
    }
    int32_t avgx = (int32_t)(sum[0] / total_sample);
    int32_t avgy = (int32_t)(sum[1] / total_sample); 
    int32_t avgz = (int32_t)(sum[2] / total_sample); 
    offsets[0] = avgx;
    offsets[1] = avgy;
    offsets[2] = avgz - 8192;
    offsets[3] = (int32_t)(sum[3] / total_sample);
    offsets[4] = (int32_t)(sum[4] / total_sample);
    offsets[5] = (int32_t)(sum[5] / total_sample);
}

// adc_fxns
void adc_init(void)
{
    adc1_config_width(anal_res);
    adc1_config_channel_atten(adc_gpio, atten_val);
}

// the fixed point math fxns

// the float to fixed point converter fxn
static inline int32_t float_to_fix(float x)
{
    return (int32_t)llroundf(x * (float)q_bit_shift);

}

// the int32 to fixed point conversion
static inline int32_t int_to_fixed(int32_t x)
{
    return x << q_bit;
}

// the fixed point number multiplication
static inline int32_t q_mul(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * (int64_t)b) >> q_bit);
}

// the fixed point devision
static inline int32_t q_div(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a << q_bit) / b);
}

//replacing the fixed point math functions with more robust static inlin macros
#define q_clamp(x,min,max)  ((x)<(min)?(min):((x)>(max)?(max):(x)))


//custom abs function
static inline int32_t abs_32(int32_t x){
    return (int32_t)(x<0 ? -(int64_t)x : x);
}

static IRAM_ATTR inline int32_t angle_lut(int32_t y, int32_t x)
{
    
    if (x == 0 && y == 0) return 0;  //if both are zero then it must output zero

    if(x==0 && y!=0)  return y>0 ? q_90 : -q_90;   //if x is xero and y is not then it must be 90 degrees or -90 degrees

    int32_t x_abs = abs_32(x), y_abs = abs_32(y), val;
    bool flag = x_abs < y_abs ? 1 : 0 ;
    uint32_t nume_ = flag? x_abs : y_abs , deno_ = flag? y_abs : x_abs;

    //index calculation part
    uint32_t div = q_div(nume_,deno_);
    uint64_t scale = ((uint64_t)div)*1023u;
    uint32_t indx = (uint32_t)(scale>>16);

    //now finding the result form the array
    val= atan_table[indx];
    
    //if ratio grater than one 
    if(flag) val= q_90- val;
    
    // quadrant correction

      //no changes in first quad
      //2nd quad
    if(x<0&&y>=0) val = q_180 - val;
      //3rd quad
    if(x<0&&y<=0) val = -q_180 + val;
      //4th quad
    if(x>0&&y<=0) val = - val;

    if (val >= q_180) val -= q_360;
    if (val <-q_180) val += q_360;

    return val;


}

//lowpass filter for jitter control
static inline void lowpass_fil(void){
    for(int i = 0; i < 6;i++ ) filtered[i] = filtered[i]+q_mul( a_term[i],(q16val[i]-filtered[i]));
    for(int i = 0; i < 6; i++) q16val[i] = filtered[i];
}

//bias calculation 
void static inline bias_estim(){
    const int32_t bias_ki = float_to_fix(0.00005);
    const int32_t error_ki= float_to_fix(0.02);
    for(int i = 0 ; i < 2 ; i++){
        q16val[i+3]=q16val[i+3]- bias[i];
    }
    for(int i = 0 ; i < 2 ; i++){
        w.actual_angl[i+3]+= q_mul(q16val[i+3], one_khz);
    
    }
    //acclero angles and and the 
    w.actual_angl[0]= angle_lut(q16val[1],q16val[2]);
    w.actual_angl[1]= angle_lut(-q16val[0],q16val[2]);

    for(int i = 0 ; i < 2 ; i++){
        error[i]= w.actual_angl[i]-w.actual_angl[i+3];
        w.actual_angl[i+3]+= q_mul(error[i],error_ki);
        bias[i]+=q_mul(error[i],bias_ki);
        final_angles[i]= w.actual_angl[i];
        
    }
    final_angles[2]= q16val[5];


 
    
}


//error calculation for pid
void static inline error_calculation()
{
    
    for (int i = 0; i < 3; i++) w.error[i] = int_to_fixed((int32_t)data.command[i]) -final_angles[i];
    
}


static inline void pid_calc(void)
{
    const int32_t Kp = float_to_fix(data.p);
    const int32_t Ki = float_to_fix(data.i);
    const int32_t Kd = float_to_fix(data.d);
    const int32_t YawKp = float_to_fix(y_kp);
    const int32_t YawKi = float_to_fix(y_ki);
    const int32_t YawKd = float_to_fix(y_kd);
    const int32_t INT_LIM = float_to_fix(200.0f);
    

    for (int i = 0; i < 3; i++)
    {
        const int32_t Kp_use = (i == 2) ? YawKp : Kp;
        const int32_t Ki_use = (i == 2) ? YawKi : Ki;
        const int32_t Kd_use = (i == 2) ? YawKd : Kd;

        int32_t err = w.error[i];
        int32_t diff = final_angles[i]-w.final_tn1[i]; //this term is differnt it calculate the error btw t and t-1 of sensor readings
        int32_t err_dt = q_mul(err,one_khz);
        
        w.intigrator[i] = q_clamp(w.intigrator[i] + q_mul(err_dt,Ki_use), -INT_LIM, INT_LIM);
        w.pid[i] = q_mul(err, Kp_use) + w.intigrator[i] - q_mul(q_div(diff,one_khz), Kd_use);
        w.prev_error[i] = err;
        w.final_tn1[i] = final_angles[i];

    }
}

//creating a irs timer for perfect 1ms loop
gptimer_handle_t control_timer = NULL;
volatile bool control_tick = false;
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    control_tick = true;
    return pdFALSE;       
}

//this part is from the official documentation of esp32 isr
void setup_1ms_timer(void) {
    ESP_LOGI(TAG, "Initializing 1ms hardware timer...");
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,   
        .direction = GPTIMER_COUNT_UP,        
        .resolution_hz = 1000000,            
    };
    
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &control_timer));
    ESP_LOGI(TAG, "Timer created");
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(control_timer, &cbs, NULL));
    ESP_LOGI(TAG, "ISR callback registered");
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,                   
        .reload_count = 0,                     
        .flags.auto_reload_on_alarm = true,    
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(control_timer, &alarm_config));
    ESP_LOGI(TAG, "Alarm configured for 1ms");
    
    ESP_ERROR_CHECK(gptimer_enable(control_timer));
    ESP_LOGI(TAG, "Timer enabled");
    
    ESP_ERROR_CHECK(gptimer_start(control_timer));
    ESP_LOGI(TAG, "Timer started - running at EXACTLY 1ms intervals");
}

//creating tasks
void IRAM_ATTR control_task(void *pvParameters){
   // TickType_t last_wake = xTaskGetTickCount();
   while (1)
   {
   
   
    if (control_tick) 
    {
        control_tick = false;
        tn= esp_timer_get_time();
         diff= tn-ti;
         ti= tn;
        get_raw_mpu();
        //lowpass_fil();
        bias_estim();
        error_calculation();
        pid_calc();
        const int32_t base_throttle = int_to_fixed(data.command[3]);

        w.mot_th[0] = hover_+((base_throttle - w.pid[0] - w.pid[1] + w.pid[2]) >> q_bit);
        w.mot_th[1] = hover_+((base_throttle - w.pid[0] + w.pid[1] - w.pid[2]) >> q_bit);
        w.mot_th[2] = hover_+((base_throttle + w.pid[0] + w.pid[1] + w.pid[2]) >> q_bit);
        w.mot_th[3] = hover_+((base_throttle + w.pid[0] - w.pid[1] - w.pid[2]) >> q_bit);

        for (int i = 0; i < 4; i++) w.mot_th[i] = q_clamp(w.mot_th[i], 0, pwm_max_duty);


        if (new_data_flag)
        {
            esp_now_time[1]= esp_timer_get_time();
            ESP_LOGI(TAG, "Received data: angle=%f, loop_time=%lld, raw=%d", ((float)final_angles[2])*(1.0f/65536.0f), diff, (int)data.command[0]);
            new_data_flag = false;
        }
       

      //this was the pervious code from the arduino
      switch(data.arm){
        case arm_:
             esp_now_time[0]=  esp_timer_get_time();
             if(esp_now_time[0]-esp_now_time[1]<500000){

                 if(hover_<max_hov){
                      hover_=hover_+1;
                    }

                 emg_thrust=hover_;

                }else{
                  emg_thrust= emg_thrust-1;
                  emg_thrust= q_clamp(emg_thrust,0,150);
                  for ( int i = 0; i<4;i++){
                      w.mot_th[i]= emg_thrust;
                    }
                  
               }
            pwm_update();
            break;
        case dis_arm:

             hover_=0;
              for ( int i = 0; i<4;i++){
              w.mot_th[i]= 0;
             }
             pwm_update();

             break;
        }
         //vTaskDelay(pdMS_TO_TICKS(100));
        
        
       
        
    }
  }
    
}

// now the main
void app_main(void)
{
    // initialisations
    clock_config();
    channel_config();
    ledc_set_duty(timer_speed, mot2_cha, 0);
    ledc_update_duty(timer_speed, mot2_cha);
    i2c_init();
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_now_init_fxn();
    mpu_6050_init();
    offset_cal(500);
    adc_init();
    setup_1ms_timer();
    // printf("%ld",offsets[2]);

    y_kp=0;
    y_ki=0;
    y_kd=0; 
    
    //for loop time
    //TickType_t last_wake = xTaskGetTickCount();
    xTaskCreate(control_task, "control tasks", 8192, NULL, 7, NULL);
  


 
}

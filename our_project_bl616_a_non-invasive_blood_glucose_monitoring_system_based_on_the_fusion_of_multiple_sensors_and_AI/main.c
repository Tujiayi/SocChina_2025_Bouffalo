/* ==============================header file=================================== */
#include "FreeRTOS.h"
#include "shell.h"
#include "task.h"
#include "board.h"

#include "bluetooth.h"
#include "conn.h"
#include "conn_internal.h"
#if defined(BL702) || defined(BL602)
#include "ble_lib_api.h"
#elif defined(BL616)
#include "btble_lib_api.h"
#include "bl616_glb.h"
#include "rfparam_adapter.h"
#elif defined(BL808)
#include "btble_lib_api.h"
#include "bl808_glb.h"
#endif
#include "ble_tp_svc.h"
#include "hci_driver.h"
#include "hci_core.h"
#include "bflb_mtd.h"
#include "easyflash.h"
#if defined(CONFIG_BT_OAD_SERVER)
#include "oad_main.h"
#include "oad_service.h"
#endif
#define DBG_TAG "MAIN"
#include "log.h"
#include "bl_fw_api.h"
#include "wifi_mgmr_ext.h"
#include "wifi_mgmr.h"
#include "bflb_irq.h"
#include "bflb_uart.h"
#include "mem.h"
#include <lwip/tcpip.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "bflb_wdg.h"
#include "ring_buffer.h"
#include "bflb_rtc.h"

#include "timers.h"
#include "task.h"
#include "bflb_mtimer.h"
#include "board.h"
#include "string.h" 
#include "lcd.h"
#include "lcd_conf_user.h"
#include "touch_conf_user.h"
#include "touch.h"	
#include "max30102.h"
#include "si7021.h"
#include "gy906.h"
#include "math.h"
#include "fhm_onechannel_16k_20.h"
#include "bsp_es8388.h"
#include "bflb_i2s.h"
#include "bflb_i2c.h"
#include "bflb_dma.h"
#include "bflb_gpio.h"
#include "minimp3.h"

#include "mnist_valid_q.h"
#include "tinymaix.h"

#include "bflb_adc.h"
/* ============================================================================ */
/* ========================LCD Touch Config==================================== */
#define SENSOR_AREA_X1 0
#define SENSOR_AREA_Y1 0
#define SENSOR_AREA_X2 240
#define SENSOR_AREA_Y2 100
touch_coord_t touch_max_point = {
        .coord_x = 240,
        .coord_y = 320,
    };
/* ============================================================================ */
//Define task handle
TaskHandle_t max30102_task_handle;
TaskHandle_t si7021_task_handle;
TaskHandle_t gy906_task_handle;
TaskHandle_t lcd_task_handle;
//Define a device handle to control two Si7021 sensors: 
//Si7021_1 measures the temperature and humidity of the finger, while Si7021_2 measures the external temperature and humidity
SI7021_Handle si7021_1 ={
	.sda_pin = GPIO_PIN_25,
	.scl_pin = GPIO_PIN_23,
};
SI7021_Handle si7021_2 ={
	.sda_pin = GPIO_PIN_24,
	.scl_pin = GPIO_PIN_16,
};
// Global sensor data structure
typedef struct{
	int32_t heart_rate;
	int32_t spo2;
	float temperature_wrist;               						//temp_wrist is the temperature of the finger.
	float humidity_wrist;
	float temperature_ext;                
	float humidity_ext;                  						//temp_ext is the temperature of the environment.
	float ir_temperature;              							//Infrared temperature
	float blood_flow;                                           //blood flow velocity
	float blood_sugar;                                          //blood glucose level
}SensorData;
volatile SensorData sensordata;
/* ============================================================================ */
__attribute__((section(".psram_data"))) uint32_t *aun_ir_buffer  = NULL; 	        						//IR LED   Infrared light data
__attribute__((section(".psram_data"))) uint32_t *aun_red_buffer  = NULL;           						//Red LED	Red Light Data
/* ============================================================================ */
//Task max30102 - Measuring heart rate, blood oxygen level and blood flow velocity
void vMax30102_task(void *pvParameters) {
	static struct bflb_device_s *gpio;
	#define MAX_BRIGHTNESS 255
	#define INTERRUPT_REG 0X00
	int32_t n_ir_buffer_length; //data length
	int32_t n_sp02; //SPO2
	int8_t ch_spo2_valid;   	//Used to indicate the validity of the sp02 value, 1 indicates valid, and 0 indicates invalid.
	int32_t n_heart_rate;   	//Heart rate value
	int8_t  ch_hr_valid;   		//Used to indicate the validity of the heart rate value, 1 represents validity and 0 represents invalidity.
	uint8_t Temp;
	uint32_t un_min, un_max, un_prev_data;  
	int i;
	int32_t n_brightness;
	float f_temp;
	uint8_t temp[6];
	uint8_t str[100];
	uint8_t dis_hr=0,dis_spo2=0;
	gpio = bflb_device_get_by_name("gpio");
	MAX30102_Init();
	un_min=0x3FFFF;
	un_max=0;
	n_ir_buffer_length=500;            	//The buffer length is 500, capable of storing 5-second sample values.
	/* Malloc */
    aun_ir_buffer = malloc(500 * sizeof(uint32_t));
    if (aun_ir_buffer == NULL) {
        free(aun_ir_buffer);
        printf("\r aun_ir_buffer do not have space\n");
    }
    memset(aun_ir_buffer, 0, 500 * sizeof(uint32_t));
    aun_red_buffer = malloc(500 * sizeof(uint32_t));
    if (aun_red_buffer == NULL) {
        free(aun_red_buffer);
        printf("\r aun_red_buffer do not have space\n");
    }
    memset(aun_red_buffer, 0, 500 * sizeof(uint32_t));
    //Read the first 500 sample values
    taskENTER_CRITICAL();
	for (i=0;i<n_ir_buffer_length;i++) {
        while(bflb_gpio_read ( gpio, MAX30102_INT_PIN ));   //Wait until the MAX30102_INT_PIN pin goes low, indicating that new data is available for reading.
        max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);
        aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];      // ?1?7?1?7?0?5?1?7?0?0?1?7?1?7?0?1?1?7?0?6?1?7?1?7?1?7?1?7?0?5
        aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   	 // ?1?7?1?7?0?5?1?7?0?0?1?7?1?7?0?1?1?7?0?6?1?7?1?7?1?7?1?7?0?5     
        if(un_min>aun_red_buffer[i]) un_min=aun_red_buffer[i];   				    //Update to calculate the minimum value
        if(un_max<aun_red_buffer[i]) un_max=aun_red_buffer[i];    					//Update to calculate the maximum value
        // printf("red=%i,", aun_red_buffer[i]);
        // printf("ir=%i\r\n", aun_ir_buffer[i]);
    }
    taskEXIT_CRITICAL(); 
	un_prev_data=aun_red_buffer[i];
	//Calculate the heart rate and blood oxygen saturation of 500 sample values
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	float BF = maxim_blood_flow_rate(aun_ir_buffer,  n_ir_buffer_length);
	while(1) {
		//Remove the first 100 sample values and move the last 400 sets of sample values to the first 400 positions in the buffer area.
		for(i=100;i<500;i++) {
			aun_red_buffer[i-100]=aun_red_buffer[i];			//Move the first 100 to 500 sample values to the first 400 positions in the buffer area.
			aun_ir_buffer[i-100]=aun_ir_buffer[i];				//Move the first 100 to 500 sample values to the first 400 positions in the buffer area.		
			//update the signal min and max
			if(un_min>aun_red_buffer[i])						//earch for the minimum value of the buffer zone after the shift, within the range of 0 to 400.
			un_min=aun_red_buffer[i];
			if(un_max<aun_red_buffer[i])						//Find the maximum value of the buffer area after the shift, within the range of 0 to 400.
			un_max=aun_red_buffer[i];
		}		
		//Read the new sample values from the last 100 positions of the buffer and place them at positions 400 to 500.
		for(i=400;i<500;i++) {
			un_prev_data=aun_red_buffer[i-1];					//Before calculating the heart rate, 100 sets of samples were taken, and the obtained values were placed in the positions of the 400-500 cache array.
			while(bflb_gpio_read ( gpio, MAX30102_INT_PIN ));
			max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);		//Read the sensor values and assign them to the temp array
			aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    //Combine the values to obtain the actual number. The range 400-500 represents the newly read data.
			aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];     //Combine the values to obtain the actual number. The range 400-500 represents the newly read data.
			if(aun_red_buffer[i]>un_prev_data) {
                //Compare the newly obtained set of values with the previous set of values
				f_temp=aun_red_buffer[i]-un_prev_data;
				f_temp/=(un_max-un_min);
				f_temp*=MAX_BRIGHTNESS;							
				n_brightness-=(int)f_temp;
				if(n_brightness<0) n_brightness = 0;
			} else {
				f_temp=un_prev_data-aun_red_buffer[i];
				f_temp/=(un_max-un_min);
				f_temp*=MAX_BRIGHTNESS;			    			
				n_brightness+=(int)f_temp;
				if(n_brightness>MAX_BRIGHTNESS) n_brightness = MAX_BRIGHTNESS;
			}
            if(ch_hr_valid == 1 && n_heart_rate<120) {    			
                // ch_hr_valid == 1 && ch_spo2_valid ==1 && n_heart_rate<120 && n_sp02<101
                dis_hr = n_heart_rate;
                dis_spo2 = n_sp02;
            } else {
                dis_hr = 0;
                dis_spo2 = 0;
            }
	    }
		maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
		float BF = maxim_blood_flow_rate(aun_ir_buffer,  n_ir_buffer_length);
        //Process the data and update it to the global structure.
        sensordata.heart_rate = dis_hr;
        sensordata.spo2 = dis_spo2;  
		sensordata.blood_flow = BF;    
		vTaskDelay(100); 						// Task delay
    }
}

//Task si7021 - Measure environmental temperature and humidity as well as finger temperature and humidity
void vSi7021_task(void *pvParameters) {
	while(1) {	  
        taskENTER_CRITICAL();
        //data11 and data represent the finger temperature and humidity as well as the environmental temperature and humidity.
        float data11 = Si7021_Measure(&si7021_1, HUMI_NOHOLD_MASTER);  
        if (data11 >100) data11 = 100;         //When the humidity exceeds 100%, set it to 100%.
        float data12 = Si7021_TEMP_Measure(&si7021_1);
        sensordata.humidity_wrist = data11;
        sensordata.temperature_wrist = data12;
        //Data 21 and data 22 represent environmental humidity and temperature respectively.
        float data21 = Si7021_Measure(&si7021_2, HUMI_NOHOLD_MASTER);  
        if (data21 >100) data21 = 100;         //When the humidity exceeds 100%, set it to 100%. 
        float data22 = Si7021_TEMP_Measure(&si7021_2);
        sensordata.humidity_ext = data21;
        sensordata.temperature_ext = data22;
        taskEXIT_CRITICAL();
        vTaskDelay(100);	
    }
}
 
//GY906 Task - Measure Infrared Temperature
void vGy906_task(void *pvParameters) {
    //UBaseType_t freeNum;
    while (1) {
        taskENTER_CRITICAL();
        float temp = SMBus_ReadTemp();
        sensordata.ir_temperature = temp;
		taskEXIT_CRITICAL();
		vTaskDelay(100);				
    }
}

/* ===================================================== */
#define AUTO_CONNECT_WIFI (1)
#if AUTO_CONNECT_WIFI
// clang-format off
/* config your wifi ssid and password */
uint8_t wifi_sta_connet[50] = "wifi_sta_connect";
extern Ring_Buffer_Type shell_rb;
extern void shell_release_sem(void);
SemaphoreHandle_t sem_wifi_init_done;
SemaphoreHandle_t sem_wifi_connect_done;
SemaphoreHandle_t sem_wifi_disconnect;
struct bflb_device_s *wdg;
static TaskHandle_t ota_task_handle;
#endif
volatile uint32_t wifi_strat_flag = 1;
/* WIFI ID PASSID */ 
volatile uint8_t *PASSID_Wifi[20];
volatile uint8_t *SSID_Wifi[20];
volatile struct bflb_device_s *gpio;
static struct bflb_device_s *uart0;
struct bflb_device_s *rtc;
struct bflb_tm g_time;
extern void shell_init_with_task(struct bflb_device_s *shell);
/* ===================================================================== */
/* ble task */
static void ble_connected(struct bt_conn *conn, u8_t err) {
    if(err || conn->type != BT_CONN_TYPE_LE) {
        return;
    }
    printf("%s",__func__);
}

static void ble_disconnected(struct bt_conn *conn, u8_t reason) { 
    int ret;
    if(conn->type != BT_CONN_TYPE_LE) {
        return;
    }
    printf("%s",__func__);
    // enable adv
    ret = set_adv_enable(true);
    if(ret) {
        printf("Restart adv fail. \r\n");
    }
}

static struct bt_conn_cb ble_conn_callbacks = {
	.connected	=   ble_connected,
	.disconnected	=   ble_disconnected,
};

static void ble_start_adv(void) {
    struct bt_le_adv_param param;
    int err = -1;
    struct bt_data adv_data[1] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL)
    };
    struct bt_data adv_rsp[1] = {
        BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, "BL616")
    };
    memset(&param, 0, sizeof(param));
    // Set advertise interval
    param.interval_min = BT_GAP_ADV_FAST_INT_MIN_2;
    param.interval_max = BT_GAP_ADV_FAST_INT_MAX_2;
    /*Get adv type, 0:adv_ind,  1:adv_scan_ind, 2:adv_nonconn_ind 3: adv_direct_ind*/
    param.options = (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_ONE_TIME); 
    err = bt_le_adv_start(&param, adv_data, ARRAY_SIZE(adv_data), adv_rsp, ARRAY_SIZE(adv_rsp));
    if(err){
        printf("Failed to start advertising (err %d) \r\n", err);
    }
    printf("Start advertising success.\r\n");
}

bool ble_check_oad(u32_t cur_file_ver, u32_t new_file_ver) {
    return true;
}

void bt_enable_cb(int err) {
    if (!err) {
        bt_addr_le_t bt_addr;
        bt_get_local_public_address(&bt_addr);
        printf("BD_ADDR:(MSB)%02x:%02x:%02x:%02x:%02x:%02x(LSB) \r\n",
            bt_addr.a.val[5], bt_addr.a.val[4], bt_addr.a.val[3], bt_addr.a.val[2], bt_addr.a.val[1], bt_addr.a.val[0]);
        bt_conn_cb_register(&ble_conn_callbacks);
        ble_tp_init();
        #if defined(CONFIG_BT_OAD_SERVER)
            oad_service_enable(ble_check_oad);
            // start advertising
            ble_start_adv();
        #endif
    }
}

static TaskHandle_t app_start_handle;
static void app_start_task(void *pvParameters) {
    // Initialize BLE controller
    #if defined(BL702) || defined(BL602)
    ble_controller_init(configMAX_PRIORITIES - 1);
    #else
    btble_controller_init(configMAX_PRIORITIES - 1);
    #endif
    // Initialize BLE Host stack
    hci_driver_init();
    bt_enable(bt_enable_cb);
    vTaskDelete(NULL);
}
/* ===================================================================== */
/* ===================================================================== */
/* wifi task demo */
TaskHandle_t LWIP_Task_Handler;             
wifi_mgmr_scan_params_t wifi_list;
int wifi_statues =  -1;
static TaskHandle_t wifi_fw_task;
static wifi_conf_t conf = {
    .country_code = "CN",
};
int wifi_start_firmware_task(void) {
    LOG_I("Starting wifi ...\r\n");
    /* enable wifi clock */
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_IP_WIFI_PHY | GLB_AHB_CLOCK_IP_WIFI_MAC_PHY | GLB_AHB_CLOCK_IP_WIFI_PLATFORM);
    GLB_AHB_MCU_Software_Reset(GLB_AHB_MCU_SW_WIFI);
    /* Enable wifi irq */
    extern void interrupt0_handler(void);
    bflb_irq_attach(WIFI_IRQn, (irq_callback)interrupt0_handler, NULL);
    bflb_irq_enable(WIFI_IRQn);
    xTaskCreate(wifi_main, (char *)"fw", 2048, NULL, configMAX_PRIORITIES - 6, &wifi_fw_task);
    return 0;
}

volatile TaskHandle_t mqtt_Task_Handler;
extern void mqtt_task(void *pvParameter);
extern void example_mqtt(void);
volatile uint32_t time_get_flag = 1;
volatile uint32_t wifi_state = 0;
volatile uint32_t ai_chat_state = 1;
volatile uint32_t ai_get_time_state = 1;
void wifi_event_handler(uint32_t code) {
    uint32_t ret = 0;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    switch (code) {
        case CODE_WIFI_ON_INIT_DONE: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_INIT_DONE\r\n", __func__);
            wifi_mgmr_init(&conf);
        } break;
        case CODE_WIFI_ON_MGMR_DONE: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_MGMR_DONE\r\n", __func__);
            // do {
            //     wifi_statues = wifi_mgmr_sta_quickconnect(PASSID_Wifi, SSID_Wifi, 2442, 5000);
            // } while(wifi_statues!=0);
            #if AUTO_CONNECT_WIFI
                ret = xSemaphoreGiveFromISR(sem_wifi_init_done, &xHigherPriorityTaskWoken);
                if (ret == pdPASS) {
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            #endif
        } break;
        case CODE_WIFI_ON_SCAN_DONE: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_SCAN_DONE\r\n", __func__);
            wifi_mgmr_sta_scanlist(); 
        } break;
        case CODE_WIFI_ON_CONNECTED: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_CONNECTED\r\n", __func__);
            void mm_sec_keydump();
            mm_sec_keydump();
        } break;
        case CODE_WIFI_ON_GOT_IP: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_GOT_IP\r\n", __func__);
            wifi_state = 1;
            #if AUTO_CONNECT_WIFI
                ret = xSemaphoreGiveFromISR(sem_wifi_connect_done, &xHigherPriorityTaskWoken);
                if (ret == pdPASS) {
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            #endif
            if (wifi_state == 1 && ai_get_time_state == 1) {
                ai_get_time_state = 0;
                uint8_t get_time_cmd[10] = "get_time\r";
                Ring_Buffer_Write(&shell_rb, get_time_cmd, sizeof(get_time_cmd) - 1);
                shell_release_sem(); 
            }
        } break;
        case CODE_WIFI_ON_DISCONNECT: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_DISCONNECT\r\n", __func__);
            wifi_state = 0;
            #if AUTO_CONNECT_WIFI
                ret = xSemaphoreGiveFromISR(sem_wifi_disconnect, &xHigherPriorityTaskWoken);
                if (ret == pdPASS) {
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }
            #endif
        } break;
        case CODE_WIFI_ON_AP_STARTED: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_AP_STARTED\r\n", __func__);
        } break;
        case CODE_WIFI_ON_AP_STOPPED: {
            LOG_I("[APP] [EVT] %s, CODE_WIFI_ON_AP_STOPPED\r\n", __func__);
        } break;
        case CODE_WIFI_ON_AP_STA_ADD: {
            LOG_I("[APP] [EVT] [AP] [ADD] %lld\r\n", xTaskGetTickCount());
        } break;
        case CODE_WIFI_ON_AP_STA_DEL: {
            LOG_I("[APP] [EVT] [AP] [DEL] %lld\r\n", xTaskGetTickCount());
        } break;
        default: {
            LOG_I("[APP] [EVT] Unknown code %u \r\n", code);
        }
    }
}

void ota_task(void *param);
/**
 * @brief       lwIP????????
 * @param       pvParameters : ???????(?????)
 * @retval      ??
 */
void lwip_demo_task(void *pvParameters) {
    while (1) {
        if(wifi_strat_flag == 0) {
            bt_le_adv_stop();
            vTaskDelay(5000);
            wifi_start_firmware_task();
            #if AUTO_CONNECT_WIFI
                xTaskCreate(ota_task, "ota_task", 512, NULL, configMAX_PRIORITIES - 17, &ota_task_handle);
            #endif
            wifi_strat_flag = 2;
        }
        if(wifi_strat_flag == 2 && time_get_flag == 0) {
            break;    
        }  
        vTaskDelay(500);
    }
    vTaskDelete(NULL);
}
/* ===================================================================== */
#if AUTO_CONNECT_WIFI
void ota_task(void *param) {
    if (xSemaphoreTake(sem_wifi_init_done, portMAX_DELAY) == pdTRUE) {
        strcat(wifi_sta_connet, " ");
        strcat(wifi_sta_connet, PASSID_Wifi);
        strcat(wifi_sta_connet, " ");
        strcat(wifi_sta_connet, SSID_Wifi);
        strcat(wifi_sta_connet, "\r");
        Ring_Buffer_Write(&shell_rb, wifi_sta_connet, sizeof(wifi_sta_connet) - 1);
        shell_release_sem();
    }
    if (xSemaphoreTake(sem_wifi_connect_done, portMAX_DELAY) == pdTRUE) {

    }
    while (1) {
        if (xSemaphoreTake(sem_wifi_disconnect, portMAX_DELAY) == pdTRUE) {
            if (wifi_strat_flag == 2 && wifi_state == 0) {
                Ring_Buffer_Write(&shell_rb, wifi_sta_connet, sizeof(wifi_sta_connet) - 1);
                shell_release_sem();
            }
        }
        vTaskDelay(1000);
    }
}
#endif
/* ===================================================================== */
TaskHandle_t LCD_Task_Handler;            
void lcd_demo_task(void *pvParameters);    
__attribute__((section(".psram_data"))) volatile uint8_t *buf_time_data[20];
volatile uint32_t ble_flag = 1;
volatile uint8_t *blood_sugar_ble[5];
volatile uint32_t year, month, day, hour, minute, second;
volatile void parse_datetime(uint8_t *datetime_str, uint32_t *year, uint32_t *month, uint32_t *day, uint32_t *hour, uint32_t *minute, uint32_t *second) {
    sscanf(datetime_str, "%d-%d-%d %d:%d:%d", year, month, day, hour, minute, second);
}
char datetime_str[64];
float GLU_Tinymaix = 0;
void lcd_demo_task(void *pvParameters) {
    lcd_init();
    lcd_clear(0x0000);
	lcd_set_dir(2,1);
    while (1) {
        if(wifi_strat_flag == 2 && time_get_flag == 0) {
            printf("Flag out: %d\r\n", time_get_flag);
            break;
        } else {
            printf("Flag in: %d\r\n", time_get_flag);
        }
        vTaskDelay(1000);
    }
    lcd_draw_area( 0, 0, 240, 130, LCD_COLOR_RGB(255, 255, 255));
	lcd_draw_str_ascii16_with_chinese(10,40,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"ÎÂ¶È:",5);
	lcd_draw_str_ascii16_with_chinese(75,40,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"¡ãC",5);
	lcd_draw_str_ascii16_with_chinese(110,40,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"Êª¶È:",5);
	lcd_draw_str_ascii16_with_chinese(175,40,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"%",5);
	lcd_draw_str_ascii16_with_chinese(10,70,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"ÑªÌÇ:",5);
	lcd_draw_str_ascii16_with_chinese(100,70,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"mmol/L",6);
	lcd_draw_str_ascii16_with_chinese(10,100,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"ÉãÈëÁ¿ÖÖÀà:",11);
	lcd_draw_str_ascii16_with_chinese(110,100,LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255),"¿ìÌ¼",10);
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    uint8_t point_num = 0;
    touch_coord_t touch_coord;
    // bool last_touch = 0;  // Last touch status: 0 = Not touched, 1 = Touched
    //Convert the data into a string and store it in an array, which is used for LCD display.
	char temper_str[20];
    char humid_str[20];
	char bloodsuger_str[20];
    rtc = bflb_device_get_by_name("rtc");
    bflb_rtc_set_time(rtc, 0);
    g_time.tm_sec = second;
    g_time.tm_min = minute;
    g_time.tm_hour = hour;
    g_time.tm_wday = 7;
    g_time.tm_mday = day;
    g_time.tm_mon = month - 1;
    g_time.tm_year = year - 1900;
    bflb_rtc_set_utc_time(&g_time);
    // float blood_sugar = 0;
    bool last_touch = 0;
    while (1) {
        // printf("LCD : %s\n", buf_time_data);
        bflb_rtc_get_utc_time(&g_time);
        snprintf(datetime_str, sizeof(datetime_str), "%d-%d-%d %d:%d:%d  ", g_time.tm_year + 1900, g_time.tm_mon + 1, g_time.tm_mday, g_time.tm_hour, g_time.tm_min, g_time.tm_sec);
        lcd_draw_str_ascii16(10, 10, LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255), datetime_str, 22);
        taskENTER_CRITICAL();
		// int temper = sensordata.temperature_ext;  // environment temperature
		// int humid = sensordata.humidity_ext;      // environment humidity
		// blood_sugar = (sensordata.blood_sugar + GLU_Tinymaix) / 2.0;  // blood glucose level
        // if (blood_sugar > 10 || blood_sugar < 4.5) blood_sugar = 0.0;
        taskEXIT_CRITICAL();
		touch_read(&point_num, &touch_coord, 1);
        // sprintf(bloodsuger_str,"%.1f",blood_sugar);
        sprintf(bloodsuger_str,"%.1f",sensordata.blood_sugar);
        if (point_num && last_touch == 0) {
            last_x = touch_coord.coord_x;
            last_y = touch_coord.coord_y<=160?touch_coord.coord_y+160:touch_coord.coord_y-160;
            // Determine whether the touch is within the specified circular button area
            if (last_x >= SENSOR_AREA_X1 && last_x <= SENSOR_AREA_X2 &&
                last_y >= SENSOR_AREA_Y1 && last_y <= SENSOR_AREA_Y2){
                if (ble_flag == 0) {
                    lcd_draw_str_ascii16(55, 70, LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255), blood_sugar_ble, strlen(blood_sugar_ble));      // lCD?1?7?1?7?0?5?0?4?1?7?1?7?0?5
                    vTaskDelay(200);
                    ble_flag = 1;
                } else {
                    vTaskDelay(200);
                    lcd_draw_str_ascii16(55, 70, LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255), bloodsuger_str, strlen(bloodsuger_str));      // lCD?1?7?1?7?0?5?0?4?1?7?1?7?0?5    
                }
                printf("x=%d, y=%d\r\n",last_x,last_y);
            }
            last_touch = 1;
		} else {
            last_touch = 0;
            vTaskDelay(200);
        }
        point_num = 0;
		// lcd_draw_area(71, 50, 94, 50, 0x0000);     //Remove data remnants
		sprintf(temper_str,"%d", (int)sensordata.temperature_ext);    
        sprintf(humid_str,"%d", (int)sensordata.humidity_ext);
		// Display data on the LCD
		lcd_draw_str_ascii16(55, 40, LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255), temper_str, strlen(temper_str));  // ?1?7?1?7?0?5?1?7?1?7?1?7?1?7?1?7?0?9?1?7
		lcd_draw_str_ascii16(155, 40, LCD_COLOR_RGB(0, 0, 0), LCD_COLOR_RGB(255, 255, 255), humid_str, strlen(humid_str));   // ?1?7?1?7?0?5?1?7?1?7?1?7?1?7?0?5?1?7?1?7
        printf("utc time:%u-%u-%u, %u:%u:%u, wday:%u\r\n",
               g_time.tm_year + 1900, g_time.tm_mon + 1, g_time.tm_mday,
               g_time.tm_hour, g_time.tm_min, g_time.tm_sec,
               g_time.tm_wday);
        vTaskDelay(700);
    }
}
/* ===================================================================== */
/* minimp3 for mp3 data decode */
/*---------------------------------------------------------*/
__attribute__((section(".psram_data"))) uint8_t *pcm_data_tx     __attribute__((aligned(4))) = NULL;  //1024kB for playing the tts audio
__attribute__((section(".psram_data"))) uint8_t *psram_mp3_buf   __attribute__((aligned(4))) = NULL;  //kB for storing the mp3 data
#define PCM_BUFFER_SIZE    (1152 * 2 * 2)
#define PCM_DATA_TX        (1024 * 1024)
void decode_mp3_buffer(const uint8_t *mp3_data, size_t mp3_size)
{
    mp3dec_t mp3d;
    mp3dec_frame_info_t info;
    int mp3_decode_loop = 0;
    int16_t pcm[PCM_BUFFER_SIZE];
    size_t offset = 0;
    size_t tx_offset = 0; // ???pcm_data_tx??ï¿½ï¿½??ï¿½ï¿½??
    mp3dec_init(&mp3d);
    while (offset < mp3_size){
        // ??????? MP3 ?
        int samples = mp3dec_decode_frame(&mp3d, mp3_data + offset, mp3_size - offset, pcm, &info);
        if (samples > 0) {
            size_t pcm_bytes = samples * sizeof(int16_t);
            if (tx_offset + pcm_bytes <= PCM_DATA_TX) { // ??????
                memcpy(pcm_data_tx + tx_offset, pcm, pcm_bytes);
                tx_offset += pcm_bytes;
            } else {
                // ??????????????
                break;
            }
        }
        if (info.frame_bytes == 0) {
            break;
        }
        offset += info.frame_bytes;
        mp3_decode_loop = mp3_decode_loop + 1;
    }
    printf("\r%dHZ\n",info.hz);
    printf("offset = %zu, tx_offset = %zu, mp3_decode_loop = %d\n", offset, tx_offset, mp3_decode_loop);
    i2s_tx_dma_init(pcm_data_tx, tx_offset);
}
/*---------------------------------------------------------*/
struct bflb_device_s *i2s0;
struct bflb_device_s *i2c0;
struct bflb_device_s *dma0_ch0;
uint8_t audio_played_flag = 1; // ?????????????????
static ES8388_Cfg_Type ES8388Cfg = {
    .work_mode = ES8388_CODEC_MDOE,          /*!< ES8388 work mode */
    .role = ES8388_SLAVE,                    /*!< ES8388 role */
    .mic_input_mode = ES8388_DIFF_ENDED_MIC, /*!< ES8388 mic input mode */
    .mic_pga = ES8388_MIC_PGA_3DB,           /*!< ES8388 mic PGA */
    .i2s_frame = ES8388_LEFT_JUSTIFY_FRAME,  /*!< ES8388 I2S frame */
    .data_width = ES8388_DATA_LEN_16,        /*!< ES8388 I2S dataWitdh */
};

void dma0_ch0_isr(void *arg)
{
    if(audio_played_flag == 1) {
        audio_played_flag = 0;
        memset(pcm_data_tx, 0, 1024*1024);
        bflb_mtimer_delay_ms(100); // ??? DMA ???????
        printf("\r tc done \n");
    }
    bflb_dma_channel_stop(dma0_ch0);
}

void i2s_gpio_init()
{
    struct bflb_device_s *gpio;
    gpio = bflb_device_get_by_name("gpio");
    /* I2S_FS */
    bflb_gpio_init(gpio, GPIO_PIN_1, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_DI */
    bflb_gpio_init(gpio, GPIO_PIN_10, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_DO */
    bflb_gpio_init(gpio, GPIO_PIN_3, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_BCLK */
    bflb_gpio_init(gpio, GPIO_PIN_0, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* MCLK CLKOUT */
    bflb_gpio_init(gpio, GPIO_PIN_2, GPIO_FUNC_CLKOUT | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2C0_SCL */
    bflb_gpio_init(gpio, GPIO_PIN_14, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);
    /* I2C0_SDA */
    bflb_gpio_init(gpio, GPIO_PIN_15, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_2);
}

void i2s_dma_init(uint8_t voice_flag) {
    static struct bflb_dma_channel_lli_pool_s tx_llipool[100];
    static struct bflb_dma_channel_lli_transfer_s tx_transfers[1];
    struct bflb_i2s_config_s i2s_cfg = {
        .bclk_freq_hz = 16000 * 32 * 1, /* bclk = Sampling_rate * frame_width * channel_num */
        .role = I2S_ROLE_MASTER,
        .format_mode = ES8388_STD_I2S_FRAME,
        .channel_mode = I2S_CHANNEL_MODE_NUM_1,
        .frame_width = I2S_SLOT_WIDTH_16,
        .data_width = I2S_SLOT_WIDTH_16,
        .fs_offset_cycle = 0,
        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
    };
    struct bflb_dma_channel_config_s tx_config = {
        .direction = DMA_MEMORY_TO_PERIPH,
        .src_req = DMA_REQUEST_NONE,
        .dst_req = DMA_REQUEST_I2S_TX,
        .src_addr_inc = DMA_ADDR_INCREMENT_ENABLE,
        .dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE,
        .src_burst_count = DMA_BURST_INCR1,
        .dst_burst_count = DMA_BURST_INCR1,
        .src_width = DMA_DATA_WIDTH_16BIT,
        .dst_width = DMA_DATA_WIDTH_16BIT,
    };
    printf("i2s init\r\n");
    i2s0 = bflb_device_get_by_name("i2s0");
    /* i2s init */
    bflb_i2s_init(i2s0, &i2s_cfg);
    /* enable dma */
    bflb_i2s_link_txdma(i2s0, true);
    bflb_i2s_link_rxdma(i2s0, true);
    printf("dma init\r\n");
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");
    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_irq_attach(dma0_ch0, dma0_ch0_isr, NULL);
    if (voice_flag == 2) {
        tx_transfers[0].src_addr = (uint32_t)Bloodsugar_high;
        tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
        tx_transfers[0].nbytes = sizeof(Bloodsugar_high);
    } else if (voice_flag == 3){
        tx_transfers[0].src_addr = (uint32_t)Bloodsugar_low;
        tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
        tx_transfers[0].nbytes = sizeof(Bloodsugar_low);
    }   
    printf("dma lli init\r\n");
    uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 12, tx_transfers, 1);
    bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
    printf("dma lli num: %d \r\n", num);
    bflb_dma_channel_start(dma0_ch0);
    bflb_i2s_feature_control(i2s0, I2S_CMD_DATA_ENABLE, I2S_CMD_DATA_ENABLE_TX | I2S_CMD_DATA_ENABLE_RX);
}

void i2s_tx_dma_init(uint8_t *arr, uint32_t size) {
    static struct bflb_dma_channel_lli_pool_s tx_llipool[100];
    static struct bflb_dma_channel_lli_transfer_s tx_transfers[1];
    struct bflb_i2s_config_s i2s_cfg = {
        .bclk_freq_hz = 24000 * 16 * 1,
        .role = I2S_ROLE_MASTER,
        .format_mode = I2S_MODE_LEFT_JUSTIFIED,
        .channel_mode = I2S_CHANNEL_MODE_NUM_2,
        .frame_width = I2S_SLOT_WIDTH_16,
        .data_width = I2S_SLOT_WIDTH_16,
        .fs_offset_cycle = 0,
        .tx_fifo_threshold = 0,
        .rx_fifo_threshold = 0,
    };
    struct bflb_dma_channel_config_s tx_config = {
        .direction = DMA_MEMORY_TO_PERIPH,
        .src_req = DMA_REQUEST_NONE,
        .dst_req = DMA_REQUEST_I2S_TX,
        .src_addr_inc = DMA_ADDR_INCREMENT_ENABLE,
        .dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE,
        .src_burst_count = DMA_BURST_INCR1,
        .dst_burst_count = DMA_BURST_INCR1,
        .src_width = DMA_DATA_WIDTH_16BIT,
        .dst_width = DMA_DATA_WIDTH_16BIT,
    };
    printf("i2s init\r\n");
    i2s0 = bflb_device_get_by_name("i2s0");
    bflb_i2s_init(i2s0, &i2s_cfg);
    bflb_i2s_link_txdma(i2s0, true);
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");
    bflb_dma_channel_init(dma0_ch0, &tx_config);
    bflb_dma_channel_irq_attach(dma0_ch0, dma0_ch0_isr, NULL);
    tx_transfers[0].src_addr = (uint32_t)arr;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
    tx_transfers[0].nbytes = size;
    printf("dma lli init\r\n");
    uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
    bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
    bflb_dma_channel_start(dma0_ch0);
    bflb_i2s_feature_control(i2s0, I2S_CMD_DATA_ENABLE, I2S_CMD_DATA_ENABLE_TX);
}

void mclk_out_init()
{
#ifdef BL616
    /* output MCLK,
    Will change the clock source of i2s,
    It needs to be called before i2s is initialized
    clock source 25M
    */
    GLB_Set_I2S_CLK(ENABLE, 2, GLB_I2S_DI_SEL_I2S_DI_INPUT, GLB_I2S_DO_SEL_I2S_DO_OUTPT);
    GLB_Set_Chip_Clock_Out3_Sel(GLB_CHIP_CLK_OUT_3_I2S_REF_CLK);
#endif
}

uint8_t voice_flag = 2;
uint8_t ai_chat_flag = 1;
TaskHandle_t Voice_Task_Handler;
void voice_task(void *pvParameters) {
    while (1) {
        if(wifi_strat_flag == 2 && time_get_flag == 0) {
            break;
        }
        vTaskDelay(500);
    }
    vTaskDelay(500);
    pvParameters = pvParameters;
    printf("\n\ri2s dma test\n\r");
    /* gpio init */
    i2s_gpio_init();
    /* init ES8388 Codec */
    printf("es8388 init\n\r");
    ES8388_Init(&ES8388Cfg);
    ES8388_Set_Voice_Volume(100);
    /* mclk clkout init */
    mclk_out_init();
    /* i2s init */
    // i2s_dma_init();
    /* enable i2s tx and rx */
    bflb_i2s_feature_control(i2s0, I2S_CMD_DATA_ENABLE, I2S_CMD_DATA_ENABLE_TX | I2S_CMD_DATA_ENABLE_RX);
    printf("test end\n\r");
    // xTaskCreate(example_mqtt, "eample_mqtt", 1024, NULL, configMAX_PRIORITIES - 13, NULL);
    while (1) {
        if (voice_flag == 1) {
            if(ai_chat_flag == 1) {
                uint8_t ai_chat_cmd[100] = "ai_chat %E4%BD%A0%E6%98%AF%E8%B0%81%EF%BC%9F\r";
                Ring_Buffer_Write(&shell_rb, ai_chat_cmd, sizeof(ai_chat_cmd) - 1);
                shell_release_sem();
                ai_chat_flag = 0;
            }
            voice_flag = 0;
        } else if (voice_flag == 2) {
            i2s_dma_init(voice_flag);
            voice_flag = 0;
        } else if (voice_flag == 3) {
            i2s_dma_init(voice_flag);
            voice_flag = 0;
        }
        vTaskDelay(500);
    }
}
/*---------------------------------------------------------*/
// float GLU_Tinymaix = 0;
static tm_err_t layer_cb(tm_mdl_t* mdl, tml_head_t* lh) {   
    //dump middle result
    int h = lh->out_dims[1];
    int w = lh->out_dims[2];
    int ch= lh->out_dims[3];
    mtype_t* output = TML_GET_OUTPUT(mdl, lh);
    for(int y=0; y<h; y++){
        for(int x=0; x<w; x++){
            for(int c=0; c<ch; c++){
                GLU_Tinymaix = output[(y*w+x)*ch+c];
            }
        }
    }
    return TM_OK;
}
volatile double BMR;
void tinymaix_1(void) {
    printf("Glucose Prediction Demo\n");
    TM_DBGT_INIT();
    tm_mdl_t mdl;
    // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    float glucose_input[4] = {148.59f, 97.0f, 75.0f, 28.71f};
    for(int i=0; i<4; i++){
        TM_PRINTF("%f,", glucose_input[i]);
        TM_PRINTF("\n");
    }
    // ×¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý½á¹¹ - ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½MNISTÊ¾ï¿½ï¿½ï¿½ï¿½Ê½
    tm_mat_t in_float = {1, 1, 1, 4, {(mtype_t*)glucose_input}};
    tm_mat_t in = {1, 1, 1, 4, {NULL}};
    tm_mat_t outs[1];
    tm_err_t res;
    // ï¿½ï¿½Ó¡Ä£ï¿½ï¿½ï¿½ï¿½Ï¢
    tm_stat((tm_mdlbin_t*)mdl_data);  // glucose_modelï¿½æ»»Îªï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    // ï¿½ï¿½ï¿½ï¿½Ä£ï¿½ï¿½ - ï¿½ï¿½È«ï¿½ï¿½ï¿½ï¿½MNISTÊ¾ï¿½ï¿½ï¿½ï¿½Ê½
    res = tm_load(&mdl, mdl_data, NULL, layer_cb, &in);
    if(res != TM_OK) {
        printf("Load failed.\n");
    }
    while(1){
        glucose_input[0] = BMR;
        glucose_input[1] = sensordata.spo2;
        glucose_input[2] = sensordata.heart_rate;
        glucose_input[3] = sensordata.blood_flow;
        tm_mat_t in_float = {1, 1, 1, 4, {(mtype_t*)glucose_input}};
        // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
        TM_DBGT_START();
        res = tm_run(&mdl, &in_float, outs); // ï¿½Þ¸Äºï¿½Ä´ï¿½ï¿½ï¿??ï¿½ï¿½È·ï¿½ï¿½ï¿½Ý²ï¿½ï¿½ï¿½
        TM_DBGT("tm_run");
        if(res == TM_OK) {
            // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
            printf("Blood Sugar in tinymaix:%.1f\r\n", GLU_Tinymaix);
        } else {
            printf("Inference error: %d\n", res);
        }
        vTaskDelay(1000);
    }
    tm_unload(&mdl);
}

//Define the area for ECG
#define CURVE_X 10
#define CURVE_Y 200
#define CURVE_W 220
#define CURVE_H 100
struct bflb_device_s *adc;
#define TEST_ADC_CHANNEL_10 1
#define TEST_ADC_CHANNELS (TEST_ADC_CHANNEL_10)
#define CURVE_POINT_NUM 64
struct bflb_adc_channel_s chan[] = {
#if TEST_ADC_CHANNEL_10
    { .pos_chan = ADC_CHANNEL_10,
      .neg_chan = ADC_CHANNEL_GND },
#endif
};
volatile uint32_t raw_data[TEST_ADC_CHANNELS];
volatile uint8_t read_count = 0;

void adc_isr(int irq, void *arg) {
    uint32_t intstatus = bflb_adc_get_intstatus(adc);
    if (intstatus & ADC_INTSTS_ADC_READY) {
        bflb_adc_int_clear(adc, ADC_INTCLR_ADC_READY);
        uint8_t count = bflb_adc_get_count(adc);
        for (size_t i = 0; i < count; i++) {
            if (read_count < TEST_ADC_CHANNELS) { 
                raw_data[read_count] = bflb_adc_read_raw(adc);
                read_count++;
            } else {                
                (void)bflb_adc_read_raw(adc);
            }
        }
    }
}

//Data output task - Used for data processing and serial port display
void AD8232_Task(void *pvParameter) {
    uint16_t curve_data[CURVE_POINT_NUM] = {0};
    uint16_t curve_index = 0;
	while (1) {
		taskENTER_CRITICAL();
        read_count = 0;
        bflb_adc_start_conversion(adc);
        uint32_t last_clear_tick = 0;
        while (read_count < TEST_ADC_CHANNELS) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        for (size_t j = 0; j < TEST_ADC_CHANNELS; j++) {
            struct bflb_adc_result_s result;
            bflb_adc_parse_result(adc, (uint32_t *)&raw_data[j], &result, 1);
            // Store the result in curve_data for visualization
            curve_data[curve_index++] = result.millivolt;
            if (curve_index >= CURVE_POINT_NUM) {
                curve_index = 0; // Reset index if it exceeds the buffer size
            }
        }
        bflb_adc_stop_conversion(adc);
        lcd_draw_area(CURVE_X, CURVE_Y, CURVE_X + CURVE_W, CURVE_Y + CURVE_H, 0x0000);
        lcd_draw_curve(CURVE_X, CURVE_Y , CURVE_W, CURVE_H, curve_data, CURVE_POINT_NUM, 0, 3200, LCD_COLOR_RGB(0,0,255)); 
        taskEXIT_CRITICAL();  
		vTaskDelay(10);
	}
}


//Data output task - Used for data processing and serial port display
void mqtt_task(void *pvParameter) {
    while (1) {
        if(wifi_strat_flag == 2 && time_get_flag == 0) {
            break;
        } 
        vTaskDelay(500);
    }
    vTaskDelay(1000);
    // const uint8_t mqtt_cmd[10] = "mqtt_pub\r";
    // Ring_Buffer_Write(&shell_rb, mqtt_cmd, sizeof(mqtt_cmd) - 1);
    // shell_release_sem(); 
    example_mqtt();
	while(1) {
		vTaskDelay(1000);
	}
}
/* ===================================================================== */		

//Data output task - Used for data processing and serial port display
void vOutput_task(void *pvParameter) {
    //Calculation of finger metabolic rate and definitions of various parameters					 
	double bloodsugar;  //blood glucose level
	while(1) {
		taskENTER_CRITICAL();
        //Skin radiation temperature, measured in Kelvin (K)
		// M = 0.99 * 5.89e-8 * (pow((sensordata.ir_temperature  + 273.15)), 4)) + 2.38 * pow( (sensordata.temperature_wrist - sensordata.temperature_ext) , 0.25) * (sensordata.temperature_wrist - sensordata.temperature_ext) + 3.054 * (0.256 * sensordata.temperature_wrist - 3.37) * (1 - sensordata.humidity_ext * 0.01);
		BMR = (0.99 * 5.89e-8 * (pow((sensordata.ir_temperature  + 273.15), 4)) + \
              2.38 * pow((sensordata.temperature_wrist - sensordata.temperature_ext), 0.25) * (sensordata.temperature_wrist - sensordata.temperature_ext) + \
               3.054 * (0.256 * sensordata.temperature_wrist - 3.37) * (1 - sensordata.humidity_ext * 0.01)) \
               / 3.6; 
		bloodsugar = -23.9574 - 0.0310 * BMR + 0.000000 * sensordata.spo2 + 0.0873 * sensordata.heart_rate + 0.9751 * sensordata.blood_flow;  //Formula for calculating blood sugar level
		if (bloodsugar < 4.1 || bloodsugar > 10) bloodsugar = 0;  //Handling of abnormal blood sugar values: Set to 0 when the value is lower than 4.1 or higher than 10.
		sensordata.blood_sugar = bloodsugar;
        printf("Heart Rate:%d,Spo2:%d\n", sensordata.heart_rate, sensordata.spo2);
		printf("Temp_Wrist:%.1f,Humi_Wrist:%.1f\n",sensordata.temperature_wrist, sensordata.humidity_wrist);
		printf("Temp_Ext:%.1f,Humi_Ext:%.1f\n",sensordata.temperature_ext, sensordata.humidity_ext);
		printf ("IR_Temp:%.1f\n", sensordata.ir_temperature);
		printf("BMR:%.2f\n", BMR);
		printf("blood_flow:%.2f\n", sensordata.blood_flow);
        if (ble_flag == 0) {
            printf("Blood Sugar:%s\n", blood_sugar_ble); 
        } else {
            printf("Blood Sugar:%.1f\n", sensordata.blood_sugar); 
        }
		printf("\r\n");
        taskEXIT_CRITICAL();
		vTaskDelay(1000);
	}
}
/* ===================================================================== */
int main(void) {
    board_init();
    /* ===================================================================== */
    board_adc_gpio_init();
    adc = bflb_device_get_by_name("adc");
    /* adc clock = XCLK / 2 / 32 */
    struct bflb_adc_config_s cfg;
    cfg.clk_div = ADC_CLK_DIV_32;
    cfg.scan_conv_mode = true;
    cfg.continuous_conv_mode = false;
    cfg.differential_mode = false;
    cfg.resolution = ADC_RESOLUTION_16B;
    cfg.vref =ADC_VREF_3P2V;
    bflb_adc_init(adc, &cfg);
    bflb_adc_channel_config(adc, chan, TEST_ADC_CHANNELS);
    bflb_adc_rxint_mask(adc, false);
    bflb_irq_attach(adc->irq_num, adc_isr, NULL);
    bflb_irq_enable(adc->irq_num);
    /* ===================================================================== */
    // peripherals initial
	touch_init(&touch_max_point);
	SMBus_Init();
	SI7021_IIC_Init(&si7021_1);			
	SI7021_IIC_Init(&si7021_2);	
	MAX30102_Init();
    sensordata.humidity_ext = Si7021_Measure(&si7021_2, HUMI_NOHOLD_MASTER);
    if (sensordata.humidity_ext > 100) {
		sensordata.humidity_ext = 100;
	}
    sensordata.temperature_ext = Si7021_TEMP_Measure(&si7021_2);
    /* ===================================================================== */
    #if AUTO_CONNECT_WIFI
        sem_wifi_init_done = xSemaphoreCreateBinary();
        sem_wifi_connect_done = xSemaphoreCreateBinary();
        sem_wifi_disconnect = xSemaphoreCreateBinary();
    #endif
    configASSERT((configMAX_PRIORITIES > 4));
    uart0 = bflb_device_get_by_name("uart0");
    shell_init_with_task(uart0);
    bflb_mtd_init();
    /* ble stack need easyflash kv */
    easyflash_init();
    #if defined(BL616)
        /* Init rf */
        if (0 != rfparam_init(0, NULL, 0)) {
            printf("PHY RF init failed!\r\n");
            return 0;
        }
    #endif
    LOG_I("PHY RF init success!\r\n");
    tcpip_init(NULL, NULL);
    /* 1.pcm_data_tx */
    pcm_data_tx = malloc(1024*1024); //1024kB TX Buf
    if (pcm_data_tx == NULL) {
        free(pcm_data_tx);
        printf("\r pcm_data_tx do not have space\n");
    }
    memset(pcm_data_tx, 0, 1024*1024);
    printf("\r pcm_data_tx OK: 0x%08X\n",pcm_data_tx);
    bflb_mtimer_delay_ms(100); // wait for the memory to be ready
    /* 5.psram_mp3_buf for mp3 */
    /* This buf stores the MP3 data for STT and TTS */
    psram_mp3_buf = malloc(1024*512); //512kB RX Buf
    if (psram_mp3_buf == NULL) {
        free(psram_mp3_buf);
        printf("\r psram_mp3_buf do not have space\n");
    }
    memset(psram_mp3_buf, 0, 1024*512);
    printf("\r psram_mp3_buf OK: 0x%08X\n",psram_mp3_buf);
    bflb_mtimer_delay_ms(100); // wait for the memory to be ready
    /* ===================================================================== */

    /* ===================================================================== */
    xTaskCreate(app_start_task, (char *)"app_start", 1024, NULL, configMAX_PRIORITIES - 2, &app_start_handle);
    xTaskCreate(lwip_demo_task, (char *)"lwip_demo_task", 1024, NULL, configMAX_PRIORITIES - 11, &LWIP_Task_Handler);
    xTaskCreate(lcd_demo_task, (char *)"lcd_demo_task", 1024, NULL, configMAX_PRIORITIES - 10, &LCD_Task_Handler);
    xTaskCreate(voice_task, (char *)"voice_task", 512, NULL, configMAX_PRIORITIES - 10, &Voice_Task_Handler);
    xTaskCreate(vMax30102_task, "MAX30102_Task", 512, NULL, configMAX_PRIORITIES - 11, &max30102_task_handle);
    xTaskCreate(vSi7021_task, "SI7021_Task", 512, NULL, configMAX_PRIORITIES - 12, &si7021_task_handle);
    xTaskCreate(vGy906_task, "GY906_Task", 512, NULL, configMAX_PRIORITIES - 12, &gy906_task_handle);
    xTaskCreate(AD8232_Task, "AD8232_Task", 512, NULL, configMAX_PRIORITIES - 12, NULL);
    xTaskCreate(tinymaix_1, "tinymaix_1", 512, NULL, configMAX_PRIORITIES - 13, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", 1024, NULL, configMAX_PRIORITIES - 5, &mqtt_Task_Handler);
    xTaskCreate(vOutput_task, "Output", 512, NULL, configMAX_PRIORITIES - 13, NULL);
    #if AUTO_CONNECT_WIFI
        // printf("Config Watchdog...\r\n");
        // struct bflb_wdg_config_s wdg_cfg;
        // wdg_cfg.clock_source = WDG_CLKSRC_32K;
        // wdg_cfg.clock_div = 31;
        // wdg_cfg.comp_val = 30000;
        // wdg_cfg.mode = WDG_MODE_RESET;
        // wdg = bflb_device_get_by_name("watchdog");
        // bflb_wdg_init(wdg, &wdg_cfg);
        // bflb_wdg_start(wdg);
        // bflb_wdg_stop(wdg);
        // bflb_wdg_reset_countervalue(wdg);
        // printf("Next delay 30s, wdg will reset it.\r\n");
        // xTaskCreate(ota_task, "ota_task", 2048, NULL, configMAX_PRIORITIES - 17, &ota_task_handle);
    #endif
    vTaskStartScheduler();
    while (1) {
    }
}
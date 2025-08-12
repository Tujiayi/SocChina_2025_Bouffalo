
/**
 * @file
 * A simple program that subscribes to a topic.
 */
#include "FreeRTOS_POSIX.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <lwip/errno.h>
#include <netdb.h>
#include "utils_getopt.h"
#include "mqtt.h"

#define ADDRESS     "iot-06z00bjpwkwaqbv.mqtt.iothub.aliyuncs.com"
#define PORT        "1883"
#define CLIENTID    "k2ac35IRoHv.BL618|securemode=2,signmethod=hmacsha256,timestamp=1751461230645|"   //设备的名称
#define USERNAME    "BL618\&k2ac35IRoHv" //产品ID
#define PASSWORD    "aba21df35d55512af89f4236db07642a986076fbd03b0331e65ab5bfda5417ed"  //使用onennet的token工具生成密码 工具地址 https://open.iot.10086.cn/doc/v5/fuse/detail/242
#define SUBTOPIC    "/sys/k2ac35IRoHv/BL618/thing/event/property/post_reply"//订阅topic  XXXXXX是产品ID   ??????是设备名称
#define PUBTOPIC    "/sys/k2ac35IRoHv/BL618/thing/event/property/post"      //发布topic
#define DATATOPIC    "/k2ac35IRoHv/BL618/user/data_BL618_to_PC"      //数据流转topic
#define DATATOPIC_2    "/k2ac35IRoHv/BL618/user/data_PC_to_BL618"      //数据流转topic

uint8_t sendbuf[1024]; /* sendbuf should be large enough to hold multiple whole mqtt messages */
volatile uint8_t recvbuf[512]; /* recvbuf should be large enough any whole mqtt message expected to be received */
// uint8_t message[128];
// uint8_t *sendbuf  __attribute__((aligned(4))) = NULL; /* sendbuf should be large enough to hold multiple whole mqtt messages */
// uint8_t *recvbuf  __attribute__((aligned(4))) = NULL; /* recvbuf should be large enough any whole mqtt message expected to be received */
// uint8_t *message  __attribute__((aligned(4))) = NULL;
// extern uint8_t *sendbuf; /* sendbuf should be large enough to hold multiple whole mqtt messages */
// extern uint8_t *recvbuf; /* recvbuf should be large enough any whole mqtt message expected to be received */
// extern uint8_t *message;
// uint8_t *sendbuf = NULL; /* sendbuf should be large enough to hold multiple whole mqtt messages */
// volatile uint8_t *recvbuf = NULL; /* recvbuf should be large enough any whole mqtt message expected to be received */
uint8_t *message = NULL;

static TaskHandle_t client_daemon;
int test_sockfd;
char* addr;

/*
    A template for opening a non-blocking POSIX socket.
*/
static int open_nb_socket(char* addr, char* port);

static int open_nb_socket(char* addr, char* port) {
    struct addrinfo hints = {0};
    hints.ai_family = AF_UNSPEC; /* IPv4 or IPv6 */
    hints.ai_socktype = SOCK_STREAM; /* Must be TCP */
    int sockfd = -1;
    int rv;
    struct addrinfo *p, *servinfo;
    /* get address information */
    rv = getaddrinfo(addr, port, &hints, &servinfo);
    if(rv != 0) {
        printf("Failed to open socket (getaddrinfo): %s\r\n", rv);
        return -1;
    }
    /* open the first possible socket */
    for(p = servinfo; p != NULL; p = p->ai_next) {
        sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (sockfd == -1) continue;
        /* connect to server */
        rv = connect(sockfd, p->ai_addr, p->ai_addrlen);
        if(rv == -1) {
          close(sockfd);
          sockfd = -1;
          continue;
        }
        break;
    }  
    /* free servinfo */
    freeaddrinfo(servinfo);
    /* make non-blocking */
    if (sockfd != -1) {
        int iMode = 1;
        ioctlsocket(sockfd, FIONBIO, &iMode);
    }
    return sockfd;
}

/**
 * @brief The function will be called whenever a PUBLISH message is received.
 */
static void publish_callback_1(void** unused, struct mqtt_response_publish *published);

/**
 * @brief The client's refresher. This function triggers back-end routines to
 *        handle ingress/egress traffic to the broker.
 *
 * @note All this function needs to do is call \ref __mqtt_recv and
 *       \ref __mqtt_send every so often. I've picked 100 ms meaning that
 *       client ingress/egress traffic will be handled every 100 ms.
 */
static void client_refresher(void* client);

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
extern float BMR;
extern SensorData sensordata;
bool Flag_record = 0;

void example_mqtt(void) {
    int ret = 0;
    /* Malloc */
    // recvbuf = malloc(512 * sizeof(uint8_t));
    // if (recvbuf == NULL) {
    //     free(recvbuf);
    //     printf("\r recvbuf do not have space\n");
    // }
    // memset(recvbuf, 0, 512 * sizeof(uint8_t));
    // sendbuf = malloc(512 * sizeof(uint8_t));
    // if (sendbuf == NULL) {
    //     free(sendbuf);
    //     printf("\r sendbuf do not have space\n");
    // }
    // memset(sendbuf, 0, 512 * sizeof(uint8_t));
    message = malloc(128 * sizeof(uint8_t));
    if (message == NULL) {
        free(message);
        printf("\r message do not have space\n");
    }
    memset(message, 0, 128 * sizeof(uint8_t));
    /* open the non-blocking TCP socket (connecting to the broker) */
    test_sockfd = open_nb_socket(ADDRESS, PORT);
    if (test_sockfd < 0) {
        printf("Failed to open socket: %d\r\n", test_sockfd);
    }
    /* setup a client */
    struct mqtt_client client;
    mqtt_init(&client, test_sockfd, sendbuf, sizeof(sendbuf), recvbuf, sizeof(recvbuf), publish_callback_1);
    /* Create an anonymous session */
    char* client_id = CLIENTID;
    /* Ensure we have a clean session */
    uint8_t connect_flags = MQTT_CONNECT_CLEAN_SESSION;
    /* Send connection request to the broker. */
    ret = mqtt_connect(&client, client_id, NULL, NULL, 0, USERNAME, PASSWORD, connect_flags, 400);
    vTaskDelay(1000);
    /* check that we don't have any errors */
    if (client.error != MQTT_OK) {
        printf("error: %s\r\n", mqtt_error_str(client.error));
        vTaskDelay(1000);
    }
    /* start a thread to refresh the client (handle egress and ingree client traffic) */
    xTaskCreate(client_refresher, (char*)"client_ref", 512,  &client, 10, &client_daemon);
    /* subscribe */
    mqtt_subscribe(&client, DATATOPIC_2, 0);
    /* start publishing the time */
    printf("listening for '%s' messages.\r\n", DATATOPIC_2);
    /* block wait CTRL-C exit */
    while(1) {
        /* publisher*/
        if (Flag_record == 1) {
            memset(message, 0, sizeof(message));
            sprintf(message,"\"params\":{\"BMR\":%f , \"SPO2\":%d , \"PF\":%d , \"BV\":%f , \"GLU\":%f}",
                    BMR, sensordata.spo2, sensordata.heart_rate, sensordata.blood_flow, sensordata.blood_sugar);
            ret = mqtt_publish(&client, DATATOPIC,
                                message, strlen(message) + 1,
                                MQTT_PUBLISH_QOS_0);
            if (ret != MQTT_OK) {
                printf("ERROR! mqtt_publish() %s\r\n", mqtt_error_str(client.error));
            }
            Flag_record = 0;
        }
        memset(message, 0, sizeof(message));
        sprintf(message,"{\"method\":\"thing.service.property.post\", \"id\":\"12345\", "
                        " \"params\":{\"Status_device\":1, \"BMR\":%f , \"SPO2\":%d , \"PF\":%d , \"BV\":%f , \"GLU\":%f, \"Flag_record\":%d}, "
                        "\"version\" : \"1.0.0\"}",
                BMR, sensordata.spo2, sensordata.heart_rate, sensordata.blood_flow, sensordata.blood_sugar, Flag_record);
        ret = mqtt_publish(&client, PUBTOPIC,
                             message, strlen(message) + 1,
                             MQTT_PUBLISH_QOS_0);
        if (ret != MQTT_OK) {
            printf("ERROR! mqtt_publish() %s\r\n", mqtt_error_str(client.error));
        }
        printf("MQTT Test.\r\n");
        vTaskDelay(3000);
    }

    /* disconnect */
    free(recvbuf);
    free(sendbuf);
    free(message);
    /* exit */
}

static void publish_callback_1(void** unused, struct mqtt_response_publish *published)
{
    if(strstr(published->topic_name,"/k2ac35IRoHv/BL618/user/data_PC_to_BL618")) {
        /* note that published->topic_name is NOT null-terminated (here we'll change it to a c-string) */
        char* topic_name = (char*) malloc(published->topic_name_size + 1);
        memcpy(topic_name, published->topic_name, published->topic_name_size);
        topic_name[published->topic_name_size] = '\0';
        char* topic_msg = (char*) malloc(published->application_message_size + 1);
        memcpy(topic_msg, published->application_message, published->application_message_size);
        topic_msg[published->application_message_size] = '\0';
        printf("Received publish('%s'): %s\r\n", topic_name, topic_msg);
        free(topic_name);
        free(topic_msg);
        Flag_record = 1;
    }
}

static void client_refresher(void* client){
    while(1){
        mqtt_sync((struct mqtt_client*) client);
        vTaskDelay(100);
    }
}

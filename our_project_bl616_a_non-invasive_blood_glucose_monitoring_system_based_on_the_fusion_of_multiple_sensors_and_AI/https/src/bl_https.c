#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdlib.h>
#include <string.h>
#include "https.h"
#include "bl_error.h"
#include "bl_https.h"
#include "mbedtls/ssl.h"
#include "cJSON.h"
#include "bflb_mtimer.h"
#define DBG_TAG "HTTP/S"
#include "log.h"

extern void decode_mp3_buffer(const uint8_t *mp3_data, size_t mp3_size);
extern uint8_t *psram_mp3_buf;

void *memmem(const void *haystack, size_t n, const void *needle, size_t m);
void *memmem(const void *haystack, size_t n, const void *needle, size_t m)
{
	const unsigned char *y = (const unsigned char *)haystack;
	const unsigned char *x = (const unsigned char *)needle;
	size_t j, k, l;
	if (m > n || !m || !n)
		return NULL;
	if (1 != m) {
		if (x[0] == x[1]) {
			k = 2;
			l = 1;
		} else {
			k = 1;
			l = 2;
		}
		j = 0;
		while (j <= n - m) {
			if (x[1] != y[j + 1]) {
				j += k;
			} else {
				if (!memcmp(x + 2, y + j + 2, m - 2)
				    && x[0] == y[j])
					return (void *)&y[j];
				j += l;
			}
		}
	} else
		do {
			if (*y == *x)
				return (void *)y;
			y++;
		} while (--n);

	return NULL;
}

static int isdigit_c(int c)
{
    return c >= '0' && c <= '9';
}

// 从 HTTP 响应中提取 content 字段（需确保输入是完整 JSON 串）
const char* extract_content_from_json(const char* json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        printf("JSON 解析失败！\n");
        return NULL;
    }
    const cJSON *choices = cJSON_GetObjectItem(root, "choices");
    if (!choices || !cJSON_IsArray(choices)) {
        printf("字段 'choices' 无效！\n");
        cJSON_Delete(root);
        return NULL;
    }
    const cJSON *first_choice = cJSON_GetArrayItem(choices, 0);
    if (!first_choice) {
        printf("choices[0] 不存在！\n");
        cJSON_Delete(root);
        return NULL;
    }
    const cJSON *message = cJSON_GetObjectItem(first_choice, "message");
    if (!message || !cJSON_IsObject(message)) {
        printf("message 字段无效！\n");
        cJSON_Delete(root);
        return NULL;
    }
    const cJSON *content = cJSON_GetObjectItem(message, "content");
    if (!content || !cJSON_IsString(content)) {
        printf("content 字段无效！\n");
        cJSON_Delete(root);
        return NULL;
    }
    // 注意：content->valuestring 指向的内容仍归 cJSON 管理，使用后需要复制出来
    const char *result = content->valuestring;
    char *copy = strdup(result);  // 建议复制一份，避免 root 删除后失效
    cJSON_Delete(root);
    return copy;
}

/* parm server(in):       hostname or ip address of https server
 * parm port(in):         https server port
 * parm request(in):      https request
 * parm req_len(in):      https request len(excluding '\0')
 * parm response(out):    https response(excluding http header)
 * parm res_len(in, out): pass in addr of A, A stores capacity of buffer response
 *                        pass out https response body length(ie no http header)
 * out:                   HTTP status code or BL_HTTPSC error code
 */
int https_request(const char *server, uint16_t port, const uint8_t *request, int req_len, char *content_buf, int content_buf_len, int *res_len) {
    int ret_val = BL_HTTPSC_OK;
    uint8_t *rcv_buf = NULL;
    int buf_sz = 16 * 1024;
    int32_t fd, ret, send_ret, rcv_ret;
    int resp_write_off = 0;
    int flag = 0;
    /* 1. allocate buf size */
    /*------------------------------------------*/
    rcv_buf = malloc(buf_sz);
    if (!rcv_buf) {
        printf("Memory allocation failed for receive buffer.\n");
        return BL_HTTPSC_RET_ERR_MEM;
    }
    memset(rcv_buf, 0, buf_sz);
    /*------------------------------------------*/
    /* 2. TCP/SSL connection*/
    /*------------------------------------------*/
    fd = blTcpSslConnect(server, port);
    printf("Bouffalo Connect fd = 0x%08lx\r\n", fd);
    if (fd == BL_TCP_CREATE_CONNECT_ERR) {
        printf("Failed to connect to server.\n");
        ret_val = BL_HTTPSC_RET_ERR;
        goto exit;
    } 
    /*------------------------------------------*/
    do {
        ret = blTcpSslState(fd);
        if (ret == BL_TCP_CONNECTING) {
            continue;
        } else if (ret != BL_TCP_NO_ERROR) {
            printf("SSL connection error: %d\n", ret);
            ret_val = BL_HTTPSC_RET_ERR;
            goto exit;
        }
    } while (ret == BL_TCP_CONNECTING); 
    /*------------------------------------------*/
    send_ret = blTcpSslSend(fd, request, req_len);
    if (send_ret <= 0) {
        printf("Failed to send request.\n");
        ret_val = BL_HTTPSC_RET_ERR;
        goto exit;
    }
    vTaskDelay(100);
    if (ret == BL_TCP_NO_ERROR) {
        send_ret = blTcpSslSend(fd, request, req_len);
        if (send_ret > 0) {
            while(1) {
                /*------------------------------------------*/
                rcv_ret = blTcpSslRead(fd, rcv_buf, buf_sz);
                if (rcv_ret>0) { 
                  printf("\rflag:%d\n", flag);
                  flag = flag + 1;
                } else { }
                // Here is the code for receiving the CONTENT from the struct
                if (flag >= 3) 
                {
                    printf("rcv_ret = %ld\n", rcv_ret);
                    /*------------------------------------------*/
                    printf("\r=======Received Data=======\n");
                    for (int i = 0; i < rcv_ret; i++) {
                        printf("%c", rcv_buf[i]);
                    }
                    printf("\n");
                    /*------------------------------------------*/
                    char* content = extract_content_from_json(rcv_buf);
                    if (content) {
                        strncpy(content_buf, content, content_buf_len - 1);
                        free(content); // free the used RAM
                        ret_val = BL_HTTPSC_OK;
                        break;
                    }
                    else {
                        printf("\r============================\n");
                        printf("\rContent: %s\n", content);
                        printf("\r============================\n");
                        free(content);  // free the used RAM
                        break;
                    }
                } 
                else if (rcv_ret < 0) {
                    printf("\rrcv_ret = %ld\n", rcv_ret);
                    ret_val = BL_HTTPSC_RET_ERR;
                    break;
                }
            }
        } else {
            printf("\rssl tcp send data failed\n");
        }
    }
exit:
    blTcpSslDisconnect(fd);
    *res_len = resp_write_off;
    free(rcv_buf);
    return ret_val, rcv_ret;
}

static void test_chatgpt_request(void *param, char *reply)
{
    const char *host = "api.chatanywhere.tech";
    const char *api_key = "sk-KYj3AomcRm9pkBjNYyIwL2AkiawBoji6RTjrXSfyozhmvKg0"; // api Key2
    // const char *api_key = "sk-KYj3AomcRm9pkBjNYyIwL2AkiawBoji6RTjrXSfyozhmvKg0";
    /*---------------------------------------------------------*/
    // 检查一下传入的 prompt 内容
    const char *prompt = (const char *)param;
    if (prompt == NULL || strlen(prompt) == 0) {
        printf("\rPrompt is NULL or empty!\r\n");
        vTaskDelete(NULL);
        return;
    }
    printf("Using prompt: %s\r\n", prompt);
    /*---------------------------------------------------------*/
    /*---------------------------------------------------------*/
    char json_body[256];
    char send_buf[1024];
    // 拼接 AI 的请求内容
    snprintf(json_body, sizeof(json_body),
            "{"
            "\"model\": \"gpt-4o\","
            "\"messages\": ["
            "    {\"role\": \"user\", \"content\": \"%s\"}"
            "],"
            "\"temperature\": 0.7"
            "}", prompt);
    printf("Request body: %s\r\n", json_body);
    // 构建 HTTP 请求
    snprintf(send_buf, sizeof(send_buf),
            "POST /v1/chat/completions HTTP/1.1\r\n" //注意这里的接口填法
            "Host: %s\r\n"
            "Authorization: Bearer %s\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "\r\n"
            "%s",
            host, api_key, (int)strlen(json_body), json_body);
    // AI 的请求内容放在了最后
    /*---------------------------------------------------------*/
    /*---------------------------------------------------------*/
    // 响应缓冲区
    uint8_t *resp_buf = malloc(16 * 1024);
    if (!resp_buf) {
        LOG_E("malloc failed\r\n");
        return;
    }
    int resp_len = 16 * 1024;
    /*---------------------------------------------------------*/
    /*---------------------------------------------------------*/
    uint32_t start_time = bflb_mtimer_get_time_us();
    // 发送 HTTPS 请求
    // 在https_request函数里面会做主要的接收处理
    printf("\rsize:%d\n",sizeof(resp_buf));
    int status_code, length = https_request(host, 443, (uint8_t *)send_buf, strlen(send_buf), resp_buf, resp_len, &resp_len);
    uint32_t stop_time = bflb_mtimer_get_time_us();
    printf("total time: %d us\r\n", stop_time - start_time);
    printf("status_code: %d, resp_len: %d\r\n", status_code, resp_len);
    /*---------------------------------------------------------*/
    /*---------------------------------------------------------*/
    printf("\r============================\n");
    printf("\r%s\n", resp_buf);
    strncpy(reply, (char *)resp_buf, 4095);
    printf("\r============================\n");
    /*---------------------------------------------------------*/
    free(resp_buf); // free the used RAM
}

/* voice request */
int voice_https_request(const char *server, uint16_t port, const uint8_t *request, int req_len, char *content_buf, int content_buf_len, int *res_len)
{
    int count = 0;
    int ret_val = BL_HTTPSC_OK;
    uint8_t *rcv_buf = NULL;
    int buf_sz = 32 * 1024;
    int psram_offset = 0;
    int32_t fd, ret, send_ret, rcv_ret;
    int http_head_proced = 0;
    int status_code = BL_HTTPSC_RET_ERR;
    int write_len;
    int flag = 0;
    int loop_count = 0;
    int head_flag = 0;
    int head_status = 0;
    /* 1. allocate buf size */
    /*------------------------------------------*/
    rcv_buf = malloc(buf_sz);
    if (rcv_buf == NULL) {
        free(rcv_buf);
        printf("\rrcv_buf do not have space\n");
        return BL_HTTPSC_RET_ERR_MEM;
    }
    memset(rcv_buf, 0, buf_sz);
    /*------------------------------------------*/
    /* 2. TCP/SSL connection*/
    /*------------------------------------------*/
    fd = blTcpSslConnect(server, port);
    printf("Bouffalo Connect fd = 0x%08lx\r\n", fd);
    if (fd == BL_TCP_CREATE_CONNECT_ERR) {
        printf("\rssl connect error\n");
        ret_val = BL_HTTPSC_RET_ERR;
        goto exit;
    }
    /*------------------------------------------*/
    do {
        ret = blTcpSslState(fd);
        if (ret == BL_TCP_CONNECTING) {
            continue;
        } else if (ret != BL_TCP_NO_ERROR) {
            printf("SSL connection error: %d\n", ret);
            ret_val = BL_HTTPSC_RET_ERR;
            goto exit;
        }
    } while (ret == BL_TCP_CONNECTING); 
    /*------------------------------------------*/
    if (ret == BL_TCP_NO_ERROR) {
        send_ret = blTcpSslSend(fd, request, req_len);
        printf("\r send_ret %d \n", send_ret);
        // vTaskDelay(100);
        if (send_ret > 0) {
            /* Init the status varible */
            head_flag = 0; head_status = 0;
            while(1) {
                /*------------------------------------------*/
                    vTaskDelay(100);
                    rcv_ret = blTcpSslRead(fd, rcv_buf, buf_sz);
                    printf("\rdata length = %d \n", rcv_ret);
                    printf("\r%d \n", loop_count);
                    head_status = rcv_ret;
                    /* status judgement */
                    if(head_flag == 0 && head_status == 0){
                        head_flag = 0; /* 1.nothing received */
                    }else if(head_flag ==0 && head_status > 0){
                        head_flag = 1; /* 2.head reveiced */
                    }else if(head_flag == 1 && head_status > 0){
                        head_flag = 2; /* 3.data received */
                    }else if(head_flag == 2 && head_status > 0){
                        head_flag = 2; /* 4.continue to receive the data */
                    }else if(head_flag == 2 && head_status == 0){
                        head_flag = 3; /* 5.transfer finished */
                    }
                    /* deal with the situations */
                    if (head_flag == 3){
                        printf("\rData transfer finished! \n");
                        goto exit;
                    }else{
                        switch (head_flag) {
                            case 0:
                                if(loop_count <= 100){
                                   printf("\rNothing received! \n");
                                }else if (loop_count > 100){
                                   printf("\rConnection lost! \n");
                                   goto exit;
                                }
                            break;

                            case 1:
                                /* Send ACK signal to server */
                                ret = blTcpSslState(fd);
                                /* print the frame head */
                                printf("\r ret = %d \n",ret);
                                // for (int i = 0; i < rcv_ret; i++){
                                //    printf("%c", rcv_buf[i]);
                                // }
                                printf("\n");
                            break;

                            case 2:
                                /* Send ACK signal to server */
                                ret = blTcpSslState(fd);
                                /* Store the mp3 data into the content_buf */
                                if (psram_offset + rcv_ret < content_buf_len) {
                                    printf("\r Store the data into the content_buf \n");
                                    memcpy(content_buf + psram_offset, rcv_buf, rcv_ret);
                                    psram_offset += rcv_ret;
                                } else if (psram_offset + rcv_ret >= content_buf_len) {
                                    printf("\r content_buf is not enough! \n");
                                    ret_val = BL_HTTPSC_RET_ERR;
                                    goto exit;
                                }
                                /* print the mp3 data on screen */
                                printf("\r ret = %d \n",ret);
                                // for (int i = 0; i < rcv_ret; i++){
                                //    printf("%02x", rcv_buf[i]);
                                // }
                                printf("\n");
                            break;
                        
                            default:
                            break;
                        }
                    }
                    loop_count = loop_count + 1;
                /*------------------------------------------*/
            }//while(1)
        }
    }
exit:
    /* TCP disconnected */
    printf("\n");
    blTcpSslDisconnect(fd);
    *res_len = psram_offset; // 返回接收到的mp3数据总长度
    free(rcv_buf);
    return ret_val;
}

int voiceAssist_request(void *param)
{
    const char *host = "api.chatanywhere.tech";
    const char *api_key = "sk-KYj3AomcRm9pkBjNYyIwL2AkiawBoji6RTjrXSfyozhmvKg0"; // api Key2
    /*---------------------------------------------------------*/
    // 检查一下传入的 prompt 内容
    const char *prompt = (const char *)param;
    if (prompt == NULL || strlen(prompt) == 0) {
        printf("\rPrompt is NULL or empty!\r\n");
        vTaskDelete(NULL);
        return;
    }
    printf("Using prompt: %s\r\n", prompt);
    /*---------------------------------------------------------*/
    /*---------------------------------------------------------*/
    // 拼接 AI 的请求内容
    char json_body[512];
    snprintf(json_body, sizeof(json_body),
        "{"
        "\"model\": \"tts-1\","
        "\"input\": \"%s\","
        "\"voice\": \"nova\""
        "}", prompt);  // 合成的文本
    printf("\r Request body: %s\r\n", json_body);
    // 构建 HTTP 请求
    char send_buf[2048];
    snprintf(send_buf, sizeof(send_buf),
        "POST /v1/audio/speech HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: Bearer %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "\r\n"
        "%s",
        host, api_key, (int)strlen(json_body), json_body);
    printf("\r Request body: %s\r\n", send_buf);
    /*---------------------------------------------------------*/
    int resp_len = 512 * 1024; // length of pcm_data_rx
    int mp3_len = 0; // length of mp3 data
    /*---------------------------------------------------------*/
    /*---------------------------------------------------------*/
    // 发送 HTTPS 请求
    // 在https_request函数里面会做主要的接收处理
    printf("\r The mp3 buf size is:%d \n", 512 * 1024);
    int status_code = voice_https_request(host, 443, (uint8_t *)send_buf, strlen(send_buf), psram_mp3_buf, resp_len, &mp3_len);
    printf("status_code: %d, mp3_len: %d\r\n", status_code, mp3_len);
    return status_code, mp3_len; // 返回状态码 和接收到的mp3数据总长度
}

extern uint8_t audio_played_flag;
extern TaskHandle_t mqtt_Task_Handler;
static void test_https_entry(void *param)
{
    // vTaskSuspend(mqtt_Task_Handler);
    /*---------------------------------------------------------*/
    // 响应缓冲区
    uint8_t *reply_content = malloc(4 * 1024);
    if (!reply_content) {
        LOG_E("malloc failed\r\n");
        return;
    }
    int reply_len = 4* 1024;
    /*---------------------------------------------------------*/
    const char *prompt = (const char *)param;
    printf("\r============================\n");
    printf("\rReceived prompt: %s\n", prompt);
    printf("\r============================\n");
    test_chatgpt_request(prompt, reply_content);
    printf("\rTransmit prompt: %s\n", reply_content);

    int ret, mp3_file_length = 0;
    ret, mp3_file_length = voiceAssist_request(reply_content);
    printf("\rThe MP3 file size is %dkB,%d\n", mp3_file_length/1024,mp3_file_length);

    if (ret == BL_HTTPSC_OK && mp3_file_length > 0){
      audio_played_flag = 1; // 设置音频播放标志
      printf("\rStart mp3 decode, len=%d\n", mp3_file_length);
      decode_mp3_buffer(psram_mp3_buf, mp3_file_length);
    }

    free(reply_content); // free the used PSRAM
    // vTaskResume(mqtt_Task_Handler);
    vTaskDelete(NULL);
}

#ifdef CONFIG_SHELL
#include <shell.h>

extern uint32_t wifi_state;
static int check_wifi_state(void) {
    if (wifi_state == 1) {
        return 0;
    } else {
        return 1;
    }
}

int cmd_https_test(int argc, char **argv)
{
    uint32_t ret = 0;
    ret = check_wifi_state();
    if (ret != 0) {
        printf("your wifi not connected!\r\n");
        return 0;
    }
    if (argc < 2) {
        printf("Usage: ai_chat <prompt>\r\n");
        return 0;
    }
    // 使用 static 避免 prompt 内容在 xTaskCreate 后失效
    static char prompt_buffer[512] = {0};
    prompt_buffer[0] = '\0';
    for (int i = 1; i < argc; i++) {
        strncat(prompt_buffer, argv[i], sizeof(prompt_buffer) - strlen(prompt_buffer) - 1);
        if (i < argc - 1) {
            strncat(prompt_buffer, " ", sizeof(prompt_buffer) - strlen(prompt_buffer) - 1);
        }
    }
    printf("Creating task with prompt: %s\r\n", prompt_buffer);
    xTaskCreate(test_https_entry, "test_https", 8192, prompt_buffer, configMAX_PRIORITIES -8, NULL);
    return 0;
}
/*---------------------------------------------------------*/
/*---------------------------------------------------------*/
const int proc_http_head(const uint8_t *buf, int buf_len, int *status_code, int *body_start_off)
{
    char status_code_buf[4] = {0};
    void *head_end_pos = NULL;
    if (buf_len < 16) { // "HTTP/1.1 XYZ\r\n\r\n"
        return BL_HTTPSC_RET_HTTP_ERR;
    }
    if (!((memcmp(buf, "HTTP/1.0 ", 9) == 0) || (0 == memcmp(buf, "HTTP/1.1 ", 9)))) {
        return BL_HTTPSC_RET_HTTP_ERR;
    }
    memcpy(status_code_buf, buf + 9, 3);
    if (!(isdigit_c(status_code_buf[0]) && isdigit_c(status_code_buf[1]) && isdigit_c(status_code_buf[2]))) {
        return BL_HTTPSC_RET_HTTP_ERR;
    }
    *status_code = atoi(status_code_buf);
    head_end_pos = memmem(buf, buf_len, "\r\n\r\n", 4);
    if (!head_end_pos) {
        return BL_HTTPSC_HEAD_END_NOT_FOUND;
    }
    *body_start_off = (uint8_t *)head_end_pos - buf + 4;
    return BL_HTTPSC_OK;
}

int https_request_get_time(const char *server, uint16_t port, const uint8_t *request, int req_len, uint8_t *response, int *res_len)
{
    int ret_val = BL_HTTPSC_OK;
    uint8_t *rcv_buf = NULL;
    int buf_sz = 16 * 1024;
    int32_t fd, ret, send_ret, rcv_ret = 0;
    int resp_write_off = 0;
    int http_head_proced = 0;
    int status_code = BL_HTTPSC_RET_ERR;
    int write_len;
    /* 1. allocate buf size */
    /*------------------------------------------*/
    rcv_buf = malloc(buf_sz);
    if (rcv_buf == NULL) {
        free(rcv_buf);
        printf("Memory allocation failed for receive buffer.\n");
        return BL_HTTPSC_RET_ERR_MEM;
    }
    memset(rcv_buf, 0, buf_sz);
    /* 2. TCP/SSL connection*/
    /*------------------------------------------*/
    fd = blTcpSslConnect_get_time(server, port);
    printf("Bouffalo Connect fd = 0x%08lx\r\n", fd);
    if (fd == BL_TCP_CREATE_CONNECT_ERR) {
        printf("Failed to connect to server.\n");
        ret_val = BL_HTTPSC_RET_ERR;
        goto exit;
    } 
    /*------------------------------------------*/
    do {
        ret = blTcpSslState(fd);
        if (ret == BL_TCP_CONNECTING) {
            continue;
        } else if (ret != BL_TCP_NO_ERROR) {
            printf("SSL connection error: %d\n", ret);
            ret_val = BL_HTTPSC_RET_ERR;
            goto exit;
        }
    } while (ret == BL_TCP_CONNECTING); 
    /*------------------------------------------*/
    send_ret = blTcpSslSend(fd, request, req_len);
    if (send_ret <= 0) {
        printf("Failed to send request.\n");
        ret_val = BL_HTTPSC_RET_ERR;
        goto exit;
    }
    vTaskDelay(100);
    while (1) {
        while (rcv_ret <= 0) {
            rcv_ret = blTcpSslRead(fd, rcv_buf, buf_sz);
            if (rcv_ret > 0) {
                rcv_buf[rcv_ret] = '\0'; // Null terminate the buffer for printing
            }
        }
        printf("---------------------------\r\n");
        printf("%s", rcv_buf);
        printf("---------------------------\r\n");
        if (!http_head_proced) {
            int body_start_off = -1;
            int proc_head_r = proc_http_head(rcv_buf, rcv_ret, &status_code, &body_start_off);
            printf("proc_head_r %d, status_code %d, body_start_off %d\r\n", proc_head_r, status_code, body_start_off);
            if (proc_head_r == BL_HTTPSC_OK) {
                write_len = rcv_ret - body_start_off;
                if (write_len + resp_write_off > *res_len) {
                    ret_val = BL_HTTPSC_RET_ERR_BUF_TOO_SMALL;
                    break;
                }
                http_head_proced = 1;
                memcpy(response, rcv_buf + body_start_off, write_len);
                resp_write_off += write_len;
            }
        } else {
            write_len = rcv_ret;
            if (write_len + resp_write_off > *res_len) {
                ret_val = BL_HTTPSC_RET_ERR_BUF_TOO_SMALL;
                break;
            }
            printf("Copy to resp @off %d, len %d, 1st char %02X\r\n", resp_write_off, write_len, *rcv_buf);
            memcpy(response + resp_write_off, rcv_buf, write_len);
            resp_write_off += write_len;
        }
        ret_val = status_code;
        break;
    }
exit:
    blTcpSslDisconnect(fd);
    *res_len = resp_write_off;
    free(rcv_buf);
    return ret_val;
}
extern uint8_t *buf_time_data[20];
extern uint32_t time_get_flag;
extern uint32_t year, month, day, hour, minute, second;
extern void parse_datetime(uint8_t *datetime_str, uint32_t *year, uint32_t *month, uint32_t *day, uint32_t *hour, uint32_t *minute, uint32_t *second);
static void https_get_time(void *param)
{
    const char *host = "timezone.market.alicloudapi.com";
    const char *appcode = "32a1e5a1856641d884b7765684f5f3a9"; // 替换为你的实际AppCode
    const char *city = "%E5%8C%97%E4%BA%AC";
    // 构造GET请求行和头部信息
    char send_buf[512] = {0}; // 确保这个数组足够大以容纳整个请求
    snprintf(send_buf, sizeof(send_buf) - 1,
             "GET /timezone?city=%s HTTP/1.1\r\n"  // 这里用“北京”的URL编码作为示例
             "Host: %s\r\n"
             "User-Agent: curl/8.9.1\r\n"
             "Accept: */*\r\n"
             "Authorization: APPCODE %s\r\n"
             "\r\n", city, host, appcode);
    printf("Sending request:\n%s\n", send_buf);
    uint8_t *buf = malloc(16 * 1024);
    if (NULL == buf) {
        printf("malloc failed\n");
        return;
    }
    int resp_len = 16 * 1024;

    int status_code;
    status_code = https_request_get_time(host, 443, (uint8_t *)send_buf, strlen(send_buf), buf, &resp_len);

    if (status_code == BL_HTTPSC_OK_get_time) {
        printf("HTTP Status Code: 200 OK\n");
        printf("Response Body:\n%s\n", buf);
    } else {
        printf("HTTP Request Failed with code: %d\n", status_code);
    }
    // 提取data_now
    const char *search_str = "\"date_now\":\"";
    char *start_pos = strstr(buf, search_str);
    if (start_pos == NULL) {
        printf("No time data.");
    }
    // 跳过 "date_now":" 的长度，找到实际数据开始的位置
    start_pos += strlen(search_str);
    // 查找下一个双引号，即 date_now 值结束的位置
    char *end_pos = strchr(start_pos, '"');
    // 计算 date_now 值的长度
    size_t date_now_len = end_pos - start_pos;
    // 分配内存用于存储 date_now 的值，并复制字符串
    char *result = (char*)malloc(date_now_len + 1); // +1 for null terminator
    if (result == NULL) { // 检查内存分配是否成功
        perror("malloc failed");
    }
    // 复制 date_now 字段的值
    strncpy(result, start_pos, date_now_len);
    result[date_now_len] = '\0'; // 确保字符串以 null 结尾
    printf("%s\n", result); // 打印提取出的 date_now 值
    memset(buf_time_data, '\0', sizeof(buf_time_data));
    strcpy(buf_time_data, result);
    printf("%s\n", buf_time_data);
    parse_datetime(buf_time_data, &year, &month, &day, &hour, &minute, &second);
    time_get_flag = 0;
    free(result);
    free(buf);
    vTaskDelete(NULL);
}

int cmd_get_time() {
    uint32_t ret = 0;
    ret = check_wifi_state();
    if (ret != 0) {
        printf("your wifi not connected!\r\n");
        return 0;
    }
    xTaskCreate(https_get_time, "get time", 2048, NULL, configMAX_PRIORITIES - 10, NULL);
    return 0;
}
SHELL_CMD_EXPORT_ALIAS(cmd_https_test, ai_chat, wifi https client test);
SHELL_CMD_EXPORT_ALIAS(cmd_get_time, get_time, wifi https get time);
#endif

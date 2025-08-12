#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>

#include <lwip/sockets.h>
#include <lwip/tcp.h>
#include <lwip/err.h>
#include <lwip/dns.h>
#include <lwip/netdb.h>

#include <mbedtls/net.h>
#include <mbedtls/debug.h>
#include <mbedtls/ssl.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/error.h>
#include <mbedtls/certs.h>

#include "bl_error.h"
#include "https.h"

#define DBG_TAG "HTTP/S"
#include "log.h"


#if defined(BL_VERIFY)
const char bl_test_cli_key_rsa[] =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIFgTCCBGmgAwIBAgIQOXJEOvkit1HX02wQ3TE1lTANBgkqhkiG9w0BAQwFADB7\r\n"
"MQswCQYDVQQGEwJHQjEbMBkGA1UECAwSR3JlYXRlciBNYW5jaGVzdGVyMRAwDgYD\r\n"
"VQQHDAdTYWxmb3JkMRowGAYDVQQKDBFDb21vZG8gQ0EgTGltaXRlZDEhMB8GA1UE\r\n"
"AwwYQUFBIENlcnRpZmljYXRlIFNlcnZpY2VzMB4XDTE5MDMxMjAwMDAwMFoXDTI4\r\n"
"MTIzMTIzNTk1OVowgYgxCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpOZXcgSmVyc2V5\r\n"
"MRQwEgYDVQQHEwtKZXJzZXkgQ2l0eTEeMBwGA1UEChMVVGhlIFVTRVJUUlVTVCBO\r\n"
"ZXR3b3JrMS4wLAYDVQQDEyVVU0VSVHJ1c3QgUlNBIENlcnRpZmljYXRpb24gQXV0\r\n"
"aG9yaXR5MIICIjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAgBJlFzYOw9sI\r\n"
"s9CsVw127c0n00ytUINh4qogTQktZAnczomfzD2p7PbPwdzx07HWezcoEStH2jnG\r\n"
"vDoZtF+mvX2do2NCtnbyqTsrkfjib9DsFiCQCT7i6HTJGLSR1GJk23+jBvGIGGqQ\r\n"
"Ijy8/hPwhxR79uQfjtTkUcYRZ0YIUcuGFFQ/vDP+fmyc/xadGL1RjjWmp2bIcmfb\r\n"
"IWax1Jt4A8BQOujM8Ny8nkz+rwWWNR9XWrf/zvk9tyy29lTdyOcSOk2uTIq3XJq0\r\n"
"tyA9yn8iNK5+O2hmAUTnAU5GU5szYPeUvlM3kHND8zLDU+/bqv50TmnHa4xgk97E\r\n"
"xwzf4TKuzJM7UXiVZ4vuPVb+DNBpDxsP8yUmazNt925H+nND5X4OpWaxKXwyhGNV\r\n"
"icQNwZNUMBkTrNN9N6frXTpsNVzbQdcS2qlJC9/YgIoJk2KOtWbPJYjNhLixP6Q5\r\n"
"D9kCnusSTJV882sFqV4Wg8y4Z+LoE53MW4LTTLPtW//e5XOsIzstAL81VXQJSdhJ\r\n"
"WBp/kjbmUZIO8yZ9HE0XvMnsQybQv0FfQKlERPSZ51eHnlAfV1SoPv10Yy+xUGUJ\r\n"
"5lhCLkMaTLTwJUdZ+gQek9QmRkpQgbLevni3/GcV4clXhB4PY9bpYrrWX1Uu6lzG\r\n"
"KAgEJTm4Diup8kyXHAc/DVL17e8vgg8CAwEAAaOB8jCB7zAfBgNVHSMEGDAWgBSg\r\n"
"EQojPpbxB+zirynvgqV/0DCktDAdBgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rID\r\n"
"ZsswDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQFMAMBAf8wEQYDVR0gBAowCDAG\r\n"
"BgRVHSAAMEMGA1UdHwQ8MDowOKA2oDSGMmh0dHA6Ly9jcmwuY29tb2RvY2EuY29t\r\n"
"L0FBQUNlcnRpZmljYXRlU2VydmljZXMuY3JsMDQGCCsGAQUFBwEBBCgwJjAkBggr\r\n"
"BgEFBQcwAYYYaHR0cDovL29jc3AuY29tb2RvY2EuY29tMA0GCSqGSIb3DQEBDAUA\r\n"
"A4IBAQAYh1HcdCE9nIrgJ7cz0C7M7PDmy14R3iJvm3WOnnL+5Nb+qh+cli3vA0p+\r\n"
"rvSNb3I8QzvAP+u431yqqcau8vzY7qN7Q/aGNnwU4M309z/+3ri0ivCRlv79Q2R+\r\n"
"/czSAaF9ffgZGclCKxO/WIu6pKJmBHaIkU4MiRTOok3JMrO66BQavHHxW/BBC5gA\r\n"
"CiIDEOUMsfnNkjcZ7Tvx5Dq2+UUTJnWvu6rvP3t3O9LEApE9GQDTF1w52z97GA1F\r\n"
"zZOFli9d31kWTz9RvdVFGD/tSo7oBmF0Ixa1DVBzJ0RHfxBdiSprhTEUxOipakyA\r\n"
"vGp4z7h/jnZymQyd/teRCBaho1+V\r\n"
"-----END CERTIFICATE-----\r\n";
const int32_t bl_test_cas_pem_len = sizeof(bl_test_cli_key_rsa);

const char bl_test_cli_key_rsa_get_time[] =
"-----BEGIN CERTIFICATE-----\r\n"
"MIIG0zCCBbugAwIBAgIMZ/sfItri/YU2g9ifMA0GCSqGSIb3DQEBCwUAMFMxCzAJ\r\n"
"BgNVBAYTAkJFMRkwFwYDVQQKExBHbG9iYWxTaWduIG52LXNhMSkwJwYDVQQDEyBH\r\n"
"bG9iYWxTaWduIEdDQyBSMyBPViBUTFMgQ0EgMjAyNDAeFw0yNTAzMTEwODI3MDFa\r\n"
"Fw0yNTA5MDQwMDAwMDBaMIGFMQswCQYDVQQGEwJDTjERMA8GA1UECBMIWmhlSmlh\r\n"
"bmcxETAPBgNVBAcTCEhhbmdaaG91MS0wKwYDVQQKEyRBbGliYWJhIChDaGluYSk\r\n"
"VGVjaG5vbG9neSBDby4sIEx0ZC4xITAfBgNVBAMMGCoubWFya2V0LmFsaWNsb3Vk\r\n"
"YXBpLmNvbTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAK7On6yr9FGk\r\n"
"a7TEF+1hOZbG+Df0DQ3rFcBkeD7YsL5ZSAADNHeODW4Cx3CExOZIXOzoyu4g8G4f\r\n"
"iFbXax7aZIjEIa/D3KBImxIOzJGy+D8iqn7oq5etKh8kfL+tj6aFSs31+vFIZbm6\r\n"
"scnnkCtAMvPBLERUG4j1YTFjzoeIIR7tRmllEQ3DZCj9CNT7ewFGg65ayzlWUx5z\r\n"
"26v8/k1n48jS72A6S/H/ZKtJVUNpu4fm/sdcx0c7QeaE6+hXvp1RD31s3u1zf7aO\r\n"
"fmgCazC2GfDIHQPou0dPQ5L/jIKGbQR4bkO2hVdZGr9IYnGlKwxbDvK5A0jlO8vB\r\n"
"4iGI3N3PqecCAwEAAaOCA3IwggNuMA4GA1UdDwEB/wQEAwIFoDAMBgNVHRMBAf8E\r\n"
"AjAAMIGTBggrBgEFBQcBAQSBhjCBgzBGBggrBgEFBQcwAoY6aHR0cDovL3NlY3Vy\r\n"
"ZS5nbG9iYWxzaWduLmNvbS9jYWNlcnQvZ3NnY2NyM292dGxzY2EyMDI0LmNydDA5\r\n"
"BggrBgEFBQcwAYYtaHR0cDovL29jc3AuZ2xvYmFsc2lnbi5jb20vZ3NnY2NyM292\r\n"
"dGxzY2EyMDI0MFcGA1UdIARQME4wCAYGZ4EMAQICMEIGCisGAQQBoDIKAQIwNDAy\r\n"
"BggrBgEFBQcCARYmaHR0cHM6Ly93d3cuZ2xvYmFsc2lnbi5jb20vcmVwb3NpdG9y\r\n"
"eS8wQQYDVR0fBDowODA2oDSgMoYwaHR0cDovL2NybC5nbG9iYWxzaWduLmNvbS9n\r\n"
"c2djY3Izb3Z0bHNjYTIwMjQuY3JsMDsGA1UdEQQ0MDKCGCoubWFya2V0LmFsaWNs\r\n"
"b3VkYXBpLmNvbYIWbWFya2V0LmFsaWNsb3VkYXBpLmNvbTAdBgNVHSUEFjAUBggr\r\n"
"BgEFBQcDAQYIKwYBBQUHAwIwHwYDVR0jBBgwFoAU2tOoCEgMNDdY7uWndS5Z/Nbc\r\n"
"PDgwHQYDVR0OBBYEFGjdzIr8Vm/wum3H2bEHDYZvv/WIMIIBfgYKKwYBBAHWeQIE\r\n"
"AgSCAW4EggFqAWgAdQCvGBoo1oyj4KmKTJxnqwn4u7wiuq68sTijoZ3T+bYDDQAA\r\n"
"AZWEUIzEAAAEAwBGMEQCIDUnkx6YWgK9ZzduBGsKsMRnQ9bKgquGGfmRIZd2c0lG\r\n"
"AiAGTgf4n65EkoMefOcjZ1WRxLqrUmKlIDU45pCpKwq2YAB2ABLxTjS9U3JMhAYZ\r\n"
"w48/ehP457Vih4icbTAFhOvlhiY6AAABlYRQjZ8AAAQDAEcwRQIgfuCgzYhAyQKe\r\n"
"1mIuc3ks6BEIs0FxdkWg8C1I2bG6OsQCIQC6sNvKFKZmHKcd832Zyryc+6utl7vK\r\n"
"pxMNCN47Wg187QB3AA3h8jAr0w3BQGISCepVLvxHdHyx1+kw7w5CHrR+Tqo0AAAB\r\n"
"lYRQiqUAAAQDAEgwRgIhAOdzk4APSMTO9oITpcfI8HzKocfcQbXwrimvjchRW0nL\r\n"
"AiEAhHLTa6tFdHI9fQknlcB5Td69kQn79Mv/0imS92Hu6jswDQYJKoZIhvcNAQEL\r\n"
"BQADggEBAATOLjF481EtNrzz761LJWA8L6vPp8c9tbpNJn+OIY/txe5Ai83kxsoE\r\n"
"gSxuEpFD/TgHyTguGlMFss9W4pF3ahC1+YOOp2XlXwFroRBNRVj72/LOu1B1nevS\r\n"
"INjDDRhk1tWiO/rr/UkISGPSSp6EzsOqTzsioJ8mreVhuiwIF8BrsuvZQm1Vecv9\r\n"
"nU9tXiqyXjcqbaHmwBXUB+SSsart/vv3YEjV3qIduJR8lg5+uHBd4hbZ4NKCskfs\r\n"
"oInOk5ImEtA2s4W2P3CL+NKG9bmiK8kyA8KR6HE7CPz+UNmXAECtcca86AGO2rYy\r\n"
"etIimMFJBpZs4ptu29ERTlmfPcYM1co=\r\n"
"-----END CERTIFICATE-----\r\n";
const int32_t bl_test_cas_pem_len_get_time = sizeof(bl_test_cli_key_rsa_get_time);

#endif

typedef struct _https_context {
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_net_context server_fd;
#if defined(BL_VERIFY)
    mbedtls_x509_crt cacert;
#endif
} https_context_t;

https_context_t *bl_hsbuf = NULL;

static void bl_debug( void *ctx, int level,
                      const char *file, int line,
                      const char *str )
{
    ((void) level);

    LOG_I("%s:%04d: %s\r\n", file, line, str);
}

static int is_valid_ip_address(const char *ipAddress)
{
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ipAddress, &(sa.sin_addr));

    return result != 0;
}


int32_t blTcpSslConnect_get_time(const char *dst, uint16_t port)
{
    in_addr_t dst_addr;
    int ret;

    struct sockaddr_in servaddr;
    int flags;
    int reuse = 1;

    if (NULL != bl_hsbuf) {
        return -1;
    }
    bl_hsbuf = malloc(sizeof(https_context_t));
    if (NULL == bl_hsbuf) {
        return -1;
    }
    if (NULL == dst) {
        return BL_TCP_ARG_INVALID;
    }

    if (is_valid_ip_address(dst)) {
        dst_addr = inet_addr(dst);
    } else {
        struct hostent *hostinfo = gethostbyname(dst);
        if (!hostinfo) {
            return -1;
        }
        dst_addr = ((struct in_addr *) hostinfo->h_addr)->s_addr;
        printf("dst_addr is %08lX\n", *(uint32_t *)&dst_addr);
    }

    mbedtls_ssl_init(&bl_hsbuf->ssl);

#if defined(BL_VERIFY)
    mbedtls_x509_crt_init( &bl_hsbuf->cacert );
#endif

    mbedtls_ctr_drbg_init(&bl_hsbuf->ctr_drbg);
    mbedtls_ssl_config_init(&bl_hsbuf->conf);
    mbedtls_entropy_init(&bl_hsbuf->entropy);

    if ((ret = mbedtls_ctr_drbg_seed(&bl_hsbuf->ctr_drbg, mbedtls_entropy_func, &bl_hsbuf->entropy,
                                     NULL, 0)) != 0) {
        return BL_TCP_CREATE_CONNECT_ERR;
    }

#if defined(BL_VERIFY)
    ret = mbedtls_x509_crt_parse( &bl_hsbuf->cacert, (const unsigned char *)bl_test_cli_key_rsa_get_time,
                                   bl_test_cas_pem_len_get_time );

    if (ret < 0) {
        printf("\r\n mbedtls_x509_crt_parse returned -0x%x\r\n", -ret);
    }
#endif

    if ((ret = mbedtls_ssl_config_defaults(&bl_hsbuf->conf,
                                           MBEDTLS_SSL_IS_CLIENT,
                                           MBEDTLS_SSL_TRANSPORT_STREAM,
                                           MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
        printf("mbedtls_ssl_config_defaults returned %d", ret);
        return BL_TCP_CREATE_CONNECT_ERR;
    }

#if defined(BL_VERIFY)
    mbedtls_ssl_conf_authmode(&bl_hsbuf->conf, MBEDTLS_SSL_VERIFY_REQUIRED/*MBEDTLS_SSL_VERIFY_OPTIONAL*/);
    mbedtls_ssl_conf_ca_chain( &bl_hsbuf->conf, &bl_hsbuf->cacert, NULL );
#else
    mbedtls_ssl_conf_authmode(&bl_hsbuf->conf, MBEDTLS_SSL_VERIFY_NONE);
#endif

    mbedtls_ssl_conf_rng(&bl_hsbuf->conf, mbedtls_ctr_drbg_random, &bl_hsbuf->ctr_drbg);

    //todo
    mbedtls_ssl_conf_read_timeout(&bl_hsbuf->conf, 0);
    //mbedtls_ssl_set_timer_cb(&ssl, &ssl_timer, f_set_timer, f_get_timer);

    mbedtls_ssl_conf_dbg( &bl_hsbuf->conf, bl_debug, stdout );

#if defined(MBEDTLS_DEBUG_C)
    // mbedtls_debug_set_threshold(4);
#endif

    if ((ret = mbedtls_ssl_setup(&bl_hsbuf->ssl, &bl_hsbuf->conf)) != 0) {
        printf("mbedtls_ssl_setup returned -0x%x\r\n", -ret);
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    mbedtls_net_init(&bl_hsbuf->server_fd);

    bl_hsbuf->server_fd.fd = socket(AF_INET, SOCK_STREAM, 0);

    if (bl_hsbuf->server_fd.fd < 0) {
        printf("ssl creat socket fd failed\r\n");
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    flags = fcntl(bl_hsbuf->server_fd.fd, F_GETFL, 0);
    if (flags < 0 || fcntl(bl_hsbuf->server_fd.fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        printf("ssl fcntl: %s\r\n", strerror(errno));
        close(bl_hsbuf->server_fd.fd);
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    if (setsockopt(bl_hsbuf->server_fd.fd, SOL_SOCKET, SO_REUSEADDR,
                   (const char*) &reuse, sizeof(reuse)) != 0) {
        close(bl_hsbuf->server_fd.fd);
        printf("ssl set SO_REUSEADDR failed\r\n");
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    memset(&servaddr, 0, sizeof(struct sockaddr_in));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = dst_addr;
    servaddr.sin_port = htons(port);

    if (connect(bl_hsbuf->server_fd.fd, (struct sockaddr*)&servaddr, sizeof(struct sockaddr_in)) == 0) {
        //printf("ssl dst %s errno %d\r\n", dst, errno);
    } else {
        //printf("ssl dst %s errno %d\r\n", dst, errno);
        if (errno == EINPROGRESS) {
            //printf("ssl tcp conncet noblock\r\n");
        } else {
            close(bl_hsbuf->server_fd.fd);
            return BL_TCP_CREATE_CONNECT_ERR;
        }
    }

    //todo
    //mbedtls_ssl_set_bio(&bl_hsbuf->ssl, &bl_hsbuf->server_fd, mbedtls_net_send, mbedtls_net_recv, mbedtls_net_recv_timeout); //noblock
    mbedtls_ssl_set_bio(&bl_hsbuf->ssl, &bl_hsbuf->server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);
    return (int32_t)&bl_hsbuf->ssl;
}


int32_t blTcpSslConnect(const char *dst, uint16_t port)
{
    in_addr_t dst_addr;
    int ret;

    struct sockaddr_in servaddr;
    int flags;
    int reuse = 1;

    if (NULL != bl_hsbuf) {
        return -1;
    }
    bl_hsbuf = malloc(sizeof(https_context_t));
    if (NULL == bl_hsbuf) {
        return -1;
    }
    if (NULL == dst) {
        return BL_TCP_ARG_INVALID;
    }

    if (is_valid_ip_address(dst)) {
        dst_addr = inet_addr(dst);
    } else {
        struct hostent *hostinfo = gethostbyname(dst);
        if (!hostinfo) {
            return -1;
        }
        dst_addr = ((struct in_addr *) hostinfo->h_addr)->s_addr;
        printf("dst_addr is %08lX\n", *(uint32_t *)&dst_addr);
    }

    mbedtls_ssl_init(&bl_hsbuf->ssl);

#if defined(BL_VERIFY)
    mbedtls_x509_crt_init( &bl_hsbuf->cacert );
#endif

    mbedtls_ctr_drbg_init(&bl_hsbuf->ctr_drbg);
    mbedtls_ssl_config_init(&bl_hsbuf->conf);
    mbedtls_entropy_init(&bl_hsbuf->entropy);

    if ((ret = mbedtls_ctr_drbg_seed(&bl_hsbuf->ctr_drbg, mbedtls_entropy_func, &bl_hsbuf->entropy,
                                     NULL, 0)) != 0) {
        return BL_TCP_CREATE_CONNECT_ERR;
    }

#if defined(BL_VERIFY)
    ret = mbedtls_x509_crt_parse( &bl_hsbuf->cacert, (const unsigned char *)bl_test_cli_key_rsa,
                                   bl_test_cas_pem_len );

    if (ret < 0) {
        printf("\r\n mbedtls_x509_crt_parse returned -0x%x\r\n", -ret);
    }
#endif

    if ((ret = mbedtls_ssl_config_defaults(&bl_hsbuf->conf,
                                           MBEDTLS_SSL_IS_CLIENT,
                                           MBEDTLS_SSL_TRANSPORT_STREAM,
                                           MBEDTLS_SSL_PRESET_DEFAULT)) != 0) {
        printf("mbedtls_ssl_config_defaults returned %d", ret);
        return BL_TCP_CREATE_CONNECT_ERR;
    }

#if defined(BL_VERIFY)
    mbedtls_ssl_conf_authmode(&bl_hsbuf->conf, MBEDTLS_SSL_VERIFY_REQUIRED/*MBEDTLS_SSL_VERIFY_OPTIONAL*/);
    mbedtls_ssl_conf_ca_chain( &bl_hsbuf->conf, &bl_hsbuf->cacert, NULL );
#else
    mbedtls_ssl_conf_authmode(&bl_hsbuf->conf, MBEDTLS_SSL_VERIFY_NONE);
#endif

    mbedtls_ssl_conf_rng(&bl_hsbuf->conf, mbedtls_ctr_drbg_random, &bl_hsbuf->ctr_drbg);

    //todo
    mbedtls_ssl_conf_read_timeout(&bl_hsbuf->conf, 0);
    //mbedtls_ssl_set_timer_cb(&ssl, &ssl_timer, f_set_timer, f_get_timer);

    mbedtls_ssl_conf_dbg( &bl_hsbuf->conf, bl_debug, stdout );

#if defined(MBEDTLS_DEBUG_C)
    // mbedtls_debug_set_threshold(4);
#endif

    if ((ret = mbedtls_ssl_setup(&bl_hsbuf->ssl, &bl_hsbuf->conf)) != 0) {
        printf("mbedtls_ssl_setup returned -0x%x\r\n", -ret);
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    mbedtls_net_init(&bl_hsbuf->server_fd);

    bl_hsbuf->server_fd.fd = socket(AF_INET, SOCK_STREAM, 0);

    if (bl_hsbuf->server_fd.fd < 0) {
        printf("ssl creat socket fd failed\r\n");
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    flags = fcntl(bl_hsbuf->server_fd.fd, F_GETFL, 0);
    if (flags < 0 || fcntl(bl_hsbuf->server_fd.fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        printf("ssl fcntl: %s\r\n", strerror(errno));
        close(bl_hsbuf->server_fd.fd);
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    if (setsockopt(bl_hsbuf->server_fd.fd, SOL_SOCKET, SO_REUSEADDR,
                   (const char*) &reuse, sizeof(reuse)) != 0) {
        close(bl_hsbuf->server_fd.fd);
        printf("ssl set SO_REUSEADDR failed\r\n");
        return BL_TCP_CREATE_CONNECT_ERR;
    }

    memset(&servaddr, 0, sizeof(struct sockaddr_in));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = dst_addr;
    servaddr.sin_port = htons(port);

    if (connect(bl_hsbuf->server_fd.fd, (struct sockaddr*)&servaddr, sizeof(struct sockaddr_in)) == 0) {
        //printf("ssl dst %s errno %d\r\n", dst, errno);
    } else {
        //printf("ssl dst %s errno %d\r\n", dst, errno);
        if (errno == EINPROGRESS) {
            //printf("ssl tcp conncet noblock\r\n");
        } else {
            close(bl_hsbuf->server_fd.fd);
            return BL_TCP_CREATE_CONNECT_ERR;
        }
    }

    //todo
    //mbedtls_ssl_set_bio(&bl_hsbuf->ssl, &bl_hsbuf->server_fd, mbedtls_net_send, mbedtls_net_recv, mbedtls_net_recv_timeout); //noblock
    mbedtls_ssl_set_bio(&bl_hsbuf->ssl, &bl_hsbuf->server_fd, mbedtls_net_send, mbedtls_net_recv, NULL);
    return (int32_t)&bl_hsbuf->ssl;
}

void blTcpSslDisconnect(int32_t fd)
{
    mbedtls_ssl_context *pssl = (mbedtls_ssl_context *)fd;

    if (NULL == pssl) {
        printf("\rblTcpSslDisconnect\r\n");
    }

    mbedtls_ssl_close_notify(pssl);

    mbedtls_net_free((mbedtls_net_context*)(pssl->p_bio));

#if defined(BL_VERIFY)
    mbedtls_x509_crt_free( &bl_hsbuf->cacert );
#endif

    mbedtls_ssl_free( pssl );
    mbedtls_ssl_config_free( &bl_hsbuf->conf );
    mbedtls_ctr_drbg_free( &bl_hsbuf->ctr_drbg );
    mbedtls_entropy_free( &bl_hsbuf->entropy );

    free(bl_hsbuf);

    bl_hsbuf = NULL;
    printf("\rblTcpSslDisconnect end\r\n");
}

int32_t blTcpSslState(int32_t fd)
{
    // printf("blTcpSslState start\r\n");
    int errcode = BL_TCP_NO_ERROR;
    mbedtls_ssl_context *pssl = (mbedtls_ssl_context *)fd;
    int tcp_fd = ((mbedtls_net_context*)(pssl->p_bio))->fd;
    int ret;

    fd_set rset, wset;
    int ready_n;

    struct timeval timeout;

    socklen_t len =  sizeof(int);

    if (tcp_fd < 0 || NULL == pssl) {
        return BL_TCP_ARG_INVALID;
    }

    FD_ZERO(&rset);
    FD_ZERO(&wset);
    FD_CLR(tcp_fd, &rset);
    FD_CLR(tcp_fd, &wset);
    FD_SET(tcp_fd, &rset);
    FD_SET(tcp_fd, &wset);

    timeout.tv_sec = 2;
    timeout.tv_usec = 0;

    ready_n = select(tcp_fd + 1, &rset, &wset, NULL, &timeout);

    if (0 == ready_n) {
        errcode = BL_TCP_CONNECTING;
    } else if (ready_n < 0) {
        errcode = BL_TCP_CONNECT_ERR;
    } else {
        if (FD_ISSET(tcp_fd, &wset) != 0) {
            errcode = BL_TCP_CONNECTING;

            if(pssl->state != MBEDTLS_SSL_HANDSHAKE_OVER) {
                ret = mbedtls_ssl_handshake_step( pssl );
                // printf("mbedtls_ssl_handshake_step return = 0X%X\r\n", -ret);

                if ((0 != ret) && (MBEDTLS_ERR_SSL_WANT_READ != ret)) {
                    errcode = BL_TCP_CONNECT_ERR;
                }
            } else {
                errcode = BL_TCP_NO_ERROR;
            }
        } else {
            if (0 != getsockopt(tcp_fd, SOL_SOCKET, SO_ERROR, &ret, &len)) {
                errcode = BL_TCP_CONNECT_ERR;
            }
            if (0 != ret) {
                errcode = BL_TCP_CONNECT_ERR;
            }
            errcode = BL_TCP_CONNECT_ERR;
        }
    }

    return errcode;
}

int32_t blTcpSslSend(int32_t fd, const uint8_t* buf, uint16_t len)
{
    //puts("blTcpSslSend start\r\n");
    int ret = 0;
    mbedtls_ssl_context *pssl = (mbedtls_ssl_context *)fd;

    if (NULL == buf || NULL == pssl) {
        return BL_TCP_ARG_INVALID;
    }

    ret = mbedtls_ssl_write(pssl, buf, len);

    if(ret > 0) {
        return ret;
    } else {
        printf("blTcpSslsend error ret = 0X%X\r\n", -ret);
        return BL_TCP_SEND_ERR;
    }
    return ret;
}

int32_t blTcpSslRead(int32_t fd, uint8_t* buf, uint16_t len)
{
    int ret = 0;
    mbedtls_ssl_context *pssl = (mbedtls_ssl_context *)fd;

    if (NULL == buf || NULL == pssl) {
        return BL_TCP_ARG_INVALID;
    }

    ret = mbedtls_ssl_read(pssl, buf, len);

    if(ret > 0) {
        return ret;
    } else if(MBEDTLS_ERR_SSL_WANT_READ == ret) {
        return 0;
    } else {
        printf("blTcpSslRead ret = 0X%X\r\n", ret);
        return BL_TCP_READ_ERR;
    }
}

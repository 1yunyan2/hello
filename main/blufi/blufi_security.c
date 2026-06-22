/*
 * BluFi 安全协商实现（DH 密钥协商 + AES-CFB 加解密 + CRC16 校验）
 *
 * 来源：ESP-IDF 官方示例 examples/bluetooth/blufi/main/blufi_security.c
 * 许可证：Unlicense OR CC0-1.0（Espressif 示例，可自由使用）
 * 修改点：仅把 #include "blufi_example.h" 改为 #include "blufi_security.h"
 *         （函数实现保持原样，确保与 BluFi 协议栈兼容）
 *
 * 工作原理：手机与设备通过 DH（Diffie-Hellman）交换公钥，各自算出相同的
 * 共享密钥，再用 MD5 派生出 16 字节 AES 密钥，之后 BluFi 帧用该密钥 AES-CFB
 * 加密传输。取代了原 Unified Provisioning 的固定 PoP 密码 "abcd1234"。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"
#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
#include "esp_bt.h"
#endif

#include "esp_blufi_api.h"
#include "blufi_security.h"

#include "mbedtls/aes.h"
#include "mbedtls/dhm.h"
#include "mbedtls/md5.h"
#include "esp_crc.h"

/*
   SEC_TYPE_xxx 是 "BLUFI 协商密钥" 过程中的自定义数据包类型。
   如果用户使用其他协商过程来交换（或生成）密钥，应自行重新定义这些类型。
 */
#define SEC_TYPE_DH_PARAM_LEN 0x00  /* DH 参数长度 */
#define SEC_TYPE_DH_PARAM_DATA 0x01 /* DH 参数数据（含 P, G, 对端公钥） */
#define SEC_TYPE_DH_P 0x02          /* DH 素数 P（未直接使用） */
#define SEC_TYPE_DH_G 0x03          /* DH 生成元 G（未直接使用） */
#define SEC_TYPE_DH_PUBLIC 0x04     /* DH 公钥（未直接使用） */

/* BluFi 安全上下文结构体，保存 DH 协商与 AES 加解密所需的所有状态 */
struct blufi_security
{
#define DH_SELF_PUB_KEY_LEN 128
    uint8_t self_public_key[DH_SELF_PUB_KEY_LEN]; /* 本端 DH 公钥 */
#define SHARE_KEY_LEN 128
    uint8_t share_key[SHARE_KEY_LEN]; /* DH 协商出的共享密钥 */
    size_t share_len;                 /* 共享密钥实际长度 */
#define PSK_LEN 16
    uint8_t psk[PSK_LEN];    /* 由共享密钥经 MD5 派生出的 16 字节 AES 密钥 */
    uint8_t *dh_param;       /* 接收对端 DH 参数的临时缓冲区 */
    int dh_param_len;        /* DH 参数长度 */
    uint8_t iv[16];          /* AES-CFB 初始向量（IV） */
    mbedtls_dhm_context dhm; /* mbedTLS DH 上下文 */
    mbedtls_aes_context aes; /* mbedTLS AES 上下文 */
};
static struct blufi_security *blufi_sec; /* 全局安全上下文单例 */

/* 随机数生成回调，供 mbedTLS 内部调用，使用 ESP32 硬件随机数 */
static int myrand(void *rng_state, unsigned char *output, size_t len)
{
    esp_fill_random(output, len);
    return (0);
}

extern void btc_blufi_report_error(esp_blufi_error_state_t state);

/**
 * @brief DH 密钥协商数据处理函数
 *
 * 解析手机端发来的 DH 参数（P/G/对端公钥），生成本端公钥与共享密钥，
 * 再通过 MD5 派生出 AES 密钥，最后将本端公钥返回给手机端。
 *
 * @param data        对端发来的协商数据
 * @param len         数据长度
 * @param output_data 输出数据（本端公钥），由调用方发送回对端
 * @param output_len  输出数据长度
 * @param need_free   输出数据是否需要调用方释放
 */
void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free)
{
    if (data == NULL || len < 3)
    {
        BLUFI_ERROR("BLUFI Invalid data format");
        btc_blufi_report_error(ESP_BLUFI_DATA_FORMAT_ERROR);
        return;
    }

    int ret;
    uint8_t type = data[0]; /* 数据包首字节为类型标识 */

    if (blufi_sec == NULL)
    {
        BLUFI_ERROR("BLUFI Security is not initialized");
        btc_blufi_report_error(ESP_BLUFI_INIT_SECURITY_ERROR);
        return;
    }

    switch (type)
    {
    case SEC_TYPE_DH_PARAM_LEN:
        /* 第一阶段：接收 DH 参数总长度，分配缓冲区 */
        blufi_sec->dh_param_len = ((data[1] << 8) | data[2]);
        if (blufi_sec->dh_param)
        {
            free(blufi_sec->dh_param);
            blufi_sec->dh_param = NULL;
        }
        blufi_sec->dh_param = (uint8_t *)malloc(blufi_sec->dh_param_len);
        if (blufi_sec->dh_param == NULL)
        {
            blufi_sec->dh_param_len = 0; /* 重置长度，避免使用未分配的内存 */
            btc_blufi_report_error(ESP_BLUFI_DH_MALLOC_ERROR);
            BLUFI_ERROR("%s, malloc failed\n", __func__);
            return;
        }
        break;
    case SEC_TYPE_DH_PARAM_DATA:
    {
        /* 第二阶段：接收 DH 参数数据，完成密钥协商 */
        if (blufi_sec->dh_param == NULL)
        {
            BLUFI_ERROR("%s, blufi_sec->dh_param == NULL\n", __func__);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        if (len < (blufi_sec->dh_param_len + 1))
        {
            BLUFI_ERROR("%s, invalid dh param len\n", __func__);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        /* 1. 读取对端 DH 参数（P, G, 对端公钥）到 mbedTLS DH 上下文 */
        uint8_t *param = blufi_sec->dh_param;
        memcpy(blufi_sec->dh_param, &data[1], blufi_sec->dh_param_len);
        ret = mbedtls_dhm_read_params(&blufi_sec->dhm, &param, &param[blufi_sec->dh_param_len]);
        if (ret)
        {
            BLUFI_ERROR("%s read param failed %d\n", __func__, ret);
            btc_blufi_report_error(ESP_BLUFI_READ_PARAM_ERROR);
            return;
        }
        free(blufi_sec->dh_param);
        blufi_sec->dh_param = NULL;

        const int dhm_len = mbedtls_dhm_get_len(&blufi_sec->dhm);

        if (dhm_len > DH_SELF_PUB_KEY_LEN)
        {
            BLUFI_ERROR("%s dhm len not support %d\n", __func__, dhm_len);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        /* 2. 生成本端 DH 公钥 */
        ret = mbedtls_dhm_make_public(&blufi_sec->dhm, dhm_len, blufi_sec->self_public_key, DH_SELF_PUB_KEY_LEN, myrand, NULL);
        if (ret)
        {
            BLUFI_ERROR("%s make public failed %d\n", __func__, ret);
            btc_blufi_report_error(ESP_BLUFI_MAKE_PUBLIC_ERROR);
            return;
        }

        /* 3. 计算共享密钥（双方独立计算，结果相同） */
        ret = mbedtls_dhm_calc_secret(&blufi_sec->dhm,
                                      blufi_sec->share_key,
                                      SHARE_KEY_LEN,
                                      &blufi_sec->share_len,
                                      myrand, NULL);
        if (ret)
        {
            BLUFI_ERROR("%s mbedtls_dhm_calc_secret failed %d\n", __func__, ret);
            btc_blufi_report_error(ESP_BLUFI_DH_PARAM_ERROR);
            return;
        }

        /* 4. 用 MD5 将共享密钥派生为 16 字节 AES 密钥 */
        ret = mbedtls_md5(blufi_sec->share_key, blufi_sec->share_len, blufi_sec->psk);

        if (ret)
        {
            BLUFI_ERROR("%s mbedtls_md5 failed %d\n", __func__, ret);
            btc_blufi_report_error(ESP_BLUFI_CALC_MD5_ERROR);
            return;
        }

        /* 5. 设置 AES 加密密钥（CFB 模式加解密使用同一密钥） */
        mbedtls_aes_setkey_enc(&blufi_sec->aes, blufi_sec->psk, PSK_LEN * 8);

        /* 6. 将本端公钥作为输出，由 BluFi 协议栈发回给手机端 */
        *output_data = &blufi_sec->self_public_key[0];
        *output_len = dhm_len;
        *need_free = false;
    }
    break;
    case SEC_TYPE_DH_P:
        break;
    case SEC_TYPE_DH_G:
        break;
    case SEC_TYPE_DH_PUBLIC:
        break;
    }
}

/**
 * @brief AES-CFB128 加密函数
 *
 * 使用协商出的 AES 密钥对 BluFi 数据帧进行加密。
 * iv8 为 BluFi 帧头中的序列号，作为 IV 的首字节以保证每次加密 IV 不同。
 *
 * @param iv8        IV 的首字节（来自 BluFi 帧序列号）
 * @param crypt_data 待加密数据（原地加密）
 * @param crypt_len  数据长度
 * @return 加密后长度（成功），-1 失败
 */
int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    int ret;
    size_t iv_offset = 0;
    uint8_t iv0[16];

    if (!blufi_sec)
    {
        return -1;
    }

    memcpy(iv0, blufi_sec->iv, sizeof(blufi_sec->iv));
    iv0[0] = iv8; /* 将 iv8 设置为 iv0[0] */

    ret = mbedtls_aes_crypt_cfb128(&blufi_sec->aes, MBEDTLS_AES_ENCRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
    if (ret)
    {
        return -1;
    }

    return crypt_len;
}

/**
 * @brief AES-CFB128 解密函数
 *
 * 使用协商出的 AES 密钥对 BluFi 数据帧进行解密。
 * 结构与加密函数完全对称。
 *
 * @param iv8        IV 的首字节（来自 BluFi 帧序列号）
 * @param crypt_data 待解密数据（原地解密）
 * @param crypt_len  数据长度
 * @return 解密后长度（成功），-1 失败
 */
int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len)
{
    int ret;
    size_t iv_offset = 0;
    uint8_t iv0[16];

    if (!blufi_sec)
    {
        return -1;
    }

    memcpy(iv0, blufi_sec->iv, sizeof(blufi_sec->iv));
    iv0[0] = iv8; /* 将 iv8 设置为 iv0[0] */

    ret = mbedtls_aes_crypt_cfb128(&blufi_sec->aes, MBEDTLS_AES_DECRYPT, crypt_len, &iv_offset, iv0, crypt_data, crypt_data);
    if (ret)
    {
        return -1;
    }

    return crypt_len;
}

/**
 * @brief CRC16 校验和计算
 *
 * 对 BluFi 帧数据进行 CRC16 校验，用于检测传输错误。
 * 注意：iv8 参数在此实现中未使用。
 *
 * @param iv8  保留参数（未使用）
 * @param data 待校验数据
 * @param len  数据长度
 * @return CRC16 校验值
 */
uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len)
{
    /* iv8 被忽略，不参与计算 */
    return esp_crc16_be(0, data, len);
}

/**
 * @brief 初始化 BluFi 安全上下文
 *
 * 分配安全结构体内存，初始化 mbedTLS DH 和 AES 上下文，清零 IV。
 *
 * @return ESP_OK 成功，ESP_FAIL 内存分配失败
 */
esp_err_t blufi_security_init(void)
{
    blufi_sec = (struct blufi_security *)malloc(sizeof(struct blufi_security));
    if (blufi_sec == NULL)
    {
        return ESP_FAIL;
    }

    memset(blufi_sec, 0x0, sizeof(struct blufi_security));

    mbedtls_dhm_init(&blufi_sec->dhm);
    mbedtls_aes_init(&blufi_sec->aes);

    memset(blufi_sec->iv, 0x0, sizeof(blufi_sec->iv));
    return 0;
}

/**
 * @brief 反初始化 BluFi 安全上下文
 *
 * 释放 DH 参数缓冲区，释放 mbedTLS 资源，清零并释放安全结构体。
 */
void blufi_security_deinit(void)
{
    if (blufi_sec == NULL)
    {
        return;
    }
    if (blufi_sec->dh_param)
    {
        free(blufi_sec->dh_param);
        blufi_sec->dh_param = NULL;
    }
    mbedtls_dhm_free(&blufi_sec->dhm);
    mbedtls_aes_free(&blufi_sec->aes);

    memset(blufi_sec, 0x0, sizeof(struct blufi_security));

    free(blufi_sec);
    blufi_sec = NULL;
}

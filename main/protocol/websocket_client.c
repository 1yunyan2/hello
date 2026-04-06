#include "websocket_client.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "esp_mac.h"
#include "cJSON.h"
#include <string.h>

ESP_EVENT_DEFINE_BASE(PROTOCOL_EVENT);
#define TAG "Protocol"

#define protocol_send_text(protocol, fmtstr, ...)                                                                    \
    do                                                                                                               \
    {                                                                                                                \
        if (!esp_websocket_client_is_connected(protocol->websocket_client))                                          \
            return;                                                                                                  \
        char *_message = NULL;                                                                                       \
        asprintf(&_message, fmtstr, ##__VA_ARGS__);                                                                  \
        ESP_LOGD(TAG, "Send JSON: %s", _message);                                                                    \
        esp_websocket_client_send_text(protocol->websocket_client, _message, strlen(_message), pdMS_TO_TICKS(5000)); \
        free(_message);                                                                                              \
    } while (0)

struct protocol
{
    esp_websocket_client_handle_t websocket_client;
    char *session_id;
    esp_event_handler_t callback;
    void *handler_args;
};

static void protocol_hello_handler(protocol_t *protocol, cJSON *root)
{
    if (protocol->session_id)
    {
        free(protocol->session_id);
        protocol->session_id = NULL;
    }
    cJSON *session_id = cJSON_GetObjectItem(root, "session_id");
    if (cJSON_IsString(session_id))
    {
        protocol->session_id = strdup(session_id->valuestring);
    }
    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_HELLO, NULL);
}

static void protocol_llm_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *emotion = cJSON_GetObjectItem(root, "emotion");
    if (cJSON_IsString(emotion))
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_LLM, emotion->valuestring);
    }
}

static void protocol_stt_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *text = cJSON_GetObjectItem(root, "text");
    if (cJSON_IsString(text))
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_STT, text->valuestring);
    }
}

static void protocol_tts_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *state = cJSON_GetObjectItem(root, "state");
    if (!cJSON_IsString(state))
        return;
    if (strcmp(state->valuestring, "start") == 0)
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_START, NULL);
    }
    else if (strcmp(state->valuestring, "stop") == 0)
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_STOP, NULL);
    }
    else if (strcmp(state->valuestring, "sentence_start") == 0)
    {
        cJSON *text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text))
            protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_SENTENCE_START, text->valuestring);
    }
}

static void protocol_iot_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *commands = cJSON_GetObjectItem(root, "commands");
    if (commands)
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_IOT, commands);
}

static void protocol_websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    protocol_t *protocol = (protocol_t *)handler_args;
    if (!protocol->callback)
        return;
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id)
    {
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "Websocket Error");
        break;
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Websocket Connected");
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_CONNECTED, NULL);
        break;
    case WEBSOCKET_EVENT_DATA:
        if (data->op_code == 0x02)
        { // Binary Audio
            binary_data_t bin = {.ptr = (void *)data->data_ptr, .size = data->data_len};
            protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_AUDIO, &bin);
            return;
        }
        if (data->op_code == 0x01)
        { // Text JSON
            cJSON *root = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (!root)
                return;
            cJSON *type = cJSON_GetObjectItem(root, "type");
            if (cJSON_IsString(type))
            {
                if (strcmp(type->valuestring, "hello") == 0)
                    protocol_hello_handler(protocol, root);
                else if (strcmp(type->valuestring, "llm") == 0)
                    protocol_llm_handler(protocol, root);
                else if (strcmp(type->valuestring, "stt") == 0)
                    protocol_stt_handler(protocol, root);
                else if (strcmp(type->valuestring, "tts") == 0)
                    protocol_tts_handler(protocol, root);
                else if (strcmp(type->valuestring, "iot") == 0)
                    protocol_iot_handler(protocol, root);
            }
            cJSON_Delete(root);
        }
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
    case WEBSOCKET_EVENT_FINISH:
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_DISCONNECTED, NULL);
        break;
    }
}

protocol_t *protocol_create(const char *url, const char *token)
{
    protocol_t *protocol = (protocol_t *)calloc(1, sizeof(protocol_t));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    char *headers = NULL;
    asprintf(&headers, "Device-Id: %s\r\nClient-Id: %s\r\nAuthorization: Bearer %s\r\nProtocol-Version: 1\r\n",
             mac_str, mac_str, token ? token : "");

    esp_websocket_client_config_t websocket_cfg = {
        .uri = url,
        .headers = headers,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .network_timeout_ms = 5000,
        .reconnect_timeout_ms = 5000,
    };
    protocol->websocket_client = esp_websocket_client_init(&websocket_cfg);
    esp_websocket_register_events(protocol->websocket_client, WEBSOCKET_EVENT_ANY, protocol_websocket_event_handler, protocol);
    free(headers);
    return protocol;
}

void protocol_destroy(protocol_t *protocol)
{
    if (protocol->session_id)
        free(protocol->session_id);
    esp_websocket_client_destroy(protocol->websocket_client);
    free(protocol);
}

void protocol_connect(protocol_t *protocol)
{
    if (!esp_websocket_client_is_connected(protocol->websocket_client))
        esp_websocket_client_start(protocol->websocket_client);
}

void protocol_disconnect(protocol_t *protocol)
{
    if (esp_websocket_client_is_connected(protocol->websocket_client))
        esp_websocket_client_stop(protocol->websocket_client);
}

bool protocol_is_connected(protocol_t *protocol)
{
    return esp_websocket_client_is_connected(protocol->websocket_client);
}

void protocol_send_hello(protocol_t *protocol)
{
    protocol_send_text(protocol, "{\"audio_params\":{\"channels\":1,\"format\":\"opus\",\"frame_duration\":60,\"sample_rate\":16000},\"transport\":\"websocket\",\"type\":\"hello\",\"version\":1}");
}

void protocol_send_wake_word(protocol_t *protocol, const char *wake_word)
{
    protocol_send_text(protocol, "{\"session_id\":\"%s\",\"state\":\"detect\",\"text\":\"%s\",\"type\":\"listen\"}", protocol->session_id ? protocol->session_id : "", wake_word);
}

void protocol_send_start_listening(protocol_t *protocol, protocol_listen_type_t type)
{
    static const char *mode_str[] = {"auto", "manual", "realtime"};
    protocol_send_text(protocol, "{\"mode\":\"%s\",\"session_id\":\"%s\",\"state\":\"start\",\"type\":\"listen\"}", mode_str[type], protocol->session_id ? protocol->session_id : "");
}

void protocol_send_stop_listening(protocol_t *protocol)
{
    protocol_send_text(protocol, "{\"session_id\":\"%s\",\"state\":\"stop\",\"type\":\"listen\"}", protocol->session_id ? protocol->session_id : "");
}

void protocol_send_audio_data(protocol_t *protocol, binary_data_t *data)
{
    if (esp_websocket_client_is_connected(protocol->websocket_client))
        esp_websocket_client_send_bin(protocol->websocket_client, data->ptr, data->size, pdMS_TO_TICKS(10000));
}

void protocol_send_abort_speaking(protocol_t *protocol)
{
    protocol_send_text(protocol, "{\"reason\":\"wake_word_detected\",\"session_id\":\"%s\",\"type\":\"abort\"}", protocol->session_id ? protocol->session_id : "");
}

void protocol_send_iot(protocol_t *protocol, protocol_iot_message_type_t type, cJSON *json)
{
    // 防止在断开连接时调用底层 API 导致报错
    if (!esp_websocket_client_is_connected(protocol->websocket_client))
    {
        return;
    }
    const char *type_str[] = {"descriptors", "states"};
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "session_id", protocol->session_id ? protocol->session_id : "");
    cJSON_AddStringToObject(root, "type", "iot");
    cJSON_AddBoolToObject(root, "update", cJSON_True);
    cJSON_AddItemToObject(root, type_str[type], json);
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    esp_websocket_client_send_text(protocol->websocket_client, json_str, strlen(json_str), pdMS_TO_TICKS(10000));
    free(json_str);
}

void protocol_register_callback(protocol_t *protocol, esp_event_handler_t callback, void *handler_args)
{
    protocol->callback = callback;
    protocol->handler_args = handler_args;
}
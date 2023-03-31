#ifndef MQTT_CONSTANTS_H
#define MQTT_CONSTANTS_H

#define PAYLOAD_SMART_SWITCH_OFF "OFF"
#define PAYLOAD_SMART_SWITCH_ON "ON"
#define TOPIC_CLOSE_BLIND "asol/close_bedroom_blind"
#define TOPIC_KV_GET "asol/kv/get/"
#define TOPIC_KV_GET_RESP "asol/kv/get-resp/"
#define TOPIC_KV_SET "asol/kv/set/"
#define TOPIC_KV_SET_RESP "asol/kv/set-resp/"
#define TOPIC_MONITOR_OFF "asol/monitor_off"
#define TOPIC_MONITOR_ON "asol/monitor_on"
#define TOPIC_OPEN_BLIND "asol/open_bedroom_blind"
#define TOPIC_SESSION_BEGAN "asol/session/began"
#define TOPIC_SESSION_BEGIN "asol/session/begin"
#define TOPIC_SESSION_END "asol/session/end"
#define TOPIC_SESSION_ENDED "asol/session/ended"
#define TOPIC_SESSION_PAUSE "asol/session/pause"
#define TOPIC_SESSION_PAUSED "asol/session/paused"
#define TOPIC_SESSION_RESUME "asol/session/resume"
#define TOPIC_SESSION_RESUMED "asol/session/resumed"
#define TOPIC_SMART_SWITCH "cmnd/tasmota_sw1/Power1"
#define TOPIC_STATE_REPORT_JSON "asol/state-report-json"
#define TOPIC_STREAM_END "asol/stream/end"
#define TOPIC_STREAM_START "asol/stream/begin"
#define TOPIC_TRIGGER_DSLR "asol/dslr-crop-capture"
#define TOPIC_TRIGGER_DSLR_RESP "asol/dslr-crop-capture-resp"

#endif // MQTT_CONSTANTS_H

# IoT模块重构设计方案 - ML307R替代ESP32 WiFi

## 1. 背景与目标

### 1.1 背景

现有系统使用ESP32 WiFi模块与云端通信，wifi.c负责：
- properties_changed（属性上报）
- set_properties（云端下发写属性）
- get_properties（云端下发读属性）

现改为使用ML307R 4G模块，通过MQTT协议与云端通信。

**关键变化**：
- 原来：通过UART与ESP32通信，使用**串口文本协议**
- 现在：通过MQTT与服务器通信，使用**JSON协议**

### 1.2 目标

将wifi.c重构为iot.c，保留原有属性解析/上报逻辑，通过ML307R的MQTT接口与云端通信。

## 2. 协议对比

### 2.1 串口协议（ESP32 → 云端）

通过`get_down`轮询获取命令，使用文本格式：

```
// 下行命令获取
↑ get_down
↓ down set_properties 1 1 10

// 上报属性
↑ properties_changed 2 1 0 2 2 220.50\r
↓ ok

// 响应结果
↑ result 1 1 0\r
↓ ok
```

### 2.2 MQTT协议（ML307R → 云端）

通过订阅`down`主题接收命令，发布JSON到`up`主题：

```json
// 上报属性 (MQTT发布到 up 主题)
{
  "id": 2534,
  "method": "properties_changed",
  "params": [
    {"siid": 2, "piid": 1, "value": 0},
    {"siid": 2, "piid": 2, "value": 220.50}
  ]
}

// 读取属性响应 (MQTT发布到 up 主题)
{
  "id": 2534,
  "method": "result",
  "params": [
    {"siid": 1, "piid": 2, "value": 10, "code": 0},
    {"siid": 1, "piid": 3, "code": -4003}
  ]
}
```

```json
// 下行命令接收 (MQTT订阅 down 主题)
{
  "id": 2534,
  "method": "get_properties",
  "params": [
    {"siid": 1, "piid": 2},
    {"siid": 1, "piid": 3}
  ]
}
```

### 2.3 MQTT主题定义

| 主题 | 方向 | 用途 |
|------|------|------|
| `up` | 设备→服务器 | 属性上报、事件上报、结果响应 |
| `down` | 服务器→设备 | 云端下发命令（get/set_properties、action等） |

## 3. 设计方案

### 3.1 核心架构

```
┌─────────────────────────────────────────────────────────┐
│                        ml307r.c                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │ - ml307r_mqtt_connected                          │   │
│  │ - mqtt_downlink_cb() 回调                        │   │
│  │ - ml307r_mqtt_publish()                         │   │
│  └─────────────────────────────────────────────────┘   │
└───────────────────────┬─────────────────────────────────┘
                        │ MQTT已连接
                        ▼
┌─────────────────────────────────────────────────────────┐
│                        iot.c                             │
│  ┌─────────────────────────────────────────────────┐   │
│  │ iot_task()                                       │   │
│  │  - 处理MQTT下行JSON解析                         │   │
│  │  - 构造MQTT上行JSON响应                         │   │
│  │  - 调度属性上报（立即/定时）                    │   │
│  └─────────────────────────────────────────────────┘   │
│                           │                             │
│  ┌────────────────────────┴────────────────────────┐   │
│  │              复用wifi.c属性处理逻辑               │   │
│  │  - 属性值映射 (siid/piid → sys_param)           │   │
│  │  - 阈值校验 (wifi_set_properties_reply)          │   │
│  │  - 定时上报调度逻辑                              │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

### 3.2 调用流程

**初始化流程**：
```
ml307r_init()
  → ml307r_mqtt_connect()
  → MQTT连接成功
  → 设置 mqtt_downlink_cb = iot_mqtt_downlink_handler
  → 置 ml307r_mqtt_connected = true
```

**主循环**：
```c
// 在ml307r_task()中，当MQTT已连接时调用
if (ml307r_mqtt_connected) {
    iot_task();
}
```

### 3.3 iot_task() 职责

```c
void iot_task(void) {
    // 1. 处理MQTT下行消息（通过回调mqtt_downlink_cb）
    //    - 解析JSON: method, params[]
    //    - 根据method分发:
    //      - get_properties → 构造result JSON响应
    //      - set_properties → 解析参数，设置sys_param，构造result JSON响应
    //      - action         → 解析参数，执行动作，构造result JSON响应
    //    - 通过ml307r_mqtt_publish("up", json, qos)回复

    // 2. 即时属性上报
    //    - 检测属性变化标志
    //    - 构造properties_changed JSON
    //    - ml307r_mqtt_publish("up", json, qos)

    // 3. 定时属性上报
    //    - prop_ct_count >= 5分钟 → iot_report_ct_properties()
    //    - prop_inv_count >= 3分钟 → iot_report_inverter_scheduled()
}

void iot_mqtt_downlink_handler(const char *topic, const char *payload) {
    // 解析JSON并分发处理
    // 存储待处理的命令，在iot_task()中处理
}
```

## 4. JSON处理函数设计

### 4.1 新增函数

```c
// iot.c

// MQTT下行JSON解析
void iot_parse_mqtt_downlink(const char *json);           // 解析下行JSON
int iot_get_properties_handler(uint32_t msg_id, const char *params_json);  // 处理get_properties
int iot_set_properties_handler(uint32_t msg_id, const char *params_json); // 处理set_properties
int iot_action_handler(uint32_t msg_id, const char *params_json);         // 处理action

// MQTT上行JSON构造
char* iot_build_result_json(uint32_t msg_id, const char *method, const char *params_json, int code);
char* iot_build_properties_changed_json(void);  // 构造properties_changed JSON

// 主任务
void iot_task(void);                              // 主任务（MQTT连接后调用）
void iot_mqtt_downlink_handler(const char *topic, const char *payload);  // 下行回调
```

### 4.2 复用wifi.c函数（需适配）

| 原函数 | 用途 | 改动点 |
|--------|------|--------|
| `wifi_properties_para()` | 解析下行参数 | 参数解析逻辑复用，适配JSON格式 |
| `wifi_get_properties_reply()` | 构造属性读取响应 | 改为输出JSON格式 |
| `wifi_set_properties_reply()` | 构造属性设置响应 | 改为输出JSON格式，添加阈值校验 |
| `report_ct_siid2_properties()` | CT siid2属性上报 | 改为构造JSON数组 |
| `report_ct_siid3_properties()` | CT siid3属性上报 | 改为构造JSON数组 |
| `report_inverter_properties()` | INV立即属性上报 | 改为构造JSON数组 |
| `report_inverter_properties_scheduled()` | INV定时属性上报 | 改为构造JSON数组 |
| `report_inverter_set_param()` | INV设置参数上报 | 改为构造JSON数组 |

### 4.3 JSON构造示例

```c
// properties_changed JSON构造示例
char* iot_build_properties_changed_json(void) {
    static char json[512];
    // 复用wifi.c中的属性获取逻辑，构造JSON数组
    // {"id":1234,"method":"properties_changed","params":[{"siid":2,"piid":1,"value":0},...]}
    return json;
}

// result JSON构造示例
char* iot_build_result_json(uint32_t msg_id, int siid, int piid, int code, const char *value) {
    static char json[256];
    // {"id":2534,"method":"result","params":[{"siid":1,"piid":2,"value":10,"code":0}]}
    return json;
}
```

## 5. ML307R接口扩展

### 5.1 ml307r.h 新增

```c
// MQTT下行回调类型
typedef void (*mqtt_downlink_cb)(const char *topic, const char *payload);

// 设置下行回调（在MQTT连接成功后调用）
int ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb);

// 获取MQTT连接状态
bool ml307r_mqtt_is_connected(void);

// MQTT发布（已存在）
int ml307r_mqtt_publish(const char *topic, const char *payload, int qos);
```

### 5.2 ml307r.c MQTT回调实现

```c
static mqtt_downlink_cb s_mqtt_downlink_cb = NULL;

void ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb) {
    s_mqtt_downlink_cb = cb;
}

// 当MQTT收到down主题消息时调用
void ml307r_mqtt_on_downlink(const char *topic, const char *payload) {
    if (s_mqtt_downlink_cb) {
        s_mqtt_downlink_cb(topic, payload);
    }
}
```

## 6. 上报策略

### 6.1 即时上报（立即触发）

| 条件 | 触发函数 | JSON method |
|------|----------|-------------|
| INV告警状态变化 | `iot_report_inverter_properties()` | properties_changed |
| INV设置参数变化 | `iot_report_inverter_set_param()` | properties_changed |
| 配对成功 | properties_changed 3 5 | properties_changed |
| 解配成功 | properties_changed 3 6 | properties_changed |

### 6.2 定时上报（周期触发）

| 周期 | 触发函数 | 内容 |
|------|----------|------|
| 5分钟 | `iot_report_ct_siid2()` | CT1/CT2/CT3功率、电压、频率 |
| 5分钟 | `iot_report_ct_siid3()` | 配对列表、功率模式、馈网限值 |
| 3分钟 | `iot_report_inverter_scheduled()` | 发电量、PV输入、累计发电量、温度 |

## 7. 下行命令处理流程

```c
// MQTT收到down主题消息
void iot_mqtt_downlink_handler(const char *topic, const char *payload) {
    cJSON *root = cJSON_Parse(payload);
    if (!root) return;
    
    int msg_id = cJSON_GetObjectItem(root, "id")->valueint;
    const char *method = cJSON_GetObjectItem(root, "method")->valuestring;
    cJSON *params = cJSON_GetObjectItem(root, "params");
    
    if (strcmp(method, "get_properties") == 0) {
        iot_handle_get_properties(msg_id, params);
    } else if (strcmp(method, "set_properties") == 0) {
        iot_handle_set_properties(msg_id, params);
    } else if (strcmp(method, "action") == 0) {
        iot_handle_action(msg_id, params);
    }
    
    cJSON_Delete(root);
}

void iot_handle_get_properties(uint32_t msg_id, cJSON *params) {
    // 解析siid/piid数组
    // 复用wifi_properties_para()解析逻辑
    // 调用wifi_get_properties_reply()获取属性值
    // 构造result JSON并发布到"up"主题
}
```

## 8. 文件改动清单

### 8.1 新增文件

| 文件 | 路径 | 说明 |
|------|------|------|
| iot.h | app/inc/ | IoT模块头文件 |
| iot.c | app/src/ | IoT模块实现（重构自wifi.c） |

### 8.2 修改文件

| 文件 | 改动 |
|------|------|
| app/src/ml307r.c | 添加mqtt_downlink_cb机制 |
| app/src/ml307r.h | 添加mqtt回调相关接口 |

### 8.3 删除/保留文件

| 文件 | 说明 |
|------|------|
| app/src/wifi.c | 重构为iot.c，原文件删除 |
| app/inc/wifi.h | 重构为iot.h，原文件删除 |

## 9. 注意事项

1. **JSON解析库**：需要使用cJSON库解析和构造JSON
2. **MQTT主题**：需要与云端确认具体的up/down主题名称
3. **消息ID**：MQTT使用id字段关联请求和响应
4. **INV_DEVICE_MAX_NUM**：如需支持更多逆变器，可调整为8

## 10. 测试要点

1. MQTT连接建立后，IoT任务正常调度
2. 云端下发get_properties能够返回正确属性值（JSON格式）
3. 云端下发set_properties能够正确解析并设置参数
4. 属性变化时能够立即通过MQTT properties_changed上报
5. 定时属性能够按周期正确上报
6. MQTT断开后自动重连，恢复后继续工作

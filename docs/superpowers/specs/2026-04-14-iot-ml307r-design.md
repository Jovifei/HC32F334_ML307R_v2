# IoT模块重构设计方案 - ML307R替代ESP32 WiFi

## 1. 背景与目标

### 1.1 背景

现有系统使用ESP32 WiFi模块与云端通信，wifi.c负责：
- properties_changed（属性上报）
- set_properties（云端下发写属性）
- get_properties（云端下发读属性）

现改为使用ML307R 4G模块，通过MQTT协议与云端通信。

### 1.2 目标

将wifi.c重构为iot.c，保留原有属性解析/上报逻辑，通过ML307R的MQTT接口与云端通信。

## 2. 设计方案

### 2.1 核心架构

```
┌─────────────────────────────────────────────────────────┐
│                        ml307r.c                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │ ml307r_mqtt_connected flag                       │   │
│  │ (ML307R MQTTS连接成功后置true)                   │   │
│  └─────────────────────────────────────────────────┘   │
└───────────────────────┬─────────────────────────────────┘
                        │ ml307r_mqtt_connected == true
                        │ 触发调用 iot_task()
                        ▼
┌─────────────────────────────────────────────────────────┐
│                        iot.c                            │
│  ┌─────────────────────────────────────────────────┐   │
│  │ iot_task()                                       │   │
│  │  - 处理MQTT下行消息                              │   │
│  │  - 调度属性上报（立即/定时）                     │   │
│  │  - 200ms轮询get_down                             │   │
│  └─────────────────────────────────────────────────┘   │
│                           │                             │
│  ┌────────────────────────┴────────────────────────┐   │
│  │              复用wifi.c函数                      │   │
│  │  - wifi_properties_para()                        │   │
│  │  - wifi_get/set_properties_reply()              │   │
│  │  - report_ct_siid2/3_properties()                │   │
│  │  - report_inverter_properties()                 │   │
│  │  - report_inverter_properties_scheduled()       │   │
│  └─────────────────────────────────────────────────┘   │
└───────────────────────┬─────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────┐
│                    ML307R MQTT接口                       │
│  - ml307r_mqtt_publish(topic, payload, qos)              │
│  - ml307r_mqtt_set_downlink_callback(cb)                │
└─────────────────────────────────────────────────────────┘
```

### 2.2 调用流程

**初始化流程（无独立iot_init）**：
```
ml307r_init()
  → ml307r_mqtt_connect()
  → MQTT连接成功回调
  → 设置 iot_downlink_callback = iot_mqtt_downlink_handler
  → 置 ml307r_mqtt_connected = true
```

**主循环**：
```c
// 在ml307r_task()中，当MQTT已连接时调用
if (ml307r_mqtt_connected) {
    iot_task();
}
```

### 2.3 iot_task() 职责

```c
void iot_task(void) {
    // 1. 处理MQTT下行回调消息（如果有）
    //    - 解析set_properties/get_properties
    //    - 构造result响应
    //    - 通过ml307r_mqtt_publish()回复

    // 2. 即时属性上报（关键告警、状态突变）
    //    - 检测属性变化标志
    //    - 立即通过ml307r_mqtt_publish()上报properties_changed

    // 3. 定时属性上报
    //    - prop_ct_count >= 5分钟 → iot_report_ct_properties()
    //    - prop_inv_count >= 3分钟 → iot_report_inverter_scheduled()

    // 4. get_down轮询（200ms间隔）
    //    - 检测get_down触发条件
    //    - 调用wifi_properties_para()解析
    //    - 构造result响应并mqtt_publish
}
```

### 2.4 MQTT主题定义

参考DMIOT协议，使用MQTT主题进行通信：

| 方向 | 主题 | 用途 |
|------|------|------|
| 上行 | `iot/up/link` | 属性上报（properties_changed） |
| 上行 | `iot/up/result` | 命令响应（result） |
| 下行 | `iot/down/link` | 云端下发命令（set/get_properties） |

**payload格式**（复用原有DMIOT文本协议）：
```
// properties_changed 上报
properties_changed 2 1 0 2 2 220.50 2 3 50.00\r

// result 响应
result 4 2 0 4 3 0 1500.00\r
```

## 3. 函数设计

### 3.1 新增函数（iot.c）

```c
// iot.c 头文件新增
void iot_task(void);                              // 主任务（MQTT连接后调用）
void iot_mqtt_downlink_handler(const char *topic, const char *payload);  // 下行回调

// 属性上报（复用wifi.c）
void iot_report_ct_properties(uint8_t siid);      // CT设备属性定时上报
void iot_report_inverter_properties(uint8_t inv_idx);    // INV属性立即上报
void iot_report_inverter_scheduled(uint8_t inv_idx);     // INV属性定时上报
void iot_report_inverter_set_param(uint8_t inv_idx);    // INV设置参数上报

// 属性变化检测
void iot_check_and_report_immediate(void);         // 检查即时上报条件
```

### 3.2 复用函数（来自wifi.c）

| 原函数名 | 新函数名 | 改动 |
|----------|----------|------|
| `wifi_properties_para()` | 复用 | 无需改动 |
| `wifi_get_properties_reply()` | 复用 | 无需改动 |
| `wifi_set_properties_reply()` | 复用 | 无需改动 |
| `report_ct_siid2_properties()` | `iot_report_ct_siid2()` | 重命名 |
| `report_ct_siid3_properties()` | `iot_report_ct_siid3()` | 重命名 |
| `report_inverter_properties()` | `iot_report_inverter_properties()` | 重命名 |
| `report_inverter_properties_scheduled()` | `iot_report_inverter_scheduled()` | 重命名 |
| `report_inverter_set_param()` | `iot_report_inverter_set_param()` | 重命名 |

### 3.3 ML307R接口扩展（ml307r.h）

```c
// MQTT下行回调类型
typedef void (*mqtt_downlink_cb)(const char *topic, const char *payload);

// 设置下行回调（在MQTT连接成功后调用）
int ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb);

// 获取MQTT连接状态
bool ml307r_mqtt_is_connected(void);
```

### 3.4 ml307r.c 改动

在mqtt_connect成功回调中：
```c
// 连接成功后设置回调
ml307r_mqtt_set_downlink_callback(iot_mqtt_downlink_handler);
```

## 4. 数据结构

### 4.1 iot_tx_flag_t（复用wifi_tx_flag_t）

```c
typedef struct {
    uint16_t restore;
    uint16_t device;
    uint16_t mcu_version;
    // ... 其他标志复用
    uint32_t prop_ct_count;          // CT定时上报计数器(ms)
    uint32_t prop_inv_count;        // INV定时上报计数器(ms)
    uint8_t immediate_report_pair;  // 配对立即上报标志
    uint8_t immediate_report_bind;   // 绑定立即上报标志
    uint8_t immediate_report_unbind; // 解绑立即上报标志
    uint32_t get_down_count;         // get_down轮询计数器
} iot_tx_flag_t;
```

### 4.2 iot_tx_type_t

```c
typedef enum {
    IOT_TX_NULL = 0,
    IOT_PROP_EVENT = 1,      // 属性事件上报
    IOT_GET_DOWN = 4,        // 获取下行命令
    // ... 其他类型复用
} iot_tx_type_t;
```

## 5. 上报策略

### 5.1 即时上报（立即触发）

| 条件 | 触发函数 | 内容 |
|------|----------|------|
| INV告警状态变化 | `iot_report_inverter_properties()` | online_state, work_state, fault_param等 |
| INV设置参数变化 | `iot_report_inverter_set_param()` | connection_point, power_limit等 |
| 配对成功 | properties_changed 3 5 | 绑定SN |
| 解配成功 | properties_changed 3 6 | 解绑SN |

### 5.2 定时上报（周期触发）

| 周期 | 触发函数 | 内容 |
|------|----------|------|
| 5分钟 | `iot_report_ct_siid2()` | CT1/CT2/CT3功率、电压、频率 |
| 5分钟 | `iot_report_ct_siid3()` | 配对列表、功率模式、馈网限值 |
| 3分钟 | `iot_report_inverter_scheduled()` | 发电量、PV输入、累计发电量、温度 |

### 5.3 get_down轮询（200ms）

```c
if (iot_tx_flag.get_down_count >= GET_DOWN_MS_INTERVAL) {
    iot_tx_flag.get_down_count = 0;
    // 通过MQTT发送get_down请求
    ml307r_mqtt_publish("iot/up/get_down", "get_down", 0);
}
```

## 6. 文件改动清单

### 6.1 新增文件

| 文件 | 路径 | 说明 |
|------|------|------|
| iot.h | app/inc/ | IoT模块头文件 |
| iot.c | app/src/ | IoT模块实现 |

### 6.2 修改文件

| 文件 | 改动 |
|------|------|
| app/src/wifi.c | 重命名为 app/src/iot.c（或保留wifi.c作为复用库） |
| app/src/ml307r.c | 添加mqtt_downlink_callback机制 |
| app/src/ml307r.h | 添加mqtt回调相关接口 |
| app/src/main.c | 调用ml307r_task()，由ml307r内部调用iot_task() |

### 6.3 删除文件（可选）

| 文件 | 说明 |
|------|------|
| app/src/wifi.c | 如完全被iot.c取代则删除 |
| app/inc/wifi.h | 如完全被iot.h取代则删除 |

## 7. 错误处理

### 7.1 MQTT断开重连

```c
// 在ml307r_task()中
if (!ml307r_mqtt_is_connected() && ml307r_get_state() == ML307R_STATE_CONNECTED) {
    // 尝试重新连接
    ml307r_mqtt_connect();
}
```

### 7.2 属性上报失败

- MQTT未连接时，属性变化标志位保留
- 连接恢复后立即上报pending的属性变化

### 7.3 get_down超时

- 使用200ms计数器，超时后重新轮询
- 连续超时时累积计数，用于诊断

## 8. 代码复用策略

### 8.1 最小改动原则

尽量复用wifi.c中已有的：
- 属性解析逻辑（wifi_properties_para）
- 属性响应构造（wifi_get/set_properties_reply）
- 属性上报内容构造（report_ct_xxx, report_inverter_xxx）

### 8.2 替换策略

```
wifi.c中的:
  bytes_to_wifi(buffer, len)  →  ml307r_mqtt_publish(topic, buffer, qos)
  wifi_tx_buffer              →  本地临时buffer，通过mqtt_publish发送
```

## 9. 测试要点

1. MQTT连接建立后，IoT任务正常调度
2. 属性变化时能够立即通过MQTT上报
3. 定时属性能够按周期正确上报
4. 云端下发set_properties能够被正确解析和响应
5. 云端下发get_properties能够返回正确属性值
6. MQTT断开后自动重连，恢复后继续工作

## 10. 注意事项

1. **ML307R UART与ESP32 UART可能不同**：需要确认ML307R使用哪个UART
2. **MQTT主题**：需要与云端协商确认正确的主题名称
3. **DMIOT协议兼容性**：确保properties_changed格式与云端期望一致
4. **INV_DEVICE_MAX_NUM**：如需支持更多逆变器，可调整为8

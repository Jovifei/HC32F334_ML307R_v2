# ML307R 工程记忆

## 工程概述

本项目是 **ML307R 4G CAT-M1 模组**的嵌入式驱动实现，基于冲激模组开发板。主要实现 MQTT 物联网云平台接入，包含设备注册(一次)、证书管理、MQTT连接、上行/下行数据通信。

### 核心模块

| 文件                        | 职责                                            |
| --------------------------- | ----------------------------------------------- |
| `app/src/ml307r.c`          | ML307R状态机主逻辑 (Phase 1-7)                  |
| `app/src/uart_at.c`         | AT命令收发、RX解析、URC回调管理                 |
| `app/src/device_register.c` | 设备凭证管理、EEPROM存储                        |
| `app/src/crypto.c`          | MD5 prov_code计算                               |
| `app/src/at_parser.c`       | MQTT命令封装 (config/connect/subscribe/publish) |
| `app/inc/config.h`          | 产品ID/密钥/SN/服务器地址等宏定义               |

---

## IoT模块架构 (iot.c/iot.h)

ESP32 WiFi模块已替换为ML307R 4G模块，新增IoT抽象层：

### 核心文件

| 文件 | 职责 |
|------|------|
| `app/inc/iot.h` | IoT模块头文件，接口声明 |
| `app/src/iot.c` | IoT模块实现 (~940行) |
| `app/src/ml307r.c` | ML307R状态机 + MQTT回调集成 |

### 主要接口

```c
void iot_task(void);                           // 主任务（MQTT连接后调用）
void iot_mqtt_downlink_handler(topic, payload); // MQTT下行回调
void iot_report_immediate(uint8_t inv_idx);    // INV立即属性上报
void iot_report_bind(const char *sn);          // 配对绑定上报
void iot_report_unbind(const char *sn);        // 解绑上报
void iot_trigger_ct_report(void);              // CT定时上报触发
void iot_trigger_inverter_report(uint8_t inv_idx); // INV定时上报触发
bool iot_is_connected(void);                   // 获取连接状态
```

### MQTT通信

- **下行**: 订阅 `down` 主题，JSON格式消息通过回调处理
- **上行**: 发布到 `up` 主题，JSON格式

```json
// 上报属性 (properties_changed)
{"id":123,"method":"properties_changed","params":[{"siid":2,"piid":1,"value":0},...]}

// 下行命令
{"id":123,"method":"get_properties","params":[{"siid":1,"piid":2},...]}
```

### 上报策略

| 类型 | 触发条件 | 目标 |
|------|----------|------|
| 即时上报 | INV状态/告警变化 | properties_changed |
| CT定时上报 | 5分钟周期 | siid=2, siid=3 交替 |
| INV定时上报 | 3分钟周期 | siid=4~11 |

### ml307r.h 新增接口

```c
typedef void (*mqtt_downlink_cb)(const char *topic, const char *payload);
void ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb);
bool ml307r_mqtt_is_connected(void);
```

---

## 状态机架构 (ml307r_task)

采用**非阻塞状态机** + **两步AT命令模式**：

```
at_command_start(cmd, timeout_ms) → 启动命令发送
at_command_check() → 查询结果:
  - AT_NB_IDLE   = 还在等待
  - AT_NB_OK     = 收到OK
  - AT_NB_ERR    = 收到ERROR
```

**Phase 1**: AT测试 → ATE0 → CPIN? → CSQ → CEREG(等待30次) → MIPCALL=1,1
**Phase 2**: CA证书写入 (AT+MSSLCERTWR)
**Phase 3**: Client Key + Client Cert 写入
**Phase 4**: HTTPS设备注册 (新设备) / 跳过(已有凭证)
**Phase 5**: SSL双向认证 (MSSLCFG cert + auth)
**Phase 6**: MQTT连接 (MQTTCFG ssl/keepalive/clean → MQTTCONN)
**Phase 7**: MQTT订阅 (down/<PRODUCT_ID>/<device_id>)

---

## 凭证存储机制

**EEPROM地址**: 0x1A0, 192字节 (cred_store_t结构)

**结构体**:

```c
typedef struct __attribute__((packed)) {
    char product_id[32];
    char product_secret[32];
    char product_model[32];
    char device_sn[32];
    char device_id[32];
    char device_key[64];
    uint8_t valid;   // 0x55 = 已注册
    uint8_t crc8;    // CRC8校验
} cred_store_t;
```

**加载流程** (启动时一次性执行):

```c
device_register_init();
device_register_set_info(PRODUCT_ID, PRODUCT_SECRET, PRODUCT_MODEL, PRODUCT_SN);
device_register_load_from_flash();  // 尝试加载EEPROM凭证
```

**判断逻辑**: `device_register_get_state() == DEVICE_REG_SUCCESS` → 跳过阶段4

---

## HTTPS设备注册流程

**URL**: `https://api.cn.dream-maker.com:8443/APIServerV2/tool/mqtt/preset`

**AT命令序列** (ML307R专用MHTTP命令):

```
AT+MSSLCFG="cert",0,"ca.cer"                              // 绑定CA(单向认证用于HTTPS)
AT+MHTTPCREATE="https://api.cn.dream-maker.com:8443"     // 创建HTTP实例
AT+MHTTPCFG="ssl",0,1,0                                   // 绑定SSL
AT+MHTTPCFG="header",0,"Content-Type: application/json"  // 设置Header
AT+MHTTPCONTENT=0,0,<body_len>                            // 进入数据模式
> (收到>后发送JSON body)
AT+MHTTPREQUEST=0,2,0,"/APIServerV2/tool/mqtt/preset"    // POST请求
```

**JSON Body格式**:

```json
{
  "product_id": "<PRODUCT_ID>",
  "sn": "<PRODUCT_SN>",
  "prov_code": "<prov_code>",
  "mark": "++++++"
}
```

**prov_code计算**: `md5_encrypt_code(product_secret, device_sn, code_out)`

- 取MD5(product_secret + device_sn)的第5-12字节(共8字节)转为16位hex字符串

**URC响应解析**: 监听 `+MHTTPURC` 关键字，解析JSON中的 `device_id` 和 `device_key`

---

## AT命令调试技巧

**uart_at.c** 中 `handle_parsed_line()` 已添加调试打印:

```c
DEBUG_4G_PRINTF(" >>> RX: [%s]\r\n", line);
```

**URC回调调试**:

```c
DEBUG_4G_PRINTF(" DBG URC: [%s]\r\n", line);
```

**RX缓冲区内容检查**:

```c
char dbg_buf[64];
int n = at_read_response(dbg_buf, sizeof(dbg_buf));
DEBUG_4G_PRINTF(" RX(%d)=[%s]\r\n", n, dbg_buf);
```

---

## 当前实现状态

### 已完成

- ✅ Phase 1-3: AT测试、证书写入
- ✅ Phase 4: HTTPS注册流程(含URC解析、EEPROM存储)
- ✅ Phase 5: SSL双向认证
- ✅ Phase 6: MQTT连接
- ✅ Phase 7: MQTT订阅
- ✅ 设备凭证EEPROM持久化
- ✅ URC回调机制
- ✅ **IoT抽象层 (iot.c/iot.h)**: MQTT通信抽象，替代ESP32 WiFi
- ✅ **properties_changed上报**: CT设备5分钟周期，INV设备3分钟周期
- ✅ **set_properties处理**: 解析JSON，设置sys_param参数
- ✅ **get_properties处理**: 解析JSON，查询属性值并响应
- ✅ **ML307R MQTT回调机制**: 下行消息通过回调分发

### 待验证

- ⚠️ HTTPS注册URC响应的实际格式 (`+MHTTPURC: "content",...` 的完整内容)
- ⚠️ device_id/device_key解析是否正确
- ⚠️ MQTT连接是否成功 (需观察 `+MQTTURC: "conn",0,0`)
- ⚠️ MQTT订阅Topic格式是否正确

---

## 关键配置 (config.h)

```c
#define PRODUCT_ID        "689adc659f04ec32f7642fbb"
#define PRODUCT_SECRET    "e8127Lx3dG9l3Hy7"    // 用于prov_code计算
#define PRODUCT_MODEL     "CT_METER_V1"
#define PRODUCT_SN         "1234567890"          // 设备序列号
#define MQTT_SERVER       "mqtt.cn.dream-maker.com"
#define MQTT_PORT         8883
```

---

## 注意事项

1. **状态机重入问题**: `at_command_start` 只在 `AT_NB_IDLE` 时才启动新命令
2. **EEPROM初始化**: 只需在启动时执行一次，重复调用 `device_register_init()` 会重置状态
3. **URC回调**: 注册后自动生效，通过 `at_register_urc(keyword, callback)` 实现
4. **非阻塞设计**: 状态机每次调用只处理一个状态，需要多次调用推进
5. **AT+MSSLCERTWR命令格式**: `AT+MSSLCERTWR="<filename>",0,<length>` (无SSL上下文ID参数)
6. **ML307R特定**: 使用 `AT+MHTTPCREATE/MHTTPCFG/MHTTPCONTENT/MHTTPREQUEST` 而非标准HTTP命令

## DMIOT协议参考

协议文档位置: `PDF/DMIOT模组串口指令.docx`, `PDF/DMIOT模组MQTT通信协议.docx`

**关键区别 - 串口协议 vs MQTT协议**:
- 串口: `get_down` 轮询文本命令，`properties_changed 2 1 0\r`
- MQTT: 订阅 `down` 主题接收JSON，发布JSON到 `up` 主题

**JSON格式示例**:
```json
// 上报 properties_changed
{"id":123,"method":"properties_changed","params":[{"siid":2,"piid":1,"value":0}]}

// 响应 result
{"id":123,"method":"result","params":[{"siid":1,"piid":2,"value":10,"code":0}]}
```

---

## 文件位置

```
d:\work\ML307R_F334\
├── app/
│   ├── inc/
│   │   ├── ml307r.h        # 状态机类型定义 + MQTT回调接口
│   │   ├── iot.h           # IoT模块头文件 (新增)
│   │   ├── config.h        # 产品配置宏
│   │   └── device_register.h
│   └── src/
│       ├── ml307r.c        # 主状态机 + MQTT回调集成
│       ├── iot.c           # IoT模块实现 (新增，~940行)
│       ├── uart_at.c       # AT命令收发、URC管理
│       ├── device_register.c # 凭证管理、EEPROM读写
│       └── at_parser.c     # MQTT命令封装
├── docs/superpowers/
│   ├── specs/2026-04-14-iot-ml307r-design.md  # 设计文档
│   └── plans/2026-04-14-iot-ml307r-implementation.md # 实现计划
├── MDK/                    # Keil工程
└── docs/ML307R_串口调试操作步骤.docx  # AT命令文档
```

## Git提交记录

主要commits:
- `720f30c` fix(ml307r): remove duplicate get/set_properties handling
- `391cbf3` chore: delete obsolete wifi.c and wifi.h files
- `59287a5` feat(iot): complete iot.c full property logic
- `8c2ca55` feat(iot): integrate iot_task with ml307r MQTT connection
- `1054ab4` feat(ml307r): implement MQTT callback mechanism
- `6a58ebd` feat(iot): add IoT module header with MQTT interface declarations

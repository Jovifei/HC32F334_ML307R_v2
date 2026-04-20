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
AT+MSSLCFG="cert",0,"ca.cer"
AT+MHTTPCREATE="https://api.cn.dream-maker.com:8443"
AT+MHTTPCFG="ssl",0,1,0
AT+MHTTPCFG="header",0,"Content-Type: application/json"
AT+MHTTPCONTENT=0,0,<body_len>
> (收到>后发送JSON body)
AT+MHTTPREQUEST=0,2,0,"/APIServerV2/tool/mqtt/preset"
```

**prov_code计算**: `md5_encrypt_code(product_secret, device_sn, code_out)`
取MD5(product_secret + device_sn)的第5-12字节(共8字节)转为16位hex字符串

---

## MQTT 下行消息处理

### URC 格式 (ML307R 实际格式)

```
+MQTTURC: "publish",<conn_id>,<qos>,"<topic>",<len>,<len>,{<json>}
```

例如:

```
+MQTTURC: "publish",0,0,"down/689adc659f04ec32f7642fbb/69dde479495848939efba625",65,65,{"id":8,"method":"get_properties","params":[{"siid":1,"piid":6}]}
```

注意：有**两个**重复的 len 字段，payload 是 JSON 对象（不是引号包裹的字符串）。

### 已修复的 Bug (at_parser.c)

**Bug**: `strncmp(line, "+MQTTURC: \"message\"", 18)` 的前18字符与 `"publish"` URC 相同，导致 message 分支误匹配 publish URC，sscanf 解析失败但无 return，继续执行到 `Unknown method`。

**修复** (第125行):

```c
// 修复前（BUG）:
const char *prefix = "+MQTTURC: \"message\"";
// 修复后:
const char *prefix = "+MQTTURC: \"message\",\"";
```

同时在第139行添加 `return;` 防止 sscanf 失败时继续执行。

### 消息分发 (on_mqtt_message → ml307r.c)

支持的 method：

- `get_properties` → `handle_get_properties(buf, msg_id)`
- `set_properties` → `handle_set_properties(buf, msg_id)`
- `time` → `handle_time_sync(buf)`

---

## 当前实现状态

### 已完成

- ✅ Phase 1-7: 完整状态机
- ✅ 设备凭证EEPROM持久化
- ✅ URC回调机制
- ✅ MQTT publish URC 解析 (at_parser.c "publish" 分支)
- ✅ get_properties / set_properties / time 消息分发
- ✅ properties_changed 定时上报
- ✅ main.c 100ms 非阻塞定时门控 iot_task()
- ✅ 修复 message 分支误匹配 publish URC 的 bug

### 待验证

- ⚠️ get_properties / set_properties 烧录后实测响应是否正确
- ⚠️ properties_changed 上报所有参数（当前只上报了电网频率和电网电压）

---

## 关键配置 (config.h)

```c
#define PRODUCT_ID        "689adc659f04ec32f7642fbb"
#define PRODUCT_SECRET    "e8127Lx3dG9l3Hy7"
#define PRODUCT_MODEL     "GC-CTST3C"
#define PRODUCT_SN        "GTEST1000000011"
#define MQTT_SERVER       "mqtt.cn.dream-maker.com"
#define MQTT_PORT         8883
```

---

## 注意事项

1. **状态机重入**: `at_command_start` 只在 `AT_NB_IDLE` 时才启动新命令
2. **EEPROM初始化**: 只需启动时执行一次
3. **URC回调**: 通过 `at_register_urc(keyword, callback)` 注册
4. **非阻塞设计**: 状态机每次调用只处理一个状态
5. **AT+MSSLCERTWR格式**: `AT+MSSLCERTWR="<filename>",0,<length>` (无SSL上下文ID)
6. **ML307R特定HTTP**: 使用 `MHTTPCREATE/MHTTPCFG/MHTTPCONTENT/MHTTPREQUEST`

---

## 文件位置

```
d:\work\ML307R_F334\
├── app/
│   ├── inc/
│   │   ├── ml307r.h
│   │   ├── config.h
│   │   └── device_register.h
│   └── src/
│       ├── ml307r.c        # 主状态机 + MQTT消息处理
│       ├── uart_at.c       # AT命令收发、URC管理
│       ├── device_register.c
│       ├── at_parser.c     # MQTT命令封装 + URC解析
│       └── iot.c           # IoT任务（100ms周期）
├── MDK/                    # Keil工程
└── docs/
```

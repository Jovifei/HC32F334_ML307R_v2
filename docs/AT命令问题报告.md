# ML307R AT命令问题报告

## 概述

本报告基于 `FS-LCore-M307系列用户手册_V1.2-20251215.pdf` 文档，对 `app/src/ml307r.c` 和 `app/src/device_register.c` 中的AT命令发送流程进行分析，指出与文档不一致或可能存在问题的地方。

---

## 1. AT命令格式 ✅ 已正确处理

| 项目 | 状态 | 说明 |
|------|------|------|
| `\r\n` 后缀 | ✅ 正确 | `uart_at.c:164` 自动添加 |

```c
// uart_at.c:164
snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
```

---

## 2. 初始化流程不完整 ⚠️ 需修改

### 文档要求 (4.3节)

```
1. AT                 -> "OK"
2. AT+CPIN?          -> "READY"
3. AT+CSQ            -> 信号质量 10~31
4. AT+CEREG?         -> 网络注册状态
5. AT+CGATT?         -> 附着状态 0/1
```

### 代码实现对比

| 步骤 | 文档要求 | 代码实现 | 状态 |
|------|----------|----------|------|
| 1 | `AT` | ✅ `AT` | 正确 |
| 2 | `AT+CPIN?` | ✅ `AT+CPIN?` | 正确 |
| 3 | `AT+CSQ` | ❌ 未执行 | **缺失** |
| 4 | `AT+CEREG?` | ✅ `AT+CEREG?` | 正确 |
| 5 | `AT+CGATT?` | ❌ 未执行 | **缺失** |

### 需修改位置

**文件**: `app/src/ml307r.c`

**函数**: `ml307r_init()`

**建议在 `AT+CPIN?` 之后、`AT+CEREG?` 之前添加**：

```c
// --- 文档步骤3: 信号质量检查 ---
DEBUG_4G_PRINTF(" >>> AT send: AT+CSQ\r\n");
ret = at_send_command("AT+CSQ", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
DEBUG_4G_PRINTF(" <<< AT+CSQ resp: %s, ret=%d\r\n", resp, ret);
if (ret != 0)
{
    DEBUG_4G_PRINTF(" !!! Signal quality check failed !!!\r\n");
    s_ml_state = ML307R_STATE_ERROR;
    return -1;
}

// 解析信号强度
int rssi = 99;
if (strstr(resp, "+CSQ:") != NULL)
{
    sscanf(strstr(resp, "+CSQ:"), "+CSQ: %d,%*d", &rssi);
    DEBUG_4G_PRINTF(" Signal: RSSI=%d\r\n", rssi);
    if (rssi < 10 || rssi > 31)
    {
        DEBUG_4G_PRINTF(" !!! Signal quality poor (RSSI=%d) !!!\r\n", rssi);
        s_ml_state = ML307R_STATE_ERROR;
        return -1;
    }
}

// --- 文档步骤5: 附着状态检查 ---
DEBUG_4G_PRINTF(" >>> AT send: AT+CGATT?\r\n");
ret = at_send_command("AT+CGATT?", "OK", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
DEBUG_4G_PRINTF(" <<< AT+CGATT? resp: %s, ret=%d\r\n", resp, ret);
if (ret != 0 || strstr(resp, "+CGATT: 1") == NULL)
{
    DEBUG_4G_PRINTF(" !!! GPRS detach status !!!\r\n");
    s_ml_state = ML307R_STATE_ERROR;
    return -1;
}
```

---

## 3. HTTP数据发送流程 ⚠️ 需修改

### 问题描述

**文件**: `app/src/device_register.c`

**函数**: `device_register_request()`

**问题**: `AT+HTTPDATA` 命令发送数据后，代码直接 `delay_ms(500)` 然后发送 `AT+HTTPACTION=1`，没有确认模组是否正确接收数据。

### 当前代码 (第196-203行)

```c
char data_cmd[32];
snprintf(data_cmd, sizeof(data_cmd), "AT+HTTPDATA=%d,10000", (int)strlen(body));
ret = at_send_command(data_cmd, "DOWNLOAD", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
if (ret != 0)
    goto cleanup;

at_send_raw((uint8_t*)body, strlen(body));
delay_ms(500);  // ⚠️ 盲目延时，没有确认
```

### 建议修改

```c
char data_cmd[32];
snprintf(data_cmd, sizeof(data_cmd), "AT+HTTPDATA=%d,10000", (int)strlen(body));
ret = at_send_command(data_cmd, "DOWNLOAD", AT_TIMEOUT_DEFAULT, resp, sizeof(resp));
if (ret != 0)
    goto cleanup;

// 发送数据内容
at_send_raw((uint8_t*)body, strlen(body));

// 添加数据结束标记 (某些模组需要)
uint8_t crlf[2] = {'\r', '\n'};
at_send_raw(crlf, 2);

// 等待模组确认 (收到 > 提示符后发送数据，然后等待 OK)
uint32_t start = sys_param.timer.timer_1ms_count;
while (s_rx_head == s_rx_tail)
{
    if ((uint32_t)(sys_param.timer.timer_1ms_count - start) >= 5000)
    {
        DEBUG_4G_PRINTF(" !!! HTTPDATA timeout !!!\r\n");
        goto cleanup;
    }
}
at_flush_rx();

// 延时确保模组处理完毕
delay_ms(500);
```

---

## 4. MQTT命令格式 ⚠️ 需确认

### 问题描述

代码中MQTT命令格式与文档略有差异，需确认是否影响功能。

### 文档示例

```
AT+MQTTCONN=0,"broker.emqx.io",1883,"client_id","username","password"
AT+MQTTSUB=0,"topic",0
AT+MQTTPUB=0,"topic",0,0,0,5,"HELLO"
```

### 代码实现 (at_parser.c)

```c
// AT+MQTTCONN=0,"host",port,"client_id","username","password"
snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=%d,\"%s\",%d,\"%s\",\"%s\",\"%s\"",
         MQTT_CONN_ID, host, port, client_id, username, password);

// AT+MQTTPUB=0,"topic",qos,0,0,data_len,"data"
snprintf(cmd, sizeof(cmd), "AT+MQTTPUB=%d,\"%s\",%d,0,0,%d,\"%s\"",
         MQTT_CONN_ID, topic, qos, data_len, data);
```

**差异**: 代码中MQTTPUB命令有额外的 `0,0` 参数，建议对照文档确认是否为保留参数。

---

## 5. 总结

| 序号 | 问题 | 严重程度 | 文件 | 建议 |
|------|------|----------|------|------|
| 1 | 缺少 `AT+CSQ` 信号质量检查 | 中 | ml307r.c | 添加 |
| 2 | 缺少 `AT+CGATT?` 附着状态检查 | 中 | ml307r.c | 添加 |
| 3 | `AT+HTTPDATA` 后缺少确认机制 | 高 | device_register.c | 修改 |
| 4 | MQTT命令参数需确认 | 低 | at_parser.c | 确认 |

---

## 6. 修改优先级

1. **高优先级**: 修复 `AT+HTTPDATA` 数据发送流程
2. **中优先级**: 添加 `AT+CSQ` 和 `AT+CGATT?` 检查
3. **低优先级**: 确认MQTT命令参数

---

*报告生成时间: 2026-04-09*

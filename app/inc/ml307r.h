#ifndef ML307R_H
#define ML307R_H

#include <stdbool.h>

/*============================================================================
 ML307R 4G模组接口定义
============================================================================*/

/**
 * ML307R模组状态枚举
 * - ML307R_STATE_INIT: 初始化状态
 * - ML307R_STATE_SIM_CHECK: SIM卡检查状态
 * - ML307R_STATE_REGISTERED: 已完成注册
 * - ML307R_STATE_DIAL: PDP激活状态
 * - ML307R_STATE_CONNECTED: 连接成功
 * - ML307R_STATE_ERROR: 错误状态
 */
typedef enum {
    ML307R_STATE_INIT = 0,
    ML307R_STATE_SIM_CHECK,
    ML307R_STATE_REGISTERED,
    ML307R_STATE_DIAL,
    ML307R_STATE_CONNECTED,
    ML307R_STATE_ERROR
} ml307r_state_t;

/**
 * MQTT客户端状态枚举
 * - MQTT_STATE_DISCONNECTED: 断开连接
 * - MQTT_STATE_CONNECTING: 连接中
 * - MQTT_STATE_CONNECTED: 已连接
 * - MQTT_STATE_ERROR: 错误状态
 */
typedef enum {
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

/**
 * 信号质量结构体
 * - rssi: 信号强度指示(0-31, 99=未知)
 * - ber: 误码率(0-7, 99=未知)
 */
typedef struct {
    int rssi;   // 0-31, 99=unknown
    int ber;    // 0-7, 99=unknown
} signal_quality_t;

/*============================================================================
 ML307R AT协议错误码（参考 docs/错误码.md）
============================================================================*/

/* SSL 错误码（参考SSL用户手册 V5.4.5 第41页） */
#define ML307R_SSL_ERR_PARAM                         (50)  // 参数错误
#define ML307R_SSL_ERR_UNKNOWN                       (750) // SSL/TLS/DTLS 未知错误
#define ML307R_SSL_ERR_INIT_RESOURCE                 (751) // SSL/TLS/DTLS 初始化资源错误
#define ML307R_SSL_ERR_SERVER_CERT_VERIFY_FAIL       (752) // SSL/TLS/DTLS 服务器证书验证失败
#define ML307R_SSL_ERR_NEGOTIATE_TIMEOUT             (753) // SSL/TLS/DTLS 协商超时
#define ML307R_SSL_ERR_NEGOTIATE_FAIL                (754) // SSL/TLS/DTLS 协商失败
#define ML307R_SSL_ERR_CERTKEY_UNKNOWN               (760) // CERTS/KEYS 未知错误
#define ML307R_SSL_ERR_CERTKEY_INVALID               (761) // CERTS/KEYS 无效（格式/内容错误）
#define ML307R_SSL_ERR_CERTKEY_NOT_EXIST             (762) // CERTS/KEYS 不存在
#define ML307R_SSL_ERR_CERTKEY_ALREADY_EXIST         (763) // CERTS/KEYS 已存在同名证书/密钥
#define ML307R_SSL_ERR_CERTKEY_WRITE_FAIL            (764) // CERTS/KEYS 写入失败
#define ML307R_SSL_ERR_CERTKEY_BUSY_WRITING          (765) // CERTS/KEYS 正在证书/密钥写入中
#define ML307R_SSL_ERR_CERTKEY_READ_FAIL             (766) // CERTS/KEYS 读取失败
#define ML307R_SSL_ERR_CERTKEY_DELETE_FAIL           (767) // CERTS/KEYS 删除失败
#define ML307R_SSL_ERR_CERTKEY_TOO_LARGE             (768) // CERTS/KEYS 过大
#define ML307R_SSL_ERR_CERTKEY_LOAD_FAIL             (769) // CERTS/KEYS 加载失败

/* MQTT/MQTTS 错误码（参考MQTT用户手册 V6.8.5 第36页） */
#define ML307R_MQTT_ERR_UNKNOWN                      (600) // 未知错误
#define ML307R_MQTT_ERR_INVALID_PARAM                (601) // 无效参数
#define ML307R_MQTT_ERR_NOT_CONNECTED_OR_CONN_FAIL   (602) // 未连接或连接失败
#define ML307R_MQTT_ERR_CONNECTING                   (603) // 连接中
#define ML307R_MQTT_ERR_ALREADY_CONNECTED            (604) // 已经连接
#define ML307R_MQTT_ERR_NETWORK                      (605) // 网络错误
#define ML307R_MQTT_ERR_STORAGE                      (606) // 存储错误
#define ML307R_MQTT_ERR_STATE                        (607) // 状态错误
#define ML307R_MQTT_ERR_DNS                          (608) // DNS错误

/* HTTP/HTTPS 错误码（参考HTTP_HTTPS用户手册 V6.1.4 第46页） */
#define ML307R_HTTP_ERR_OPERATION_NOT_ALLOWED        (3)   // 操作不允许
#define ML307R_HTTP_ERR_MALLOC_FAIL                  (23)  // 内存分配失败
#define ML307R_HTTP_ERR_PARAM                        (50)  // 参数错误
#define ML307R_HTTP_ERR_UNKNOWN                      (650) // 未知错误
#define ML307R_HTTP_ERR_NO_FREE_CLIENT               (651) // 无空闲客户端
#define ML307R_HTTP_ERR_CLIENT_NOT_CREATED           (652) // 客户端未创建
#define ML307R_HTTP_ERR_CLIENT_BUSY                  (653) // 客户端忙
#define ML307R_HTTP_ERR_URL_PARSE_FAIL               (654) // URL解析失败
#define ML307R_HTTP_ERR_SSL_NOT_ENABLED              (655) // SSL未使能
#define ML307R_HTTP_ERR_CONNECT_FAIL                 (656) // 连接失败
#define ML307R_HTTP_ERR_SEND_FAIL                    (657) // 数据发送失败
#define ML307R_HTTP_ERR_OPEN_FILE_FAIL               (658) // 打开文件失败

/* HTTP/HTTPS URC事件类型（+MHTTPURC: "err",<httpid>,<error_code> 第35页） */
#define ML307R_HTTP_URC_ERR_DNS_RESOLVE_FAIL         (1)   // 域名解析失败
#define ML307R_HTTP_URC_ERR_CONNECT_SERVER_FAIL      (2)   // 连接服务器失败
#define ML307R_HTTP_URC_ERR_CONNECT_SERVER_TIMEOUT   (3)   // 连接服务器超时
#define ML307R_HTTP_URC_ERR_SSL_HANDSHAKE_FAIL       (4)   // SSL握手失败
#define ML307R_HTTP_URC_ERR_CONN_ABNORMAL_DISCONNECT (5)   // 连接异常断开
#define ML307R_HTTP_URC_ERR_RESPONSE_TIMEOUT         (6)   // 等待响应超时
#define ML307R_HTTP_URC_ERR_RECV_PARSE_FAIL          (7)   // 接收数据解析失败
#define ML307R_HTTP_URC_ERR_CACHE_NOT_ENOUGH         (8)   // 缓存空间不足
#define ML307R_HTTP_URC_ERR_PACKET_LOSS              (9)   // 数据丢包
#define ML307R_HTTP_URC_ERR_WRITE_FILE_FAIL          (10)  // 写文件失败
#define ML307R_HTTP_URC_ERR_UNKNOWN                  (255) // 未知错误

/* TCP/UDP/DNS/PING 错误码（参考TCP_IP用户手册 V5.1.5 第71页） */
#define ML307R_TCPIP_ERR_UNKNOWN                     (550) // TCP/IP 未知错误
#define ML307R_TCPIP_ERR_NOT_USED                    (551) // TCP/IP 未被使用
#define ML307R_TCPIP_ERR_ALREADY_USED                (552) // TCP/IP 已被使用
#define ML307R_TCPIP_ERR_NOT_CONNECTED               (553) // TCP/IP 未连接
#define ML307R_TCPIP_ERR_SOCKET_CREATE_FAIL          (554) // SOCKET 创建失败
#define ML307R_TCPIP_ERR_SOCKET_BIND_FAIL            (555) // SOCKET 绑定失败
#define ML307R_TCPIP_ERR_SOCKET_LISTEN_FAIL          (556) // SOCKET 监听失败
#define ML307R_TCPIP_ERR_SOCKET_CONN_REFUSED         (557) // SOCKET 连接被拒绝
#define ML307R_TCPIP_ERR_SOCKET_CONN_TIMEOUT         (558) // SOCKET 连接超时
#define ML307R_TCPIP_ERR_SOCKET_CONN_FAIL            (559) // SOCKET 连接失败（网络异常）
#define ML307R_TCPIP_ERR_SOCKET_WRITE_FAIL           (560) // SOCKET 写入异常
#define ML307R_TCPIP_ERR_SOCKET_READ_FAIL            (561) // SOCKET 读取异常
#define ML307R_TCPIP_ERR_SOCKET_ACCEPT_FAIL          (562) // SOCKET 接受异常
#define ML307R_TCPIP_ERR_PDP_NOT_ACTIVATED           (570) // PDP 未激活
#define ML307R_TCPIP_ERR_PDP_ACTIVATE_FAIL           (571) // PDP 激活失败
#define ML307R_TCPIP_ERR_PDP_DEACTIVATE_FAIL         (572) // PDP 去激活失败
#define ML307R_TCPIP_ERR_APN_NOT_CONFIGURED          (575) // APN 未配置
#define ML307R_TCPIP_ERR_PORT_BUSY                   (576) // 端口忙碌
#define ML307R_TCPIP_ERR_UNSUPPORTED_IPV4_IPV6       (577) // 不支持的IPV4/IPV6
#define ML307R_TCPIP_ERR_DNS_PARSE_FAIL_OR_BAD_IP    (580) // DNS解析失败或错误IP格式
#define ML307R_TCPIP_ERR_DNS_BUSY                    (581) // DNS忙碌
#define ML307R_TCPIP_ERR_PING_BUSY                   (582) // PING忙碌

/* PING 结果码（+MPING: <result>，TCP_IP用户手册 V5.1.5 第56页） */
#define ML307R_PING_RESULT_OK                        (0) // 成功
#define ML307R_PING_RESULT_DNS_RESOLVE_FAIL          (1) // DNS 解析失败
#define ML307R_PING_RESULT_DNS_RESOLVE_TIMEOUT       (2) // DNS 解析超时
#define ML307R_PING_RESULT_RESPONSE_ERROR            (3) // 响应错误
#define ML307R_PING_RESULT_RESPONSE_TIMEOUT          (4) // 响应超时
#define ML307R_PING_RESULT_OTHER_ERROR               (5) // 其他错误

/*============================================================================
 ML307R 4G模组对外接口
============================================================================*/

/**
 * ML307R模组初始化
 * 执行AT通信测试、关闭回显、SIM卡检查、网络注册、PDP激活
 * @return 0=成功, -1=失败
 */
int ml307r_init(void);

/**
 * 获取ML307R模组当前状态
 * @return ml307r_state_t 当前状态
 */
ml307r_state_t ml307r_get_state(void);

/**
 * 获取ML307R模组信号质量
 * 通过AT+CSQ命令查询RSSI和BER
 * @param sq 信号质量结构体指针，用于输出
 * @return 0=成功, -1=失败
 */
int ml307r_get_signal_quality(signal_quality_t *sq);

/**
 * 判断ML307R模组是否欠费
 * 条件：RSSI在10-31范围内(信号良好)，但状态不是已注册/激活/连接中
 * @return true=欠费, false=正常
 */
bool ml307r_is_arrears(void);

/**
 * ML307R模组重新连接
 * 关闭无线功能、开启无线功能，等待网络注册、重新激活PDP
 * @return 0=成功, -1=失败
 */
int ml307r_reconnect(void);

/*============================================================================
 MQTT客户端接口
============================================================================*/

/**
 * MQTT客户端连接
 * 获取设备凭证、构造Topic、配置MQTT、建立连接、订阅下行Topic
 * @return 0=成功, -1=失败
 */
int ml307r_mqtt_connect(void);

/**
 * MQTT客户端断开连接
 * @return 0=成功, -1=失败
 */
int ml307r_mqtt_disconnect(void);

/**
 * MQTT发布消息
 * @param topic 主题
 * @param payload 消息载荷
 * @param qos 消息服务质量(0/1)
 * @return 0=成功, -1=失败
 */
int ml307r_mqtt_publish(const char *topic, const char *payload, int qos);

/**
 * 获取MQTT客户端当前状态
 * @return mqtt_state_t 当前状态
 */
mqtt_state_t ml307r_mqtt_get_state(void);

/**
 * 查询MQTT是否处于已连接且可发布状态
 * @return true=可发布, false=未连接
 */
bool ml307r_mqtt_is_connected(void);

/*============================================================================
 主任务接口
============================================================================*/

/**
 * ML307R主任务
 * 在主循环中调用，负责设备注册、模组初始化、MQTT连接、循环上报
 */
void ml307r_task(void);

#endif // ML307R_H

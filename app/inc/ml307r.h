#ifndef ML307R_H
#define ML307R_H

#include <stdbool.h>

/*============================================================================
 ML307R 4Gģ�����Ͷ���
============================================================================*/

/**
 * ML307Rģ��״̬ö��
 * - ML307R_STATE_INIT: ��ʼ��״̬
 * - ML307R_STATE_SIM_CHECK: SIM�����״̬
 * - ML307R_STATE_REGISTERED: ������ע��
 * - ML307R_STATE_DIAL: PDP����״̬
 * - ML307R_STATE_CONNECTED: ���ӳɹ�
 * - ML307R_STATE_ERROR: ����״̬
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
 * MQTT�ͻ���״̬ö��
 * - MQTT_STATE_DISCONNECTED: �Ͽ�����
 * - MQTT_STATE_CONNECTING: ������
 * - MQTT_STATE_CONNECTED: ������
 * - MQTT_STATE_ERROR: ����״̬
 */
typedef enum {
    MQTT_STATE_DISCONNECTED = 0,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

/**
 * �ź������ṹ��
 * - rssi: �ź�ǿ��ָʾ(0-31, 99=δ֪)
 * - ber: ������(0-7, 99=δ֪)
 */
typedef struct {
    int rssi;   // 0-31, 99=unknown
    int ber;    // 0-7, 99=unknown
} signal_quality_t;

/*============================================================================
 ML307R ATЭ������루���� docs/������.md��
============================================================================*/

/* SSL ��������루SSL�û��ֲ� V5.4.5����41ҳ�� */
#define ML307R_SSL_ERR_PARAM                         (50)  // ��������
#define ML307R_SSL_ERR_UNKNOWN                       (750) // SSL/TLS/DTLS δ֪����
#define ML307R_SSL_ERR_INIT_RESOURCE                 (751) // SSL/TLS/DTLS ��ʼ����Դ����
#define ML307R_SSL_ERR_SERVER_CERT_VERIFY_FAIL       (752) // SSL/TLS/DTLS ������֤����֤ʧ��
#define ML307R_SSL_ERR_NEGOTIATE_TIMEOUT             (753) // SSL/TLS/DTLS Э�̳�ʱ
#define ML307R_SSL_ERR_NEGOTIATE_FAIL                (754) // SSL/TLS/DTLS Э��ʧ��
#define ML307R_SSL_ERR_CERTKEY_UNKNOWN               (760) // CERTS/KEYS δ֪����
#define ML307R_SSL_ERR_CERTKEY_INVALID               (761) // CERTS/KEYS ��Ч����ʽ/���ݴ���
#define ML307R_SSL_ERR_CERTKEY_NOT_EXIST             (762) // CERTS/KEYS ������
#define ML307R_SSL_ERR_CERTKEY_ALREADY_EXIST         (763) // CERTS/KEYS �Ѵ���ͬ����֤�����Կ
#define ML307R_SSL_ERR_CERTKEY_WRITE_FAIL            (764) // CERTS/KEYS д�����
#define ML307R_SSL_ERR_CERTKEY_BUSY_WRITING          (765) // CERTS/KEYS ����֤��/��Կ����д����
#define ML307R_SSL_ERR_CERTKEY_READ_FAIL             (766) // CERTS/KEYS ��ȡ����
#define ML307R_SSL_ERR_CERTKEY_DELETE_FAIL           (767) // CERTS/KEYS ɾ������
#define ML307R_SSL_ERR_CERTKEY_TOO_LARGE             (768) // CERTS/KEYS ����
#define ML307R_SSL_ERR_CERTKEY_LOAD_FAIL             (769) // CERTS/KEYS ����ʧ��

/* MQTT/MQTTS ��������루MQTT�û��ֲ� V6.8.5����36ҳ�� */
#define ML307R_MQTT_ERR_UNKNOWN                      (600) // δ֪����
#define ML307R_MQTT_ERR_INVALID_PARAM                (601) // ��Ч����
#define ML307R_MQTT_ERR_NOT_CONNECTED_OR_CONN_FAIL   (602) // δ���ӻ�����ʧ��
#define ML307R_MQTT_ERR_CONNECTING                   (603) // ��������
#define ML307R_MQTT_ERR_ALREADY_CONNECTED            (604) // �Ѿ�����
#define ML307R_MQTT_ERR_NETWORK                      (605) // �������
#define ML307R_MQTT_ERR_STORAGE                      (606) // �洢����
#define ML307R_MQTT_ERR_STATE                        (607) // ״̬����
#define ML307R_MQTT_ERR_DNS                          (608) // DNS����

/* HTTP/HTTPS ��������루HTTP_HTTPS�û��ֲ� V6.1.4����46ҳ�� */
#define ML307R_HTTP_ERR_OPERATION_NOT_ALLOWED        (3)   // ������������
#define ML307R_HTTP_ERR_MALLOC_FAIL                  (23)  // �ڴ����ʧ��
#define ML307R_HTTP_ERR_PARAM                        (50)  // ��������
#define ML307R_HTTP_ERR_UNKNOWN                      (650) // δ֪����
#define ML307R_HTTP_ERR_NO_FREE_CLIENT               (651) // �޿��пͻ���
#define ML307R_HTTP_ERR_CLIENT_NOT_CREATED           (652) // �ͻ���δ����
#define ML307R_HTTP_ERR_CLIENT_BUSY                  (653) // �ͻ���æ
#define ML307R_HTTP_ERR_URL_PARSE_FAIL               (654) // URL����ʧ��
#define ML307R_HTTP_ERR_SSL_NOT_ENABLED              (655) // SSLδʹ��
#define ML307R_HTTP_ERR_CONNECT_FAIL                 (656) // ����ʧ��
#define ML307R_HTTP_ERR_SEND_FAIL                    (657) // ���ݷ���ʧ��
#define ML307R_HTTP_ERR_OPEN_FILE_FAIL               (658) // ���ļ�ʧ��

/* HTTP/HTTPS URC�����¼���+MHTTPURC: "err",<httpid>,<error_code>����35ҳ�� */
#define ML307R_HTTP_URC_ERR_DNS_RESOLVE_FAIL         (1)   // ��������ʧ��
#define ML307R_HTTP_URC_ERR_CONNECT_SERVER_FAIL      (2)   // ���ӷ�����ʧ��
#define ML307R_HTTP_URC_ERR_CONNECT_SERVER_TIMEOUT   (3)   // ���ӷ�������ʱ
#define ML307R_HTTP_URC_ERR_SSL_HANDSHAKE_FAIL       (4)   // SSL����ʧ��
#define ML307R_HTTP_URC_ERR_CONN_ABNORMAL_DISCONNECT (5)   // �����쳣�Ͽ�
#define ML307R_HTTP_URC_ERR_RESPONSE_TIMEOUT         (6)   // ������Ӧ��ʱ
#define ML307R_HTTP_URC_ERR_RECV_PARSE_FAIL          (7)   // �������ݽ���ʧ��
#define ML307R_HTTP_URC_ERR_CACHE_NOT_ENOUGH         (8)   // ����ռ䲻��
#define ML307R_HTTP_URC_ERR_PACKET_LOSS              (9)   // ���ݶ���
#define ML307R_HTTP_URC_ERR_WRITE_FILE_FAIL          (10)  // д�ļ�ʧ��
#define ML307R_HTTP_URC_ERR_UNKNOWN                  (255) // δ֪����

/* TCP/UDP/DNS/PING �����루TCP_IP�û��ֲ� V5.1.5����71ҳ�� */
#define ML307R_TCPIP_ERR_UNKNOWN                     (550) // TCP/IP δ֪����
#define ML307R_TCPIP_ERR_NOT_USED                    (551) // TCP/IP δ��ʹ��
#define ML307R_TCPIP_ERR_ALREADY_USED                (552) // TCP/IP �ѱ�ʹ��
#define ML307R_TCPIP_ERR_NOT_CONNECTED               (553) // TCP/IP δ����
#define ML307R_TCPIP_ERR_SOCKET_CREATE_FAIL          (554) // SOCKET ����ʧ��
#define ML307R_TCPIP_ERR_SOCKET_BIND_FAIL            (555) // SOCKET ��ʧ��
#define ML307R_TCPIP_ERR_SOCKET_LISTEN_FAIL          (556) // SOCKET ����ʧ��
#define ML307R_TCPIP_ERR_SOCKET_CONN_REFUSED         (557) // SOCKET ���ӱ��ܾ�
#define ML307R_TCPIP_ERR_SOCKET_CONN_TIMEOUT         (558) // SOCKET ���ӳ�ʱ
#define ML307R_TCPIP_ERR_SOCKET_CONN_FAIL            (559) // SOCKET ����ʧ�ܣ������쳣��
#define ML307R_TCPIP_ERR_SOCKET_WRITE_FAIL           (560) // SOCKET д���쳣
#define ML307R_TCPIP_ERR_SOCKET_READ_FAIL            (561) // SOCKET ��ȡ�쳣
#define ML307R_TCPIP_ERR_SOCKET_ACCEPT_FAIL          (562) // SOCKET �����쳣
#define ML307R_TCPIP_ERR_PDP_NOT_ACTIVATED           (570) // PDP δ����
#define ML307R_TCPIP_ERR_PDP_ACTIVATE_FAIL           (571) // PDP ����ʧ��
#define ML307R_TCPIP_ERR_PDP_DEACTIVATE_FAIL         (572) // PDP ȥ����ʧ��
#define ML307R_TCPIP_ERR_APN_NOT_CONFIGURED          (575) // APN δ����
#define ML307R_TCPIP_ERR_PORT_BUSY                   (576) // �˿�æµ
#define ML307R_TCPIP_ERR_UNSUPPORTED_IPV4_IPV6       (577) // ��֧�ֵ�IPV4/IPV6
#define ML307R_TCPIP_ERR_DNS_PARSE_FAIL_OR_BAD_IP    (580) // DNS����ʧ�ܻ�����IP��ʽ
#define ML307R_TCPIP_ERR_DNS_BUSY                    (581) // DNSæµ
#define ML307R_TCPIP_ERR_PING_BUSY                   (582) // PINGæµ

/* PING ����루+MPING: <result>��TCP_IP�û��ֲ� V5.1.5����56ҳ�� */
#define ML307R_PING_RESULT_OK                        (0) // �ɹ�
#define ML307R_PING_RESULT_DNS_RESOLVE_FAIL          (1) // DNS ����ʧ��
#define ML307R_PING_RESULT_DNS_RESOLVE_TIMEOUT       (2) // DNS ������ʱ
#define ML307R_PING_RESULT_RESPONSE_ERROR            (3) // ��Ӧ����
#define ML307R_PING_RESULT_RESPONSE_TIMEOUT          (4) // ��Ӧ��ʱ
#define ML307R_PING_RESULT_OTHER_ERROR               (5) // ��������

/*============================================================================
 ML307R 4Gģ������ӿ�
============================================================================*/

/**
 * ML307Rģ���ʼ��
 * ִ��ATͨ�Ų��ԡ��رջ��ԡ�SIM����顢����ע�ᡢPDP����
 * @return 0=�ɹ�, -1=ʧ��
 */
int ml307r_init(void);

/**
 * ��ȡML307Rģ�鵱ǰ״̬
 * @return ml307r_state_t ��ǰ״̬
 */
ml307r_state_t ml307r_get_state(void);

/**
 * ��ȡML307Rģ���ź�����
 * ͨ��AT+CSQ�����ѯRSSI��BER
 * @param sq �ź������ṹ��ָ�룬�������
 * @return 0=�ɹ�, -1=ʧ��
 */
int ml307r_get_signal_quality(signal_quality_t *sq);

/**
 * �ж�ML307Rģ���Ƿ�Ƿ��
 * ������RSSI��10-31��Χ��(�ź�����)����״̬������ע��/����/������
 * @return true=Ƿ��, false=����
 */
bool ml307r_is_arrears(void);

/**
 * ML307Rģ����������
 * �ر���Ƶ��������Ƶ���ȴ�����ע�ᡢ���¼���PDP
 * @return 0=�ɹ�, -1=ʧ��
 */
int ml307r_reconnect(void);

/*============================================================================
 MQTT�ͻ��˽ӿ�
============================================================================*/

/**
 * MQTT�ͻ�������
 * ��ȡ�豸ƾ�ݡ�����Topic������MQTT���������ӡ���������Topic
 * @return 0=�ɹ�, -1=ʧ��
 */
int ml307r_mqtt_connect(void);

/**
 * MQTT�ͻ��˶Ͽ�����
 * @return 0=�ɹ�, -1=ʧ��
 */
int ml307r_mqtt_disconnect(void);

/**
 * MQTT������Ϣ
 * @param topic ����
 * @param payload ��Ϣ�غ�
 * @param qos ���������ȼ�(0/1)
 * @return 0=�ɹ�, -1=ʧ��
 */
int ml307r_mqtt_publish(const char *topic, const char *payload, int qos);

/**
 * ��ȡMQTT�ͻ��˵�ǰ״̬
 * @return mqtt_state_t ��ǰ״̬
 */
mqtt_state_t ml307r_mqtt_get_state(void);

/*============================================================================
 ������ӿ�
============================================================================*/

/**
 * ML307R��������
 * ����ѭ���е��ã������豸ע�ᡢ�����ʼ����MQTT���ӡ���ѭ���ϱ�
 */
void ml307r_task(void);

/*============================================================================
 MQTT回调接口
============================================================================*/

/**
 * MQTT下行数据回调类型
 * @param topic 主题名称 (如 "down")
 * @param payload JSON数据指针 (只读)
 */
typedef void (*mqtt_downlink_cb)(const char *topic, const char *payload);

/**
 * 设置MQTT下行回调
 * @param cb 回调函数指针，MQTT收到down主题消息时调用
 */
void ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb);

/**
 * 获取MQTT连接状态
 * @return true=已连接, false=未连接
 */
bool ml307r_mqtt_is_connected(void);

#endif // ML307R_H

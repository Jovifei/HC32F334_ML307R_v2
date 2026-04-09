#ifndef UART_AT_H
#define UART_AT_H

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 UART ATͨ�Žӿ�
 ������ML307R 4Gģ��ͨ�ŵ�AT����ͨ��
============================================================================*/

/**
 * ��ʼ��UART2�ӿ�
 * ����115200 8N1��ʹ��RX�ж�
 */
void uart_at_init(void);

/**
 * ����AT����ȴ���������Ӧ
 * �����ȴ�ֱ���յ�expected_ok�ַ�����ʱ
 * @param cmd AT�����ַ���
 * @param expected_ok ��������Ӧ�ؼ���
 * @param timeout_ms ��ʱʱ��(����)
 * @param response ��Ӧ���������������
 * @param resp_len ��Ӧ����������
 * @return 0=�ɹ�, -1=��ʱ, -2=����
 */
int at_send_command(const char *cmd, const char *expected_ok,
                    uint32_t timeout_ms, char *response, int resp_len);

/**
 * ����ԭʼ����(������Ӧ)
 * ����XMODEM OTA����
 * @param data ����ָ��
 * @param len ���ݳ���
 */
void at_send_raw(const uint8_t *data, uint16_t len);

/**
 * ע��URC( unsolicited result code)�ص�
 * ���յ�����keyword����Ϣʱ�Զ�����callback
 * @param keyword �ؼ���
 * @param callback �ص�����
 */
typedef void (*urc_callback_t)(const char *line);
void at_register_urc(const char *keyword, urc_callback_t callback);

/**
 * ���RX������
 */
void at_flush_rx(void);

/**
 * ����UART���������ݣ����������е��ã�
 * ���������յ��е������URC�ص�
 */
void uart_at_process(void);

#endif // UART_AT_H

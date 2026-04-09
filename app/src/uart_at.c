#include "uart_at.h"
#include "config.h"
#include "hc32_ll.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define UART_AT_RX_BUF_SIZE 256

// ==================== é™æ€å˜ï¿?? ====================

static uint8_t s_rx_buf[256];
static volatile uint16_t s_rx_head =
    0;                         // å†™æŒ‡é’ˆï¼ˆä¸­æ–­å†™å…¥ï¿??
static uint16_t s_rx_tail = 0; // è¯»æŒ‡é’ˆï¼ˆä»»åŠ¡è¯»å–ï¿??

// å‘½ä»¤ç»“æœ: 0=ç­‰å¾…ï¿??, 1=OK, -1=ERROR, -1=è¶…æ—¶
static volatile int8_t s_cmd_result = 0;

// RXæ•°æ®å°±ç»ªæ ‡å¿—ï¼ˆä¸­æ–­è®¾ç½®ï¼Œä¸»å¾ªç¯æ¸…é™¤ï¼‰
static volatile bool s_rx_data_ready = false;

// RX line assembly buffer (foreground parsing)
static char s_line_buf[256];
static uint16_t s_line_len = 0;

// URC å›è°ƒï¿??
typedef struct {
  char keyword[24];
  urc_callback_t cb;
  bool used;
} urc_entry_t;
static urc_entry_t s_urc_tbl[4];

// ==================== å†…éƒ¨å‡½æ•° ====================

// å‰å‘å£°æ˜ï¼ˆISR åœ¨å‡½æ•°å®šä¹‰ä¹‹å‰è°ƒï¿?? parse_rx_linesï¿??
static void parse_rx_lines_budget(uint16_t max_bytes);

static void handle_parsed_line(const char *line) {
  if (line == NULL || line[0] == '\0') {
    return;
  }

  if (s_cmd_result == 0) {
    if (strstr(line, "OK") != NULL || strcmp(line, "OK") == 0) {
      s_cmd_result = 1;
    } else if (strstr(line, "ERROR") != NULL || strstr(line, "+CME ERROR") != NULL ||
               strstr(line, "+CMS ERROR") != NULL) {
      s_cmd_result = -1;
    }
  }

  // æ£€ï¿?? URC
  for (size_t j = 0; j < sizeof(s_urc_tbl) / sizeof(s_urc_tbl[0]); j++) {
    if (s_urc_tbl[j].used && strstr(line, s_urc_tbl[j].keyword) != NULL) {
      if (s_urc_tbl[j].cb) {
        s_urc_tbl[j].cb(line);
      }
      break;
    }
  }
}

// USART2 ä¸­æ–­æœåŠ¡ç¨‹åºï¼ˆML307R 4Gæ¨¡ç»„ï¼?
void USART2_Handler(void) {
  // RX ä¸­æ–­ï¼ˆæ¯ä¸ªå­—èŠ‚åˆ°è¾¾æ—¶è§¦å‘ï¼?
  if (USART_GetStatus(CM_USART2, USART_FLAG_RX_FULL) == SET) {
    uint16_t data = USART_ReadData(CM_USART2);
    uint8_t ch = (uint8_t)(data & 0xFF);
    USART_ClearStatus(CM_USART2, USART_FLAG_RX_FULL);
    uint16_t next = (s_rx_head + 1) % UART_AT_RX_BUF_SIZE;
    if (next != s_rx_tail) {
      s_rx_buf[s_rx_head] = ch;
      s_rx_head = next;
    }
    // ÓĞĞÂ×Ö½Ú£ºÍ¨ÖªÇ°Ì¨½âÎö£¨²»ÔÚISRÀï½âÎö£©
    s_rx_data_ready = true;
    (void)USART_GetStatus(CM_USART2, USART_FLAG_RX_FULL);
  }

  // RX timeout
  // ä¸­æ–­ï¼ˆä¸€å¸§æ•°æ®æ¥æ”¶å®Œæ¯•ï¼ŒRXçº¿ç©ºé—?10bitsæ—¶è§¦å‘ï¼‰
  if (USART_GetStatus(CM_USART2, USART_FLAG_RX_TIMEOUT) == SET) {
    USART_ClearStatus(CM_USART2, USART_FLAG_RX_TIMEOUT);
    s_rx_data_ready = true;  // åªè®¾ç½®æ ‡å¿—ï¼Œä¸åœ¨ä¸­æ–­é‡Œè§£æ?
  }
}

// Ô¤ËãÊ½½âÎö£ºÃ¿´Î×î¶àÏû·Ñ max_bytes ¸ö×Ö½Ú£¬±ÜÃâÖ÷Ñ­»·Ò»´Î³Ô¿Õµ¼ÖÂ¿¨¶Ù
static void parse_rx_lines_budget(uint16_t max_bytes) {
  uint16_t processed = 0;
  while (s_rx_tail != s_rx_head && processed < max_bytes) {
    uint8_t ch = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
    processed++;

    if (ch == '\r' || ch == '\n') {
      if (s_line_len > 0) {
        s_line_buf[s_line_len] = '\0';
        handle_parsed_line(s_line_buf);
        s_line_len = 0;
      }
      continue;
    }

    if (s_line_len < (sizeof(s_line_buf) - 1)) {
      s_line_buf[s_line_len++] = (char)ch;
    } else {
      // ĞĞ¹ı³¤£º±£³Ö¶ªÆúÖ±µ½Óöµ½»»ĞĞ£¬±ÜÃâÒç³ö
    }
  }
}

// ==================== å…¬å¼€ API ====================

void uart_at_init(void) {
  s_rx_head = 0;
  s_rx_tail = 0;
  s_cmd_result = 0;
  s_rx_data_ready = false;
  s_line_len = 0;
  memset(s_urc_tbl, 0, sizeof(s_urc_tbl));

  // ä½¿èƒ½ USART2 RX ä¸­æ–­å’Œè¶…æ—¶ä¸­æ–­ï¼ˆåœ? NVIC ä¸­å·²é…ç½®ä¼˜å…ˆçº§ï¼‰
  USART_FuncCmd(CM_USART2, USART_INT_RX, ENABLE);
  USART_FuncCmd(CM_USART2, USART_RX_TIMEOUT, ENABLE);
  USART_FuncCmd(CM_USART2, USART_INT_RX_TIMEOUT, ENABLE);
}

int at_send_command(const char *cmd, const char *expected_ok,
                    uint32_t timeout_ms, char *response, int resp_len) {
  (void)expected_ok;
  if (cmd == NULL)
    return -2;

  at_flush_rx();

  // å‘ï¿½? AT å‘½ä»¤ï¼ˆè‡ªåŠ¨åŠ  \r\nï¿??
  char cmd_buf[320];
  snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

  uint16_t len = (uint16_t)strlen(cmd_buf);
  for (uint16_t i = 0; i < len; i++) {
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET)
      ;
    USART_WriteData(CM_USART2, (uint16_t)cmd_buf[i]);
  }

  // ç­‰å¾…å“åº”ï¼ˆè½®è¯¢æ–¹å¼ï¼‰
  s_cmd_result = 0;
  uint32_t start = sys_param.timer.timer_1ms_count;

  while (s_cmd_result == 0) {
    if (s_rx_data_ready || (s_rx_head != s_rx_tail)) {
      s_rx_data_ready = false;
      parse_rx_lines_budget(64);
    }
    if ((uint32_t)(sys_param.timer.timer_1ms_count - start) >= timeout_ms) {
      s_cmd_result = -1; // è¶…æ—¶
    }
  }

  // å¤åˆ¶å“åº”
  if (response != NULL && resp_len > 0) {
    uint16_t copied = 0;
    while (s_rx_tail != s_rx_head && copied < (uint16_t)(resp_len - 1)) {
      response[copied++] = s_rx_buf[s_rx_tail];
      s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
    }
    response[copied] = '\0';
  }

  return (s_cmd_result == 1) ? 0 : -2;
}

void at_send_raw(const uint8_t *data, uint16_t len) {
  if (data == NULL || len == 0)
    return;
  for (uint16_t i = 0; i < len; i++) {
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET)
      ;
    USART_WriteData(CM_USART2, (uint16_t)data[i]);
  }
}

void at_register_urc(const char *keyword, urc_callback_t callback) {
  if (keyword == NULL || callback == NULL)
    return;
  for (size_t i = 0; i < sizeof(s_urc_tbl) / sizeof(s_urc_tbl[0]); i++) {
    if (!s_urc_tbl[i].used) {
      strncpy(s_urc_tbl[i].keyword, keyword, sizeof(s_urc_tbl[i].keyword) - 1);
      s_urc_tbl[i].keyword[sizeof(s_urc_tbl[i].keyword) - 1] = '\0';
      s_urc_tbl[i].cb = callback;
      s_urc_tbl[i].used = true;
      break;
    }
  }
}

void at_flush_rx(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_rx_head = 0;
  s_rx_tail = 0;
  s_rx_data_ready = false;
  s_line_len = 0;
  __set_PRIMASK(primask);
}

void uart_at_process(void) {
  if (s_rx_data_ready || (s_rx_head != s_rx_tail)) {
    s_rx_data_ready = false;
    parse_rx_lines_budget(64);
  }
}

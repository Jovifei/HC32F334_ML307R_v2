//
// Included Files
//
#include "board.h"
#include "main.h"
#include "debug.h"
#include "eeprom.h"

// ================= SN命令帧相关定义 =================
#define FRAME_HEADER 0xFACE
#define SN_FRAME_SIZE 21   // 完整SN帧大小: 帧头2 + 长度1 + 命令码2 + SN15 + 校验1
#define SN_CMD_CODE 0x2001 // 写入SN命令码
#define MAX_FRAME_SIZE 40  // 最大帧缓冲区大小

// 返回产测系统执行的结果码
typedef enum
{
	RESULT_SUCCESS = 0x00,		  // 成功
	RESULT_ERROR_LENGTH = 0x01,	  // 长度错误
	RESULT_ERROR_PARAM = 0x02,	  // 参数错误
	RESULT_ERROR_CHECKSUM = 0x03, // 校验和错误
	RESULT_ERROR_WRITE = 0x04,	  // 写入失败
} result_code_t;

// 串口3接收帧缓冲区和状态变量
static uint8_t usart3_rx_buffer[MAX_FRAME_SIZE];
static uint8_t usart3_rx_count = 0;
static uint8_t usart3_frame_length = 0;
static uint8_t usart3_frame_ready = 0;

uint8_t debug_txbuffer[100];
uint8_t debug_rebuffer[100];
uint8_t debug_reindex = 0;
union
{
	float f;
	uint16_t c[2];
} f2c;

/*---------------------------------------------------------------------------
 Name        : void debug_task(void)
 Input       : 无
 Output      : 无
 Description : 调试任务函数，每 1ms 被中断触发执行一次（依赖于 sys_param.debug_1ms_count）。
---------------------------------------------------------------------------*/
void debug_task(void)
{
	if (sys_param.timer.debug_1ms_count >= 2)
	{
		uint16_t index = 0;
		sys_param.timer.debug_1ms_count = 0;

		f2c.f = (float)sys_param.state;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		debug_txbuffer[index++] = sys_param.fault.data >> 24 & 0xff;
		debug_txbuffer[index++] = sys_param.fault.data >> 16 & 0xff;
		debug_txbuffer[index++] = sys_param.fault.data >> 8 & 0xff;
		debug_txbuffer[index++] = sys_param.fault.data & 0xff;

		f2c.f = sys_param.ct1.rms_value;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct2.rms_value;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct3.rms_value;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.grid.ua_vol_rms;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.grid.phase_id.sequence_k;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct1.power.fix_dir_power;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct2.power.fix_dir_power;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct3.power.fix_dir_power;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct1.power.avg_power;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct1.power.ct_sub1g_boardcast_power_avg;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct2.power.ct_sub1g_boardcast_power_avg;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct3.power.ct_sub1g_boardcast_power_avg;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.ct1.power.fix_dir_power + sys_param.ct2.power.fix_dir_power + sys_param.ct3.power.fix_dir_power;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.grid.grid_frequency;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.grid.zero_cross.period_samples;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.grid.zero_cross.last_half_period;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		f2c.f = sys_param.grid.zero_cross.half_period;
		debug_txbuffer[index++] = f2c.c[0];
		debug_txbuffer[index++] = f2c.c[0] >> 8;
		debug_txbuffer[index++] = f2c.c[1];
		debug_txbuffer[index++] = f2c.c[1] >> 8;

		debug_txbuffer[index++] = 0x00;
		debug_txbuffer[index++] = 0x00;
		debug_txbuffer[index++] = 0x80;
		debug_txbuffer[index++] = 0x7f;
		debug_send_bytes(debug_txbuffer, index);
	}
}

/*将串口发送映射到打印函数上*/
int fputc(int ch, FILE *f)
{
	/* Wait Tx data register empty */
	while (RESET == USART_GetStatus(CM_USART3, USART_FLAG_TX_EMPTY))
	{
	}
	USART_WriteData(CM_USART3, (uint8_t)ch);
	return ch;
}

void debug_send_bytes(uint8_t *buffer, uint8_t len)
{
	uint16_t i = 0;
	for (i = 0; i < len; i++)
	{
		/// LIN_writeSCICharBlocking(UART_DEBUG_BASE, buffer[i]);
		/* Wait Tx data register empty */
		while (RESET == USART_GetStatus(CM_USART3, USART_FLAG_TX_EMPTY))
		{
		}
		USART_WriteData(CM_USART3, buffer[i]);
	}
}

/* INT_SRC_USART3_RI Callback. */
void USART3_Handler(void)
{
	// 串口3中断处理 - 帧接收模式
	if (USART_GetStatus(CM_USART3, USART_FLAG_RX_FULL))
	{
		uint8_t received_byte = USART_ReadData(CM_USART3);

		// 等待帧头第一个字节 0xFA
		if (usart3_rx_count == 0)
		{
			if (received_byte == 0xFA)
			{
				usart3_rx_buffer[usart3_rx_count++] = received_byte;
			}
		}
		// 等待帧头第二个字节 0xCE
		else if (usart3_rx_count == 1)
		{
			if (received_byte == 0xCE)
			{
				usart3_rx_buffer[usart3_rx_count++] = received_byte;
			}
			else
			{
				usart3_rx_count = 0; // 不是正确的帧头，重新开始
			}
		}
		// 接收长度字段
		else if (usart3_rx_count == 2)
		{
			usart3_rx_buffer[usart3_rx_count++] = received_byte;
			usart3_frame_length = 3 + received_byte;

			if (usart3_frame_length < 6 || usart3_frame_length > MAX_FRAME_SIZE)
			{
				usart3_rx_count = 0;
				usart3_frame_length = 0;
			}
		}
		else // 按照长度继续接收剩余字节
		{
			usart3_rx_buffer[usart3_rx_count++] = received_byte;

			// 接收完成
			if (usart3_rx_count == usart3_frame_length)
			{
				// 验证帧头
				if (usart3_rx_buffer[0] == 0xFA && usart3_rx_buffer[1] == 0xCE)
				{
					usart3_frame_ready = 1; // 设置帧就绪标志
				}

				// 重置接收状态
				usart3_rx_count = 0;
				usart3_frame_length = 0;
			}
		}

		// 缓冲区溢出保护
		if (usart3_rx_count >= MAX_FRAME_SIZE)
		{
			usart3_rx_count = 0;
			usart3_frame_length = 0;
		}
	}

	USART_ClearStatus(CM_USART3, USART_FLAG_ALL);
	__DSB(); /* Arm Errata 838869 */
}

/*---------------------------------------------------------------------------
 Name		:	uint16_t KLPF_Function_Float(uint16_t inputValue, uint8_t LPFK, uint8_t channelIndex)
 Input	:	NO
 Output	:	NO
 Description:         LPF：Y(n) = Y(n-1) + LPFK*(X(n) - Y(n-1));
---------------------------------------------------------------------------*/
uint16_t KLPF_Function_Float(uint16_t inputValue, float LPFK, uint8_t channelIndex)
{
	static float float_accumulator[5] = {0};
	static uint16_t float_first_run[5] = {0};

	if (channelIndex >= 5)
		return inputValue;

	// 首次运行的初始化
	if (float_first_run[channelIndex] == 0)
	{
		float_accumulator[channelIndex] = (float)inputValue;
		float_first_run[channelIndex] = 1;
		return inputValue;
	}

	// 浮点滤波计算
	// float alpha = (float)LPFK / 128.0f;
	float diff = (float)inputValue - float_accumulator[channelIndex];
	float_accumulator[channelIndex] += LPFK * diff;

	return (uint16_t)(float_accumulator[channelIndex] + 0.5f); // 四舍五入
}

/*---------------------------------------------------------------------------
 Name        : uint8_t calculate_frame_checksum(const uint8_t *data, uint8_t len)
 Input       : data - 数据缓冲区, len - 数据长度
 Output      : 校验和
 Description : 计算帧校验和（所有字节累加取低8位）
---------------------------------------------------------------------------*/
static uint8_t calculate_frame_checksum(const uint8_t *data, uint8_t len)
{
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < len; i++)
	{
		checksum += data[i];
	}
	return checksum & 0xFF;
}

/*---------------------------------------------------------------------------
 Name        : void send_sn_reply(uint16_t cmd_code, result_code_t result)
 Input       : cmd_code - 命令码, result - 结果码
 Output      : 无
 Description : 发送SN命令回复帧
---------------------------------------------------------------------------*/
static void send_sn_reply(uint16_t cmd_code, result_code_t result)
{
	uint8_t reply_buffer[7];

	// 构建回复帧: 0xCEFA + 长度 + 命令码 + 结果码 + 校验
	reply_buffer[0] = (FRAME_HEADER >> 8) & 0xFF;
	reply_buffer[1] = FRAME_HEADER & 0xFF;
	reply_buffer[2] = 0x04;
	reply_buffer[3] = (cmd_code >> 8) & 0xFF;
	reply_buffer[4] = cmd_code & 0xFF;
	reply_buffer[5] = (uint8_t)result;							 // 结果码
	reply_buffer[6] = calculate_frame_checksum(reply_buffer, 6); // 计算校验和

	debug_send_bytes(reply_buffer, 7); // 发送回复帧
}

/*---------------------------------------------------------------------------
 Name        : void process_sn_command(void)
 Input       : 无
 Output      : 无
 Description : 处理SN写入命令（写入后回读验证）
---------------------------------------------------------------------------*/
static void process_sn_command(void)
{
	// 解析命令码
	uint16_t cmd_code = (usart3_rx_buffer[3] << 8) | usart3_rx_buffer[4];

	// 只处理SN写入命令 0x2001
	if (cmd_code != SN_CMD_CODE)
	{
		send_sn_reply(cmd_code, RESULT_ERROR_PARAM);
		return;
	}

	// 检查数据长度（帧头2 + 长度1 + 命令码2 + SN15 + 校验1 = 21）
	uint8_t data_length = usart3_rx_buffer[2];
	if (data_length != (SN_FRAME_SIZE - 3)) // 去掉帧头与长度字段
	{
		send_sn_reply(cmd_code, RESULT_ERROR_LENGTH);
		return;
	}

	// 计算校验和（从帧头到SN最后一个字节）
	uint8_t calculated_checksum = calculate_frame_checksum(usart3_rx_buffer, 20);
	uint8_t received_checksum = usart3_rx_buffer[20];

	if (calculated_checksum != received_checksum)
	{
		send_sn_reply(cmd_code, RESULT_ERROR_CHECKSUM);
		return;
	}

	// 提取SN（15字节）
	char sn[16];
	memcpy(sn, &usart3_rx_buffer[5], SN_LENGTH);
	sn[SN_LENGTH] = '\0';

	// 写入SN到EEPROM
	int write_result = eeprom_write_sn(sn);

	if (write_result != 0)
	{
		// 写入失败
		sys_param.flash_sn_com_normal = false;
		send_sn_reply(cmd_code, RESULT_ERROR_WRITE);
		return;
	}

	// 回读验证
	char readback_sn[16];
	int read_result = eeprom_read_sn(readback_sn);

	if (read_result != 0)
	{
		// 回读失败
		sys_param.flash_sn_com_normal = false;
		send_sn_reply(cmd_code, RESULT_ERROR_WRITE);
		return;
	}

	// 比对写入的SN和回读的SN
	if (strncmp(sn, readback_sn, SN_LENGTH) != 0)
	{
		// 回读数据不匹配
		sys_param.flash_sn_com_normal = false;
		send_sn_reply(cmd_code, RESULT_ERROR_WRITE);
		return;
	}

	// 写入并验证成功
	sys_param.flash_sn_com_normal = true;
	// sys_param.wifi.restore_wifi_cmd = 1; // 重置wifi
	send_sn_reply(cmd_code, RESULT_SUCCESS);
}

/*---------------------------------------------------------------------------
 Name        : void debug_sn_task(void)
 Input       : 无
 Output      : 无
 Description : SN命令处理任务（在主循环中调用）
---------------------------------------------------------------------------*/
void debug_sn_task(void)
{
	// 检查是否有帧就绪
	if (usart3_frame_ready == 1)
	{
		usart3_frame_ready = 0;
		process_sn_command();
	}
}

/*启动logo展示*/
void boot_logo_print(void)
{
	printf("/*******************************************************************************************/\r\n");
	printf("/*      :?tzLOwOQJxi _cccczzzzzXv    lcccczzzzzzzi      :?tzLOwOQJxi      .!)nC0mZLXt-     */\r\n");
	printf("/*   .}Z&$$$@&#M&$$f  j$$$$8&&&&&*I  ?$$$$8&&&&&&?   .}Z&$$$@&#M&$$f   ^/b@$$$&#WB$$$MYi   */\r\n");
	printf("/*  ;m$$$#u_:`''^;_l  f$$$h'``````.  -$$$8i``````.  ;m$$$#u_:`''^;_l  >k$$$p1I^'`:-C$$$8(  */\r\n");
	printf("/* 'p$$$q;            f$$$a;'''''`   -$$$@<''''''  'p$$$q;           'h$$$0`        n$$$B  */\r\n");
	printf("/* -$$$$?             f$$$$@@@@@@u   -$$$$@@@@@@O. -$$$$?            ?$$$$_         ;@$$$  */\r\n");
	printf("/* -$$$$(     +0@@@@  f$$$&zccccc]   -$$$@Uccccc|  -$$$$(            +$$$$(         <B$$$  */\r\n");
	printf("/* `m$$$W1'     -@@@  f$$$k          -$$$8'        `m$$$W1'          '0$$$M?       !p$$$m  */\r\n");
	printf("/*  ,C@$$$aYt)[1|uO1  f$$$M@((((((<  -$$$Bj((((((?  ,C@$$$aYt)[1|uO1  ^c@$$$qx([)j0B$$@Y'  */\r\n");
	printf("/*'   <nqM$$$$$$$$&~  t@@@@$$$$$$$v  _B@@@$$$$$$$m    <nqM$$$$$$$$&1    **M$$$$$$$Wwx**    */\r\n");
	printf("/*******************************************************************************************/\r\n");
}

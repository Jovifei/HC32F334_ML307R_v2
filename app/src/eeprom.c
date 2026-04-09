//
// Included Files
//
#include "board.h"
#include "main.h"
#include "eeprom.h"
#include "wifi.h"
#include "grid.h"
#include "string.h"
#include "stdio.h"

/**
 * @brief  BSP I2C write.
 * @param  [in] I2Cx                Pointer to I2C instance register base.
 *                                  This parameter can be a value of the following:
 *         @arg CM_I2Cx:            I2C instance register base.
 * @param  [in] u16DevAddr:         Device address.
 * @param  [in] pu8Reg:             Pointer to the register address or memory address.
 * @param  [in] u8RegLen:           Length of register address or memory address.
 * @param  [in] pu8Buf:             The pointer to the buffer contains the data to be write.
 * @param  [in] u32Len:             Buffer size in byte.
 * @retval int32_t:
 *            - LL_OK:              Success
 *            - LL_ERR:             Receive NACK
 *            - LL_ERR_TIMEOUT:     Timeout
 *            - LL_ERR_INVD_PARAM:  pu8Buf is NULL
 */
int32_t BSP_I2C_Write(CM_I2C_TypeDef *I2Cx, uint16_t u16DevAddr, const uint8_t *pu8Reg, uint8_t u8RegLen, const uint8_t *pu8Buf, uint32_t u32Len)
{
	int32_t i32Ret;

	I2C_SWResetCmd(I2Cx, ENABLE);
	I2C_SWResetCmd(I2Cx, DISABLE);
	i32Ret = I2C_Start(I2Cx, BSP_I2C_TIMEOUT);
	if (LL_OK == i32Ret)
	{

		i32Ret = I2C_TransAddr(I2Cx, u16DevAddr, I2C_DIR_TX, BSP_I2C_TIMEOUT);

		if (LL_OK == i32Ret)
		{
			i32Ret = I2C_TransData(I2Cx, pu8Reg, u8RegLen, BSP_I2C_TIMEOUT);
			if (LL_OK == i32Ret)
			{
				i32Ret = I2C_TransData(I2Cx, pu8Buf, u32Len, BSP_I2C_TIMEOUT);
			}
		}
	}
	(void)I2C_Stop(I2Cx, BSP_I2C_TIMEOUT);
	return i32Ret;
}

/**
 * @brief  BSP I2C read.
 * @param  [in] I2Cx                Pointer to I2C instance register base.
 *                                  This parameter can be a value of the following:
 *         @arg CM_I2Cx:            I2C instance register base.
 * @param  [in] u16DevAddr:         Device address.
 * @param  [in] pu8Reg:             Pointer to the register address or memory address.
 * @param  [in] u8RegLen:           Length of register address or memory address.
 * @param  [in] pu8Buf:             The pointer to the buffer contains the data to be read.
 * @param  [in] u32Len:             Buffer size in byte.
 * @retval int32_t:
 *            - LL_OK:              Success
 *            - LL_ERR:             Receive NACK
 *            - LL_ERR_TIMEOUT:     Timeout
 *            - LL_ERR_INVD_PARAM:  pu8Buf is NULL
 */
int32_t BSP_I2C_Read(CM_I2C_TypeDef *I2Cx, uint16_t u16DevAddr, const uint8_t *pu8Reg, uint8_t u8RegLen, uint8_t *pu8Buf, uint32_t u32Len)
{
	int32_t i32Ret;
	I2C_SWResetCmd(I2Cx, ENABLE);
	I2C_SWResetCmd(I2Cx, DISABLE);
	i32Ret = I2C_Start(I2Cx, BSP_I2C_TIMEOUT);
	if (LL_OK == i32Ret)
	{
		i32Ret = I2C_TransAddr(I2Cx, u16DevAddr, I2C_DIR_TX, BSP_I2C_TIMEOUT);

		if (LL_OK == i32Ret)
		{
			i32Ret = I2C_TransData(I2Cx, pu8Reg, u8RegLen, BSP_I2C_TIMEOUT);
			if (LL_OK == i32Ret)
			{
				i32Ret = I2C_Restart(I2Cx, BSP_I2C_TIMEOUT);
				if (LL_OK == i32Ret)
				{
					if (1UL == u32Len)
					{
						I2C_AckConfig(I2Cx, I2C_NACK);
					}

					i32Ret = I2C_TransAddr(I2Cx, u16DevAddr, I2C_DIR_RX, BSP_I2C_TIMEOUT);
					if (LL_OK == i32Ret)
					{
						i32Ret = I2C_MasterReceiveDataAndStop(I2Cx, pu8Buf, u32Len, BSP_I2C_TIMEOUT);
					}
					I2C_AckConfig(I2Cx, I2C_ACK);
				}
			}
		}
	}

	if (LL_OK != i32Ret)
	{
		(void)I2C_Stop(I2Cx, BSP_I2C_TIMEOUT);
	}

	return i32Ret;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t calculate_crc8(const uint8_t *data, uint16_t len)
 Input       : data: 数据指针, len: 数据长度
 Output      : CRC8校验码
 Description : 计算CRC8校验码
---------------------------------------------------------------------------*/
uint8_t calculate_crc8(const uint8_t *data, uint16_t len)
{
	uint8_t crc = CRC8_INIT_VALUE;
	uint16_t i, j;

	for (i = 0; i < len; i++)
	{
		crc ^= data[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 0x80)
			{
				crc = (crc << 1) ^ CRC8_POLYNOMIAL;
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	return crc;
}

/*---------------------------------------------------------------------------
 Name        : bool verify_record_crc8(const eeprom_device_record_t *record)
 Input       : record: 记录指针
 Output      : CRC8校验结果
 Description : 验证记录的CRC8校验码
---------------------------------------------------------------------------*/
bool verify_record_crc8(const eeprom_device_record_t *record)
{
	// 计算CRC，-1是不包括最后的crc8字段
	uint8_t calculated_crc = calculate_crc8((const uint8_t *)record, sizeof(eeprom_device_record_t) - 1);

	return (calculated_crc == record->crc8);
}

/*---------------------------------------------------------------------------
 Name        : const char* product_model_code_to_string(uint8_t code)
 Input       : code: 产品型号代码
 Output      : 产品型号字符串
 Description : 将产品型号代码转换为字符串
---------------------------------------------------------------------------*/
const char *product_model_code_to_string(uint8_t code)
{
	switch (code)
	{
	case PRODUCT_MODEL_CODE_MI800S:
		return "GE-MI800S";
	case PRODUCT_MODEL_CODE_MI2500S:
		return "GE-MI2500S";
	// 后续可扩展其他产品型号...
	default:
		return "UNKNOWN";
	}
}

/*---------------------------------------------------------------------------
 Name        : uint8_t product_model_string_to_code(const char *model_str)
 Input       : model_str: 产品型号字符串
 Output      : 产品型号代码
 Description : 将产品型号字符串转换为代码
---------------------------------------------------------------------------*/
uint8_t product_model_string_to_code(const char *model_str)
{
	if (model_str == NULL)
	{
		return 0;
	}

	if (strcmp(model_str, "GE-MI800S") == 0)
	{
		return PRODUCT_MODEL_CODE_MI800S;
	}
	else if (strcmp(model_str, "GE-MI2500S") == 0)
	{
		return PRODUCT_MODEL_CODE_MI2500S;
	}
	// 后续可扩展其他产品型号...

	return 0; // 未知型号
}

/*---------------------------------------------------------------------------
 Name        : static int32_t eeprom_i2c_write(uint16_t addr, const uint8_t *data, uint32_t len)
 Description : EEPROM写入辅助函数（正确的HT24LC08地址处理）
---------------------------------------------------------------------------*/
static int32_t eeprom_i2c_write(uint16_t addr, const uint8_t *data, uint32_t len)
{
	int32_t ret;

	// 提取10位地址的高2位（页选择位P1,P0）
	uint8_t page_bits = (addr >> 8) & 0x03;

	// 构造设备地址：基础地址0x50 + 页选择位P1,P0
	// 设备地址格式: 1 0 1 0 A2 P1 P0 R/W
	// 由于A2=0，基础地址为0x50，页选择位左移1位放入P1,P0位置
	uint8_t device_addr = EEPROM_I2C_ADDR | (page_bits);

	// 地址的低8位
	uint8_t low_addr_byte = (uint8_t)(addr & 0xFF);

	// 使用1字节地址模式（实际发送的是10位地址的低8位）
	ret = BSP_I2C_Write(CM_I2C, device_addr, &low_addr_byte, 1, data, len);

	return ret;
}

/*---------------------------------------------------------------------------
 Name        : static int32_t eeprom_i2c_read(uint16_t addr, uint8_t *data, uint32_t len)
 Description : EEPROM读取辅助函数（正确的HT24LC08地址处理）
---------------------------------------------------------------------------*/
static int32_t eeprom_i2c_read(uint16_t addr, uint8_t *data, uint32_t len)
{
	int32_t ret;

	// 提取10位地址的高2位（页选择位P1,P0）
	uint8_t page_bits = (addr >> 8) & 0x03;

	// 构造设备地址：基础地址0x50 + 页选择位P1,P0
	uint8_t device_addr = EEPROM_I2C_ADDR | (page_bits);

	// 地址的低8位
	uint8_t low_addr_byte = (uint8_t)(addr & 0xFF);

	// 使用1字节地址模式
	ret = BSP_I2C_Read(CM_I2C, device_addr, &low_addr_byte, 1, data, len);

	return ret;
}

/*---------------------------------------------------------------------------
 Name        : static int eeprom_read_device_record(uint8_t index, eeprom_device_record_t *record)
 Input       : index: 设备索引, record: 记录指针
 Output      : 0: 成功, -1: 读取失败, -2: CRC校验失败
 Description : 从EEPROM中读取设备记录 (48字节，分3次读取，支持2字节地址)
---------------------------------------------------------------------------*/
static int eeprom_read_device_record(uint8_t index, eeprom_device_record_t *record)
{
	if (index >= EEPROM_MAX_DEVICES || record == NULL)
	{
		return -1;
	}

	// 计算记录的起始地址
	uint16_t base_addr = E2P_DEVICE_BASE_ADDR + (index * EEPROM_DEVICE_RECORD_SIZE);

	// 48字节分三次读取（每次16字节）
	// 第一页：0-15字节
	uint16_t addr1 = base_addr;
	int32_t ret = eeprom_i2c_read(addr1, (uint8_t *)record, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// 第二页：16-31字节
	uint16_t addr2 = base_addr + E2P_PAGE_LEN;
	ret = eeprom_i2c_read(addr2, ((uint8_t *)record) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// 第三页：32-47字节
	uint16_t addr3 = base_addr + (E2P_PAGE_LEN * 2);
	ret = eeprom_i2c_read(addr3, ((uint8_t *)record) + (E2P_PAGE_LEN * 2), E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// // 打印完整的记录数据
	// printf("Data to read  [%d bytes]: ", sizeof(eeprom_device_record_t));
	// for (int i = 0; i < sizeof(eeprom_device_record_t); i++)
	// {
	// 	printf("%02X ", *(((uint8_t *)record) + i));
	// }
	// printf("\r\n");

	// 验证CRC8
	if (!verify_record_crc8(record))
	{
		return -2; // CRC校验失败
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : static int eeprom_write_device_record(uint8_t index, const eeprom_device_record_t *record)
 Input       : index: 设备索引, record: 记录指针
 Output      : 0: 成功, -1: 写入失败
 Description : 向EEPROM中写入设备记录 (48字节，分3次写入，支持2字节地址)
---------------------------------------------------------------------------*/
static int eeprom_write_device_record(uint8_t index, const eeprom_device_record_t *record)
{
	if (index >= EEPROM_MAX_DEVICES || record == NULL)
	{
		return -1;
	}

	// 创建副本并计算CRC8
	eeprom_device_record_t record_copy;
	memcpy(&record_copy, record, sizeof(eeprom_device_record_t));
	record_copy.crc8 = calculate_crc8((const uint8_t *)&record_copy, sizeof(eeprom_device_record_t) - 1);

	// 计算记录的起始地址
	uint16_t base_addr = E2P_DEVICE_BASE_ADDR + (index * EEPROM_DEVICE_RECORD_SIZE);

	// // 打印要写入的完整数据（包含CRC8）
	// printf("Data to write [%d bytes]: ", sizeof(eeprom_device_record_t));
	// for (int i = 0; i < sizeof(eeprom_device_record_t); i++)
	// {
	// 	printf("%02X ", *(((uint8_t *)&record_copy) + i));
	// }

	// 48字节分三次写入（每次16字节）
	// 第一页：0-15字节
	uint16_t addr1 = base_addr;
	int32_t ret = eeprom_i2c_write(addr1, (const uint8_t *)&record_copy, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	delay_ms(10); // 等待页写入完成

	// 第二页：16-31字节
	uint16_t addr2 = base_addr + E2P_PAGE_LEN;
	ret = eeprom_i2c_write(addr2, ((const uint8_t *)&record_copy) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}
	delay_ms(10); // 等待页写入完成

	// 第三页：32-47字节
	uint16_t addr3 = base_addr + (E2P_PAGE_LEN * 2);
	ret = eeprom_i2c_write(addr3, ((const uint8_t *)&record_copy) + (E2P_PAGE_LEN * 2), E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}
	delay_ms(10); // 等待页写入完成

	// 验证写入
	eeprom_device_record_t verify_record;
	if (eeprom_read_device_record(index, &verify_record) == 0)
	{
		if (memcmp(&record_copy, &verify_record, sizeof(eeprom_device_record_t)) != 0)
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : static uint8_t find_free_siid(void)
 Input       : 无
 Output      : SIID, 0表示未找到
 Description : 查找空闲的SIID
---------------------------------------------------------------------------*/
static uint8_t find_free_siid(void)
{
	bool siid_used[SIID_MAX - SIID_MIN + 1] = {false};

	// 标记已使用的SIID
	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		uint8_t siid = sys_param.paired_inv_info[i].siid;
		if (siid >= SIID_MIN && siid <= SIID_MAX)
		{
			siid_used[siid - SIID_MIN] = true;
		}
	}

	// 查找第一个未使用的SIID
	for (uint8_t siid = SIID_MIN; siid <= SIID_MAX; siid++)
	{
		if (!siid_used[siid - SIID_MIN])
		{
			return siid;
		}
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_init_and_load_devices(void)
 Input       : 无
 Output      : 0: 成功
 Description : 初始化并加载设备信息
---------------------------------------------------------------------------*/
int eeprom_init_and_load_devices(void)
{
	eeprom_device_record_t record;
	int read_result;

	// 清空内存中的设备信息
	memset(sys_param.paired_inv_info, 0, sizeof(sys_param.paired_inv_info));

	// 从EEPROM读取所有设备记录
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		read_result = eeprom_read_device_record(i, &record);

		if (read_result == 0) // 读取成功且CRC校验通过
		{
			if (record.valid == EEPROM_RECORD_VALID && record.siid >= SIID_MIN && record.siid <= SIID_MAX)
			{
				// 复制基础信息
				memcpy(sys_param.paired_inv_info[i].device_sn, record.device_sn, SN_LENGTH);
				sys_param.paired_inv_info[i].device_sn[SN_LENGTH] = '\0';
				sys_param.paired_inv_info[i].sub1g_addr = record.sub1g_addr;
				sys_param.paired_inv_info[i].siid = record.siid;
				sys_param.paired_inv_info[i].is_valid = true;
				sys_param.paired_inv_info[i].online_state = 1; // 微逆不在线，等待上报设置为在线
				sys_param.paired_inv_info[i].offline_updata_ms = PAIRED_INV_ONLINE_TIMEOUT_S;

				// 复制产品型号
				strncpy(sys_param.paired_inv_info[i].product_model, record.product_model, PRODUCT_MODEL_MAX_LEN);
				sys_param.paired_inv_info[i].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

				// 复制相位信息
				sys_param.paired_inv_info[i].phase = record.phase;
			}
			else
			{
				sys_param.paired_inv_info[i].online_state = 0; // 读取失败，设置没有匹配的微逆
			}
		}
		else if (read_result == -2) // CRC校验失败
		{
			sys_param.paired_inv_info[i].online_state = 0; // 读取失败，设置没有匹配的微逆
		}
		else if (read_result == -1) // 读取失败
		{
			sys_param.paired_inv_info[i].online_state = 0; // 读取失败，设置没有匹配的微逆
		}
	}

	// 加载用户配对列表
	eeprom_load_user_pair_list();

	// 加载配置参数
	int consumption_ret = eeprom_load_set_param();
	if (consumption_ret == 0)
	{
		printf("\n1.EEPROM配置参数\n");
		printf(" 参数名称               | 加载值\n");
		printf(" -----------------------|--------------\n");
		printf(" 允许并网功率           | %d\n", sys_param.to_grid_power_limit);
		printf(" 功率工作模式           | %d\n", sys_param.power_work_mode);
		printf(" 防逆流开关             | %d\n", sys_param.anti_backflow_switch);
		printf(" CT相序(sequence_k)     | %d\n", sys_param.grid.phase_id.sequence_k);
		printf(" CT1方向                | %d\n", (int)(sys_param.ct1.power.power_direction));
		printf(" CT2方向                | %d\n", (int)(sys_param.ct2.power.power_direction));
		printf(" CT3方向                | %d\n", (int)(sys_param.ct3.power.power_direction));
	}
	else if (consumption_ret == -2)
	{
		printf("EEPROM: Config CRC error, reset to default and saved\n");
	}
	else
	{
		printf("EEPROM: Config read failed, reset to default and saved\n");
	}

	// 新区域有效则直接使用
	int elec_ret = eeprom_load_elec_consumption();
	if (elec_ret != 0)
	{
		// 新区域无效，尝试从旧配置区迁移（升级兼容）
		eeprom_migrate_elec_consumption();
	}
	printf(" 用电量Wh               | %u\n", sys_param.hmi.electricity_consumption);
	printf("=======================================\n");

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_add_device(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code)
 Input       : device_sn: 设备序列号, sub1g_addr: sub1g_inv地址, product_model_code: 产品型号代码
 Output      : SIID, 0表示添加失败
 Description : 根据微逆的SN、sub1g地址和产品型号添加设备到EEPROM
---------------------------------------------------------------------------*/
uint8_t eeprom_add_device(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code)
{
	if (device_sn == NULL || strlen(device_sn) > SN_LENGTH)
	{
		// printf("Invalid parameters\n");
		return 0;
	}

	uint8_t existing_siid = eeprom_find_siid_by_addr(sub1g_addr);
	if (existing_siid != 0)
	{
		// printf("Device with sub1g_addr %u already exists with SIID %u\n", sub1g_addr, existing_siid);
		return 0;
	}

	// 查找空闲eeprom区域
	int8_t free_slot = -1;
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].siid == 0)
		{
			free_slot = i;
			break;
		}
	}

	if (free_slot == -1)
	{
		return 0; // 空间已满
	}

	// 分配SIID
	uint8_t new_siid = find_free_siid();
	if (new_siid == 0)
	{
		return 0; // 无可用SIID
	}

	// 将产品型号代码转换为字符串
	const char *model_str = product_model_code_to_string(product_model_code);

	// 更新内存
	memset(sys_param.paired_inv_info[free_slot].device_sn, 0, sizeof(sys_param.paired_inv_info[free_slot].device_sn));
	strncpy(sys_param.paired_inv_info[free_slot].device_sn, device_sn, SN_LENGTH);
	sys_param.paired_inv_info[free_slot].device_sn[SN_LENGTH] = '\0';
	sys_param.paired_inv_info[free_slot].sub1g_addr = sub1g_addr;
	sys_param.paired_inv_info[free_slot].siid = new_siid;
	sys_param.paired_inv_info[free_slot].is_valid = true;

	// 保存产品型号字符串
	strncpy(sys_param.paired_inv_info[free_slot].product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	sys_param.paired_inv_info[free_slot].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	// 初始化phase为0（未识别），之后会进行相序识别
	sys_param.paired_inv_info[free_slot].phase = 0;

	// printf("Added device with SIID %u to slot %d\n", new_siid, free_slot);

	// 写入EEPROM
	eeprom_device_record_t record;
	memset(&record, 0, sizeof(record));
	strncpy(record.device_sn, device_sn, SN_LENGTH);
	record.device_sn[SN_LENGTH] = '\0';
	record.sub1g_addr = sub1g_addr;
	record.siid = new_siid;
	record.valid = EEPROM_RECORD_VALID;
	record.phase = 0; // 初始化为未识别

	// 保存产品型号到EEPROM
	strncpy(record.product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	record.product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	int write_ret = eeprom_write_device_record(free_slot, &record);
	if (write_ret != 0)
	{
		// 写入失败，清除内存
		memset(&sys_param.paired_inv_info[free_slot], 0, sizeof(inv_device_t));
		return 0;
	}

	return new_siid;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_add_device_by_sn_only(const char *device_sn)
 Input       : device_sn: 设备序列号
 Output      : SIID, 0表示添加失败
 Description : 只根据SN预分配设备，sub1g_addr设置为0，等待微逆广播时更新
			   用于用户扫码绑定时立即分配SIID，后台可以直接读取设备信息
---------------------------------------------------------------------------*/
uint8_t eeprom_add_device_by_sn_only(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0 || strlen(device_sn) > SN_LENGTH)
	{
		return 0;
	}

	// 检查是否已经存在相同SN的设备
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].is_valid)
		{
			// 使用存储SN的实际长度进行前缀匹配
			size_t stored_len = strlen(sys_param.paired_inv_info[i].device_sn);
			if (stored_len > 0 && strncmp(sys_param.paired_inv_info[i].device_sn, device_sn, stored_len) == 0)
			{
				// 已存在，返回已分配的SIID
				return sys_param.paired_inv_info[i].siid;
			}
		}
	}

	// 查找空闲eeprom区域
	int8_t free_slot = -1;
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].siid == 0)
		{
			free_slot = i;
			break;
		}
	}

	if (free_slot == -1)
	{
		return 0;
	}

	// 分配SIID
	uint8_t new_siid = find_free_siid();
	if (new_siid == 0)
	{
		return 0; // 无可用SIID
	}

	// 更新内存
	memset(&sys_param.paired_inv_info[free_slot], 0, sizeof(inv_device_t));
	strncpy(sys_param.paired_inv_info[free_slot].device_sn, device_sn, SN_LENGTH);
	sys_param.paired_inv_info[free_slot].device_sn[SN_LENGTH] = '\0';
	sys_param.paired_inv_info[free_slot].sub1g_addr = 0; // 暂时为0，等待微逆广播时更新
	sys_param.paired_inv_info[free_slot].siid = new_siid;
	sys_param.paired_inv_info[free_slot].is_valid = true;
	sys_param.paired_inv_info[free_slot].online_state = 1; // 配对未在线
	sys_param.paired_inv_info[free_slot].phase = 0;		   // 未识别相位

	// 产品型号暂时设置为未知，等待微逆广播时更新
	strncpy(sys_param.paired_inv_info[free_slot].product_model, "UNKNOWN", PRODUCT_MODEL_MAX_LEN);
	sys_param.paired_inv_info[free_slot].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	// 写入EEPROM
	eeprom_device_record_t record;
	memset(&record, 0, sizeof(record));
	strncpy(record.device_sn, device_sn, SN_LENGTH);
	record.device_sn[SN_LENGTH] = '\0';
	record.sub1g_addr = 0; // 暂时为0
	record.siid = new_siid;
	record.valid = EEPROM_RECORD_VALID;
	record.phase = 0;
	strncpy(record.product_model, "UNKNOWN", PRODUCT_MODEL_MAX_LEN);
	record.product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	int write_ret = eeprom_write_device_record(free_slot, &record);
	if (write_ret != 0)
	{
		// 写入失败，清除内存
		memset(&sys_param.paired_inv_info[free_slot], 0, sizeof(inv_device_t));
		return 0;
	}

	return new_siid;
}

/*---------------------------------------------------------------------------
 Name        : int8_t eeprom_find_inv_index_by_sn(const char *device_sn)
 Input       : device_sn: 设备序列号
 Output      : 设备索引（0-7），未找到返回-1
 Description : 根据SN查找已配对设备的索引
---------------------------------------------------------------------------*/
int8_t eeprom_find_inv_index_by_sn(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0)
	{
		return -1;
	}

	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].is_valid)
		{
			size_t stored_len = strlen(sys_param.paired_inv_info[i].device_sn);
			if (stored_len > 0 && strncmp(sys_param.paired_inv_info[i].device_sn, device_sn, stored_len) == 0)
			{
				return i;
			}
		}
	}

	return -1;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_update_device_sub1g_addr(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code)
 Input       : device_sn: 设备序列号
			   sub1g_addr: SUB1G地址
			   product_model_code: 产品型号代码
 Output      : 0=成功, -1=失败
 Description : 根据SN更新已预分配设备的sub1g地址和产品型号
			   用于微逆广播时更新预分配设备的信息
---------------------------------------------------------------------------*/
int eeprom_update_device_sub1g_addr(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code)
{
	if (device_sn == NULL || strlen(device_sn) == 0)
	{
		return -1;
	}

	// 查找设备
	int8_t slot = eeprom_find_inv_index_by_sn(device_sn);
	if (slot < 0)
	{
		return -1; // 设备不存在
	}

	// 将产品型号代码转换为字符串
	const char *model_str = product_model_code_to_string(product_model_code);

	// 更新内存
	sys_param.paired_inv_info[slot].sub1g_addr = sub1g_addr;
	strncpy(sys_param.paired_inv_info[slot].product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	sys_param.paired_inv_info[slot].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';
	sys_param.paired_inv_info[slot].online_state = 2; // 设备在线
	sys_param.paired_inv_info[slot].prop_changed = true;

	// 读取现有EEPROM记录
	eeprom_device_record_t record;
	if (eeprom_read_device_record(slot, &record) != 0)
	{
		return -1;
	}

	// 更新记录
	record.sub1g_addr = sub1g_addr;
	strncpy(record.product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	record.product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	// 写回EEPROM
	if (eeprom_write_device_record(slot, &record) != 0)
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_remove_device_by_sub1g_addr(uint32_t sub1g_addr)
 Input       : sub1g_addr: sub1g_inv地址
 Output      : 0: 成功, -1: 失败
 Description : 根据sub1g_inv地址从EEPROM中移除设备
---------------------------------------------------------------------------*/
int eeprom_remove_device_by_sub1g_addr(uint32_t sub1g_addr)
{
	// 查找设备
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr && sys_param.paired_inv_info[i].siid != 0)
		{
			// 清除内存
			memset(&sys_param.paired_inv_info[i], 0, sizeof(inv_device_t));

			// 清除EEPROM，写入0表示无效
			eeprom_device_record_t record;
			memset(&record, 0, sizeof(record));
			record.valid = EEPROM_RECORD_INVALID;

			if (eeprom_write_device_record(i, &record) != 0)
			{
				return -1;
			}

			return 0;
		}
	}

	return -1; // 未找到
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_remove_device_by_siid(uint8_t siid)
 Input       : siid: SIID
 Output      : 0: 成功, -1: 失败
 Description : 根据SIID从EEPROM中移除设备
---------------------------------------------------------------------------*/
int eeprom_remove_device_by_siid(uint8_t siid)
{
	// 检查SIID是否有效
	if (siid < SIID_MIN || siid > SIID_MAX)
	{
		return -1;
	}

	// 查找设备
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].siid == siid)
		{
			// 清除内存
			memset(&sys_param.paired_inv_info[i], 0, sizeof(inv_device_t));

			// 清除EEPROM
			eeprom_device_record_t record;
			memset(&record, 0, sizeof(record));
			record.valid = EEPROM_RECORD_INVALID;

			if (eeprom_write_device_record(i, &record) != 0)
			{
				return -1;
			}

			return 0;
		}
	}

	return -1; // 未找到
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_find_siid_by_addr(uint32_t sub1g_addr)
 Input       : sub1g_addr: sub1g_inv地址
 Output      : SIID, 0表示未找到
 Description : 根据sub1g_inv地址查找SIID
---------------------------------------------------------------------------*/
uint8_t eeprom_find_siid_by_addr(uint32_t sub1g_addr)
{
	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		if (sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr &&
			sys_param.paired_inv_info[i].siid >= SIID_MIN && sys_param.paired_inv_info[i].siid <= SIID_MAX)
		{
			return sys_param.paired_inv_info[i].siid;
		}
	}

	return 0; // 未找到
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_get_device_count(void)
 Input       : 无
 Output      : 设备数量
 Description : 获取EEPROM中存储的设备数量
---------------------------------------------------------------------------*/
uint8_t eeprom_get_device_count(void)
{
	uint8_t count = 0;

	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		if (sys_param.paired_inv_info[i].siid >= SIID_MIN && sys_param.paired_inv_info[i].siid <= SIID_MAX)
		{
			count++;
		}
	}

	return count;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_clear_all_devices(void)
 Input       : 无
 Output      : 0: 成功, -1: 失败
 Description : 清除EEPROM中所有设备记录
---------------------------------------------------------------------------*/
int eeprom_clear_all_devices(void)
{
	eeprom_device_record_t record;
	memset(&record, 0, sizeof(record));
	record.valid = EEPROM_RECORD_INVALID;

	// 清除所有EEPROM记录
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (eeprom_write_device_record(i, &record) != 0)
		{
			return -1;
		}
	}

	// 清除内存
	memset(sys_param.paired_inv_info, 0, sizeof(sys_param.paired_inv_info));

	return 0;
}
/*---------------------------------------------------------------------------
 Name        : int eeprom_write_user_pair_record(uint8_t index, const eeprom_user_pair_record_t *record)
 Input       : index: 记录索引(0-7), record: 用户配对记录指针
 Output      : 0: 成功, -1: 失败
 Description : 向EEPROM写入单条用户配对记录（32字节，分两页写入）
---------------------------------------------------------------------------*/
int eeprom_write_user_pair_record(uint8_t index, const eeprom_user_pair_record_t *record)
{
	if (index >= EEPROM_USER_PAIR_MAX_NUM || record == NULL)
	{
		return -1;
	}

	// 计算并填充CRC8（不包括crc8字段本身）
	eeprom_user_pair_record_t temp_record = *record;
	temp_record.crc8 = calculate_crc8((const uint8_t *)&temp_record, sizeof(eeprom_user_pair_record_t) - 1);

	// 计算记录的起始地址
	uint16_t base_addr = EEPROM_USER_PAIR_BASE_ADDR + (index * EEPROM_USER_PAIR_RECORD_SIZE);

	// 32字节分两次写入（每次16字节）
	// 第一页：0-15字节
	int32_t ret = eeprom_i2c_write(base_addr, (const uint8_t *)&temp_record, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// EEPROM写入后需要延时等待写入完成
	delay_ms(10);

	// 第二页：16-31字节
	ret = eeprom_i2c_write(base_addr + E2P_PAGE_LEN, ((const uint8_t *)&temp_record) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// EEPROM写入后需要延时等待写入完成
	delay_ms(10);

	// 验证写入
	eeprom_user_pair_record_t verify_record;
	if (eeprom_read_user_pair_record(index, &verify_record) == 0)
	{
		if (memcmp(&temp_record, &verify_record, sizeof(eeprom_user_pair_record_t)) != 0)
		{
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_read_user_pair_record(uint8_t index, eeprom_user_pair_record_t *record)
 Input       : index: 记录索引(0-7), record: 用户配对记录指针
 Output      : 0: 成功, -1: 读取失败, -2: CRC校验失败
 Description : 从EEPROM读取单条用户配对记录（32字节，分两页读取）
---------------------------------------------------------------------------*/
int eeprom_read_user_pair_record(uint8_t index, eeprom_user_pair_record_t *record)
{
	if (index >= EEPROM_USER_PAIR_MAX_NUM || record == NULL)
	{
		return -1;
	}

	// 计算记录的起始地址
	uint16_t base_addr = EEPROM_USER_PAIR_BASE_ADDR + (index * EEPROM_USER_PAIR_RECORD_SIZE);

	// 32字节分两次读取（每次16字节）
	// 第一页：0-15字节
	int32_t ret = eeprom_i2c_read(base_addr, (uint8_t *)record, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// 第二页：16-31字节
	ret = eeprom_i2c_read(base_addr + E2P_PAGE_LEN, ((uint8_t *)record) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// 验证CRC8
	uint8_t calculated_crc = calculate_crc8((const uint8_t *)record, sizeof(eeprom_user_pair_record_t) - 1);
	if (calculated_crc != record->crc8)
	{
		return -2; // CRC校验失败
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_save_user_pair_device_by_sn(const char *device_sn)
 Input       : device_sn: 设备SN
 Output      : 0: 成功, -1: 失败
 Description : 保存单个用户配对设备到EEPROM
---------------------------------------------------------------------------*/
int eeprom_save_user_pair_device_by_sn(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0 || strlen(device_sn) > SN_LENGTH)
	{
		return -1;
	}

	eeprom_user_pair_record_t record;
	int8_t target_slot = -1;

	// 1. 先查找是否已存在该SN
	for (uint8_t i = 0; i < EEPROM_USER_PAIR_MAX_NUM; i++)
	{
		if (sys_param.user_pair_list[i].is_valid &&
			strncmp(sys_param.user_pair_list[i].device_sn, device_sn, SN_LENGTH) == 0)
		{
			target_slot = i;
			break;
		}
	}

	// 2. 如果不存在，查找空槽位
	if (target_slot < 0)
	{
		for (uint8_t i = 0; i < EEPROM_USER_PAIR_MAX_NUM; i++)
		{
			if (!sys_param.user_pair_list[i].is_valid)
			{
				target_slot = i;
				break;
			}
		}
	}

	// 3. 如果没有空槽位，返回失败
	if (target_slot < 0)
	{
		return -1;
	}

	// 4. 准备记录并写入EEPROM
	memset(&record, 0, sizeof(record));

	// 复制SN，最多15字节
	strncpy(record.device_sn, device_sn, SN_LENGTH);
	// 不需要添加'\0'，因为只有15字节

	record.valid = EEPROM_RECORD_VALID;

	if (eeprom_write_user_pair_record(target_slot, &record) != 0)
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_clear_user_pair_list_device_by_sn(const char *device_sn)
 Input       : device_sn: 设备SN
 Output      : 0: 成功, -1: 失败
 Description : 从EEPROM中删除指定SN的用户配对设备
---------------------------------------------------------------------------*/
int eeprom_clear_user_pair_list_device_by_sn(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0)
	{
		return -1;
	}

	// 1. 在内存中查找该SN的槽位
	int8_t target_slot = -1;
	for (uint8_t i = 0; i < EEPROM_USER_PAIR_MAX_NUM; i++)
	{
		if (sys_param.user_pair_list[i].is_valid &&
			strncmp(sys_param.user_pair_list[i].device_sn, device_sn, SN_LENGTH) == 0)
		{
			target_slot = i;
			break;
		}
	}

	// 2. 如果未找到，返回失败
	if (target_slot < 0)
	{
		return -1;
	}

	// 3. 清除EEPROM记录
	eeprom_user_pair_record_t record;
	memset(&record, 0, sizeof(record));
	record.valid = EEPROM_RECORD_INVALID;

	if (eeprom_write_user_pair_record(target_slot, &record) != 0)
	{
		return -1;
	}

	// 4. 清除内存
	sys_param.user_pair_list[target_slot].is_valid = false;
	memset(sys_param.user_pair_list[target_slot].device_sn, 0, sizeof(sys_param.user_pair_list[target_slot].device_sn));

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_load_user_pair_list(void)
 Input       : 无
 Output      : 0: 成功, -1: 失败
 Description : 从EEPROM加载整个用户配对列表（初始化时调用）
---------------------------------------------------------------------------*/
int eeprom_load_user_pair_list(void)
{
	eeprom_user_pair_record_t record;
	bool all_failed = true;

	// 清空内存中的用户配对列表
	memset(sys_param.user_pair_list, 0, sizeof(sys_param.user_pair_list));

	for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
	{
		// 读取EEPROM记录
		if (eeprom_read_user_pair_record(i, &record) != 0)
		{
			// printf("Record %d read failed, skip.\n", i);
			sys_param.user_pair_list[i].is_valid = false; // 仅标记失败
			continue;
		}

		// 检查记录有效性
		if (record.valid == EEPROM_RECORD_VALID)
		{
			strncpy(sys_param.user_pair_list[i].device_sn, record.device_sn, SN_LENGTH);
			sys_param.user_pair_list[i].device_sn[SN_LENGTH] = '\0'; // 添加结束符
			sys_param.user_pair_list[i].is_valid = true;
			all_failed = false;
		}
		else
		{
			sys_param.user_pair_list[i].is_valid = false;
		}
	}

	// 如果所有记录都失败，才返回错误
	return all_failed ? -1 : 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_clear_user_pair_list(void)
 Input       : 无
 Output      : 0: 成功, -1: 失败
 Description : 清空EEPROM中的所有用户配对列表
---------------------------------------------------------------------------*/
int eeprom_clear_user_pair_list(void)
{
	eeprom_user_pair_record_t record;
	memset(&record, 0, sizeof(record));
	record.valid = EEPROM_RECORD_INVALID;

	for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
	{
		if (eeprom_write_user_pair_record(i, &record) != 0)
		{
			return -1;
		}
	}

	// 清空内存
	memset(sys_param.user_pair_list, 0, sizeof(sys_param.user_pair_list));

	return 0;
}

// //------------------------------- 下面是测试使用的函数，为了测试EEPROM  --------------------------------------------

/*---------------------------------------------------------------------------
	Name        : void print_device_list(void)
	Input       : 无
	Output      : 无
	Description : 打印设备列表
---------------------------------------------------------------------------*/
void print_device_list(void)
{
	printf("\r\n2.微逆已经配对列表显示\r\n");
	printf("  索引 | SIID | sub1g地址(3B) | EEPROM地址范围 | 设备SN           | 产品型号  | 所在CTx相序\n");
	printf("  -----|------|---------------|----------------|------------------|-----------|------------\n");
	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		if (sys_param.paired_inv_info[i].siid >= SIID_MIN && sys_param.paired_inv_info[i].siid <= SIID_MAX)
		{
			sys_param.sub1g.state = 2; // 2 - 未发现已配对设备

			uint16_t eeprom_start = E2P_DEVICE_BASE_ADDR + (i * EEPROM_DEVICE_RECORD_SIZE);
			uint16_t eeprom_end = eeprom_start + EEPROM_DEVICE_RECORD_SIZE - 1;
			printf("   %d   |  %d   |  0x%06X     |  0x%04X-0x%04X | %-16s | %s |   CT%d\n",
				   i,
				   sys_param.paired_inv_info[i].siid,
				   sys_param.paired_inv_info[i].sub1g_addr & 0xFFFFFF,
				   eeprom_start,
				   eeprom_end,
				   sys_param.paired_inv_info[i].device_sn,
				   sys_param.paired_inv_info[i].product_model,
				   sys_param.paired_inv_info[i].phase);
		}
	}
	printf("\n=======================================");
	printf("\r\n3.用户配对列表显示\r\n");
	printf(" 索引 | EEPROM地址范围 | 设备SN  \n");
	printf(" ------|------------------|------------------\n");
	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		if (sys_param.user_pair_list[i].is_valid)
		{
			printf("  %-4d | 0x%04X - 0x%04X  | %s\n",
				   i,
				   EEPROM_USER_PAIR_BASE_ADDR + (i * EEPROM_USER_PAIR_RECORD_SIZE),
				   EEPROM_USER_PAIR_BASE_ADDR + ((i + 1) * EEPROM_USER_PAIR_RECORD_SIZE) - 1,
				   sys_param.user_pair_list[i].device_sn);
		}
	}
	printf("\n=======================================\n");
	// 读取CT板SN
	char sn_buffer[16];
	if (eeprom_read_sn(sn_buffer) == 0) // SN读取成功
	{
		sys_param.flash_sn_com_normal = true;
		printf(" CT SN: %s", wifi_info.sn);
	}
	else // SN读取失败
	{
		sys_param.flash_sn_com_normal = false;
		memset(wifi_info.sn, 0, sizeof(wifi_info.sn));
		printf(" CT SN: Not Writen (use MAC address)");
	}
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_update_device_phase(uint32_t sub1g_addr, uint8_t phase)
 Input       : sub1g_addr - SUB1G地址
			   phase - 相位 (0=未识别, 1=A相, 2=B相, 3=C相)
 Output      : 0=成功, -1=失败
 Description : 更新设备的相位信息到EEPROM和内存
---------------------------------------------------------------------------*/
int eeprom_update_device_phase(uint32_t sub1g_addr, uint8_t phase)
{
	if (phase > 3)
	{
		return -1;
	}

	// 查找设备
	int8_t slot = -1;
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr)
		{
			slot = i;
			break;
		}
	}

	if (slot == -1)
	{
		return -1; // 设备不存在
	}

	// 更新内存
	sys_param.paired_inv_info[slot].phase = phase;
	sys_param.paired_inv_info[slot].prop_changed = true;

	// 读取现有EEPROM记录
	eeprom_device_record_t record;
	if (eeprom_read_device_record(slot, &record) != 0)
	{
		return -1;
	}

	// 更新phase字段
	record.phase = phase;

	// 写回EEPROM
	if (eeprom_write_device_record(slot, &record) != 0)
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_load_set_param(void)
 Input       : 无
 Output      : 0: 成功, -1: 失败
 Description : 从EEPROM读取用电量信息、电网功率限制、工作模式、是否允许输出
---------------------------------------------------------------------------*/
int eeprom_load_set_param(void)
{
	eeprom_consumption_record_t record;
	uint16_t reg_addr = EEPROM_CONSUMPTION_BASE_ADDR;
	int result = 0;

	// 读取EEPROM
	int32_t ret = eeprom_i2c_read(reg_addr, (uint8_t *)&record, sizeof(eeprom_consumption_record_t));

	if (ret == LL_OK)
	{
		// 验证CRC
		uint8_t calc_crc = calculate_crc8((uint8_t *)&record, sizeof(eeprom_consumption_record_t) - 1);

		if (record.valid == EEPROM_RECORD_VALID && calc_crc == record.crc8)
		{
			// DEBUG_PRINTF("EEPROM: CRC success record.electricity_consumption = %d \r\n", record.electricity_consumption);

			// CRC校验通过，加载配置参数
			sys_param.to_grid_power_limit = record.to_grid_power_limit;
			sys_param.power_work_mode = record.power_work_mode;
			sys_param.anti_backflow_switch = record.antiflow_enable;

			// 只有sequence_k在1-6范围内且tag==sequence_k*10+sequence_k时才使用存储值
			if (record.sequence_k >= 1 && record.sequence_k <= 6 &&
				record.sequence_k_tag == record.sequence_k * 10 + record.sequence_k)
			{
				DEBUG_PRINTF("EEPROM: sequence_k = %d, record.sequence_k_tag = %d, valid!\r\n", record.sequence_k, record.sequence_k_tag);
				// 伴生校验通过：使用存储的相序值
				sys_param.grid.phase_id.sequence_k = record.sequence_k;
				sys_param.grid.phase_id.identification_valid = 1;
				update_ct_to_phase_mapping(record.sequence_k);
			}
			else
			{
				DEBUG_PRINTF("EEPROM: sequence_k = %d, record.sequence_k_tag = %d, invalid, use default sequence_k = 1, Save EEP\r\n", record.sequence_k, record.sequence_k_tag);
				// 校验不通过、sequence_k=0：默认sequence_k=1
				sys_param.grid.phase_id.sequence_k = 1;
				sys_param.grid.phase_id.identification_valid = 1;
				update_ct_to_phase_mapping(1);
				eeprom_save_set_param();
			}

			// 加载CT功率方向（仅接受1或-1，否则默认1）
			sys_param.ct1.power.power_direction = (record.ct1_dir == -1) ? -1.0f : 1.0f;
			sys_param.ct2.power.power_direction = (record.ct2_dir == -1) ? -1.0f : 1.0f;
			sys_param.ct3.power.power_direction = (record.ct3_dir == -1) ? -1.0f : 1.0f;

			return 0;
		}
		else
		{
			// CRC校验失败或记录无效
			result = -2;
		}
	}
	else
	{
		// I2C读取失败
		result = -1;
	}

	// 读取失败或CRC错误：统一设置默认值并写回EEPROM
	sys_param.to_grid_power_limit = 0;
	sys_param.power_work_mode = 1;
	sys_param.anti_backflow_switch = 1;
	sys_param.grid.phase_id.sequence_k = 1;
	sys_param.grid.phase_id.identification_valid = 1;
	update_ct_to_phase_mapping(1);
	// CT方向保持system_param_init已设置的默认值1，无需重复赋值
	eeprom_save_set_param();
	DEBUG_PRINTF("EEPROM: Load failed! result = %d, set default and save eep \r\n", result);

	return result;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_save_set_param()
 Input       : consumption - 用电量(Wh)
 Output      : 0: 成功, -1: 失败
 Description : 保存用电量到EEPROM
---------------------------------------------------------------------------*/
int eeprom_save_set_param()
{
	eeprom_consumption_record_t record;
	uint16_t reg_addr = EEPROM_CONSUMPTION_BASE_ADDR;

	// 准备记录
	memset(&record, 0, sizeof(record));
	record.electricity_consumption = 0; // 配置区不再存储用电量
	record.to_grid_power_limit = sys_param.to_grid_power_limit;
	record.power_work_mode = sys_param.power_work_mode;
	record.antiflow_enable = sys_param.anti_backflow_switch;
	record.sequence_k = sys_param.grid.phase_id.sequence_k;
	record.sequence_k_tag = record.sequence_k * 10 + record.sequence_k;
	record.ct1_dir = (sys_param.ct1.power.power_direction < 0.0f) ? (int8_t)(-1) : (int8_t)(1);
	record.ct2_dir = (sys_param.ct2.power.power_direction < 0.0f) ? (int8_t)(-1) : (int8_t)(1);
	record.ct3_dir = (sys_param.ct3.power.power_direction < 0.0f) ? (int8_t)(-1) : (int8_t)(1);
	record.valid = EEPROM_RECORD_VALID;

	// 计算CRC
	record.crc8 = calculate_crc8((uint8_t *)&record, sizeof(eeprom_consumption_record_t) - 1);

	// 写入EEPROM
	int32_t ret = eeprom_i2c_write(reg_addr, (uint8_t *)&record, sizeof(eeprom_consumption_record_t));

	if (ret != LL_OK)
	{
		DEBUG_PRINTF(" Save failed!");
		return -1;
	}
	DEBUG_PRINTF("Save sequence_k = %d, sequence_k_tag = %d,", sys_param.grid.phase_id.sequence_k, record.sequence_k_tag);

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_save_elec_consumption(void)
 Input       : 无
 Output      : 0: 成功, -1: 失败
 Description : 将用电量单独保存到独立EEPROM页（频繁写入区域）
---------------------------------------------------------------------------*/
int eeprom_save_elec_consumption(void)
{
	eeprom_elec_record_t record;
	uint16_t reg_addr = EEPROM_ELEC_BASE_ADDR;

	memset(&record, 0, sizeof(record));
	record.electricity_consumption = sys_param.hmi.electricity_consumption;
	record.valid = EEPROM_RECORD_VALID;
	record.crc8 = calculate_crc8((uint8_t *)&record, sizeof(eeprom_elec_record_t) - 1);

	int32_t ret = eeprom_i2c_write(reg_addr, (uint8_t *)&record, sizeof(eeprom_elec_record_t));
	if (ret != LL_OK)
	{
		DEBUG_PRINTF("[EEPROM] Elec consumption save failed!\r\n");
		return -1;
	}

	delay_ms(10);
	DEBUG_PRINTF("[EEPROM] Elec consumption saved: %u Wh\r\n", sys_param.hmi.electricity_consumption);
	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_load_elec_consumption(void)
 Input       : 无
 Output      : 0: 成功, -1: 失败
 Description : 从独立EEPROM页加载用电量
---------------------------------------------------------------------------*/
int eeprom_load_elec_consumption(void)
{
	eeprom_elec_record_t record;
	uint16_t reg_addr = EEPROM_ELEC_BASE_ADDR;

	int32_t ret = eeprom_i2c_read(reg_addr, (uint8_t *)&record, sizeof(eeprom_elec_record_t));
	if (ret == LL_OK)
	{
		uint8_t calc_crc = calculate_crc8((uint8_t *)&record, sizeof(eeprom_elec_record_t) - 1);
		if (record.valid == EEPROM_RECORD_VALID && calc_crc == record.crc8)
		{
			sys_param.hmi.electricity_consumption = record.electricity_consumption;
			DEBUG_PRINTF("[EEPROM] Elec consumption loaded: %u Wh\r\n", sys_param.hmi.electricity_consumption);
			return 0;
		}
		else
		{
			DEBUG_PRINTF("[EEPROM] Elec consumption CRC error or invalid, default 0\r\n");
		}
	}
	else
	{
		DEBUG_PRINTF("[EEPROM] Elec consumption read failed, default 0\r\n");
	}

	// 读取失败或CRC错误：默认0
	sys_param.hmi.electricity_consumption = 0;
	return -1;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_migrate_elec_consumption(void)
 Input       : 无
 Output      : 0: 已迁移, 1: 无需迁移(旧区域为0), -1: 旧区域读取失败/CRC无效
 Description : OTA升级兼容，仅在新独立区域无效时调用。
			   读旧配置区electricity_consumption，非0则迁移到新区域并清零旧区域。
			   旧区域CRC校验失败时跳过（数据不可信），默认值0由load函数已设置。
---------------------------------------------------------------------------*/
int eeprom_migrate_elec_consumption(void)
{
	eeprom_consumption_record_t old_record;
	uint16_t reg_addr = EEPROM_CONSUMPTION_BASE_ADDR;

	// 读旧配置区完整16字节
	int32_t ret = eeprom_i2c_read(reg_addr, (uint8_t *)&old_record, sizeof(eeprom_consumption_record_t));
	if (ret != LL_OK)
	{
		DEBUG_PRINTF("[EEPROM] Migration: old area read failed, skip\r\n");
		return -1;
	}

	// CRC校验
	uint8_t calc_crc = calculate_crc8((uint8_t *)&old_record, sizeof(eeprom_consumption_record_t) - 1);
	if (old_record.valid != EEPROM_RECORD_VALID || calc_crc != old_record.crc8)
	{
		DEBUG_PRINTF("[EEPROM] Migration: old area CRC invalid, skip\r\n");
		return -1;
	}

	// 检查旧区域的electricity_consumption是否非0
	if (old_record.electricity_consumption == 0)
	{
		DEBUG_PRINTF("[EEPROM] Migration: old elec=0, no need to migrate\r\n");
		return 1;
	}

	// 将旧值写入新独立区域
	DEBUG_PRINTF("[EEPROM] Migration: old elec=%u Wh, migrating...\r\n", old_record.electricity_consumption);
	sys_param.hmi.electricity_consumption = old_record.electricity_consumption;
	int save_ret = eeprom_save_elec_consumption();
	if (save_ret != 0)
	{
		DEBUG_PRINTF("[EEPROM] Migration: save to new area failed!\r\n");
		return -1;
	}

	// 旧区域electricity_consumption清0并回写
	old_record.electricity_consumption = 0;
	old_record.crc8 = calculate_crc8((uint8_t *)&old_record, sizeof(eeprom_consumption_record_t) - 1);
	ret = eeprom_i2c_write(reg_addr, (uint8_t *)&old_record, sizeof(eeprom_consumption_record_t));
	if (ret != LL_OK)
	{
		DEBUG_PRINTF("[EEPROM] Migration: clear old area failed!\r\n");
		return -1;
	}
	delay_ms(10);

	DEBUG_PRINTF("[EEPROM] Migration: done, %u Wh moved to new area\r\n", sys_param.hmi.electricity_consumption);
	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_write_sn(const char *sn)
 Input       : sn - 15字节的SN字符串
 Output      : 0: 成功, -1: 失败
 Description : 将CT板SN写入EEPROM
---------------------------------------------------------------------------*/
int eeprom_write_sn(const char *sn)
{
	// 参数检查
	if (sn == NULL)
	{
		return -1;
	}

	// 检查SN长度（必须为15字节）
	size_t sn_len = strlen(sn);
	if (sn_len != SN_LENGTH)
	{
		return -1;
	}

	eeprom_sn_record_t record;
	memset(&record, 0, sizeof(eeprom_sn_record_t));

	memcpy(record.device_sn, sn, SN_LENGTH);
	record.valid = EEPROM_RECORD_VALID;

	uint16_t addr = EEPROM_SN_BASE_ADDR;
	int32_t ret = eeprom_i2c_write(addr, (const uint8_t *)&record, EEPROM_SN_SIZE);

	if (ret != LL_OK)
	{
		return -1;
	}

	delay_ms(10);

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_read_sn(char *sn)
 Input       : sn - 用于存储读取的SN的缓冲区（至少16字节）
 Output      : 0: 成功, -1: 失败
 Description : 从EEPROM读取CT板SN，并自动复制到wifi_info
---------------------------------------------------------------------------*/
int eeprom_read_sn(char *sn)
{
	if (sn == NULL)
	{
		return -1;
	}

	// 读取EEPROM数据
	eeprom_sn_record_t record;
	uint16_t addr = EEPROM_SN_BASE_ADDR;
	int32_t ret = eeprom_i2c_read(addr, (uint8_t *)&record, EEPROM_SN_SIZE);

	if (ret != LL_OK)
	{
		return -1;
	}

	if (record.valid != EEPROM_RECORD_VALID)
	{
		return -1;
	}

	// 复制SN到输出缓冲区（15字节）
	memcpy(sn, record.device_sn, SN_LENGTH);
	sn[SN_LENGTH] = '\0'; // 添加字符串结束符

	// 将SN复制到wifi_info
	memcpy(wifi_info.sn, sn, SN_LENGTH);
	wifi_info.sn[SN_LENGTH] = '\0';

	return 0;
}

/*eof*/

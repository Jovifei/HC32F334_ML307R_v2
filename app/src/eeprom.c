//
// Included Files
//
#include "board.h"
#include "main.h"
#include "eeprom.h"
#include "wifi_info.h"
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
 Input       : data: ����ָ��, len: ���ݳ���
 Output      : CRC8У����
 Description : ����CRC8У����
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
 Input       : record: ��¼ָ��
 Output      : CRC8У����
 Description : ��֤��¼��CRC8У����
---------------------------------------------------------------------------*/
bool verify_record_crc8(const eeprom_device_record_t *record)
{
	// ����CRC��-1�ǲ���������crc8�ֶ�
	uint8_t calculated_crc = calculate_crc8((const uint8_t *)record, sizeof(eeprom_device_record_t) - 1);

	return (calculated_crc == record->crc8);
}

/*---------------------------------------------------------------------------
 Name        : const char* product_model_code_to_string(uint8_t code)
 Input       : code: ��Ʒ�ͺŴ���
 Output      : ��Ʒ�ͺ��ַ���
 Description : ����Ʒ�ͺŴ���ת��Ϊ�ַ���
---------------------------------------------------------------------------*/
const char *product_model_code_to_string(uint8_t code)
{
	switch (code)
	{
	case PRODUCT_MODEL_CODE_MI800S:
		return "GE-MI800S";
	case PRODUCT_MODEL_CODE_MI2500S:
		return "GE-MI2500S";
	// ��������չ������Ʒ�ͺ�...
	default:
		return "UNKNOWN";
	}
}

/*---------------------------------------------------------------------------
 Name        : uint8_t product_model_string_to_code(const char *model_str)
 Input       : model_str: ��Ʒ�ͺ��ַ���
 Output      : ��Ʒ�ͺŴ���
 Description : ����Ʒ�ͺ��ַ���ת��Ϊ����
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
	// ��������չ������Ʒ�ͺ�...

	return 0; // δ֪�ͺ�
}

/*---------------------------------------------------------------------------
 Name        : static int32_t eeprom_i2c_write(uint16_t addr, const uint8_t *data, uint32_t len)
 Description : EEPROMд�븨����������ȷ��HT24LC08��ַ������
---------------------------------------------------------------------------*/
static int32_t eeprom_i2c_write(uint16_t addr, const uint8_t *data, uint32_t len)
{
	int32_t ret;

	// ��ȡ10λ��ַ�ĸ�2λ��ҳѡ��λP1,P0��
	uint8_t page_bits = (addr >> 8) & 0x03;

	// �����豸��ַ��������ַ0x50 + ҳѡ��λP1,P0
	// �豸��ַ��ʽ: 1 0 1 0 A2 P1 P0 R/W
	// ����A2=0��������ַΪ0x50��ҳѡ��λ����1λ����P1,P0λ��
	uint8_t device_addr = EEPROM_I2C_ADDR | (page_bits);

	// ��ַ�ĵ�8λ
	uint8_t low_addr_byte = (uint8_t)(addr & 0xFF);

	// ʹ��1�ֽڵ�ַģʽ��ʵ�ʷ��͵���10λ��ַ�ĵ�8λ��
	ret = BSP_I2C_Write(CM_I2C, device_addr, &low_addr_byte, 1, data, len);

	return ret;
}

/*---------------------------------------------------------------------------
 Name        : static int32_t eeprom_i2c_read(uint16_t addr, uint8_t *data, uint32_t len)
 Description : EEPROM��ȡ������������ȷ��HT24LC08��ַ������
---------------------------------------------------------------------------*/
static int32_t eeprom_i2c_read(uint16_t addr, uint8_t *data, uint32_t len)
{
	int32_t ret;

	// ��ȡ10λ��ַ�ĸ�2λ��ҳѡ��λP1,P0��
	uint8_t page_bits = (addr >> 8) & 0x03;

	// �����豸��ַ��������ַ0x50 + ҳѡ��λP1,P0
	uint8_t device_addr = EEPROM_I2C_ADDR | (page_bits);

	// ��ַ�ĵ�8λ
	uint8_t low_addr_byte = (uint8_t)(addr & 0xFF);

	// ʹ��1�ֽڵ�ַģʽ
	ret = BSP_I2C_Read(CM_I2C, device_addr, &low_addr_byte, 1, data, len);

	return ret;
}

/*---------------------------------------------------------------------------
 Name        : static int eeprom_read_device_record(uint8_t index, eeprom_device_record_t *record)
 Input       : index: �豸����, record: ��¼ָ��
 Output      : 0: �ɹ�, -1: ��ȡʧ��, -2: CRCУ��ʧ��
 Description : ��EEPROM�ж�ȡ�豸��¼ (48�ֽڣ���3�ζ�ȡ��֧��2�ֽڵ�ַ)
---------------------------------------------------------------------------*/
static int eeprom_read_device_record(uint8_t index, eeprom_device_record_t *record)
{
	if (index >= EEPROM_MAX_DEVICES || record == NULL)
	{
		return -1;
	}

	// �����¼����ʼ��ַ
	uint16_t base_addr = E2P_DEVICE_BASE_ADDR + (index * EEPROM_DEVICE_RECORD_SIZE);

	// 48�ֽڷ����ζ�ȡ��ÿ��16�ֽڣ�
	// ��һҳ��0-15�ֽ�
	uint16_t addr1 = base_addr;
	int32_t ret = eeprom_i2c_read(addr1, (uint8_t *)record, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// �ڶ�ҳ��16-31�ֽ�
	uint16_t addr2 = base_addr + E2P_PAGE_LEN;
	ret = eeprom_i2c_read(addr2, ((uint8_t *)record) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// ����ҳ��32-47�ֽ�
	uint16_t addr3 = base_addr + (E2P_PAGE_LEN * 2);
	ret = eeprom_i2c_read(addr3, ((uint8_t *)record) + (E2P_PAGE_LEN * 2), E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// // ��ӡ�����ļ�¼����
	// printf("Data to read  [%d bytes]: ", sizeof(eeprom_device_record_t));
	// for (int i = 0; i < sizeof(eeprom_device_record_t); i++)
	// {
	// 	printf("%02X ", *(((uint8_t *)record) + i));
	// }
	// printf("\r\n");

	// ��֤CRC8
	if (!verify_record_crc8(record))
	{
		return -2; // CRCУ��ʧ��
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : static int eeprom_write_device_record(uint8_t index, const eeprom_device_record_t *record)
 Input       : index: �豸����, record: ��¼ָ��
 Output      : 0: �ɹ�, -1: д��ʧ��
 Description : ��EEPROM��д���豸��¼ (48�ֽڣ���3��д�룬֧��2�ֽڵ�ַ)
---------------------------------------------------------------------------*/
static int eeprom_write_device_record(uint8_t index, const eeprom_device_record_t *record)
{
	if (index >= EEPROM_MAX_DEVICES || record == NULL)
	{
		return -1;
	}

	// ��������������CRC8
	eeprom_device_record_t record_copy;
	memcpy(&record_copy, record, sizeof(eeprom_device_record_t));
	record_copy.crc8 = calculate_crc8((const uint8_t *)&record_copy, sizeof(eeprom_device_record_t) - 1);

	// �����¼����ʼ��ַ
	uint16_t base_addr = E2P_DEVICE_BASE_ADDR + (index * EEPROM_DEVICE_RECORD_SIZE);

	// // ��ӡҪд����������ݣ�����CRC8��
	// printf("Data to write [%d bytes]: ", sizeof(eeprom_device_record_t));
	// for (int i = 0; i < sizeof(eeprom_device_record_t); i++)
	// {
	// 	printf("%02X ", *(((uint8_t *)&record_copy) + i));
	// }

	// 48�ֽڷ�����д�루ÿ��16�ֽڣ�
	// ��һҳ��0-15�ֽ�
	uint16_t addr1 = base_addr;
	int32_t ret = eeprom_i2c_write(addr1, (const uint8_t *)&record_copy, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	delay_ms(10); // �ȴ�ҳд�����

	// �ڶ�ҳ��16-31�ֽ�
	uint16_t addr2 = base_addr + E2P_PAGE_LEN;
	ret = eeprom_i2c_write(addr2, ((const uint8_t *)&record_copy) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}
	delay_ms(10); // �ȴ�ҳд�����

	// ����ҳ��32-47�ֽ�
	uint16_t addr3 = base_addr + (E2P_PAGE_LEN * 2);
	ret = eeprom_i2c_write(addr3, ((const uint8_t *)&record_copy) + (E2P_PAGE_LEN * 2), E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}
	delay_ms(10); // �ȴ�ҳд�����

	// ��֤д��
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
 Input       : ��
 Output      : SIID, 0��ʾδ�ҵ�
 Description : ���ҿ��е�SIID
---------------------------------------------------------------------------*/
static uint8_t find_free_siid(void)
{
	bool siid_used[SIID_MAX - SIID_MIN + 1] = {false};

	// �����ʹ�õ�SIID
	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		uint8_t siid = sys_param.paired_inv_info[i].siid;
		if (siid >= SIID_MIN && siid <= SIID_MAX)
		{
			siid_used[siid - SIID_MIN] = true;
		}
	}

	// ���ҵ�һ��δʹ�õ�SIID
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
 Input       : ��
 Output      : 0: �ɹ�
 Description : ��ʼ���������豸��Ϣ
---------------------------------------------------------------------------*/
int eeprom_init_and_load_devices(void)
{
	eeprom_device_record_t record;
	int read_result;

	// ����ڴ��е��豸��Ϣ
	memset(sys_param.paired_inv_info, 0, sizeof(sys_param.paired_inv_info));

	// ��EEPROM��ȡ�����豸��¼
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		read_result = eeprom_read_device_record(i, &record);

		if (read_result == 0) // ��ȡ�ɹ���CRCУ��ͨ��
		{
			if (record.valid == EEPROM_RECORD_VALID && record.siid >= SIID_MIN && record.siid <= SIID_MAX)
			{
				// ���ƻ�����Ϣ
				memcpy(sys_param.paired_inv_info[i].device_sn, record.device_sn, SN_LENGTH);
				sys_param.paired_inv_info[i].device_sn[SN_LENGTH] = '\0';
				sys_param.paired_inv_info[i].sub1g_addr = record.sub1g_addr;
				sys_param.paired_inv_info[i].siid = record.siid;
				sys_param.paired_inv_info[i].is_valid = true;
				sys_param.paired_inv_info[i].online_state = 1; // ΢�治���ߣ��ȴ��ϱ�����Ϊ����
				sys_param.paired_inv_info[i].offline_updata_ms = PAIRED_INV_ONLINE_TIMEOUT_S;

				// ���Ʋ�Ʒ�ͺ�
				strncpy(sys_param.paired_inv_info[i].product_model, record.product_model, PRODUCT_MODEL_MAX_LEN);
				sys_param.paired_inv_info[i].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

				// ������λ��Ϣ
				sys_param.paired_inv_info[i].phase = record.phase;
			}
			else
			{
				sys_param.paired_inv_info[i].online_state = 0; // ��ȡʧ�ܣ�����û��ƥ���΢��
			}
		}
		else if (read_result == -2) // CRCУ��ʧ��
		{
			sys_param.paired_inv_info[i].online_state = 0; // ��ȡʧ�ܣ�����û��ƥ���΢��
		}
		else if (read_result == -1) // ��ȡʧ��
		{
			sys_param.paired_inv_info[i].online_state = 0; // ��ȡʧ�ܣ�����û��ƥ���΢��
		}
	}

	// �����û�����б�
	eeprom_load_user_pair_list();

	// �������ò���
	int consumption_ret = eeprom_load_set_param();
	if (consumption_ret == 0)
	{
		printf("\n1.EEPROM���ò���\n");
		printf(" ��������               | ����ֵ\n");
		printf(" -----------------------|--------------\n");
		printf(" ������������           | %d\n", sys_param.to_grid_power_limit);
		printf(" ���ʹ���ģʽ           | %d\n", sys_param.power_work_mode);
		printf(" ����������             | %d\n", sys_param.anti_backflow_switch);
		printf(" CT����(sequence_k)     | %d\n", sys_param.grid.phase_id.sequence_k);
		printf(" CT1����                | %d\n", (int)(sys_param.ct1.power.power_direction));
		printf(" CT2����                | %d\n", (int)(sys_param.ct2.power.power_direction));
		printf(" CT3����                | %d\n", (int)(sys_param.ct3.power.power_direction));
	}
	else if (consumption_ret == -2)
	{
		printf("EEPROM: Config CRC error, reset to default and saved\n");
	}
	else
	{
		printf("EEPROM: Config read failed, reset to default and saved\n");
	}

	// ��������Ч��ֱ��ʹ��
	int elec_ret = eeprom_load_elec_consumption();
	if (elec_ret != 0)
	{
		// ��������Ч�����ԴӾ�������Ǩ�ƣ��������ݣ�
		eeprom_migrate_elec_consumption();
	}
	printf(" �õ���Wh               | %u\n", sys_param.hmi.electricity_consumption);
	printf("=======================================\n");

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_add_device(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code)
 Input       : device_sn: �豸���к�, sub1g_addr: sub1g_inv��ַ, product_model_code: ��Ʒ�ͺŴ���
 Output      : SIID, 0��ʾ����ʧ��
 Description : ����΢���SN��sub1g��ַ�Ͳ�Ʒ�ͺ������豸��EEPROM
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

	// ���ҿ���eeprom����
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
		return 0; // �ռ�����
	}

	// ����SIID
	uint8_t new_siid = find_free_siid();
	if (new_siid == 0)
	{
		return 0; // �޿���SIID
	}

	// ����Ʒ�ͺŴ���ת��Ϊ�ַ���
	const char *model_str = product_model_code_to_string(product_model_code);

	// �����ڴ�
	memset(sys_param.paired_inv_info[free_slot].device_sn, 0, sizeof(sys_param.paired_inv_info[free_slot].device_sn));
	strncpy(sys_param.paired_inv_info[free_slot].device_sn, device_sn, SN_LENGTH);
	sys_param.paired_inv_info[free_slot].device_sn[SN_LENGTH] = '\0';
	sys_param.paired_inv_info[free_slot].sub1g_addr = sub1g_addr;
	sys_param.paired_inv_info[free_slot].siid = new_siid;
	sys_param.paired_inv_info[free_slot].is_valid = true;

	// �����Ʒ�ͺ��ַ���
	strncpy(sys_param.paired_inv_info[free_slot].product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	sys_param.paired_inv_info[free_slot].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	// ��ʼ��phaseΪ0��δʶ�𣩣�֮����������ʶ��
	sys_param.paired_inv_info[free_slot].phase = 0;

	// printf("Added device with SIID %u to slot %d\n", new_siid, free_slot);

	// д��EEPROM
	eeprom_device_record_t record;
	memset(&record, 0, sizeof(record));
	strncpy(record.device_sn, device_sn, SN_LENGTH);
	record.device_sn[SN_LENGTH] = '\0';
	record.sub1g_addr = sub1g_addr;
	record.siid = new_siid;
	record.valid = EEPROM_RECORD_VALID;
	record.phase = 0; // ��ʼ��Ϊδʶ��

	// �����Ʒ�ͺŵ�EEPROM
	strncpy(record.product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	record.product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	int write_ret = eeprom_write_device_record(free_slot, &record);
	if (write_ret != 0)
	{
		// д��ʧ�ܣ�����ڴ�
		memset(&sys_param.paired_inv_info[free_slot], 0, sizeof(inv_device_t));
		return 0;
	}

	return new_siid;
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_add_device_by_sn_only(const char *device_sn)
 Input       : device_sn: �豸���к�
 Output      : SIID, 0��ʾ����ʧ��
 Description : ֻ����SNԤ�����豸��sub1g_addr����Ϊ0���ȴ�΢��㲥ʱ����
			   �����û�ɨ���ʱ��������SIID����̨����ֱ�Ӷ�ȡ�豸��Ϣ
---------------------------------------------------------------------------*/
uint8_t eeprom_add_device_by_sn_only(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0 || strlen(device_sn) > SN_LENGTH)
	{
		return 0;
	}

	// ����Ƿ��Ѿ�������ͬSN���豸
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].is_valid)
		{
			// ʹ�ô洢SN��ʵ�ʳ��Ƚ���ǰ׺ƥ��
			size_t stored_len = strlen(sys_param.paired_inv_info[i].device_sn);
			if (stored_len > 0 && strncmp(sys_param.paired_inv_info[i].device_sn, device_sn, stored_len) == 0)
			{
				// �Ѵ��ڣ������ѷ����SIID
				return sys_param.paired_inv_info[i].siid;
			}
		}
	}

	// ���ҿ���eeprom����
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

	// ����SIID
	uint8_t new_siid = find_free_siid();
	if (new_siid == 0)
	{
		return 0; // �޿���SIID
	}

	// �����ڴ�
	memset(&sys_param.paired_inv_info[free_slot], 0, sizeof(inv_device_t));
	strncpy(sys_param.paired_inv_info[free_slot].device_sn, device_sn, SN_LENGTH);
	sys_param.paired_inv_info[free_slot].device_sn[SN_LENGTH] = '\0';
	sys_param.paired_inv_info[free_slot].sub1g_addr = 0; // ��ʱΪ0���ȴ�΢��㲥ʱ����
	sys_param.paired_inv_info[free_slot].siid = new_siid;
	sys_param.paired_inv_info[free_slot].is_valid = true;
	sys_param.paired_inv_info[free_slot].online_state = 1; // ���δ����
	sys_param.paired_inv_info[free_slot].phase = 0;		   // δʶ����λ

	// ��Ʒ�ͺ���ʱ����Ϊδ֪���ȴ�΢��㲥ʱ����
	strncpy(sys_param.paired_inv_info[free_slot].product_model, "UNKNOWN", PRODUCT_MODEL_MAX_LEN);
	sys_param.paired_inv_info[free_slot].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	// д��EEPROM
	eeprom_device_record_t record;
	memset(&record, 0, sizeof(record));
	strncpy(record.device_sn, device_sn, SN_LENGTH);
	record.device_sn[SN_LENGTH] = '\0';
	record.sub1g_addr = 0; // ��ʱΪ0
	record.siid = new_siid;
	record.valid = EEPROM_RECORD_VALID;
	record.phase = 0;
	strncpy(record.product_model, "UNKNOWN", PRODUCT_MODEL_MAX_LEN);
	record.product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	int write_ret = eeprom_write_device_record(free_slot, &record);
	if (write_ret != 0)
	{
		// д��ʧ�ܣ�����ڴ�
		memset(&sys_param.paired_inv_info[free_slot], 0, sizeof(inv_device_t));
		return 0;
	}

	return new_siid;
}

/*---------------------------------------------------------------------------
 Name        : int8_t eeprom_find_inv_index_by_sn(const char *device_sn)
 Input       : device_sn: �豸���к�
 Output      : �豸������0-7����δ�ҵ�����-1
 Description : ����SN����������豸������
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
 Input       : device_sn: �豸���к�
			   sub1g_addr: SUB1G��ַ
			   product_model_code: ��Ʒ�ͺŴ���
 Output      : 0=�ɹ�, -1=ʧ��
 Description : ����SN������Ԥ�����豸��sub1g��ַ�Ͳ�Ʒ�ͺ�
			   ����΢��㲥ʱ����Ԥ�����豸����Ϣ
---------------------------------------------------------------------------*/
int eeprom_update_device_sub1g_addr(const char *device_sn, uint32_t sub1g_addr, uint8_t product_model_code)
{
	if (device_sn == NULL || strlen(device_sn) == 0)
	{
		return -1;
	}

	// �����豸
	int8_t slot = eeprom_find_inv_index_by_sn(device_sn);
	if (slot < 0)
	{
		return -1; // �豸������
	}

	// ����Ʒ�ͺŴ���ת��Ϊ�ַ���
	const char *model_str = product_model_code_to_string(product_model_code);

	// �����ڴ�
	sys_param.paired_inv_info[slot].sub1g_addr = sub1g_addr;
	strncpy(sys_param.paired_inv_info[slot].product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	sys_param.paired_inv_info[slot].product_model[PRODUCT_MODEL_MAX_LEN] = '\0';
	sys_param.paired_inv_info[slot].online_state = 2; // �豸����
	sys_param.paired_inv_info[slot].prop_changed = true;

	// ��ȡ����EEPROM��¼
	eeprom_device_record_t record;
	if (eeprom_read_device_record(slot, &record) != 0)
	{
		return -1;
	}

	// ���¼�¼
	record.sub1g_addr = sub1g_addr;
	strncpy(record.product_model, model_str, PRODUCT_MODEL_MAX_LEN);
	record.product_model[PRODUCT_MODEL_MAX_LEN] = '\0';

	// д��EEPROM
	if (eeprom_write_device_record(slot, &record) != 0)
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_remove_device_by_sub1g_addr(uint32_t sub1g_addr)
 Input       : sub1g_addr: sub1g_inv��ַ
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ����sub1g_inv��ַ��EEPROM���Ƴ��豸
---------------------------------------------------------------------------*/
int eeprom_remove_device_by_sub1g_addr(uint32_t sub1g_addr)
{
	// �����豸
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].sub1g_addr == sub1g_addr && sys_param.paired_inv_info[i].siid != 0)
		{
			// ����ڴ�
			memset(&sys_param.paired_inv_info[i], 0, sizeof(inv_device_t));

			// ���EEPROM��д��0��ʾ��Ч
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

	return -1; // δ�ҵ�
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_remove_device_by_siid(uint8_t siid)
 Input       : siid: SIID
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ����SIID��EEPROM���Ƴ��豸
---------------------------------------------------------------------------*/
int eeprom_remove_device_by_siid(uint8_t siid)
{
	// ���SIID�Ƿ���Ч
	if (siid < SIID_MIN || siid > SIID_MAX)
	{
		return -1;
	}

	// �����豸
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (sys_param.paired_inv_info[i].siid == siid)
		{
			// ����ڴ�
			memset(&sys_param.paired_inv_info[i], 0, sizeof(inv_device_t));

			// ���EEPROM
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

	return -1; // δ�ҵ�
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_find_siid_by_addr(uint32_t sub1g_addr)
 Input       : sub1g_addr: sub1g_inv��ַ
 Output      : SIID, 0��ʾδ�ҵ�
 Description : ����sub1g_inv��ַ����SIID
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

	return 0; // δ�ҵ�
}

/*---------------------------------------------------------------------------
 Name        : uint8_t eeprom_get_device_count(void)
 Input       : ��
 Output      : �豸����
 Description : ��ȡEEPROM�д洢���豸����
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
 Input       : ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ���EEPROM�������豸��¼
---------------------------------------------------------------------------*/
int eeprom_clear_all_devices(void)
{
	eeprom_device_record_t record;
	memset(&record, 0, sizeof(record));
	record.valid = EEPROM_RECORD_INVALID;

	// �������EEPROM��¼
	for (uint8_t i = 0; i < EEPROM_MAX_DEVICES; i++)
	{
		if (eeprom_write_device_record(i, &record) != 0)
		{
			return -1;
		}
	}

	// ����ڴ�
	memset(sys_param.paired_inv_info, 0, sizeof(sys_param.paired_inv_info));

	return 0;
}
/*---------------------------------------------------------------------------
 Name        : int eeprom_write_user_pair_record(uint8_t index, const eeprom_user_pair_record_t *record)
 Input       : index: ��¼����(0-7), record: �û���Լ�¼ָ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ��EEPROMд�뵥���û���Լ�¼��32�ֽڣ�����ҳд�룩
---------------------------------------------------------------------------*/
int eeprom_write_user_pair_record(uint8_t index, const eeprom_user_pair_record_t *record)
{
	if (index >= EEPROM_USER_PAIR_MAX_NUM || record == NULL)
	{
		return -1;
	}

	// ���㲢���CRC8��������crc8�ֶα�����
	eeprom_user_pair_record_t temp_record = *record;
	temp_record.crc8 = calculate_crc8((const uint8_t *)&temp_record, sizeof(eeprom_user_pair_record_t) - 1);

	// �����¼����ʼ��ַ
	uint16_t base_addr = EEPROM_USER_PAIR_BASE_ADDR + (index * EEPROM_USER_PAIR_RECORD_SIZE);

	// 32�ֽڷ�����д�루ÿ��16�ֽڣ�
	// ��һҳ��0-15�ֽ�
	int32_t ret = eeprom_i2c_write(base_addr, (const uint8_t *)&temp_record, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// EEPROMд�����Ҫ��ʱ�ȴ�д�����
	delay_ms(10);

	// �ڶ�ҳ��16-31�ֽ�
	ret = eeprom_i2c_write(base_addr + E2P_PAGE_LEN, ((const uint8_t *)&temp_record) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// EEPROMд�����Ҫ��ʱ�ȴ�д�����
	delay_ms(10);

	// ��֤д��
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
 Input       : index: ��¼����(0-7), record: �û���Լ�¼ָ��
 Output      : 0: �ɹ�, -1: ��ȡʧ��, -2: CRCУ��ʧ��
 Description : ��EEPROM��ȡ�����û���Լ�¼��32�ֽڣ�����ҳ��ȡ��
---------------------------------------------------------------------------*/
int eeprom_read_user_pair_record(uint8_t index, eeprom_user_pair_record_t *record)
{
	if (index >= EEPROM_USER_PAIR_MAX_NUM || record == NULL)
	{
		return -1;
	}

	// �����¼����ʼ��ַ
	uint16_t base_addr = EEPROM_USER_PAIR_BASE_ADDR + (index * EEPROM_USER_PAIR_RECORD_SIZE);

	// 32�ֽڷ����ζ�ȡ��ÿ��16�ֽڣ�
	// ��һҳ��0-15�ֽ�
	int32_t ret = eeprom_i2c_read(base_addr, (uint8_t *)record, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// �ڶ�ҳ��16-31�ֽ�
	ret = eeprom_i2c_read(base_addr + E2P_PAGE_LEN, ((uint8_t *)record) + E2P_PAGE_LEN, E2P_PAGE_LEN);
	if (ret != LL_OK)
	{
		return -1;
	}

	// ��֤CRC8
	uint8_t calculated_crc = calculate_crc8((const uint8_t *)record, sizeof(eeprom_user_pair_record_t) - 1);
	if (calculated_crc != record->crc8)
	{
		return -2; // CRCУ��ʧ��
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_save_user_pair_device_by_sn(const char *device_sn)
 Input       : device_sn: �豸SN
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ���浥���û�����豸��EEPROM
---------------------------------------------------------------------------*/
int eeprom_save_user_pair_device_by_sn(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0 || strlen(device_sn) > SN_LENGTH)
	{
		return -1;
	}

	eeprom_user_pair_record_t record;
	int8_t target_slot = -1;

	// 1. �Ȳ����Ƿ��Ѵ��ڸ�SN
	for (uint8_t i = 0; i < EEPROM_USER_PAIR_MAX_NUM; i++)
	{
		if (sys_param.user_pair_list[i].is_valid &&
			strncmp(sys_param.user_pair_list[i].device_sn, device_sn, SN_LENGTH) == 0)
		{
			target_slot = i;
			break;
		}
	}

	// 2. ��������ڣ����ҿղ�λ
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

	// 3. ���û�пղ�λ������ʧ��
	if (target_slot < 0)
	{
		return -1;
	}

	// 4. ׼����¼��д��EEPROM
	memset(&record, 0, sizeof(record));

	// ����SN�����15�ֽ�
	strncpy(record.device_sn, device_sn, SN_LENGTH);
	// ����Ҫ����'\0'����Ϊֻ��15�ֽ�

	record.valid = EEPROM_RECORD_VALID;

	if (eeprom_write_user_pair_record(target_slot, &record) != 0)
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_clear_user_pair_list_device_by_sn(const char *device_sn)
 Input       : device_sn: �豸SN
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ��EEPROM��ɾ��ָ��SN���û�����豸
---------------------------------------------------------------------------*/
int eeprom_clear_user_pair_list_device_by_sn(const char *device_sn)
{
	if (device_sn == NULL || strlen(device_sn) == 0)
	{
		return -1;
	}

	// 1. ���ڴ��в��Ҹ�SN�Ĳ�λ
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

	// 2. ���δ�ҵ�������ʧ��
	if (target_slot < 0)
	{
		return -1;
	}

	// 3. ���EEPROM��¼
	eeprom_user_pair_record_t record;
	memset(&record, 0, sizeof(record));
	record.valid = EEPROM_RECORD_INVALID;

	if (eeprom_write_user_pair_record(target_slot, &record) != 0)
	{
		return -1;
	}

	// 4. ����ڴ�
	sys_param.user_pair_list[target_slot].is_valid = false;
	memset(sys_param.user_pair_list[target_slot].device_sn, 0, sizeof(sys_param.user_pair_list[target_slot].device_sn));

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_load_user_pair_list(void)
 Input       : ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ��EEPROM���������û�����б�����ʼ��ʱ���ã�
---------------------------------------------------------------------------*/
int eeprom_load_user_pair_list(void)
{
	eeprom_user_pair_record_t record;
	bool all_failed = true;

	// ����ڴ��е��û�����б�
	memset(sys_param.user_pair_list, 0, sizeof(sys_param.user_pair_list));

	for (uint8_t i = 0; i < USER_PAIR_LIST_MAX_NUM; i++)
	{
		// ��ȡEEPROM��¼
		if (eeprom_read_user_pair_record(i, &record) != 0)
		{
			// printf("Record %d read failed, skip.\n", i);
			sys_param.user_pair_list[i].is_valid = false; // �����ʧ��
			continue;
		}

		// ����¼��Ч��
		if (record.valid == EEPROM_RECORD_VALID)
		{
			strncpy(sys_param.user_pair_list[i].device_sn, record.device_sn, SN_LENGTH);
			sys_param.user_pair_list[i].device_sn[SN_LENGTH] = '\0'; // ���ӽ�����
			sys_param.user_pair_list[i].is_valid = true;
			all_failed = false;
		}
		else
		{
			sys_param.user_pair_list[i].is_valid = false;
		}
	}

	// ������м�¼��ʧ�ܣ��ŷ��ش���
	return all_failed ? -1 : 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_clear_user_pair_list(void)
 Input       : ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ���EEPROM�е������û�����б�
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

	// ����ڴ�
	memset(sys_param.user_pair_list, 0, sizeof(sys_param.user_pair_list));

	return 0;
}

// //------------------------------- �����ǲ���ʹ�õĺ�����Ϊ�˲���EEPROM  --------------------------------------------

/*---------------------------------------------------------------------------
	Name        : void print_device_list(void)
	Input       : ��
	Output      : ��
	Description : ��ӡ�豸�б�
---------------------------------------------------------------------------*/
void print_device_list(void)
{
	printf("\r\n2.΢���Ѿ�����б���ʾ\r\n");
	printf("  ���� | SIID | sub1g��ַ(3B) | EEPROM��ַ��Χ | �豸SN           | ��Ʒ�ͺ�  | ����CTx����\n");
	printf("  -----|------|---------------|----------------|------------------|-----------|------------\n");
	for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
	{
		if (sys_param.paired_inv_info[i].siid >= SIID_MIN && sys_param.paired_inv_info[i].siid <= SIID_MAX)
		{
			sys_param.sub1g.state = 2; // 2 - δ����������豸

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
	printf("\r\n3.�û�����б���ʾ\r\n");
	printf(" ���� | EEPROM��ַ��Χ | �豸SN  \n");
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
	// ��ȡCT��SN
	char sn_buffer[16];
	if (eeprom_read_sn(sn_buffer) == 0) // SN��ȡ�ɹ�
	{
		sys_param.flash_sn_com_normal = true;
		printf(" CT SN: %s", wifi_info.sn);
	}
	else // SN��ȡʧ��
	{
		sys_param.flash_sn_com_normal = false;
		memset(wifi_info.sn, 0, sizeof(wifi_info.sn));
		printf(" CT SN: Not Writen (use MAC address)");
	}
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_update_device_phase(uint32_t sub1g_addr, uint8_t phase)
 Input       : sub1g_addr - SUB1G��ַ
			   phase - ��λ (0=δʶ��, 1=A��, 2=B��, 3=C��)
 Output      : 0=�ɹ�, -1=ʧ��
 Description : �����豸����λ��Ϣ��EEPROM���ڴ�
---------------------------------------------------------------------------*/
int eeprom_update_device_phase(uint32_t sub1g_addr, uint8_t phase)
{
	if (phase > 3)
	{
		return -1;
	}

	// �����豸
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
		return -1; // �豸������
	}

	// �����ڴ�
	sys_param.paired_inv_info[slot].phase = phase;
	sys_param.paired_inv_info[slot].prop_changed = true;

	// ��ȡ����EEPROM��¼
	eeprom_device_record_t record;
	if (eeprom_read_device_record(slot, &record) != 0)
	{
		return -1;
	}

	// ����phase�ֶ�
	record.phase = phase;

	// д��EEPROM
	if (eeprom_write_device_record(slot, &record) != 0)
	{
		return -1;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_load_set_param(void)
 Input       : ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ��EEPROM��ȡ�õ�����Ϣ�������������ơ�����ģʽ���Ƿ��������
---------------------------------------------------------------------------*/
int eeprom_load_set_param(void)
{
	eeprom_consumption_record_t record;
	uint16_t reg_addr = EEPROM_CONSUMPTION_BASE_ADDR;
	int result = 0;

	// ��ȡEEPROM
	int32_t ret = eeprom_i2c_read(reg_addr, (uint8_t *)&record, sizeof(eeprom_consumption_record_t));

	if (ret == LL_OK)
	{
		// ��֤CRC
		uint8_t calc_crc = calculate_crc8((uint8_t *)&record, sizeof(eeprom_consumption_record_t) - 1);

		if (record.valid == EEPROM_RECORD_VALID && calc_crc == record.crc8)
		{
			// DEBUG_PRINTF("EEPROM: CRC success record.electricity_consumption = %d \r\n", record.electricity_consumption);

			// CRCУ��ͨ�����������ò���
			sys_param.to_grid_power_limit = record.to_grid_power_limit;
			sys_param.power_work_mode = record.power_work_mode;
			sys_param.anti_backflow_switch = record.antiflow_enable;

			// ֻ��sequence_k��1-6��Χ����tag==sequence_k*10+sequence_kʱ��ʹ�ô洢ֵ
			if (record.sequence_k >= 1 && record.sequence_k <= 6 &&
				record.sequence_k_tag == record.sequence_k * 10 + record.sequence_k)
			{
				DEBUG_PRINTF("EEPROM: sequence_k = %d, record.sequence_k_tag = %d, valid!\r\n", record.sequence_k, record.sequence_k_tag);
				// ����У��ͨ����ʹ�ô洢������ֵ
				sys_param.grid.phase_id.sequence_k = record.sequence_k;
				sys_param.grid.phase_id.identification_valid = 1;
				update_ct_to_phase_mapping(record.sequence_k);
			}
			else
			{
				DEBUG_PRINTF("EEPROM: sequence_k = %d, record.sequence_k_tag = %d, invalid, use default sequence_k = 1, Save EEP\r\n", record.sequence_k, record.sequence_k_tag);
				// У�鲻ͨ����sequence_k=0��Ĭ��sequence_k=1
				sys_param.grid.phase_id.sequence_k = 1;
				sys_param.grid.phase_id.identification_valid = 1;
				update_ct_to_phase_mapping(1);
				eeprom_save_set_param();
			}

			// ����CT���ʷ��򣨽�����1��-1������Ĭ��1��
			sys_param.ct1.power.power_direction = (record.ct1_dir == -1) ? -1.0f : 1.0f;
			sys_param.ct2.power.power_direction = (record.ct2_dir == -1) ? -1.0f : 1.0f;
			sys_param.ct3.power.power_direction = (record.ct3_dir == -1) ? -1.0f : 1.0f;

			return 0;
		}
		else
		{
			// CRCУ��ʧ�ܻ��¼��Ч
			result = -2;
		}
	}
	else
	{
		// I2C��ȡʧ��
		result = -1;
	}

	// ��ȡʧ�ܻ�CRC����ͳһ����Ĭ��ֵ��д��EEPROM
	sys_param.to_grid_power_limit = 0;
	sys_param.power_work_mode = 1;
	sys_param.anti_backflow_switch = 1;
	sys_param.grid.phase_id.sequence_k = 1;
	sys_param.grid.phase_id.identification_valid = 1;
	update_ct_to_phase_mapping(1);
	// CT���򱣳�system_param_init�����õ�Ĭ��ֵ1�������ظ���ֵ
	eeprom_save_set_param();
	DEBUG_PRINTF("EEPROM: Load failed! result = %d, set default and save eep \r\n", result);

	return result;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_save_set_param()
 Input       : consumption - �õ���(Wh)
 Output      : 0: �ɹ�, -1: ʧ��
 Description : �����õ�����EEPROM
---------------------------------------------------------------------------*/
int eeprom_save_set_param()
{
	eeprom_consumption_record_t record;
	uint16_t reg_addr = EEPROM_CONSUMPTION_BASE_ADDR;

	// ׼����¼
	memset(&record, 0, sizeof(record));
	record.electricity_consumption = 0; // ���������ٴ洢�õ���
	record.to_grid_power_limit = sys_param.to_grid_power_limit;
	record.power_work_mode = sys_param.power_work_mode;
	record.antiflow_enable = sys_param.anti_backflow_switch;
	record.sequence_k = sys_param.grid.phase_id.sequence_k;
	record.sequence_k_tag = record.sequence_k * 10 + record.sequence_k;
	record.ct1_dir = (sys_param.ct1.power.power_direction < 0.0f) ? (int8_t)(-1) : (int8_t)(1);
	record.ct2_dir = (sys_param.ct2.power.power_direction < 0.0f) ? (int8_t)(-1) : (int8_t)(1);
	record.ct3_dir = (sys_param.ct3.power.power_direction < 0.0f) ? (int8_t)(-1) : (int8_t)(1);
	record.valid = EEPROM_RECORD_VALID;

	// ����CRC
	record.crc8 = calculate_crc8((uint8_t *)&record, sizeof(eeprom_consumption_record_t) - 1);

	// д��EEPROM
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
 Input       : ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ���õ����������浽����EEPROMҳ��Ƶ��д������
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
 Input       : ��
 Output      : 0: �ɹ�, -1: ʧ��
 Description : �Ӷ���EEPROMҳ�����õ���
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

	// ��ȡʧ�ܻ�CRC����Ĭ��0
	sys_param.hmi.electricity_consumption = 0;
	return -1;
}

/*---------------------------------------------------------------------------
 Name        : int eeprom_migrate_elec_consumption(void)
 Input       : ��
 Output      : 0: ��Ǩ��, 1: ����Ǩ��(������Ϊ0), -1: �������ȡʧ��/CRC��Ч
 Description : OTA�������ݣ������¶���������Чʱ���á�
			   ����������electricity_consumption����0��Ǩ�Ƶ����������������
			   ������CRCУ��ʧ��ʱ���������ݲ����ţ���Ĭ��ֵ0��load���������á�
---------------------------------------------------------------------------*/
int eeprom_migrate_elec_consumption(void)
{
	eeprom_consumption_record_t old_record;
	uint16_t reg_addr = EEPROM_CONSUMPTION_BASE_ADDR;

	// ��������������16�ֽ�
	int32_t ret = eeprom_i2c_read(reg_addr, (uint8_t *)&old_record, sizeof(eeprom_consumption_record_t));
	if (ret != LL_OK)
	{
		DEBUG_PRINTF("[EEPROM] Migration: old area read failed, skip\r\n");
		return -1;
	}

	// CRCУ��
	uint8_t calc_crc = calculate_crc8((uint8_t *)&old_record, sizeof(eeprom_consumption_record_t) - 1);
	if (old_record.valid != EEPROM_RECORD_VALID || calc_crc != old_record.crc8)
	{
		DEBUG_PRINTF("[EEPROM] Migration: old area CRC invalid, skip\r\n");
		return -1;
	}

	// ���������electricity_consumption�Ƿ��0
	if (old_record.electricity_consumption == 0)
	{
		DEBUG_PRINTF("[EEPROM] Migration: old elec=0, no need to migrate\r\n");
		return 1;
	}

	// ����ֵд���¶�������
	DEBUG_PRINTF("[EEPROM] Migration: old elec=%u Wh, migrating...\r\n", old_record.electricity_consumption);
	sys_param.hmi.electricity_consumption = old_record.electricity_consumption;
	int save_ret = eeprom_save_elec_consumption();
	if (save_ret != 0)
	{
		DEBUG_PRINTF("[EEPROM] Migration: save to new area failed!\r\n");
		return -1;
	}

	// ������electricity_consumption��0����д
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
 Input       : sn - 15�ֽڵ�SN�ַ���
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ��CT��SNд��EEPROM
---------------------------------------------------------------------------*/
int eeprom_write_sn(const char *sn)
{
	// �������
	if (sn == NULL)
	{
		return -1;
	}

	// ���SN���ȣ�����Ϊ15�ֽڣ�
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
 Input       : sn - ���ڴ洢��ȡ��SN�Ļ�����������16�ֽڣ�
 Output      : 0: �ɹ�, -1: ʧ��
 Description : ��EEPROM��ȡCT��SN�����Զ����Ƶ�wifi_info
---------------------------------------------------------------------------*/
int eeprom_read_sn(char *sn)
{
	if (sn == NULL)
	{
		return -1;
	}

	// ��ȡEEPROM����
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

	// ����SN�������������15�ֽڣ�
	memcpy(sn, record.device_sn, SN_LENGTH);
	sn[SN_LENGTH] = '\0'; // �����ַ���������

	// ��SN���Ƶ�wifi_info
	memcpy(wifi_info.sn, sn, SN_LENGTH);
	wifi_info.sn[SN_LENGTH] = '\0';

	return 0;
}

/*eof*/

//
// Included Files
//
#include "board.h"
#include "main.h"
#include "arm_math.h"
#include "mmi.h"
#include "string.h"
#include "stdio.h"
#include "wifi.h"
#include "ota_max.h"

#define LED_GREEN_ON GPIO_SetPins(GPIO_PORT_F, GPIO_PIN_02)
#define LED_GREEN_OFF GPIO_ResetPins(GPIO_PORT_F, GPIO_PIN_02)
#define LED_RED_ON GPIO_SetPins(GPIO_PORT_C, GPIO_PIN_13)
#define LED_RED_OFF GPIO_ResetPins(GPIO_PORT_C, GPIO_PIN_13)

serial_msg_t msg_input;
serial_msg_t msg_output;

static void hmi_update_display(void);
static void bytes_to_serial(char *buffer, uint16_t len);

// 按键逻辑：长按3秒复位wifi和sub1g改为启动微逆相位识别
/* 闪灯逻辑：
-红绿交替快闪-微逆逆流报警
-红灯闪烁-有故障（慢闪）；CT不在线（快闪）
-红绿闪烁-初始化和相序识别或升级中或配网中
-红灯常亮-未联网和默认状态
-绿灯快闪-微逆相位识别进行中
-绿灯常亮-微逆识别成功
*/
/*---------------------------------------------------------------------------
 Name        : static led_state_t determine_led_state(void)
 Input       : 无
 Output      : LED状态
 Description : 根据系统状态确定LED显示状态
---------------------------------------------------------------------------*/
static led_state_t determine_led_state(void)
{

    // 微逆相关状态
    if (sys_param.fft_identify.is_ffting)
    {
        return LED_STATE_GREEN_BLINK_FAST; // 微逆识别进行中：绿灯快闪
    }

    // 故障状态
    if (sys_param.fault.data > 0)
    {
        return LED_STATE_RED_BLINK_SLOW; // 有故障：红灯慢闪
    }

    // 系统状态相关
    switch (sys_param.state)
    {
    case SYS_INIT:
    case SYS_WAIT_CT:
    case SYS_PHASE_IDENTIFY:
        return LED_STATE_ALTERNATING_SLOW; // 初始化或相序识别：红绿交替慢闪

    case SYS_NORMAL_RUN:
        // 检查CT在线状态，后期需要改为 &&，3个CT需要同时在线才显示绿灯
        if (sys_param.ct1.status.connect_status == CT_STATUS_ONLINE ||
            sys_param.ct2.status.connect_status == CT_STATUS_ONLINE ||
            sys_param.ct3.status.connect_status == CT_STATUS_ONLINE)
        {
            return LED_STATE_GREEN_ON; // CT在线：绿灯常亮
        }
        else
        {
            return LED_STATE_GREEN_BLINK_SLOW; // CT不在线：绿灯慢闪
        }

    default:
        return LED_STATE_RED_ON; // 默认状态：红灯常亮
    }
}

/*---------------------------------------------------------------------------
 Name        : static void led_control_output(led_state_t state, uint32_t timer, uint8_t phase)
 Input       :
 Output      :
 Description :
---------------------------------------------------------------------------*/
static void led_control_output(led_state_t state, uint32_t timer, uint8_t phase)
{
    switch (state)
    {
    case LED_STATE_OFF:
        LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
        LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        break;

    case LED_STATE_RED_ON:
        LED_RED_ON;    // GPIO_writePin(LED_RED_GPIO, 1);
        LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        break;

    case LED_STATE_GREEN_ON:
        LED_RED_OFF;  // GPIO_writePin(LED_RED_GPIO, 0);
        LED_GREEN_ON; // GPIO_writePin(LED_GREEN_GPIO, 1);
        break;

    case LED_STATE_RED_BLINK_SLOW:
        if (timer < (LED_BLINK_SLOW_PERIOD * LED_ON_RATIO))
        {
            LED_RED_ON;    // GPIO_writePin(LED_RED_GPIO, 1);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        else
        {
            LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        break;

    case LED_STATE_RED_BLINK_FAST:
        if (timer < (LED_BLINK_FAST_PERIOD * LED_ON_RATIO))
        {
            LED_RED_ON;    // GPIO_writePin(LED_RED_GPIO, 1);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        else
        {
            LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        break;

    case LED_STATE_GREEN_BLINK_SLOW:
        if (timer < (LED_BLINK_SLOW_PERIOD * LED_ON_RATIO))
        {
            LED_RED_OFF;  // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_ON; // GPIO_writePin(LED_GREEN_GPIO, 1);
        }
        else
        {
            LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        break;

    case LED_STATE_GREEN_BLINK_FAST:
        if (timer < (LED_BLINK_FAST_PERIOD * LED_ON_RATIO))
        {
            LED_RED_OFF;  // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_ON; // GPIO_writePin(LED_GREEN_GPIO, 1);
        }
        else
        {
            LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        break;

    case LED_STATE_ALTERNATING_SLOW:
        if (timer < (LED_BLINK_ALTERNATING_PERIOD * LED_ON_RATIO))
        {
            if (phase == 0)
            {
                LED_RED_ON;    // GPIO_writePin(LED_RED_GPIO, 1);
                LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
            }
            else
            {
                LED_RED_OFF;  // GPIO_writePin(LED_RED_GPIO, 0);
                LED_GREEN_ON; // GPIO_writePin(LED_GREEN_GPIO, 1);
            }
        }
        else
        {
            LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        break;

    case LED_STATE_ALTERNATING_FAST:
        if (timer < (LED_BLINK_FAST_PERIOD * LED_ON_RATIO))
        {
            if (phase == 0)
            {
                LED_RED_ON;    // GPIO_writePin(LED_RED_GPIO, 1);
                LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
            }
            else
            {
                LED_RED_OFF;  // GPIO_writePin(LED_RED_GPIO, 0);
                LED_GREEN_ON; // GPIO_writePin(LED_GREEN_GPIO, 1);
            }
        }
        else
        {
            LED_RED_OFF;   // GPIO_writePin(LED_RED_GPIO, 0);
            LED_GREEN_OFF; // GPIO_writePin(LED_GREEN_GPIO, 0);
        }
        break;
    }
}

/*---------------------------------------------------------------------------
 Name        : void led_state_machine_update(mmi_t *mmi)
 Input       :
 Output      :
 Description : LED状态机更新函数
---------------------------------------------------------------------------*/
void led_state_machine_update(mmi_t *mmi)
{
    // 确定目标LED状态
    led_state_t target_state = determine_led_state();

    // 状态改变时重置定时器和相位
    if (target_state != mmi->led_state)
    {
        mmi->led_state = target_state;
        mmi->led_timer = 0;
        mmi->led_phase = 0;
    }

    // 更新定时器
    mmi->led_timer = mmi->led_count;

    // 根据状态更新定时器周期和相位
    uint32_t period = 0;
    switch (mmi->led_state)
    {
    case LED_STATE_RED_BLINK_SLOW:
    case LED_STATE_GREEN_BLINK_SLOW:
        period = LED_BLINK_SLOW_PERIOD;
        break;

    case LED_STATE_RED_BLINK_FAST:
    case LED_STATE_GREEN_BLINK_FAST:
    case LED_STATE_ALTERNATING_FAST:
        period = LED_BLINK_FAST_PERIOD;
        break;

    case LED_STATE_ALTERNATING_SLOW:
        period = LED_BLINK_ALTERNATING_PERIOD;
        break;

    default:
        period = 1000; // 默认周期
        break;
    }

    // 计算当前周期内的定时器值
    uint32_t cycle_timer = mmi->led_timer % period;

    // 交替闪烁需要更新相位
    if (mmi->led_state == LED_STATE_ALTERNATING_SLOW || mmi->led_state == LED_STATE_ALTERNATING_FAST)
    {
        // 每个周期切换相位
        if (cycle_timer == 0 && mmi->led_timer > 0)
        {
            mmi->led_phase = (mmi->led_phase == 0) ? 1 : 0;
        }
    }

    // 控制LED输出
    led_control_output(mmi->led_state, cycle_timer, mmi->led_phase);
}

/*---------------------------------------------------------------------------
 Name        : static bool parse_ota_start_data(...)
 Input       : data - 二进制数据指针
               data_len - 数据长度
 Output      : true-解析成功, false-解析失败
 Description : 解析OTA开始命令的二进制数据,直接赋值给g_ota_manager
               数据格式: fw_type(1B) + length(4B大端) + version(字符串) + crc(4B大端)
               串口示例: 4 110000(0x0001ADB0) V1.0.0 12345(0x00003039)
---------------------------------------------------------------------------*/
static bool parse_ota_start_data(uint8_t *data, uint16_t data_len)
{
    // 最小长度: fw_type(1) + length(4) + crc(4) = 9字节 + 至少1字节版本号
    if (data_len < 10)
    {
        DEBUG_PRINTF("[MMI] OTA parse: data too short, len=%d (need ≥10)\r\n", data_len);
        return false;
    }

    uint16_t index = 0;

    // fw_type (1字节)
    uint8_t fw_type = data[index++];

    // fw_length (4字节,大端序)
    uint32_t fw_length = ((uint32_t)data[index] << 24) |
                         ((uint32_t)data[index + 1] << 16) |
                         ((uint32_t)data[index + 2] << 8) |
                         ((uint32_t)data[index + 3]);
    index += 4;

    // fw_version (字符串,剩余长度 - 4字节CRC)
    uint16_t version_len = data_len - index - 4;
    if (version_len == 0 || version_len > (OTA_VERSION_MAX_LEN - 1))
    {
        DEBUG_PRINTF("[MMI] OTA parse: invalid version_len=%d\r\n", version_len);
        return false;
    }

    // fw_crc (4字节,大端序)
    uint32_t fw_crc = ((uint32_t)data[index + version_len] << 24) |
                      ((uint32_t)data[index + version_len + 1] << 16) |
                      ((uint32_t)data[index + version_len + 2] << 8) |
                      ((uint32_t)data[index + version_len + 3]);

    // 直接赋值给g_ota_manager
    g_ota_manager.fw_type = (ota_fw_type_t)fw_type;
    g_ota_manager.fw_length = fw_length;
    g_ota_manager.fw_crc = fw_crc;

    memcpy(g_ota_manager.fw_version, &data[index], version_len);
    g_ota_manager.fw_version[version_len] = '\0';

    DEBUG_PRINTF("\r\n[MMI] OTA parse success: fw_type=%d, fw_length=%u, fw_version=%s, fw_crc=0x%08X\r\n", g_ota_manager.fw_type, g_ota_manager.fw_length, g_ota_manager.fw_version, g_ota_manager.fw_crc);

    return true;
}

/*---------------------------------------------------------------------------
 Name        : void mmi_task(void)
 Input       : 无
 Output      : 无
 Description : 人机交互主任务函数
---------------------------------------------------------------------------*/
void mmi_task(void)
{
    static uint32_t led_count = 0;
    if (led_count != sys_param.mmi.led_count)
    {
        led_count = sys_param.mmi.led_count;
        // 更新LED状态机
        led_state_machine_update(&sys_param.mmi);
    }

    if (sys_param.mmi.display_receive_flag == 1)
    {
    }
    if (sys_param.mmi.display_timer_ms >= 1000)
    {
        sys_param.mmi.display_timer_ms = 0;
        hmi_update_display();
    }
    if (msg_input.flag == 1)
    {
        msg_input.flag = 0;

        // if (msg_input.cmd != 0x0001)
        // {
        //     // 打印接收的数据
        //     DEBUG_PRINTF("[MMI] Received cmd: 0x%04X, type=%d, code=%d, length=%d ", msg_input.cmd, msg_input.type, msg_input.code, msg_input.cmd_data_length);
        //     if (msg_input.type == 0)
        //     {
        //         for (int i = 0; i < msg_input.cmd_data_length; i++)
        //         {
        //             DEBUG_PRINTF("0x%02X ", msg_input.cmd_data[i]);
        //         }
        //     }
        //     DEBUG_PRINTF("\r\n");
        // }

        if (msg_input.cmd == 0x0001 && msg_input.type == 1 && msg_input.code == 0)
        {
            if (msg_input.cmd_data[0] == 0x01)
                sys_param.wifi.restore_wifi_cmd = 1;
        }
        // CMD=1000: 升级开始
        // iot: CMD=1000,TYPE=0,code=0,DATA={fw_type:4,length:110000,version:"V1.0.0",crc:0x12345678}
        // mcu: CMD=1000,TYPE=1,code=0,DATA={result:0/1/2}
        else if (msg_input.cmd == 0x1000 && msg_input.type == 0 && msg_input.code == 0)
        {
            ota_start_result_t result;

            // 如果OTA正在进行中，直接返回BUSY
            if (g_ota_manager.ota_in_progress)
            {
                result = OTA_START_BUSY;
                DEBUG_PRINTF("[MMI] OTA already in progress, ignore duplicate start command\r\n");
            }
            else
            {
                // 解析数据并直接填充到g_ota_manager
                if (parse_ota_start_data(msg_input.cmd_data, msg_input.cmd_data_length))
                {
                    // 检查OTA启动状态
                    result = ota_check_start_status();
                    DEBUG_PRINTF("[MMI] OTA start response: result=%d\r\n", result);
                }
                else
                {
                    // 解析失败
                    result = OTA_START_ERROR;
                    DEBUG_PRINTF("[MMI] OTA parse failed, send error response\r\n");
                }
            }

            // 构建发送回复消息
            msg_output.type = 0x01;
            msg_output.cmd = 0x1000;
            msg_output.code = 0;
            msg_output.cmd_data[0] = (uint8_t)result;
            msg_output.cmd_data_length = 1;
            serial_msg_send(&msg_output);
            DEBUG_PRINTF("[MMI] OTA start send hmi response: type=%d cmd=0x%06X code=%d result=%d\r\n", msg_output.type, msg_output.cmd, msg_output.code, result);
        }

        // CMD=1001: 固件数据（WiFi回复MCU请求）
        // mcu: CMD=1001,TYPE=0,code=0,DATA={ADDRESS:0x000000,LENGTH:128}
        // iot: CMD=1001,TYPE=1,code=0,DATA={DATA:N*BYTE}
        else if (msg_input.cmd == 0x1001 && msg_input.type == 1)
        {
            // 固件数据直接拷贝
            ota_copy_wifi_fw_data(msg_input.cmd_data, msg_input.cmd_data_length);
        }

        // CMD=1002: 查询完成状态
        // mcu: CMD=1002,TYPE=0,code=0,DATA={}
        // iot: CMD=1002,TYPE=1,code=0
        else if (msg_input.cmd == 0x1002 && msg_input.type == 1)
        {
            // 处理wifi返回接收OTA执行结果
        }
    }
}

/*---------------------------------------------------------------------------
 Name        : hmi_update_inv_states
 Description : 更新HMI显示的逆变器状态
---------------------------------------------------------------------------*/
static void hmi_update_inv_states(void)
{
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        // 如果槽位无效,清空数据
        if (!sys_param.paired_inv_info[i].is_valid || sys_param.paired_inv_info[i].siid == 0)
        {
            sys_param.hmi.inv[i].state = 0;
            memset(sys_param.hmi.inv[i].short_sn, 0, sizeof(sys_param.hmi.inv[i].short_sn));
            sys_param.hmi.inv[i].comm_rssi = 0;
            sys_param.hmi.inv[i].comm_plr = 0;
            sys_param.hmi.inv[i].power_en = 0;
            sys_param.hmi.inv[i].power_limit = 0;
            sys_param.hmi.inv[i].power_mode = 0;
            sys_param.hmi.inv[i].power_output = 0;
            continue;
        }

        // 1. 在线状态: 直接使用online_state
        sys_param.hmi.inv[i].state = sys_param.paired_inv_info[i].online_state;

        // 2. SN后6位
        uint8_t sn_len = strlen(sys_param.paired_inv_info[i].device_sn);
        if (sn_len >= 6)
        {
            memcpy(sys_param.hmi.inv[i].short_sn, &sys_param.paired_inv_info[i].device_sn[sn_len - 6], 6);
            sys_param.hmi.inv[i].short_sn[6] = '\0';
        }
        else
        {
            // SN长度不足6位，则全部复制
            strncpy(sys_param.hmi.inv[i].short_sn, sys_param.paired_inv_info[i].device_sn, 6);
            sys_param.hmi.inv[i].short_sn[sn_len] = '\0';
            sys_param.hmi.inv[i].short_sn[6] = '\0'; // 确保终止符在数组范围内
        }

        // 3. 工作模式判断
        if (sys_param.paired_inv_info[i].antiflow_enable == 1)
        {
            sys_param.hmi.inv[i].power_mode = 0; // 防逆流发电
        }
        else
        {
            if (sys_param.paired_inv_info[i].grid_power < (float)sys_param.paired_inv_info[i].power_limit)
            {
                sys_param.hmi.inv[i].power_mode = 1; // 自由发电
            }
            else
            {
                sys_param.hmi.inv[i].power_mode = 2; // 限流发电
            }
        }

        // 4. 通信信号强度
        sys_param.hmi.inv[i].comm_rssi = sys_param.paired_inv_info[i].inv_rssi;

        // 5. 通信丢包率: 使用2分钟统计的丢包率（0-100）
        sys_param.hmi.inv[i].comm_plr = sys_param.paired_inv_info[i].plr;

        // 6. 功率开关: 直接使用power_enable
        sys_param.hmi.inv[i].power_en = sys_param.paired_inv_info[i].power_enable;

        // 7. 功率限制: 直接使用power_limit 需要 uint32转uint16 (0-2500W)
        sys_param.hmi.inv[i].power_limit = (uint16_t)sys_param.paired_inv_info[i].power_limit;

        // 8. 输出功率: float转uint16
        sys_param.hmi.inv[i].power_output = (uint16_t)sys_param.paired_inv_info[i].grid_power;
    }
}

/*---------------------------------------------------------------------------
 Name        : hmi_update_power_and_energy
 Description : 更新功率和发电量信息
---------------------------------------------------------------------------*/
static void hmi_update_power_and_energy(void)
{
    // 更新三相功率: float转int16,小于10W显示为0
    if (fabsf(sys_param.ct1.power.fix_dir_power) < 10.0f)
    {
        sys_param.hmi.power_l1 = 0;
    }
    else
    {
        sys_param.hmi.power_l1 = (int16_t)sys_param.ct1.power.fix_dir_power;
    }

    if (fabsf(sys_param.ct2.power.fix_dir_power) < 10.0f)
    {
        sys_param.hmi.power_l2 = 0;
    }
    else
    {
        sys_param.hmi.power_l2 = (int16_t)sys_param.ct2.power.fix_dir_power;
    }

    if (fabsf(sys_param.ct3.power.fix_dir_power) < 10.0f)
    {
        sys_param.hmi.power_l3 = 0;
    }
    else
    {
        sys_param.hmi.power_l3 = (int16_t)sys_param.ct3.power.fix_dir_power;
    }

    // 计算所有已配对设备的累计发电量总和
    float total_energy = 0.0f;
    float today_energy = 0.0f;
    float today_energy_time = 0.0f;
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].siid != 0)
        {
            total_energy += sys_param.paired_inv_info[i].lifetime_energy;
            today_energy += sys_param.paired_inv_info[i].today_energy;
            if (sys_param.paired_inv_info[i].today_power_time > today_energy_time)
            {
                today_energy_time = sys_param.paired_inv_info[i].today_power_time;
            }
        }
    }

    // 最大的today_power_time更新sys_param.ct_today_power_time
    sys_param.ct_today_power_time = today_energy_time;

    // 转换为uint32 (Wh)
    if (total_energy < 0.0f)
    {
        sys_param.hmi.electricity_generation = 0;
    }
    else
    {
        // 更新统计所有设备的今日发电量，累计发电量
        sys_param.hmi.electricity_generation = (uint32_t)total_energy;
        sys_param.ct_today_energy = today_energy;
    }
}

/*---------------------------------------------------------------------------
 Name        : hmi_update_subg_status
 Description : 更新SUB1G状态信息
---------------------------------------------------------------------------*/
static void hmi_update_subg_status(void)
{
    uint8_t online_count = 0;
    uint8_t offline_count = 0;
    uint8_t discovered_count = 0;

    // 统计已配对设备的在线/离线数量
    for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.paired_inv_info[i].is_valid && sys_param.paired_inv_info[i].siid != 0)
        {
            if (sys_param.paired_inv_info[i].online_state == 2)
            {
                online_count++;
            }
            else if (sys_param.paired_inv_info[i].online_state == 1)
            {
                offline_count++;
            }
        }
    }

    // 统计请求配对的设备数量
    for (uint8_t i = 0; i < UNPAIRED_DEVICE_MAX_NUM; i++)
    {
        if (sys_param.inv_request_pair_list[i].is_valid)
        {
            discovered_count++;
        }
    }

    // 更新sub1g连接/断开设备数量
    sys_param.hmi.subg_connected = online_count;
    sys_param.hmi.subg_disconnected = offline_count;
    sys_param.hmi.subg_discovered = discovered_count;

    // 更新sub1g状态
    // 0-无配对设备，1-有配对设备但都不在线，2-至少有一个配对设备在线
    if (online_count > 0)
    {
        sys_param.hmi.subg_state = 2; // 有设备在线
    }
    else if (offline_count > 0)
    {
        sys_param.hmi.subg_state = 1; // 有设备但都离线
    }
    else
    {
        sys_param.hmi.subg_state = 0; // 无配对设备
    }

    sys_param.hmi.subg_antibackflow = sys_param.anti_backflow_switch;
}

/*---------------------------------------------------------------------------
 Name        : hmi_update_wifi_cmd
 Description : 更新WiFi命令状态
---------------------------------------------------------------------------*/
static void hmi_update_wifi_cmd(void)
{
    // 检查WiFi复位命令
    if (sys_param.wifi.restore_wifi_cmd == 1)
    {
    }
    else
    {
    }
}

/*---------------------------------------------------------------------------
 Name        : hmi_update_all_params
 Description : 更新所有HMI参数(主函数,每1秒调用一次)
---------------------------------------------------------------------------*/
void hmi_update_all_params(void)
{
    if (g_ota_manager.disable_property_report)
    {
        return;
    }

    // 更新功率和发电量
    hmi_update_power_and_energy();

    // 更新SUB1G状态
    hmi_update_subg_status();

    // 更新8个微逆的状态
    hmi_update_inv_states();

    // 更新WiFi命令
    hmi_update_wifi_cmd();
}

void hmi_update_display(void) // 更新HMI显示内容
{
    uint16_t index = 0;
    uint16_t temp = 0;
    uint16_t i = 0, j = 0;

    msg_output.type = 0;
    msg_output.cmd = 0x0001;
    msg_output.code = 0;
    // 固定数据19字节
    msg_output.cmd_data[index++] = (sys_param.hmi.electricity_generation >> 24) & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.electricity_generation >> 16) & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.electricity_generation >> 8) & 0xff;
    msg_output.cmd_data[index++] = sys_param.hmi.electricity_generation & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.electricity_consumption >> 24) & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.electricity_consumption >> 16) & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.electricity_consumption >> 8) & 0xff;
    msg_output.cmd_data[index++] = sys_param.hmi.electricity_consumption & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.power_l1 >> 8) & 0xff;
    msg_output.cmd_data[index++] = sys_param.hmi.power_l1 & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.power_l2 >> 8) & 0xff;
    msg_output.cmd_data[index++] = sys_param.hmi.power_l2 & 0xff;
    msg_output.cmd_data[index++] = (sys_param.hmi.power_l3 >> 8) & 0xff;
    msg_output.cmd_data[index++] = sys_param.hmi.power_l3 & 0xff;
    msg_output.cmd_data[index++] = sys_param.hmi.subg_state;
    msg_output.cmd_data[index++] = sys_param.hmi.subg_connected;
    msg_output.cmd_data[index++] = sys_param.hmi.subg_disconnected;
    msg_output.cmd_data[index++] = sys_param.hmi.subg_discovered;
    msg_output.cmd_data[index++] = sys_param.hmi.subg_antibackflow;
    // 8个微逆 8*(15+7*4)字节 = 8*43 =344字节
    for (i = 0; i < INV_DEVICE_MAX_NUM; i++)
    {
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].state;
        memcpy(&msg_output.cmd_data[index], sys_param.hmi.inv[i].short_sn, 6);
        index += 6;
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].comm_rssi;
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].comm_plr;
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].power_en;
        msg_output.cmd_data[index++] = (sys_param.hmi.inv[i].power_limit >> 8) & 0xff;
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].power_limit & 0xff;
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].power_mode;
        msg_output.cmd_data[index++] = (sys_param.hmi.inv[i].power_output >> 8) & 0xff;
        msg_output.cmd_data[index++] = sys_param.hmi.inv[i].power_output & 0xff;
        for (j = 0; j < 4; j++)
        {
            msg_output.cmd_data[index++] = sys_param.paired_inv_info[i].pv[j].state;
            temp = sys_param.paired_inv_info[i].pv[j].power * 10;
            msg_output.cmd_data[index++] = (temp >> 8) & 0xff;
            msg_output.cmd_data[index++] = temp & 0xff;
            temp = sys_param.paired_inv_info[i].pv[j].voltage * 10;
            msg_output.cmd_data[index++] = (temp >> 8) & 0xff;
            msg_output.cmd_data[index++] = temp & 0xff;
            temp = sys_param.paired_inv_info[i].pv[j].current * 10;
            msg_output.cmd_data[index++] = (temp >> 8) & 0xff;
            msg_output.cmd_data[index++] = temp & 0xff;
        }
    }
    msg_output.cmd_data_length = index;
    serial_msg_send(&msg_output);
}

void serial_msg_send(serial_msg_t *msg_out)
{
    uint8_t buffer[MSG_DATA_MAX_LEN + 9];
    uint16_t index = 0;
    uint8_t checksum = 0;
    uint16_t temp = 0;
    buffer[index++] = 0xFA;
    buffer[index++] = 0xCE;
    buffer[index++] = 0;
    buffer[index++] = 0;
    if (msg_out->type == 1)
        temp = msg_out->cmd | 0x8000;
    else
        temp = msg_out->cmd;
    buffer[index++] = (temp >> 8) & 0xFF;
    buffer[index++] = temp & 0xFF;
    if (msg_out->code != 0)
    {
        buffer[index++] = (msg_out->code >> 8) & 0xFF;
        buffer[index++] = msg_out->code & 0xFF;
    }
    else
    {
        buffer[index++] = (msg_out->cmd_data_length >> 8) & 0xFF;
        buffer[index++] = msg_out->cmd_data_length & 0xFF;
    }
    if (msg_out->code == 0)
    {
        for (uint16_t i = 0; i < msg_out->cmd_data_length; i++)
        {
            buffer[index++] = msg_out->cmd_data[i];
        }
    }
    temp = index - 4;
    buffer[2] = (temp >> 8) & 0xFF;
    buffer[3] = temp & 0xFF;
    for (uint16_t i = 0; i < index; i++)
    {
        checksum += buffer[i];
    }
    buffer[index++] = checksum;
    bytes_to_serial((char *)buffer, index);
}

void serial_msg_parse(uint8_t byte, serial_msg_t *msg_in)
{
    static uint8_t index = 0;
    static uint8_t checksum = 0;
    static uint16_t packet_index = 0;
    static uint16_t packet_length = 0;
    static uint8_t type = 0;
    static uint16_t cmd = 0;
    static uint16_t code;                      // 0x8000-0x8010
    static uint16_t cmd_data_length;           // 命令长度
    static uint16_t cmd_data_index;            //
    static uint8_t cmd_data[MSG_DATA_MAX_LEN]; // 命令数据
    switch (index)
    {
    case 0:
        if (byte == 0xFA)
            index = 1;
        checksum = byte;
        break;

    case 1:
        if (byte == 0xCE)
            index = 2;
        checksum += byte;
        break;

    case 2:
        packet_length = 0;
        packet_length |= (uint16_t)byte << 8;
        index = 3;
        checksum += byte;
        break;

    case 3:
        packet_length |= (uint16_t)byte;
        if (packet_length <= MSG_DATA_MAX_LEN + 4)
            index = 4;
        else
            index = 0;
        checksum += byte;
        packet_index = 0;
        break;

    case 4:
        cmd = 0;
        cmd |= (uint16_t)byte << 8;
        index = 5;
        packet_index++;
        checksum += byte;
        break;

    case 5:
        cmd |= (uint16_t)byte;
        if (cmd & 0x8000)
            type = 1;
        else
            type = 0;
        index = 6;
        packet_index++;
        checksum += byte;
        break;

    case 6:
        cmd_data_length = 0;
        cmd_data_length |= (uint16_t)byte << 8;
        index = 7;
        packet_index++;
        checksum += byte;
        break;

    case 7:
        cmd_data_length |= (uint16_t)byte;
        if ((cmd_data_length & 0x7FFF) <= MSG_DATA_MAX_LEN) // 数据长度
            index = 8;
        else
            index = 0;
        if (cmd_data_length & 0x8000)
            code = cmd_data_length;
        else
            code = 0;
        if (cmd_data_length == 0)
            index = 9;
        cmd_data_index = 0;
        packet_index++;
        checksum += byte;
        break;

    case 8:
        cmd_data[cmd_data_index++] = byte;
        packet_index++;
        if (cmd_data_index >= cmd_data_length || packet_index >= packet_length)
        {
            index = 9;
        }
        checksum += byte;
        break;

    case 9:
        if (checksum == byte)
        {
            if (cmd_data_index == cmd_data_length && packet_index == packet_length)
            {
                msg_in->type = type;
                msg_in->cmd = cmd & 0x7FFF;
                msg_in->code = code;
                msg_in->cmd_data_length = cmd_data_length;
                if (cmd_data_length > 0)
                    memcpy(msg_in->cmd_data, cmd_data, cmd_data_length);
                msg_in->flag = 1;
            }
        }
        index = 0;
        break;

    default:
        index = 0;
        break;
    }
}

static void bytes_to_serial(char *buffer, uint16_t len)
{
    uint16_t i = 0;
    for (i = 0; i < len; i++)
    {
        /* Wait Tx data register empty */
        USART_WriteData(CM_USART4, buffer[i]);
        while (RESET == USART_GetStatus(CM_USART4, USART_FLAG_TX_EMPTY))
        {
        }
    }
}

void USART4_Handler(void)
{
    uint8_t received_byte;

    if (USART_GetStatus(CM_USART4, USART_FLAG_RX_FULL))
    {
        received_byte = USART_ReadData(CM_USART4);
        serial_msg_parse(received_byte, &msg_input);
    }

    USART_ClearStatus(CM_USART4, USART_FLAG_ALL);
    __DSB(); /* Arm Errata 838869 */
}

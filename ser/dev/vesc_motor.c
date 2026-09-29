#include "vesc_motor.h"

const int pairs_of_poles=14;//极对数

void VESC_Set_Rpm_Struct(VESC_Motor* vesc_motor,int32_t RPM)//
{
	uint8_t data[4]={0};
    uint32_t id=(vesc_motor->Basic_Parameters.ID|((uint32_t)CAN_PACKET_SET_RPM<<8));
	//RPM
	//Int 32
	RPM*=pairs_of_poles;//RPM转换为ERPM
	data[0]=(RPM>>24)&(0xFF);
	data[1]=(RPM>>16)&(0xFF);
	data[2]=(RPM>>8)&(0xFF);
	data[3]=(RPM)&(0xFF);
	/*
	can通信中,采用大端序,即高字节在前
	*/
	vesc_can_send(vesc_motor->Basic_Parameters.bus, id, data, 4);
}

void VESC_Set_Current_Struct(VESC_Motor* vesc_motor,int32_t current)//
{
	uint8_t data[4]={0};
    uint32_t id=(vesc_motor->Basic_Parameters.ID|((uint32_t)CAN_PACKET_SET_CURRENT<<8));
	//current
	//Int 32
	// 数据封装：电流值转换为大端序（高字节在前）
	data[0]=(current>>24)&(0xFF);		// 第0字节：电流值第24~31位
	data[1]=(current>>16)&(0xFF);		// 第1字节：电流值第16~23位
	data[2]=(current>>8)&(0xFF);		// 第2字节：电流值第8~15位
	data[3]=(current)&(0xFF);				// 第3字节：电流值第0~7位
	/*
	can通信中,采用大端序,即高字节在前
	*/
	vesc_can_send(vesc_motor->Basic_Parameters.bus, id, data, 4);
}

void VESC_Set_Pos_Struct(VESC_Motor* vesc_motor,int32_t pos)//
{
	uint8_t data[4]={0};
    uint32_t id=(vesc_motor->Basic_Parameters.ID|((uint32_t)CAN_PACKET_SET_POS<<8));
	//pos
	//Int 32

	data[0]=((int32_t)pos>>24)&(0xFF);
	data[1]=((int32_t)pos>>16)&(0xFF);
	data[2]=((int32_t)pos>>8)&(0xFF);
	data[3]=((int32_t)pos)&(0xFF);
	/*
	can通信中,采用大端序,即高字节在前
	*/
	vesc_can_send(vesc_motor->Basic_Parameters.bus, id, data, 4);
}

void VESC_Motor_Struct_Init(VESC_Motor* vesc_motor,bsp_can_bus_t bus,uint16_t id)
{
	// 初始化基础参数：绑定CAN句柄和电机ID
	vesc_motor->Basic_Parameters.bus = bus;
	vesc_motor->Basic_Parameters.ID=id;

	// 初始化控制函数指针（关联具体实现函数）
	vesc_motor->SetCurrent=VESC_Set_Current_Struct;			//设置电流
	vesc_motor->SetRpm=VESC_Set_Rpm_Struct;				    //设置转速
	vesc_motor->SetPos=VESC_Set_Pos_Struct;				    //设置位置
}


uint32_t VESC_Motor_RxDispatch(VESC_Motor *motors, uint8_t count,
                        uint32_t ext_id, const uint8_t *data, uint8_t len){
    uint8_t ctrl_id;
    uint8_t i;

    if ((motors == 0) || (data == 0) || (len < 8u)) { return SER_ERR_PARAM; }

    ctrl_id = (uint8_t)(ext_id & 0xFFu);

    for (i = 0u; i < count; i++) {
        if ((uint8_t)motors[i].Basic_Parameters.ID == ctrl_id) {
            return VESC_Motor_OnCanRx(&motors[i], ext_id, data, len);
        }
    }
    return SER_ERR_NOENT;   /* 没有匹配的电调 ID */


}


int32_t VESC_Motor_OnCanRx(VESC_Motor *m, uint32_t ext_id,
                           const uint8_t *data, uint8_t len)
{
    uint8_t ctrl_id;
    uint8_t packet;

    if ((m == 0) || (data == 0) || (len < 8u)) { return SER_ERR_PARAM; }

    ctrl_id = (uint8_t)(ext_id & 0xFFu);          /* bit7~0  ：电调 ID  */
    packet  = (uint8_t)((ext_id >> 8) & 0xFFu);   /* bit15~8 ：指令枚举 */

    if (ctrl_id != (uint8_t)m->Basic_Parameters.ID) { return SER_ERR_NOENT; }

    switch (packet)
    {
        case CAN_PACKET_STATUS:          /* 转速 / 电流 / 占空比 */
        {
            /* ERPM 是 32 位大端【有符号】数：整体拼成 int32，符号自动正确。
             * 比原来"高两字节都是 0xFF 才按负数处理"的写法更稳：
             * 原写法在 ERPM = -1 时会算成 0，幅值超过 ±65535 也会出错。 */
            int32_t erpm_raw = (int32_t)(((uint32_t)data[0] << 24) |
                                         ((uint32_t)data[1] << 16) |
                                         ((uint32_t)data[2] <<  8) |
                                          (uint32_t)data[3]);

            m->RxData.ERpm = (float)erpm_raw;
            m->RxData.Rpm  = m->RxData.ERpm / (float)pairs_of_poles;   /* 极对数换算 */

            /* 电流、占空比同为 16 位大端有符号 */
            m->RxData.Total_Current =
                (float)(int16_t)(((uint16_t)data[4] << 8) | data[5]) / 10.0f;    /* ÷10   → A     */
            m->RxData.Duty =
                (float)(int16_t)(((uint16_t)data[6] << 8) | data[7]) / 1000.0f;  /* ÷1000 → -1~1  */
            break;
        }

        case CAN_PACKET_STATUS_2:        /* 安时消耗 / 回收：暂不解析 */
        {
            break;
        }

        case CAN_PACKET_STATUS_3:        /* 瓦时消耗 / 回收：暂不解析 */
        {
            break;
        }

        case CAN_PACKET_STATUS_4:        /* PID 位置（温度 / 输入电流暂不解析） */
        {
            /* 注意这里是【大端】，与 VESC 其他字段一致。
             * 你原代码写的是 (data[7])|(data[6]<<8)，字节序是反的。
             * 若实测原写法才对，说明固件版本不同，改回去即可。 */
            m->RxData.PID_Position_Last = m->RxData.PID_Position_Now;
            m->RxData.PID_Position_Now  =
                (float)(int16_t)(((uint16_t)data[6] << 8) | data[7]) / 50.0f;

            /* 处理 0/360 跨零，累计圈数（注意：Cnt 在这里是"圈数"） */
            if ((m->RxData.PID_Position_Now - m->RxData.PID_Position_Last) < -180.0f) {
                m->RxData.Cnt++;                    /* 正向跨零，如 350°→10° */
            } else if ((m->RxData.PID_Position_Now - m->RxData.PID_Position_Last) > 180.0f) {
                m->RxData.Cnt--;                    /* 反向跨零，如 10°→350° */
            }

            /* 累计总角度 = 圈数×360° + 当前角度 */
            m->RxData.Total_Position =
                (float)m->RxData.Cnt * 360.0f + m->RxData.PID_Position_Now;
            break;
        }

        case CAN_PACKET_STATUS_5:        /* 电压 / 里程计：暂不解析 */
        {
            break;
        }

        default:
        {
            /* 不是反馈报文（也可能是本机刚发出的指令被回环收到），忽略 */
            return SER_ERR_NOENT;
        }
    }

    return SER_OK;
}
/*
 * main.c
 */
#include <includes.h>

void CANTx_Tsk(void);
void CANRx_Tsk(void);

int main(void)
{
	static uint16_t cnt = 0;
	float state;

	APF_Init();
	BSP_Init();
	ExtDA_Init();

	for (;;)
	{
		if (MainTskTrigger)
		{

			MainTskTrigger = 0;		//清触发标志
			cnt++;

			//关断接触器合闸条件是否满足：直流母线平均电压大于交流侧整流电压
			state = CheckPwrState(Upcc_ab, Upcc_bc, Upcc_ca, Udc_average);
			if (state)
			{
				GPIO_WritePin(GPIO_PWRON, 1);
			}
			// LED闪烁表示程序正常运行
			if (cnt > 2500)
				GPIO_WritePin(GPIO_LED33,1);
			else
				GPIO_WritePin(GPIO_LED33,0);
			//CAN任务
			if (cnt > 5000)
			{
				cnt = 0;
				CANTx_Tsk();
			}
			if (1 == FlagRxCAN)
			{
				FlagRxCAN = 0;
				CANRx_Tsk();
			}
		}
	}
	return 0;
}

void CANTx_Tsk(void)
{
//	static unsigned char msgid = 0x101;
	static int16 udc;

	/*
	 * 显示电压
	 */
	ucTXMsgData[0] = ((int16) UdcA & 0xFF00) >> 8;
	ucTXMsgData[1] = (int16) UdcA & 0xFF;
	ucTXMsgData[2] = ((int16) UdcB & 0xFF00) >> 8;
	ucTXMsgData[3] = (int16) UdcB & 0xFF;
	ucTXMsgData[4] = ((int16) UdcC & 0xFF00) >> 8;
	ucTXMsgData[5] = (int16) UdcC & 0xFF;
	ucTXMsgData[6] = ((int16) UacRectifier & 0xFF00) >> 8;
	ucTXMsgData[7] = (int16) UacRectifier & 0xFF;
//	udc++;
//	ucTXMsgData[6] = ((int16) udc & 0xFF00) >> 8;
//	ucTXMsgData[7] = (int16) udc & 0xFF;

	sTXCANMessage.ui32MsgID = 0x102;
	sTXCANMessage.ui32MsgIDMask = 0;
	sTXCANMessage.ui32Flags = MSG_OBJ_NO_FLAGS;
	sTXCANMessage.ui32MsgLen = 8;
	sTXCANMessage.pucMsgData = ucTXMsgData;
	CANMessageSet(CANA_BASE, 1, &sTXCANMessage, MSG_OBJ_TYPE_TX);

	ucTXMsgData[6] = ((int16) APF_State & 0xFF00) >> 8;
	ucTXMsgData[7] = (int16) APF_State & 0xFF;

	sTXCANMessage.ui32MsgID = 0x108;
	sTXCANMessage.ui32MsgIDMask = 0;
	sTXCANMessage.ui32Flags = MSG_OBJ_NO_FLAGS;
	sTXCANMessage.ui32MsgLen = 8;
	sTXCANMessage.pucMsgData = ucTXMsgData;
	CANMessageSet(CANA_BASE, 1, &sTXCANMessage, MSG_OBJ_TYPE_TX);
}

void CANRx_Tsk(void)
{
	switch (sRXCANMessage.ui32MsgID)
	{
	case 0x201:
		if (1 == ucRXMsgData[0])
			APF_State &= (~APF_STATE_BIT_STOP);
		else
			APF_State |= APF_STATE_BIT_STOP;
		if (1 == ucRXMsgData[1])
			APF_State |= APF_STATE_BIT_TEST;
		else
			APF_State &= (~APF_STATE_BIT_TEST);
		break;
	case 0x202:
		PI_Ialfa.Kp = ((ucRXMsgData[0] << 8) | ucRXMsgData[1]) * 0.01;
		PI_Ialfa.Ki = ((ucRXMsgData[2] << 8) | ucRXMsgData[3]) * 0.01
				* APF_SAMPLE_PERIOD / PI_Ialfa.Kp;
		PI_Ibeta.Kp = PI_Ialfa.Kp;
		PI_Ibeta.Ki = PI_Ialfa.Ki;

		PI_Udcp.Kp = ((ucRXMsgData[4] << 8) | ucRXMsgData[5]) * 0.01;
		PI_Udcp.Ki = ((ucRXMsgData[6] << 8) | ucRXMsgData[7]) * 0.01
				* APF_SAMPLE_PERIOD / PI_Udcp.Kp;
		break;
	case 0x203:
		UdcRef = (ucRXMsgData[0] << 8) | ucRXMsgData[1];
		QRef = (ucRXMsgData[2] << 8) | ucRXMsgData[3];
		CmpsateSet = (ucRXMsgData[4] << 8) | ucRXMsgData[5];
		break;
	case 0x205:
		PI_Udcn_d.Kp = ((ucRXMsgData[0] << 8) | ucRXMsgData[1]) * 0.01;
		PI_Udcn_d.Ki = ((ucRXMsgData[2] << 8) | ucRXMsgData[3]) * 0.01
				* APF_SAMPLE_PERIOD / PI_Udcn_d.Kp;
		PI_Udcn_q.Kp = PI_Udcn_d.Kp;
		PI_Udcn_q.Ki = PI_Udcn_d.Ki;
//		pid_instance_Udcp.Kp = (ucRXMsgData[4] << 8) | ucRXMsgData[5];
//		pid_instance_Udcp.Kp = pid_instance_Ialfa.Kp;
//		pid_instance_Udcp.Ki = ((ucRXMsgData[6] << 8) | ucRXMsgData[7]) * APF_SWITCH_PERIOD;
//		pid_instance_Udcp.Ki = pid_instance_Ialfa.Ki;
		break;
	case 0x206:
		RpGain = ((ucRXMsgData[0] << 8) | ucRXMsgData[1]) * 0.1;
		RpKr = ((ucRXMsgData[2] << 8) | ucRXMsgData[3]) * 0.01;
		RpLeadingBeat = (ucRXMsgData[4] << 8) | ucRXMsgData[5];
		RpCutoffFreqence = (ucRXMsgData[6] << 8) | ucRXMsgData[7];
		break;
	default:
		break;
	}
}


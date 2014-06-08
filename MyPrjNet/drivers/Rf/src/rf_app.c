/*
 * rf_app.c
 *
 *  Created on: 2014年3月25日
 *      Author: Administrator
 */

#include <includes.h>

#define PKTSTATUS_CCA	1 << 4
#define PKTSTATUS_CS	1 << 6

#define RF_PAPD_INT_FLAG_IS_SET()                 RfGDOIntFlag
#define RF_CLEAR_PAPD_PIN_INT_FLAG()              (RfGDOIntFlag = 0)
#define RF_CONFIG_GDO0_AS_PAPD_SIGNAL()           halSpiWriteReg(CCxxx0_IOCFG0, 27)
#define RF_CONFIG_GDO0_AS_SYNC_SIGNAL()         halSpiWriteReg(CCxxx0_IOCFG0, 6)

// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
	uint8_t FSCTRL2;		//自已加的
	uint8_t FSCTRL1;   // Frequency synthesizer control.
	uint8_t FSCTRL0;   // Frequency synthesizer control.
	uint8_t FREQ2;     // Frequency control word, high uint8_t.
	uint8_t FREQ1;     // Frequency control word, middle uint8_t.
	uint8_t FREQ0;     // Frequency control word, low uint8_t.
	uint8_t MDMCFG4;   // Modem configuration.
	uint8_t MDMCFG3;   // Modem configuration.
	uint8_t MDMCFG2;   // Modem configuration.
	uint8_t MDMCFG1;   // Modem configuration.
	uint8_t MDMCFG0;   // Modem configuration.
	uint8_t CHANNR;    // Channel number.
	uint8_t DEVIATN; // Modem deviation setting (when FSK modulation is enabled).
	uint8_t FREND1;    // Front end RX configuration.
	uint8_t FREND0;    // Front end RX configuration.
	uint8_t MCSM1;
	uint8_t MCSM0;     // Main Radio Control State Machine configuration.
	uint8_t FOCCFG;    // Frequency Offset Compensation Configuration.
	uint8_t BSCFG;     // Bit synchronization Configuration.
	uint8_t AGCCTRL2;  // AGC control.
	uint8_t AGCCTRL1;  // AGC control.
	uint8_t AGCCTRL0;  // AGC control.
	uint8_t FSCAL3;    // Frequency synthesizer calibration.
	uint8_t FSCAL2;    // Frequency synthesizer calibration.
	uint8_t FSCAL1;    // Frequency synthesizer calibration.
	uint8_t FSCAL0;    // Frequency synthesizer calibration.
	uint8_t FSTEST;    // Frequency synthesizer calibration control
	uint8_t TEST2;     // Various test settings.
	uint8_t TEST1;     // Various test settings.
	uint8_t TEST0;     // Various test settings.
	uint8_t IOCFG2;    // GDO2 output pin configuration
	uint8_t IOCFG0;    // GDO0 output pin configuration
	uint8_t PKTCTRL1;  // Packet automation control.
	uint8_t PKTCTRL0;  // Packet automation control.
	uint8_t ADDR;      // Device address.
	uint8_t PKTLEN;    // Packet length.
} RF_SETTINGS;

/////////////////////////////////////////////////////////////////
const RF_SETTINGS rfSettings =
{ 0x00, 0x08,   // FSCTRL1   Frequency synthesizer control.
		0x00,   // FSCTRL0   Frequency synthesizer control.
		0x10,   // FREQ2     Frequency control word, high byte.
		0xA7,   // FREQ1     Frequency control word, middle byte.
		0x62,   // FREQ0     Frequency control word, low byte.
		0x5B,   // MDMCFG4   Modem configuration.
		0xF8,   // MDMCFG3   Modem configuration.
		0x03,   // MDMCFG2   Modem configuration.
		0x22,   // MDMCFG1   Modem configuration.
		0xF8,   // MDMCFG0   Modem configuration.

		0x00,   // CHANNR    Channel number.
		0x47, // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
		0xB6,   // FREND1    Front end RX configuration.
		0x10,   // FREND0    Front end RX configuration.
		0x3C,	// MCSM1
		0x18,   // MCSM0     Main Radio Control State Machine configuration.
		0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
		0x1C,   // BSCFG     Bit synchronization Configuration.
		0xFF,   // AGCCTRL2  AGC control.XXX C7 -> 0XFF
		0x40,   // AGCCTRL1  AGC control. XXX 00->0X40
		0xB2,   // AGCCTRL0  AGC control.

		0xEA,   // FSCAL3    Frequency synthesizer calibration.
		0x2A,   // FSCAL2    Frequency synthesizer calibration.
		0x00,   // FSCAL1    Frequency synthesizer calibration.
		0x11,   // FSCAL0    Frequency synthesizer calibration.
		0x59,   // FSTEST    Frequency synthesizer calibration.
		0x81,   // TEST2     Various test settings.
		0x35,   // TEST1     Various test settings.
		0x09,   // TEST0     Various test settings.
		0x0E,   // IOCFG2    GDO2 output pin configuration.
		0x06, // IOCFG0    GDO0 output pin configuration. Refer to SmartRF?Studio User Manual for detailed pseudo register explanation.

		0x04,   // PKTCTRL1  Packet automation control.
		0x05,   // PKTCTRL0  Packet automation control.
		0x00,   // ADDR      Device address.
		0x0C    // PKTLEN    Packet length.
//    0XFF	// PKTLEN    Packet length.
		};

//*****************************************************************************************
//函数名：void halRfWriteRfSettings(RF_SETTINGS *pRfSettings)
//输入：无
//输出：无
//功能描述：配置CC1100的寄存器
//*****************************************************************************************
void halRfWriteRfSettings(void)
{

//	halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL2);//自已加的
	// Write register settings
	halSpiWriteReg(CCxxx0_FSCTRL1, rfSettings.FSCTRL1);
	halSpiWriteReg(CCxxx0_FSCTRL0, rfSettings.FSCTRL0);
	halSpiWriteReg(CCxxx0_FREQ2, rfSettings.FREQ2);
	halSpiWriteReg(CCxxx0_FREQ1, rfSettings.FREQ1);
	halSpiWriteReg(CCxxx0_FREQ0, rfSettings.FREQ0);
	halSpiWriteReg(CCxxx0_MDMCFG4, rfSettings.MDMCFG4);
	halSpiWriteReg(CCxxx0_MDMCFG3, rfSettings.MDMCFG3);
	halSpiWriteReg(CCxxx0_MDMCFG2, rfSettings.MDMCFG2);
	halSpiWriteReg(CCxxx0_MDMCFG1, rfSettings.MDMCFG1);
	halSpiWriteReg(CCxxx0_MDMCFG0, rfSettings.MDMCFG0);
	halSpiWriteReg(CCxxx0_CHANNR, rfSettings.CHANNR);
	halSpiWriteReg(CCxxx0_DEVIATN, rfSettings.DEVIATN);
	halSpiWriteReg(CCxxx0_FREND1, rfSettings.FREND1);
	halSpiWriteReg(CCxxx0_FREND0, rfSettings.FREND0);
	halSpiWriteReg(CCxxx0_MCSM1, rfSettings.MCSM1);	//XXX 这个地方是我新加的
	halSpiWriteReg(CCxxx0_MCSM0, rfSettings.MCSM0);
	halSpiWriteReg(CCxxx0_FOCCFG, rfSettings.FOCCFG);
	halSpiWriteReg(CCxxx0_BSCFG, rfSettings.BSCFG);
	halSpiWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
	halSpiWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);
	halSpiWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
	halSpiWriteReg(CCxxx0_FSCAL3, rfSettings.FSCAL3);
	halSpiWriteReg(CCxxx0_FSCAL2, rfSettings.FSCAL2);
	halSpiWriteReg(CCxxx0_FSCAL1, rfSettings.FSCAL1);
	halSpiWriteReg(CCxxx0_FSCAL0, rfSettings.FSCAL0);
	halSpiWriteReg(CCxxx0_FSTEST, rfSettings.FSTEST);
	halSpiWriteReg(CCxxx0_TEST2, rfSettings.TEST2);
	halSpiWriteReg(CCxxx0_TEST1, rfSettings.TEST1);
	halSpiWriteReg(CCxxx0_TEST0, rfSettings.TEST0);
	halSpiWriteReg(CCxxx0_IOCFG2, rfSettings.IOCFG2);
	halSpiWriteReg(CCxxx0_IOCFG0, rfSettings.IOCFG0);
	halSpiWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
	halSpiWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
	halSpiWriteReg(CCxxx0_ADDR, rfSettings.ADDR);
	halSpiWriteReg(CCxxx0_PKTLEN, rfSettings.PKTLEN);
}

void RF_STROBE_IDLE_AND_WAIT(void)
{
	halSpiStrobe( CCxxx0_SIDLE);
	while (halSpiStrobe(CCxxx0_SNOP) & 0XF0)
		;
}

//*****************************************************************************************
//函数名：void halRfSendPacket(uint8_t *txBuffer, uint8_t size)
//输入：发送的缓冲区，发送数据个数
//输出：无
//功能描述：CC1100发送一组数据
//*****************************************************************************************

void halRfSendPacket(uint8_t *txBuffer, uint8_t size)
{
	uint32_t event;
	halRfTransmitEnable();
	RF_STROBE_IDLE_AND_WAIT();
	halRfPrepareToTx(txBuffer, size);
	RfState = RFSTATE_TX;
	halRfSyncPinINTCmd(ENABLE);	//开同步中断
	halSpiStrobe(CCxxx0_STX);		//进入发送模式发送数据
	rt_event_recv(&event_rf, RFSTATE_TX,
	RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &event);//等待接收中断
	halRfSyncPinINTCmd(DISABLE);	//关同步中断
	RfState = RFSTATE_IDLE;
}

int8_t RfCalRssiValue(uint8_t rssi)
{
	int16_t db;
	if (rssi >= 128)
	{
		db = (int16_t) (rssi - 256.0) / 2 - RSSI_OFFSET;
	}
	else
	{
		db = (int16_t) (rssi / 2.0) - RSSI_OFFSET;
	}

	return db;
}

uint8_t halRfReceivePacket(uint8_t *rxBuffer, uint8_t *status)
{
	uint8_t packetLength;

	if ((halSpiReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)) //如果接的字节数不为0
	{
		packetLength = halSpiReadReg(CCxxx0_RXFIFO); //读出第一个字节，此字节为该帧数据长度
		if (packetLength != 6)
		{
			halSpiStrobe(CCxxx0_SFRX);		//清洗接收缓冲区
			return 0;	//因为是固定数据包传输，所以这里接收长度不为6就返回
		}
		halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength); //读出所有接收到的数据

		// Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
		halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2); 	//读出CRC校验位
		halSpiStrobe(CCxxx0_SFRX);		//清洗接收缓冲区
		return packetLength;			//如果校验成功返回接收成功
	}
	halSpiStrobe(CCxxx0_SFRX);		//清洗接收缓冲区
	return 0;
}

void halRfSyncPinINTCmd(uint8_t state)
{
	if (state == 0)
	{
		EXTI->IMR &= ~EXTI_Line7;	//清除该位
	}
	else
	{
		EXTI->IMR |= EXTI_Line7;	//使能该位
	}
}

void halRfRxModeOn(void)
{
	RfState = RFSTATE_RX;
	halRfReceiveEnable();
	halRfSyncPinINTCmd(ENABLE);	//开接收同步中断
	halSpiStrobe(CCxxx0_SRX);		//进入接收状态
}

void halRfRxModeOff(void)
{
	/* disable receive interrupts */
	halRfSyncPinINTCmd(DISABLE);

	/* turn off radio */
	RF_STROBE_IDLE_AND_WAIT();

	/* flush the receive FIFO of any residual data */
	halSpiStrobe( CCxxx0_SFRX);

	RF_CLEAR_PAPD_PIN_INT_FLAG();

	RfState = RFSTATE_IDLE;
}

void halRfPrepareToTx(uint8_t *txBuffer, uint8_t size)
{
	halSpiStrobe( CCxxx0_SFTX); // flush the tx fifo
	halSpiWriteReg(CCxxx0_TXFIFO, size);
	halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);	//写入要发送的数据
}

/*
 ********************************************************************************
 返回值： 1，发送失败；0，发送成功
 ********************************************************************************
 */
uint8_t halRfTransmit(uint8_t *txBuffer, uint8_t size)
{
	uint8_t ccaRetries = 4, returnValue = 0;
	uint16_t timeout;

	halRfRxModeOff();
	halRfPrepareToTx(txBuffer, size);
	ccaRetries = 4;
	RF_CONFIG_GDO0_AS_PAPD_SIGNAL();
	for (;;)
	{
		halRfReceiveEnable();
		halSpiStrobe(CCxxx0_SRX);
		rt_delay_us(900);
		RfGDOIntFlag = 0;
		halRfSyncPinINTCmd(ENABLE);	//开同步中断
		RfState = RFSTATE_TX;
		halRfTransmitEnable();
		halSpiStrobe(CCxxx0_STX);		//进入发送模式发送数据
		rt_delay_us(30);
		if (RfGDOIntFlag)
		{
			RfGDOIntFlag = 0;
			Rf_WaitTxComplete(3000);
			break;
		}
		else
		{
			halRfSyncPinINTCmd(DISABLE);	//关同步中断
			/* turn off radio */
			RF_STROBE_IDLE_AND_WAIT();
			halSpiStrobe(CCxxx0_SFRX);	//清除Tx FIFO
			if (ccaRetries != 0)
			{
				rt_delay_us(1000);
				halRfPrepareToTx(txBuffer, size);
				ccaRetries--;
			}
			else
			{
				returnValue = 1;
				break;
			}
		}
	}
	halRfSyncPinINTCmd(DISABLE);
	RF_STROBE_IDLE_AND_WAIT();
//	RF_CONFIG_GDO0_AS_SYNC_SIGNAL();
	RfState = RFSTATE_IDLE;
	return returnValue;
}

uint8_t Rf_WaitTxComplete(uint16_t t)
{
	uint8_t timeout = 0;
	uint16_t count;

	count = t / 16;
	do
	{
		rt_delay_us(16);
//		timeout = RfGDOIntFlag;
		timeout = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
	} while (!timeout && count--);
	return timeout;
}

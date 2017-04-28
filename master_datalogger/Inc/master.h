/*
 * master.h
 *
 *  Created on: Apr 8, 2017
 *      Author: Constellations
 */

#ifndef MASTER_H_
#define MASTER_H_

// AllCell BMS CAN Bus

/*
 * Use this function to parse CAN byte stream into structures.
 * memcpy(&parsedData, data, sizeof(data));
 */

#define _CAR 1

#ifdef _CAR

#define _ERRORHANDLER_CAN1TRANSMIT 1
#define _REBROADCAST_ALLCELL 1

#endif

#ifndef _CAR

#define _DEBUG_ON 1

#ifdef _DEBUG_ON
#define _REBROADCAST_ALLCELL 1
#define _CAN_PRINTF 1
#endif

#endif

// #define _CAN_BMS_REBROADCAST 1

typedef enum {
	AllCell_Bat_State_ID = 0x0CFF2020,
	AllCell_Bat_Info_ID = 0x0CFF2120,
	AllCell_Bat_Current_ID = 0x18FF8020,
	AllCell_Bat_Voltage_ID =  0x18FF8120,
	AllCell_Bat_Temperature_ID = 0x18FF8220,
	AllCell_Bat_Status_ID =  0x18FF8320,
	AllCell_Bat_PwAvailable_ID = 0x18FF8420,
	AllCell_Bat_RTC_ID = 0x18FFD020,
	AllCell_Bat_RTC_SET = 0x18FF1621,
} AllCell_BMS_CAN_ID;

typedef struct {
	double currentInfoMsg; // Amps
	double voltageInfoMsg; // Volts
	double impedance; // mOhms
	double currentCurMsg; // Amps
	double chargeLim; // Amps
	double dischargeLim; // Amps
	double voltageVoltMsg; // Volts
	double mincellVoltage; // Volts
	double maxcellVoltage; // Volts
	double percentCharged; // %
	double currentCapacity; // Ahr
} AllCell_Bat_DataDoubles;

typedef struct {
	uint16_t current;
	uint16_t voltage;
	uint8_t temperature;
	uint8_t bat_percentage;
} masterCAN1_BMS;


typedef struct {
	uint8_t State;
	uint16_t Fault_Code;
	uint16_t Timer;
	uint16_t Reset;
	uint8_t Balance;
} AllCell_Bat_State;

typedef struct {
	uint16_t Current; // Amps
	uint16_t Voltage; // Volts
	uint8_t Temp; // Celsius
	uint16_t Impedance; // mOhms
} AllCell_Bat_Info;

typedef struct {
	uint16_t Current;
	uint16_t Charge_Limit;
	uint16_t Discharge_Limit;
} AllCell_Bat_Current;

typedef struct {
	uint16_t Voltage;
	uint16_t Min_Cell_Voltage;
	uint8_t Nb_Min_Voltage;
	uint16_t Max_Cell_Voltage;
	uint8_t Nb_Max_Voltage;
} AllCell_Bat_Voltage;

typedef struct {
	uint8_t Temp_BMS;
	uint8_t Avg_Cell_Temp;
	uint8_t Min_Cell_Temp;
	uint8_t Nb_Min_Temp;
	uint8_t Max_Cell_Temp;
	uint8_t Nb_Max_Temp;
	// uint8_t Temp_Ambient; // Unused for this BMS version
} AllCell_Bat_Temperature;

typedef struct {
	uint8_t SOC;
	uint16_t Capacity;
	// uint8_t SOH; // Unused for this BMS version
} AllCell_Bat_Status;

typedef struct {
	uint32_t PwAvailable_Charge;
	uint32_t PwAvailable_Discharge;
} AllCell_Bat_PwAvailable;

typedef struct {
	uint8_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
} AllCell_Bat_RTC;


//-----------------------------

typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS
} CAN_PACKET_ID;
typedef enum {
	ecoMotion_MotorControl = 0x01,
	ecoMotion_Speed = 0x02,
	ecoMotion_FrontWheels = 0x03,
	ecoMotion_Master_BMS = 0x04,
	ecoMotion_Humidity = 0x05,
	ecoMotion_Temperature = 0x06,
	ecoMotion_Throttle = 0x20,
	ecoMotion_Master = 0x30,
	ecoMotion_MasterBMS = 0x31,
	ecoMotion_MasterRTC = 0x3A,
	ecoMotion_Display = 0x40,
	ecoMotion_Error_Throttle = 0xFFF,
	ecoMotion_Error_Master = 0x0FEF,
	ecoMotion_Error_Display = 0xFDF,
} CAN_DEVICE_ID;
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
int getTim1Prescaler();
static void setCANbitRate(uint16_t bitRate, uint16_t periphClock, CAN_HandleTypeDef* theHcan);
void parseBMSCAN(CanRxMsgTypeDef *BMSRxMsg);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
char *itoa (int value, char *result, int base);
void speedCalc(int clockSpeed, float wheelDiameter, int compareVal, float* rpmVal, float* speedVal);
void __io_putchar(uint8_t ch);
void printUART2();
void setAllCellRTC();

#endif /* MASTER_H_ */

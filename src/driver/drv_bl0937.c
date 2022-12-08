#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "../hal/hal_pins.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../httpclient/http_client.h"

#if PLATFORM_BEKEN

#include "BkDriverTimer.h"
#include "BkDriverGpio.h"
#include "sys_timer.h"
#include "gw_intf.h"

// HLW8012 aka BL0937

#define ELE_HW_TIME 1
#define HW_TIMER_ID 0
#else

#endif

#include "../monocypher/monocypher.h"

// Those can be set by Web page pins configurator
// The below are default values for Mycket smart socket
int GPIO_HLW_SEL = 24; // pwm4
int GPIO_HLW_CF = 7;
int GPIO_HLW_CF1 = 8;

bool g_sel = true;
uint32_t res_v = 0;
uint32_t res_c = 0;
uint32_t res_p = 0;
float BL0937_VREF = 0.13253012048f;
float BL0937_PREF = 1.5f;
float BL0937_CREF = 0.0118577075f;
float BL0937_PMAX = 3680.0f;
float last_p = 0.0f;

volatile uint32_t g_vc_pulses = 0;
volatile uint32_t g_p_pulses = 0;
static portTickType pulseStamp;

void HlwCf1Interrupt(unsigned char pinNum) {  // Service Voltage and Current
	g_vc_pulses++;
}
void HlwCfInterrupt(unsigned char pinNum) {  // Service Power
	g_p_pulses++;
}
int BL0937_PowerSet(const void* context, const char* cmd, const char* args, int cmdFlags) {
	float realPower;

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	realPower = atof(args);
	BL0937_PREF = realPower / res_p;

	// UPDATE: now they are automatically saved
	CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_POWER, BL0937_PREF);

	{
		char dbg[128];
		snprintf(dbg, sizeof(dbg), "PowerSet: you gave %f, set ref to %f\n", realPower, BL0937_PREF);
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
	}
	return 0;
}

int BL0937_PowerMax(const void* context, const char* cmd, const char* args, int cmdFlags) {
	float maxPower;

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	maxPower = atof(args);
	if ((maxPower > 200.0) && (maxPower < 7200.0f))
	{
		BL0937_PMAX = maxPower;
		// UPDATE: now they are automatically saved
		CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_POWER_MAX, BL0937_PMAX);
		{
			char dbg[128];
			snprintf(dbg, sizeof(dbg), "PowerMax: set max to %f\n", BL0937_PMAX);
			addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
		}
	}
	return 0;
}

int BL0937_PowerRef(const void* context, const char* cmd, const char* args, int cmdFlags) {

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	BL0937_PREF = atof(args);

	// UPDATE: now they are automatically saved
	CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_POWER, BL0937_PREF);

	return 0;
}
int BL0937_CurrentRef(const void* context, const char* cmd, const char* args, int cmdFlags) {

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	BL0937_CREF = atof(args);

	// UPDATE: now they are automatically saved
	CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT, BL0937_CREF);

	return 0;
}
int BL0937_VoltageRef(const void* context, const char* cmd, const char* args, int cmdFlags) {

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	BL0937_VREF = atof(args);

	// UPDATE: now they are automatically saved
	CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_VOLTAGE, BL0937_VREF);

	return 0;
}
int BL0937_VoltageSet(const void* context, const char* cmd, const char* args, int cmdFlags) {
	float realV;

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	realV = atof(args);
	BL0937_VREF = realV / res_v;

	// UPDATE: now they are automatically saved
	CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_VOLTAGE, BL0937_VREF);

	{
		char dbg[128];
		snprintf(dbg, sizeof(dbg), "VoltageSet: you gave %f, set ref to %f\n", realV, BL0937_VREF);
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
	}

	return 0;
}
int BL0937_CurrentSet(const void* context, const char* cmd, const char* args, int cmdFlags) {
	float realI;

	if (args == 0 || *args == 0) {
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "This command needs one argument");
		return 1;
	}
	realI = atof(args);
	BL0937_CREF = realI / res_c;

	// UPDATE: now they are automatically saved
	CFG_SetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT, BL0937_CREF);

	{
		char dbg[128];
		snprintf(dbg, sizeof(dbg), "CurrentSet: you gave %f, set ref to %f\n", realI, BL0937_CREF);
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
	}
	return 0;
}

void BL0937_Init()
{
	BL_Shared_Init();
	// if not found, this will return the already set value
	GPIO_HLW_SEL = PIN_FindPinIndexForRole(IOR_BL0937_SEL, GPIO_HLW_SEL);
	GPIO_HLW_CF = PIN_FindPinIndexForRole(IOR_BL0937_CF, GPIO_HLW_CF);
	GPIO_HLW_CF1 = PIN_FindPinIndexForRole(IOR_BL0937_CF1, GPIO_HLW_CF1);

	// UPDATE: now they are automatically saved
	BL0937_VREF = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_VOLTAGE, BL0937_VREF);
	BL0937_PREF = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_POWER, BL0937_PREF);
	BL0937_CREF = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_CURRENT, BL0937_CREF);
	BL0937_PMAX = CFG_GetPowerMeasurementCalibrationFloat(CFG_OBK_POWER_MAX, BL0937_PMAX);

	HAL_PIN_Setup_Output(GPIO_HLW_SEL);
	HAL_PIN_SetOutputValue(GPIO_HLW_SEL, g_sel);

	HAL_PIN_Setup_Input_Pullup(GPIO_HLW_CF1);

#if PLATFORM_BEKEN
	gpio_int_enable(GPIO_HLW_CF1, IRQ_TRIGGER_FALLING_EDGE, HlwCf1Interrupt);
#else

#endif
	HAL_PIN_Setup_Input_Pullup(GPIO_HLW_CF);
#if PLATFORM_BEKEN
	gpio_int_enable(GPIO_HLW_CF, IRQ_TRIGGER_FALLING_EDGE, HlwCfInterrupt);
#else

#endif

	CMD_RegisterCommand("PowerSet", "", BL0937_PowerSet, "Sets current power value for calibration", NULL);
	CMD_RegisterCommand("VoltageSet", "", BL0937_VoltageSet, "Sets current V value for calibration", NULL);
	CMD_RegisterCommand("CurrentSet", "", BL0937_CurrentSet, "Sets current I value for calibration", NULL);
	CMD_RegisterCommand("PREF", "", BL0937_PowerRef, "Sets the calibration multiplier", NULL);
	CMD_RegisterCommand("VREF", "", BL0937_VoltageRef, "Sets the calibration multiplier", NULL);
	CMD_RegisterCommand("IREF", "", BL0937_CurrentRef, "Sets the calibration multiplier", NULL);
	CMD_RegisterCommand("PowerMax", "", BL0937_PowerMax, "Sets Maximum power value measurement limiter", NULL);

	g_vc_pulses = 0;
	g_p_pulses = 0;
	pulseStamp = xTaskGetTickCount();
}

#define DATA_SEND_PERIOD_SEC 60
#define FIRST_DATA_SEND_DELAY 10
static bool dataSendingStarted = false;
static uint32_t secondsSkipped;

// void randBuf(uint8_t* buf, int size) {
// 	for (int i = 0; i < size; i++)
// 		buf[i] = rand() % 255;
// }

void BL0937_RunFrame()
{
	float final_v;
	float final_c;
	float final_p;
	portTickType ticksElapsed;

	ticksElapsed = (xTaskGetTickCount() - pulseStamp);

#if PLATFORM_BEKEN
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
#else

#endif
	if (g_sel) {
		res_v = g_vc_pulses;
		g_sel = false;
	}
	else {
		res_c = g_vc_pulses;
		g_sel = true;
	}
	HAL_PIN_SetOutputValue(GPIO_HLW_SEL, g_sel);
	g_vc_pulses = 0;

	res_p = g_p_pulses;
	g_p_pulses = 0;
#if PLATFORM_BEKEN
	GLOBAL_INT_RESTORE();
#else

#endif

	pulseStamp = xTaskGetTickCount();
	//addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER,"Voltage pulses %i, current %i, power %i\n", res_v, res_c, res_p);

	final_v = res_v * BL0937_VREF;
	final_v *= (float)ticksElapsed;
	final_v /= (1000.0f / (float)portTICK_PERIOD_MS);

	final_c = res_c * BL0937_CREF;
	final_c *= (float)ticksElapsed;
	final_c /= (1000.0f / (float)portTICK_PERIOD_MS);

	final_p = res_p * BL0937_PREF;
	final_p *= (float)ticksElapsed;
	final_p /= (1000.0f / (float)portTICK_PERIOD_MS);

	/* patch to limit max power reading, filter random reading errors */
	if (final_p > BL0937_PMAX)
	{
		/* MAX value breach, use last value */
		{
			char dbg[128];
			snprintf(dbg, sizeof(dbg), "Power reading: %f exceeded MAX limit: %f, Last: %f\n", final_p, BL0937_PMAX, last_p);
			addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
		}
		final_p = last_p;
	}
	else {
		/* Valid value save for next time */
		last_p = final_p;
	}
#if 0
	{
		char dbg[128];
		snprintf(dbg, sizeof(dbg), "Voltage %f, current %f, power %f\n", final_v, final_c, final_p);
		addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, dbg);
	}
#endif
	BL_ProcessUpdate(final_v, final_c, final_p);

	//HTTPS://api.solarbro.eu/devices/testDevice/states
	static char jsonData[1024];
	static char* contentType = "application/json";
	static char* username = "device0000000000";
	static char* password = "nFy2i1u10eBdE8w7";

	if (dataSendingStarted) {
		if (secondsSkipped >= DATA_SEND_PERIOD_SEC) {
			// snprintf(jsonData, sizeof(jsonData), "{'voltage':%.2f,'current':%.2f,'power':%.2f,'uptime':%d,'driver':'%s','chipset':'%s','deviceName':'%s','macAddr':'%02X:%02X:%02X:%02X:%02X:%02X','wifiStrength':%d}",
			// 	final_v, final_c, final_p, Time_getUpTimeSeconds(), "BL0937", PLATFORM_MCU_NAME, g_cfg.longDeviceName, g_cfg.mac[0], g_cfg.mac[1], g_cfg.mac[2], g_cfg.mac[3], g_cfg.mac[4], g_cfg.mac[5], HAL_GetWifiStrength());
			//HTTPClient_Async_SendPost("HTTP://64.225.66.88:4589/tick?device_id=123", jsonData, contentType, username, password);
			snprintf(jsonData, sizeof(jsonData), "test");
			HTTPClient_Async_SendPost("HTTP://webhook.site/85ce01d4-dbe9-49ab-8c22-e33afc54c71f", jsonData, contentType, username, password);

			secondsSkipped = 0;
		}
		else {
			secondsSkipped++;
		}
	}
	else {
		if (Main_HasWiFiConnected()) {
			secondsSkipped = DATA_SEND_PERIOD_SEC - FIRST_DATA_SEND_DELAY;
			dataSendingStarted = true;
		}
	}

	// uint8_t key[32];    /* Random, secret session key  */
	// uint8_t nonce[24];    /* Use only once per key       */
	// uint8_t plain_text[12] = "Lorem ipsum"; /* Secret message */
	// uint8_t mac[16];    /* Message authentication code */
	// uint8_t cipher_text[12];              /* Encrypted message */

	// randBuf(key, 32); //arc4random_buf(key, 32);
	// randBuf(nonce, 24); //arc4random_buf(nonce, 24);
	// crypto_lock(mac, cipher_text, key, nonce, plain_text, sizeof(plain_text));
	// /* Wipe secrets if they are no longer needed */
	// // crypto_wipe(plain_text, 12);
	// // crypto_wipe(key, 32);
	// /* Transmit cipher_text, nonce, and mac over the network,
	//  * store them in a file, etc.
	//  */

	// ADDLOG_INFO(LOG_FEATURE_GENERAL, "ENC TEST: %s\r\n", cipher_text);
}


#include <ESP32Servo.h>
#include "esp32-hal-ledc.h"
#include <WiFi.h>
#include <arduino.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <max6675.h>
#include <Ticker.h>
#include <Preferences.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include "esp_task_wdt.h"
#include "webfiles.h"
#include <ArduinoJson.h>
#include "TFT_UPLOAD.h"

// OTA Options
#define VERSION "v1.2"
#define APP "SmartReflowOven"
const char *urlBase = "http://update.somp3.cn:5002/update";

// —— Pin Definitions ——

#define CS_PIN 33
#define SCK_PIN 18
#define DO_PIN 23

#define SSR1 12
#define SSR2 25
#define SSR_PWM_FREQ 5000
#define SSR_PWM_BITS 8

#define BUZZER_PIN 13

#define SERVO_PIN 4
#define FAN_PIN 27
#define FAN_PWM_FREQUENCY 25000
#define FAN_PWM_BITS 8
#define FAN_DEFAULT_SPEED 180
#define T_COLD 24
#define OPEN_POS 80
#define FAN_ON_DURATION 5

#define SSR_PWM_CHANNEL 10 // Select PWM channel 2 (0-15 available)
#define FAN_PWM_CHANNEL 12 // Select PWM channel 3

void handleRoot(AsyncWebServerRequest *request);

String IPAddr;
// Nextion Serial
// HardwareSerial nextion(2);
Servo myservo;
// —— Global Buffer ——
char input[32];
volatile bool UART_Received_Flag = false;
uint8_t UART_Received_Data[5] = {'p', '0', 'x', 'x', 'x'};
#define MAX_REFLOW_PROFILES 4

struct ReflowProfile
{
	float firstHeatUpRate;
	uint32_t SoakTemperature;
	uint32_t SoakTime;
	float secondHeatUpRate;
	uint32_t ReflowTemperature;
	uint32_t ReflowTime;
	char ProfileName[32];
};

struct ReflowConfig
{
	float KP;
	float Ki;
	float KD;
	float firstHeatUpRate;
	uint32_t SoakTemperature;
	uint32_t SoakTime;
	float secondHeatUpRate;
	uint32_t ReflowTemperature;
	uint32_t ReflowTime;
	unsigned int FanSpeed;
	unsigned int InvertedFan;
	ReflowProfile profiles[MAX_REFLOW_PROFILES]; // Four backup profiles
												 // uint32_t version;
} ReflowParameters;

String staSSID, staPASS, staIP, staMASK, staGW, staDNS;

struct Point
{
	float time;
	float temp;
};

typedef struct
{
	float A0;		// Derived coefficient A0 = Kp + Ki + Kd
	float A1;		// Derived coefficient A1 = -(Kp + 2 * Kd)
	float A2;		// Derived coefficient A2 = Kd
	float state[3]; // Error values e[n-1], e[n-2], output[n-1]
	float Kp;		// Proportional coefficient
	float Ki;		// Integral coefficient
	float Kd;		// Derivative coefficient
} arm_pid_instance_f32;

arm_pid_instance_f32 PID;

// —— Status & Environment ——
Preferences preferences;
MAX6675 thermocouple(SCK_PIN, CS_PIN, DO_PIN);
Ticker ticker;

bool ReflowEnable = false, BuzzerEnable = false;
bool ServoEnable = false;
int ServoPos = 0;

bool isConfiguring = false;
bool isUploadingTFT = false;
bool isUploadingFirmware = false;
bool FanEnable = false;
bool RebootEnable = false;

uint16_t ReflowIndex = 0, PhaseIndex[5] = {0};

const char *PhaseName[] = {"PREHEAT", "SOAK", "HEAT UP", "REFLOW", "COOLING"};
char currentPhaseName[16] = "IDLE";

uint8_t ReflowCurve[4000];
bool TempDrawEnable = false;
uint32_t TempDrawCounter = 0;
float temp = T_COLD, lastTemp = T_COLD, start_temp, duty = 0;
char ConsoleMSG[256] = "IDLE";
uint32_t TimerBUZZER = 0, TimerGui = 0, TimerFAN = 0, TimerDoor = 0;
int beep = 0;
int RunningTimeHalfSeconds = 0;

// PID Controller Functions
void arm_pid_init_f32(arm_pid_instance_f32 *S, int32_t resetStateFlag)
{
	S->A0 = S->Kp + S->Ki + S->Kd;
	S->A1 = -S->Kp - 2.0f * S->Kd;
	S->A2 = S->Kd;

	if (resetStateFlag)
	{
		S->state[0] = 0.0f;
		S->state[1] = 0.0f;
		S->state[2] = 0.0f;
	}
}

float arm_pid_f32(arm_pid_instance_f32 *S, float in)
{
	float out;

	out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

	S->state[1] = S->state[0];
	S->state[0] = in;
	S->state[2] = out;

	return out;
}

// — Interrupt Callback —
void IRAM_ATTR onUartReceive()
{
	uint8_t Temp_UART_Received_Data[5];
	// if (nextion.available() >= 5) {
	if (isUploadingTFT || isUploadingFirmware)
		return;
	nextion.readBytes(Temp_UART_Received_Data, 5);
	if (Temp_UART_Received_Data[0] < 128)
	{
		UART_Received_Flag = true;
		//  }
		memcpy(UART_Received_Data, Temp_UART_Received_Data, 5);
		Serial.println("onUartReceive:");
		for (int i = 0; i < 5; i++)
		{
			Serial.print((char)Temp_UART_Received_Data[i]); // ASCII character
			Serial.print(" (0x");
			Serial.print(Temp_UART_Received_Data[i], HEX); // HEX value
			Serial.print(") ");
		}
		Serial.println();
	}
	// while (nextion.read() >= 0);
}

void checkForUpdates(void)
{
	String checkUrl = String(urlBase);
	checkUrl.concat("?ver=" + String(VERSION));
	checkUrl.concat("&dev=" + String(APP));

	Serial.println("INFO: Checking for updates at URL: " + String(checkUrl));

	WiFiClient client;
	t_httpUpdate_return ret = httpUpdate.update(client, checkUrl);

	switch (ret)
	{
	default:
	case HTTP_UPDATE_FAILED:
		Serial.println("ERROR: HTTP_UPDATE_FAILD Error (" + String(httpUpdate.getLastError()) + "): " + httpUpdate.getLastErrorString().c_str());
		break;
	case HTTP_UPDATE_NO_UPDATES:
		Serial.println("INFO: HTTP_UPDATE_NO_UPDATES");
		break;
	case HTTP_UPDATE_OK:
		Serial.println("INFO status: HTTP_UPDATE_OK");
		break;
	}
}
static void NEXTION_CMD(const char *cmd)
{
	nextion.write(cmd, strlen(cmd));
	nextion.write(0xFF);
	nextion.write(0xFF);
	nextion.write(0xFF);
}

static void NEXTION_SendFloat(const char *ID, float number)
{
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%.2f\"", ID, number);
	NEXTION_CMD(buf);
}
void NEXTION_SendFloat_CurrentTemp(const char *ID, float number)
{
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%.1f\"", ID, number);
	NEXTION_CMD(buf);
}

static void NEXTION_SenduInt(const char *ID, int number)
{
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%lu\"", ID, number);
	NEXTION_CMD(buf);
}

static void NEXTION_SendString(const char *ID, const char *string)
{
	char buf[50];
	int len = sprintf(buf, "%s.txt=\"%s\"", ID, string);
	NEXTION_CMD(buf);
}
static void NEXTION_SendStringCat(const char *ID, const char *string)
{
	char buf[50];
	int len = sprintf(buf, "%s.txt+=\"%s\r\n\"", ID, string);
	NEXTION_CMD(buf);
}
static void NextionDrawDot(uint32_t x, uint32_t y)
{
	char buf[64];
	int len = sprintf(buf, "cirs %lu,%lu,2,RED", x, y); // RGB888: 0.160.255 RGB565:1311

	NEXTION_CMD(buf);
	// Clear return buffer (prevent blocking)
	while (Serial.available())
		Serial.read();
}

void Clear_UART_Received_Data()
{
	unsigned char defaultUart[5] = {'x', 'x', 'x', 'x', 'x'};
	for (int i = 0; i < 5; i++)
	{
		UART_Received_Data[i] = defaultUart[i];
	}

	UART_Received_Flag = 0;
}

float HandleKeyPad(int maxValue = 255)
{

	// clear Input
	for (int i = 0; i < 20; i++)
	{
		input[i] = 0;
	}

	uint8_t index = 0;

	NEXTION_SendString("t0", "");
	UART_Received_Flag = 0;

	while (strncmp((char *)UART_Received_Data, "enter", 5) != 0)
	{

		if (strncmp((char *)UART_Received_Data, "abbre", 5) == 0)
			return 9999;
		if (strncmp((char *)UART_Received_Data, "kback", 5) == 0)
			return 8888;

		if (UART_Received_Flag == 1)
		{
			input[index] = UART_Received_Data[4];
			UART_Received_Flag = 0;
			index++;
			NEXTION_SendString("t0", input);
		}
	}
	if (atof(input) >= maxValue)
	{
		return maxValue;
	}
	else
		return atof(input);
}
void initDefaultParams()
{
	ReflowParameters.KP = 85;
	ReflowParameters.Ki = 0.05f;
	ReflowParameters.KD = 130;
	ReflowParameters.FanSpeed = FAN_DEFAULT_SPEED; // Default value
	ReflowParameters.InvertedFan = 0;
	// Current profile in use (default uses profiles[0])
	ReflowParameters.firstHeatUpRate = 0.75f;
	ReflowParameters.SoakTemperature = 130;
	ReflowParameters.SoakTime = 100;
	ReflowParameters.secondHeatUpRate = 1.0f;
	ReflowParameters.ReflowTemperature = 165;
	ReflowParameters.ReflowTime = 100;

	// Recipe 0
	strcpy(ReflowParameters.profiles[0].ProfileName, "Lead 138C");
	ReflowParameters.profiles[0].firstHeatUpRate = 0.75f;
	ReflowParameters.profiles[0].SoakTemperature = 130;
	ReflowParameters.profiles[0].SoakTime = 100;
	ReflowParameters.profiles[0].secondHeatUpRate = 1.0f;
	ReflowParameters.profiles[0].ReflowTemperature = 165;
	ReflowParameters.profiles[0].ReflowTime = 100;

	// Recipe 1
	strcpy(ReflowParameters.profiles[1].ProfileName, "Lead 148C");
	ReflowParameters.profiles[1].firstHeatUpRate = 0.75f;
	ReflowParameters.profiles[1].SoakTemperature = 145;
	ReflowParameters.profiles[1].SoakTime = 110;
	ReflowParameters.profiles[1].secondHeatUpRate = 1.0f;
	ReflowParameters.profiles[1].ReflowTemperature = 190;
	ReflowParameters.profiles[1].ReflowTime = 95;

	// Recipe 2
	strcpy(ReflowParameters.profiles[2].ProfileName, "Lead 183C");
	ReflowParameters.profiles[2].firstHeatUpRate = 0.75f;
	ReflowParameters.profiles[2].SoakTemperature = 160;
	ReflowParameters.profiles[2].SoakTime = 120;
	ReflowParameters.profiles[2].secondHeatUpRate = 1.0f;
	ReflowParameters.profiles[2].ReflowTemperature = 215;
	ReflowParameters.profiles[2].ReflowTime = 90;

	// Recipe 3
	strcpy(ReflowParameters.profiles[3].ProfileName, "Lead 217C");
	ReflowParameters.profiles[3].firstHeatUpRate = 0.75f;
	ReflowParameters.profiles[3].SoakTemperature = 175;
	ReflowParameters.profiles[3].SoakTime = 130;
	ReflowParameters.profiles[3].secondHeatUpRate = 1.0f;
	ReflowParameters.profiles[3].ReflowTemperature = 240;
	ReflowParameters.profiles[3].ReflowTime = 85;

	// ReflowParameters.version = CONFIG_VERSION;
}
void ResetSettingsToDefault()
{
	initDefaultParams();
	staSSID = "";
	staPASS = "";
	staIP = "";
	staMASK = "";
	staGW = "";
	staDNS = "";
	SaveReflowParameters();
}
void SaveReflowParameters()
{
	preferences.begin("reflow", false);

	// Save structure data
	preferences.putBytes("cfg", &ReflowParameters, sizeof(ReflowParameters));

	// Save WiFi configuration
	preferences.putString("ssid", staSSID);
	preferences.putString("pass", staPASS);
	preferences.putString("ip", staIP);
	preferences.putString("mask", staMASK);
	preferences.putString("gw", staGW);
	preferences.putString("dns", staDNS);

	// Optional: Save version number for compatibility check
	// preferences.putUInt("version", CONFIG_VERSION);

	preferences.end();

	Serial.println("Reflow parameters saved successfully");
}

bool loadReflowParameters()
{
	preferences.begin("reflow", true); // Read-only mode

	// Check if config data exists and size is correct
	size_t cfgSize = preferences.getBytesLength("cfg");
	if (cfgSize != sizeof(ReflowParameters))
	{
		Serial.printf("Config size mismatch: expected %d, got %d\n",
					  sizeof(ReflowParameters), cfgSize);
		preferences.end();
		return false;
	}

	// Load structure data
	size_t bytesRead = preferences.getBytes("cfg", &ReflowParameters, sizeof(ReflowParameters));
	if (bytesRead != sizeof(ReflowParameters))
	{
		Serial.println("Failed to read config data");
		preferences.end();
		return false;
	}

	// Load WiFi configuration (with default values)
	staSSID = preferences.getString("ssid", "");
	staPASS = preferences.getString("pass", "");
	staIP = preferences.getString("ip", "192.168.1.170");
	staMASK = preferences.getString("mask", "255.255.255.0");
	staGW = preferences.getString("gw", "192.168.1.1");
	staDNS = preferences.getString("dns", "192.168.1.1");

	preferences.end();

	Serial.println("Reflow parameters loaded successfully");
	return true;
}

String formatTime(uint32_t RunningTimeHalfSeconds)
{
	uint32_t totalSeconds = RunningTimeHalfSeconds / 2;
	uint32_t minutes = totalSeconds / 60;
	uint32_t seconds = totalSeconds % 60;

	char buf[16];
	sprintf(buf, "%02um:%02us", minutes, seconds);
	return String(buf);
}

void Update_Page_0()
{
	const float dx = 0.20833f, dy = 0.7143f;
	const uint32_t OffsetX = 35, OffsetY = 240;
	Serial.println("Update_Page_0");
	uint8_t defaultUart[5] = {'p', '0', 'x', 'x', 'x'};
	for (int i = 0; i < 5; i++)
	{
		UART_Received_Data[i] = defaultUart[i];
	}

	if (ReflowEnable)
	{
		TempDrawEnable = true;
	}
	if (TempDrawEnable && TempDrawCounter < 4000)
	{
		NextionDrawDot(OffsetX + (uint32_t)((float)(TempDrawCounter)*dx), OffsetY - (uint32_t)((float)(temp)*dy));

		if (ReflowCurve[TempDrawCounter] == 0)
		{
			TempDrawEnable = false;
		}
	}

	NEXTION_SendFloat_CurrentTemp("t0", temp);
	NEXTION_SendFloat("t1", ReflowParameters.firstHeatUpRate);
	NEXTION_SenduInt("t2", ReflowParameters.SoakTemperature);
	NEXTION_SenduInt("t3", ReflowParameters.SoakTime);
	NEXTION_SendFloat("t4", ReflowParameters.secondHeatUpRate);
	NEXTION_SenduInt("t5", ReflowParameters.ReflowTemperature);
	NEXTION_SenduInt("t6", ReflowParameters.ReflowTime);
	NEXTION_SendString("t7", IPAddr.c_str());
	if (ReflowEnable)
	{
		NEXTION_SendString("t8", formatTime(RunningTimeHalfSeconds).c_str());
	}
	NEXTION_SendString("g1", ConsoleMSG);
}

void Update_Page_2()
{
	if (ReflowParameters.InvertedFan == 1)
	{
		NEXTION_CMD("c0.val=1");
	}
	else
	{
		NEXTION_CMD("c0.val=0");
	}

	uint8_t defaultUart[5] = {'p', '2', 'x', 'x', 'x'};
	for (int i = 0; i < 5; i++)
	{
		UART_Received_Data[i] = defaultUart[i];
	}

	NEXTION_SendFloat_CurrentTemp("t0", ReflowParameters.KP);
	NEXTION_SendFloat("t1", ReflowParameters.Ki);
	NEXTION_SendFloat("t2", ReflowParameters.KD);
	NEXTION_SendString("t3", VERSION);
	// String msg;
	NEXTION_SendString("t4", String(ReflowParameters.FanSpeed).c_str());


}

void Update_Page_3()
{
	uint8_t defaultUart[5] = {'p', '3', 'x', 'x', 'x'};
	for (int i = 0; i < 5; i++)
	{
		UART_Received_Data[i] = defaultUart[i];
	}
	NEXTION_SendFloat_CurrentTemp("t0", ReflowParameters.firstHeatUpRate);
	NEXTION_SenduInt("t1", ReflowParameters.SoakTime);
	NEXTION_SenduInt("t2", ReflowParameters.SoakTemperature);
	NEXTION_SendFloat("t3", ReflowParameters.secondHeatUpRate);
	NEXTION_SenduInt("t4", ReflowParameters.ReflowTime);
	NEXTION_SenduInt("t5", ReflowParameters.ReflowTemperature);
}
void Update_Page_6()
{
	uint8_t defaultUart[5] = {'p', '6', 'x', 'x', 'x'};
	for (int i = 0; i < 5; i++)
	{
		UART_Received_Data[i] = defaultUart[i];
	}
}

void calculateReflowCurve()
{
	for (int i = 0; i < 4000; i++)
	{
		ReflowCurve[i] = 0;
	}

	int index = 0;
	float timestep = 0.5;
	// First Heat Up:
	while (T_COLD + timestep * ReflowParameters.firstHeatUpRate <= ReflowParameters.SoakTemperature)
	{
		ReflowCurve[index] = T_COLD + timestep * ReflowParameters.firstHeatUpRate;
		index++;
		timestep = timestep + 0.5;
	}
	PhaseIndex[1] = index;

	// Soak
	int Soakduration = 2 * ReflowParameters.SoakTime;

	for (int i = 0; i < Soakduration; i++)
	{
		ReflowCurve[index + i] = ReflowParameters.SoakTemperature;
	}

	// Second Heat Up:
	index = index + Soakduration;
	PhaseIndex[2] = index;
	timestep = 0.5;
	while (ReflowParameters.SoakTemperature + timestep * ReflowParameters.secondHeatUpRate <= ReflowParameters.ReflowTemperature)
	{
		ReflowCurve[index] = ReflowParameters.SoakTemperature + (uint8_t)timestep * ReflowParameters.secondHeatUpRate;
		index++;
		timestep = timestep + 0.5;
	}
	PhaseIndex[3] = index;

	// Reflow
	int Reflowduration = 2 * ReflowParameters.ReflowTime;

	for (int i = 0; i < Reflowduration; i++)
	{
		ReflowCurve[index + i] = ReflowParameters.ReflowTemperature;
	}

	index = index + Reflowduration;
	ReflowCurve[index] = 0;
	PhaseIndex[4] = index;

	// Cooldown
	timestep = 0.5;
	while (ReflowParameters.ReflowTemperature - timestep * 1.8 >= T_COLD)
	{
		ReflowCurve[index] = ReflowParameters.ReflowTemperature - timestep * 1.8;
		index++;
		timestep = timestep + 0.5;
	}
}

void DrawThickLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t thickness, uint16_t color565)
{
	char cmd[64];
	for (int8_t offset = -(thickness / 2); offset <= (thickness / 2); offset++)
	{
		if (abs(x1 - x0) > abs(y1 - y0))
		{
			sprintf(cmd, "line %u,%u,%u,%u,%u", x0, y0 + offset, x1, y1 + offset, color565);
		}
		else
		{
			sprintf(cmd, "line %u,%u,%u,%u,%u", x0 + offset, y0, x1 + offset, y1, color565);
		}
		NEXTION_CMD(cmd);
	}
}

void ConvertProfiletoCoordinate(int nProfile, Point *ProfileCurve)
{
	// Time points
	float soakTemp = (nProfile == -1) ? ReflowParameters.SoakTemperature : ReflowParameters.profiles[nProfile].SoakTemperature;
	float firstHeatUpRate = (nProfile == -1) ? ReflowParameters.firstHeatUpRate : ReflowParameters.profiles[nProfile].firstHeatUpRate;
	float soakTime = (nProfile == -1) ? ReflowParameters.SoakTime : ReflowParameters.profiles[nProfile].SoakTime;
	float secondHeatUpRate = (nProfile == -1) ? ReflowParameters.secondHeatUpRate : ReflowParameters.profiles[nProfile].secondHeatUpRate;
	float reflowTemp = (nProfile == -1) ? ReflowParameters.ReflowTemperature : ReflowParameters.profiles[nProfile].ReflowTemperature;
	float reflowTime = (nProfile == -1) ? ReflowParameters.ReflowTime : ReflowParameters.profiles[nProfile].ReflowTime;

	float t0 = 0;
	float t1 = (soakTemp - 25) / firstHeatUpRate;
	float t2 = t1 + soakTime;
	float t3 = t2 + (reflowTemp - soakTemp) / secondHeatUpRate;
	float t4 = t3 + reflowTime;
	float t5 = t4 + (reflowTemp - 25) / 1.8f;

	float temp0 = 25;
	float temp1 = soakTemp;
	float temp2 = soakTemp;
	float temp3 = reflowTemp;
	float temp4 = reflowTemp;
	float temp5 = 25;

	ProfileCurve[0] = {t0, temp0};
	ProfileCurve[1] = {t1, temp1};
	ProfileCurve[2] = {t2, temp2};
	ProfileCurve[3] = {t3, temp3};
	ProfileCurve[4] = {t4, temp4};
	ProfileCurve[5] = {t5, temp5};
	/*
	for (int i = 0; i < 6; i++) {
		Serial.printf("P%d: t=%.1f, T=%.1f\n", i, ProfileCurve[i].time, ProfileCurve[i].temp);
	}
	*/
}

String GetProfileAsJSONString(int nProfile, Point curve[6])
{
	String json = "{\n";

	// Get profile name
	const char *name = "DEFAULT";
	if (nProfile >= 0)
	{
		name = ReflowParameters.profiles[nProfile].ProfileName;
	}

	json += "  \"name\": \"" + String(name) + "\",\n";
	json += "  \"points\": [\n";

	for (int i = 0; i < 6; i++)
	{
		json += "    {\"time\": " + String((int)curve[i].time) + ", \"temp\": " + String((int)curve[i].temp) + "}";
		if (i < 5)
			json += ",";
		json += "\n";
	}

	json += "  ]\n";
	json += "}";

	return json;
}

void Draw_Reflow_Curve()
{
	float dx = 0.41667f; // 275px / 660s
	float dy = 0.7143f;	 // 175px / 245°C
	const uint32_t OffsetX = 35;
	const uint32_t OffsetY = 240;

	// Time points
	float t0 = 0;
	float t1 = (ReflowParameters.SoakTemperature - 25) / ReflowParameters.firstHeatUpRate;
	float t2 = t1 + ReflowParameters.SoakTime;
	float t3 = t2 + (ReflowParameters.ReflowTemperature - ReflowParameters.SoakTemperature) / ReflowParameters.secondHeatUpRate;
	float t4 = t3 + ReflowParameters.ReflowTime;
	float t5 = t4 + (ReflowParameters.ReflowTemperature - 25) / 1.8;

	// Temperature points
	float temp0 = 25;
	float temp1 = ReflowParameters.SoakTemperature;
	float temp2 = ReflowParameters.SoakTemperature;
	float temp3 = ReflowParameters.ReflowTemperature;
	float temp4 = ReflowParameters.ReflowTemperature;
	float temp5 = 25;

	// Screen coordinate conversion
	uint32_t x0 = OffsetX + (uint32_t)(t0 * dx);
	uint32_t y0 = OffsetY - (uint32_t)(temp0 * dy);
	uint32_t x1 = OffsetX + (uint32_t)(t1 * dx);
	uint32_t y1 = OffsetY - (uint32_t)(temp1 * dy);
	uint32_t x2 = OffsetX + (uint32_t)(t2 * dx);
	uint32_t y2 = OffsetY - (uint32_t)(temp2 * dy);
	uint32_t x3 = OffsetX + (uint32_t)(t3 * dx);
	uint32_t y3 = OffsetY - (uint32_t)(temp3 * dy);
	uint32_t x4 = OffsetX + (uint32_t)(t4 * dx);
	uint32_t y4 = OffsetY - (uint32_t)(temp4 * dy);

	uint32_t x5 = OffsetX + (uint32_t)(t5 * dx);
	uint32_t y5 = OffsetY - (uint32_t)(temp0 * dy);

	// Draw line segments
	DrawThickLine(x0, y0, x1, y1, 3, 1311); // Preheat
	DrawThickLine(x1, y1, x2, y2, 3, 1311); // Soak
	DrawThickLine(x2, y2, x3, y3, 3, 1311); // Heat up
	DrawThickLine(x3, y3, x4, y4, 3, 1311); // Reflow
	DrawThickLine(x4, y4, x5, y5, 3, 1311); // Cooling (optional disable)
}

void Draw_Reflow_Curve2()
{
	float dx = 0.20833;	 // 275px / 660s / 500 ms
	float_t dy = 0.7143; // 175px / 245 degrees
	uint32_t OffsetX = 35;
	uint32_t OffsetY = 240;
	uint32_t index = 0;

	while (ReflowCurve[index] != 0)
	{
		NextionDrawDot(OffsetX + (uint32_t)((float)(index)*dx), OffsetY - (uint32_t)((float)(ReflowCurve[index]) * dy));
		index = index + 4;
	}
}
// —— Reflow Control ——
void startReflow()
{
	if (ReflowEnable)
	{
		return;
	}

	ReflowEnable = true;

	ServoEnable = false;
	myservo.write(0);
	ServoPos = 0;

	FanEnable = false;
	BuzzerEnable = false;

	RunningTimeHalfSeconds = 0;
	NEXTION_CMD("page 0");
	Draw_Reflow_Curve();
	TempDrawCounter = 0;
	Update_Page_0();
}

void stopReflow()
{
	if (ReflowEnable)
	{
		ReflowEnable = false;
		TempDrawEnable = false;

		ServoEnable = false;
		myservo.write(0);
		ServoPos = 0;

		FanEnable = false;
		BuzzerEnable = false;

		strcpy(ConsoleMSG, "STOPPED");
		strcpy(currentPhaseName, "IDLE");
		ledcWrite(SSR_PWM_CHANNEL, 0);
		Update_Page_0();
	}
	else
	{
		// if (ServoEnable){
		ServoEnable = false;
		myservo.write(0);
		ServoPos = 0;
		//}
		if (ReflowParameters.InvertedFan == 0)
			ledcWrite(FAN_PWM_CHANNEL, 0);
		else
			ledcWrite(FAN_PWM_CHANNEL, 255);
		ledcWrite(SSR_PWM_CHANNEL, 0);
		FanEnable = false;
		BuzzerEnable = false;
	}
}
#define MIN_VALID_TEMP -50.0f // Minimum valid temperature
#define MAX_VALID_TEMP 300.0f // Maximum valid temperature
// —— Scheduled Task ——
void ticker500ms()
{
	if (isUploadingTFT || isUploadingFirmware)
	{
		return;
	}

	TempDrawCounter++;
	temp = thermocouple.readCelsius();
	if (isnan(temp) || temp < MIN_VALID_TEMP || temp > MAX_VALID_TEMP)
	{
		stopReflow();
		strcpy(ConsoleMSG, "Temperature Reading Error ... STOPPED");
	}
	else
	{
		if (!ReflowEnable)
			start_temp = temp;
	}
	if (ReflowEnable && temp < start_temp + 8 && RunningTimeHalfSeconds > 180)
	{ // After heating for 2 minutes, still at 24 degrees
		stopReflow();
		strcpy(ConsoleMSG, "Temperature Reading Error ... STOPPED");
	}

	float maxChange = 10.0; // Maximum change of 10 degrees in 0.5 seconds

	if (fabs(temp - lastTemp) > maxChange && millis() > 3000)
	{
		// Limit rate of change instead of completely rejecting
		if (temp > lastTemp)
		{
			temp = lastTemp + maxChange; // Maximum rise of 10 degrees
		}
		else
		{
			temp = lastTemp - maxChange; // Maximum drop of 10 degrees
		}
	}
	lastTemp = temp;

	if (ReflowEnable && ReflowIndex < PhaseIndex[4])
	{
		RunningTimeHalfSeconds++;
		/*
		// Phase prompts
		if (ReflowIndex < PhaseIndex[1]) strcpy(ConsoleMSG, "PREHEAT");
		else if (ReflowIndex < PhaseIndex[2]) strcpy(ConsoleMSG, "SOAK");
		else if (ReflowIndex < PhaseIndex[3]) strcpy(ConsoleMSG, "HEAT UP");
		else if (ReflowIndex < PhaseIndex[4]) strcpy(ConsoleMSG, "REFLOW");
		*/
		if (ReflowIndex == PhaseIndex[0])
		{
			strcpy(currentPhaseName, "PREHEAT");
			sprintf(ConsoleMSG, "PREHEAT");
		}
		if (ReflowIndex == PhaseIndex[1])
		{
			strcpy(currentPhaseName, "SOAK");
			sprintf(ConsoleMSG, "SOAK");
		}
		if (ReflowIndex == PhaseIndex[2])
		{
			strcpy(currentPhaseName, "HEAT UP");
			sprintf(ConsoleMSG, "HEAT UP");
		}
		if (ReflowIndex == PhaseIndex[3])
		{
			strcpy(currentPhaseName, "REFLOW");
			sprintf(ConsoleMSG, "REFLOW");
		}

		// PID control
		float err = ReflowCurve[ReflowIndex] - temp;
		duty = arm_pid_f32(&PID, err);

		// Limit output
		if (duty > 1000)
		{
			duty = 1000;
			PID.Ki = 0;
		}
		else if (duty < 0)
		{
			duty = 0;
		}
		else
		{
			PID.Ki = ReflowParameters.Ki;
		}

		uint32_t pwm = map((int)duty, 0, 1000, 0, (1 << SSR_PWM_BITS) - 1);
		ledcWrite(SSR_PWM_CHANNEL, pwm);

		ReflowIndex++;

		if (ReflowIndex >= PhaseIndex[4])
		{
			strcpy(ConsoleMSG, "FINISHED, OPEN DOOR");
			strcpy(currentPhaseName, "COOLING");
			BuzzerEnable = true;
			ServoEnable = true;
			ServoPos = 0;
			TimerDoor = millis();

			FanEnable = true;
			TimerFAN = millis();

			ReflowEnable = false;
		}
	}
	else
	{
		ReflowIndex = 0;
		ledcWrite(SSR_PWM_CHANNEL, 0);
	}
}

void HandleGui()
{
	/*
		for (int i = 0; i < 5; i++) {
			Serial.print((char)UART_Received_Data[i]);  // ASCII character
			Serial.print(" (0x");
			Serial.print(UART_Received_Data[i], HEX);  // HEX value
			Serial.print(") ");
		}
		Serial.println();
		*/
	// ###################Page0##########################
	if (strncmp((char *)UART_Received_Data, "p0xxx", 5) == 0)
	{
		Update_Page_0();
	}

	if (strncmp((char *)UART_Received_Data, "p0b00", 5) == 0)
	{
		if (!ReflowEnable)
		{
			startReflow();
		}
		Update_Page_0();
	}

	if (strncmp((char *)UART_Received_Data, "p0b01", 5) == 0)
	{
		stopReflow();
		Update_Page_0();
	}
	if (strncmp((char *)UART_Received_Data, "p0b02", 5) == 0)
	{
		if (!ReflowEnable)
		{
			NEXTION_CMD("page 3");
			Update_Page_3();
		}
		else
			Update_Page_0();
	}

	// ###################Page2##########################

	if (strncmp((char *)UART_Received_Data, "p2xxx", 5) == 0)
	{
		Update_Page_2();
	}

	if (strncmp((char *)UART_Received_Data, "p2b00", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.KP;
		}

		ReflowParameters.KP = Output;
		PID.Kp = ReflowParameters.KP;
		arm_pid_init_f32(&PID, 1);

		NEXTION_CMD("page 2");
		Update_Page_2();
	}

	if (strncmp((char *)UART_Received_Data, "p2b01", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.Ki;
		}
		ReflowParameters.Ki = Output;
		PID.Ki = ReflowParameters.Ki;
		arm_pid_init_f32(&PID, 1);

		NEXTION_CMD("page 2");
		Update_Page_2();
	}

	if (strncmp((char *)UART_Received_Data, "p2b02", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.KD;
		}
		ReflowParameters.KD = Output;
		PID.Kd = ReflowParameters.KD;
		arm_pid_init_f32(&PID, 1);

		NEXTION_CMD("page 2");
		Update_Page_2();
	}

	if (strncmp((char *)UART_Received_Data, "p2b03", 5) == 0)
	{
		Update_Page_3();
		SaveReflowParameters();
	}

	if (strncmp((char *)UART_Received_Data, "p2b04", 5) == 0)
	{
		if (!ReflowEnable)
		{
			if (FanEnable == true)
			{
				FanEnable = false;
			}
			else
			{
				FanEnable = true;
			}
		}
		Clear_UART_Received_Data();
	}
	if (strncmp((char *)UART_Received_Data, "p2b05", 5) == 0)
	{
		if (!ReflowEnable)
		{
			// Move the servo from 0° to 180°
			for (int pos = 0; pos <= OPEN_POS; pos++)
			{
				myservo.write(pos); // Set the servo to the specified angle
				delay(15);			// Slight delay to allow the servo to move
			}

			delay(1000); // Wait for 1 second

			// Move the servo from 180° back to 0°
			for (int pos = OPEN_POS; pos >= 0; pos--)
			{
				myservo.write(pos); // Set the servo to the specified angle
				delay(15);			// Slight delay to allow the servo to move
			}

			delay(1000); // Wait for 1 second
		}
		Clear_UART_Received_Data(); // Clear the UART received data
	}

	if (strncmp((char *)UART_Received_Data, "p2b06", 5) == 0)
	{
		if (!ReflowEnable)
		{
			NEXTION_CMD("page 6");
			Update_Page_6();
		}
	}
	if (strncmp((char *)UART_Received_Data, "p2b07", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.FanSpeed;
		}
		ReflowParameters.FanSpeed = Output;

		NEXTION_CMD("page 2");
		Update_Page_2();
	}

	if (strncmp((char *)UART_Received_Data, "p2c11", 5) == 0)
	{
		ReflowParameters.InvertedFan = 1;
		Serial.printf("Inverted Fan Enabled %d\n", ReflowParameters.InvertedFan);
		Update_Page_2();
	}
	if (strncmp((char *)UART_Received_Data, "p2c10", 5) == 0)
	{
		ReflowParameters.InvertedFan = 0;
		Serial.printf("Inverted Fan Disabled %d\n", ReflowParameters.InvertedFan);
		Update_Page_2();
	}

	// ###################Page 3########################

	if (strncmp((char *)UART_Received_Data, "p3xxx", 5) == 0)
	{
		Update_Page_3();
	}

	if (strncmp((char *)UART_Received_Data, "p3b00", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.firstHeatUpRate;
		}

		if (Output < 0.2)
			Output = 0.2;
		if (Output > 1.5)
			Output = 1.5;

		ReflowParameters.firstHeatUpRate = Output;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	if (strncmp((char *)UART_Received_Data, "p3b01", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.SoakTime;
		}

		if (Output > 300)
			Output = 300;

		ReflowParameters.SoakTime = Output;

		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	if (strncmp((char *)UART_Received_Data, "p3b02", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.SoakTemperature;
		}

		if (Output < 30)
			Output = 30;
		if (Output > 240)
			Output = 240;

		ReflowParameters.SoakTemperature = Output;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	if (strncmp((char *)UART_Received_Data, "p3b03", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.secondHeatUpRate;
		}

		if (Output < 0.2)
			Output = 0.2;
		if (Output > 1.5)
			Output = 1.5;

		ReflowParameters.secondHeatUpRate = Output;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	if (strncmp((char *)UART_Received_Data, "p3b04", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.ReflowTime;
		}

		if (Output > 300)
			Output = 300;

		ReflowParameters.ReflowTime = Output;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	if (strncmp((char *)UART_Received_Data, "p3b05", 5) == 0)
	{
		float Output = 0;
		Output = HandleKeyPad();
		while (Output == 9999)
		{
			Clear_UART_Received_Data();
			Output = HandleKeyPad();
		}
		if (Output == 8888)
		{
			Output = ReflowParameters.ReflowTemperature;
		}
		if (Output < 30)
			Output = 30;
		if (Output > 240)
			Output = 240;
		ReflowParameters.ReflowTemperature = Output;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	if (strncmp((char *)UART_Received_Data, "p3b06", 5) == 0)
	{
		Update_Page_2();
	}

	if (strncmp((char *)UART_Received_Data, "p3b07", 5) == 0)
	{
		Update_Page_0();
		Draw_Reflow_Curve();
		SaveReflowParameters();
	}
	// Save current parameters to profile 0 (Lead 138°C)
	if (strncmp((char *)UART_Received_Data, "p3bs0", 5) == 0)
	{
		ReflowParameters.profiles[0].firstHeatUpRate = ReflowParameters.firstHeatUpRate;
		ReflowParameters.profiles[0].SoakTime = ReflowParameters.SoakTime;
		ReflowParameters.profiles[0].SoakTemperature = ReflowParameters.SoakTemperature;
		ReflowParameters.profiles[0].secondHeatUpRate = ReflowParameters.secondHeatUpRate;
		ReflowParameters.profiles[0].ReflowTime = ReflowParameters.ReflowTime;
		ReflowParameters.profiles[0].ReflowTemperature = ReflowParameters.ReflowTemperature;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	// Load profile 0 (Lead 138°C)
	if (strncmp((char *)UART_Received_Data, "p3b08", 5) == 0)
	{
		ReflowParameters.firstHeatUpRate = ReflowParameters.profiles[0].firstHeatUpRate;
		ReflowParameters.SoakTime = ReflowParameters.profiles[0].SoakTime;
		ReflowParameters.SoakTemperature = ReflowParameters.profiles[0].SoakTemperature;
		ReflowParameters.secondHeatUpRate = ReflowParameters.profiles[0].secondHeatUpRate;
		ReflowParameters.ReflowTime = ReflowParameters.profiles[0].ReflowTime;
		ReflowParameters.ReflowTemperature = ReflowParameters.profiles[0].ReflowTemperature;
		Update_Page_3();
		SaveReflowParameters();
		calculateReflowCurve();
	}

	// Save profile 1 (Lead 148°C)
	if (strncmp((char *)UART_Received_Data, "p3bs1", 5) == 0)
	{
		ReflowParameters.profiles[1].firstHeatUpRate = ReflowParameters.firstHeatUpRate;
		ReflowParameters.profiles[1].SoakTime = ReflowParameters.SoakTime;
		ReflowParameters.profiles[1].SoakTemperature = ReflowParameters.SoakTemperature;
		ReflowParameters.profiles[1].secondHeatUpRate = ReflowParameters.secondHeatUpRate;
		ReflowParameters.profiles[1].ReflowTime = ReflowParameters.ReflowTime;
		ReflowParameters.profiles[1].ReflowTemperature = ReflowParameters.ReflowTemperature;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	// Load profile 1 (Lead 148°C)
	if (strncmp((char *)UART_Received_Data, "p3b09", 5) == 0)
	{
		ReflowParameters.firstHeatUpRate = ReflowParameters.profiles[1].firstHeatUpRate;
		ReflowParameters.SoakTime = ReflowParameters.profiles[1].SoakTime;
		ReflowParameters.SoakTemperature = ReflowParameters.profiles[1].SoakTemperature;
		ReflowParameters.secondHeatUpRate = ReflowParameters.profiles[1].secondHeatUpRate;
		ReflowParameters.ReflowTime = ReflowParameters.profiles[1].ReflowTime;
		ReflowParameters.ReflowTemperature = ReflowParameters.profiles[1].ReflowTemperature;
		Update_Page_3();
		SaveReflowParameters();
		calculateReflowCurve();
	}

	// Save profile 2 (Lead 183°C)
	if (strncmp((char *)UART_Received_Data, "p3bs2", 5) == 0)
	{
		ReflowParameters.profiles[2].firstHeatUpRate = ReflowParameters.firstHeatUpRate;
		ReflowParameters.profiles[2].SoakTime = ReflowParameters.SoakTime;
		ReflowParameters.profiles[2].SoakTemperature = ReflowParameters.SoakTemperature;
		ReflowParameters.profiles[2].secondHeatUpRate = ReflowParameters.secondHeatUpRate;
		ReflowParameters.profiles[2].ReflowTime = ReflowParameters.ReflowTime;
		ReflowParameters.profiles[2].ReflowTemperature = ReflowParameters.ReflowTemperature;
		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	// Load profile 2 (Lead 183°C)
	if (strncmp((char *)UART_Received_Data, "p3b10", 5) == 0)
	{
		ReflowParameters.firstHeatUpRate = ReflowParameters.profiles[2].firstHeatUpRate;
		ReflowParameters.SoakTime = ReflowParameters.profiles[2].SoakTime;
		ReflowParameters.SoakTemperature = ReflowParameters.profiles[2].SoakTemperature;
		ReflowParameters.secondHeatUpRate = ReflowParameters.profiles[2].secondHeatUpRate;
		ReflowParameters.ReflowTime = ReflowParameters.profiles[2].ReflowTime;
		ReflowParameters.ReflowTemperature = ReflowParameters.profiles[2].ReflowTemperature;
		Update_Page_3();
		SaveReflowParameters();
		calculateReflowCurve();
	}

	// Save profile 3 (Lead 217°C)
	if (strncmp((char *)UART_Received_Data, "p3bs3", 5) == 0)
	{
		ReflowParameters.profiles[3].firstHeatUpRate = ReflowParameters.firstHeatUpRate;
		ReflowParameters.profiles[3].SoakTime = ReflowParameters.SoakTime;
		ReflowParameters.profiles[3].SoakTemperature = ReflowParameters.SoakTemperature;
		ReflowParameters.profiles[3].secondHeatUpRate = ReflowParameters.secondHeatUpRate;
		ReflowParameters.profiles[3].ReflowTime = ReflowParameters.ReflowTime;
		ReflowParameters.profiles[3].ReflowTemperature = ReflowParameters.ReflowTemperature;

		NEXTION_CMD("page 3");
		Update_Page_3();
		calculateReflowCurve();
	}

	// Load profile 3 (Lead 217°C)
	if (strncmp((char *)UART_Received_Data, "p3b11", 5) == 0)
	{
		ReflowParameters.firstHeatUpRate = ReflowParameters.profiles[3].firstHeatUpRate;
		ReflowParameters.SoakTime = ReflowParameters.profiles[3].SoakTime;
		ReflowParameters.SoakTemperature = ReflowParameters.profiles[3].SoakTemperature;
		ReflowParameters.secondHeatUpRate = ReflowParameters.profiles[3].secondHeatUpRate;
		ReflowParameters.ReflowTime = ReflowParameters.profiles[3].ReflowTime;
		ReflowParameters.ReflowTemperature = ReflowParameters.profiles[3].ReflowTemperature;
		Update_Page_3();
		SaveReflowParameters();
		calculateReflowCurve();
	}

	// Restore factory default configuration
	if (strncmp((char *)UART_Received_Data, "p3br7", 5) == 0)
	{
		ReflowParameters.profiles[3] = {0.75, 175, 100, 1.0, 240, 90};	// Lead 217°C
		ReflowParameters.profiles[2] = {0.75, 150, 100, 1.0, 230, 100}; // Lead 183°C
		ReflowParameters.profiles[1] = {0.75, 140, 100, 1.0, 175, 100}; // Lead 148°C
		ReflowParameters.profiles[0] = {0.75, 130, 100, 1.0, 165, 100}; // Lead 138°C

		// Set current parameters to profile 0 (Lead 138°C)
		ReflowParameters.firstHeatUpRate = 0.75;
		ReflowParameters.SoakTime = 100;
		ReflowParameters.SoakTemperature = 130;
		ReflowParameters.secondHeatUpRate = 1.0;
		ReflowParameters.ReflowTime = 100;
		ReflowParameters.ReflowTemperature = 165;

		calculateReflowCurve();
		Update_Page_0();
	}

	// Restore factory default profiles (not all tested)
	if (strncmp((char *)UART_Received_Data, "p3br7", 5) == 0)
	{
		// Restore default four reflow profiles [0 ~ 3]

		// Lead 217°C – profiles[3] – lead-free
		ReflowParameters.profiles[3].firstHeatUpRate = 0.75f;
		ReflowParameters.profiles[3].SoakTemperature = 175;
		ReflowParameters.profiles[3].SoakTime = 100;
		ReflowParameters.profiles[3].secondHeatUpRate = 1.0f;
		ReflowParameters.profiles[3].ReflowTemperature = 240;
		ReflowParameters.profiles[3].ReflowTime = 90;

		// Lead 183°C – profiles[2] – lead-free standard solder paste
		ReflowParameters.profiles[2].firstHeatUpRate = 0.75f;
		ReflowParameters.profiles[2].SoakTemperature = 150;
		ReflowParameters.profiles[2].SoakTime = 100;
		ReflowParameters.profiles[2].secondHeatUpRate = 1.0f;
		ReflowParameters.profiles[2].ReflowTemperature = 230;
		ReflowParameters.profiles[2].ReflowTime = 100;

		// Lead 148°C – profiles[1]
		ReflowParameters.profiles[1].firstHeatUpRate = 0.75f;
		ReflowParameters.profiles[1].SoakTemperature = 140;
		ReflowParameters.profiles[1].SoakTime = 100;
		ReflowParameters.profiles[1].secondHeatUpRate = 1.0f;
		ReflowParameters.profiles[1].ReflowTemperature = 175;
		ReflowParameters.profiles[1].ReflowTime = 100;

		// Lead 138°C – profiles[0] – ultra-low temperature solder paste
		ReflowParameters.profiles[0].firstHeatUpRate = 0.75f;
		ReflowParameters.profiles[0].SoakTemperature = 130;
		ReflowParameters.profiles[0].SoakTime = 100;
		ReflowParameters.profiles[0].secondHeatUpRate = 1.0f;
		ReflowParameters.profiles[0].ReflowTemperature = 165;
		ReflowParameters.profiles[0].ReflowTime = 100;

		// Set current parameters to profile 0 (default 138°C curve)
		ReflowParameters.firstHeatUpRate = ReflowParameters.profiles[0].firstHeatUpRate;
		ReflowParameters.SoakTemperature = ReflowParameters.profiles[0].SoakTemperature;
		ReflowParameters.SoakTime = ReflowParameters.profiles[0].SoakTime;
		ReflowParameters.secondHeatUpRate = ReflowParameters.profiles[0].secondHeatUpRate;
		ReflowParameters.ReflowTemperature = ReflowParameters.profiles[0].ReflowTemperature;
		ReflowParameters.ReflowTime = ReflowParameters.profiles[0].ReflowTime;

		// Update display and reflow curve
		calculateReflowCurve();
		Update_Page_0();
	}

	// ################### Page 3 ########################
	if (strncmp((char *)UART_Received_Data, "p6b02", 5) == 0)
	{
		ResetSettingsToDefault();
		Clear_UART_Received_Data();
		RebootEnable = true;
	}

	if (strncmp((char *)UART_Received_Data, "p6b01", 5) == 0)
	{
		NEXTION_CMD("page 2");
		Update_Page_2();
		Serial.println("p6b01");
	}
}

AsyncWebServer server(80);

// 获取 MIME 类型
String getContentType(const String &path)
{
	if (path.endsWith(".html"))
		return "text/html";
	if (path.endsWith(".css"))
		return "text/css";
	if (path.endsWith(".js"))
		return "application/javascript";
	if (path.endsWith(".json"))
		return "application/json";
	if (path.endsWith(".ico"))
		return "image/x-icon";
	if (path.endsWith(".png"))
		return "image/png";
	if (path.endsWith(".svg"))
		return "image/svg+xml";
	if (path.endsWith(".jpg") || path.endsWith(".jpeg"))
		return "image/jpeg";
	if (path.endsWith(".txt"))
		return "text/plain";
	return "application/octet-stream";
}

bool handleStaticRequest(AsyncWebServerRequest *request, const String &uri)
{
	for (int i = 0; i < webFilesCount; ++i)
	{
		if (uri == webFiles[i].path)
		{
			request->send(200, getContentType(uri).c_str(), webFiles[i].content, webFiles[i].size);
			return true;
		}
	}

	if (uri == "/" || uri == "/index")
	{
		return handleStaticRequest(request, "/index.html");
	}
	return false;
}

void notFoundHandler(AsyncWebServerRequest *request)
{
	if (!handleStaticRequest(request, request->url()))
	{
		request->send(404, "text/plain", "404 Not Found");
	}
}

void handleFirmwareUpload(AsyncWebServerRequest *request, String filename,
						  size_t index, uint8_t *data, size_t len, bool final)
{
	static size_t totalSize = 0;

	if (!isUploadingFirmware)
		stopReflow();

	isUploadingFirmware = true;

	if (index == 0)
	{
		Serial.printf("UploadStart: %s\n", filename.c_str());
		totalSize = request->contentLength();

		if (!Update.begin(UPDATE_SIZE_UNKNOWN))
		{
			Update.printError(Serial);
		}
	}

	if (Update.write(data, len) != len)
	{
		Update.printError(Serial);
	}

	float percent = ((index + len) * 100.0) / totalSize;
	Serial.printf("Upload Progress: %.2f%%\n", percent);

	if (final)
	{
		if (Update.end(true))
		{
			Serial.println("Update Success, restarting...");
			request->send(200, "text/plain", "OK"); // Changed to plain text "OK"
			RebootEnable = true;
		}
		else
		{
			Update.printError(Serial);
			request->send(500, "text/plain", "FAILED"); // Changed to plain text
		}
	}
	isUploadingFirmware = false;
}

void handleTempsRequest(AsyncWebServerRequest *request)
{
	String json = "{";
	json += "\"current\":" + String(temp, 1) + ",";
	if (ReflowEnable)
		json += "\"time\":" + String(ReflowIndex) + ",";
	else
		json += "\"time\": 0 ,";
	json += "\"running\":" + String(ReflowEnable ? "true" : "false") + ",";
	json += "\"phase\":\"" + String(currentPhaseName) + "\"";
	json += "}";

	request->send(200, "application/json", json);
}
#define POINTS_PER_PROFILE 6
void handleProfilesRequest(AsyncWebServerRequest *request)
{
	String json = "[";
	for (int i = -1; i < MAX_REFLOW_PROFILES; i++)
	{
		Point curve[POINTS_PER_PROFILE];
		String profileName;
		ConvertProfiletoCoordinate(i, curve);
		if (i == -1)
			profileName = "DEFAULT";
		else
		{
			profileName = ReflowParameters.profiles[i].ProfileName;
			Serial.print(i);
			Serial.print(":");
			Serial.println(profileName);
		}

		json += "\r\n{\"name\":\"" + profileName + "\",\"points\":[";
		for (int j = 0; j < POINTS_PER_PROFILE; j++)
		{
			json += "{\"time\":" + String(curve[j].time, 0) + ",\"temp\":" + String(curve[j].temp, 0) + "}";
			if (j < POINTS_PER_PROFILE - 1)
				json += ",";
		}
		json += "]}";
		if (i < MAX_REFLOW_PROFILES - 1)
			json += ",";
	}
	json += "]";

	request->send(200, "application/json", "{\"profiles\":" + json + "}");
}

void handleSelectProfileRequest(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
	StaticJsonDocument<200> doc;
	DeserializationError error = deserializeJson(doc, data);

	if (error)
	{
		request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
		return;
	}

	if (!doc.containsKey("nProfile"))
	{
		request->send(400, "application/json", "{\"error\":\"Missing nProfile\"}");
		return;
	}

	int selected_profile = int(doc["nProfile"]) - 1;
	Serial.printf("Selected profile set to: %d\n", selected_profile);
	if (selected_profile != -1)
	{
		ReflowParameters.firstHeatUpRate = ReflowParameters.profiles[selected_profile].firstHeatUpRate;
		ReflowParameters.SoakTime = ReflowParameters.profiles[selected_profile].SoakTime;
		ReflowParameters.SoakTemperature = ReflowParameters.profiles[selected_profile].SoakTemperature;
		ReflowParameters.secondHeatUpRate = ReflowParameters.profiles[selected_profile].secondHeatUpRate;
		ReflowParameters.ReflowTime = ReflowParameters.profiles[selected_profile].ReflowTime;
		ReflowParameters.ReflowTemperature = ReflowParameters.profiles[selected_profile].ReflowTemperature;
		calculateReflowCurve();
		if (strncmp((char *)UART_Received_Data, "p0xxx", 5) == 0)
		{
			NEXTION_CMD("page 0");
			Update_Page_0();
			Draw_Reflow_Curve();
		}
	}
	request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleScan(AsyncWebServerRequest *request)
{
	int n = WiFi.scanNetworks();
	String ssidsJson = "[";
	for (int i = 0; i < n; i++)
	{
		ssidsJson += "\"" + WiFi.SSID(i) + "\"";
		if (i < n - 1)
			ssidsJson += ",";
	}
	ssidsJson += "]";
	request->send(200, "application/json", ssidsJson);
}

void startAP()
{
	isConfiguring = true;
	WiFi.mode(WIFI_AP_STA);
	WiFi.softAP("SmartReflowOven");
	Serial.print("AP mode started, IP: ");
	Serial.println(WiFi.softAPIP());

	server.on("/", handleRoot);
	server.on("/scan", handleScan);
	server.begin();
	Serial.println("Config WebServer started in AP mode");
}
void connectSTA()
{
	String msg;
	Serial.println("Starting WiFi connection process...");
	Serial.printf("Saved SSID: %s\n", staSSID.c_str());

	// If no saved SSID, start AP mode for configuration
	if (staSSID.length() == 0)
	{
		Serial.println("No SSID configured, starting AP mode for configuration");
		NEXTION_CMD("page 4");
		startAP();
		return;
	}

	NEXTION_SendString("t0", VERSION);

	Serial.println("Attempting STA connection...");
	NEXTION_SendStringCat("t1", "Attempting STA connection...\n");

	WiFi.mode(WIFI_STA);
	// WiFi.disconnect();  // Clear previous connection state
	esp_wifi_start();			   // Start WiFi driver
	esp_wifi_set_max_tx_power(84); // Max TX power 21 dBm
	esp_wifi_set_ps(WIFI_PS_NONE); // Disable power save, maintain high receive power

	IPAddress ip, mask, gw, dns;

	// Configure static IP if provided
	if (staIP.length() > 0)
	{
		if (ip.fromString(staIP))
		{
			mask.fromString(staMASK.length() > 0 ? staMASK : "255.255.255.0");
			gw.fromString(staGW.length() > 0 ? staGW : "192.168.1.1");
			dns.fromString(staDNS.length() > 0 ? staDNS : "8.8.8.8");

			if (WiFi.config(ip, gw, mask, dns))
			{
				Serial.printf("Static IP configured: %s\n", staIP.c_str());
				msg = "Static IP configured: " + staIP + "\n";
				NEXTION_SendStringCat("t1", msg.c_str());
			}
			else
			{
				Serial.println("Failed to configure static IP, using DHCP");
			}
		}
		else
		{
			Serial.println("Invalid IP format, using DHCP");
		}
	}
	else
	{
		Serial.println("Using DHCP");
		NEXTION_SendStringCat("t1", "Using DHCP\n");
	}

	WiFi.begin(staSSID.c_str(), staPASS.c_str());
	Serial.printf("Connecting to %s", staSSID.c_str());
	msg = "Connecting to " + staSSID + "\n";
	NEXTION_SendStringCat("t1", msg.c_str());

	unsigned long startAttempt = millis();
	int dotCount = 0;

	while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 30000)
	{
		delay(500);
		Serial.print(".");
		dotCount++;
		// New line every 10 dots for clarity
		if (dotCount % 10 == 0)
		{
			Serial.println();
		}

		// Check for fatal errors
		if (WiFi.status() == WL_CONNECT_FAILED || WiFi.status() == WL_NO_SSID_AVAIL)
		{
			Serial.printf("\nConnection failed with status: %d\n", WiFi.status());
			msg = "Error Code: " + String(WiFi.status()) + "\n";
			NEXTION_SendStringCat("t1", msg.c_str());
			delay(1000);
			break;
		}
	}

	Serial.println();

	if (WiFi.status() == WL_CONNECTED)
	{
		Serial.println("WiFi connected successfully!");
		Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
		Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
		Serial.printf("Signal: %d dBm\n", WiFi.RSSI());
		msg = "Signal: " + String(WiFi.RSSI()) + " dBm\n";
		NEXTION_SendStringCat("t1", msg.c_str());
		NEXTION_SendStringCat("t1", WiFi.localIP().toString().c_str());
		IPAddr = WiFi.localIP().toString();
	}
	else
	{
		Serial.printf("Connection failed (Status: %d), starting AP mode\n", WiFi.status());
		NEXTION_CMD("page 4");
		startAP(); // Only start AP mode if connection fails
	}
}

String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Smart Reflow Oven Config</title>
  <style>
    body {font-family:sans-serif; max-width:400px; margin:auto; padding:20px;}
    input, select, button {width:100%; padding:8px; margin:5px 0; box-sizing:border-box; border:1px solid #ccc;}
    input:focus {border-color:#007BFF; outline:none;}
    button {background:#007BFF; color:#fff; border:none; border-radius:4px; cursor:pointer;}
    button:disabled {background:#6c757d; cursor:not-allowed; opacity:0.6;}
    h1, h2 {text-align:center;}
    .form-group {margin-bottom:10px;}
    label {display:block; margin-bottom:3px; font-weight:bold;}
  </style>
  <script>
    function scanWiFi() {
      const ssidSelect = document.getElementById('ssid');
      ssidSelect.innerHTML = '<option>Scanning...</option>';
      validateForm(); // Disable submit while scanning
      
      fetch('/scan')
        .then(resp => resp.json())
        .then(data => {
          ssidSelect.innerHTML = '';
          if(data.length === 0) {
            ssidSelect.innerHTML = '<option>No networks found</option>';
          } else {
            data.forEach(ssid => {
              let opt = document.createElement('option');
              opt.text = ssid;
              ssidSelect.add(opt);
            });
          }
          validateForm(); // Re-enable after scan
        })
        .catch(() => {
          ssidSelect.innerHTML = '<option>Error scanning WiFi</option>';
          validateForm(); // Re-enable after error
        });
    }

    function isValidIP(ip) {
      const ipRegex = /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
      return ipRegex.test(ip.trim());
    }

    function validateForm() {
      const ssidSelect = document.getElementById('ssid');
      const passwordInput = document.getElementById('password');
      const ipInput = document.getElementById('ip');
      const maskInput = document.getElementById('mask');
      const gwInput = document.getElementById('gw');
      const dnsInput = document.getElementById('dns');
      const submitBtn = document.getElementById('submitBtn');
      
      const selectedSSID = ssidSelect.value;
      const password = passwordInput.value.trim();
      const ip = ipInput.value.trim();
      const mask = maskInput.value.trim();
      const gateway = gwInput.value.trim();
      const dns = dnsInput.value.trim();
      
      const invalidSSIDs = ['Scanning...', 'No networks found', 'Error scanning WiFi', '-- Please scan WiFi --'];
      const isSSIDValid = selectedSSID && !invalidSSIDs.includes(selectedSSID);
      const isPasswordValid = password.length > 0;
      const isIPValid = isValidIP(ip);
      const isMaskValid = isValidIP(mask);
      const isGatewayValid = isValidIP(gateway);
      const isDNSValid = isValidIP(dns);
      
      const allValid = isSSIDValid && isPasswordValid && isIPValid && isMaskValid && isGatewayValid && isDNSValid;
      submitBtn.disabled = !allValid;
      
      [ipInput, maskInput, gwInput, dnsInput].forEach((input, index) => {
        const isValid = [isIPValid, isMaskValid, isGatewayValid, isDNSValid][index];
        if (input.value.trim() && !isValid) {
          input.style.borderColor = '#dc3545';
          input.style.borderWidth = '2px';
        } else {
          input.style.borderColor = '';
          input.style.borderWidth = '';
        }
      });
    }

    window.onload = function() {
      document.getElementById('ssid').addEventListener('change', validateForm);
      document.getElementById('password').addEventListener('input', validateForm);
      document.getElementById('ip').addEventListener('input', validateForm);
      document.getElementById('mask').addEventListener('input', validateForm);
      document.getElementById('gw').addEventListener('input', validateForm);
      document.getElementById('dns').addEventListener('input', validateForm);
      validateForm(); // Initial validation
    }
  </script>
</head>
<body>
  <h1>Smart Reflow Oven</h1>
  <h2>WiFi & Network Settings</h2>
  <form method="POST">
    <div class="form-group">
      <label for="ssid">SSID:</label>
      <select id="ssid" name="ssid">
        <option>-- Please scan WiFi --</option>
      </select>
      <button type="button" onclick="scanWiFi()">Scan WiFi</button>
    </div>
    
    <div class="form-group">
      <label for="password">Password:</label>
      <input id="password" name="pass" type="password" placeholder="WiFi Password">
    </div>
    
    <div class="form-group">
      <label for="ip">IP:</label>
      <input id="ip" name="ip" value=")rawliteral" +
			  (staIP.length() ? staIP : "192.168.1.170") + R"rawliteral(" placeholder="Static IP">
    </div>
    
    <div class="form-group">
      <label for="mask">Subnet Mask:</label>
      <input id="mask" name="mask" value=")rawliteral" +
			  (staMASK.length() ? staMASK : "255.255.255.0") + R"rawliteral(" placeholder="Subnet Mask">
    </div>
    
    <div class="form-group">
      <label for="gw">Gateway:</label>
      <input id="gw" name="gw" value=")rawliteral" +
			  (staGW.length() ? staGW : "192.168.1.1") + R"rawliteral(" placeholder="Gateway">
    </div>
    
    <div class="form-group">
      <label for="dns">DNS:</label>
      <input id="dns" name="dns" value=")rawliteral" +
			  (staDNS.length() ? staDNS : "192.168.1.1") + R"rawliteral(" placeholder="DNS">
    </div>
    
    <button id="submitBtn" type="submit" disabled>Save & Reboot</button>
  </form>
</body>
</html>
)rawliteral";

void handleRoot(AsyncWebServerRequest *request)
{
	if (request->methodToString() == "POST")
	{
		if (request->hasParam("ssid", true))
			staSSID = request->getParam("ssid", true)->value();
		if (request->hasParam("pass", true))
			staPASS = request->getParam("pass", true)->value();
		if (request->hasParam("ip", true))
			staIP = request->getParam("ip", true)->value();
		if (request->hasParam("mask", true))
			staMASK = request->getParam("mask", true)->value();
		if (request->hasParam("gw", true))
			staGW = request->getParam("gw", true)->value();
		if (request->hasParam("dns", true))
			staDNS = request->getParam("dns", true)->value();

		SaveReflowParameters();

		String newIP = staIP.length() ? staIP : "192.168.1.170";
		String htmlResp =
			"<html><head><title>Smart Reflow Oven</title></head>"
			"<body style='text-align:center;'>"
			"<h2>Settings Saved</h2>"
			"<p>Device will reboot in 3 seconds...</p>"
			"<script>"
			"setTimeout(()=>{ window.location.href='http://" +
			newIP + "/'; }, 3000);"
					"</script>"
					"</body></html>";
		request->send(200, "text/html", htmlResp);
		RebootEnable = true;
		return;
	}
	request->send(200, "text/html", html); // Serve config page
}

void testServo()
{
	// Move servo from 0° to 180°
	for (int pos = 0; pos <= 180; pos++)
	{
		myservo.write(pos); // Set servo to position
		delay(15);			// Small delay for movement
	}
	delay(1000); // Wait 1 second

	// Move servo back from 180° to 0°
	for (int pos = 180; pos >= 0; pos--)
	{
		myservo.write(pos); // Set servo to position
		delay(15);			// Small delay for movement
	}
	delay(1000); // Wait 1 second
}

void setup()
{
	esp_task_wdt_init(30, true);
	Serial.begin(115200);

	// Load configuration parameters
	if (!loadReflowParameters())
	{
		Serial.println("Loading default parameters...");
		initDefaultParams();
		SaveReflowParameters();
	}
	else
	{
		Serial.println("Parameters loaded from memory");
	}

	pinMode(FAN_PIN, OUTPUT);
	if (ReflowParameters.InvertedFan == 0)
		digitalWrite(FAN_PIN, LOW); // Fan always ON (low level)
	else
		digitalWrite(FAN_PIN, HIGH); // Fan default OFF (high level)

	// Fan PWM setup
	ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQUENCY, FAN_PWM_BITS);
	ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);

	if (ReflowParameters.InvertedFan == 0)
		ledcWrite(FAN_PWM_CHANNEL, 0); // Duty cycle 0, fan ON
	else
		ledcWrite(FAN_PWM_CHANNEL, 255); // Duty cycle 255, fan OFF

	// Initialize serial communication with Nextion
	nextion.begin(DEFAULT_SPEED, SERIAL_8N1, RX_PIN, TX_PIN);
	// nextion.setTimeout(1000);  // Set timeout to 1000ms
	// nextion.setTxBufferSize(1024);
	// nextion.setRxBufferSize(1024);
	while (nextion.read() >= 0)
		; // Clear serial buffer

	nextion.onReceive(onUartReceive);
	delay(1000);
	NEXTION_CMD("page 5");

	// Initialize pins
	pinMode(SSR1, OUTPUT);
	digitalWrite(SSR1, LOW);

	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);

	ServoEnable = false;

	int servoChannel = myservo.attach(SERVO_PIN);
	Serial.printf("Servo attached to channel %d\n", servoChannel);

	// SSR PWM setup
	ledcSetup(SSR_PWM_CHANNEL, SSR_PWM_FREQ, SSR_PWM_BITS);
	ledcAttachPin(SSR1, SSR_PWM_CHANNEL);
	ledcWrite(SSR_PWM_CHANNEL, 0); // Duty cycle 0, SSR OFF

	connectSTA();

	if (isConfiguring)
		return;

	NEXTION_SendStringCat("t1", "Checking update ...");
	checkForUpdates();
	delay(1000);
	NEXTION_CMD("page 0");

	calculateReflowCurve();

	// Start timer
	ticker.attach(0.5, ticker500ms);

	Update_Page_0();
	Draw_Reflow_Curve();

	// Initialize PID controller
	PID.Kp = ReflowParameters.KP;
	PID.Ki = ReflowParameters.Ki;
	PID.Kd = ReflowParameters.KD;
	arm_pid_init_f32(&PID, 1);

	server.onNotFound(notFoundHandler);

	// HTTP endpoints
	server.on("/prepare/status", HTTP_GET, [](AsyncWebServerRequest *request)
			  {
        String s="IDLE";
        switch(prepareState){
            case PREP_IDLE: s="IDLE"; break;
            case PREP_IN_PROGRESS: s="IN_PROGRESS"; break;
            case PREP_DONE: s="DONE"; break;
            case PREP_ERROR: s="ERROR"; break;
        }
        request->send(200,"application/json","{\"state\":\""+s+"\"}"); });

	server.on("/prepare", HTTP_GET, [](AsyncWebServerRequest *request)
			  {
        if(!request->hasParam("size")){
            request->send(400,"text/plain","Missing size parameter");
            return;
        }
        size_t size = request->getParam("size")->value().toInt();
        isUploadingTFT = true;
        stopReflow();
        ticker.detach();
        startPrepare(size);
        request->send(200,"text/plain","Preparing screen..."); });

	server.on("/upload", HTTP_POST,
			  // Request completion handler
			  [](AsyncWebServerRequest *request)
			  {
            if (upgradeState == UPGRADE_COMPLETE && totalReceived == expectedFileSize)
                request->send(200, "text/plain", "File uploaded successfully, screen upgrade completed");
            else
                request->send(500, "text/plain", "Upload error");

            upgradeState = UPGRADE_IDLE;
            totalReceived = 0;
            expectedFileSize = 0;
            currentPacketSize = 0; },
			  // Upload data handler
			  [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
			  {
            if(index==0 && upgradeState!=UPGRADE_READY) {
                Serial.println("Screen not ready");
                request->send(400,"text/plain","Screen not ready");
                upgradeState = UPGRADE_IDLE;
                totalReceived = 0;
                currentPacketSize = 0;
                return;
            }

            upgradeState = UPGRADE_TRANSFERRING;

            for(size_t i=0;i<len;i++){
                packetBuffer[currentPacketSize]=data[i];
                currentPacketSize++;
                totalReceived++;

                if (currentPacketSize == NEXTION_PACKET_SIZE || (final && i == len - 1)) {
                    size_t packetNumber = (totalReceived - currentPacketSize) / NEXTION_PACKET_SIZE + 1;
                    float progress = (float)totalReceived / expectedFileSize * 100;

                    Serial.printf("\n=== Packet #%u ===\n", packetNumber);
                    Serial.printf("Size: %u bytes\n", currentPacketSize);
                    Serial.printf("Progress: %.1f%% (%u/%u)\n", progress, totalReceived, expectedFileSize);

                    // Send packet with retry
                    if (!sendPacket(packetBuffer, currentPacketSize, 3)) {
                        Serial.printf("✗ Packet #%u failed, stopping transfer\n", packetNumber);
                        upgradeState = UPGRADE_ERROR;
                        return;
                    }

                    Serial.printf("✓ Packet #%u sent successfully\n", packetNumber);

                    currentPacketSize = 0;
                    lastProgressTime = millis();
                }
            }

            if(final){
                upgradeState=UPGRADE_COMPLETE;
                Serial.println("File transfer completed");
                delay(2000);
                Serial.println("Rebooting...");
                RebootEnable = true;
            } });

	server.on("/getVersion", HTTP_GET, [](AsyncWebServerRequest *request)
			  { request->send(200, "application/json", String("{\"version\":\"") + VERSION + "\"}"); });

	server.on("/temps", HTTP_GET, handleTempsRequest);
	server.on("/profiles", HTTP_GET, handleProfilesRequest);

	server.on("/uploadFirmware", HTTP_POST, [](AsyncWebServerRequest *request)
			  { request->send(Update.hasError() ? 500 : 200, "text/html", (Update.hasError()) ? "FAIL" : "OK"); }, handleFirmwareUpload);

	server.on("/getPID", HTTP_GET, [](AsyncWebServerRequest *request)
			  {
        String json = "{";
        json += "\"p\":" + String(ReflowParameters.KP, 2) + ",";
        json += "\"i\":" + String(ReflowParameters.Ki, 2) + ",";
        json += "\"d\":" + String(ReflowParameters.KD, 2);
        json += "}";
        request->send(200, "application/json", json); });

	server.on("/start", HTTP_POST, [](AsyncWebServerRequest *request)
			  {
        if (!ReflowEnable) {
            startReflow();
            request->send(200, "application/json", "{\"status\":\"started\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Already running\"}");
        } });

	server.on("/stop", HTTP_POST, [](AsyncWebServerRequest *request)
			  {
        if (ReflowEnable) {
            stopReflow();
            request->send(200, "application/json", "{\"status\":\"stopped\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Already stopped\"}");
        } });

	server.on("/setPID", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
			  {
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, data);

            if (error)
            {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
            }

            if (!doc.containsKey("p") || !doc.containsKey("i") || !doc.containsKey("d"))
            {
                request->send(400, "application/json", "{\"error\":\"Missing keys\"}");
                return;
            }

            ReflowParameters.KP = doc["p"];
            ReflowParameters.Ki = doc["i"];
            ReflowParameters.KD = doc["d"];
            SaveReflowParameters();

            request->send(200, "application/json", "{\"success\":\"true\"}"); });

	server.on("/selectProfile", HTTP_POST, [](AsyncWebServerRequest *request) {}, nullptr, handleSelectProfileRequest);

	server.begin();

	Serial.println("Setup completed!");
}

void beepBeep()
{
	if (BuzzerEnable)
	{
		unsigned long now = millis();
		if (digitalRead(BUZZER_PIN) && now - TimerBUZZER > 100)
		{
			Serial.println(2);
			TimerBUZZER = now;
			digitalWrite(BUZZER_PIN, LOW);
		}
		else if (!digitalRead(BUZZER_PIN) && now - TimerBUZZER > 100 && beep < 2)
		{
			Serial.println(3);
			TimerBUZZER = now;
			digitalWrite(BUZZER_PIN, HIGH);
			beep++;
		}
		else if (now - TimerBUZZER > 3000)
		{
			Serial.println(4);
			beep = 0;
		}
	}
}

void fanControl()
{
	if (FanEnable)
	{
		if (ReflowParameters.InvertedFan == 0)
			ledcWrite(FAN_PWM_CHANNEL, ReflowParameters.FanSpeed);
		else
			ledcWrite(FAN_PWM_CHANNEL, 255 - ReflowParameters.FanSpeed);
		unsigned long now = millis();
		if (now - TimerFAN > 1000 * 60 * FAN_ON_DURATION)
		{
			if (ReflowParameters.InvertedFan == 0)
				ledcWrite(FAN_PWM_CHANNEL, 0);
			else
				ledcWrite(FAN_PWM_CHANNEL, 255);

			FanEnable = false;
		}
	}
	else
	{
		if (ReflowParameters.InvertedFan == 0)
			ledcWrite(FAN_PWM_CHANNEL, 0);
		else
			ledcWrite(FAN_PWM_CHANNEL, 255);
	}
}

void doorControl()
{
	if (ServoEnable)
	{
		unsigned long now = millis();
		if (now - TimerDoor > 10)
		{
			myservo.write(ServoPos++);
			TimerDoor = now;
		}
		if (ServoPos >= OPEN_POS)
		{
			ServoEnable = false;
		}
	}
}

void loop()
{
	static unsigned long lastGuiUpdate = 0;
	if (!isConfiguring && !isUploadingTFT && !isUploadingFirmware)
	{
		if (millis() - lastGuiUpdate > 505)
		{
			lastGuiUpdate = millis();
			HandleGui();
		}

		beepBeep();
		doorControl();
		fanControl();
	}
	if (RebootEnable)
	{
		delay(1000);
		ESP.restart();
	}
	if (!ReflowEnable)
		ledcWrite(SSR_PWM_CHANNEL, 0);
	delay(10);
}
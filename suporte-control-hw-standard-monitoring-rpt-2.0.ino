#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>
#include <pthread.h>
#include <PubSubClient.h>
#define E32_TTL_1W
#include <LoRa_E32.h>
#include <MCP23017.h>
#include <ModbusMaster.h>

#include <Dns.h>       // DNSClient
#include <Update.h>
#include "esp_ota_ops.h"
#include <RTClib.h>
#include <Firebase_ESP_Client.h>  // Firebase
#include "addons/TokenHelper.h"   // Firebase Provide the token generation process info.

//BLUETOOTH
#include <NimBLEDevice.h>  // BLE

//MEMÓRIA
#include <EEPROM.h>

//To DateTime Network
#include <NTPClient.h>

using namespace std;

//To RS485
#include <SoftwareSerial.h>

//To PCF8574
#include <PCF8574.h>
#include <Wire.h>

//To Ebyte
#define pinToM0_Ebyte GPIO_NUM_4
#define pinToM1_Ebyte GPIO_NUM_17
#define pinToRX_Ebyte GPIO_NUM_25
#define pinToTX_Ebyte GPIO_NUM_26
#define pinToAux_Ebyte GPIO_NUM_34

//#define OUT GPIO_NUM_19

//To Config
#define CONFIG GPIO_NUM_16

//To Input
#define IN0 0
#define IN1 1
#define IN2 2

//To RS 485
#define pinTo_RE_DE_485 GPIO_NUM_33
#define pinTo_RO_485 GPIO_NUM_27
#define pinTo_DI_485 GPIO_NUM_32

#define DI6 39  // I7
#define DI7 36  // I8

#define ETH_CS 13
#define ETH_RST 14

#define ETH_SCK 18
#define ETH_MISO 19
#define ETH_MOSI 23

//To LED
#define pinToLED01 GPIO_NUM_2   //LED

// To serial
#define SERIAL_FREQ 115200
#define SERIAL_9600 57600

String _DEVICE_NAME_BLE = "MÓDULO SC-RPT V1.0";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

#define EEPROM_SIZE 1024

#define MCP23017_ADDR 0x20
MCP23017 mcp = MCP23017(MCP23017_ADDR);

bool _BIT_OUT[8];
bool _BIT_IN[8];

#define _OUT_ 0

SemaphoreHandle_t mcpLock = NULL;

String _SENSOR_NAME = "";
String _SENSOR_VALUE_UTC = "";

char ntk[128];
char psw[128];
char tableDB[128];
char deviceDB[128];

String _READ_LORA = "";

const char* ntpServer = "pool.ntp.org";

int seg = 0, minuto = 0, hora = 0, dia = 0, diaSemana = 0, mes = 0, ano = 0;

int startRushHour[2];
int endRushHour[2];
bool enableRushHour = false;

TaskHandle_t thLedUpdateHandle = NULL;

void setLED(void* arg);
void mcpRegister(void* arg);
void confirmDevice(void* arg);
static bool readLineFromUart(Stream& uart, char* buf, uint16_t bufSize, uint32_t timeoutMs);;
void checkUpdateOnFirebaseWifi();
void readE32();
void tmt_CODE();
int schedulingByAngle[100];            //(AGENDAMENTO POR ANGULO)
int schedulingTime[100][2];            //HORARIO
char schedulingDWeek[100][7];          //DIAS DA SEMANA
char schedulingOnOff[100];             //ACIONAR/PARAR
char schedulingEnabled[100];           //Habilitado/Desabilitado
char schedulingAdvanceAndReturn[100];  //AVANÇO/RETORNO
char schedulingWithWater[100];         //COM ÁGUA/SEM ÁGUA
int schedulingPerct[100];
int schedulingCount = 0;
int schedulingPermission[100];
float schedulingAngle[100][2];  //(INICIO, FIM)
int schedulingStopRoute[100];   //PARADA POR PERCURSO
int _delayFailure;
int _tempoDesligar;

bool setRelayA1 = false;
bool setRelayA2 = false;
bool setWithWater = false;
String setOTA = "";

bool setScheduling = false;
bool setStopScheduling = false;
bool setStopNotification = false;
bool setRunByAngle = false;

const uint8_t MESSAGE_MAX_LEN = 52;     // ajuste conforme sua aplicação
const uint8_t CMD_FIRMWARE_CHUNK = 82;  // 'R'
const uint8_t GPS_FW_CHUNK_MAX_DATA = 8;

static const uint16_t RX_MAX = 54;        // máx. de bytes por quadro (sobras serão descartadas)
static const uint32_t RX_TIMEOUT = 3500;  // ms — compatível com ciclo do mestre (~2700-3500ms)
static const uint32_t IDLE_SLICE = 10;    // ms (yield cooperativo)

unsigned long lastPingSuccessMillis;

void saveRebootType( int type );
void saveHydrometer(float _current_hydrometer, bool _toReset);
void hydrometer(void* arg);
bool encodeMessage(const String& message, uint8_t* packet, uint8_t& packetLen);
void ledUpdate(void* arg);
void ledSend(void* arg);
void ledRead(void* arg);
void disconnectWIFI();
bool connectWiFi();
String httpGETRequest(String serverName);
String registroDataHora();
void bt(void* arg);
void commandBT(std::string command);
void getConfig();
void acionamento();
void pinInit();
void updateIDevice(String iDevice);
void updateFirmware();
bool driveFailure();
bool timeFailure();
bool waitAuxHigh();
void drainUartRx(HardwareSerial& ser);
void configuration433();
void printParameters(struct Configuration configuration);
void sendE32Safe();
void readE32Safe();
void saveChannel( int channel );

String recordInDevice = "-";
String recordAux1 = "-";

int countInDevice0 = 0;
int countInDevice1 = 0;
int countAux10 = 0;
int countAux11 = 0;
int countDriveFailureProg = 0;
int countDriveFailure0 = 0;
int countDriveFailure1 = 0;

String _GLOBAL_NAME = "";
String _GLOBAL_EVENT = "";
bool _GET_GLOBAL_EVENT = false;

String _GLOBAL_NOTIFICATION = "";
bool _GET_NOTIFICATION = false;

String _GLOBAL_DRIVE_FAILURE_PROG = "";
bool _GET_DRIVE_FAILURE_PROG = false;

int _TIME_DRIVE_FAILURE = 5;

#define NTP_SERVERS "0.nl.pool.ntp.org", "1.nl.pool.ntp.org", "2.nl.pool.ntp.org"
#define UTC_OFFSET +1

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool taskCompleted = false;

#define API_KEY "AIzaSyCSdiDHukknV_tX_AffQaHnZMKZ9vvAf64"  // Firebase: Define the API Key
#define USER_EMAIL "service@suportecontrol.com.br"         // Firebase: Define the user Email
#define USER_PASSWORD "Suportecontrol123*"                 // Firebase: Define password
#define STORAGE_BUCKET_ID "sc-firmware-esp32.appspot.com"  // Firebase: Define the Firebase storage bucket ID e.g bucket-name.appspot.com

//To Frequency
#define BASE_FREQUENCY_170 0x28
#define BASE_FREQUENCY_433 0x17
#define BASE_FREQUENCY_900 0x32
#define ENABLE_RSSI false

String swversion = __FILE__;

hw_timer_t* tWIFI = NULL;
portMUX_TYPE tWiFiMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t* tHORIMETER = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t* tGPS = NULL;
portMUX_TYPE tmrGps = portMUX_INITIALIZER_UNLOCKED;

int32_t horimeter = 0;  //valor em minutos
int firstReadingHourmeter = 0;

unsigned long previousMillis = 0;
const long interval = 700;

bool _WIFI_ON = false;
bool _WIFI_NTW = false;
bool _ACTIVITY = false;
bool _act = false;

int _N_ROUTES = 0;
String _RUN_BY_ANGLE = "0";
String _RUSH_HOUR_RUN_BY_ANGLE = "0";

String _ADVANCE = "";
String _RETURN = "";

String _UPDATE_LD = "";
bool _GET_LD = false;

String _RELAYA1 = "";
String _RELAYA2 = "";
bool _FIRST_LD = true;

String _GPS = "";
String _RECORD_GPS = "";
String _RECORD_UTC = "";
int _COUNT_UTC = 0;

time_t _LAST_TIME;
time_t _RECORD_TIME;
time_t _CONN_RECORD_TIME;
time_t _COUNT_TIME_DISPLACEMENT;

bool _REBOOT_TYPE_SEND = true;
int _CURRENT_REBOOT_TYPE = -1;

bool _ENABLE_COUNTING = false;

bool _DISABLE_DRIVE_FAILURE = false;

bool _SAFE_AREA_FAILURE = false;
bool _SAFE_AREA_ENABLE_INVERTER = true;
int _count_safe_area_enable_inverter = 0;

bool setRushHourStartNotification = false;
bool setRushHourEndNotification = false;
int _INDEX_AGDH = 0;

bool _SET_RUSH_HOUR = false;

bool setPermissionRushHour = false;
String _PERMISSION_RUSH_HOUR = "";

String clientGET = "";
String clientPATCH = "";
String clientPATCHSensor = "";

String _NETWORK = "";
String _PASSWORD = "";
String _FREQUENCY = "0";

int _LATENCY = 0;
float _CHANNEL = -1.0;
int _ADDL = -1;
int _ADDH = -1;

//To Timer
hw_timer_t* timer = NULL;  //Timer
uint64_t counterrnd = 0;   //Contar o tempo para trasmitir
uint64_t tm = 0;           //Auxiliar timer

// To Receive Flag
bool rcv = false;  //Se recebeu uma mensagem

//Ramdom Number
int rnd = 0;  //Intervalo de envio

// To LoRa
LoRa_E32* e32ttl;

bool ebyte32 = false;
float channel = 0;

// UUIDs do Nordic UART Service (NUS)
static NimBLEUUID NUS_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID NUS_RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");  // App -> ESP32 (WRITE)
static NimBLEUUID NUS_TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");  // ESP32 -> App (NOTIFY)

NimBLEServer* pServer;

static bool g_fwInProgress = false;
static size_t g_fwWritten = 0;

/************************FIRMWARE*********************************************************/

#define HYDRO_WINDOW_MS     5000
#define HYDRO_MAX_PULSES    4096

uint32_t _HYDRO_PULSE_TIME[HYDRO_MAX_PULSES];
uint16_t _HYDRO_HEAD = 0;
uint16_t _HYDRO_TAIL = 0;

String pathHTTPClient = "http://200.98.81.127:3000/";  //PRODUÇÃO
//String pathHTTPClient = "http://localhost:3000/"; //DESENVOLVIMENTO LOCAL

#define FIRMWARE_VERSION "2.0.1"
const char FIRMWARE_VERSION_DATA[] = "SC-FW-VERSION:" FIRMWARE_VERSION;

String _MODEL = "RPT";

bool _PRODUCTION = true;
bool _TO_PRINT = false;  // mude para false em produção
/*****************************************************************************************/

TaskHandle_t setLEDHandle = nullptr;

String type_application = "";

bool _LOAD_LORA = true;

time_t _RECORD_TIME_F;

bool _PERMISSION_TRIGERING = true;
bool _VISIBLE_CH_RELAY = false;

/******************************************************************************************/
String _CURRENT_SET_HYDROMETER_PULSES_PER_M3 = "";
String _CURRENT_SET_HYDROMETER = "";
float _CURRENT_HYDROMETER = 0.0;
int _CURRENT_HYDROMETER_PULSES_PER_M3 = 0;
int _GET_CURRENT_HYDROMETER_PULSES_PER_M3 = 0;
uint32_t _COUNT_PULSES_HYDROMETER = 0;
float _CURRENT_HYDROMETER_FLOW = 0.0f;
uint32_t _LAST_HYDROMETER_PULSE_MS = 0;
bool _CURRENT_SET_HYDROMETER_VALUE_ON_THE_RPT = false;
bool _CURRENT_SET_HYDROMETER_PULSES_PER_M3_ON_THE_RPT = false;
int _COUNT_ATTEMPTS_PULSES_PER_M3_RESPONSE = 0;
int _COUNT_ATTEMPTS_VALUE_RESPONSE = 0;

int _VALUE_RPT_CHANNEL = 0;
bool _SET_RPT_CHANNEL = false;
int _COUNT_ATTEMPTS_RPT_CHANNEL = 0;

bool _SET_RESET_RPT = false;
int _COUNT_ATTEMPTS_RESET_RPT = 0;
/******************************************************************************************/

bool _ENERGIZED_MODULE = true;

struct Telemetry {
  int cmd;          // 32
  char version[8];  // "1.3.7"
  double lat;       // -18.639295000
  double lng;       // -46.500635000
  int sats;         // 8
  float pressure;   // -2.00
  float offset;
  int getPressure;
  float minOffset;
  float maxOffset;
};

static void rtrim(char* s) {
  if (!s) return;
  size_t n = strlen(s);
  while (n && (s[n - 1] == ' ' || s[n - 1] == '\r' || s[n - 1] == '\n' || s[n - 1] == '\t')) {
    s[--n] = '\0';
  }
}

static bool parseTelemetryCsv(const char* line, Telemetry& out) {
  if (!line || !*line) return false;

  char tmp[RX_MAX];
  strncpy(tmp, line, sizeof(tmp) - 1);
  tmp[sizeof(tmp) - 1] = '\0';
  rtrim(tmp);

  if (strchr(tmp, ',') == nullptr) return false;

  char* save = nullptr;
  char* tok = strtok_r(tmp, ",", &save);
  if (!tok) return false;

  char* endptr = nullptr;
  long vcmd = strtol(tok, &endptr, 10);
  if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;
  out.cmd = (int)vcmd;

  out.version[0] = '\0';
  out.lat = 0.0;
  out.lng = 0.0;
  out.sats = 0;
  out.pressure = 0.0f;
  out.offset = 0.0f;
  out.getPressure = 0;
  out.minOffset = 0.0f;
  out.maxOffset = 0.0f;

  if (out.cmd == 32) {

    const int MAXF = 5;
    const char* fields[MAXF] = { nullptr };
    int i = 0;
    while (i < MAXF && (tok = strtok_r(nullptr, ",", &save)) != nullptr) {
      while (*tok == ' ' || *tok == '\t') tok++;
      fields[i++] = tok;
    }

    if (i < 4) return false;

    // version
    size_t L = strcspn(fields[0], " \t\r\n");
    if (L >= sizeof(out.version)) L = sizeof(out.version) - 1;
    memcpy(out.version, fields[0], L);
    out.version[L] = '\0';

    // lat
    endptr = nullptr;
    double vlat = strtod(fields[1], &endptr);
    if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;

    // lng
    endptr = nullptr;
    double vlng = strtod(fields[2], &endptr);
    if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;

    // sats
    endptr = nullptr;
    long vsats = strtol(fields[3], &endptr, 10);
    if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;

    // Optional pressão (5º campo, se presente)
    float vpress = 0.0f;
    if (i >= 5 && fields[4] && *fields[4]) {
      endptr = nullptr;
      double p = strtod(fields[4], &endptr);
      if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;
      vpress = (float)p;
    }

    if (vlat < -90.0 || vlat > 90.0) return false;
    if (vlng < -180.0 || vlng > 180.0) return false;
    if (vsats < 0 || vsats > 64) return false;

    out.lat = vlat;
    out.lng = vlng;
    out.sats = (int)vsats;
    out.pressure = vpress;  // 0.0 se não veio no frame
    return true;
  }

  else if (out.cmd == 34) {
    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    char* end2 = nullptr;
    double voffset = strtod(tok, &end2);
    if (*end2 != '\0' && *end2 != ' ' && *end2 != '\t') return false;

    out.offset = (float)voffset;
    return true;
  }

  else if (out.cmd == 64) {
    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    char* end2 = nullptr;
    long vGet = strtol(tok, &end2, 10);
    if (*end2 != '\0' && *end2 != ' ' && *end2 != '\t') return false;
    if (vGet != 0 && vGet != 1) return false;

    out.getPressure = (int)vGet;

    if (out.getPressure == 0) {
      return true;  // apenas "64,0"
    } else {
      const int EXPECTED = 2;
      const char* fields[EXPECTED] = { nullptr };
      for (int j = 0; j < EXPECTED; ++j) {
        tok = strtok_r(nullptr, ",", &save);
        if (!tok) return false;
        while (*tok == ' ' || *tok == '\t') tok++;
        fields[j] = tok;
      }

      endptr = nullptr;
      double vmin = strtod(fields[0], &endptr);
      if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;

      endptr = nullptr;
      double vmax = strtod(fields[1], &endptr);
      if (*endptr != '\0' && *endptr != ' ' && *endptr != '\t') return false;

      out.minOffset = (float)vmin;
      out.maxOffset = (float)vmax;
      return true;
    }
  }

  else if (out.cmd == 81) {  //INICIAR ATUALIZAÇÃO DE FIRMWARE

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  else if (out.cmd == 83) { //CONFIGURACAO DE NUMERO DE PULSOS POR M3 - HIDRÔMETRO

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  else if (out.cmd == 84) { //CONFIGURAÇÃO DE VALOR EXECUTADO PARA ZERO (0)

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  else if (out.cmd == 85) {  //ATUALIZAR CANAL DO RPT

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  else if (out.cmd == 86) {  //RESET

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  else if (out.cmd == 90) { 

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  else if (out.cmd == 91) {  //RETORNO RPT

    tok = strtok_r(nullptr, ",", &save);
    if (!tok) return false;
    while (*tok == ' ' || *tok == '\t') tok++;

    return true;
  }

  return false;
}

static inline void trim_std(std::string& s) {
  // remove espaços/CR/LF nas pontas
  auto is_ws = [](unsigned char c) {
    return c == ' ' || c == '\t' || c == '\r' || c == '\n';
  };
  size_t i = 0, j = s.size();
  while (i < j && is_ws(s[i])) ++i;
  while (j > i && is_ws(s[j - 1])) --j;
  if (i > 0 || j < s.size()) s = s.substr(i, j - i);
}

static std::vector<std::string> split_csv(const char* cstr) {
  std::vector<std::string> v;
  const char* p = cstr;
  const char* b = p;
  for (; *p; ++p) {
    if (*p == ',') {
      v.emplace_back(b, p - b);
      b = p + 1;
    }
  }
  v.emplace_back(b, p - b);
  // trim em cada campo
  for (auto& f : v) trim_std(f);
  return v;
}

static bool parse_int(const std::string& s, int& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  long v = strtol(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') return false;  // lixo na string
  if (v < INT_MIN || v > INT_MAX) return false;
  out = (int)v;
  return true;
}

static bool parse_float(const std::string& s, float& out) {
  if (s.empty()) return false;
  bool has_digit = false;
  bool has_dot = false;
  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (i == 0 && c == '-') {
      continue;
    }
    if (c == '.') {
      if (has_dot) return false;
      has_dot = true;
      continue;
    }
    if (c >= '0' && c <= '9') {
      has_digit = true;
      continue;
    }
    return false;
  }
  if (!has_digit) return false;
  char* end = nullptr;
  errno = 0;
  float v = std::strtof(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') return false;
  if (errno == ERANGE) return false;
  if (!std::isfinite(v)) return false;
  out = v;
  return true;
}

bool timeFailure() {
  time_t _CURRENT_TIME;
  time(&_CURRENT_TIME);
  return (difftime(_CURRENT_TIME, _RECORD_TIME_F) > _LATENCY) ? true : false;
}

void toPrint(const String& message, bool localPrint = false) {

  if (_TO_PRINT == false && localPrint == false) return;

  try {
    if (!Serial) {
      return;
    }

    if (message.length() == 0) {
      Serial.println("[WARN] toPrint: mensagem vazia");
      return;
    }

    if (message.length() > 4096) {
      // Proteção contra strings muito grandes que podem causar overflow
      Serial.println("[ERROR] toPrint: mensagem muito grande (" + String(message.length()) + " bytes)");
      return;
    }

    Serial.print(message);

  } catch (...) {
  }
}

void toPrint(const char* message, bool localPrint = false) {

  if (_TO_PRINT == false && localPrint == false) return;

  try {
    if (!Serial) return;
    if (!message) {
      Serial.println("[WARN] toPrint: ponteiro nulo");
      return;
    }

    size_t len = strlen(message);
    if (len == 0) {
      Serial.println("[WARN] toPrint: mensagem vazia");
      return;
    }

    if (len > 4096) {
      Serial.println("[ERROR] toPrint: mensagem muito grande");
      return;
    }

    Serial.print(message);

  } catch (...) {
    // Silencioso em caso de erro
  }
}

class ServerCallbacks : public NimBLEServerCallbacks {

  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.printf("Client address: %s\n", connInfo.getAddress().toString().c_str());

    /**
         *  We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments.
         */
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.printf("Client disconnected - start advertising\n");
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  }

  uint32_t onPassKeyDisplay() override {
    Serial.printf("Server Passkey Display\n");
    /**
         * This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
    return 123456;
  }

  void onConfirmPassKey(NimBLEConnInfo& connInfo, uint32_t pass_key) override {
    Serial.printf("The passkey YES/NO number: %" PRIu32 "\n", pass_key);
    /** Inject false if passkeys don't match. */
    NimBLEDevice::injectConfirmPasskey(connInfo, true);
  }

  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    /** Check that encryption was successful, if not we disconnect the client */
    if (!connInfo.isEncrypted()) {
      NimBLEDevice::getServer()->disconnect(connInfo.getConnHandle());
      Serial.printf("Encrypt connection failed - disconnecting client\n");
      return;
    }

    Serial.printf("Secured connection to: %s\n", connInfo.getAddress().toString().c_str());
  }

} serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {

    String valueToSend = "";
    valueToSend += _NETWORK;
    valueToSend += ",";
    valueToSend += _PASSWORD;
    valueToSend += ",";
    valueToSend += String(_CHANNEL);
    valueToSend += ",";
    valueToSend += String((_ADDH * 256) + _ADDL);
    valueToSend += ",";
    valueToSend += String(_FREQUENCY);
    valueToSend += ",";
    valueToSend += FIRMWARE_VERSION;
    valueToSend += ",";
    valueToSend += _MODEL;
    valueToSend += ",";
    valueToSend += String(_LATENCY);

    pCharacteristic->setValue(valueToSend);

    Serial.printf("%s : onRead(), value: %s\n", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());

  }

  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {

    std::string value = pCharacteristic->getValue();

    Serial.printf("%s : onWrite(), len=%d\n",
                  pCharacteristic->getUUID().toString().c_str(),
                  value.length());

    if (value.length() < 4) {
      Serial.println("Frame inválido: muito curto");
      return;
    }

    commandBT(value);
  }

  void onStatus(NimBLECharacteristic* pCharacteristic, int code) override {
    Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
  }

  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    std::string str = "Client ID: ";
    str += connInfo.getConnHandle();
    str += " Address: ";
    str += connInfo.getAddress().toString();
    if (subValue == 0) {
      str += " Unsubscribed to ";
    } else if (subValue == 1) {
      str += " Subscribed to notifications for ";
    } else if (subValue == 2) {
      str += " Subscribed to indications for ";
    } else if (subValue == 3) {
      str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID());

    Serial.printf("%s\n", str.c_str());
  }
} chrCallbacks;

class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
  void onWrite(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
    std::string dscVal = pDescriptor->getValue();
    Serial.printf("Descriptor written value: %s\n", dscVal.c_str());
  }

  void onRead(NimBLEDescriptor* pDescriptor, NimBLEConnInfo& connInfo) override {
    Serial.printf("%s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
  }
} dscCallbacks;

bool isWithinTimeRange(int startHour, int startMinute, int endHour, int endMinute, int currentHour, int currentMinute) {
  int start = startHour * 60 + startMinute;
  int end = endHour * 60 + endMinute;
  int current = currentHour * 60 + currentMinute;

  if (start < end) {
    return current >= start && current < end;
  } else {
    return current >= start || current < end;
  }
}

static bool readLineFromUart(Stream& uart, uint8_t* buf, uint16_t bufSize, uint16_t& payloadLen, uint32_t timeoutMs) {

  payloadLen = 0;

  if (!buf || bufSize == 0) return false;

  uint32_t t0 = millis();

  uint8_t len = 0;
  bool firstByteRead = false;

  while ((millis() - t0) < timeoutMs && !firstByteRead) {
    if (uart.available() > 0) {
      int b = uart.read();
      if (b < 0) {
        delay(IDLE_SLICE);
        continue;
      }

      len = (uint8_t)b;
      firstByteRead = true;
      t0 = millis();  // renova o timeout depois de receber o tamanho
    } else {
      delay(IDLE_SLICE);
    }
  }

  if (!firstByteRead) {
    return false;
  }

  if (len > bufSize) {

    uint16_t toDiscard = len;
    while ((millis() - t0) < timeoutMs && toDiscard > 0) {
      if (uart.available() > 0) {
        int b = uart.read();
        if (b >= 0) {
          toDiscard--;
          t0 = millis();
        }
      } else {
        delay(IDLE_SLICE);
      }
    }
    return false;
  }

  uint16_t idx = 0;

  while ((millis() - t0) < timeoutMs && idx < len) {
    if (uart.available() > 0) {
      int b = uart.read();
      if (b < 0) {
        delay(IDLE_SLICE);
        continue;
      }

      buf[idx++] = (uint8_t)b;
      t0 = millis();  // renova timeout a cada byte recebido
    } else {
      delay(IDLE_SLICE);
    }
  }

  if (idx != len) {
    return false;
  }

  payloadLen = len;
  return true;
}

void saveHydrometer(float _current_hydrometer, bool _toReset) {

  String _hdm = String(_current_hydrometer);
  int len = _hdm.length();

  // Serial.print("_CURRENT_HYDROMETER: ");
  // Serial.print(String(_hdm));
  // Serial.print(", ");
  // Serial.print(String(len));
  // Serial.print("\n");

  EEPROM.write(512, len);
  for (int c = 0; c < len; c++) {
    EEPROM.write(513 + c, _hdm[c]);
  }

  EEPROM.commit();

  if ( _toReset == true ) {
    xTaskCreate([](void*) {
      vTaskDelay(pdMS_TO_TICKS(500));
      esp_restart();
    },"rebooter", 2048, nullptr, 1, nullptr);
  }

}

void _funSetHydrometer() {
  _CURRENT_HYDROMETER = 0.0;
  saveHydrometer(_CURRENT_HYDROMETER, true);
}

void saveHydrometerPulsesPerM3(int value) {

  String _hdmppm3 = String(value);
  int len = _hdmppm3.length();

  // Serial.print("_CURRENT_HYDROMETER_PULSES_PER_M3: ");
  // Serial.print(String(_hdmppm3));
  // Serial.print(", ");
  // Serial.print(String(len));
  // Serial.print("\n");

  EEPROM.write(500, len);
  for (int c = 0; c < len; c++) {
    EEPROM.write(501 + c, _hdmppm3[c]);
  }

  EEPROM.commit();

  xTaskCreate([](void*) {
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
  },"rebooter", 2048, nullptr, 1, nullptr);

}

void _funSetHydrometerPulsesPerM3(int value) {
  saveHydrometerPulsesPerM3(value);
  //_CURRENT_SET_HYDROMETER_PULSES_PER_M3 = "N";
}

void _funOTA() {
  //_CURRENT_OTA = _LAST_OTA;
  //toPrint("_ota: ");
  //toPrint(String(_CURRENT_OTA) + "\n");

  /*
  if (_CURRENT_OTA == "B") {

    _LAST_OTA = "";
    _CURRENT_OTA = "C";

  } else if (_CURRENT_OTA == "C") {

    String dateTime = registroDataHora();

    if (isValidDateTime(dateTime, "")) {
      _GLOBAL_EVENT += "MÓDULO REINICIADO";
      _GLOBAL_EVENT += "*";
      _GLOBAL_EVENT += dateTime;
      _GLOBAL_EVENT += "|";

      _GLOBAL_NOTIFICATION  = _GLOBAL_NAME;
      _GLOBAL_NOTIFICATION += ": MÓDULO REINICIADO";

      _GET_GLOBAL_EVENT = true;
      _GET_NOTIFICATION = true;
    }

    _LAST_OTA = "";
    _CURRENT_OTA = "-";
    _ENERGIZED_MODULE = false; //FORÇAR PARA NÃO AVISAR MÓDULO ENERGIZADO

  } else if (_CURRENT_OTA == "S") {  // INICIAR ATUALIZAÇÃO

    _LAST_OTA = "";
    _CURRENT_OTA = "R";

  } else if (_CURRENT_OTA == "R") {

    String dateTime = registroDataHora();

    if (isValidDateTime(dateTime, "")) {
      _GLOBAL_EVENT += "ATUALIZAÇÃO DE FIRMWARE CONCLUÍDA";
      _GLOBAL_EVENT += "*";
      _GLOBAL_EVENT += dateTime;
      _GLOBAL_EVENT += "|";

      _GLOBAL_NOTIFICATION  = _GLOBAL_NAME;
      _GLOBAL_NOTIFICATION += ": ATUALIZAÇÃO DE FIRMWARE CONCLUÍDA";

      _GET_GLOBAL_EVENT = true;
      _GET_NOTIFICATION = true;
    }

    _LAST_OTA = "";
    _CURRENT_OTA = "-";
    _ENERGIZED_MODULE = false; 

  }
  */
}

void saveRebootType( int type ) {

  toPrint("REBOOT TYPE: " + String(type));

  EEPROM.write(600, type);
  
  EEPROM.commit();

}

bool waitAuxHigh() {
  uint32_t startTime = millis();
  while (digitalRead(pinToAux_Ebyte) == LOW) {
    if (millis() - startTime > 1000) {
      Serial.println("Erro: Módulo E32 ocupado (AUX LOW) por muito tempo.");
      return false;
    }
    delay(10);
  }
  return true;
}

void drainUartRx(HardwareSerial& ser) {
  uint32_t quietMs = 3;
  uint32_t last = millis();
  while (true) {
    while (ser.available()) {
      (void)ser.read();
      last = millis();
    }
    if (millis() - last >= quietMs) break;
  }
}

void stopLED() {
  if (setLEDHandle) {
    vTaskDelete(setLEDHandle);  // NÃO pode ser chamado de ISR
    setLEDHandle = nullptr;
    digitalWrite(pinToLED01, 0);
  }
}

void mcpRegister(void* args) {

  (void)args;

  uint8_t _last_value = 0, loop = 0;

  for (;;) {

    uint8_t _current_value = 0;

    for (uint8_t i = 0; i < 8; ++i) {
      if (_BIT_OUT[i]) _current_value |= (1u << i);
    }

    if (_current_value != _last_value) {
      const int MAX_RETRY = 5;
      const int RETRY_DELAY_MS = 5;
      bool written = false;

      // Protege o barramento I2C/MCP enquanto tentamos escrever/verificar
      if (mcpLock) xSemaphoreTake(mcpLock, portMAX_DELAY);

      for (int attempt = 0; attempt < MAX_RETRY; attempt++) {
        mcp.writeRegister(MCP23017Register::GPIO_B, _current_value);  // ESCRITA

        delay(RETRY_DELAY_MS);

        uint8_t v = mcp.readRegister(MCP23017Register::GPIO_B);

        if (v == _current_value) {
          toPrint("[MCP] write verify success\n");
          _last_value = _current_value;
          written = true;
          break;
        }
      }

      if (!written) {
        toPrint("[ERR] mcpRegister: write verify failed. expected=0x - ");
        toPrint(String(_current_value, HEX));
        toPrint("\tlastread=0x");
        toPrint(String(mcp.readRegister(MCP23017Register::GPIO_B), HEX) + "\n");
      }

      if (mcpLock) xSemaphoreGive(mcpLock);
    }

    uint8_t _value_in = mcp.readRegister(MCP23017Register::GPIO_A);

    for (uint8_t i = 0; i < 8; ++i) {
      _BIT_IN[i] = (_value_in >> i) & 0x01;
    }

    if (digitalRead(DI6) == 1) { _BIT_IN[6] = 1; } else { _BIT_IN[6] = 0; }

    if (digitalRead(DI7) == 1) { _BIT_IN[7] = 1; } else { _BIT_IN[7] = 0; }

    delay(100);
  }

  vTaskDelete(NULL);
}

void setLED(void* arg) {
  for (;;) {
    _ACTIVITY ? digitalWrite(pinToLED01, 1) : digitalWrite(pinToLED01, 0);
    _ACTIVITY = !_ACTIVITY;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void readRadioLoRa(void* arg) {

  for (;;) {

    if (ebyte32) {
      readE32Safe();
      sendE32Safe();
    }

    std::string line = _READ_LORA.c_str();
    trim_std(line);
    int type = -1;

    if (!line.empty()) {
        
        auto fields = split_csv(line.c_str());

        if (!fields.empty()) {

          if (!parse_int(fields[0], type)) {
            toPrint("PRIMEIRO PARSE_INT NÃO REALIZADO\n");
          } else if (type == 83) {  //CONFIGURAÇÃO DE NÚMERO DE PULSOS - HIDRÔMETRO
          
            int _v = 0;

            if (!parse_int(fields[1], _v)) {
              toPrint("SEGUNDO PARSE_INT NÃO REALIZADO\n");
            } else {
              _GET_CURRENT_HYDROMETER_PULSES_PER_M3 = _v;
              _COUNT_ATTEMPTS_PULSES_PER_M3_RESPONSE = 0;
              _CURRENT_SET_HYDROMETER_PULSES_PER_M3_ON_THE_RPT = true;
              time(&_RECORD_TIME_F);
            }

            Serial.println( "[HIDROMETRO] NÚMERO DE PULSOS: " + String(_v));
          
          } else if (type == 84) {  // RECONFIGURAR HIDRÔMETRO
          
            int _v = 0;

            if (!parse_int(fields[1], _v)) {
              toPrint("SEGUNDO PARSE_INT NÃO REALIZADO\n");
            } else {
              _COUNT_ATTEMPTS_VALUE_RESPONSE = 0;
              _CURRENT_SET_HYDROMETER_VALUE_ON_THE_RPT = true;
              time(&_RECORD_TIME_F);
            }

            Serial.print( "ZERAR HIDRÔMETRO: " + String(_v) + "\n");
          
          } else if (type == 85) {
          
          if (!parse_int(fields[1], _VALUE_RPT_CHANNEL)) {
          } else {
            //Serial.print("SET CHANNEL LORA -> ");
            //Serial.println(String( _VALUE_RPT_CHANNEL ));
            _SET_RPT_CHANNEL = true;
            _COUNT_ATTEMPTS_RPT_CHANNEL = 0;
            time(&_RECORD_TIME_F);
          }
          
        } else if (type == 86) {
          int _V = 0;
          if (!parse_int(fields[1], _V)) {
          } else {
            //Serial.println("<- SET RESET RPT ->");
            _SET_RESET_RPT = (_V == 1) ? true : false;
            _COUNT_ATTEMPTS_RESET_RPT = 0;
            time(&_RECORD_TIME_F);
          }
          
        }  else if (type == 90) {  // RETORNO RPT
          
            if (fields.size() == 2 || fields.size() == 3) {

              int _v = 0;

              if (!parse_int(fields[1], _v)) {
                toPrint("SEGUNDO PARSE_INT NÃO REALIZADO\n");
              } else {
                (_v == 1) ? updateIDevice("l") : updateIDevice("d");
                time(&_RECORD_TIME_F);
              }

              if (fields.size() == 3) {
                
                if (!parse_int(fields[2], _v)) {
                  toPrint("TERCEIRO PARSE_INT NÃO REALIZADO\n");
                } else {
                  if (_v == 1) _REBOOT_TYPE_SEND = false;
                }

              }

            }
          
          }
        }
      }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
}

void IRAM_ATTR timerHorimeter() {
  portENTER_CRITICAL_ISR(&timerMux);
  horimeter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR timerWiFi() {
  _act = true;
}

bool driveFailure() {
  time_t _CURRENT_TIME;
  time(&_CURRENT_TIME);
  return (difftime(_CURRENT_TIME, _RECORD_TIME) > _TIME_DRIVE_FAILURE) ? true : false;
}

bool encodeMessage(const String& message, uint8_t* packet, uint8_t& packetLen) {

  if (!packet) {
    packetLen = 0;
    return false;
  }

  int msgLen = message.length();

  if (msgLen <= 0) {
    packetLen = 0;
    return false;
  }

  if (msgLen > (MESSAGE_MAX_LEN - 1)) {
    return false;
  }

  packet[0] = (uint8_t)msgLen;

  for (int i = 0; i < msgLen; i++) {
    packet[i + 1] = static_cast<uint8_t>(message[i]);
  }

  packetLen = (uint8_t)(msgLen + 1);

  return true;
}

std::vector<std::string> splitString(std::string str, char splitter) {
  std::vector<std::string> result;
  std::string current = "";
  for (int i = 0; i < str.size(); i++) {
    if (str[i] == splitter) {
      if (current != "") {
        result.push_back(current);
        current = "";
      }
      continue;
    }
    current += str[i];
  }
  if (current.size() != 0)
    result.push_back(current);
  return result;
}

String httpGETRequest(String serverName) {
  WiFiClient client;
  HTTPClient http;

  http.begin(client, serverName);
  http.setTimeout(1200);
  http.setConnectTimeout(1200);

  int httpResponseCode = http.GET();

  String payload = "-";

  if (httpResponseCode == 200) {
    _WIFI_NTW = true;
    payload = http.getString();
  } else {
    _WIFI_NTW = false;
  }

  http.end();

  return payload;
}

bool httpPATCHRequest(String serverName, String payload) {
  WiFiClient client;
  HTTPClient http;
  bool result = false;

  http.begin(client, serverName);
  http.setTimeout(1200);
  http.setConnectTimeout(1200);

  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.PATCH(payload);

  if (httpResponseCode == 200) {
    _WIFI_NTW = true;
    result = true;
  } else {
    _WIFI_NTW = false;
  }

  http.end();

  return result;
}

String registroDataHora() {
  String sAux;
  String registroDH = "";

  if (dia < 10) {
    sAux = "0";
    sAux += String(dia);
  } else {
    sAux = String(dia);
  }

  registroDH += String(sAux[0]);
  registroDH += String(sAux[1]);
  registroDH += "-";

  if (mes < 10) {
    sAux = "0";
    sAux += String(mes);
  } else {
    sAux = String(mes);
  }

  registroDH += String(sAux[0]);
  registroDH += String(sAux[1]);
  registroDH += "-";

  sAux = "20";
  sAux += String(ano);

  registroDH += String(sAux[0]);
  registroDH += String(sAux[1]);
  registroDH += String(sAux[2]);
  registroDH += String(sAux[3]);
  registroDH += "*";

  if (hora < 10) {
    sAux = "0";
    sAux += String(hora);
  } else {
    sAux = String(hora);
  }

  registroDH += String(sAux[0]);
  registroDH += String(sAux[1]);
  registroDH += ":";

  if (minuto < 10) {
    sAux = "0";
    sAux += String(minuto);
  } else {
    sAux = String(minuto);
  }

  registroDH += String(sAux[0]);
  registroDH += String(sAux[1]);
  registroDH += ":";

  if (seg < 10) {
    sAux = "0";
    sAux += String(seg);
  } else {
    sAux = String(seg);
  }

  registroDH += String(sAux[0]);
  registroDH += String(sAux[1]);

  return registroDH;
}

void confirmDevice(void* arg) {

  (void)arg;

  for (int i = 0; i < 3; i++) {
    digitalWrite(pinToLED01, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(pinToLED01, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  vTaskDelete(NULL);
}

void commandBT(std::string command) {

  uint8_t list[command.length()];

  for (int i = 0; i < command.length(); i++) {
    list[i] = command[i];
  }

  if (list[0] == 0x02) {
    if (list[2] == 0x17) {
      xTaskCreate(confirmDevice, "confirmDevice", 2048, NULL, 1, NULL);
    } else if (list[2] == 0x18) {
      digitalWrite(pinToLED01, 1);
    } else if (list[2] == 0x19) {
      digitalWrite(pinToLED01, 0);
    } else if (list[2] == 0x20) {
      EEPROM.write(0, list[1]);
      _NETWORK = "";
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(1 + c, list[3 + c]);
        _NETWORK += list[3 + c];
      }
      EEPROM.commit();
    } else if (list[2] == 0x21) {
      EEPROM.write(128, list[1]);
      _PASSWORD = "";
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(129 + c, list[3 + c]);
        _PASSWORD += list[3 + c];
      }
      EEPROM.commit();
    } else if (list[2] == 0x22) {
      EEPROM.write(256, list[1]);
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(257 + c, list[3 + c]);
        psw[c] = list[3 + c];
      }
      EEPROM.commit();
    } else if (list[2] == 0x23) {
      EEPROM.write(384, list[1]);
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(385 + c, list[3 + c]);
        psw[c] = list[3 + c];
      }
      EEPROM.commit();
    } else if (list[2] == 0x24) {
      EEPROM.write(448, list[1]);
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(449 + c, list[3 + c]);
      }
      EEPROM.commit();
    } else if (list[2] == 0x25) {
      
      saveRebootType(0); //CONFIGURAÇÕES GERAIS/MÓDULO REINICIADO

      xTaskCreate ([](void*) { 
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
      },"rebooter", 2048, nullptr, 1, nullptr);

    } else if (list[2] == 0x26) {
      EEPROM.write(210, list[1]);
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(211 + c, list[3 + c]);
      }
      EEPROM.commit();
    } else if (list[2] == 0x27) {
      EEPROM.write(220, list[1]);
      for (int c = 0; c < list[1]; c++) {
        EEPROM.write(221 + c, list[3 + c]);
      }
      EEPROM.commit();
    } else if (list[2] == 0x80) {

      if (g_fwInProgress) {
        Update.end(false);
        g_fwInProgress = false;
        g_fwWritten = 0;
      }

      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }

      g_fwInProgress = true;
      g_fwWritten = 0;

      stopLED();
      xTaskCreate(ledUpdate, "ledUpdate", 2048, NULL, 1, &thLedUpdateHandle);

      _CURRENT_REBOOT_TYPE = 2;
      _REBOOT_TYPE_SEND = true; //ENVIAR POR RÁDIO QUE ESTÁ ATUALIZANDO...

    } else if (list[2] == 0x81) {

      const uint8_t* payload = &list[3];
      const uint8_t payloadLen = list[1];

      if (!g_fwInProgress) {
        return;
      }

      if (payloadLen == 0) {
        return;
      }

      size_t written = Update.write((uint8_t*)payload, payloadLen);
      if (written != payloadLen) {
        Serial.print("[FW-BLE] ERRO write. written=");
        Serial.print(written);
        Serial.print(" esperado=");
        Serial.println(payloadLen);
        Update.printError(Serial);

        Update.end(false);
        g_fwInProgress = false;
        g_fwWritten = 0;

        return;
      }

      g_fwWritten += written;

    } else if (list[2] == 0x82) {

      if (thLedUpdateHandle != NULL) {
        vTaskDelete(thLedUpdateHandle);
        thLedUpdateHandle = NULL;
      }

      xTaskCreate(setLED, "setLED", 2048, NULL, 1, &setLEDHandle);

      if (!g_fwInProgress) {
        return;
      }

      if (!Update.end(true)) {
        //Serial.println("[FW-BLE] Update.end() falhou");
        Update.printError(Serial);
        g_fwInProgress = false;
        g_fwWritten = 0;
        return;
      }

      if (!Update.isFinished()) {
        //Serial.println("[FW-BLE] Update não finalizado corretamente");
        g_fwInProgress = false;
        g_fwWritten = 0;
        return;
      }

      Serial.println("[FW-BLE] Firmware atualizado com sucesso! Reiniciando...");
      g_fwInProgress = false;
      g_fwWritten = 0;

      saveRebootType(1); //ATUALIZAÇÃO DE FIRMWARE

      delay(300);
      ESP.restart();
    }
  }
}

void getConfig() {
  int i, count, v;
  String _value_, _hdm, _hdmppm3;
  float j = 0.0;

  try {

    i = EEPROM.read(0);
    i = ( i == 255 ) ? 0 : i;
    _NETWORK = "";
    for (count = 0; count < i; count++) {
      _NETWORK += (char)EEPROM.read(count + 1);
    }

    i = EEPROM.read(128);
    i = ( i == 255 ) ? 0 : i;
    _PASSWORD = "";
    for (count = 0; count < i; count++) {
      _PASSWORD += (char)EEPROM.read(count + 129);
    }

    i = EEPROM.read(210);
    i = ( i == 255 ) ? 0 : i;
    String _channel = "";
    for (count = 0; count < i; count++) {
      _channel = _channel + String((char)EEPROM.read(count + 211));
    }
    _CHANNEL = stof(_channel.c_str());

     i = EEPROM.read(220);
    i = ( i == 255 ) ? 0 : i;
    String _latency = "";
    for (count = 0; count < i; count++) {
      _latency = _latency + String((char)EEPROM.read(count + 221));
    }

    _LATENCY = stoi(_latency.c_str());

    i = EEPROM.read(384);
    for (count = 0; count < i; count++) {
      deviceDB[count] = EEPROM.read(count + 385);
      Serial.print(deviceDB[count]);
    }
    Serial.print("\n");

    i = EEPROM.read(448);
    String s = "";
    for (count = 0; count < i; count++) {
      s = s + String((char)EEPROM.read(count + 449));
    }

    Serial.println(s);
    _ADDL = stoi(s.c_str()) % 256;
    _ADDH = stoi(s.c_str()) / 256;

    /******************************/
    _value_ = EEPROM.read(500);

    if (parse_int(_value_.c_str(), i)) {
      if (i == 255) { i = 0; }
    } else {
      i = 0;
    }

    for (count = 0; count < i; count++) {
      _hdmppm3 = _hdmppm3 + String((char)EEPROM.read(count + 501));
    }

    if (!parse_int(_hdmppm3.c_str(), v)) {
      _CURRENT_HYDROMETER_PULSES_PER_M3 = 0;
    } else {
      _CURRENT_HYDROMETER_PULSES_PER_M3 = v;
    }
    /******************************/

    _value_ = EEPROM.read(512);

    if (parse_int(_value_.c_str(), i)) {
      if (i == 255) { i = 0; }
    } else {
      i = 0;
    }

    for (count = 0; count < i; count++) {
      _hdm = _hdm + String((char)EEPROM.read(count + 513));
    }

    if (!parse_float(_hdm.c_str(), j)) {
      _CURRENT_HYDROMETER = 0.0;
    } else {
      _CURRENT_HYDROMETER = j;
    }
    /*******************************/

    _value_ = EEPROM.read(600);

    if (!parse_int(_value_.c_str(), _CURRENT_REBOOT_TYPE)) {
      _CURRENT_REBOOT_TYPE = 255;
    }

    saveRebootType( 255 ); //ZERA REGISTRO

    /*******************************/

    Serial.print("NETWORK: ");
    Serial.print(_NETWORK);
    Serial.print(", PASSWORD: ");
    Serial.print(_PASSWORD);
    Serial.print(", ADDRESS: ");
    Serial.print(String((_ADDH * 256) + _ADDL));
    Serial.print(", CHANNEL: ");
    Serial.print(_CHANNEL);
    Serial.print(", FIRMWARE-RDO: ");
    Serial.print(FIRMWARE_VERSION);
    Serial.print(", LATENCY: ");
    Serial.print(_LATENCY);
    Serial.print(", _CURRENT_HYDROMETER: ");
    Serial.print(String(_CURRENT_HYDROMETER));
    Serial.print(", _CURRENT_HYDROMETER_PULSES_PER_M3: ");
    Serial.print(String(_CURRENT_HYDROMETER_PULSES_PER_M3));
    Serial.print(", _CURRENT_REBOOT_TYPE: ");
    Serial.println(String(_CURRENT_REBOOT_TYPE));
  } catch (...) {}
}

void acionamento() {
  int count;
  time_t _CURRENT_TIME;

  for (count = 0; count < schedulingCount * 2; count++) {
    if (schedulingPermission[count] != minuto) {
      if ((schedulingTime[count][0] == hora) && (schedulingTime[count][1] == minuto) && (schedulingDWeek[count][diaSemana] == '1') && (schedulingEnabled[count] == 'E')) {

        schedulingPermission[count] = minuto;

        if (schedulingOnOff[count] == 'B') {  // MODO MONITORAMENTO PADRÃO
          updateIDevice("l");

          _GLOBAL_EVENT += "LIGA AUTOMATICAMENTE*";
          _GLOBAL_EVENT += registroDataHora();
          _GLOBAL_EVENT += "|";

          _GLOBAL_NOTIFICATION = "LIGA AUTOMATICAMENTE - ";
          _GLOBAL_NOTIFICATION += _GLOBAL_NAME;

          _GET_GLOBAL_EVENT = true;
          _GET_NOTIFICATION = true;
        } else if (schedulingOnOff[count] == 'H' && _ENABLE_COUNTING == true) {
          updateIDevice("d");
          setStopScheduling = true;

          _GLOBAL_EVENT += "DESLIGA AUTOMATICAMENTE*";
          _GLOBAL_EVENT += registroDataHora();
          _GLOBAL_EVENT += "|";

          _GLOBAL_NOTIFICATION = "DESLIGA AUTOMATICAMENTE - ";
          _GLOBAL_NOTIFICATION += _GLOBAL_NAME;

          _GET_GLOBAL_EVENT = true;
          _GET_NOTIFICATION = true;
        }
      } else {
        schedulingPermission[count] = -1;
      }
    }
  }

  time(&_CURRENT_TIME);

  if (isWithinTimeRange(startRushHour[0], startRushHour[1], endRushHour[0], endRushHour[1], hora, minuto) && _SET_RUSH_HOUR == false && _ENABLE_COUNTING == true && enableRushHour == true) {  //SE DENTRO DO HORÁRIO DE PONTA
    if (_PERMISSION_RUSH_HOUR == "S") {
      updateIDevice("d");
      _RUSH_HOUR_RUN_BY_ANGLE = "0";
      setRushHourStartNotification = true;
    }
    _SET_RUSH_HOUR = true;
  } else if (!isWithinTimeRange(startRushHour[0], startRushHour[1], endRushHour[0], endRushHour[1], hora, minuto) && _SET_RUSH_HOUR == true) {

    if (_RUSH_HOUR_RUN_BY_ANGLE == "0") {
      if (isWithinTimeRange(
            schedulingTime[_INDEX_AGDH + 0][0],
            schedulingTime[_INDEX_AGDH + 0][1],
            schedulingTime[_INDEX_AGDH + 1][0],
            schedulingTime[_INDEX_AGDH + 1][1], hora, minuto)
          && (schedulingEnabled[_INDEX_AGDH] == 'E')) {
        updateIDevice("l");
        setScheduling = true;
        _RUN_BY_ANGLE = "0";
        setRushHourEndNotification = true;
      }
    }

    setPermissionRushHour = true;
    _SET_RUSH_HOUR = false;
  }
}

void updateFirmware() {
  ESP.restart();
}

void triggering(void* arg) {

  unsigned int _c_time = 0;
  bool readyToTurnOn = false;

  _PERMISSION_TRIGERING = false;
  _ENABLE_COUNTING = true;

  //time (&_RECORD_TIME);
  //time (&_RECORD_TIME_0);

  if (_ENABLE_COUNTING == true && _VISIBLE_CH_RELAY == false) {  // SEM VÍNCULO COM CHAVE

    //Serial.println("SEM VÍNCULO COM CHAVE......");

    _BIT_OUT[_OUT_] = 1;

    while (_ENABLE_COUNTING == true && _VISIBLE_CH_RELAY == false) { delay(100); }  // ENQUANTO ESTIVER SEM BOMBEAMENTO...

    _BIT_OUT[_OUT_] = 0;
  }

  if (_ENABLE_COUNTING == true && _VISIBLE_CH_RELAY == true) {  // COM VÍNCULO COM CHAVE

    Serial.println("COM VÍNCULO COM CHAVE......");

    do {

      while (_BIT_IN[IN1] == 1 && _ENABLE_COUNTING == true) {
        delay(100);
        Serial.print(".");
      }  // ENQUANTO ESTIVER COM RESERVATORIO CHEIO...

      Serial.println("");

      _c_time = 0;

      while (_BIT_IN[IN1] == 0 && readyToTurnOn == false && _ENABLE_COUNTING == true) {

        Serial.print(_c_time);
        Serial.print(", ");

        if (_c_time >= 100) {

          readyToTurnOn = true;
        }

        _c_time++;

        delay(100);
      }

      if (_ENABLE_COUNTING == false) {

        Serial.println("THREAD CLOSED 1 ...");

        _PERMISSION_TRIGERING = true;

        vTaskDelete(NULL);
      }

    } while (readyToTurnOn == false);

    Serial.println("");

    _BIT_OUT[_OUT_] = 1;

    while (_ENABLE_COUNTING == true && _VISIBLE_CH_RELAY == true) {

      while (_BIT_IN[IN1] == 1 && _ENABLE_COUNTING == true) {

        delay(100);

        _BIT_OUT[_OUT_] = 0;

        Serial.print(".");

      }  // ENQUANTO ESTIVER CHEIO...

      Serial.println("");

      _c_time = 0;
      readyToTurnOn = false;

      while (_BIT_IN[IN1] == 0 && readyToTurnOn == false && _ENABLE_COUNTING == true) {  // ENQUANDO ESTIVER BOMBEANDO...

        Serial.print(_c_time);
        Serial.print(", ");

        if (_c_time >= 100) {

          readyToTurnOn = true;
        }

        _c_time++;

        delay(100);
      }

      if (readyToTurnOn == true && _ENABLE_COUNTING == true) {

        _BIT_OUT[_OUT_] = 1;
      }
    }

    if (_VISIBLE_CH_RELAY == false) {  //EM CASO DE DESATIVAR COM VÍNCULO, DESLIGAR O SISTEMA

      _BIT_OUT[_OUT_] = 0;
      _ENABLE_COUNTING = false;  //DESATIVAR QUALQUER SERVIÇO DE MONITORAMENTO, FALHA FUNCIONAMENTO, GPS, DENTRE OUTROS.....
    }
  }

  Serial.println("THREAD CLOSED 2 ...");

  _PERMISSION_TRIGERING = true;

  vTaskDelete(NULL);
}

void updateIDevice(String iDevice) {
  if (iDevice == "l") {
    _BIT_OUT[_OUT_] = 1;
  } else if (iDevice == "d" || iDevice == "null") {
    _BIT_OUT[_OUT_] = 0;
  }
}

void fcsDownloadCallback(FCS_DownloadStatusInfo info) {
  if (info.status == fb_esp_fcs_download_status_init) {
    Serial.printf("New update found\n");
    Serial.printf("Downloading firmware %s (%d bytes)\n", info.remoteFileName.c_str(), info.fileSize);
  } else if (info.status == fb_esp_fcs_download_status_download) {
    Serial.printf("Downloaded %d%s\n", (int)info.progress, "%");
  } else if (info.status == fb_esp_fcs_download_status_complete) {
    Serial.println("Donwload firmware completed.");
    Serial.println();
  } else if (info.status == fb_esp_fcs_download_status_error) {
    Serial.printf("New firmware update not available or download failed, %s\n", info.errorMsg.c_str());
  }
}

void disconnectWIFI() {
  WiFi.disconnect();
}

bool connectWiFi() {

  uint32_t timeoutMs = 50000;

  WiFi.disconnect(false, true);

  WiFi.mode(WIFI_STA);
  WiFi.begin(_NETWORK, _PASSWORD);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < timeoutMs) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    return true;
  } 
  return false;
}

void pinInit() {

  Serial.begin(115200);

  pinMode(pinToLED01, OUTPUT);
  digitalWrite(pinToLED01, 0);

  pinMode(CONFIG, INPUT_PULLUP);
  pinMode(pinToM0_Ebyte, OUTPUT);
  pinMode(pinToM1_Ebyte, OUTPUT);
  pinMode(pinToAux_Ebyte, INPUT_PULLUP);

  pinMode(DI6, INPUT);
  pinMode(DI7, INPUT);

  pinMode(pinTo_RE_DE_485, OUTPUT);
  digitalWrite(pinTo_RE_DE_485, LOW);

  Wire.begin();
  mcp.init();
  mcp.portMode(MCP23017Port::A, 0b11111111);          //PORTA A COMO ENTRADA
  mcp.portMode(MCP23017Port::B, 0);                   //PORTA B COMO SAÍDA
  mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A
  mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

  for (int i = 0; i < 8; i++) {
    _BIT_OUT[i] = 0;
    _BIT_IN[i] = 0;
  }

  // cria mutex para proteger I2C/MCP durante escritas/verificações
  mcpLock = xSemaphoreCreateMutex();
  if (mcpLock == NULL) {
    Serial.println("[ERR] falha ao criar mcpLock");
  }

  xTaskCreate(mcpRegister, "mcpRegister", 4096, NULL, 1, NULL);

  //
  EEPROM.begin(EEPROM_SIZE);

  tHORIMETER = timerBegin(0, 80, true);  //TIMER 0
  timerAttachInterrupt(tHORIMETER, &timerHorimeter, true);
  timerAlarmWrite(tHORIMETER, 60000000, true);  // 1 MINUTO
  timerAlarmEnable(tHORIMETER);
  timerStop(tHORIMETER);

  tWIFI = timerBegin(1, 80, true);  //TIMER 1
  timerAttachInterrupt(tWIFI, &timerWiFi, true);
  timerAlarmWrite(tWIFI, 30000000, true);  //30 segundos
  timerAlarmEnable(tWIFI);
  timerStop(tWIFI);

  _FIRST_LD = true;

  delay(1000);

}

void bluetoothLESetup(void) {

  NimBLEDevice::init("NimBLE");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  NimBLEService* pDeadService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  NimBLECharacteristic* pBeefCharacteristic =
    pDeadService->createCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);  // only allow writing if paired / encrypted

  pBeefCharacteristic->setValue("Burger");
  pBeefCharacteristic->setCallbacks(&chrCallbacks);

  NimBLEService* pBaadService = pServer->createService("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
  NimBLECharacteristic* pFoodCharacteristic =
    pBaadService->createCharacteristic("F00D", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

  pFoodCharacteristic->setValue("Fries");
  pFoodCharacteristic->setCallbacks(&chrCallbacks);

  NimBLEDescriptor* pC01Ddsc = pFoodCharacteristic->createDescriptor("C01D", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC, 20);
  pC01Ddsc->setValue("Send it back!");
  pC01Ddsc->setCallbacks(&dscCallbacks);

  pDeadService->start();
  pBaadService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName(_DEVICE_NAME_BLE.c_str());
  pAdvertising->addServiceUUID(pDeadService->getUUID());
  pAdvertising->addServiceUUID(pBaadService->getUUID());

  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.printf("Advertising Started\n");
}

void configTimer() {
  timer = timerBegin(0, 80, true);
}

void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.HEAD, BIN);
  Serial.print(" ");
  Serial.print(configuration.HEAD, DEC);
  Serial.print(" ");
  Serial.println(configuration.HEAD, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, BIN);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, BIN);
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  _FREQUENCY = String(configuration.getChannelDescription());
  Serial.print("Frequency: ");
  Serial.println(_FREQUENCY);
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte  : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRate());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRate());

  Serial.print(F("OptionTrans        : "));
  Serial.print(configuration.OPTION.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFixedTransmissionDescription());
  Serial.print(F("OptionPullup       : "));
  Serial.print(configuration.OPTION.ioDriveMode, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getIODroveModeDescription());
  Serial.print(F("OptionWakeup       : "));
  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
  Serial.print(F("OptionFEC          : "));
  Serial.print(configuration.OPTION.fec, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getFECDescription());
  Serial.print(F("OptionPower        : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());

  Serial.println("----------------------------------------");
}

bool selectLora() {

  if (e32ttl) {
    delete e32ttl;
    e32ttl = nullptr;
  }

  e32ttl = new LoRa_E32(pinToRX_Ebyte, pinToTX_Ebyte, &Serial2, pinToAux_Ebyte, pinToM0_Ebyte, pinToM1_Ebyte, UART_BPS_RATE_9600, SERIAL_8N1);
  e32ttl->begin();

  if (!waitAuxHigh()) {
    Serial.println("[E32] Timeout aguardando AUX após setConfiguration()");
    return false;
  }

  drainUartRx(Serial2);

  configuration433();

  return ebyte32;
}

void configuration433() {

  if (e32ttl == nullptr) {
    return;
  }

  Status status = e32ttl->setMode(MODE_3_PROGRAM);  //Set as Programming Mode M0=1 / M1=1
  delay(60);

  channel = _CHANNEL;

  ResponseStructContainer c = e32ttl->getConfiguration();  //Configuracao

  if (c.data == nullptr) {
    Serial.println("Erro ao obter configuração do módulo.");
    return;
  }

  Configuration configuration = *(Configuration*)c.data;

  configuration.ADDL = _ADDL;                                      // ENDEREÇO BAIXO
  configuration.ADDH = _ADDH;                                      // ENDEREÇO ALTO
  configuration.CHAN = channel;                                    // CANAL 410 ~ 441
  configuration.OPTION.fec = FEC_1_ON;                             // PROTEÇÃO CONTRA RUÍDO, LIGADO DIMINUI DISTANCIA E OTIMIZA COMUNICAÇÃO
  configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;  //TRANSMISSÃO FIXA, DEFINE QUE COMUNICARÁ SOMENTE COM MÓDULOS DE MESMO ENDEREÇO NO CANAL
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  configuration.OPTION.transmissionPower = POWER_30;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;  // SEM EFEITO PARA MODO 0. INTERVALO DE TEMPO EM ms PARA O RECEPTOR QUE OPERE NO MODO 2 (SLEEP)
  configuration.SPED.airDataRate = AIR_DATA_RATE_000_03;
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  ResponseStatus rs = e32ttl->setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);  // Set configuration changed and set to hold the configuration
  delay(50);
  c.close();

  if (rs.code != SUCCESS) {
    Serial.print("Falha ao configurar o módulo: ");
    return;
  }

  printParameters(configuration);

  status = e32ttl->setMode(MODE_0_NORMAL);  //Muda para Modo de Operação M0=0 M1=0

  if (!waitAuxHigh()) {
    Serial.println("[E32] Timeout aguardando AUX após setConfiguration()");
    return;
  }

  drainUartRx(Serial2);

  Serial.print("Normal Mode: ");
  Serial.println(status);

  ebyte32 = true;
}

void sendMessage() {

  if (e32ttl == nullptr) {
    Serial.println("Erro: Objeto e32ttl não está inicializado.");
    return;
  }

  String message = "91,";
  message += String(FIRMWARE_VERSION);
  message += ",";
  (_BIT_IN[IN0] == 1) ? message += "1," : message += "0,";
  (_BIT_IN[IN1] == 1) ? message += "1," : message += "0,";
  (_BIT_IN[IN2] == 1) ? message += "1" : message += "0";
  message += "\n";

  ResponseStatus rs = e32ttl->sendFixedMessage(_ADDH, _ADDL, channel, message);

  if (rs.code == 1) {
    //Serial.print("Mensagem enviada com sucesso: ");
    //Serial.println(message);
  } else {
    Serial.print("Falha ao enviar mensagem: ");
    Serial.println(rs.getResponseDescription());
  }

}

void ledUpdate(void* arg) {

  (void)arg;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  vTaskDelete(NULL);
}

void saveChannel( int channel ) {

  String _channel = String(channel);

  EEPROM.write(210, (uint8_t) _channel.length());

  for (int i = 0; i < _channel.length(); i++) {
    EEPROM.write(211 + i, (uint8_t) _channel[i]);
    //Serial.print(_channel[i]);
  }
  //Serial.println("");
  EEPROM.commit();

  xTaskCreate([](void*) {
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart(); 
  }, "rebooter", 2048, nullptr, 1, nullptr);

}

void ledSend(void* arg) {

  (void)arg;

  vTaskDelay(pdMS_TO_TICKS(100));

  vTaskDelete(NULL);
}

void ledRead(void* arg) {

  (void)arg;

  digitalWrite(pinToLED01, 1);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(pinToLED01, 0);

  vTaskDelete(NULL);
}

void readEeprom() {
  _ADDL = EEPROM.read(0);
  _ADDH = EEPROM.read(1);
  delay(50);
}

void sendE32() {

  if (e32ttl == nullptr) {
    Serial.println("Erro: Objeto e32ttl não está inicializado.");
    return;
  }

  if (!waitAuxHigh()) {
    return;
  }

  String message = "";

  if ( _CURRENT_SET_HYDROMETER_PULSES_PER_M3_ON_THE_RPT == true && _COUNT_ATTEMPTS_PULSES_PER_M3_RESPONSE < 3 ) {
    message = "83,1";
    _COUNT_ATTEMPTS_PULSES_PER_M3_RESPONSE++;
  } else if ( _CURRENT_SET_HYDROMETER_PULSES_PER_M3_ON_THE_RPT == true && _COUNT_ATTEMPTS_PULSES_PER_M3_RESPONSE >= 3 ) {
   _CURRENT_SET_HYDROMETER_PULSES_PER_M3_ON_THE_RPT = false;
   _funSetHydrometerPulsesPerM3(_GET_CURRENT_HYDROMETER_PULSES_PER_M3);
  } else if ( _CURRENT_SET_HYDROMETER_VALUE_ON_THE_RPT == true && _COUNT_ATTEMPTS_VALUE_RESPONSE < 3 ) {
    message = "84,1";
    _COUNT_ATTEMPTS_VALUE_RESPONSE++;
  } else if ( _CURRENT_SET_HYDROMETER_VALUE_ON_THE_RPT == true && _COUNT_ATTEMPTS_VALUE_RESPONSE >= 3 ) {
   _CURRENT_SET_HYDROMETER_VALUE_ON_THE_RPT = false;
   _funSetHydrometer();
  } else if (_SET_RPT_CHANNEL == true && _COUNT_ATTEMPTS_RPT_CHANNEL < 5) {
    message += "85,1";
    _COUNT_ATTEMPTS_RPT_CHANNEL++;
  } else if (_SET_RPT_CHANNEL == true && _COUNT_ATTEMPTS_RPT_CHANNEL >= 5) {
    saveChannel( _VALUE_RPT_CHANNEL );
  } else if (_SET_RESET_RPT == true && _COUNT_ATTEMPTS_RESET_RPT < 5) {
    message += "86,1";
    _COUNT_ATTEMPTS_RESET_RPT++;
  } else if (_SET_RESET_RPT == true && _COUNT_ATTEMPTS_RESET_RPT >= 5) {
    ESP.restart();
  } else {
    message = "91,";
    message += String(FIRMWARE_VERSION);
    message += ",";
    (_BIT_IN[IN0] == 1) ? message += "1," : message += "0,";
    (_BIT_IN[IN1] == 1) ? message += "1," : message += "0,";
    (_BIT_IN[IN2] == 1) ? message += "1," : message += "0,";
    message += String(_CURRENT_HYDROMETER_PULSES_PER_M3);
    message += ",";
    message += String(_CURRENT_HYDROMETER, 2);
    message += ",";
    message += String(_CURRENT_HYDROMETER_FLOW, 2);

    if ( _REBOOT_TYPE_SEND == true ) {
      message += ",";
      message += String(_CURRENT_REBOOT_TYPE);
    }

  }

  uint8_t packet[MESSAGE_MAX_LEN];
  uint8_t packetLen = 0;

  if (encodeMessage(message, packet, packetLen)) {

    //Serial.print("message: ");
    //Serial.print(String(message) + "\n");

    ResponseStatus rs = e32ttl->sendFixedMessage(_ADDH, _ADDL, channel, packet, packetLen);

    if (rs.code == 1) {
      //Serial.print("Mensagem enviada com sucesso: ");
      //Serial.println(message);
    } else {
      Serial.print("Falha ao enviar mensagem: ");
      Serial.println(rs.getResponseDescription());
    }
  }

  xTaskCreate(ledSend, "ledSend", 2048, NULL, 1, NULL);
  
}

void readE32() {

  _READ_LORA = "";

  if (e32ttl == nullptr) {
    return;
  }

  const unsigned long timeout = RX_TIMEOUT + (80 * (esp_random() % 6));
  unsigned long startTime = millis();

  while (e32ttl->available() == 0 && (millis() - startTime < timeout)) {
    delay(10);
  }
  
  if (e32ttl->available() == 0) {
    while (Serial2.available() > 0) (void)Serial2.read();  // limpa buffer
    return;
  }

  if (!waitAuxHigh()) {
    return;
  }

  uint8_t rxBuf[52];
  uint16_t rxLen = 0;
  bool got = readLineFromUart(Serial2, rxBuf, sizeof(rxBuf), rxLen, 500);

  xTaskCreate(ledRead, "ledRead", 2048, NULL, 1, NULL);

  if (got) {

    String msg;

    msg.reserve(rxLen);

    for (uint16_t i = 0; i < rxLen; i++) {
      msg += (char)rxBuf[i];
    }

    Telemetry t;

    if (parseTelemetryCsv(msg.c_str(), t)) {

      _READ_LORA = msg;

      //Serial.print("READ_LORA: ");
      //Serial.print(String(_READ_LORA) + "\n");

    } else {
      while (Serial2.available() > 0) (void)Serial2.read();
    }

  } else {
    while (Serial2.available() > 0) (void)Serial2.read();
  }

  delay(100);

}

void sendE32Safe() {
  try {
    sendE32();  // sua função robusta (sem String e usando Serial da UART do E32)
  } catch (const std::bad_alloc& e) {
    Serial.println("[TX] bad_alloc");
  } catch (const std::exception& e) {
    Serial.print("[TX] std::exception: ");
    Serial.println(e.what());  // pode imprimir msg útil
  } catch (...) {
    Serial.println("[TX] unknown exception");
  }
}

void readE32Safe() {
  try {
    readE32();
  } catch (const std::bad_alloc& e) {
    while (Serial2.available() > 0) (void)Serial2.read();
    Serial.println("[RX] bad_alloc");
  } catch (const std::exception& e) {
    while (Serial2.available() > 0) (void)Serial2.read();
    Serial.println("[RX] std::exception");
  } catch (...) {
    while (Serial2.available() > 0) (void)Serial2.read();
    Serial.println("[RX] unknown exception");
  }
}

void hydrometer(void* arg) {

  enum {
    WAIT_FALLING,
    WAIT_NEXT_RISING
  };

  bool lastState = _BIT_IN[IN2];

  uint8_t state = lastState ? WAIT_FALLING : WAIT_NEXT_RISING;

  for (;;) {

    bool currentState = _BIT_IN[IN2];

    if (_CURRENT_HYDROMETER_PULSES_PER_M3 <= 0) {
      //Serial.print(".");  //Forçar ficar aqui até que _CURRENT_HYDROMETER_PULSES_PER_M3 receba algum valor
    } else if (currentState != lastState) {

      switch (state) {

        case WAIT_FALLING:

          // HIGH -> LOW
          if (lastState && !currentState) {
            state = WAIT_NEXT_RISING;
          }

          break;

        case WAIT_NEXT_RISING:

          if (!lastState && currentState) {

            _COUNT_PULSES_HYDROMETER++;

            uint32_t now = millis();

            if (_LAST_HYDROMETER_PULSE_MS > 0) {

              uint32_t deltaMs = now - _LAST_HYDROMETER_PULSE_MS;

              if (deltaMs > 0) {
                float deltaSeconds = deltaMs / 1000.0f;
                _CURRENT_HYDROMETER_FLOW = 3600.0f / (deltaSeconds * _CURRENT_HYDROMETER_PULSES_PER_M3);
                //Serial.printf("[HYDROMETER] Vazao: %.2f m3/h\n", _CURRENT_HYDROMETER_FLOW);
              }
            }

            _LAST_HYDROMETER_PULSE_MS = now;

            //Serial.println("_COUNT_PULSES_HYDROMETER: " + String(_COUNT_PULSES_HYDROMETER));

            if (_CURRENT_HYDROMETER_PULSES_PER_M3 > 0 && _COUNT_PULSES_HYDROMETER >= _CURRENT_HYDROMETER_PULSES_PER_M3) {

              _COUNT_PULSES_HYDROMETER = 0;

              _CURRENT_HYDROMETER += 1.0f;

              saveHydrometer(_CURRENT_HYDROMETER, false);

              //Serial.printf("[HYDROMETER] Total: %.0f m3\n",_CURRENT_HYDROMETER);
            }

            // O HIGH atual já é o início do próximo ciclo
            state = WAIT_FALLING;
          }

          break;
      }

      lastState = currentState;
    }

    uint32_t now = millis();

    if (_LAST_HYDROMETER_PULSE_MS > 0) {

      uint32_t elapsed = now - _LAST_HYDROMETER_PULSE_MS;

      // sem pulso por mais de 2 intervalos da vazão máxima
      if (elapsed > 30000) {

        _CURRENT_HYDROMETER_FLOW = 0.0f;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(400));
  }
}

void setup() {

  pinInit();

  getConfig();

  Serial.println(FIRMWARE_VERSION_DATA);

  xTaskCreate(hydrometer, "hydrometer", 4096, NULL, 1, NULL);
  
  bluetoothLESetup();

  if (digitalRead(CONFIG) == 0) {
    digitalWrite(pinToLED01, 1);
    while (true);
  } else {
    digitalWrite(pinToLED01, 0);
  }

  xTaskCreate(setLED, "setLED", 2048, NULL, 1, &setLEDHandle);
  
  delay(1000);

  if (_ADDL == -1 || _ADDH == -1 || _CHANNEL == -1.0) {
    Serial.println("RESET\n");
    stopLED();
    ESP.restart();
  }

  if (!selectLora()) {
    stopLED();
    ESP.restart();
  }

   if (_PRODUCTION == true) {
    if (connectWiFi()) {
      checkUpdateOnFirebaseWifi();
      disconnectWIFI();
    }
  }

  delay(1000);

  xTaskCreate(readRadioLoRa, "readRadioLoRa", 4096, NULL, 1, NULL);

  stopLED();

}

void loop() {

  bool _loopControl = true;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {

    if (timeFailure()) {
      updateIDevice("d");
    }

    previousMillis = currentMillis;
  }
}

void checkUpdateOnFirebaseWifi() {
  string _deviceDB = deviceDB;
  _deviceDB.erase(remove(_deviceDB.begin(), _deviceDB.end(), ':'), _deviceDB.end());
  string _STR = "SC";
  _STR.append(_deviceDB);
  _STR.append(".bin");
  const char* FIRMWARE_PATH = _STR.data();

  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  config.token_status_callback = tokenStatusCallback;  // see addons/TokenHelper.h

#if defined(ESP8266)
  fbdo.setBSSLBufferSize(1024, 1024);
#endif
  config.fcs.download_buffer_size = 2048;

  Firebase.reconnectWiFi(true);

  Firebase.begin(&config, &auth);

  if (Firebase.ready() && !taskCompleted) {
    taskCompleted = true;

    Serial.println("\nChecking for new firmware update available...\n");

    if (!Firebase.Storage.downloadOTA(&fbdo, STORAGE_BUCKET_ID, FIRMWARE_PATH, fcsDownloadCallback)) {
      Serial.println(fbdo.errorReason());
    } else {  // Delete the file after update
      Serial.printf("Delete file... %s\n", Firebase.Storage.deleteFile(&fbdo, STORAGE_BUCKET_ID, FIRMWARE_PATH) ? "ok" : fbdo.errorReason().c_str());
      Serial.println("Restarting...\n\n");
      EEPROM.write(449, 1);
      EEPROM.commit();
      delay(2000);
      ESP.restart();
    }
  }
}

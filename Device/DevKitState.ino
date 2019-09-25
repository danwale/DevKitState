#include "Arduino.h"
#include "AZ3166WiFi.h"
#include "AzureIotHub.h"
#include "DevKitMQTTClient.h"
#include "SystemVersion.h"
#include "Sensor.h"
#include "parson.h"
#include "SystemTickCounter.h"
#include "config.h"

DevI2C *ext_i2c;
LSM6DSLSensor *acc_gyro;
HTS221Sensor *ht_sensor;
LIS2MDLSensor *magnetometer;
IRDASensor *IrdaSensor;
LPS22HBSensor *pressureSensor;
RGB_LED rgbLed;

static bool hasWifi = false;
static int userLEDState = 0;
static int rgbLEDState = 0;
static int rgbLEDR = 0;
static int rgbLEDG = 0;
static int rgbLEDB = 0;

int messageCount = 1;
int sentMessageCount = 0;
static bool messageSending = true;
static uint64_t send_interval_ms;

int btnAState;

static void InitWifi()
{
  Screen.print(2, "Connecting...");

  if (WiFi.begin() == WL_CONNECTED)
  {
    IPAddress ip = WiFi.localIP();
    Screen.print(1, ip.get_address());
    hasWifi = true;
    Screen.print(2, "Running... \r\n");
  }
  else
  {
    hasWifi = false;
    Screen.print(1, "No Wi-Fi\r\n ");
  }
}

void parseTwinMessage(DEVICE_TWIN_UPDATE_STATE updateState, const char *message)
{
  JSON_Value *root_value;
  root_value = json_parse_string(message);
  if (json_value_get_type(root_value) != JSONObject)
  {
    if (root_value != NULL)
    {
      json_value_free(root_value);
    }
    LogError("parse %s failed", message);
    return;
  }
  JSON_Object *root_object = json_value_get_object(root_value);

  if (updateState == DEVICE_TWIN_UPDATE_COMPLETE)
  {
    JSON_Object *desired_object = json_object_get_object(root_object, "desired");
    if (desired_object != NULL)
    {
      if (json_object_has_value(desired_object, "userLEDState"))
      {
        userLEDState = json_object_get_number(desired_object, "userLEDState");
      }
      if (json_object_has_value(desired_object, "rgbLEDState"))
      {
        rgbLEDState = json_object_get_number(desired_object, "rgbLEDState");
      }
      if (json_object_has_value(desired_object, "rgbLEDR"))
      {
        rgbLEDR = json_object_get_number(desired_object, "rgbLEDR");
      }
      if (json_object_has_value(desired_object, "rgbLEDG"))
      {
        rgbLEDG = json_object_get_number(desired_object, "rgbLEDG");
      }
      if (json_object_has_value(desired_object, "rgbLEDB"))
      {
        rgbLEDB = json_object_get_number(desired_object, "rgbLEDB");
      }
    }
  }
  else
  {
    if (json_object_has_value(root_object, "userLEDState"))
    {
      userLEDState = json_object_get_number(root_object, "userLEDState");
    }
    if (json_object_has_value(root_object, "rgbLEDState"))
    {
      rgbLEDState = json_object_get_number(root_object, "rgbLEDState");
    }
    if (json_object_has_value(root_object, "rgbLEDR"))
    {
      rgbLEDR = json_object_get_number(root_object, "rgbLEDR");
    }
    if (json_object_has_value(root_object, "rgbLEDG"))
    {
      rgbLEDG = json_object_get_number(root_object, "rgbLEDG");
    }
    if (json_object_has_value(root_object, "rgbLEDB"))
    {
      rgbLEDB = json_object_get_number(root_object, "rgbLEDB");
    }
  }

  if (rgbLEDState == 0)
  {
    rgbLed.turnOff();
  }
  else
  {
    rgbLed.setColor(rgbLEDR, rgbLEDG, rgbLEDB);
  }

  pinMode(LED_USER, OUTPUT);
  digitalWrite(LED_USER, userLEDState);
  json_value_free(root_value);
}

static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  parseTwinMessage(updateState, temp);
  free(temp);
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    sentMessageCount++;
  }

  char line1[20];
  sprintf(line1, "Count: %d/%d", sentMessageCount, messageCount);
  Screen.print(2, line1);

  messageCount++;
}

static int DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
  LogInfo("Try to invoke method %s", methodName);
  const char *responseMessage = "\"Successfully invoke device method\"";
  int result = 200;

  if (strcmp(methodName, "start") == 0)
  {
    LogInfo("Start sending temperature and humidity data");
    messageSending = true;
  }
  else if (strcmp(methodName, "stop") == 0)
  {
    LogInfo("Stop sending temperature and humidity data");
    messageSending = false;
  }
  else
  {
    LogInfo("No method %s found", methodName);
    responseMessage = "\"No method found\"";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

static void ReportConfirmationCallback(int statusCode)
{
  LogInfo("Status Code for Report State: %d", statusCode);
}

void setup()
{
  rgbLed.turnOff();
  Screen.init();
  Screen.print(0, "IoT DevKit");
  Screen.print(2, "Initializing...");
  Screen.print(3, " > WiFi");

  pinMode(USER_BUTTON_A, INPUT);
  btnAState = digitalRead(USER_BUTTON_A);

  hasWifi = false;
  InitWifi();
  if (!hasWifi)
  {
    return;
  }

  SetupMQTTClient();
}

static void SetupMQTTClient()
{
  Screen.print(3, " > IoT Hub");
  DevKitMQTTClient_Init(true);
  DevKitMQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  DevKitMQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  DevKitMQTTClient_SetReportConfirmationCallback(ReportConfirmationCallback);
  DevKitMQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);

  send_interval_ms = SystemTickCounterRead();
}

bool i2cError = false;
int sensorMotion;
int sensorPressure;
int sensorMagnetometer;
int sensorHumidityAndTemperature;
int sensorIrda;

float temperature;
float humidity;
float pressure;
int axes[3];

bool createTelemetryMessage(int messageId, char *payload)
{
  JSON_Value *root_value = json_value_init_object();
  JSON_Object *root_object = json_value_get_object(root_value);

  char *serialized_string = NULL;

  json_object_set_number(root_object, "messageId", messageId);

  json_object_set_number(root_object, "temperature", temperature);
  json_object_set_number(root_object, "humitidy", humidity);
  json_object_set_number(root_object, "pressure", pressure);
  serialized_string = json_serialize_to_string(root_value);

  snprintf(payload, MESSAGE_TELEMETRY_MAX_LEN, "%s", serialized_string);
  json_free_serialized_string(serialized_string);
  json_value_free(root_value);

  if (temperature >= TEMPERATURE_ALERT) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

int getInterval()
{
  return INTERVAL;
}

void loop()
{
  if (hasWifi)
  {
    DevKitMQTTClient_Check();

    const char *firmwareVersion = getDevkitVersion();
    const char *wifiSSID = WiFi.SSID();
    int wifiRSSI = WiFi.RSSI();
    const char *wifiIP = (const char *)WiFi.localIP().get_address();
    const char *wifiMask = (const char *)WiFi.subnetMask().get_address();
    byte mac[6];
    char macAddress[18];
    WiFi.macAddress(mac);
    snprintf(macAddress, 18, "%02x-%02x-%02x-%02x-%02x-%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    try
    {
      ext_i2c = new DevI2C(D14, D15);
      i2cError = false;
    }
    catch (int error)
    {
      i2cError = true;
      sensorMotion = 0;
      sensorPressure = 0;
      sensorMagnetometer = 0;
      sensorHumidityAndTemperature = 0;
      sensorIrda = 0;
    }

    int sensorInitResult;

    if (!i2cError)
    {
      try
      {
        acc_gyro = new LSM6DSLSensor(*ext_i2c, D4, D5);
        sensorInitResult = acc_gyro->init(NULL);
        acc_gyro->enableAccelerator();
        acc_gyro->enableGyroscope();

        if (sensorInitResult == 0)
        {
          sensorMotion = 1;
        }
        else
        {
          sensorMotion = 0;
        }
      }
      catch (int error)
      {
        sensorMotion = 0;
      }

      try
      {
        ht_sensor = new HTS221Sensor(*ext_i2c);
        sensorInitResult = ht_sensor->init(NULL);

        if (sensorInitResult == 0)
        {
          sensorHumidityAndTemperature = 1;
          ht_sensor->reset();
          ht_sensor->getTemperature(&temperature);
          ht_sensor->getHumidity(&humidity);
        }
        else
        {
          sensorHumidityAndTemperature = 0;
          humidity = -1;
          temperature = -1;
        }
      }
      catch (int error)
      {
        sensorHumidityAndTemperature = 0;
        humidity = -1;
        temperature = -1;
      }

      try
      {
        magnetometer = new LIS2MDLSensor(*ext_i2c);
        sensorInitResult = magnetometer->init(NULL);

        if (sensorInitResult == 0)
        {
          sensorMagnetometer = 1;
          magnetometer->getMAxes(axes);
        }
        else
        {
          sensorMagnetometer = 0;
          axes[0] = -1;
          axes[1] = -1;
          axes[2] = -1;
        }
      }
      catch (int error)
      {
        sensorMagnetometer = 0;
        axes[0] = -1;
        axes[1] = -1;
        axes[2] = -1;
      }

      try
      {
        pressureSensor = new LPS22HBSensor(*ext_i2c);
        sensorInitResult = pressureSensor->init(NULL);

        if (sensorInitResult == 0)
        {
          sensorPressure = 1;
          pressureSensor->getPressure(&pressure);
        }
        else
        {
          sensorPressure = 0;
          pressure = -1;
        }
      }
      catch (int error)
      {
        sensorPressure = 0;
        pressure = -1;
      }

      try
      {
        IrdaSensor = new IRDASensor();
        sensorInitResult = IrdaSensor->init();

        if (sensorInitResult == 0)
        {
          sensorIrda = 1;
        }
        else
        {
          sensorIrda = 0;
        }
      }
      catch (int error)
      {
        sensorIrda = 0;
      }
    }

    if (rgbLEDState == 0)
    {
      rgbLed.turnOff();
    }
    else
    {
      rgbLed.setColor(rgbLEDR, rgbLEDG, rgbLEDB);
    }

    pinMode(LED_USER, OUTPUT);
    digitalWrite(LED_USER, userLEDState);

    char *state;

    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);

    // Only reported properties:
    (void)json_object_set_string(root_object, "firmwareVersion", firmwareVersion);
    (void)json_object_dotset_string(root_object, "wifi.wifiSSID", wifiSSID);
    (void)json_object_dotset_number(root_object, "wifi.wifiRSSI", wifiRSSI);
    (void)json_object_dotset_string(root_object, "wifi.wifiIP", wifiIP);
    (void)json_object_dotset_string(root_object, "wifi.wifiMask", wifiMask);
    (void)json_object_dotset_string(root_object, "wifi.macAddress", macAddress);

    (void)json_object_dotset_number(root_object, "sensor.sensorMotion", sensorMotion);
    (void)json_object_dotset_number(root_object, "sensor.sensorHumidityAndTemperature", sensorHumidityAndTemperature);
    (void)json_object_dotset_number(root_object, "sensor.temperature", temperature);
    (void)json_object_dotset_number(root_object, "sensor.humidity", humidity);
    (void)json_object_dotset_number(root_object, "sensor.sensorMagnetometer", sensorMagnetometer);
    (void)json_object_dotset_number(root_object, "sensor.magnetometerAxesX", axes[0]);
    (void)json_object_dotset_number(root_object, "sensor.magnetometerAxesY", axes[1]);
    (void)json_object_dotset_number(root_object, "sensor.magnetometerAxesZ", axes[2]);
    (void)json_object_dotset_number(root_object, "sensor.sensorPressure", sensorPressure);
    (void)json_object_dotset_number(root_object, "sensor.pressure", pressure);
    (void)json_object_dotset_number(root_object, "sensor.sensorIrda", sensorIrda);

    state = json_serialize_to_string(root_value);

    json_value_free(root_value);
    if (!DevKitMQTTClient_ReportState(state))
    {
      Screen.print(3, "R.State Failed");
    }
    else
    {
      Screen.print(3, "R.State Success");
    }

    if (messageSending && (int)(SystemTickCounterRead() - send_interval_ms) >= getInterval())
    {
      char messagePayload[MESSAGE_TELEMETRY_MAX_LEN];
      bool temperatureAlert = createTelemetryMessage(messageCount, messagePayload);
      EVENT_INSTANCE* message = DevKitMQTTClient_Event_Generate(messagePayload, MESSAGE);
      DevKitMQTTClient_Event_AddProp(message, "temperatureAlert", temperatureAlert ? "true" : "false");
      DevKitMQTTClient_SendEventInstance(message);

      send_interval_ms = SystemTickCounterRead();
    }
  }
  else
  {
    // Press button A to try connect WiFi again
    if (btnAState == HIGH && digitalRead(USER_BUTTON_A) == LOW)
    {
      Screen.print(1, "Retry WiFi...", true);
      InitWifi();

      if (hasWifi)
      {
        SetupMQTTClient();
      }
    }
  }

  delay(5000);
}

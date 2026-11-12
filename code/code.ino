
#include <OneWire.h>
#include <DallasTemperature.h>



const float COOL_ON_C = 18.5;  
const float COOL_OFF_C = 18.0; 
const float HEAT_ON_C = 15.0; 
const float HEAT_OFF_C = 16.5; 

const unsigned long MIN_STATE_MS = 15000; 
const unsigned long REVERSE_DEAD_MS = 10000; 
const unsigned long PRINT_INTERVAL_MS = 2000;

// Safety limits
const float NTC_MAX_C = 70.0;
const float NTC_MIN_C = -40.0;
const float AIR_MAX_C = 40.0;
const float AIR_MIN_C = 0.0;

// Pins
const int RELAY_COOL_PIN = 11;
const int RELAY_HEAT_PIN = 10;
const bool RELAY_ACTIVE_LOW = true;
const int DS_PIN_1 = 8;
const int DS_PIN_2 = 9;
const int NTC_PINS[4] = {A0, A1, A2, A3};

float PULLUP_R[4] = {32000.0, 32000.0, 32000.0, 32000.0};
const float NTC_R_NOMINAL = 10000.0;
const float NTC_BETA = 3950.0;
const float NTC_T_NOMINAL = 25.0;

OneWire ow1(DS_PIN_1);
OneWire ow2(DS_PIN_2);
DallasTemperature ds1(&ow1);
DallasTemperature ds2(&ow2);

enum Mode { MODE_OFF, MODE_HEAT, MODE_COOL };
enum RunState { RUNNING, STOPPED, FAULT };

Mode mode = MODE_OFF;
Mode lastActiveMode = MODE_OFF;
RunState runState = RUNNING;

float air1C = NAN, air2C = NAN, airC = NAN;
float ntcC[4] = {NAN, NAN, NAN, NAN};

unsigned long modeSince = 0;
unsigned long lastPrint = 0;
unsigned long dsRequestedAt = 0;
String faultMsg = "";

void writeRelay(int pin, bool on) {
  digitalWrite(pin, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setMode(Mode m) {
  if (m == mode) 
  {
    return;
  }
  if (mode != MODE_OFF)
  {
    lastActiveMode = mode;
  }
  writeRelay(RELAY_COOL_PIN, false);
  writeRelay(RELAY_HEAT_PIN, false);
  delay(50);
  mode = m;
  modeSince = millis();
  writeRelay(RELAY_COOL_PIN, mode == MODE_COOL);
  writeRelay(RELAY_HEAT_PIN, mode == MODE_HEAT);
}

float readNtcC(int i) 
{
  long sum = 0;
  for (int s = 0; s < 8; s++)
  {
    sum += analogRead(NTC_PINS[i]);
  }
  float adc = sum / 8.0;
  if (adc < 5 || adc > 1018)
  {
    return NAN;
  }
  float r = PULLUP_R[i] * adc / (1023.0 - adc);
  float invT = 1.0 / (NTC_T_NOMINAL + 273.15) + log(r / NTC_R_NOMINAL) / NTC_BETA;
  return 1.0 / invT - 273.15;
}

float validDs(float t) 
{
  return (t == DEVICE_DISCONNECTED_C) ? NAN : t;
}

void readSensors() 
{
  for (int i = 0; i < 4; i++)
  {
    ntcC[i] = readNtcC(i);
  }
  if (millis() - dsRequestedAt >= 800) 
  {
    air1C = validDs(ds1.getTempCByIndex(0));
    air2C = validDs(ds2.getTempCByIndex(0));

    if (!isnan(air1C) && !isnan(air2C)) airC = (air1C + air2C) / 2.0;
    else if (!isnan(air1C)) airC = air1C;
    else if (!isnan(air2C)) airC = air2C;
    else airC = NAN;

    ds1.requestTemperatures();
    ds2.requestTemperatures();
    dsRequestedAt = millis();
  }
}

void fault(const String &msg) 
{
  setMode(MODE_OFF);
  runState = FAULT;
  faultMsg = msg;
  Serial.print(F("FAULT: "));
  Serial.println(msg);
}

void checkSafety() 
{
  for (int i = 0; i < 4; i++) 
  {
    if (isnan(ntcC[i]))      { fault("NTC " + String(i + 1) + " open/shorted"); return; }
    if (ntcC[i] > NTC_MAX_C) { fault("TEC " + String(i + 1) + " too hot"); return; }
    if (ntcC[i] < NTC_MIN_C) { fault("TEC " + String(i + 1) + " too cold"); return; }
  }
  if (isnan(airC))
  {
    fault("Both DS18B20s not responding"); 
    return;
  }
  if (airC > AIR_MAX_C)
  {
    fault("Chamber air too hot"); 
    return;
  }
  if (airC < AIR_MIN_C)
  {
    fault("Chamber air too cold"); 
    return;
  }
}

void control() {
  unsigned long inMode = millis() - modeSince;
  if (inMode < MIN_STATE_MS) 
  {
    return;
  }
  Mode want = mode;
  switch (mode) 
  {
    case MODE_OFF:
      if (airC > COOL_ON_C) want = MODE_COOL;
      else if (airC < HEAT_ON_C) want = MODE_HEAT;
      break;
    case MODE_COOL:
      if (airC <= COOL_OFF_C) want = MODE_OFF;
      break;
    case MODE_HEAT:
      if (airC >= HEAT_OFF_C) want = MODE_OFF;
      break;
  }
  if (mode == MODE_OFF && want != MODE_OFF &&
      want != lastActiveMode && lastActiveMode != MODE_OFF &&
      inMode < REVERSE_DEAD_MS) return;

  setMode(want);
}

const char *modeName() 
{
  return mode == MODE_HEAT ? "HEAT" : mode == MODE_COOL ? "COOL" : "OFF";
}

void printStatus() {
  Serial.print(F("air="));   Serial.print(airC, 2);
  Serial.print(F(" (DS1=")); Serial.print(air1C, 1);
  Serial.print(F(" DS2="));  Serial.print(air2C, 1);
  Serial.print(F(") mode=")); Serial.print(modeName());
  Serial.print(F(" tec="));
  for (int i = 0; i < 4; i++)
  {
    Serial.print(ntcC[i], 1); 
    Serial.print(i < 3 ? ',' : ' ');
  }
  Serial.print(F("state="));
  if (runState == RUNNING) 
  {
    Serial.println(F("RUN"));
  }
  else if (runState == STOPPED)
  {
    Serial.println(F("STOP"));
  }
  else
  {
    Serial.print(F("FAULT(")); 
    Serial.print(faultMsg); 
    Serial.println(')');
  }
}

void handleSerial() {
  if (!Serial.available()) 
  {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) 
  {
    return;
  }
  char c = toupper(line.charAt(0));

  if (c == 'O') 
  {
    setMode(MODE_OFF); runState = STOPPED; 
    Serial.println(F("Stopped"));
  } 
  else if (c == 'R') 
  {
    runState = RUNNING; faultMsg = ""; 
    Serial.println(F("Running"));
  } 
  else if (c == '?') 
  {
    printStatus();
  }
}

void setup() {
  pinMode(RELAY_COOL_PIN, OUTPUT);
  pinMode(RELAY_HEAT_PIN, OUTPUT);
  writeRelay(RELAY_COOL_PIN, false);
  writeRelay(RELAY_HEAT_PIN, false);

  for (int i = 0; i < 4; i++) 
  {
    pinMode(NTC_PINS[i], INPUT_PULLUP);
  }

  Serial.begin(115200);
  ds1.begin(); ds2.begin();
  ds1.setResolution(12); ds2.setResolution(12);
  ds1.setWaitForConversion(false); ds2.setWaitForConversion(false);
  ds1.requestTemperatures(); ds2.requestTemperatures();
  dsRequestedAt = millis();
  delay(800);

  modeSince = millis() - MIN_STATE_MS;  
  Serial.println(F("TEC chamber TEST: cool to 18, heat only below 15"));
}

void loop() {
  handleSerial();
  readSensors();

  if (runState == RUNNING) 
  {
    checkSafety();
    if (runState == RUNNING) 
    {
      control();
    }
  } 
  else if (mode != MODE_OFF) 
  {
    setMode(MODE_OFF);
  }

  if (millis() - lastPrint >= PRINT_INTERVAL_MS) 
  {
    printStatus();
    lastPrint = millis();
  }
}

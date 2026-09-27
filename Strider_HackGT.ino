#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

const int rightMotorPin = 18;
const int leftMotorPin = 19;

const int pwmFreq = 50;
const int pwmBits = 14;
const int neutralPWM = 1500;

const float maxPower = 40.0;
const unsigned long assistTime = 400;
const unsigned long rampUpTime = 100;
const unsigned long rampDownTime = 300;
const unsigned long cooldownTime = 600;

const float stepSpeed = 15.0;
const int stepSamples = 2;

Adafruit_MPU6050 rightIMU;
Adafruit_MPU6050 leftIMU;
TwoWire leftWire = TwoWire(1);

float rightBias = 0;
float leftBias = 0;

bool rightAssist = false;
bool leftAssist = false;

unsigned long rightAssistStart = 0;
unsigned long leftAssistStart = 0;
unsigned long rightCooldown = 0;
unsigned long leftCooldown = 0;

int rightCount = 0;
int leftCount = 0;


#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3

void setupPWM()
{
  ledcAttach(rightMotorPin, pwmFreq, pwmBits);
  ledcAttach(leftMotorPin, pwmFreq, pwmBits);
}

void writeDuty(int pin, uint32_t duty)
{
  ledcWrite(pin, duty);
}

#else

const int rightChannel = 0;
const int leftChannel = 1;

void setupPWM()
{
  ledcSetup(rightChannel, pwmFreq, pwmBits);
  ledcSetup(leftChannel, pwmFreq, pwmBits);

  ledcAttachPin(rightMotorPin, rightChannel);
  ledcAttachPin(leftMotorPin, leftChannel);
}

void writeDuty(int pin, uint32_t duty)
{
  if (pin == rightMotorPin)
    ledcWrite(rightChannel, duty);
  else
    ledcWrite(leftChannel, duty);
}

#endif


uint32_t pulseToDuty(int pulse)
{
  float period = 1000000.0 / pwmFreq;
  uint32_t maxDuty = (1UL << pwmBits) - 1;

  return (uint32_t)(((float)pulse / period) * maxDuty + 0.5);
}


void setMotor(int pin, int pulse)
{
  writeDuty(pin, pulseToDuty(pulse));
}


void stopMotors()
{
  setMotor(rightMotorPin, neutralPWM);
  setMotor(leftMotorPin, neutralPWM);
}


float smoothStep(float x)
{
  if (x <= 0)
    return 0;

  if (x >= 1)
    return 1;

  return x * x * (3 - 2 * x);
}


float getPower(unsigned long elapsed)
{
  if (elapsed >= assistTime)
    return 0;

  if (elapsed < rampUpTime)
  {
    float x = (float)elapsed / rampUpTime;
    return maxPower * smoothStep(x);
  }

  float x = (float)(elapsed - rampUpTime) / rampDownTime;
  return maxPower * (1 - smoothStep(x));
}


int rightPWM(float power)
{
  return neutralPWM - (int)(500.0 * power / 100.0 + 0.5);
}


int leftPWM(float power)
{
  return neutralPWM + (int)(500.0 * power / 100.0 + 0.5);
}


bool getLegSpeed(float &rightSpeed, float &leftSpeed)
{
  sensors_event_t ra, rg, rt;
  sensors_event_t la, lg, lt;

  if (!rightIMU.getEvent(&ra, &rg, &rt))
    return false;

  if (!leftIMU.getEvent(&la, &lg, &lt))
    return false;

  float rightRaw = rg.gyro.z * 180.0 / PI;
  float leftRaw = lg.gyro.z * 180.0 / PI;

  rightSpeed = rightRaw - rightBias;
  leftSpeed = -(leftRaw - leftBias);

  return true;
}


bool calibrate()
{
  stopMotors();

  delay(3000);

  float rightSum = 0;
  float leftSum = 0;
  int samples = 0;

  unsigned long start = millis();

  while (millis() - start < 2000)
  {
    sensors_event_t ra, rg, rt;
    sensors_event_t la, lg, lt;

    if (!rightIMU.getEvent(&ra, &rg, &rt))
      return false;

    if (!leftIMU.getEvent(&la, &lg, &lt))
      return false;

    rightSum += rg.gyro.z * 180.0 / PI;
    leftSum += lg.gyro.z * 180.0 / PI;
    samples++;

    stopMotors();
    delay(10);
  }

  if (samples == 0)
    return false;

  rightBias = rightSum / samples;
  leftBias = leftSum / samples;

  return true;
}


void checkRightStep(float speed, unsigned long time)
{
  if (rightAssist || time < rightCooldown)
  {
    rightCount = 0;
    return;
  }

  if (speed >= stepSpeed)
    rightCount++;
  else
    rightCount = 0;

  if (rightCount >= stepSamples)
  {
    rightAssist = true;
    rightAssistStart = time;
    rightCount = 0;
  }
}


void checkLeftStep(float speed, unsigned long time)
{
  if (leftAssist || time < leftCooldown)
  {
    leftCount = 0;
    return;
  }

  if (speed >= stepSpeed)
    leftCount++;
  else
    leftCount = 0;

  if (leftCount >= stepSamples)
  {
    leftAssist = true;
    leftAssistStart = time;
    leftCount = 0;
  }
}


float updateRightAssist(unsigned long time)
{
  if (!rightAssist)
    return 0;

  unsigned long elapsed = time - rightAssistStart;

  if (elapsed >= assistTime)
  {
    rightAssist = false;
    rightCooldown = time + cooldownTime;
    return 0;
  }

  return getPower(elapsed);
}


float updateLeftAssist(unsigned long time)
{
  if (!leftAssist)
    return 0;

  unsigned long elapsed = time - leftAssistStart;

  if (elapsed >= assistTime)
  {
    leftAssist = false;
    leftCooldown = time + cooldownTime;
    return 0;
  }

  return getPower(elapsed);
}


void setup()
{
  setupPWM();
  stopMotors();

  Wire.begin(21, 22);
  leftWire.begin(32, 33);

  if (!rightIMU.begin(0x68, &Wire))
  {
    while (true)
    {
      stopMotors();
      delay(100);
    }
  }

  if (!leftIMU.begin(0x68, &leftWire))
  {
    while (true)
    {
      stopMotors();
      delay(100);
    }
  }

  rightIMU.setGyroRange(MPU6050_RANGE_500_DEG);
  leftIMU.setGyroRange(MPU6050_RANGE_500_DEG);

  rightIMU.setFilterBandwidth(MPU6050_BAND_44_HZ);
  leftIMU.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!calibrate())
  {
    while (true)
    {
      stopMotors();
      delay(100);
    }
  }
}


void loop()
{
  float rightSpeed = 0;
  float leftSpeed = 0;

  if (!getLegSpeed(rightSpeed, leftSpeed))
  {
    stopMotors();
    delay(100);
    return;
  }

  unsigned long time = millis();

  checkRightStep(rightSpeed, time);
  checkLeftStep(leftSpeed, time);

  float rightPower = updateRightAssist(time);
  float leftPower = updateLeftAssist(time);

  setMotor(rightMotorPin, rightPWM(rightPower));
  setMotor(leftMotorPin, leftPWM(leftPower));

  delay(5);
}

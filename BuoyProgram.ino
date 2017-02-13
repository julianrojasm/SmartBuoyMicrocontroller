

//#include <TimerOne.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

//Bluetooth
SoftwareSerial BT(12,13);

//air and water temp sensors
#define airTemp 2
#define waterTemp 3
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire2(airTemp);
OneWire oneWire3(waterTemp);

 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors2(&oneWire2);
DallasTemperature sensors3(&oneWire3);


//Setup Variables for WIND SPEED
const int sensorPin = A3; //Defines the pin that the anemometer output is connected to
int sensorValue = 0; //Variable stores the value direct from the analog pin
float sensorVoltage = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeed = 0; // Wind speed in meters per second (m/s)
 
float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
int sensorDelay = 1000; //Delay between sensor readings, measured in milliseconds (ms)
 
//Anemometer Technical Variables
//The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.
 
float voltageMin = .4; // Mininum output voltage from anemometer in mV.
float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage
 
float voltageMax = 2.0; // Maximum output voltage from anemometer in mV.
float windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage

float windFactor = 2.23694*windSpeedMax/(voltageMax - voltageMin);

//Setup Variables for SALINITY
const int sensorPinSalt0 = A0;   //well be using this one for demo
const int sensorPinSalt1 = A1; 


//function prototypes
float getTemp(DallasTemperature sensor);
float getWindSpeed();
void getWaveAccel();
int readAnalogSalt();


//Accelerometer and Gyroscope setup


/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

//variables needed for detecting wave height
float xComp, yComp, zComp, zAccelTotal, pitch, roll, accelX, accelY, accelZ, zAccelTotalcomp;
float toRad = (3.14 / 180);
int size = 150, i;
double accele[150];
double reading, one;

//variables needed for Gyroscope and Rotation

float pitchGyro, rollGyro;


void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
}
 
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  // Start Bluetooth Comunication
  BT.begin(9600);
  delay(2000);

  initSensors();
  
  // Start up the library
  sensors2.begin();
  sensors3.begin();
}

int caseNumber;
float tempSensorA;
float tempSensorW;
float windS;
int salinityAnalog;

void loop()
{
  if(BT.available())
  {
    caseNumber = BT.read();
    Serial.write(caseNumber);
    switch(caseNumber)
    {
      //AIR TEMP
      case '1':
      tempSensorA = getTemp(sensors2);
      BT.print('?');
      BT.print(tempSensorA);
      //delay(500);
      BT.print('#');
      //delay(500);
      break;
      
      //WATER TEMP
      case '2':
      tempSensorW = getTemp(sensors3);
      BT.print('!');
      BT.print(tempSensorW);
      delay(500);
      BT.print('#');
      delay(500);
      break;
      
      //WIND SPEED
      case '3':
      windS = getWindSpeed();
      BT.print('*');
      BT.print(windS);
      BT.print('#');
      break;

      //WAVE HEIGHT
      case '4':
      getWaveAccel();
      BT.print('$');
      Serial.print('$');
      for (i = 0; i < size; i++)
      {
        BT.print(accele[i]);
        BT.print('&');
        delay(20);
        Serial.print(accele[i]);
        Serial.print('&');
      }
      BT.print('#');
      Serial.print('#');
      break;

      //SALINITY
      case '5':
      salinityAnalog = readAnalogSalt();
      Serial.print(salinityAnalog);
      BT.print('%');
      BT.print(salinityAnalog);
      BT.print('#');
      break;

      //FOR ALL SENSORS!
      case '6':
      
      int infinity = 1;
      getRotation();
      BT.print('^');
      Serial.println(pitchGyro);
      BT.print(pitchGyro);
      BT.print('@');
      BT.print(rollGyro); 
      Serial.println(rollGyro);
      BT.print('#'); 
      Serial.println('#');
      delay(100);
        
      break;
      
    }
    
  }
}


float getTemp(DallasTemperature sensor)
{
  sensor.requestTemperatures();
  return sensor.getTempCByIndex(0);
}

float getWindSpeed()
{
    //Get a value from 0 to 1023 from analog pin connected to anenometer
  sensorValue = analogRead(sensorPin);
  sensorVoltage = sensorValue * voltageConversionConstant;
  if(sensorVoltage <= voltageMin)
  {
    windSpeed = 0;
  }
  else windSpeed = ( ( ( sensorVoltage - voltageMin ) * windFactor ) * 2.23694 );
  return windSpeed;
}

void getRotation()
{    
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
  
    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
  
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
      pitchGyro = orientation.pitch;
      rollGyro = orientation.roll;
    }
}

void getWaveAccel()
{ 
  for( i = 0 ; i < size; i++)
  {
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
  
    /* Calculate pitch and roll from the raw accelerometer data */
    accel.getEvent(&accel_event);
  
    if (dof.accelGetOrientation(&accel_event, &orientation))
    {
      accelX = accel_event.acceleration.x;
      accelY = accel_event.acceleration.y;
      accelZ = accel_event.acceleration.z;
      pitch = orientation.pitch;
      roll = orientation.roll;
    }
    
    xComp = ( sin(pitch * toRad) * accelX); 
    yComp = (sin(roll * toRad) * cos(pitch * toRad) * accelY);
    zComp = (cos(roll * toRad) * cos(pitch * toRad) * accelZ);

    zAccelTotalcomp = ( ( xComp + yComp + zComp) - .80 );

  
    if (zAccelTotalcomp > 9.60 && zAccelTotalcomp < 10.0)
    {
      accele[i] = 0;
    }
    else
    {
      accele[i] = zAccelTotalcomp - 9.81;
    }

    Serial.print( accele[i] );
    delay(30);
//    currentTime = millis();
//    totalTime = currentTime-initialTime;
//    //BT.println(totalTime);
      
  }
}

int readAnalogSalt()
{
  int valueRead = analogRead(sensorPinSalt0);
  return valueRead;
  
}














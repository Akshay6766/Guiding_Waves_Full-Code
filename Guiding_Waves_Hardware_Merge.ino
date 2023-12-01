/*

Wiring Instructions 

Ultrasonic Sensor

trigpin1 9 orange sensor front ul
echopin2 5 red sensor front ul
trigpin2 4 yellow right sensor ul
echopin1 6 green right sensor ul
trigpin3 7 brown left sensor ul
echopin3 8 black left sensor ul

all ul sensor 5v and gnd to breadboard power supply (light red as 5v and grey as gnd)
breadboard powersupply is powerd by 2x18650 battery delivering 8v at peak

GPS

gps rx 2 grey
gps tx 3 white
gps 5v violet to breadboard xtended 5v
gps gnd black to breadboard xtended GND

pressure sensor 

SCL white to arduino A5 
SDA black to arduino A4
VIN violet to arduino 3.3v female pin general
GND grey to arduino gnd female pin general

Gyroscope Sensor on Breadboard

SCL white to SCL pin on arduino female pin
SDA black to SDA pin on arduino female pin
5V Violet pin to arduino 5v female pin general
GND Grey pin to arduino gnd female pin general

Rescue Laser X2

5v on extended  DigitalPIN 12 on arduino female to breadboard 
GND  on extended Common GND on arduino female to breadboard 

Pressure Motor

Digital Pin 13
GND on extended Common GND on arduino female to breadboard 

Motor Right

Digital Pin 11
GND on extended Common GND on arduino female to breadboard 

Motor Left

Digital Pin 10
GND on extended Common GND on arduino female to breadboard 

Leftover Pins

0,1,A0,A1,A2,A3

Guiding Waves Primary Code
*/




#include <Wire.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "I2Cdev.h"
#include "MPU6050.h"


SoftwareSerial mySerial(2, 3); // RX, TX
TinyGPS gps;

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

#define ALTITUDE 1655.0

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
int16_t ax, ay, az,gx, gy, gz;
float accAngleX, accAngleY;
int starttime = 0;
int endtime = 0;
int loopcount =0;

SFE_BMP180 bmp180;
MPU6050 accelgyro; //(0x68); // <-- use for AD0 high
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

#define trigPin1 9
#define echoPin1 6
#define trigPin2 4
#define echoPin2 5
#define trigPin3 7
#define echoPin3 8
//#define blue 13
//#define green 11
//#define red 12

long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor,last_distance,last_distance1,last_distance2,last_distance3;
int value = 10;


void setup()
{

Serial.begin(115200);
mySerial.begin(115200);

pinMode(12, OUTPUT); //right laser and left on one pin
pinMode(11, OUTPUT); //left motor
pinMode(10, OUTPUT);//right motor
pinMode(13, OUTPUT);//right motor

//gps side
  delay(1000);

  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();

//gyro side

  // initialize device
  accelgyro.initialize();
  

 
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();


  // verify connection
  accelgyro.testConnection() ;

  delay(1000);
  
  
  accelgyro.setXAccelOffset(1816);
  accelgyro.setYAccelOffset(1134);
  accelgyro.setZAccelOffset(4736);
  accelgyro.setXGyroOffset(60);
  accelgyro.setYGyroOffset(71);
  accelgyro.setZGyroOffset(19);
  

//UL Side
pinMode(trigPin1, OUTPUT);
pinMode(echoPin1, INPUT);
pinMode(trigPin2, OUTPUT);
pinMode(echoPin2, INPUT);
pinMode(trigPin3, OUTPUT);
pinMode(echoPin3, INPUT);

//Pressure Sensor
bool success = bmp180.begin();

if (success) {
  Serial.println("BMP180 init success");
  }


}


void loop() {


digitalWrite(13,HIGH);
delay(100);
digitalWrite(13,LOW);
 
  bool newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 1000) {
    if (mySerial.available()) {
      char c = mySerial.read();
      // Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) {
        newdata = true;
        // break;  // uncomment to print new data immediately!
      }
    }
  }
  
  if (newdata) {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }

//Pressure sensor side

  char status;
  double T, P,p0,a;
  bool success = false;

Serial.println();
Serial.print("Altitude : ");
Serial.print(ALTITUDE,0);
Serial.print(" meters");


  status = bmp180.startTemperature();
  if (status != 0) {
    delay(1000);
    status = bmp180.getTemperature(T);

    if (status != 0) {
      status = bmp180.startPressure(3);

      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P, T);

        if (status != 0) {
          Serial.println();
          Serial.print("Pressure: ");
          Serial.print(P);
          Serial.println(" hPa");
          

          Serial.print("Temperature: ");
          Serial.print(T);
          Serial.println(" C"); 

          if (P > 6058.25 ) //pressure at 0.5 M depth
            {
              Serial.println("I M Drowning");
            //  digitalWrite(9,HIGH);
              digitalWrite(12,HIGH);
              starttime = millis();
              endtime = starttime;
              while ((endtime - starttime) <=11000) 
                {
                digitalWrite(13,HIGH);
                Serial.println("Airbag Inflation started");
                delay(10000);//Time for Max Inflation 10s
                digitalWrite(13,LOW);
                loopcount = loopcount+1;
                endtime = millis();
                }
              Serial.print ("Airbag Inflated Max");



            }
            else 
            {
              //digitalWrite(9,LOW);
              digitalWrite(12,LOW);
              digitalWrite(13,LOW);
            }
          
        }
      }
    }
  }


//gyro side main

  gyro_signals();
    
  Serial.println();
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  float angle = getAngle();

  // You can add a threshold to detect a rollover event
  if (abs(angle) < 20 ) {
    Serial.println("Rollover detected!");
    digitalWrite(9,HIGH);
    //digitalWrite(12,HIGH);
    
    
   
  }
  if (abs(angle) > 175 && angle <180) 
    {
      Serial.println("Gyro Level");
      digitalWrite(8,LOW);
      digitalWrite(12,LOW);
      digitalWrite(11,LOW);
      digitalWrite(10,LOW);
    }
  if (abs(angle) < 150 && angle > 20)
    {
      Serial.println("Full Gyro right");
      
      digitalWrite(11,HIGH);
      delay(200);
      digitalWrite(11,LOW);
      digitalWrite(8,LOW);
      digitalWrite(12,LOW);
      digitalWrite(10,LOW);
    }
  if (abs(angle) > -160 && angle <-100)
    {
      Serial.println("Full Gyro left");

     
      digitalWrite(10,HIGH);
      delay(200);
      digitalWrite(10,LOW);
      digitalWrite(8,LOW);
      digitalWrite(12,LOW);
      digitalWrite(11,LOW);
     
    }
      
    

  delay(100);




//UL side main

SonarSensor(trigPin3, echoPin1);
RightSensor = distance;
SonarSensor(trigPin3, echoPin2);
FrontSensor = distance;
SonarSensor(trigPin3, echoPin3);
LeftSensor = distance;

Serial.println();
Serial.print("Front Ultrasonic : ");
Serial.print(FrontSensor);
Serial.print(" cm");
Serial.println();
Serial.print("Left Ultrasonic  : ");
Serial.print(LeftSensor);
Serial.print(" cm");
Serial.println();
Serial.print("Right Ultrasonic : ");
Serial.print(RightSensor);
Serial.print(" cm");
Serial.println();

if (LeftSensor < value && FrontSensor < value && RightSensor < value)
  {
    Serial.println("Obstacle coverd Go Back");
      digitalWrite(10,HIGH);
      digitalWrite(11,HIGH);
      delay(200);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
  }
    else if (LeftSensor < value &&  RightSensor < value )
      {
        Serial.println("Obstacle On sides Go Straight ");
        
      } 
      else if (LeftSensor < value && FrontSensor < value)
        {
          Serial.println("Obstacle On front and Left Side Go Right");
          digitalWrite(11,HIGH);
          delay(300);
          digitalWrite(11,LOW);
        }
        else if (FrontSensor < value && RightSensor < value)
          {
            Serial.println("Obstacle On front and right Side Go Left");
            digitalWrite(10,HIGH);
            delay(300);
            digitalWrite(10,LOW);
          }
          else if (LeftSensor < value)
           {
              Serial.println("Obstacle on Left Sensor");
              digitalWrite(10,HIGH);
              delay(300);
              digitalWrite(10,LOW);
           }
           else if (FrontSensor < value)
            {
              Serial.println("Obstacle on Front Sensor");
              if (RightSensor < value)
              {
                 digitalWrite(11,HIGH);
                 delay(300);
                 digitalWrite(11,LOW);
              }
              if (LeftSensor < value)
              {
                 digitalWrite(11,HIGH);
                 delay(300);
                 digitalWrite(11,LOW);
              }
                
              
            }
            else if (RightSensor < value)
              {
                Serial.println(" Obstacle on RightSensor");
                 digitalWrite(11,HIGH);
                 delay(300);
                 digitalWrite(11,LOW);
              }
      
   else
  {
   // digitalWrite(red,   LOW);
    //digitalWrite(green, LOW);
    //digitalWrite(blue,  LOW);
    //digitalWrite(13,    LOW);
    Serial.println("Chuck Chuck No Obstacle");
    digitalWrite(10,LOW);
    digitalWrite(11,LOW);
  }



delay(150); //ensure smooth function of block 



  

 


}

float getAngle()  {
  float angle = atan2( AccY, AccZ) * 180 / PI;
  Serial.println();
  Serial.print("Gyro Angle : ");
  Serial.println(angle);
  Serial.println();
  abs(angle);
  return angle;
  }



void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance= duration*0.034/2;

}

void gyro_signals(void) {

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  // On Arduino, GPS characters may be lost during lengthy Serial.print()
  // On Teensy, Serial prints to USB, which has large output buffering and
  //   runs very fast, so it's not necessary to worry about missing 4800
  //   baud GPS characters.

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): ");
    Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); 
    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); 
    Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));
    Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");

  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): ");
    Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude()); Serial.print(" Course(float): ");
    printFloat(gps.f_course()); Serial.println();
  Serial.print("Speed(knots): "); printFloat(gps.f_speed_knots()); Serial.print(" (mph): ");
    printFloat(gps.f_speed_mph());
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps()); Serial.print(" (kmph): ");
    printFloat(gps.f_speed_kmph()); Serial.println();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: ");
    Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0) {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}

/*void clearSerialMonitor() {
  // Send a series of newline characters to clear the Serial Monitor
  for (int i = 0; i < 60; i++) {
    Serial.println();
   
  }

}*/

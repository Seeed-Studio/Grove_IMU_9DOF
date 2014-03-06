#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];
float sumAxyz[3];
float sumGxyz[3];
float sumMxyz[3];

#define LED_PIN 13
bool blinkState = false;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}

void loop() 
{
 int i;
 for(i=0;i<30;i++)
  // read raw accel/gyro measurements from device
  {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  getAccelValue();
  getGyroValue();
  getCompassValue();
  getHeading();
  getTiltHeading();    // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  // display tab-separated accel/gyro x/y/z values
  
  }
  
  for(i=0;i<3;i++)
  {
    
    Axyz[i]=sumAxyz[i]/30.0;
    Gxyz[i]=sumGxyz[i]/30.0;
    Mxyz[i]=sumMxyz[i]/30.0;
    sumAxyz[i] =0;
    sumGxyz[i]=0;
    sumMxyz[i]=0;
  }
  
  Serial.println("Acceleration(g) of X,Y,Z:");
  Serial.print(Axyz[0]); 
  Serial.print(",");
  Serial.print(Axyz[1]); 
  Serial.print(",");
  Serial.println(Axyz[2]); 
  Serial.println("Gyro(degress/s) of X,Y,Z:");
  Serial.print(Gxyz[0]); 
  Serial.print(",");
  Serial.print(Gxyz[1]); 
  Serial.print(",");
  Serial.println(Gxyz[2]); 
  Serial.println("Compass Value of X,Y,Z:");
  Serial.print(Mxyz[0]); 
  Serial.print(",");
  Serial.print(Mxyz[1]); 
  Serial.print(",");
  Serial.println(Mxyz[2]);
  Serial.println("The clockwise angle between the magnetic north and X-Axis:");
  Serial.print(heading);
  Serial.println(" ");
  Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
  Serial.println(tiltheading);
   Serial.println("   ");
  Serial.println("   ");
     Serial.println("   ");
  Serial.println("   ");
     Serial.println("   ");
  Serial.println("   ");
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  //delay(100);
  
}


void getAccelValue(void)
{
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
  sumAxyz[0] += Axyz[0];
  sumAxyz[1] += Axyz[1];
  sumAxyz[2] += Axyz[2];
  
}
void getGyroValue(void)
{
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
  sumGxyz[0] += Gxyz[0];
  sumGxyz[1] += Gxyz[1];
  sumGxyz[2] += Gxyz[2];
}
void getCompassValue(void)
{
  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
  sumMxyz[0] += Mxyz[0];
  sumMxyz[1] += Mxyz[1];
  sumMxyz[2] += Mxyz[2];
}
void getHeading(void)
{
  heading=180*atan2(Mxyz[1],Mxyz[0])/PI;
  if(heading <0) heading +=360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1]/cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh)/PI;
  if(yh<0)    tiltheading +=360;
}

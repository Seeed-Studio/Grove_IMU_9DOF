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


#define sample_num_mdate  5000      

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max =0;
volatile int my_max =0;
volatile int mz_max =0;

volatile int mx_min =0;
volatile int my_min =0;
volatile int mz_min =0;


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
	
	delay(1000);
	Serial.println("     ");
 
	Mxyz_init_calibrated ();
  
}

void loop() 
{   
	
	getAccel_Data();
	getGyro_Data();
	getCompassDate_calibrated(); // compass data has been calibrated here 
	getHeading();				//before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .					
	getTiltHeading();           
	
	Serial.println("calibration parameter: ");
	Serial.print(mx_centre);
	Serial.print("         ");
	Serial.print(my_centre);
	Serial.print("         ");
	Serial.print(mz_centre);
	Serial.println("     ");


	
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



void Mxyz_init_calibrated ()
{
	
	Serial.println(F("Before calibratting ,we should sample frist"));
	Serial.print("  ");
	Serial.println(F("It will takes about 2 minutes to finish data collection, which means you should  rotate and turn the board all the time within 2 minutes."));	
	Serial.print("  ");
	Serial.println(F("During  samplling ,you should make the 9DOF rotate 360 degrees around the X axis,as well as the Y aixs and the Z axis . "));
	Serial.print("  ");
	Serial.println(F("If you have prepared well ,please sent a command data 'ready' to start samplling   "));
	while(!Serial.find("ready"));	
	Serial.println("  ");
	Serial.println("ready");
	Serial.println("Sample starting......");
	Serial.println("waiting ......");
	
	get_calibration_Data ();
	
	Serial.println("     ");
	Serial.println("compass calibration parameter ");
	Serial.print(mx_centre);
	Serial.print("     ");
	Serial.print(my_centre);
	Serial.print("     ");
	Serial.println(mz_centre);
	Serial.println("    ");
}


void get_calibration_Data ()
{
		for (int i=0; i<sample_num_mdate;i++)
			{
			get_one_sample_date_mxyz();
			
			if (mx_sample[2]>=mx_sample[1])mx_sample[1] = mx_sample[2];			
			if (my_sample[2]>=my_sample[1])my_sample[1] = my_sample[2]; //find max value			
			if (mz_sample[2]>=mz_sample[1])mz_sample[1] = mz_sample[2];		
			
			if (mx_sample[2]<=mx_sample[0])mx_sample[0] = mx_sample[2];
			if (my_sample[2]<=my_sample[0])my_sample[0] = my_sample[2];//find min value
			if (mz_sample[2]<=mz_sample[0])mz_sample[0] = mz_sample[2];
						
			}
			
			mx_max = mx_sample[1];
			my_max = my_sample[1];
			mz_max = mz_sample[1];			
					
			mx_min = mx_sample[0];
			my_min = my_sample[0];
			mz_min = mz_sample[0];
	
			mx_centre = (mx_max + mx_min)/2;
			my_centre = (my_max + my_min)/2;
			mz_centre = (mz_max + mz_min)/2;	
	
}






void get_one_sample_date_mxyz()
{		
		getCompass_Data();
		mx_sample[2] = Mxyz[0];
		my_sample[2] = Mxyz[1];
		mz_sample[2] = Mxyz[2];
}	


void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384; 
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassDate_calibrated ()
{
	getCompass_Data();
	Mxyz[0] = Mxyz[0] - mx_centre;
	Mxyz[1] = Mxyz[1] - my_centre;
	Mxyz[2] = Mxyz[2] - mz_centre;	
}
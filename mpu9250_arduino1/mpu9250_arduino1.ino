// Modified from: https://www.lucidar.me/en/inertial-measurement-unit/mpu-9250-and-arduino-9-axis-imu/


#include <Wire.h>
#include <TimerOne.h>
#include <MadgwickAHRS.h>



#define    MPU9250_ADDRESS_LOW        0x68
#define    MPU9250_ADDRESS_HIGH       0x69

#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

// From taking the mean over 1000 data points for each measurement.
#define    AX_BIAS    -610 // subtracted 15 from -595
#define    AY_BIAS    -168 // added 10 to -178
#define    AZ_BIAS    1334
#define    GX_BIAS    7
#define    GY_BIAS    -94
#define    GZ_BIAS    -35

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Initial time
long int ti;
volatile bool intFlag=false;

void setupFilter() {
  filter.begin(25);
  
  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
}

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(115200);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS_HIGH,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS_HIGH,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS_HIGH,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS_HIGH,28,ACC_FULL_SCALE_2_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS_HIGH,0x37,0x02);
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
  pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  
  // Store initial time
  ti=millis();
}





// Counter
long int cpt=0;

void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}

// Main loop, read and display data
void loop()
{
  while (!intFlag);
  intFlag=false;
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
   
    // ____________________________________
    // :::  accelerometer and gyroscope ::: 
  
    // Read accelerometer and gyroscope
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS_HIGH,0x3B,14,Buf);
    
    // Create 16 bits values from 8 bits data
    
    // Accelerometer
    int16_t ax=-(Buf[0]<<8 | Buf[1]);
    int16_t ay=-(Buf[2]<<8 | Buf[3]);
    int16_t az=Buf[4]<<8 | Buf[5];
  
    // Gyroscope
    int16_t gx=-(Buf[8]<<8 | Buf[9]);
    int16_t gy=-(Buf[10]<<8 | Buf[11]);
    int16_t gz=Buf[12]<<8 | Buf[13];

    // filter.updateIMU(gx * 1.0, gy * 1.0, gz * 1.0, ax / 9800.0, ay / 9800.0, az / 9800.0);
    filter.updateIMU(gx - GX_BIAS, gy - GY_BIAS, gz - GZ_BIAS, ax - AX_BIAS, ay - AY_BIAS, az - AZ_BIAS);
    
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\t");
    
    // Display values
    
    // Accelerometer
    Serial.print (ax - AX_BIAS,DEC); 
    Serial.print ("\t");
    Serial.print (ay - AY_BIAS,DEC);
    Serial.print ("\t");
    Serial.print (az - AZ_BIAS,DEC);  
    Serial.print ("\t");
    
    // Gyroscope
    Serial.print (gx - GX_BIAS,DEC); 
    Serial.print ("\t");
    Serial.print (gy - GY_BIAS,DEC);
    Serial.print ("\t");
    Serial.print (gz - GZ_BIAS,DEC);  
    Serial.print ("\t");
    
    // End of line
    Serial.println("");

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
//  delay(100);/ // Probably don't need this with madgwick real time checking    
}

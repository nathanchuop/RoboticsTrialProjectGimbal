#include <Servo.h> //adds the servo library to control servo
#include <Wire.h> //adds wire library to communicate with I2C device (mpu6050)
Servo servo; //creates a servo object

void initialize_mpu6050(){
  //turn on mpu6050
  Wire.beginTransmission(0x68); //call Wire library to link arduino with mpu (0x68 is the address of the mpu6050)
  Wire.write(0x6B); //selects power management register of the mpu so I can configure it
  Wire.write(0x00); //turns on the mpu6050
  Wire.endTransmission(); //stops communication with current register & sends out the .write commands

  //Select sensitivity of Accelerometer (measure linear movement/position on a 3d axis)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1C); //accelerometer configuration register
  Wire.write(0x8); //selects sensitivity of accelerometer (+-4g = 8192 LSB/g)
  Wire.endTransmission();

  //Select sensitivity of Gyroscope (measures angular velocity around an axis)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); //gyroscope configuration register
  Wire.write(0x08); //selects sensitivity of gyroscope (500 deg/s = 65.5 LSB/deg/s)
  Wire.endTransmission();
}

/*Note about Accelerometer and Gyroscope for function read_mpu6050

Accelerometer Data takes 6 bytes total (2 bytes per axis, one highbyte one lowbyte)
Gyroscope Data takes 6 bytes total (2 bytes per axis, one highbyte one lowbyte)
temperature takes 2 bytes
Total Bytes: 6 + 6 + 2= 14
Will need to read 14 bytes to get all relevant sensor data
it shows up in the order: accel -> temp -> gyro


Must combine the two bytes into a 16 bit integer to accurately construct the measurement taken by the sensor
int16_t value = (highByte << 8 | lowByte);
explanation for line above:
highByte <<8    -   shifts the bits of HighByte 8 positions to the left, into the higher end of the 16 bit number
| lowByte       -   merges the high byte with the low byte into a single 16 bit number
overall result of the code: a single 16bit integer measurement from one axis of either the accelerometer or gyroscope

the int16_t value we store is the data collected with the units of Least Significant Bit. We must convert it to g or degrees
acceleration = acccelData/8192
gyro = gyroData/65.5
*/

void read_mpu6050(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //starting address for accelerometer's X-axis data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14); //takes the 14 byte data of the mpu6050 from the last register used (accelerometer X axis)

  //read & store acceleration and gyro data from the MPU6050 as a 16 bit integer
  int16_t acc_x = (Wire.read() << 8 | Wire.read()); //read automatically moves on to the next section
  int16_t acc_y = (Wire.read() << 8 | Wire.read());
  int16_t acc_z = (Wire.read() << 8 | Wire.read());
  Wire.read(); //skips over the 2 bytes of temperature data
  Wire.read();
  int16_t gy_x = (Wire.read() << 8 | Wire.read());
  int16_t gy_y = (Wire.read() << 8 | Wire.read());
  int16_t gy_z = (Wire.read() << 8 | Wire.read());

  //Convert from acceleration and gyro values of LSB to g and deg/s
  double aX = acc_x/8192.0;
  double aY = acc_y/8192.0;
  double aZ = acc_z/8192.0;
  double gX = gy_x/65.5;
  double gY = gy_y/65.5;
  double gZ = gy_z/65.5;
}

void setup() {
  servo.attach(9); //sets digital pin9 as the location for the servo object
  servo.write(0); //reset servor to 0 degrees
  delay(1000);

}

void loop() {


}










#include <Servo.h> //adds the servo library to control servo
#include <Wire.h> //adds wire library to communicate with I2C device (mpu6050)


Servo servo; //creates a servo object
int loop_interval_length = 100; //in ms
double rollRate, pitchRate;
double rollAngle, pitchAngle;


void initialize_mpu6050(){
  //turn on mpu6050
  Wire.beginTransmission(0x68); //call Wire library to link arduino with mpu (0x68 is the address of the mpu6050)
  Wire.write(0x6B); //selects power management register of the mpu so I can configure it
  Wire.write(0x00); //turns on the mpu6050
  Wire.endTransmission(); //stops communication with current register & sends out the .write commands

  //Select sensitivity of Accelerometer (measure linear movement/position on a 3d axis)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1C); //accelerometer configuration register
  Wire.write(0x10); //selects sensitivity of accelerometer (+-4g = 8192 LSB/g)
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


Must combine the two byt esinto a 16 bit integer to accurately construct the measurement taken by the sensor
int16_t value = (highByte << 8 | lowByte);
explanation for line above:
highByte <<8    -   shifts the bits of HighByte 8 positions to the left, into the higher end of the 16 bit number
| lowByte       -   merges the high byte with the low byte into a single 16 bit number
overall result of the code: a single 16bit integer measurement from one axis of either the accelerometer or gyroscope

the int16_t value we store is the data collected with the units of Least Significant Bit. We must convert it to g or degrees
acceleration = acccelData/8192
gyro = gyroData/65.5


pitch angle = rotation around the x-axis
roll angle = rotation around the y-axis
*/

void read_mpu6050(double &aX, double &aY, double &aZ, double &gX, double &gY, double &gZ){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //starting address for accelerometer's X-axis data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14); //takes the 14 byte data of the mpu6050 from the last register used (accelerometer X axis)

  //read & store acceleration and gyro data from the MPU6050 as a 16 bit integer
  int16_t accLSB_x = (Wire.read() << 8 | Wire.read()); //read automatically moves on to the next section
  int16_t accLSB_y = (Wire.read() << 8 | Wire.read());
  int16_t accLSB_z = (Wire.read() << 8 | Wire.read());
  Wire.read(); //skips over the 2 bytes of temperature data
  Wire.read();
  int16_t gy_x = (Wire.read() << 8 | Wire.read());
  int16_t gy_y = (Wire.read() << 8 | Wire.read());
  int16_t gy_z = (Wire.read() << 8 | Wire.read());

  //Convert from acceleration and gyro values of LSB to m/s^2 = g or deg/s
  aX = accLSB_x/4096.0 - 0.05; //adjusted values during calibration to all equal 1 when flat on its plane
  aY = accLSB_y/4096.0 + 0.01;
  aZ = accLSB_z/4096.0 + 0.13;
  
  rollRate = gy_x/65.5;
  pitchRate = gy_y/65.5;
  //yawRate = gy_z/65.5;

  //Calculate Roll and Pitch Angle in radians
  rollAngle = atan(aY/sqrt(aX*aX+aZ*aZ));
  pitchAngle = -atan(aX/sqrt(aY*aY + aZ*aZ));

  //Convert Roll & Pitch Angle to degrees
  rollAngle *= 180/3.1415;
  pitchAngle *= 180/3.1415;



  /*print out aX, aY, aZ to check for calibration
  Serial.print("aX = ");
  Serial.print(aX);
  Serial.print(" | aY = ");
  Serial.print(aY);
  Serial.print(" | aZ = ");
  Serial.println(aZ);
  */

  //print out Pitch and Roll Angle
  Serial.print("Roll Angle = ");
  Serial.print(rollAngle);
  Serial.print(" | Pitch Angle = ");
  Serial.println(pitchAngle);

}

void compareWithInitial(double aX, double aY, double aZ, double gX, double gY, double gZ){}

void setup() {
  Wire.begin(); //sets up I2C, allowing it to communicate with the arduino
  Serial.begin(115200); //sets up the speed of communication at 115200 bit/s
  initialize_mpu6050();

  servo.attach(9); //sets digital pin9 as the location for the servo object
  servo.write(0); //reset servor to 0 degrees
  delay(1000);


}

void loop() {
  //measures and prints out position every 100ms
  double accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
  read_mpu6050(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  delay(loop_interval_length);


}










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
  Wire.write(0x8); //selects sensitivity of accelerometer (+-4g)
  Wire.endTransmission();

  //Select sensitivity of Gyroscope (measures angular velocity around an axis)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); //gyroscope configuration register
  Wire.write(0x08); //selects sensitivity of gyroscope (500 deg/s)
  Wire.endTransmission();
}



void setup() {
  servo.attach(9); //sets digital pin9 as the location for the servo object
  servo.write(0); //reset servor to 0 degrees
  delay(1000);

}

void loop() {
  servo.write(0);
  delay(1000);
  servo.write(180);
  delay(1000);

}










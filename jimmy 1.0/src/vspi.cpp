//nrf TX Code using VSPI only because I can't figure out dual
//Code by Gabriel Qu
#include <Arduino.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <SD.h>
#include <stdarg.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>
#include <FS.h>
#include <ESP32Servo.h>


//gpt spew setups
// Constants
const double mass = 1.5;           // Rocket mass (kg)
const double inertia = 0.2868;     // Rocket moment of inertia
const double distance = 0.358;     // Distance from center to force application point (m)
const double gimbal_force = 60;    // Constant force applied by the gimbal (N)
const double time_step = 0.01;     // Simulation time step (s)
const double radian_conversion = M_PI / 180.0;

// PID constants
const double Kp = 1.0;  // Proportional gain
const double Ki = 0.1;  // Integral gain
const double Kd = 0.05; // Derivative gain

// Variables
unsigned long last_update_time = 0;
double timeC = 0.0;
double gimbal_angle_pitch_deg = 0.0;
double gimbal_angle_yaw_deg = 0.0;
double pitch_angle = 0.0; // Pitch angle (rad)
double angular_velocity_pitch = 0.0; // Angular velocity from gyroscope

// State variables for positions and velocities
double accel_x = 0.0, accel_z = 0.0; // Accelerations in X and Z directions
double velocity_x = 0.0, velocity_z = 0.0; // Velocities in X and Z directions
double position_x = 0.0, position_z = 0.0; // Positions in X and Z directions

// PID state variables
double previous_error = 0.0;
double integral = 0.0;


// Servo setup
Servo servoPitch;
Servo servoYaw;
const int servoPitchPin = 9; // Pin for pitch servo
const int servoYawPin = 10;  // Pin for yaw servo


//pin declares for chip selects
#define RF_CS 4
#define SD_CS 5

//radio setups
const byte addy[][6] = {"00001","00002"};
const uint8_t chan = 105;

//reference altitude float
float refAlt;


//Module declares
SoftwareSerial gpsSerial(17, 16); //gps is UART so different? TBH I'm not that sure
TinyGPSPlus gps;
RF24 radio; // CE, CSN
Adafruit_BMP3XX bmp;
MPU6050 mpu; // Set up IMU

//SoftwareSerial ss(17, 16); // for the gps
/*
//section for mpu data processing preloop stuff
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//data interrupt detector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
*/



struct data
{
  int16_t ax, ay, az, gx, gy, gz, satNum; //6 axis motion figures from IMU and # of satellites connected
  float lat, lon, alt; //latitude, longitude, and altitude
  bool cardCon=1; // boolean to show if SD card is connected
};
data dat1;
String buff;

//method declares for code organization - if you write your methods above where they are declared
// there is no need for this. 

void sendData();
void SDWrite();
void dataGather();
void servoMode();

//fuck ass sd code
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    dat1.cardCon = false;
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void setup()
{
  Serial.begin(115200);

  //radio initializations
  radio.begin(4,13);
  radio.setPALevel(RF24_PA_MAX, 0);
  radio.setChannel(chan);
  Serial.println("Starting to send: ");
  radio.openWritingPipe(addy[0]);

  //imu mpu and gps start declare
  Wire.begin();
  bmp.begin_I2C();
  mpu.initialize();
  gpsSerial.begin(9600); // connect gps sensor, default baud rate for L80 is 9600
  SD.begin(SD_CS);

  //BMP390 setup- oversampling and filtering
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
/*
  //MPU JUNK --------------------------------------------------------------------------------------
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-117);
  mpu.setYGyroOffset(-41);
  mpu.setZGyroOffset(-39);
  mpu.setZAccelOffset(1934); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  //end mpu junk ---------------------------------------------------------------------
*/

  //basic initialization to create/overwrite file 
  digitalWrite(SD_CS, LOW);
  File dataFile = SD.open("/data.txt");
  writeFile(SD, "/data.txt", "init");
  digitalWrite(SD_CS, HIGH);

  // Servo initialization
  servoPitch.attach(servoPitchPin);
  servoYaw.attach(servoYawPin);  

  last_update_time = millis();
}

void loop()
{
  if (millis() % 25){
  dataGather();
  //sendData();
  SDWrite();
  //servoMode();
  }
}

/* 
Simple Code explanation for this method (I'm no expert so google it more):
Since the CS is active low, which is to say the chip only works when the pin is not powered,
and you want to use two SPI Peripherals on a single SPI bus, you need to tell a peripheral 
when to turn on and off in order for data to transmit only to that peripheral. By doing digitalWrite
before and after the data transmission, you are effectively turning the module on for a short while to
send data, and then off so it doesn't interfere with the SD card write method. 

For more in depth information on the radio. methods, check the RF library this code utilizes.
*/
void sendData()
{
  digitalWrite(RF_CS, LOW);
  radio.stopListening();
  buff = String(dat1.lat, 6) + "\t" + String(dat1.lon, 6) + "\t" + String(dat1.alt) + "\t" + String(dat1.satNum) + "\t" + String(dat1.ax-1224) + "\t" + String(dat1.ay-2621)
  + "\t" + String(dat1.az+1934) + "\t" + String(dat1.gx-117) + "\t" + String(dat1.gy-41) + "\t" + String(dat1.gz-39) + "\t" + String(dat1.cardCon);
  Serial.println("Sending: ");
  Serial.println(buff);
  radio.write(&dat1, sizeof(data));
  digitalWrite(RF_CS, HIGH);
}

/*
Opens and closes communication to device by changing CS state. 
Opens a file on a FAT32 formatted SD card, and writes string buff to it.
*/
void SDWrite()
{
  
  digitalWrite(SD_CS, LOW);
  String buf = String(dat1.lat, 6) + "\t" + String(dat1.lon, 6) + "\t" + String(dat1.alt) + "\t" + String(dat1.satNum) + "\t" + String(dat1.ax-1224) + "\t" + String(dat1.ay-2621)
  + "\t" + String(dat1.az+1934) + "\t" + String(dat1.gx-117) + "\t" + String(dat1.gy-41) + "\t" + String(dat1.gz-39) + "\n";
  appendFile(SD, "/data.txt", buf.c_str());
  digitalWrite(SD_CS, HIGH);
}

/*
Check your module compatability and whatnot for this section of code, 
we use the BMP390, the jimmy IMU (BMP085?)and the Quectal L80 GPS
*/
void dataGather()
{
  //lat long data get- this code uses the tinyGPSPLus Lib, check parts for compatabilty 
  
   while (gpsSerial.available())     // check for gps data
  {
    if (gps.encode(gpsSerial.read()))   // encode gps data
    {
      dat1.lat= gps.location.lat();
      Serial.println(dat1.lat);
      dat1.lon= gps.location.lng();
      Serial.println(dat1.lon);
      dat1.satNum = gps.satellites.value();
      Serial.println(dat1.satNum);
    }
  }
  
  //mpu data write

  mpu.getMotion6(&dat1.ax, &dat1.ay, &dat1.az, &dat1.gx, &dat1.gy, &dat1.gz);
  
  //bmp altitude get
  dat1.alt = (bmp.readAltitude(1013.25)); //average value, measure and change onsite for more accurate measurement
}

void servoMode(){

  /*
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
  }
  */

  unsigned long current_time = millis();
  if (current_time - last_update_time >= time_step * 1000) {
      last_update_time = current_time;

      // Read MPU6050 data
      int16_t ax, ay, az;
      int16_t gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      // Convert accelerometer data (ax, ay, az) to tilt angle
      double tilt_angle = atan2(ay, az) * radian_conversion; // Tilt angle in radians

      // Convert gyroscope data to angular velocity (rad/s)
      angular_velocity_pitch = gy / 131.0 * radian_conversion;

      // Update pitch angle by integrating angular velocity
      pitch_angle += angular_velocity_pitch * time_step;

      // Define setpoint (desired pitch angle, in degrees)
      double setpoint = 10.0; // Desired pitch angle after 500m altitude
      if (position_z <= 500.0) {
          setpoint = 0.0; // Keep upright below 500m
      }

      // Calculate error for PID control (setpoint in radians)
      double error = (setpoint * radian_conversion) - pitch_angle;

      // PID calculations
      integral += error * time_step;
      double derivative = (error - previous_error) / time_step;
      double output = Kp * error + Ki * integral + Kd * derivative;

      // Cap output to gimbal angle limits
      if (output > 10.0) output = 10.0;
      if (output < -10.0) output = -10.0;

      gimbal_angle_pitch_deg = output;

      // Calculate forces in X and Z based on gimbal pitch angle
      double force_x = gimbal_force * cos(gimbal_angle_pitch_deg * radian_conversion);
      double force_z = gimbal_force * sin(gimbal_angle_pitch_deg * radian_conversion);

      // Calculate accelerations
      accel_x = force_x / mass;
      accel_z = force_z / mass;

      // Integrate accelerations to find velocities
      velocity_x += accel_x * time_step;
      velocity_z += accel_z * time_step;

      // Integrate velocities to find positions
      position_x += velocity_x * time_step;
      position_z += velocity_z * time_step;

      // Set servo angles
      servoPitch.write(map(gimbal_angle_pitch_deg, -10, 10, 0, 180));
      servoYaw.write(map(gimbal_angle_yaw_deg, -10, 10, 0, 180));

      // Print results
      Serial.print("Time: "); Serial.print(timeC);
      Serial.print(" s\tPitch Angle: "); Serial.print(pitch_angle * 180.0 / M_PI);
      Serial.print(" deg\tSetpoint: "); Serial.print(setpoint);
      Serial.print(" deg\tOutput: "); Serial.println(output);
      Serial.print("Tilt Angle: "); Serial.print(tilt_angle * 180.0 / M_PI);
      Serial.print(" deg\tPosition Z: "); Serial.println(position_z);

      // Save current error for next loop
      previous_error = error;

      // Increment time
      timeC += time_step;
  } 
}

//launch states- change data collection rate and operations
//Zero sensors at launch site
//physical seperation between recovery system and power9


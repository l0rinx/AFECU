

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
MPU6050 mpu;

//-------------------------------------------------------------------------------------------------------------------------------*/
#define OUTPUT_READABLE_YAWPITCHROLL

const int buttonPin = 2;  // the pin that the pushbutton is attached to

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---Orientation/Motion Variables---*/
float lastRoll = 0, lastYaw = 0;
unsigned long lastTime = 0;
/*---Orientation/Motion Variables---*/


/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/

/*---actuator setup---*/            // BOTh HAVE NO HALL EFFECT SENSORS SO YOU NEED TO DO CFD AND INTERP TO GET POS
const int actuator1ForwardPin = 2;  // Digital pin connected to relay/H-bridge controlling actuator 1 forward direction
const int actuator1ReversePin = 3;  // Digital pin connected to relay/H-bridge controlling actuator 1 reverse direction
const int actuator2ForwardPin = 4;  // Digital pin connected to relay/H-bridge controlling actuator 2 forward direction
const int actuator2ReversePin = 5;  // Digital pin connected to relay/H-bridge controlling actuator 2 reverse direction

const int movementTime = 5000;  // Estimated time in milliseconds to move the actuator 15cm (adjust as necessary)

float X = 0.6;

/*---actuator setup---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void setup() {
  //____________________
  const int buttonPin = 2;  // the pin that the pushbutton is attached to
  Wire.begin();
  //________________

  /*---actuator setup---*/
  pinMode(actuator1ForwardPin, OUTPUT);
  pinMode(actuator1ReversePin, OUTPUT);
  pinMode(actuator2ForwardPin, OUTPUT);
  pinMode(actuator2ReversePin, OUTPUT);

  // Make sure actuators are initially off
  //stopActuators();
}


/*---actuator setup---*/

// Initialize MPU6050
//      Serial.println("Initializing MPU6050...");
//if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
//      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
while (1)
  ;



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

Serial.begin(115200);  //115200 is required for Teapot Demo output
while (!Serial)
  ;

/*Initialize device*/
Serial.println(F("Initializing I2C devices..."));
mpu.initialize();
pinMode(INTERRUPT_PIN, INPUT);


/*Verify connection*/
Serial.println(F("Testing MPU6050 connection..."));
if (mpu.testConnection() == false) {
  Serial.println("MPU6050 connection failed");
  while (true)
    ;
} else {
  Serial.println("MPU6050 connection successful");
}

/*Wait for Serial input*/
Serial.println(F("\nSend any character to begin: "));
while (Serial.available() && Serial.read())
  ;  // Empty buffer
while (!Serial.available())
  ;  // Wait for data
while (Serial.available() && Serial.read())
  ;  // Empty buffer again

/* Initializate and configure the DMP*/
Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize();

/* Supply your gyro offsets here, scaled for min sensitivity */
mpu.setXGyroOffset(0);
mpu.setYGyroOffset(0);
mpu.setZGyroOffset(0);
mpu.setXAccelOffset(0);
mpu.setYAccelOffset(0);
mpu.setZAccelOffset(0);

/* Making sure it worked (returns 0 if so) */
if (devStatus == 0) {
  mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateGyro(6);
  Serial.println("These are the Active offsets: ");
  mpu.PrintActiveOffsets();
  Serial.println(F("Enabling DMP..."));  //Turning ON DMP
  mpu.setDMPEnabled(true);

  /*Enable Arduino interrupt detection*/
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
  MPUIntStatus = mpu.getIntStatus();

  /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
  Serial.println(F("DMP ready! Waiting for first interrupt..."));
  DMPReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();  //Get expected DMP packet size for later comparison
} else {
  Serial.print(F("DMP Initialization failed (code "));  //Print the error code
  Serial.print(devStatus);
  Serial.println(F(")"));
}
pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (!DMPReady) return;  // Stop the program if DMP programming fails.

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {  // Get the Latest packet
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_QUATERNION
    /* Display Quaternion values in easy matrix form: [w, x, y, z] */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    /* Display real acceleration, adjusted to remove gravity */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL

    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT

    teapotPacket[2] = FIFOBuffer[0];
    teapotPacket[3] = FIFOBuffer[1];
    teapotPacket[4] = FIFOBuffer[4];
    teapotPacket[5] = FIFOBuffer[5];
    teapotPacket[6] = FIFOBuffer[8];
    teapotPacket[7] = FIFOBuffer[9];
    teapotPacket[8] = FIFOBuffer[12];
    teapotPacket[9] = FIFOBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++;  // PacketCount, loops at 0xFF on purpose
#endif



    //Printt giv to csv


    Vector rawAccel = mpu.readRawAccel();
    Vector normAccel = mpu.readNormalizeAccel();
    Vector rawGyro = mpu.readRawGyro();

    // Sending data over serial in CSV format (ax, ay, az, gx, gy, gz)
    Serial.print(rawAccel.XAxis);
    Serial.print(",");
    Serial.print(rawAccel.YAxis);
    Serial.print(",");
    Serial.print(rawAccel.ZAxis);
    Serial.print(",");
    Serial.print(rawGyro.XAxis);
    Serial.print(",");
    Serial.print(rawGyro.YAxis);
    Serial.print(",");
    Serial.println(rawGyro.ZAxis);

    delay(100);  // Adjust the delay as needed


    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);


    if (buttonState == HIGH) {
      Serial.println("Tiresaving Mode On");
      //Now thess
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
      float RXpast = Serial.read(ypr[0] * 180 / M_PI);
      float RYpast = Serial.read(ypr[1] * 180 / M_PI);
      float RZpast = Serial.read(ypr[2] * 180 / M_PI);
      delay(750);
      float RXFuture = Serial.read(ypr[0] * 180 / M_PI);
      float RYFuture = Serial.read(ypr[1] * 180 / M_PI);
      float RZFuture = Serial.read(ypr[2] * 180 / M_PI);
      //CurrentState
      float RXcurrent = RXpast - RXFuture;
      float RYcurrent = RXpast - RXFuture;
      float RZcurrent = RXpast - RXFuture;
      //behold now fuckton of if statements

      if (RXcurrent > 0.5 && RYcurrent > 0.25) {
        Serial.print("Brake and steer less");
      }
      digitalWrite(actuator1ForwardPin, HIGH);
      delay(movementTime);  // Wait for the movement to complete
      digitalWrite(actuator1ForwardPin, LOW);

      // Retract actuator 2 15cm
      digitalWrite(actuator2ReversePin, HIGH);
      delay(movementTime);  // Wait for the movement to complete
      digitalWrite(actuator2ReversePin, LOW);
    }
    void stopActuators() {
      // Ensure all actuator control pins are off
      digitalWrite(actuator1ForwardPin, LOW);
      digitalWrite(actuator1ReversePin, LOW);
      digitalWrite(actuator2ForwardPin, LOW);
      digitalWrite(actuator2ReversePin, LOW);
    }

/*---MPU6050 Control/Status Variables---*/
unsigned long currentTime = millis();

  // Read accelerometer and gyroscope data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert to angles
  float roll = atan2(ay, az) * 180 / PI; // Roll angle
  float yaw = atan2(az, ax) * 180 / PI; // Yaw angle
  
  // Compare every 300ms
  if (currentTime - lastTime >= 300) {
    float rollDiff = abs(roll - lastRoll);
    float yawDiff = abs(yaw - lastYaw);

    if (rollDiff > 22) {
      Serial.println("state 1");
      //leververingauche //baisserdroite 
    }

     if (rollDiff > 15) {
      Serial.println("2");
      //leververingauche //baisserdroite 
    }


     if (rollDiff > 10) {
      Serial.println("3");
      //leververingauche //baisserdroite 
    }
else{

  //return to neutral 
}



    if (rollDiff > -22) {
      Serial.println("state 1");
      //leververingauche //baisserdroite 
    }

     if (rollDiff > -15) {
      Serial.println("2");
      //leververingauche //baisserdroite 
    }


     if (rollDiff > -10) {
      Serial.println("3");
      //leververingauche //baisserdroite 
    }
else{

  //return to neutral 
}

    if (yawDiff > 20) {
      Serial.println("tournage a gauche");
    }
 if (yawDiff > 20) {
      Serial.println("tournage a gauche");
    }

     if (yawDiff > 20) {
      Serial.println("tournage a gauche");
    }
    else{
      return to neutral
    }
    // Update the last recorded values
    lastRoll = roll;
    lastYaw = yaw;
    lastTime = currentTime;
  }
/*---MPU6050 Control/Status Variables---*/


  }

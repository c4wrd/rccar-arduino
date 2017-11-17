#include <Servo.h>
#include <SPI.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <Wire.h>
#include <MPU6050.h>
#include <stdarg.h>
#include <SoftwareSerial.h>


/////////////////////////////////////////////
// RC Car                                  //
/////////////////////////////////////////////

class DCMotor {
  private:
    int speedPin;
    int forwardPin;
    int reversePin;
  
  public :
    DCMotor(int speedPin, int forwardPin, int reversePin)
      : speedPin(speedPin), forwardPin(forwardPin), reversePin(reversePin) {
    }
    void setup() {
      pinMode(this->speedPin, OUTPUT);
      pinMode(this->forwardPin, OUTPUT);
      pinMode(this->reversePin, OUTPUT);
      digitalWrite(this->forwardPin, 0);
      digitalWrite(this->reversePin, 0);
    }
    /**
     * Sets the speed of the motor
     * 
     * @param speed: A value between (-100, 100), where a positive value
     *  indicates forward and a negative value indicates backwards
     */
    void setSpeed(int inSpeed) {
      int speed = map(inSpeed, -100, 100, -255, 255);
      boolean direction = speed < 0;
      if (speed == 0)
      {
          digitalWrite(this->forwardPin, LOW);
          digitalWrite(this->reversePin, LOW);
      }
      else
      {
          boolean forward = speed > 0;
          // Serial.print("Setting the forward pin "); Serial.print(this->forwardPin); Serial.print(" to value "); Serial.println(forward);
          digitalWrite(this->forwardPin, forward);
          // Serial.print("Setting the reverse pin "); Serial.print(this->reversePin); Serial.print(" to value "); Serial.println(!forward);
          digitalWrite(this->reversePin, !forward);
      }

      // Serial.print("Setting the pin "); Serial.print(this->speedPin); Serial.print(" to value "); Serial.println(abs(speed));
      analogWrite(this->speedPin, abs(speed));
    }
};

class RCCar {
  private:
    DCMotor *frontMotor;
    DCMotor *backMotor;
    Servo *servo;

  public:
    RCCar(DCMotor *frontMotor, DCMotor *backMotor, Servo *servo)
      : frontMotor(frontMotor), backMotor(backMotor), servo(servo) {}
      
    void setSpeed(int speed) {
      if (this->frontMotor != nullptr)
      {
          this->frontMotor->setSpeed(speed);
      }
      if (this->backMotor != nullptr)
      {
          this->backMotor->setSpeed(speed);
          // TODO some say polarity needs to be reversed on
          // motors, so we might need to set negative speed on one motor
          // this->backMotor->setSpeed(speed);
      }
    }
    
    void setDirection(int direction) {
      if (direction == 0) {
        //Serial.print("setting direction to 25 ( 0 )");
        this->servo->write(25);
      } else {
        int correctedDirection = map(direction, -100, 100, 0, 50);
        //Serial.print("setting direction to ");
      // Serial.println(correctedDirection);
        this->servo->write(correctedDirection);
      }
    }
    
    //virtual void getGyroInformation() = 0;
};

/////////////////////////////////////////////
// END Rc Car                              //
/////////////////////////////////////////////

int fmForwardPin= 10;
int fmBackwardPin = 4;
int fmEnablePin = 3;
DCMotor *frontMotor;

int bmForwardPin = 6;
int bmBackwardPin = 7;
int bmEnablePin = 5;
DCMotor *backMotor;

int servoPin = 9;
Servo servo;

int rearServoPin = 11;
Servo rearServo;
RCCar *car;

// #define SERIAL_RX 11
// #define SERIAL_TX 12
// SoftwareSerial Serial1(SERIAL_RX, SERIAL_TX); // RX, TX
// Wifi related variables
bool COMMAND_RECEIVED = false;        // whether or not a command was received
int COMMAND_THRESHOLD_MS = 3000;
int UDP_LISTEN_PORT= 8888;
char ssid[] = "RCCar";         // your network SSID (name)
char pass[] = "12345678";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
long LAST_COMMAND_TIME = 0;

WiFiEspUDP Udp;

// gyroscope
MPU6050 mpu;

// App constants
long GLOBAL_LOOP_TIME = 0L;

struct RCCarCommand {
  short speed;
  short direction;
};

struct RCCarResponse {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

void setup() {
  /** 
   *  Setup serial for debugging
   */
  Serial.begin(115200);   // initialize serial for debugging

  /** 
   *  Setup WiFi
   */
  //Serial1.begin(9600);
  setupWifi();
  /**
   * Setup RC Car
   */
  frontMotor = new DCMotor(fmEnablePin, fmForwardPin, fmBackwardPin);
  backMotor = new DCMotor(bmEnablePin, bmForwardPin, bmBackwardPin);
  frontMotor->setup();
  backMotor->setup();
  servo.attach(servoPin);
  rearServo.attach(rearServoPin);
  
  car = new RCCar(frontMotor, backMotor, &servo);
  car->setSpeed(0);
  car->setDirection(0);

  initGyro();
}

void loop() {
  
  GLOBAL_LOOP_TIME = millis();
  RCCarCommand command;
  bool commandReceived = serverLoop(&command);

  if ( commandReceived ) {
    //Serial.print(command.speed);
    //Serial.print(", direction=");
    //Serial.println(command.direction);
    
    car->setSpeed(command.speed);
    car->setDirection(command.direction);
    
  }

  if ( COMMAND_RECEIVED && GLOBAL_LOOP_TIME - LAST_COMMAND_TIME > COMMAND_THRESHOLD_MS ) {
    //Serial.println("No activity in 1000 ms, stopping car...");
    car->setSpeed(0);
    car->setDirection(0);
    COMMAND_RECEIVED = false;
  }
}


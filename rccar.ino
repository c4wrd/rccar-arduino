#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

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
     * @param speed: A value between -255, 255, where a positive value
     *  indicates forward and a negative value indicates backwards
     */
    void setSpeed(int speed) {
      boolean direction = speed < 0;
      if (speed == 0)
      {
          digitalWrite(this->forwardPin, LOW);
          digitalWrite(this->reversePin, LOW);
      }
      else
      {
          boolean forward = speed > 0;
          Serial.print("Setting the forward pin "); Serial.print(this->forwardPin); Serial.print(" to value "); Serial.println(forward);
          digitalWrite(this->forwardPin, forward);
          Serial.print("Setting the reverse pin "); Serial.print(this->reversePin); Serial.print(" to value "); Serial.println(!forward);
          digitalWrite(this->reversePin, !forward);
      }

      Serial.print("Setting the pin "); Serial.print(this->speedPin); Serial.print(" to value "); Serial.println(abs(speed));
      analogWrite(this->speedPin, abs(speed));
    }
};

class RCCar {
  private:
    DCMotor *frontMotor;
    DCMotor *backMotor;
    Servo *servo;

  public:
    RCCar(DCMotor *frontMotor, DCMotor *rearMotor, Servo *servo)
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
        this->servo->write(30);
      } else {
        int correctedDirection = map(direction, -45, 45, 10, 40);
        this->servo->write(correctedDirection);
      }
    }
    
    //virtual void getGyroInformation() = 0;
};

char WIFI_SSID[] = "CForward";
char PASSWORD[] = "af9d168f67";
int WIFI_STATUS = WL_IDLE_STATUS;
long LAST_COMMAND_TIME = 0L; // time last command was received
bool COMMAND_RECEIVED = false;        // whether or not a command was received
int COMMAND_THRESHOLD_MS = 250;
int UDP_LISTEN_PORT= 8888;
WiFiUDP server;

// Display Constants
#define DISPLAY_PIN 4

// App constants
long GLOBAL_LOOP_TIME = 0L;
int fmForwardPin= 9;
int fmBackwardPin = 2;
int fmEnablePin = 3;
DCMotor *frontMotor;

int bmForwardPin = 0x4;
int bmBackwardPin = 0x5;
int bmEnablePin = 0x6;
DCMotor *backMotor;

int servoPin = 11;
Servo servo;
RCCar *car;

struct RCCarCommand {
  int speed;
  int direction;
};

void setup() {
  Serial.begin(9600);      // initialize serial communication
  Serial.println("setup!");
  frontMotor = new DCMotor(fmEnablePin, fmForwardPin, fmBackwardPin);
  backMotor = new DCMotor(bmEnablePin, bmForwardPin, bmBackwardPin);
  frontMotor->setup();
  backMotor->setup();
  servo.attach(servoPin);
  servo.write(30); // 30 is middle because of small tie rod

  car = new RCCar(frontMotor, backMotor, &servo);

  setupDisplay();
  setupWifi();
  
  printWifiStatus();
}

void loop() {
  GLOBAL_LOOP_TIME = millis();
  RCCarCommand command;
  bool commandReceived = serverLoop(&command);

  if ( commandReceived ) {
    Serial.print("Received command, speed=");
    Serial.print(command.speed);
    Serial.print(", direction=");
    Serial.println(command.direction);
    
    car->setSpeed(command.speed);
    car->setDirection(command.direction);
  }

  if ( COMMAND_RECEIVED && GLOBAL_LOOP_TIME - LAST_COMMAND_TIME > COMMAND_THRESHOLD_MS ) {
    Serial.println("No activity in 1000 ms, stopping car...");
    car->setSpeed(0);
    car->setDirection(0);
    COMMAND_RECEIVED = false;
  }
}


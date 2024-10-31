#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <semphr.h>





SemaphoreHandle_t xSerialSemaphore;

TaskHandle_t pull_handle = NULL;
TaskHandle_t move_handle = NULL;
TaskHandle_t test_pull_handle = NULL;

Servo myservo;

SoftwareSerial BTSerial(10, 11);

typedef enum { STRAIGHT, FORWARD, RIGHT, BACKWARD, LEFT, UP, DOWN,STOP } Direction;

Direction dir;


void btTask(void *pvParameters __attribute__((unused))) {
  while (1) {
    if (BTSerial.available()) {
      int nm = BTSerial.read();
      if(nm=='U') dir=FORWARD;
      else if(nm=='R') dir=RIGHT;
      else if(nm=='B') dir=BACKWARD;
      else if(nm=='L') dir=LEFT;
      else if(nm=='C') dir=STRAIGHT;
      else if(nm=='Q') dir=UP;
      else if(nm=='W') dir=DOWN;
      else if(nm=='E') dir=STOP;


      
      vTaskNotifyGiveFromISR(move_handle,NULL);

      if(nm=='A' || nm=='a')
        vTaskNotifyGiveFromISR(pull_handle,NULL);
      else
       vTaskNotifyGiveFromISR(move_handle,NULL);;
    }
  }
}

void motor_task(void *pvParameters __attribute__((unused))){
  //Left
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);

  //RIGHT
  pinMode(6,OUTPUT);
  pinMode(5,OUTPUT);

  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  for(;;){
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 0) {
      Serial.println("HUYYYY");
      if(dir==FORWARD){
        // Serial.println("F");
        digitalWrite(8,HIGH);
        digitalWrite(7,LOW);

        digitalWrite(6,HIGH);
        digitalWrite(5,LOW);
      } else if(dir==RIGHT){
        // Serial.println("R");
        digitalWrite(8,LOW);
        digitalWrite(7,HIGH);

        digitalWrite(6,HIGH);
        digitalWrite(5,LOW);
      } else if(dir==LEFT){
        // Serial.println("L");
        digitalWrite(8,HIGH);
        digitalWrite(7,LOW);

        digitalWrite(6,LOW);
        digitalWrite(5,HIGH);
      } else if(dir==BACKWARD){
        // Serial.println("B");
        digitalWrite(8,LOW);
        digitalWrite(7,HIGH);

        digitalWrite(6,LOW);
        digitalWrite(5,HIGH);
      } else if(dir==STRAIGHT){
        // Serial.println("S");
        digitalWrite(8,LOW);
        digitalWrite(7,LOW);

        digitalWrite(6,LOW);
        digitalWrite(5,LOW);
      } else if(dir==UP){
        digitalWrite(12,HIGH);
        digitalWrite(13,LOW);
      } else if(dir==DOWN){
        digitalWrite(12,LOW);
        digitalWrite(13,HIGH);
      } else if(dir==STOP){
        digitalWrite(12,LOW);
        digitalWrite(13,LOW);
      } 
      // vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    }
  // }
}

bool on_pull=false;

void test_pull(void *pvParameters){
  while(1){
  //   myservo.write(360);
  // //   Serial.println("TEST PULL");
  // //     myservo.write(0);   // Rotate clockwise at full speed
  // vTaskDelay(1000/portTICK_PERIOD_MS);        // Rotate for 1 second

  // myservo.write(0);  // Stop the servo
  // vTaskDelay(1000/portTICK_PERIOD_MS);   

  // myservo.write(180); // Rotate counterclockwise at full speed
  // vTaskDelay(1000/portTICK_PERIOD_MS);   

  // myservo.write(90);  // Stop the servo
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void pull_task(void *pvParameters __attribute__((unused))) {
  while (1) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != 0) {
      // Serial.println("PLLL");
      on_pull=!on_pull;
      if(on_pull){
          xTaskCreate(test_pull, "test_pull" // A name just for humans
              ,
              128 // This stack size can be checked & adjusted by reading the
                  // Stack Highwater
              ,
              NULL // Parameters for the task
              ,
              2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                // highest, and 0 being the lowest.
              ,
              &test_pull_handle);   
      } else{
        vTaskDelete(test_pull_handle);
      }
      // angle+=90;
      // myservo.write(angle);
      // if(angle==180){
      //   angle=0;
      // }
      vTaskDelay(1000/portTICK_PERIOD_MS);
  //     Serial.println("pull");
  //       myServo.write(0);   // Rotate clockwise at full speed
  // vTaskDelay(1000);        // Rotate for 1 second

  // myServo.write(90);  // Stop the servo
  // vTaskDelay(1000);        // Pause for 1 second

  // myServo.write(180); // Rotate counterclockwise at full speed
  // vTaskDelay(1000);        // Rotate for 1 second

  // myServo.write(90);  // Stop the servo
  // vTaskDelay(1000);  
    }
  }
}

void setup() {
  Serial.begin(9600); // initialize Serial communication

  while (!Serial); // wait for the serial port to open
  BTSerial.begin(9600);

  xTaskCreate(btTask, "bt task" // A name just for humans
              ,
              128 // This stack size can be checked & adjusted by reading the
                  // Stack Highwater
              ,
              NULL // Parameters for the task
              ,
              2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                // highest, and 0 being the lowest.
              ,
              NULL);                 // Task Handle
  xTaskCreate(pull_task, "pull_task" // A name just for humans
              ,
              200 // This stack size can be checked & adjusted by reading the
                  // Stack Highwater
              ,
              NULL // Parameters for the task
              ,
              3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                // highest, and 0 being the lowest.
              ,
              &pull_handle); // Task Handle
    xTaskCreate(motor_task, "motor_task" // A name just for humans
              ,
              128 // This stack size can be checked & adjusted by reading the
                  // Stack Highwater
              ,
              NULL // Parameters for the task
              ,
              2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                // highest, and 0 being the lowest.
              ,
              &move_handle); // Task Handle
              vTaskStartScheduler();
  // servo_init();
  digitalWrite(8,HIGH);
  digitalWrite(7,LOW);
  // put your setup code here, to run once:
}


void loop() {
}

// put function definitions here:
int myFunction(int x, int y) { return x + y; }
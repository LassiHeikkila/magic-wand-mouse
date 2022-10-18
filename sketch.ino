//**************************************************************************
// Magic Wand mouse implemented by Lassi Heikkilä
// Runs on Arduino MKR WiFi 1010 + MKR IMU shield combo
//
// Starting point was Basic_RTOS_Example from FreeRTOS_SAMD21 library by Scott Briscoe
//
//**************************************************************************

#include <FreeRTOS_SAMD21.h>
#include <timers.h>
#include <queue.h>

#include <MKRIMU.h>

#include <Mouse.h>

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

#define ERROR_LED_PIN 13
#define ERROR_LED_LIGHTUP_STATE HIGH
#define MOTION_QUEUE_SIZE 16
#define BUTTON_QUEUE_SIZE 8

#define MOUSE_BUTTON_LEFT_PIN  6
#define MOUSE_BUTTON_RIGHT_PIN 7

struct Motion {
  float deltaHeading;
  float deltaPitch;
};

enum ButtonId    {ButtonLeft, ButtonRight};
enum ButtonState {ButtonPressed, ButtonReleased};

struct ButtonClick {
  ButtonId id;
  ButtonState state;
};

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_buttonTask;
TaskHandle_t Handle_gyroTask;
TaskHandle_t Handle_digitalInputTask;
TaskHandle_t Handle_motionHandlerTask;
TaskHandle_t Handle_buttonHandlerTask;

TimerHandle_t Timer_idleTimer;

QueueHandle_t Queue_motionQueue;
QueueHandle_t Queue_buttonQueue;

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account processor speed
// Use these instead of delay(...) in rtos tasks
//**************************************************************************
void myDelayUs(int us)
{
  vTaskDelay( us / portTICK_PERIOD_US );
}

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

void myDelayMsUntil(TickType_t *previousWakeTime, int ms)
{
  vTaskDelayUntil( previousWakeTime, (ms * 1000) / portTICK_PERIOD_US );  
}

//*****************************************************************
// Create a thread that prints out gyro readings periodically
// this task will run forever
//*****************************************************************

void threadGyro( void *pvParameters) {
  // we don't care about roll, only heading (left-right) or pitch (up-down)
  float prevHeading, prevPitch = 0.0;
  float heading, pitch = 0.0;
  float deltaHeading, deltaPitch = 0.0;
  float discard;
  struct Motion motion;

  while(1) {
    if(!IMU.eulerAnglesAvailable()) {
      myDelayMs(100);
    }
    IMU.readEulerAngles(heading, discard, pitch);
    // arranged so that with origin at bottom left, turning right and pointing up will increase both x and y
    deltaHeading = heading - prevHeading;
    deltaPitch = prevPitch - pitch;

    if (deltaHeading == 0 && deltaPitch == 0) {
      myDelayMs(100);
      continue;
    }
    xTimerReset(Timer_idleTimer, 0);
    motion.deltaHeading = deltaHeading;
    motion.deltaPitch = deltaPitch;

    // send the motion event to queue, don't care if succeeds or not
    xQueueSend(Queue_motionQueue, &motion, 0);

    prevHeading = heading;
    prevPitch = pitch;

    myDelayMs(10);
  }
   // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete( NULL );
}

void idleTimerCallback(TimerHandle_t timer) {
  //Serial.println("Idle timer expired!");
}

// arbitrary scaling choice:
// we want about the screen to represent around -45 degree - +45 degree field horizontally and vertically
// that means 90 degrees/ ~ 2000 pixels horizontally and 90 degrees / ~ 1000 pixels vertically (roughly 1080p res)
char scaleHorizontalMovement(float f) {
  // 1 degree ~= 22 pixels

  f = (f*22);
  if (f > 127.0) {
    return 127;
  } else if (f < -128.0) {
    return -128;
  } else {
    return char(f);
  }
}

char scaleVerticalMovement(float f) {
  // 1 degree ~= 45 pixels
  f = (f*45);
  if (f > 127.0) {
    return 127;
  } else if (f < -128.0) {
    return -128;
  } else {
    return char(f);
  }
}

void motionEventHandler( void *pvParameters ) {
  Motion motionEvt;

  while(1) {
    if (xQueueReceive(Queue_motionQueue, &motionEvt, portMAX_DELAY) != pdPASS) {
      continue;
    }

    char dx = scaleHorizontalMovement(motionEvt.deltaHeading);
    char dy = scaleVerticalMovement(motionEvt.deltaPitch);

    Mouse.move(dx, dy, 0);
  }
}

const char* SerializeButtonEvent(ButtonClick *evt) {
  switch (evt->id) {
  case ButtonLeft:
    return (evt->state == ButtonPressed ? "left button pressed" : "left button released"); 
  case ButtonRight:
    return (evt->state == ButtonPressed ? "right button pressed" : "right button released"); 
  default:
    return (evt->state == ButtonPressed ? "unknown button pressed" : "unknown button released"); 
  }
}

// interrupt handler for left button
void Interrupt_LeftButton_IRQ() {
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ButtonClick evt;
  evt.id = ButtonLeft; // by definition, this interrupt handler should only be called for left button
  evt.state = ( digitalRead(MOUSE_BUTTON_LEFT_PIN) == HIGH ? ButtonPressed : ButtonReleased ); // read gpio state
  
  // send the event to queue
  xQueueSendFromISR(Queue_buttonQueue, &evt, &xHigherPriorityTaskWoken);
  
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// interrupt handler for right button
void Interrupt_RightButton_IRQ() {
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ButtonClick evt;
  evt.id = ButtonRight; // by definition, this interrupt handler should only be called for right button
  evt.state = ( digitalRead(MOUSE_BUTTON_RIGHT_PIN) == HIGH ? ButtonPressed : ButtonReleased ); // read gpio state
  
  // send the event to queue
  xQueueSendFromISR(Queue_buttonQueue, &evt, &xHigherPriorityTaskWoken);
  
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void buttonEventHandler( void *pvParameters) {
  ButtonClick buttonEvt;

  // configure pins as inputs
  pinMode(MOUSE_BUTTON_LEFT_PIN, INPUT);
  pinMode(MOUSE_BUTTON_RIGHT_PIN, INPUT);

  // register interrupt handlers
  attachInterrupt(digitalPinToInterrupt(MOUSE_BUTTON_LEFT_PIN), Interrupt_LeftButton_IRQ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOUSE_BUTTON_RIGHT_PIN), Interrupt_RightButton_IRQ, CHANGE);

  while(1) {
    if (xQueueReceive(Queue_buttonQueue, &buttonEvt, portMAX_DELAY) != pdPASS) {
      continue;
    }

    // got button event -> do something with it
    // for now, just print to //Serial
    // next phase: usb mouse clicks
    // next phase: bluetooth mouse clicks

    //Serial.println(//SerializeButtonEvent(&buttonEvt));

    if (buttonEvt.id == ButtonLeft) {
      if (buttonEvt.state == ButtonPressed) {
        Mouse.press(MOUSE_LEFT);
      } else if (buttonEvt.state == ButtonReleased) {
        Mouse.release(MOUSE_LEFT);
      }
    } else if (buttonEvt.id == ButtonRight) {
      if (buttonEvt.state == ButtonPressed) {
        Mouse.press(MOUSE_RIGHT);
      } else if (buttonEvt.state == ButtonReleased) {
        Mouse.release(MOUSE_RIGHT);
      }
    }
  }
}

//*****************************************************************

void setup() 
{

  Mouse.begin();
  ////Serial.begin(115200);

  delay(1000); // prevents usb driver crash on startup, do not omit this
  //while (!Serial) ;  // Wait for //Serial terminal to open port before starting program
  if (!IMU.begin()) {
    //Serial.println("failed to intialize IMU!");
  }

  //Serial.println("");
  //Serial.println("******************************");
  //Serial.println("        Program start         ");
  //Serial.println("******************************");
  //Serial.flush();

  // Set the led the rtos will blink when we have a fatal rtos error
  // RTOS also Needs to know if high/low is the state that turns on the led.
  // Error Blink Codes:
  //    3 blinks - Fatal Rtos Error, something bad happened. Think really hard about what you just changed.
  //    2 blinks - Malloc Failed, Happens when you couldn't create a rtos object. 
  //               Probably ran out of heap.
  //    1 blink  - Stack overflow, Task needs more bytes defined for its stack! 
  //               Use the taskMonitor thread to help gauge how much more you need
  vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

  // sets the //Serial port to print errors to when the rtos crashes
  // if this is not set, //Serial information is not printed by default
 // vSetErrorSerial(&Serial);

  // Create the times that will be managed by the rtos
  Timer_idleTimer = xTimerCreate("Idle timer", pdMS_TO_TICKS(15000), pdFALSE, (void *) 0, idleTimerCallback);
  if (Timer_idleTimer == NULL) {
    //Serial.println("failed to create idle timer!");
  }

  // Create the queue where motion events are sent through
  Queue_motionQueue = xQueueCreate(MOTION_QUEUE_SIZE, sizeof(Motion));
  if (Queue_motionQueue == NULL) {
    //Serial.println("failed to create motion queue!");
  }
  // Create the queue where button events are sent through
  Queue_buttonQueue = xQueueCreate(BUTTON_QUEUE_SIZE, sizeof(ButtonClick));
  if (Queue_buttonQueue == NULL) {
    //Serial.println("failed to create button queue!");
  }

  // Create the relevant tasks:
  // - gyro handler
  // - motion event processor
  // - click event processor
  xTaskCreate(threadGyro,          "Task Gyro",                  256, NULL, tskIDLE_PRIORITY + 1, &Handle_gyroTask);
  xTaskCreate(motionEventHandler,  "Task MotionHandler",         256, NULL, tskIDLE_PRIORITY + 2, &Handle_motionHandlerTask);
  xTaskCreate(buttonEventHandler,  "Task ButtonEventHandler",    256, NULL, tskIDLE_PRIORITY + 4, &Handle_buttonHandlerTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while(1)
  {
	  //Serial.println("Scheduler Failed! \n");
	  //Serial.flush();
	  delay(1000);
  }

}

//*****************************************************************
// This is now the rtos idle loop
// No rtos blocking functions allowed!
//*****************************************************************
void loop() 
{
    // Optional commands, can comment/uncomment below
    //Serial.print("."); //print out dots in terminal, we only do this when the RTOS is in the idle state
    //Serial.flush();
    delay(100); //delay is interrupt friendly, unlike vNopDelayMS
}


//*****************************************************************

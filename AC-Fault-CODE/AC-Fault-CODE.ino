/*
    Quick sketch (Arduino-speak C++) to show FreeRTOS tasks running and loop()

    Sketch uses 13244 bytes (41%) of program storage space. Maximum is 32256 bytes.
    Global variables use 563 bytes (27%) of dynamic memory, leaving 1485 bytes for
    local variables. Maximum is 2048 bytes.
*/
//Test to commit to GIT

#include <Arduino_FreeRTOS.h>
#include <LibPrintf.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h> // I2C LCD
#include <queue.h>
#include <Wire.h> // I2C communication

//Define Queue
#define QUEUE_LENGTH 9
#define QUEUE_ITEM_SIZE sizeof(int)

QueueHandle_t dataQueue;

//Values used in Calculations // These values would be better in a Queue.
int mVperAmp = 100;
float I1 = 0.00;
float I2 = 0.00;
float I3 = 0.00;
int FT = 1;
int MAP = 0;
double pickup_current = 0.55;
double fault_current = 0;
double Voltage1 = 0;
double Voltage2 = 0;
double Voltage3 = 0;
double Voltage4 = 0;
double VRMS1 = 0;
double VRMS2 = 0;
double VRMS3 = 0;
double VRMS4 = 0;
double AmpsRMS1 = 0;
double AmpsRMS2 = 0;
double AmpsRMS3 = 0;
double AmpsRMS4 = 0;

// The GPIO pins for the devices
constexpr uint8_t redLED = 4; //Test Pins for test functions
constexpr uint8_t grnLED = 12;
constexpr uint8_t Buzzer_Pin = 13; // Pin for the buzzer
constexpr uint8_t LCD_1 = 11; //TBD
constexpr uint8_t LCD_2 = 10; //TBD
constexpr uint8_t Relays = 7; //Connection point for all the relays
constexpr uint8_t Button_Loc = 1; //Connection point for all the relays
constexpr uint8_t Micro_Trans = 6; //Transmit pin for the ESP
constexpr uint8_t Micro_Rec = 5; //Receive pin from the ESP
constexpr uint8_t Analog0 = PIN_A0; // connection for the Neutral line
constexpr uint8_t Analog1 = PIN_A1; // connection for the A phase.
constexpr uint8_t Analog2 = PIN_A2; // connection for the B phase.
constexpr uint8_t Analog3 = PIN_A3; // connection for the C phase. 
// I2C LCD Configuration
constexpr uint8_t LCD_COLS = 16;
constexpr uint8_t LCD_ROWS = 2;
constexpr uint8_t LCD_ADDR = 0x27; // I2C address
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// Two blink tasks (forward declarations)
void TaskBlinkRed( void *pvParameters );
void TaskBlinkGrn( void *pvParameters ); //Blinks light Shows the system is active
void TaskDisplayData( void *pvParameters ); //Displays input onto the LCD screen
void TaskReadCurrent( void *pvParameters ); //get the current values from each of the meters
void TaskActivate_Buzzer( void *pvParameters ); //Trigger the Buzzer
void TaskSPI_Communication( void *pvParameters ); //Read & Write Data to the STM microcontroller
void TaskRelay_Switch( void *pvParameters ); //Toggle the Relays
void TaskButton_Press( void *pvPatameters ); //Gets the button press

// Handles to the tasks (if we need them - optional)
TaskHandle_t redTask;
TaskHandle_t grnTask;
TaskHandle_t DisplayData; 
TaskHandle_t ReadCurrent; 
TaskHandle_t Activate_Buzzer; 
TaskHandle_t SPI_Communication; 
TaskHandle_t Relay_Switch; 
TaskHandle_t Button_Press; 

//LCD Struct to be passed into the task
struct LCD_Pin_Struct{
   int LCD_1_Pin;
   int LCD_2_Pin;
   int LCD_Pointer_1;
   int LCD_Pointer_2;
  };
struct Analog_Pins{
   int Pin_Analog_0;
   int Pin_Analog_1;
   int Pin_Analog_2;
   int Pin_Analog_3;
  };
struct Micro_Communication{
   int Transmit;
   int Receive;
  };

// SETUP
void setup() {
  Serial.begin(9600); //Begin the Serial Terminal
  printf("Setup started with portTICK_PERIOD_MS:%d\n", portTICK_PERIOD_MS);

  // GPIO pins for LEDs. Specifies if each pin is an input or and output 
  pinMode(redLED, OUTPUT);
  pinMode(grnLED, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT); 
  pinMode(LCD_1, OUTPUT); //TBD
  pinMode(LCD_2, OUTPUT); //TBD
  pinMode(Micro_Trans, OUTPUT); 
  pinMode(Micro_Rec, INPUT); 
  pinMode(Analog0, INPUT); 
  pinMode(Analog1, INPUT); 
  pinMode(Analog2, INPUT); 
  pinMode(Analog3, INPUT);
  delay(1000);
    
    Wire.begin(19, 18); // SCL-pin 19 SDA-pin 18
    lcd.init();
    lcd.backlight();

  printf("Begin");
  //Fill out the Structs
  LCD_Pin_Struct LCD = {LCD_1,LCD_2,&LCD_1,&LCD_2};
  Analog_Pins Current = {Analog0,Analog1,Analog2,Analog3};
  Micro_Communication COM = {Micro_Trans, Micro_Rec};

  //Create the Queue to send the Measurements on.
  dataQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (dataQueue == NULL) {
        Serial.println("Failed to create queue!");
        while (1);
    }


  // Stack Sizes can be optimized once functionality is proven.
  // Now set up two tasks to run independently.
  //         (Func_name,  User_name, Stk,          Parameters,  Priority,          Handler); //xTaskCreate(,,,,,)
  xTaskCreate(TaskBlink, "BlinkGrn", 256,    (void *) &grnLED,         2,         &grnTask); //Task to Blink LED
  xTaskCreate(TaskBlink, "BlinkRed", 200,    (void *) &redLED,         1,         &redTask); //Task to Blink LED
  xTaskCreate(   myLoop,     "Loop", 256,                NULL,         2,             NULL); //Task to Maintain the RTOS Loop
  xTaskCreate(   Buzzer, "Tog_Buzz",  16,(void *) &Buzzer_Pin,         1, &Activate_Buzzer); //Task to Toggle the Buzzer
  xTaskCreate( Disp_LCD, "Dis_Info", 512,       (void *) &LCD,         1,     &DisplayData); //Task to Display information to the LCD Screen
  xTaskCreate( I_Reader, "Read_Cur", 256,   (void *) &Current,         1,     &ReadCurrent); //Task to get the readings of the current
  xTaskCreate(Tog_Relys, "Tog_Rlys",   4,    (void *) &Relays,         1,    &Relay_Switch); //Task to toggle the relays //We may need to implement a Semiphore for the Relay,Button,& Buzzer Functions.
  xTaskCreate(Button_Pr,"Get_Press",  16,(void *) &Button_Loc,         2,    &Button_Press); //Task to get Button Press.  
  //create communication protocol for arduino to other board. 
  
  //StartUpMessage()      //This may not work with RTOS.        
  printf("Setup completed\n");
  //vTaskStartScheduler();
}

// void StartUpMessage(){
//   lcd.print("AC TRANSMISSION");
//   lcd.setCursor (0, 1);
//   lcd.print("      LINE     ");
//   delay(1000);
//   lcd.clear();
//   lcd.print("FAULT DECTECTION");
//   lcd.setCursor (0, 1);
//   lcd.print("     SYSTEM"); 
//   delay(1000);
// }


//Task Definitions
void Buzzer(void * parameters) {
  printf("This is task: %s\n", pcTaskGetName(NULL));

  for (;;) {
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
LiquidCrysral lcd();// TBD // include LiquidCrysral lib
double rmsVoltages[3]; // Array to store RMS voltages for phases 1, 2, 3
double rmsCurrent[3]; // Array to store RMS current for phases 1, 2, 3

// Function to calculate RMS values for a given phase
double calculateRMS(const double* voltageSamples, double& rmsVoltage, int numSamples) {
    double sumOfSquares = 0.0;
    for (int i = 0; i < numSamples; i++) {
      sumOfSquares += voltageSamples[i] * voltageSamples[i];
    }
    double meanOfSquares = sumOfSquares / numSamples;
    return sqrt(meanOfSquares);
    
  //store an ary of voltage samples
// struct CurrentReadings  {
//     double value_0[10];
//     double value_2[10];
//     double value_3[10];
  }; 
    
}

void Disp_LCD(void * parameters) {
    // update the display to use the i2c protocol. the pins on the arduino are scl: 19 and sda: 18. 
  printf("This is task: %s\n", pcTaskGetName(NULL));
  

    //initialize the LCD 
  CurrentReadings readings; //use struct to receive current readings
  const int numSamples = 10; // Adjust based on sampling rate
    
  for (;;) {
    
    if (xQueueReceive(dataQueue, &readings, portMAX_DELAY) == pdPASS) { //If the Measurements are received then display them to the LCD.
            // Process received data (e.g., print to Serial)
        for (int phase = 0; phase < 3; phase++) {
              conts double* voltageSamples = nullptr;
              double sumOfSquares = 0;

              // for (int i = 0; i < numSamples; i++) {
              // double voltage = 0;
              // switch (phase) {
              //   case 0: voltageSamples = readings.Value_0; break;
              //   case 1: voltageSamples = readings.Value_1; break;
              //   case 2: voltageSamples = readings.Value_2; break;
              // }
              
              rmsVoltages[phase] = calculateRMS(voltageSample, numSamples);
              rmsCurrent[phase] = rmsVoltages[phase] / mVperAmp;    //Calculate RMS current

            }
            // Display RMS Current for each phase on the LCD
              lcd.clear();
              for (int phase = 0; phase < 3; phase++) {
                lcd.setCursor(0, phase); // Set cursor position
                lcd.print("P");
                lcd.print(phase + 1);
                lcd.print(": ");
                lcd.print(rmsCurrent[phase], 2); // Display with 2 decimal places
                lcd.print("A"); 
              }
            // lcd.setCursor(0,0);
            // lcd.print("RMS Current:")
            // lcd.setCursor(0,1);
            // lcd.print(receivedData); // TBA for data format
            // Serial.print("Task2: Received data: ");
            // Serial.println(receivedData);
            
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

//int Measurement_Calaculations {}


struct Current_Readings {
  double Value_0;
  double Value_1;
  double Value_2;
  double Value_3;
};

struct Min_Max {
  int min;
  int max; 
};

void I_Reader(void *parameters) {
  struct Analog_Pins *data = (struct Analog_Pins *)parameters;
    

  for (;;) {
    uint32_t start_time = millis();
    struct Current_Readings Measurements = {0, 0, 0, 0}; // Initialize readings
    struct Min_Max Extrema_0 = {0,1024}; //Initialize Minimum and maximum values
    struct Min_Max Extrema_1 = {0,1024};
    struct Min_Max Extrema_2 = {0,1024};
    struct Min_Max Extrema_3 = {0,1024};

    // Sample analog values for half a second
    while ((millis() - start_time) < 500) {
      Measurements.Value_0 = analogRead(data->Pin_Analog_0);
      Measurements.Value_1 = analogRead(data->Pin_Analog_1);
      Measurements.Value_2 = analogRead(data->Pin_Analog_2);
      Measurements.Value_3 = analogRead(data->Pin_Analog_3);

      Sample_data(Measurements.Value_0,Extrema_0);
      Sample_data(Measurements.Value_1,Extrema_1);
      Sample_data(Measurements.Value_2,Extrema_2);
      Sample_data(Measurements.Value_3,Extrema_3);
    }
      Measurements.Value_0 = ((Extrema_0.max - Extrema_0.min) * 5.0)/1024.0; //vol
      Measurements.Value_1 = ((Extrema_1.max - Extrema_1.min) * 5.0)/1024.0;
      Measurements.Value_2 = ((Extrema_2.max - Extrema_2.min) * 5.0)/1024.0;
      Measurements.Value_3 = ((Extrema_3.max - Extrema_3.min) * 5.0)/1024.0;

    
    // Send measurements to the queue
    if (xQueueSend(dataQueue, &Measurements, portMAX_DELAY) != pdPASS) {
      Serial.println("Failed to send data to queue!");
    }

    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500 ms
  }
}

void Sample_data(int reading, struct Min_Max &Current_Extrema)
{
  // Finds the minimum and maximum values in the half a second time and then updates the Min_Max struct
  if (reading > Current_Extrema.max) 
  {
    Current_Extrema.max = reading;
  }
  else if (reading < Current_Extrema.min)
  {
    Current_Extrema.min = reading;
  } 
}




void Tog_Relys(void * parameters) {
  printf("This is task: %s\n", pcTaskGetName(NULL));

  for (;;) {
    if (fault_current > pickup_current)
      // open the relays to disconnect the faulty line
      digitalWrite(Relays, HIGH) // assuming HIGH means relay is open
      Serial.println("Fault detected! relays opened");
    } else {
      // close the relays if no fault
      digitalWrite(Relays, LOW); // assuming LOW means relay is closed
      Serial.println("No fault. Relays closed.")

  }
    vTaskDelay(500/ portTICK_PERIOD_MS);
  }
}

void Button_Pr(void * parameters) {
  printf("This is task: %s\n", pcTaskGetName(NULL));

  for (;;) {
     //read button state
    int buttonState = digitalRead(Button_Loc);

    if (buttonState == HIGH) {
      //perform actions based on button press e.g, reset the system, acknowledge alarms or change settings
      Serial.println("Button pressed! Performing action.")
      // add the button press handling code here
    }
    vTaskDelay( 500 / portTICK_PERIOD_MS);
  }
}


// Common blink task
void TaskBlink(void * parameters) {
  printf("This is task: %s\n", pcTaskGetName(NULL));

  uint8_t ledPin = *((uint8_t *)parameters);
  printf("Using LED pin %d\n", ledPin);

  // Must never end. Variable delay depending on LED colour
  for (;;) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    vTaskDelay((ledPin == 4 ? 200 : 500) / portTICK_PERIOD_MS);
  }
}

// Now used as part of the idle task - don't use! Whenever I tried
// to use the loop it just crashed the whole sketch (heap memory, maybe)
// if I introduced any kind of delay() or vTaskDelay().
// Whatever the cause, I recommend avoid using the loop;
void loop() {
  static unsigned long lastMillis = 0;
  static unsigned long myCount = 0;

  if (millis() - lastMillis > 750) {
    printf("std Loop: %lu\n", myCount++);
    lastMillis = millis();
  }
}

// Replacement loop under my control
void myLoop() {
  static unsigned long myCount = 0;
  for (;;) {
    printf("myLoop: %lu\n", myCount++);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

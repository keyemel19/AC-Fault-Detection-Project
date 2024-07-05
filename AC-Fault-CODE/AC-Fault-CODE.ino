/*
    Quick sketch (Arduino-speak C++) to show FreeRTOS tasks running and loop()

    Sketch uses 13244 bytes (41%) of program storage space. Maximum is 32256 bytes.
    Global variables use 563 bytes (27%) of dynamic memory, leaving 1485 bytes for
    local variables. Maximum is 2048 bytes.
*/
//Test to commit to GIT

#include <Arduino_FreeRTOS.h>
#include <LibPrintf.h>
#include <LiquidCrystal_I2C.h> // I2C LCD
#include <queue.h>
#include <Wire.h> // I2C communication
#include <Arduino.h>



LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address (address,Column,row)
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
constexpr uint8_t Buzzer_Pin = 11; // Pin for the buzzer
constexpr uint8_t LCD_1 = 9; //TBD
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

//LCD Struct to be passed into the task. These pass in pin information to each task
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

//Structures for the passing of information
struct Current_Readings {
  float Value_0; //This is used to get the reading from the device and is updated to be RMS Voltage
  float Value_1;
  float Value_2;
  float Value_3;
  float Current_0; //RMS Current
  float Current_1;
  float Current_2;
  float Current_3;
};

//Define Queue
QueueHandle_t structQueue;

struct Min_Max {
  int min;
  int max; 
};


//Task Definitions
void Buzzer(void * parameters) { // This uses PWM to ramp up and down pins 3,5,6,9,10, & 11 can be used
  printf("This is task: %s\n", pcTaskGetName(NULL));
  uint8_t buzzerPin = *((uint8_t *)parameters); //get the buzzer pin
  int frequency = 1000; // Starting frequency in Hertz
  int increment = 10;   // Frequency increment value
  int maxFrequency = 3000; // Maximum frequency in Hertz

  for (;;) {
    
    if (0){ // This should sound only if the Interrupt Semaphore is triggered
      for (int i = 0; i <= maxFrequency; i += increment) {
        tone(buzzerPin, i);
        delay(10); // Adjust ramp-up speed by changing delay value
      }

      // Ramp-down
      for (int i = maxFrequency; i >= frequency; i -= increment) {
        tone(buzzerPin, i);
        delay(10); // Adjust ramp-down speed by changing delay value
      }

      noTone(buzzerPin); // Stop the buzzer after the siren sound
      delay(1000); // Delay between siren cycles
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Function to calculate RMS values for a given phase
double calculateRMS(const double* voltageSamples, double& rmsVoltage, int numSamples) {
    double sumOfSquares = 0.0;
    for (int i = 0; i < numSamples; i++) {
      sumOfSquares += voltageSamples[i] * voltageSamples[i];
    }
    double meanOfSquares = sumOfSquares / numSamples;
    return sqrt(meanOfSquares);
}


void Disp_Current_and_Voltage(char string[],float voltage, float current ){
  lcd.clear();
  lcd.setCursor(0, 0); // Set cursor position to the top
  lcd.print(string);
  lcd.setCursor(0, 1); // Set cursor position to the top
  lcd.print("V: ");
  lcd.print(voltage);
  lcd.print(" A: ");
  lcd.print(current);
  
  unsigned long startTime = millis(); // Get the current time
  while (millis() - startTime < 1000) {} // Wait in a loop until the desired delay period has elapsed
  //vTaskDelay(500); //This will cause a feedback loop to the Disp function locking the system
}

void Disp_LCD(void * parameters) { 
  printf("This is task: %s\n", pcTaskGetName(NULL));
  struct Current_Readings readings = {0, 0, 0, 0, 0, 0, 0, 0};
  for (;;) {
    //lcd.clear();
    //vTaskDelay(pdMS_TO_TICKS(500));
    //lcd.print("CALLED");
    if (xQueueReceive(structQueue, &readings, portMAX_DELAY) == pdPASS) { //If the Measurements are received then display them to the LCD.
      
      /* struct Current_Readings { //This is currently in the readings pointer
        float Value_0; //This should be RMS voltage for neutral
        float Value_1; //Phase 1
        float Value_2; //Phase 2
        float Value_3; //Phase 3
        float Current_0;//Neutral
        float Current_1;//Phase 1
        float Current_2;//Phase 2
        float Current_3;//Phase 3
        };
      */
      printf("got");
      Disp_Current_and_Voltage("RED Line"   ,readings.Value_1,readings.Current_1);
      Disp_Current_and_Voltage("BLUE Line"  ,readings.Value_2,readings.Current_2);
      Disp_Current_and_Voltage("YELLOW Line",readings.Value_3,readings.Current_3);
      Disp_Current_and_Voltage("NEUTRAL"    ,readings.Value_0,readings.Current_0);
                 
    }
    
    lcd.clear();
    vTaskDelay(500);
    
  }
}


//int Measurement_Calaculations {}




void I_Reader(void *parameters) {
  struct Analog_Pins *data = (struct Analog_Pins *)parameters;
  printf("This is task: %s\n", pcTaskGetName(NULL));

  for (;;) {
    uint32_t start_time = millis();
    struct Current_Readings Measurements = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize readings
    struct Min_Max Extrema_0 = {0,1024}; //Initialize Minimum and maximum values
    struct Min_Max Extrema_1 = {0,1024};
    struct Min_Max Extrema_2 = {0,1024};
    struct Min_Max Extrema_3 = {0,1024};

    // Sample analog values for half a second
    while ((millis() - start_time) < 500) {
      Measurements.Value_0 = analogRead(data->Pin_Analog_0); // Collect Readings from each pin
      Measurements.Value_1 = analogRead(data->Pin_Analog_1);
      Measurements.Value_2 = analogRead(data->Pin_Analog_2);
      Measurements.Value_3 = analogRead(data->Pin_Analog_3);
      
      Sample_data(Measurements.Value_0,Extrema_0); //Find the minimum and maximum values
      Sample_data(Measurements.Value_1,Extrema_1);
      Sample_data(Measurements.Value_2,Extrema_2);
      Sample_data(Measurements.Value_3,Extrema_3);
    }
    Measurements.Value_0 = (((Extrema_0.max - Extrema_0.min) * 5.0)/1024.0)/2.0 *0.707; //RMS voltage calculations
    Measurements.Value_1 = (((Extrema_1.max - Extrema_1.min) * 5.0)/1024.0)/2.0 *0.707;
    Measurements.Value_2 = (((Extrema_2.max - Extrema_2.min) * 5.0)/1024.0)/2.0 *0.707;
    Measurements.Value_3 = (((Extrema_3.max - Extrema_3.min) * 5.0)/1024.0)/2.0 *0.707;
    
    Serial.println(Measurements.Value_0);
    Serial.println(Measurements.Value_1);
    Serial.println(Measurements.Value_2);
    Serial.println(Measurements.Value_3);



    Measurements.Current_0 = Measurements.Value_0 *1000/100;
    Measurements.Current_1 = Measurements.Value_1 *1000/100;
    Measurements.Current_2 = Measurements.Value_2 *1000/100;
    Measurements.Current_3 = Measurements.Value_3 *1000/100;


    Serial.println(Measurements.Current_0);
    Serial.println(Measurements.Current_1);
    Serial.println(Measurements.Current_2);
    Serial.println(Measurements.Current_3);
    // Send measurements to the queue
    //printf(Measurements.Current_0);
    xQueueSend(structQueue, &Measurements, portMAX_DELAY);
    // if (xQueueSend(dataQueue, &Measurements, portMAX_DELAY) != pdPASS) {
    //   Serial.println("Failed to send data to queue!");
    // }
    vTaskDelay(500); // Delay for 500 ms //this seems to crash the task
    printf("Complete-V\n");
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
    if (fault_current > pickup_current){
      // open the relays to disconnect the faulty line
      digitalWrite(Relays, HIGH); // assuming HIGH means relay is open
      Serial.println("Fault detected! relays opened");
    } else {
      // close the relays if no fault
      digitalWrite(Relays, LOW); // assuming LOW means relay is closed
      Serial.println("No fault. Relays closed.");

  }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Button_Pr(void * parameters) {
  printf("This is task: %s\n", pcTaskGetName(NULL));

  for (;;) {
     //read button state
    int buttonState = digitalRead(Button_Loc);

    if (buttonState == HIGH) {
      //perform actions based on button press e.g, reset the system, acknowledge alarms or change settings
      Serial.println("Button pressed! Performing action.");
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

// SETUP
void setup() {
  Serial.begin(9600); //Begin the Serial Terminal
  printf("Setup started with portTICK_PERIOD_MS:%d\n", portTICK_PERIOD_MS);

  // GPIO pins for LEDs. Specifies if each pin is an input or and output 
  pinMode(redLED, OUTPUT);
  pinMode(grnLED, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT); 
  pinMode(Relays, OUTPUT); 
  pinMode(Micro_Trans, OUTPUT); 
  pinMode(Micro_Rec, INPUT); 
  pinMode(Analog0, INPUT); 
  pinMode(Analog1, INPUT); 
  pinMode(Analog2, INPUT); 
  pinMode(Analog3, INPUT);
  
  lcd.init();
  lcd.backlight(); 
  lcd.begin(2, 16); // SCL-pin 19 SDA-pin 18
  

  printf("Begin\n");

  //Fill out the Structs
  Analog_Pins Pins_A = {Analog0,Analog1,Analog2,Analog3};
  Micro_Communication COM = {Micro_Trans, Micro_Rec};

  //Create the Queue to send the Measurements on.
  // I2C address
  structQueue = xQueueCreate(2, sizeof(struct Current_Readings));
    if (structQueue == NULL) {
        Serial.println("Failed to create queue!");
        //while (1);
    }


  lcd.setCursor (0, 0);
  lcd.print("AC TRANSMISSION");
  lcd.setCursor (0, 1);
  lcd.print("      LINE     ");
  unsigned long startTime = millis(); // Get the current time
    // Wait in a loop until the desired delay period has elapsed
  while (millis() - startTime < 1000) {}  // wait for 1 second. This has to be done like this because delay cannot be used. and the scheduler has not been initialized.
  lcd.clear();
  lcd.print("FAULT DETECTION");
  lcd.setCursor (0, 1);
  lcd.print("     SYSTEM"); 

  // Stack Sizes can be optimized once functionality is proven.
  // Now set up two tasks to run independently.
  //         (Func_name,  User_name, Stk,          Parameters,  Priority,          Handler); //xTaskCreate(,,,,,)
  //xTaskCreate(TaskBlink, "BlinkGrn", 128,    (void *) &grnLED,         2,         &grnTask); //Task to Blink LED
  //xTaskCreate(TaskBlink, "BlinkRed", 64,    (void *) &redLED,         1,         &redTask); //Task to Blink LED
  //xTaskCreate(   myLoop,     "Loop", 256,                NULL,         2,             NULL); //Task to Maintain the RTOS Loop
  //xTaskCreate(   Buzzer, "Tog_Buzz", 64,(void *) &Buzzer_Pin,         1,  &Activate_Buzzer); //Task to Toggle the Buzzer
  xTaskCreate( Disp_LCD, "Dis_Info", 128,       NULL,         2,     NULL); //Task to Display information to the LCD Screen
  xTaskCreate( I_Reader, "Read_Cur", 128,   (void *) &Pins_A,         2,     &ReadCurrent); //Task to get the readings of the current
  //xTaskCreate(Tog_Relys, "Tog_Rlys",  256,    (void *) &Relays,     2,    &Relay_Switch); //Task to toggle the relays //We may need to implement a Semiphore for the Relay,Button,& Buzzer Functions.
  //xTaskCreate(Button_Pr,"Get_Press",  16,(void *) &Button_Loc,         2,    &Button_Press); //Task to get Button Press.  
  //create communication protocol for arduino to other board. 
  //printf(uxTaskGetStackHighWaterMark(NULL));
  //StartUpMessage()      //This may not work with RTOS.        
  printf("Setup completed\n");
  vTaskStartScheduler();
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

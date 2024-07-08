#include <Arduino_FreeRTOS.h>
//#include <LibPrintf.h>
#include <LiquidCrystal_I2C.h> // I2C LCD
#include <queue.h>
//#include <Wire.h> // I2C communication
//#include <Arduino.h>
#include <semphr.h>



LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address (address,Column,row)
//Values used in Calculations // These values would be better in a Queue.
int FT = 1;
int MAP = 0;
double pickup_current = 0.55;
double fault_current = 0;
char Fault_Type[16] ="Blank";

QueueHandle_t structQueue;
QueueHandle_t faultQueue;
SemaphoreHandle_t InterruptSemaphore;
SemaphoreHandle_t WriteSemaphore;
// The GPIO pins for the devices
constexpr uint8_t Buzzer_Pin = 11; // Pin for the buzzer
constexpr uint8_t Relays = 7; //Connection point for all the relays
constexpr uint8_t Button_Loc = 1; //Connection point for all the relays
constexpr uint8_t Micro_Trans = 6; //Transmit pin for the ESP
constexpr uint8_t Micro_Rec = 5; //Receive pin from the ESP
constexpr uint8_t Analog0 = PIN_A0; // connection for the Neutral line
constexpr uint8_t Analog1 = PIN_A1; // connection for the A phase.
constexpr uint8_t Analog2 = PIN_A2; // connection for the B phase.
constexpr uint8_t Analog3 = PIN_A3; // connection for the C phase. 

// Two blink tasks (forward declarations)
void TaskDisplayData( void *pvParameters ); //Displays input onto the LCD screen
void TaskReadCurrent( void *pvParameters ); //get the current values from each of the meters
void TaskSPI_Communication( void *pvParameters ); //Read & Write Data to the STM microcontroller
void TasktaskRelay( void *pvParameters ); //Toggle the Relays


// Handles to the tasks (if we need them - optional)
TaskHandle_t DisplayData; 
TaskHandle_t ReadCurrent; 
TaskHandle_t Activate_Buzzer; 
TaskHandle_t SPI_Communication; 
TaskHandle_t Button_Press; 

struct Micro_Communication{
   int Transmit;
   int Receive;
  };

struct Current_Readings { //This is currently in the readings pointer
      double Value_0; //This should be RMS voltage for neutral
      double Value_1; //Phase 1
      double Value_2; //Phase 2
      double Value_3; //Phase 3
      double Current_0;//Neutral
      double Current_1;//Phase 1
      double Current_2;//Phase 2
      double Current_3;//Phase 3
};
struct Min_Max {
  double min;
  double max;
};
void setup() {
  Serial.begin(9600); 
  // GPIO pins for LEDs. Specifies if each pin is an input or and output 
  pinMode(Buzzer_Pin, OUTPUT); 
  pinMode(Relays, OUTPUT);
  pinMode(Button_Loc, INPUT);  
  pinMode(Micro_Trans, OUTPUT); 
  pinMode(Micro_Rec, INPUT); 
  pinMode(Analog0, INPUT); 
  pinMode(Analog1, INPUT); 
  pinMode(Analog2, INPUT); 
  pinMode(Analog3, INPUT);
  
  lcd.init();
  lcd.backlight(); 
  lcd.begin(2, 16); // SCL-pin 19 SDA-pin 18

  Serial.print("Begin\n");

  //Create the Queuse to send data to LCD print task
  structQueue = xQueueCreate(1, sizeof(struct Current_Readings));
  if (structQueue == NULL) {
      Serial.println("Failed to create queue!");
      while (1);
  }
  faultQueue = xQueueCreate(1, sizeof(Fault_Type)); //This only needs to send the fault type once.
  if (faultQueue == NULL) {
      Serial.println("Failed to create queue!");
      while (1);
  }

  InterruptSemaphore = xSemaphoreCreateBinary();
  //WriteSemaphore = xSemaphoreCreateBinary();

  Serial.print("Queues & Semaphores Initialized\n");
  lcd.setCursor (0, 0);
  lcd.print("AC TRANSMISSION");
  lcd.setCursor (0, 1);
  lcd.print("      LINE     ");
  unsigned long startTime = millis(); // Get the current time
  while (millis() - startTime < 500) {}  // wait for 1 second. This has to be done like this because delay cannot be used. and the scheduler has not been initialized.
  lcd.clear();
  lcd.print("FAULT DETECTION");
  lcd.setCursor (0, 1);
  lcd.print("     SYSTEM"); 

  // Stack Sizes can be optimized once functionality is proven.
  //         (Func_name,  User_name, Stk,          Parameters,  Priority,          Handler);
  xTaskCreate(  taskISR, "Task_ISR",  50,    NULL,         2,      NULL);
  xTaskCreate( Disp_LCD, "Dis_Info", 110,    NULL,         1,      NULL); //Task to Display information to the LCD Screen
  xTaskCreate( I_Reader, "Read_Cur", 150,    NULL,         0,      NULL); //Task to get the readings of the current
  Serial.print("Tasks Initialized\n");

  //xSemaphoreGive(InterruptSemaphore);
  vTaskStartScheduler();
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
  
  // unsigned long startTime = millis(); // Get the current time
  // while (millis() - startTime < 500) {} // Wait in a loop until the desired delay period has elapsed
  vTaskDelay(100);
  //vTaskDelay(500); //This will
}

void Disp_LCD(void * parameters) { 
  //Serial.print("This is task: %s\n", pcTaskGetName(NULL));
  struct Current_Readings readings = {0, 0, 0, 0, 0, 0, 0, 0};
  char Fault_Desc[16]= "Blank";
  // while (!Serial) {
  //   vTaskDelay(1);
  // }
  for (;;) {
    Serial.print("Start-P\n");
    if (xQueueReceive(structQueue, &readings, (TickType_t) 1000 ) == pdPASS) { //If the Measurements are received then display them to the LCD.

      //Serial.print("got");
      Disp_Current_and_Voltage("RED Line"   ,readings.Value_1,readings.Current_1);
      Disp_Current_and_Voltage("BLUE Line"  ,readings.Value_2,readings.Current_2);
      Disp_Current_and_Voltage("YELLOW Line",readings.Value_3,readings.Current_3);
      Disp_Current_and_Voltage("NEUTRAL"    ,readings.Value_0,readings.Current_0);
      lcd.clear();         
    }
    if (xQueueReceive(faultQueue, &Fault_Desc, (TickType_t) 1000 ) == pdPASS) { //If the Measurements are received then display them to the LCD.

      lcd.clear();
      lcd.setCursor(0, 0); // Set cursor position to the top
      lcd.print(Fault_Desc);
      lcd.setCursor(0, 1); // Set cursor position to the top
      lcd.print("Fault");
    }
    
    Serial.print("Complete-P\n");
    vTaskDelay(1000);
  }
}

void I_Reader(void *parameters) {
  //struct Analog_Pins *data = (struct Analog_Pins *)parameters;
  //Serial.print("This is task: %s\n", pcTaskGetName(NULL));
  
  for (;;) {
    Serial.print("Start-I\n");
    uint32_t start_time = millis();
    uint8_t Fault_Case =0;
    struct Current_Readings Measurements = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize readings
    struct Min_Max Extrema_0 = {0,1024}; //Initialize Minimum and maximum values
    struct Min_Max Extrema_1 = {0,1024};
    struct Min_Max Extrema_2 = {0,1024};
    struct Min_Max Extrema_3 = {0,1024};
    //xSemaphoreTake(InterruptSemaphore, portMAX_DELAY); //Takes priority of the Semaphore
    // Sample analog values for half a second
    while ((millis() - start_time) < 500) {
      Measurements.Value_0 = analogRead(Analog0); // Collect Readings from each pin
      Measurements.Value_1 = analogRead(Analog1);
      Measurements.Value_2 = analogRead(Analog2);
      Measurements.Value_3 = analogRead(Analog3);
      
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
    //vTaskDelay(1200);
    // Send measurements to the queue
    Fault_Case = 0;//Fault_Check(Measurements.Current_1,Measurements.Current_2,Measurements.Current_3,Measurements.Current_0);
    if (Fault_Case) {
      switch (Fault_Case) {
        case 0:
            strcpy(Fault_Type,"NO FAULT");
            break;
        case 1:
            strcpy(Fault_Type,"RED YELLOW");
            xSemaphoreGive(InterruptSemaphore); //Gives priority of the Semaphore
            break;
        case 2:
            strcpy(Fault_Type,"YELLOW BLUE");
            xSemaphoreGive(InterruptSemaphore);
            break;
        case 3:
            strcpy(Fault_Type,"BLUE RED");
            xSemaphoreGive(InterruptSemaphore);
            break;
        case 4:
            strcpy(Fault_Type,"Three Phase");
            xSemaphoreGive(InterruptSemaphore);
            break;
        case 5:
            strcpy(Fault_Type,"Double line Grd");
            xSemaphoreGive(InterruptSemaphore);
            break;
        case 6:
            strcpy(Fault_Type,"Line to Line");
            xSemaphoreGive(InterruptSemaphore);
            break;
        default:
            strcpy(Fault_Type,"ERROR"); //This should never occur
            break;
      
      }
      if (uxQueueSpacesAvailable(faultQueue) > 0) {
      xQueueSend(faultQueue, &Measurements, portMAX_DELAY);//queue is of length one. It will never hold more then one set of information
      } 
      else {
        Serial.print("faultQueue is full\n");
          // Handle queue full condition
      }
      digitalWrite(Relays, HIGH); //trigger Relays
      Serial.print("Complete-R\n");
      xSemaphoreGive(InterruptSemaphore);
    }
    else if (uxQueueSpacesAvailable(structQueue) > 0) {
    xQueueSend(structQueue, &Measurements, portMAX_DELAY);//queue is of length one. It will never hold more then one set of information
    } 
    else {
      Serial.print("structQueue is full\n");
        // Handle queue full condition
    }
    
    Serial.print("Complete-V\n");
    vTaskDelay(250); // Delay for 500 ms //this seems to crash the task
    }
  
}

int Fault_Check(double Line1, double Line2, double Line3, double Line0){ //line0 is neutral
  int fault_ID =0;
  if (Line1 > pickup_current || Line2 > pickup_current || Line3 > pickup_current) {
    if (Line1 > pickup_current && Line2 > pickup_current) {
      Serial.print("RED GREEN\n");
      fault_ID =1;
    } 
    else if (Line2 > pickup_current && Line3 > pickup_current) {
      Serial.print("GREEN BLUE\n");
      fault_ID =2;
    } 
    else if (Line3 > pickup_current && Line1 > pickup_current) {
      Serial.print("BLUE RED\n");
      fault_ID =3;
    } 
    else {
      if (Line0 < 0.40) {
          Serial.print("Three Phase fault\n");
          fault_ID =4;
      } else if (Line0 >= 0.38) {
          Serial.print("Double line to Ground fault\n");
          fault_ID =5;
      } else {
          Serial.print("Line to line fault\n");
          fault_ID =6;
      }
    }
} else {
    // No fault detected
    Serial.println("NO FAULT\n");
    fault_ID =0;
}
  return fault_ID;
}

void Sample_data(int reading, struct Min_Max &Current_Extrema)
{
  // Finds the minimum and maximum values in the half a second time and then updates the Min_Max struct
  if      (reading > Current_Extrema.max) {Current_Extrema.max = reading;}
  else if (reading < Current_Extrema.min) {Current_Extrema.min = reading;} 
}

void taskISR(void *pvParameters) {
  int buttonState = 0;
    while (1) {
      Serial.println("FAULT\n");
        //if given access to the Semaphore hold onto it an don't let the Current Reader run. 
      if (xSemaphoreTake(InterruptSemaphore, (TickType_t) 5 ) == pdTRUE) {
          digitalWrite(Buzzer_Pin, 1); //Multiple Times
          vTaskDelay(50);
          digitalWrite(Buzzer_Pin, 0);
          vTaskDelay(50);
          Serial.print("Complete-B\n");


          buttonState = digitalRead(Button_Loc); //Check Constantly
          if (buttonState == HIGH) {
          //Release the Semaphore
          digitalWrite(Relays, LOW);//Deactivate Relays
          } 
          else{
            xSemaphoreGive(InterruptSemaphore);
          }
          Serial.print("Complete-Check\n");
      }
        
        //Delay or yield as needed
      Serial.println("NO FAULT\n");
      vTaskDelay(100); // Adjust delay as needed
    }
}

void loop() {}//Serial.print("RUNNING\n");}

#include <Arduino_FreeRTOS.h>
//#include <LibPrintf.h>
#include <LiquidCrystal_I2C.h> // I2C LCD
#include <queue.h>
#include <Wire.h> // I2C communication
//#include <Arduino.h>
#include <semphr.h>


//Wire.setClock(400000); 
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address (address,Column,row)
//Values used in Calculations // These values would be better in a Queue.
int FT = 1;
int MAP = 0;

double fault_current = 0;
char Fault_Type[16] ="Blank";

QueueHandle_t structQueue;
QueueHandle_t faultQueue;
SemaphoreHandle_t InterruptSemaphore;
SemaphoreHandle_t WriteSemaphore;
// The GPIO pins for the devices
constexpr uint8_t Buzzer_Pin = 11; // Pin for the buzzer
constexpr uint8_t Relays = 7; //Connection point for all the relays
constexpr uint8_t Button_Loc = 2; //Connection point for all the relays
constexpr uint8_t Analog0 = PIN_A0; // connection for the Neutral line
constexpr uint8_t Analog1 = PIN_A1; // connection for the A phase.
constexpr uint8_t Analog2 = PIN_A2; // connection for the B phase.
constexpr uint8_t Analog3 = PIN_A3; // connection for the C phase. 

// Two blink tasks (forward declarations)
void Disp_LCD( void *pvParameters ); //Displays input onto the LCD screen
void I_Reader( void *pvParameters ); //get the current values from each of the meters
void taskISR( void *pvParameters ); //Read & Write Data to the STM microcontroller



// Handles to the tasks (if we need them - optional)
TaskHandle_t TaskREADER; //Used

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
  //Serial.begin(9600); 
  // GPIO pins for LEDs. Specifies if each pin is an input or and output 
  pinMode(Buzzer_Pin, OUTPUT); 
  pinMode(Relays, OUTPUT);
  pinMode(Button_Loc, INPUT);  
  pinMode(Analog0, INPUT); 
  pinMode(Analog1, INPUT); 
  pinMode(Analog2, INPUT); 
  pinMode(Analog3, INPUT);
  
  lcd.init();
  lcd.backlight(); 
  lcd.begin(2, 16); // SCL-pin 19 SDA-pin 18

  //Serial.print("Begin\n");

  //Create the Queuse to send data to LCD print task
  structQueue = xQueueCreate(1, sizeof(struct Current_Readings));
  if (structQueue == NULL) {while (1);}
  faultQueue = xQueueCreate(1, sizeof(int)); //This only needs to send the fault type once.
  if (faultQueue == NULL) {while (1);}

  InterruptSemaphore = xSemaphoreCreateBinary();
  //WriteSemaphore = xSemaphoreCreateBinary();

  // Serial.print("Queues & Semaphores Initialized\n");
  // lcd.setCursor (0, 0);
  // lcd.print("AC TRANSMISSION");
  // lcd.setCursor (0, 1);
  // lcd.print("      LINE     ");
  // unsigned long startTime = millis(); // Get the current time
  // while (millis() - startTime < 500) {}  // wait for 1 second. This has to be done like this because delay cannot be used. and the scheduler has not been initialized.
  lcd.clear();
  lcd.print("FAULT DETECTION");
  lcd.setCursor (0, 1);
  lcd.print("     SYSTEM"); 
  digitalWrite(Relays, HIGH);
  // Stack Sizes can be optimized once functionality is proven.
  //         (Func_name,  User_name, Stk,          Parameters,  Priority,          Handler);
  //xTaskCreate( Disp_LCD, "Dis_Info", 140,    NULL,         3,      NULL); //Task to Display information to the LCD Screen
  xTaskCreate( I_Reader, "Read_Cur", 180,    NULL,         2,      &TaskREADER); //Task to get the readings of the current
  xTaskCreate(  taskISR, "Task_ISR",  80,    NULL,         1,      NULL);

  //Serial.print("Tasks Initialized\n");

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
  uint32_t start_time = millis();
  // unsigned long startTime = millis(); // Get the current time
  // while (millis() - startTime < 500) {} // Wait in a loop until the desired delay period has elapsed
  while ((millis() - start_time) < 1000) {}
  //vTaskDelay(500); //This will
}

void Disp_LCD(void * parameters) { 
  //Serial.print("This is task: %s\n", pcTaskGetName(NULL));
  struct Current_Readings readings = {0, 0, 0, 0, 0, 0, 0, 0};
  int FaultCase = 0;
  // while (!Serial) {
  //   vTaskDelay(1);
  // }
  while (1) {
    //Serial.print("Start-P\n");
    if (xQueueReceive(structQueue, &readings, (TickType_t) 10 ) == pdPASS) { //If the Measurements are received then display them to the LCD.

      //Serial.print("got");
      Disp_Current_and_Voltage("RED Line"   ,readings.Value_1,readings.Current_1);
      Disp_Current_and_Voltage("BLUE Line"  ,readings.Value_2,readings.Current_2);
      Disp_Current_and_Voltage("YELLOW Line",readings.Value_3,readings.Current_3);
      Disp_Current_and_Voltage("NEUTRAL"    ,readings.Value_0,readings.Current_0);
      lcd.clear();         
    }
    //if (xQueueReceive(faultQueue, &FaultCase, (TickType_t) 1000 ) == pdPASS) { //If the Measurements are received then display them to the LCD.

      
    
    //Serial.print("Complete-P\n");
    vTaskDelay(100);
  }
}

void I_Reader(void *parameters) {
  //struct Analog_Pins *data = (struct Analog_Pins *)parameters;
  //Serial.print("This is task: %s\n", pcTaskGetName(NULL));
  char Fault_Desc[16]= "Blank";
  int Fault_Case;
  while(1) {
    //Serial.print("Start-I\n");
    uint32_t start_time = millis();
 
    struct Current_Readings Measurements = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize readings
    struct Min_Max Extrema_0 = {1024,0}; //Initialize Minimum and maximum values
    struct Min_Max Extrema_1 = {1024,0};
    struct Min_Max Extrema_2 = {1024,0};
    struct Min_Max Extrema_3 = {1024,0};
    //xSemaphoreTake(InterruptSemaphore, portMAX_DELAY); //Takes priority of the Semaphore
    // Sample analog values for half a second
    while ((millis() - start_time) < 1000) {
      Measurements.Value_0 = analogRead(Analog0); // Collect Readings from each pin
      Measurements.Value_1 = analogRead(Analog1);
      Measurements.Value_2 = analogRead(Analog2);
      Measurements.Value_3 = analogRead(Analog3);
      
      Extrema_0 = Sample_data(Measurements.Value_0,Extrema_0); //Find the minimum and maximum values
      Extrema_1 = Sample_data(Measurements.Value_1,Extrema_1);
      Extrema_2 = Sample_data(Measurements.Value_2,Extrema_2);
      Extrema_3 = Sample_data(Measurements.Value_3,Extrema_3);
    }


    Measurements.Value_0 =Extrema_0.max; //(((Extrema_0.max* 5.0/1024.0))); //+ Extrema_0.min) * 5.0)/1024.0)/(2.0 *0.707); //RMS voltage calculations Extrema_0.max - Extrema_0.min
    Measurements.Value_1 =Extrema_1.max; //(((Extrema_1.max* 5.0/1024.0))); //+ Extrema_1.min) * 5.0)/1024.0)/(2.0 *0.707);
    Measurements.Value_2 =Extrema_2.max; //(((Extrema_2.max* 5.0/1024.0))); //+ Extrema_2.min) * 5.0)/1024.0)/(2.0 *0.707);
    Measurements.Value_3 =Extrema_3.max; //(((Extrema_3.max* 5.0/1024.0))); //+ Extrema_3.min) * 5.0)/1024.0)/(2.0 *0.707);
    
    // Serial.println(Measurements.Value_0);
    // Serial.println(Measurements.Value_1);
    // Serial.println(Measurements.Value_2);
    // Serial.println(Measurements.Value_3);

    Measurements.Current_0 =Extrema_0.min; //Measurements.Value_0 /40;//*1000/100;
    Measurements.Current_1 =Extrema_1.min; //Measurements.Value_1 /40;//*1000/100;
    Measurements.Current_2 =Extrema_2.min; //Measurements.Value_2 /40;//*1000/100;
    Measurements.Current_3 =Extrema_3.min; //Measurements.Value_3 /40;//*1000/100;

    //Serial.println(Measurements.Current_0);
    // Serial.println(Measurements.Current_1);
    // Serial.println(Measurements.Current_2);
    // Serial.println(Measurements.Current_3);
    //vTaskDelay(1200);
    // Send measurements to the queue
    Fault_Case =  Fault_Check(Measurements.Current_1,Measurements.Current_2,Measurements.Current_3,Measurements.Current_0);
    // print something to lcd
    if (Fault_Case > 0) {
      // print something else to lcd
      // if (uxQueueSpacesAvailable(faultQueue) > 0) {
      //   xQueueSend(faultQueue, Fault_Case, portMAX_DELAY);//queue is of length one. It will never hold more then one set of information
      // } 
      // else {}
      switch (Fault_Case) {
        case 1:
            ////Gives priority of the Semaphore
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("RED YELLOW");
            break;
        case 2:
            
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("YELLOW BLUE");
            break;
        case 3:
            
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("BLUE RED");
            break;
        case 4:
            
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("Three Phase");
            break;
        case 5:
            
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("2x line Grd");
            break;
        case 6:
            
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("Line to Line");
            break;
        default:
             //This should never occur
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position to the top
            lcd.print("ERROR");
            break;
      }
      
      lcd.setCursor(0, 1); // Set cursor position to the top
      lcd.print(Fault_Case);
     //Queue is full
      //vTaskDelay(200);
      xSemaphoreGive(InterruptSemaphore);
    }
    //else if (uxQueueSpacesAvailable(structQueue) > 0) {
    else {
      //xQueueSend(structQueue, &Measurements, portMAX_DELAY);//queue is of length one. It will never hold more then one set of information
      Disp_Current_and_Voltage("RED Line"   ,Measurements.Value_1,Measurements.Current_1);
      Disp_Current_and_Voltage("BLUE Line"  ,Measurements.Value_2,Measurements.Current_2);
      Disp_Current_and_Voltage("YELLOW Line",Measurements.Value_3,Measurements.Current_3);
      Disp_Current_and_Voltage("NEUTRAL"    ,Measurements.Value_0,Measurements.Current_0);
      lcd.clear();   
    }     
    //lcd.print(Fault_Case);
    //Serial.print("Complete-V\n");
    vTaskDelay(100); // Delay for 500 ms //this seems to crash the task
    }
  
}


double pickup_current = 0.55;
uint8_t Fault_Check(double Line1, double Line2, double Line3, double Line0){ //line0 is neutral
  uint8_t fault_ID =0;
  if (Line1 > pickup_current || Line2 > pickup_current || Line3 > pickup_current) {
    if (Line1 > pickup_current && Line2 > pickup_current) {
      //Serial.print("RED GREEN\n");
      fault_ID =1;
    } 
    else if (Line2 > pickup_current && Line3 > pickup_current) {
      //Serial.print("GREEN BLUE\n");
      fault_ID =2;
    } 
    else if (Line3 > pickup_current && Line1 > pickup_current) {
      //Serial.print("BLUE RED\n");
      fault_ID =3;
    }
    else if (Line0 < 0.40) {
      //Serial.print("3 Phase\n");
      fault_ID =4;
    } 
    else if (Line0 >= 0.38) {
      //Serial.print("Double line to Ground fault\n");
      fault_ID =5;
    } 
    else {
      //Serial.print("Line to line fault\n");
      fault_ID =6;
    } 
    //Serial.print(Line0);
    
    //Serial.println("Fault\n");
  } 
  else {
    //No fault detected
    //Serial.println("Checked No Fault\n");
    fault_ID =0;
  }
  return fault_ID;
}

Min_Max Sample_data(int reading, struct Min_Max Current_Extrema)
{
  // Finds the minimum and maximum values in the half a second time and then updates the Min_Max struct
  if (reading > Current_Extrema.max) 
    {Current_Extrema.max = reading;}
  if (reading < Current_Extrema.min) 
    {Current_Extrema.min = reading;} 
  return Current_Extrema;
}

void taskISR(void *pvParameters) {
  int buttonState = digitalRead(Button_Loc); 
  int Occurance = 1;
    while (1) {
      //vTaskDelay(500);
        //if given access to the Semaphore hold onto it an don't let the Current Reader run. 
      if (xSemaphoreTake(InterruptSemaphore, (TickType_t) 10 ) == pdTRUE) {
          if (Occurance){
            digitalWrite(Relays,LOW); //trigger Relays
            vTaskSuspend(TaskREADER); // Suspend the Current Reader Task //THIS MAY CRASH THE CODE
            Occurance =0;
          }
          digitalWrite(Buzzer_Pin, 1); //Multiple Times
          vTaskDelay(50);
          digitalWrite(Buzzer_Pin, 0);
          vTaskDelay(50);
          //Serial.print("Complete-B\n");
          //Serial.println("FAULT\n");

           //Check Constantly
          buttonState = digitalRead(Button_Loc); 
          if (buttonState == HIGH) {
            //Release the Semaphore
            //Serial.print("Complete-Check\n");
            vTaskResume(TaskREADER);
            digitalWrite(Relays, LOW);//Deactivate Relays
            Occurance = 1;
          } 
          else{
            xSemaphoreGive(InterruptSemaphore);
          }
          
          //Serial.print("Complete-Check\n");
          //vTaskDelay(30);
      }
    else {
        //Serial.println("NO FAULT\n");
    } 
      vTaskDelay(100); // Adjust delay as needed
    }
}

void loop() {}//Serial.print("RUNNING\n");}

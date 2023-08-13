#include <mcp2515.h>
#include <SPI.h>

  #define HSPI_MISO   12
  #define HSPI_MOSI   13
  #define HSPI_SCLK   14
  #define HSPI_SS     15

/******************************************************
Pin Declarations
******************************************************/

const int CURRENT_SENSE = 36;  // Analog input pin for the current sensor
const int FSR_1= 2; // Force Sensor 1 Pin
const int FSR_2= 4; // Force Sensor 2 Pin

const int RED_LED = 21;    // RED LED Pin
const int GREEN_LED = 22;  // GREEN LED Pin
const int BLUE_LED = 23;   // BLUE LED Pin

const int MTR_CTRL_1 = 17;  // Motor Control Pin 1
const int MTR_CTRL_2 = 18;  // Motor Control Pin 2

const int SPI_CS_PIN = 15; // SPI Chip Select (CS) pin for MCP2515

hw_timer_t *GREEN_LED_TIMER = NULL;  // Hardware Timer declaration

const int numSamples = 10;                 // Number of samples to average
const unsigned long sampleInterval = 100;  // Time interval between samples in milliseconds
unsigned long lastSampleTime = 0;          // Last time a sample was taken
float totalCurrent = 0.0;                  // Accumulated total current from samples

int FSR_1_Val = 0;// Force Sensor 1 Value
int FSR_2_Val = 0;// Force Sensor 2 Value

struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(SPI_CS_PIN);


/******************************************************
Function declarations
******************************************************/

float analogToCurrent(int sensorValue);
float readCurrent();
float analogToForce(int sensorValue);
float readForce(int sensorNumber);
void motorForward();
void motorBackward();

/******************************************************
Interrupts
******************************************************/
void IRAM_ATTR timerInterrupt() 
{
  static bool ledState = false;
  digitalWrite(GREEN_LED, ledState);
  ledState = !ledState; 
}

void setup() 
{
  Serial.begin(115200);
  // LED Indicators
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Motor Control
  pinMode(MTR_CTRL_1, OUTPUT);
  pinMode(MTR_CTRL_2, OUTPUT);

  // ADC Pins

  // Interrupt Control for POWER ON LED
  GREEN_LED_TIMER = timerBegin(0, 80, true);  // Timer 0, prescaler of 80 (1 tick = 1 microsecond)
  timerAttachInterrupt(GREEN_LED_TIMER, &timerInterrupt, true); // Attach the interrupt handler
  timerAlarmWrite(GREEN_LED_TIMER, 1000000, true); // 1 second interval
  timerAlarmEnable(GREEN_LED_TIMER); // Enable the timer

  // CAN Stuff
  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x8E;
  canMsg1.data[1] = 0x87;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;

  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x0E;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x08;
  canMsg2.data[4] = 0x01;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0xA0;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}


void loop() 
{
//testing force sensors:
  Serial.print("Force Sensor 1 ADC: ");
  FSR_1_Val = analogRead(FSR_1);
  Serial.println(FSR_1_Val);
  Serial.print("Force Sensor 2 ADC: ");
  FSR_2_Val = analogRead(FSR_2);
  Serial.println(FSR_2_Val);


  totalCurrent = readCurrent();
  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);

  Serial.println("CAN Messages sent");
  delay(1000);
}

// This function converts the analog reading from the current sensor to a current
float analogToCurrent(int sensorValue) 
{

}

// This function reads the current from the current sensor and prints them to Serial
float readCurrent()
{
  // Read analog value from ADC1_CH0
  int sensorValue = analogRead(CURRENT_SENSE);

  // You can convert the analog value to voltage or current using calibration
  // For example, if you have a voltage divider with known resistances, you can calculate the voltage.
  // Modify the following formula based on your voltage divider's resistances.
  //float voltage = sensorValue * (3.3 / 4095.0); // 3.3V is the ESP32's reference voltage
  float current = 1;

  // Print the result
  Serial.print("ADC Value: ");
  Serial.print(sensorValue);
  // Serial.print(", Current ");
  // Serial.print(current, 3); // Print with 3 decimal places
  // Serial.print(", Voltage: ");
  // Serial.print(voltage, 3); // Print with 3 decimal places
  // Serial.println("V");

  return current;
}

// This function converts the analog reading from the force sensor to a force reading
float analogToForce(int sensorValue, int sensorNumber)
{


}

// This function reads the force reading from the force sensor and prints them to Serial
float readForce(int sensorNumber)
{
//if(sensorNumber == 1){
  //int sensorValue = analogRead(FSR_1);
//} else {
  //int sensorValue = analogRead(FSR_2);
}



// these functions may be inverted depending on testing with the linear actuator
void motorForward()
{
  digitalWrite(MTR_CTRL_1, HIGH);
  digitalWrite(MTR_CTRL_2, LOW);
}

// these functions may be inverted depending on testing with the linear actuator
void motorBackward()
{
  digitalWrite(MTR_CTRL_1, LOW);
  digitalWrite(MTR_CTRL_2, HIGH);
}

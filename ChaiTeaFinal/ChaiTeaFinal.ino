#include <acr/io.h> //These three libraries are pulled from Daniel Yang's encoder code from 156A. They're for using interrupt.
#include <avr/interrupt.h>
#include <math.h>
#include <i2cmaster.h> // i2c library for temp sensor

//MAE 156B Chai Latte
int stepnum = 0;
enum MODE { START, WATER_DISP, MILK_DISP, END };

MODE currentMode = START;

const int PIN_6 = 6;

// Pump constants
struct PUMP_CONSTANTS
{
  float hallSensor; 
  int NbTopsFan;
};

PUMP_CONSTANTS pump1;
PUMP_CONSTANTS pump2;

volatile int NbTopsFan; //measuring the rising edges of the signal
volatile int NbTopsFan2; //measuring the rising edges of the signal
float Calc;                               
float Volume = 0;
float VolumeRead = 0;
float time;
float dt;
float VolumeFlow;
int hallsensor = 2;    //The pin location of the sensor
int hallsensormilk = 3;    //The pin location of the sensor
const int wateronswitch = 4; // assign pin for pump powerd
const int milkonswitch = 5; // assign pin for pump powerd
int count = 1;

// Cooker constants
const int onoff = 6; // assign pin for powering cooktop on or off
const int watts = 7; //assign pin for adjusting cooktop power
int cookcount = 1; // initialize counter
int watertemp = 0;

// Solid Dispenser Constants
int masalapin = 8; //apparently pins 2 and 3 are the interrupt pins
int switchcount1;
int teapin = 9; //apparently pins 2 and 3 are the interrupt pins
int switchcount2;

void setup() {
  Serial.begin(9600);
  pinMode(hallsensor, INPUT); //initializes digital pin 2 as an input
  attachInterrupt(0, rpm, RISING); //and the interrupt is attached
  pinMode(wateronswitch, OUTPUT); //initialize digital pin 3 as output
  pinMode(milkonswitch, OUTPUT); //initialize digital pin 3 as output
  pinMode(hallsensormilk, INPUT); //initializes digital pin 2 as an input
  attachInterrupt(0, rpm2, RISING); //and the interrupt is attached
  
  pinMode(onoff,OUTPUT); // define pin as output
  pinMode(watts,OUTPUT); // define pin as output
  // i2c_init(); //Initialise the i2c bus
  PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
  
  pinMode(masalapin, INPUT);
  attachInterrupt(1,readSwitch1, CHANGE); //attach interrupt to switch function. Triggers when there is a value change.
  switchcount1 = 0;
  Serial.println("Masala Motor On");
  pinMode(teapin, INPUT);
  attachInterrupt(1,readSwitch2, CHANGE); //attach interrupt to switch function. Triggers when there is a value change.
  switchcount2 = 0;
  Serial.println("Tea Motor On");
}

void loop() {
  if (stepnum ==0)
  {
    // Code with interrupts that determine number of servings
    currentMode = WATER_DISP;
  }
  else if (stepnum == 1)
  {
    // Code for running water pump with flbowmeter response
    if(count == 1){
      // put your main code here, to run repeatedly: 
      digitalWrite(wateronswitch,HIGH);
      while (VolumeRead < 2) {
        VolumeRead = ReadVol();
      }
      delay(200);
      digitalWrite(wateronswitch,LOW);
      count = count++;
      Serial.println(count);
    }
    else {
      delay(1000);
      stepnum = stepnum + 1;
    }
  }
  else if (stepnum == 2)
  {
    // Code for turning on hot plate
    // Boils water
    // Read IR sensor and add to stepnum when 100 Celsius
    if(cookcount == 1){
      Power();
      delay(200);
      PowerUp();
      delay(200);
      while (watertemp < 100) {
        watertemp = temperature();
        delay(100);
      }
      delay(200);
      Power();
      count = count++;
      Serial.println(count);
    }
    else {
      delay(1000);
      stepnum = stepnum + 1;
    }
  }
  else if (stepnum == 3)
  {
    if (switchcount1 < 4){
      delay(500);
    }
    else {
      Serial.println("Motor Off");
      delay(1000);
      stepnum = stepnum + 1;
    }
  }
  else if (stepnum == 4)
  {
    // Code for running milk pump with flbowmeter response
    if(count == 1){
      // put your main code here, to run repeatedly: 
      digitalWrite(milkonswitch,HIGH);
      while (VolumeRead < 2) {
        VolumeRead = ReadVol2();
      }
      delay(200);
      digitalWrite(milkonswitch,LOW);
      count = count++;
      Serial.println(count);
    }
    else {
      delay(1000);
      stepnum = stepnum + 1;
    }
  }
  else if (stepnum == 5)
  {
    // Use IR sensor to control frothing
    // After certain amount of time at 100 celsius turn off hot plate
    // Reset counter
    stepnum = 0;
  }
}

void rpm ()     //This is the function that the interupt calls 
{ 
  NbTopsFan++;  //This function measures the rising and falling edge of the hall effect sensors signal
} 

void rpm2 ()     //This is the function that the interupt calls 
{ 
  NbTopsFan2++;  //This function measures the rising and falling edge of the hall effect sensors signal
} 

float ReadVol ()    
{
  time = millis();
  NbTopsFan = 0;   //Set NbTops to 0 ready for calculations
  sei();      //Enables interrupts
  delay (500);   //Wait 1 second
  cli();      //Disable interupts
  dt = time - millis();
  Calc = (NbTopsFan * 60 / 73); //(Pulse frequency x 60) / 73, = flow rate in L/hour 
  VolumeFlow = Calc/3600;
  Volume = Volume + VolumeFlow *dt /1000;
  Serial.print (Volume,DEC);//Prints number calculated above in Liters
  Serial.print (" L\r\n"); //Prints "L/hour" and returns a  new line
  return Volume;
}

float ReadVol2 ()    
{
  time = millis();
  NbTopsFan2 = 0;   //Set NbTops to 0 ready for calculations
  sei();      //Enables interrupts
  delay (500);   //Wait 1 second
  cli();      //Disable interupts
  dt = time - millis();
  Calc = (NbTopsFan2 * 60 / 73); //(Pulse frequency x 60) / 73, = flow rate in L/hour 
  VolumeFlow = Calc/3600;
  Volume = Volume + VolumeFlow *dt /1000;
  Serial.print (Volume,DEC);//Prints number calculated above in Liters
  Serial.print (" L\r\n"); //Prints "L/hour" and returns a  new line
  return Volume;
}


void Power() {
  digitalWrite(onoff,HIGH); //Turns on
  delay(200);
  digitalWrite(onoff,LOW);
}
  
void PowerUp() {
  
  for (int level = 6; level <= 10; ++level 
  {
       digitalWrite(watts, HIGH); //Power level 6
    delay(200); 
     digitalWrite(watts,LOW);
    delay(500);   
  }
/*
  digitalWrite(watts, HIGH); //Power level 7
  delay(200); 
  digitalWrite(watts,LOW);
  delay(1000);
  digitalWrite(watts, HIGH); //Power level 8
  delay(200); 
  digitalWrite(watts,LOW);
  delay(1000);
  digitalWrite(watts, HIGH); //Power level 9
  delay(200); 
  digitalWrite(watts,LOW);
  delay(1000);
  digitalWrite(watts, HIGH); //Power level 10
  delay(200); 
  digitalWrite(watts,LOW);
  delay(1000);
  */
}

float temperature() {
    int dev = 0x5A<<1;
    int data_low = 0;
    int data_high = 0;
    int pec = 0;
    
    //i2c_start_wait(dev+I2C_WRITE);
    //i2c_write(0x07);
    
    // read
    //i2c_rep_start(dev+I2C_READ);
    //data_low = i2c_readAck(); //Read 1 byte and then send ack
    //data_high = i2c_readAck(); //Read 1 byte and then send ack
    //pec = i2c_readNak();
    //i2c_stop();
    
    //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
    double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
    double tempData = 0x0000; // zero out the data
    int frac; // data past the decimal point
    
    // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
    tempData = (double)(((data_high & 0x007F) << 8) + data_low);
    tempData = (tempData * tempFactor)-0.01;
    
    float celcius = tempData - 273.15;
    Serial.print("Celcius: ");
    Serial.println(celcius);
    return celcius;
}

void readSwitch1() 
{ //function to read the encoder. When interrupt is activated, it reads the input pin and if its value is 1 (i.e. the switch is pressed), adds a count{
  if (digitalRead(masalapin) == 1){
    switchcount1 = switchcount1 + 1;
    Serial.println("INTERRUPT");
  }
}

void readSwitch2() 
{ //function to read the encoder. When interrupt is activated, it reads the input pin and if its value is 1 (i.e. the switch is pressed), adds a count{
  if (digitalRead(teapin) == 1){
    switchcount2 = switchcount2 + 1;
    Serial.println("INTERRUPT");
  }
}

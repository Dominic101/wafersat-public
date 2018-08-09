
/*
GET SDFAT LIBRARY: https://github.com/greiman/SdFat
GET SOFTSPI LIBRARY: https://github.com/MajenkoLibraries/SoftSPI
GET MCP3208 LIBRARY : https://www.arduinolibraries.info/libraries/mcp3208
GET SENSORBAR LIBRARY WITH THE CIRCULAR BUFFER : https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide#installing-the-arduino-library
*/

#include <SoftSPI.h>
#include "SdFat.h"
#include <Mcp3208.h>
#include <sensorbar.h>
#include <SPI.h>
#include <Math.h>
SdFat sd;

<<<<<<< HEAD
const int goal = 35; //goal temperature, can be an int or change to float 
float x_old = 0; // for filter
const float delta_t = 0.5; //how often we'll be taking data and adjusting duty cycle
=======
const int goal = 20; //goal temperature, can be an int or change to float 
float x_old = 0; // initialization of first term for filter
const float delta_t = 0.5; // how often we'll be taking data and adjusting duty cycle
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99
float previous_error = 0; // for the derivative of the error
float numloops,integral,current_DC; // these need to be global scope so i'm declaring them here
CircularBuffer derivative_errors(4); // this buffer will hold four derivative error values
File dataFile; // initialize a file for writing to file
String filename = ""; // initialize file name as string
bool csvnamed = false; // only when this boolean is true, the recording code runs, otherwise it'll wait for user input

const float alpha = 0.00269;
const float beta = 0.0002844;
const float kappa = 7.269*pow(10,-7);
// three constant values above used in TFD()

const float supplyVoltage = 1.024;
const float rl = 10000.0;
const float r25 = 1000.0;

const float Ki = 0.1;
const float Kp = 2.0;
const float Kd = 1.0;
// three constant values for PID control, not yet tuned

/*
 * Setting up the adc and all pins on board, MISO and MOSI for board and SD card shield are on different pins
 */
const int SPI_CS = 4;      // SPI slave select
const float ADC_VREF = 1024;    // 1.024V Vref
const float ADC_CLK = 2000000;  // SPI clock 2.0 MHz
MCP3208 adc(ADC_VREF, SPI_CS);
const int CS_PIN_SD = 10;
const int MUX_A = 5;
const int MUX_B = 6;
//const int BOARD_MISO = 12;
//const int BOARD_MOSI = 11;
//const int BOARD_CLOCK = 13;
SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
//SoftSPI boardSPI(BOARD_MOSI, BOARD_MISO, BOARD_CLOCK);


/*
 * Setting up heater.
 */
const int HEATER_PIN = 7;

/* 
 * Takes unfiltered average temperature signal and apply a low-pass filter through Laplace Transform.
 */
float davefilter(float avg_temp, float alpha = 0.3) {
  float x_new = (1-alpha*delta_t)*x_old+delta_t*pow(alpha,0.5)*avg_temp;
  float o = pow(alpha,0.5)*x_new;
  x_old = x_new;
  return o;
}

/* 
 *  Modified sigmoid function takes any number and maps to a number between 0 and 100.
 */
float sigmoid(float PID_sum) {
  float funct_shift = PID_sum-4.0;
  float sigmoid = exp(funct_shift)/(exp(funct_shift)+1);
  return sigmoid*255.0;
}

/* 
 *  Returns the average of the four derivative values in the buffer.
 */
float get_average_der() {
  return derivative_errors.averageLast(4);
}

/*
 * This updates the buffer with a new derivative error. Does not have to return anything.
 */
void update_derivatives(float new_der) {
  derivative_errors.pushElement(new_der);
}

/*
Data should be a float value from 0 to 1, as read by ADC with Vref at 3.3 V.
Using temp vs. resistance data from manufacturer as reference (data in Wafersat Google Drive), 
apply Steinhart-Hart equation to convert resistance reading to temperature. 
Coefficients alpha, beta, and kappa determined by MATLAB's least square fit. (MATLAB fitting file also available in Wafersat Google Drive)
*/
float TFD(float data) {
    float ratio = data/4096.0; //divide by 12 bits
    float vout = ratio * supplyVoltage;
    float resistance = (rl - ratio*rl)/(ratio);
    float temperature = (1/(alpha+beta*log(resistance/r25)+kappa*pow((log(resistance/r25)),3)))-273.15;
    return round(temperature*100)/100.0;
}


/*
 * This method calculates the PID value, updates previous error, buffer, and current_DC
 * Does not currently return the duty cycle, just updates the value in the global scope.
 */
void PID(float temp, float want) {
  float error = want - temp;
  if(numloops>=50) {
    integral += error*delta_t;
  }
  float d_error = (error - previous_error)/delta_t;
  previous_error = error;
  update_derivatives(d_error);
  float av_dev = 0.0;
  if(numloops>=2) {
    av_dev = get_average_der();
  }
  float PID_sum = (error*Kp)+(integral*Ki)+(av_dev*Kd);
  current_DC = sigmoid(PID_sum);
  analogWrite(HEATER_PIN, current_DC);
}

/*
 * A method that takes a row of data, and saves it in the file in the SD card
 */
void saveData(String data){
  if(!sd.card()->errorCode()){ // check the card is still there
  // now append new data file
    dataFile = sd.open(filename, FILE_WRITE);
    if (dataFile){
      dataFile.println(data);
      dataFile.close(); // close the file
    }
  }
  else{
  Serial.println("Error writing to file !");
  }
}

/*
<<<<<<< HEAD
=======
 * A method that manually reads signals from a channel using soft SPI library
 */
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99
unsigned int ADCRead(byte channel) {
  channel = constrain(channel,0,7);
  byte firstTX = 0b00000110;
  firstTX |= channel >> 2;
  byte secondTX = channel << 6;
  unsigned int receivedData;
  digitalWrite(SPI_CS, LOW);
  boardSPI.transfer(firstTX);
  receivedData = boardSPI.transfer(secondTX);
  receivedData = receivedData << 8;
  receivedData |= boardSPI.transfer(0);
  digitalWrite(SPI_CS, HIGH);
  return receivedData & 0xFFF;
}*/


void setup() {
  // initializing card
  Serial.begin(9600);
  
  Serial.println("Initializing Card...");
  pinMode(CS_PIN_SD, OUTPUT);
  if(!sd.begin(CS_PIN_SD, SD_SCK_MHZ(0.5))) {
    Serial.println("Card Failure.");
    return;
  }
  Serial.println("Card Detected!");
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // configure PIN mode, copied from example MCP3208 code
  pinMode(SPI_CS, OUTPUT);
  //pinMode(BOARD_MISO, INPUT);
  //pinMode(BOARD_MOSI, OUTPUT);
  //pinMode(BOARD_CLOCK, OUTPUT);
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  //boardSPI.begin();
  //boardSPI.setBitOrder(LSBFIRST);
  //boardSPI.setDataMode(SPI_MODE0);
  //boardSPI.setClockDivider(SPI_CLOCK_DIV0);

  // set initial PIN state, copied from example MCP3208 code
  digitalWrite(SPI_CS, HIGH);

  // initialize SPI interface for MCP3208 copied from example MCP3208 code
  SPI.begin();
  SPI.beginTransaction(settings);

  // heater
<<<<<<< HEAD
  pinMode(HEATER_PIN, OUTPUT);
=======
  // pinMode(HEATER_PIN, OUTPUT);
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99
 
  Serial.println("Setup complete.");
  Serial.println("Enter file name, please include '.csv' at the end");
}

void loop() {
  if (csvnamed == false) {
    if (Serial.available() > 0) {
      filename = Serial.readString();
      Serial.print("The file is named to: "); Serial.println(filename);
      csvnamed = true;
    }
  } else {
    float starttime = millis();
<<<<<<< HEAD
    float temp[22]; // this will hold time, 18 temperatures of the RTDs, average, filtered average, duty cycle (so size is 21)
    temp[0] = millis()/1000.0; //time in seconds
=======
    float datarow[21]; // this will hold time, 18 temperatures of the RTDs, average, filtered average, duty cycle (so size is 21)
    datarow[0] = millis()/1000.0; //time in seconds
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99

    SPI.beginTransaction(settings);
    // This should read the 18 temperatures and record them.
    digitalWrite(MUX_A, LOW);
    digitalWrite(MUX_B, HIGH);
    delay(200); //delay before setting mux
<<<<<<< HEAD
    
    temp[1] = TFD(adc.read(MCP3208::SINGLE_0));
    delay(50);
    temp[2] = TFD(adc.read(MCP3208::SINGLE_1));
    delay(50);
    temp[3] = TFD(adc.read(MCP3208::SINGLE_2));
    delay(50);
    temp[4] = TFD(adc.read(MCP3208::SINGLE_3));
    delay(50);
    temp[5] = TFD(adc.read(MCP3208::SINGLE_4));
    delay(50);
    temp[6] = TFD(adc.read(MCP3208::SINGLE_5));
    delay(50);
    
=======
    for(int i = 0; i<6; i++) {
      datarow[i+1] = TFD(ADCRead(i));
      delay(50);
    }
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99
    
    
    digitalWrite(MUX_A, HIGH);
    digitalWrite(MUX_B, LOW);
    delay(200); //delay before setting mux
<<<<<<< HEAD
    temp[7] = TFD(adc.read(MCP3208::SINGLE_0));
    delay(50);
    temp[8] = TFD(adc.read(MCP3208::SINGLE_1));
    delay(50);
    temp[9] = TFD(adc.read(MCP3208::SINGLE_2));
    delay(50);
    temp[10] = TFD(adc.read(MCP3208::SINGLE_3));
    delay(50);
    temp[11] = TFD(adc.read(MCP3208::SINGLE_4));
    delay(50);
    temp[12] = TFD(adc.read(MCP3208::SINGLE_5));
    delay(50);
=======
    for(int i = 0; i<6; i++) {
      datarow[i+7] = TFD(ADCRead(i));
      delay(50);
    }
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99
    
    
    digitalWrite(MUX_A, HIGH);
    digitalWrite(MUX_B, HIGH);
    delay(200); //delay before setting mux
<<<<<<< HEAD
    temp[13] = TFD(adc.read(MCP3208::SINGLE_0));
    delay(50);
    temp[14] = TFD(adc.read(MCP3208::SINGLE_1));
    delay(50);
    temp[15] = TFD(adc.read(MCP3208::SINGLE_2));
    delay(50);
    temp[16] = TFD(adc.read(MCP3208::SINGLE_3));
    delay(50);
    temp[17] = TFD(adc.read(MCP3208::SINGLE_4));
    delay(50);
    temp[18] = TFD(adc.read(MCP3208::SINGLE_5));
    delay(50);
=======
    for(int i = 0; i<6; i++) {
      datarow[i+13] = TFD(ADCRead(i));
      delay(50);
    }
>>>>>>> c4b165e57602330043de48aee2b54bc42940fc99
    
  
    // Calculate the average temperature of the board
    float sum = 0;
    for(int i = 1; i <= 18; i++) {
      sum += datarow[i];
    }
    
    float avg_temp = sum/18.0;
    datarow[19] = avg_temp;
    
    //this runs once for the filter
    if(numloops == 0) {
      x_old = avg_temp;
    }
  
   
    float filtered_temp = davefilter(avg_temp); //apply filter
    datarow[20] = filtered_temp;

  
    // copying old python PID, we can change how this works
    // also note that PID no longer returns duty cycle
    if(numloops < 25) {
      if(abs(filtered_temp-avg_temp)<0.5) {
        PID(filtered_temp, goal);
      }
      else {
        PID(avg_temp,goal);
      }
    }
    else {
      if(abs(filtered_temp-avg_temp)<1.5) {
        PID(filtered_temp, goal);
      }
      else {
        PID(avg_temp,goal);
      }
    }
    datarow[21] = current_DC;

  
    //building and saving the data string from temp
    String build;
    for(int i = 0; i < sizeof(datarow)/sizeof(datarow[0]); i++) {
      build += String(datarow[i]) + ",";
    }
    Serial.println(build);
    
    saveData(build);
    
    numloops += delta_t; 
    delay(500); //sleep time
  }
}

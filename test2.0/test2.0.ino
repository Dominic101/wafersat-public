#include <SD.h>

// GET MCP3208 LIBRARY : https://www.arduinolibraries.info/libraries/mcp3208
// GET SENSORBAR LIBRARY WITH THE CIRCULAR BUFFER : https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide#installing-the-arduino-library

#include <Mcp3208.h>
#include <sensorbar.h>
#include <SPI.h>
#include <SD.h>
#include <Math.h>


const int goal = 20; //goal temperature, can be an int or change to float 
float x_old = 0; // for filter
const float delta_t = 0.5; //how often we'll be taking data and adjusting duty cycle
float previous_error = 0; // for the derivative of the error
float a,integral,current_DC; //these need to be global scope so i'm declaring them here
CircularBuffer derivative_errors(4); //this buffer will hold four derivative error values
File dataFile; // for writing to a file, not implemented yet
String filename = "";
bool csvnamed = false; 

const float alpha = 0.00269;
const float beta = 0.0002844;
const float kappa = 7.269*pow(10,-7);
const float supplyVoltage = 1.024;
const float rl = 10000.0;
const float r25 = 1000.0;

const float Ki = 0.1;
const float Kp = 2.0;
const float Kd = 1.0;

/*
 * Setting up the adc, check with Ishaan about the pin number, will not necessarily be 2
 */
const int SPI_CS = 10;      // SPI slave select
const float ADC_VREF = 1024;    // 1.024V Vref
const float ADC_CLK = 500000;  // SPI clock 0.5 MHz
MCP3208 adc(ADC_VREF, SPI_CS);
const int CS_PIN_SD = 2;
const int MUX_A = 3;
const int MUX_B = 4;

/*
 * Setting up heater.
 */
//const int HEATER_PIN = 12;

/* 
 * Takes unfiltered average temperature and applies a low-pass filter.
 * This gain, a, is different from the a that is the iterator.
 */
float davefilter(float avg_temp, float a = 0.3) {
  float x_new = (1-a*delta_t)*x_old+delta_t*pow(a,0.5)*avg_temp;
  float o = pow(a,0.5)*x_new;
  x_old = x_new;
  return o;
}

/* 
 *  Modified sigmoid function takes any number and maps to a number between
 *  0 and 100.
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
For converting single data points into temperature readings.
Data should be a float value from 0 to 1, as read by ADC with Vref at 3.3 V
*/
float TFD(float data) {
    Serial.println(data);
    float ratio = data/4096.0; //divide by 12 bits
    float vout = ratio * supplyVoltage;
    float resistance = (ratio*rl)/(1-ratio);
    float temperature = (1/(alpha+beta*log(resistance/r25)+kappa*pow((log(resistance/r25)),3)))-273.15;
    return round(temperature*100)/100.0;
}


/*
 * This method calculates the PID value, updates previous error,buffer, and current_DC
 * Does not currently return the duty cycle, just updates the value in the global scope.
 */
void PID(float temp, float want) {
  float error = want - temp;
  if(a>=50) {
    integral += error*delta_t;
  }
  float d_error = (error - previous_error)/delta_t;
  previous_error = error;
  update_derivatives(d_error);
  float av_dev = 0.0;
  if(a>=2) {
    av_dev = get_average_der();
  }
  float PID_sum = (error*Kp)+(integral*Ki)+(av_dev*Kd);
  current_DC = sigmoid(PID_sum);
//  analogWrite(HEATER_PIN, current_DC);
}

void saveData(String data){
  if(SD.exists(filename)){ // check the card is still there
  // now append new data file
    dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile){
      dataFile.println(data);
      dataFile.close(); // close the file
    }
  }
  else{
  Serial.println("Error writing to file !");
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  Serial.println("Initializing Card...");
  pinMode(CS_PIN_SD, OUTPUT);
  if(!SD.begin(CS_PIN_SD, SPI_HALF_SPEED)) {
    Serial.println("Card Failure.");
    return;
  }
  Serial.println("Card Detected!");
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // configure PIN mode, copied from example MCP3208 code (do we need this?)
  pinMode(SPI_CS, OUTPUT);
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);

  // set initial PIN state, copied from example MCP3208 code (do we need this?)
  digitalWrite(SPI_CS, HIGH);

  // initialize SPI interface for MCP3208 copied from example MCP3208 code (do we need this?)
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  // heater
//  pinMode(HEATER_PIN, OUTPUT);
 
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
    float temp[21]; // this will hold time, 18 temperatures of the RTDs, average, filtered average, duty cycle (so size is 21)
    temp[0] = millis()/1000.0; //time in seconds
    
    // This should read the 18 temperatures and record them.
    digitalWrite(MUX_A, LOW);
    digitalWrite(MUX_B, HIGH);
    delay(200); //delay before setting mux
    
    temp[1] = TFD(adc.read(MCP3208::SINGLE_0));
    delay(10);
    temp[2] = TFD(adc.read(MCP3208::SINGLE_1));
    delay(10);
    temp[3] = TFD(adc.read(MCP3208::SINGLE_2));
    delay(10);
    temp[4] = TFD(adc.read(MCP3208::SINGLE_3));
    delay(10);
    temp[5] = TFD(adc.read(MCP3208::SINGLE_4));
    delay(10);
    temp[6] = TFD(adc.read(MCP3208::SINGLE_5));
    
    

    digitalWrite(MUX_A, HIGH);
    digitalWrite(MUX_B, LOW);
    delay(200); //delay before setting mux
    temp[7] = TFD(adc.read(MCP3208::SINGLE_0));
    delay(10);
    temp[8] = TFD(adc.read(MCP3208::SINGLE_1));
    delay(10);
    temp[9] = TFD(adc.read(MCP3208::SINGLE_2));
    delay(10);
    temp[10] = TFD(adc.read(MCP3208::SINGLE_3));
    delay(10);
    temp[11] = TFD(adc.read(MCP3208::SINGLE_4));
    delay(10);
    temp[12] = TFD(adc.read(MCP3208::SINGLE_5));
    

    digitalWrite(MUX_A, HIGH);
    digitalWrite(MUX_B, HIGH);
    delay(200); //delay before setting mux
    temp[13] = TFD(adc.read(MCP3208::SINGLE_0));
    delay(10);
    temp[14] = TFD(adc.read(MCP3208::SINGLE_1));
    delay(10);
    temp[15] = TFD(adc.read(MCP3208::SINGLE_2));
    delay(10);
    temp[16] = TFD(adc.read(MCP3208::SINGLE_3));
    delay(10);
    temp[17] = TFD(adc.read(MCP3208::SINGLE_4));
    delay(10);
    temp[18] = TFD(adc.read(MCP3208::SINGLE_5));
    
  
    // Calculate the average temperature of the board
    float sum = 0;
    for(int i = 1; i <= 18; i++) {
      sum += temp[i];
    }
    
    float avg_temp = sum/18.0;
    temp[19] = avg_temp;
    
    //this runs once for the filter
    if(a==0) {
      x_old = avg_temp;
    }
  
   
    float filtered_temp = davefilter(avg_temp); //apply filter
    temp[20] = filtered_temp;

  
    // copying old python PID, we can change how this works
    // also note that PID no longer returns duty cycle
    if(a<25) {
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
    temp[21] = current_DC;

  
    //building and saving the data string from temp
    String build;
    for(int i = 0; i < sizeof(temp)/sizeof(temp[0]); i++) {
      build += String(temp[i]) + ",";
    }
    Serial.println(build);
    //saveData(build);
    
    a += delta_t; //we're still doing this I guess
    delay(500); //sleep time
  }
}

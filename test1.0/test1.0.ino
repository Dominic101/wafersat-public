// GET MCP3208 LIBRARY : https://www.arduinolibraries.info/libraries/mcp3208
// GET SENSORBAR LIBRARY WITH THE CIRCULAR BUFFER : https://learn.sparkfun.com/tutorials/sparkfun-line-follower-array-hookup-guide#installing-the-arduino-library

#include <Mcp3208.h>
#include <sensorbar.h>
#include <SPI.h>
#include <SD.h>
#include <Math.h>

int RTD_list = 8; // This is not really a list anymore. Loop through int 0-7 to access the channel from adc.
int goal = 20; //goal temperature, can be an int or change to float 
float x_old = 0; // for filter
float delta_t = 0.5; //how often we'll be taking data and adjusting duty cycle
float previous_error = 0; // for the derivative of the error
float a,integral,current_DC; //these need to be global scope so i'm declaring them here
CircularBuffer derivative_errors(4); //this buffer will hold four derivative error values
File dataFile; // for writing to a file, not implemented yet

/*
 * Setting up the adc, check with Ishaan about the pin number, will not necessarily be 2
 */
#define SPI_CS      2        // SPI slave select
#define ADC_VREF    3300     // 3.3V Vref
#define ADC_CLK     1000000  // SPI clock 1.0MHz
MCP3208 adc(ADC_VREF, SPI_CS);
const int CS_PIN = 10;

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
  return sigmoid*100.0;
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
    float a = 2.725076799546500*pow(10.0,-12.0);
    float b = -1.231253679636238*pow(10.0,-8.0);
    float c = 3.046224786805958*pow(10.0,-5.0);
    float d = 0.221027985508455;
    float e = -241.9045208388455;
    float V;
    float R;

    if( data != 1 ) {
        V = data*3.3;
        R = (2700*V)/(3.3-V); //Based on voltage divider circuit with 2.7 k resistor
    //in series with RTD powered on 3.3 V,
  }

    else { // #in case of dividing by 0
        R = 100000;  //arbitrary value for extremely high R (most likely
        //due to error)
  }       
    float T = a*pow(R,4.0) + b*pow(R,3.0) + c*pow(R,2.0) + d*R + e;
    return round(T*100)/100.0;
}

/*
 * Returns current temperature of an RTD
 * RTD should be int range 0 to 7 corresponding to RTD channels
 */
float get_temp(int RTD) {
  //gets ADC reading from selected RTD, a float range 0-1 reading = float(adc.read(RTD)/4096)
  //reading = RTD_list[RTD].value; CHANGE TO THIS
  float reading = RTD; 
  float temp = TFD(reading);
    
  return temp;
}

/*
 * Returns current voltage drop over an RTD, assuming Vref connected to 3.3 V
 * RTD should be int range 0 to 7 corresponding to RTD channels
*/
float get_voltage(int RTD) {
//reading = RTD_list[RTD].value; CHANGE TO THIS
  float reading = RTD;
  float voltage = reading*3.3;
  return voltage;
}

/*
 * This method calculates the PID value, updates previous error,buffer, and current_DC
 * Does not currently return the duty cycle, just updates the value in the global scope.
 */
void PID(float temp, float want, float Ki = 0.1, float Kp = 2.0, float Kd = 1.0) {
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
  //heater.ChangeDutyCycle(current_DC); I DON'T KNOW HOW TO DO THIS YET!! 
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Initializing Card");
  pinMode(CS_PIN, OUTPUT);
  if(!SD.begin(CS_PIN)) {
    Serial.println("Card Failure");
    return;
  }
  Serial.println("Card Ready");
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // configure PIN mode, copied from example MCP3208 code (do we need this?)
  pinMode(SPI_CS, OUTPUT);

  // set initial PIN state, copied from example MCP3208 code (do we need this?)
  digitalWrite(SPI_CS, HIGH);

  // initialize SPI interface for MCP3208 copied from example MCP3208 code (do we need this?)
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  //Here I was testing opening a new file
  String filename = "Test0001.csv";
  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.close();
  
  Serial.println("Setup complete");
}

void loop() {
  float temp[12]; // this will hold time, 8 temperatures of the RTDs, average, filtered average, duty cycle (so size is 12)
  temp[0] = millis()/1000.0; //time in seconds..?

  // This should read the 8 temperatures and record them.
  for(int i = 0; i<RTD_list; i++) {
    temp[i] = TFD(adc.read(i));
  }

  // Calculate the average temperature of the board
  float sum = 0;
  for(int i = 1; i < sizeof(temp); i++) {
    sum += temp[i];
  }
  float avg_temp = sum/8.0; //8? 7? 6 RTDs total?
  temp[9] = avg_temp;
  
  //this runs once for the filter
  if(a==0) {
    x_old = avg_temp;
  }

 
  float filtered_temp = davefilter(avg_temp); //apply filter
  temp[10] = filtered_temp;

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
  temp[11] = current_DC;
  
  //Here we should save temp to file somehow

  a += delta_t; //we're still doing this I guess
  delay(delta_t*1000); //sleep time
}

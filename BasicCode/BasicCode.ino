
#include <ECE3.h>

// LEDS
int LED_BUILTIN = 57;
int LED_BUILTIN_2 = 58;

// Pins for wheel motors
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM

const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 41;


uint16_t sensorValues[8];


int leftBaseSpd = 42; // base
int rightBaseSpd = 42; // base
int newSpeedL = 0;
int newSpeedR = 0;


///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);

  pinMode(left_dir_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();
  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 

  // leds
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN_2, OUTPUT);
  
}

// Function definition
int calc8421(int x1, int x2, int x3, int x4, int x5, int x6, int x7, int x8){
  int result;
  int w[4] = {8, 4, 2, 1};
  // result = (-8 * x1 - 4 * x2 - 2 * x3 - x4 + x5 + 2 * x6 + 4 * x7 + 8 * x8) / 4;
  result = (-1 * w[0] * x1 - w[1] * x2 - w[2] * x3 - w[3] * x4 
               + w[3] * x5 + w[2] * x6 + w[1] * x7 + w[0] * x8) / 4;
  return result;
}

int summed_values[8] = {0};
int normalized_values[8] = {0};
int max[8] = {2500, 2500, 2500, 2369, 2440, 2500, 2415}; // Starting values
int min[8] = {791, 664, 711, 641, 757, 741, 804};
int error[2] = {0,0};

void loop() {
  // Read raw sensor values
  ECE3_read_IR(sensorValues);

  // check max and min values
  for (unsigned char i = 0; i < 8; i++)
  {
    // redo max and min
    if (sensorValues[i]>max[i]){
      max[i] = sensorValues[i];
    }
    if (sensorValues[i]<min[i]){
      min[i] = sensorValues[i];
    }
    int value = sensorValues[i];
    // Serial.print(i);
    // Serial.print(", ");
    // Serial.print(value);
    // Serial.print(" : "); // tab to format the raw data into columns in the Serial monitor

    // normalized
    normalized_values[i] = ((value * 1000) - (min[i] * 1000)) / (max[i] - min[i]) ; // multiply by 1000 first to avoid int rounding
  }

  // }
  // Serial.println();
  int calc = calc8421(normalized_values[0], 
                      normalized_values[1],
                      normalized_values[2],
                      normalized_values[3],
                      normalized_values[4],
                      normalized_values[5],
                      normalized_values[6],
                      normalized_values[7]);
  // Serial.print("calc ");
  // Serial.print(calc);
  // Serial.println();
  // Increment current position by the increment value
  // current_position += increment_position;

//////////////////////////////////

  // add error to error array and shift
  // error[0] = error[1];
  // error[1] = calc;

  float k_p = 0.05;
  int p = calc * k_p;

  // float k_c = 0.05;
  // int delta_error = error[1] - error[0];
  // int c = delta_error * k_c;

///////////////////////
  // Serial.print("     c ");
  // Serial.print(c);
  // Serial.print("     delta_error ");
  // Serial.print(delta_error);

  // Serial.print("     error[0] ");
  // Serial.print(error[0]);

  // Serial.print("     error[1] ");
  // Serial.print(error[1]);
  //   Serial.println();

  // Serial.println();
  // calc new speed (p)

  newSpeedL = leftBaseSpd - p;
  newSpeedR = rightBaseSpd + p;

  analogWrite(left_pwm_pin,newSpeedL);
  analogWrite(right_pwm_pin,newSpeedR);
}


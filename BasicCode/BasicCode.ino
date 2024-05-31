
#include <ECE3.h>

// LEDS
const int LED_BUILTIN = 57;
const int LED_BUILTIN_2 = 58;

// Pins for wheel motors
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM

const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

const int LED_RF = 41;


uint16_t sensorValues[8] = {0};


int leftBaseSpd = 30; // base
int rightBaseSpd = 30; // base
int newSpeedL = 0;
int newSpeedR = 0;

int crossSpeedFlag = 0;
int spinState = 0;
int spinTicks = 0;
int runs = 0;
int startTicks = 0;

int binary_values[8] = {0};
int normalized_values[8] = {0};
// int max[8] = {2500, 2500, 2500, 2369, 2440, 2500, 2415}; // Starting values
int max[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500}; // Starting values
// int min[8] = {791, 664, 711, 641, 757, 741, 804};
int min[8] = {791, 664, 711, 641, 757, 741, 741};
// int min[8] = {591, 464, 511, 441, 557, 441, 604};
int error[2] = {0,0};

// Threshold for detecting black
const int threshold = 800;

// Threshold for significant change in sensor values
const int changeThreshold = 100;


///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  crossSpeedFlag = 0;
  spinState = 0;
  spinTicks = 0;
  runs = 0;

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);

  pinMode(left_dir_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN_2, OUTPUT);
  pinMode(LED_RF, OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN_2, LOW);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_RF, LOW);

  // pinMode(LED_RF, OUTPUT);

  ECE3_Init();
  // set the data rate in bits/second for serial data transmission
  
  // Serial.begin(9600); 

  delay(2000); //Wait 2 seconds before starting 
  
}

// Function definition
int calc8421(int x1, int x2, int x3, int x4, int x5, int x6, int x7, int x8){
  int result;
  int w[4] = {1, 2, 3, 4};
  // if (startTicks < 200) {
  //     w[0] = 4;
  //     w[1] = 3;
  //     w[2] = 2;
  //     w[3] = 1;
  // }
  // result = (-8 * x1 - 4 * x2 - 2 * x3 - x4 + x5 + 2 * x6 + 4 * x7 + 8 * x8) / 4;
  result = (-1 * w[0] * x1 - w[1] * x2 - w[2] * x3 - w[3] * x4 
               + w[3] * x5 + w[2] * x6 + w[1] * x7 + w[0] * x8) / 4;
  return result;
}

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


    // Serial.println();


  if (sensorValues[0] > 2490 && 
    sensorValues[1] > 2490 && 
    sensorValues[2] > 2490 && 
    sensorValues[3] > 2490 && 
    sensorValues[4] > 2490 && 
    sensorValues[5] > 2490 && 
    sensorValues[6] > 2490 && 
    sensorValues[7] > 2490){
      
    if (crossSpeedFlag == 0){
      crossSpeedFlag = 1;
    }else if (crossSpeedFlag == 1){
      crossSpeedFlag = 2;
    }
  }else{
    if (crossSpeedFlag == 1){
      crossSpeedFlag = 0;
    }
  }

  if (crossSpeedFlag == 2){
    if (spinState == 0){
      spinState = 1;
      digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
      digitalWrite(LED_BUILTIN_2, HIGH);  // turn the LED on (HIGH is the voltage level)
      digitalWrite(LED_RF, HIGH);
      digitalWrite(left_dir_pin,HIGH);  // change directions
      digitalWrite(left_nslp_pin,HIGH);
      digitalWrite(right_dir_pin,LOW);  // change directions
      digitalWrite(right_nslp_pin,HIGH);
    }
    // Serial.print("spinState: ");
    // Serial.print(spinState);
    // Serial.print("         crossSpeedFlag: ");
    // Serial.print(crossSpeedFlag);
    // Serial.println();

    // else if (spinState = 0){
    //   spinState = 1;
    // }
  }

  
  // if (sensorValues[0] > 2400 && 
  //   sensorValues[7] > 2400){
  //   normalized_values[0] = 0;
  //   normalized_values[1] = 0;
  //   normalized_values[2] = 0;
  //   // normalized_values[3] = 0;
  //   // normalized_values[4] = 0;
  //   normalized_values[5] = 0;
  //   normalized_values[6] = 0;
  //   normalized_values[7] = 0;
  //   digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  //   digitalWrite(LED_BUILTIN_2, HIGH);  // turn the LED on (HIGH is the voltage level)
  // }
  // Serial.println();
  int calc = calc8421(normalized_values[0], 
                      normalized_values[1],
                      normalized_values[2],
                      normalized_values[3],
                      normalized_values[4],
                      normalized_values[5],
                      normalized_values[6],
                      normalized_values[7]
                      );
  // Serial.print("calc ");
  // Serial.print(calc);
  // Serial.println();
  // Increment current position by the increment value
  // current_position += increment_position;

//////////////////////////////////

  // add error to error array and shift
  error[0] = error[1];
  error[1] = calc;

  float k_p = 0.04; // 0.04  // with 25 speed 0.03
  if (startTicks < 150){
    k_p = 0.02;
  }
  int p = calc * k_p;

  float k_d = 0.25; // 0.3   // with 25 speed 0.2
  if (startTicks < 150){
    k_d = 0.1;
  }

  int delta_error = error[1] - error[0];
  int d = delta_error * k_d;

///////////////////////
  // Serial.print("     d ");
  // Serial.print(d);
  // Serial.print("     delta_error ");
  // Serial.print(delta_error);

  // Serial.print("     error[0] ");
  // Serial.print(error[0]);

  // Serial.print("     error[1] ");
  // Serial.print(error[1]);

  // Serial.println();
  // calc new speed (p)

  newSpeedL = leftBaseSpd - (p + d);
  newSpeedR = rightBaseSpd + (p + d);



  if (newSpeedL < 0){
    newSpeedL = 0;
  }
  if (newSpeedL > 255){
    newSpeedL = 255;
  }
  if (newSpeedR < 0){
    newSpeedR = 0;
  }
  if (newSpeedR > 255){
    newSpeedR = 255;
  }
  // Serial.print("     newSpeedL ");
  // Serial.print(newSpeedL);
  // Serial.print("     newSpeedR ");
  // Serial.print(newSpeedR);
  // Serial.println();

  if (spinState == 1){
    // digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    // digitalWrite(LED_BUILTIN_2, HIGH);  // turn the LED on (HIGH is the voltage level)
    // digitalWrite(LED_RF, HIGH);
    // digitalWrite(left_dir_pin,HIGH);
    // digitalWrite(left_nslp_pin,HIGH);
    // digitalWrite(right_dir_pin,LOW);
    // digitalWrite(right_nslp_pin,HIGH);

    analogWrite(left_pwm_pin,233);  // 120   80 ticks
    analogWrite(right_pwm_pin,233);
    spinTicks = spinTicks + 1;
    if (spinTicks > 115){
      if (runs == 1){
        delay(250);  
        spinTicks = 0;
        crossSpeedFlag = 0;
        spinState = 3;
        runs = 2;
        analogWrite(left_pwm_pin,0);
        analogWrite(right_pwm_pin,0);
        while(true){
          runLightShow();
        }
      }else if (runs == 0){
        analogWrite(left_pwm_pin,0);
        analogWrite(right_pwm_pin,0);
        runLightShow();
        spinTicks = 0;
        crossSpeedFlag = 0;
        spinState = 0;
        runs = 1;
        digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(LED_BUILTIN_2, LOW);  // turn the LED on (HIGH is the voltage level)
        digitalWrite(LED_RF, LOW);

        digitalWrite(left_dir_pin,LOW);
        digitalWrite(left_nslp_pin,HIGH);
        digitalWrite(right_dir_pin,LOW);
        digitalWrite(right_nslp_pin,HIGH);

      }
    }
  } else if (spinState == 0){

    // digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
    // digitalWrite(LED_BUILTIN_2, LOW);  // turn the LED on (HIGH is the voltage level)
    // digitalWrite(LED_RF, LOW);

    // digitalWrite(left_dir_pin,LOW);
    // digitalWrite(left_nslp_pin,HIGH);
    // digitalWrite(right_dir_pin,LOW);
    // digitalWrite(right_nslp_pin,HIGH);
    if (startTicks < 80){
      analogWrite(left_pwm_pin,15);
      analogWrite(right_pwm_pin,15);
      
    }else{
      analogWrite(left_pwm_pin,newSpeedL);
      analogWrite(right_pwm_pin,newSpeedR);
    }
    startTicks = startTicks + 1;
  }
  // else if (spinState == 3){
  //   analogWrite(left_pwm_pin,0);
  //   analogWrite(right_pwm_pin,0);
  //   while(true){
  //     runLightShow();
  //   }
  // }
}

void runLightShow() {
  // Turn LEDs on and off in a pattern

  // Turn all LEDs on
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN_2, HIGH);
  digitalWrite(LED_RF, HIGH);
  delay(125);  // Wait for 125 milliseconds

  // Turn all LEDs off
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN_2, LOW);
  digitalWrite(LED_RF, LOW);
  delay(125);  // Wait for 125 milliseconds

  // Turn LEDs on in a different pattern
  digitalWrite(LED_BUILTIN, HIGH);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_BUILTIN_2, HIGH);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_RF, HIGH);
  delay(62);  // Wait for 62 milliseconds

  // Turn LEDs off in reverse order
  digitalWrite(LED_RF, LOW);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_BUILTIN_2, LOW);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_BUILTIN, LOW);
  delay(62);  // Wait for 62 milliseconds

  // Add more patterns as needed
  
  // Turn all LEDs on
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN_2, HIGH);
  digitalWrite(LED_RF, HIGH);
  delay(125);  // Wait for 125 milliseconds

  // Turn all LEDs off
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN_2, LOW);
  digitalWrite(LED_RF, LOW);
  delay(125);  // Wait for 125 milliseconds

  // Turn LEDs on in a different pattern
  digitalWrite(LED_BUILTIN, HIGH);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_BUILTIN_2, HIGH);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_RF, HIGH);
  delay(62);  // Wait for 62 milliseconds

  // Turn LEDs off in reverse order
  digitalWrite(LED_RF, LOW);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_BUILTIN_2, LOW);
  delay(62);  // Wait for 62 milliseconds
  digitalWrite(LED_BUILTIN, LOW);
  delay(62);  // Wait for 62 milliseconds
}

// void  ChangeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
// /*  
//  *   This function changes the car speed gradually (in about 30 ms) from initial
//  *   speed to final speed. This non-instantaneous speed change reduces the load 
//  *   on the plastic geartrain, and reduces the failure rate of the motors. 
//  */
//   int diffLeft  = finalLeftSpd-initialLeftSpd;
//   int diffRight = finalRightSpd-initialRightSpd;
//   int stepIncrement = 20;
//   int numStepsLeft  = abs(diffLeft)/stepIncrement;
//   int numStepsRight = abs(diffRight)/stepIncrement;
//   int numSteps = max(numStepsLeft,numStepsRight);
  
//   int pwmLeftVal = initialLeftSpd;        // initialize left wheel speed 
//   int pwmRightVal = initialRightSpd;      // initialize right wheel speed 
//   int deltaLeft = (diffLeft)/numSteps;    // left in(de)crement
//   int deltaRight = (diffRight)/numSteps;  // right in(de)crement

//   for(int k=0;k<numSteps;k++) {
//     pwmLeftVal = pwmLeftVal + deltaLeft;
//     pwmRightVal = pwmRightVal + deltaRight;
//     analogWrite(left_pwm_pin,pwmLeftVal);    
//     analogWrite(right_pwm_pin,pwmRightVal); 
//     delay(30);   
//   } // end for int k
// //  if(finalLeftSpd  == 0) analogWrite(left_pwm_pin,0); ;
// //  if(finalRightSpd == 0) analogWrite(right_pwm_pin,0);
//   analogWrite(left_pwm_pin,finalLeftSpd);  
//   analogWrite(right_pwm_pin,finalRightSpd);  
// } // end void  ChangeWheelSpeeds


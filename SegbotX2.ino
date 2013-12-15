// Modified Code for the Seg-bot
// by JD Warren
// Arduino Uno (tested)
// Sparkfun SEN-11072 IMU - using X axis from accelerometer and Y axis from gyroscope 4.5x
// Steering potentiometer used to steer bot
// Gain potentiometer used to set max speed (sensitivity)
// Engage switch (button) used to enable motors
//
// By loading this code, you are taking full responsibility for what you may do it!
// Use at your own risk!!!
// If you are concerned with the safety of this project, it may not be for you.
// Test thoroughly with wheels off the ground before attempting to ride - Wear a helmet!

// Modifierad 2013-09-12

// Name Digital I/O pins
int ready_led = 8; // Indikering att segboten är aktiverad, internkopplad
int cal_led = 7; // Indikering att kalibreringen är klar, internkopplad
int batt_led_2 = 4; // Batteriindikering 23V, plint 3
int batt_led_1 = 12; // Batteriindikering 24V, plint 4
int foot_switch = 2; // Fotomkopplare, plint 5

// Name Analog input pins
int gyro_pin = A2; // connect the gyro Y axis (4.5x output) to Analog input 2
int accel_pin = A0; // connect the accelerometer X axis to Analog input 0
int steeringPot = A3; // connect the steering potentiometer to Analog input 3, skruvplint 2
int sensorPin = A1;  // Batterispänningen via spänningsdelare 68k + 10k från skruvplint 1

// value to hold the final angle
float angle = 0.00;
// the following 2 values should add together to equal 1.0
float gyro_weight = 0.98;
float accel_weight = 0.02;

// accelerometer values
int accel_reading;
int accel_raw;
int accel_avg = 0;
int accel_offset = 431;
float accel_angle;
float accel_scale = 0.01;

//gyroscope values
int gyro_avg = 0;
int gyro_offset = 407;
int gyro_raw;
int gyro_reading;
float gyro_rate;
float gyro_scale = 0.01;
// 01 by default
float gyro_angle;
//float loop_time = -0.05;
float loop_time = -0.02;

// engage button variables
int sensorValue = 0;
int engage = false;
int engage_state = 1;
// timer variables
//int last_update;
int cycle_time;
long last_cycle = 0;
// motor speed variables
float motor_out = 0;
int motor_1_out = 0;
int motor_2_out = 0;
int m1_speed = 0;
int m2_speed = 0;
int output;

// potentiometer variables
int steer_val;
int steer_range = 40;
int steer_reading;
int steer_min;
int steer_max;
int gain_val = 64;

// Reglerparametrar
float K; // Förstärkningsfaktor för reglersignalen till motorerna
float P;
float p1;
int Ti;
float I;
float u;
int h;


void calibrate()
{
  delay(200);
  // setup loop to read gyro 10 times
  for (int i = 0; i < 10; i++)
  {
    // read gyro and accel each time, add the value to the running total
    gyro_avg = gyro_avg + analogRead(gyro_pin); 
    accel_avg = accel_avg + analogRead(accel_pin); 
  }
  // with a sum of 10 readings, divide by 10 to get the average
  gyro_offset = gyro_avg / 10;
  accel_offset = accel_avg / 10;
  // Kalibrering av styrpotentiometern
  steer_max = analogRead(steeringPot) + 250;
  steer_min = analogRead(steeringPot) - 250;
  // let pin voltage settle
  delay(100);
  digitalWrite(cal_led, HIGH);
}

void setup()
{
  // Start the Serial monitor at 9600bps
  Serial.begin(9600);
  // set the foot_switch pin as an Input
  pinMode(foot_switch, INPUT);
  // enable the Arduino internal pull-up resistor on the engage_switch pin.
  digitalWrite(foot_switch, HIGH);
  // Tell Arduino to use the Aref pin for the Analog voltage, don't forget to connect 3.3v to Aref!
  analogReference(EXTERNAL);
  pinMode(batt_led_1, OUTPUT);
  pinMode(batt_led_2, OUTPUT);
  pinMode(cal_led, OUTPUT);
  pinMode(ready_led, OUTPUT);
  digitalWrite(cal_led, LOW);
  digitalWrite(batt_led_1, LOW);
  digitalWrite(batt_led_2, LOW);
  digitalWrite(ready_led, LOW);
  // calibrate gyro and accelerometer, 
  // you should keep the Seg-bot still and level when turning on so calibration will be accurate
  calibrate();

   // Reglerparametrar
  K = 1; // Förstärkningsfaktor
  h = 20; // Looptiden 20 ms
  Ti = 4000; // Tidskonstant för integrering
  p1 = K*h/Ti;

}

void loop()
{
  // Start the loop by getting a reading from the Accelerometer and converting it to an angle
  sample_accel();
  // now read the gyroscope to estimate the angle change
  sample_gyro();
  // combine the accel and gyro readings to come up with a "filtered" angle reading
  calculate_angle();
  // read the values of each potentiometer
  read_pots();
  // make sure bot is level before activating the motors
  auto_level();
  // update the motors with the new values
  update_motor_speed();
  // Kontroll av batterispänningen
  battery_level();
  // check the loop cycle time and add a delay as necessary
  time_stamp();

}

void sample_accel()
{
  // Read and convert accelerometer value
  accel_reading = analogRead(accel_pin);
  accel_raw = accel_reading - accel_offset;
  accel_raw = map(accel_raw, -130, 130, -90, 90);
  accel_angle = (float)(accel_raw * accel_scale);
}

void sample_gyro()
{
  // Read and convert gyro value
  gyro_reading = analogRead(gyro_pin);
  gyro_raw = gyro_reading - gyro_offset;
  gyro_rate = (float)(gyro_raw * gyro_scale) * -loop_time;
  gyro_angle = angle + gyro_rate;
}

void calculate_angle()
{
  angle = (float)(gyro_weight * gyro_angle) + (accel_weight * accel_angle); 
}

void read_pots()
{
  // Read and convert potentiometer values
  // Steering potentiometer
  // We want to map this into a range between -1 and 1, and set that to steer_val
  steer_reading = analogRead(steeringPot);
  steer_val = map(steer_reading,  steer_min, steer_max, steer_range, -steer_range);
}

void battery_level()
{
  // Övervakar batterispänningen
  sensorValue = analogRead(sensorPin);   
  if (sensorValue < 960 && sensorValue >= 928)
  {
    digitalWrite(batt_led_1, HIGH);  
    digitalWrite(batt_led_2, LOW);  
  }
  else if (sensorValue < 928)
  {
    digitalWrite(batt_led_2, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(batt_led_1, LOW);  
  }
  else
  {
    digitalWrite(batt_led_1, LOW);  
    digitalWrite(batt_led_2, LOW);    // turn the LED off by making the voltage LOW
  } 
}

void auto_level()
{
  // enable auto-level turn On
  //engage_state = digitalRead(engage_switch);
  if (digitalRead(foot_switch) == 1)
  {
    delay(10);
    if ( digitalRead(foot_switch) == 1)
    {
      engage = false;
      digitalWrite(ready_led, LOW);
      digitalWrite(cal_led, HIGH);
    }
  }
  else
  {
    if (engage == false)
    {
      if (angle < 0.02 && angle > -0.02)
      {
        engage = true;
        digitalWrite(ready_led, HIGH);
        digitalWrite(cal_led, LOW);
      }
      else 
      {
        engage = false;
        digitalWrite(ready_led, LOW);
        digitalWrite(cal_led, HIGH);
      }
    }
    else 
    {
      engage = true;
      digitalWrite(ready_led, HIGH);
      digitalWrite(cal_led, LOW);
    }
  }
}

void update_motor_speed()
{
  // Update the motors
  if (engage == true)
  {
    if (angle < -0.2 || angle > 0.2)
    {
      motor_out = 0;
    }
    else 
    {
      P = K * angle; // P-reglering
      u = P + I; // P-reglering + integrering
      output = (u * -1000);
      I = I + p1 * angle; // Integrering
      motor_out = map(output, -90, 90, -gain_val, gain_val); // map the angle. Ändring av insignal från 250 till 70
      if(motor_out > 0.7 * gain_val) // Extra accelerationspuls vid 70% av max hastighet framåt
      {
        motor_out = gain_val;
      }
      if(motor_out < 0.7 * -gain_val) // Extra accelerationspuls vid 70% av max hastighet bakåt
      {
        motor_out = - gain_val;
      }
    }
    // assign steering bias
    motor_1_out = motor_out + steer_val;
    motor_2_out = motor_out - steer_val;
 
    // test for and correct invalid values
    if(motor_1_out > 64)
    {
      motor_1_out = 64;
    }
    if(motor_1_out < -64)
    {
      motor_1_out = -64;
    }
    if(motor_2_out > 64)
    {
      motor_2_out = 64;
    }
    if(motor_2_out < -64)
    {
      motor_2_out = -64;
    }
    // assign final motor output values
    m1_speed = 64 + motor_1_out;
    m2_speed = 192 + motor_2_out;
    
    if (m1_speed < 1)
    {
       m1_speed = 1;
    }
    else if (m1_speed > 127)
    {
      m1_speed = 127;
    }
    if (m2_speed < 128)
    {
      m2_speed = 128;
    }
    else if (m2_speed > 255)
    {
      m2_speed = 255;
    }
  }
  else
  {
    m1_speed = 0;
    m2_speed = 0;
  }
  
  // Serial speed values write here:
  Serial.write(m1_speed);
  Serial.write(m2_speed);
}

void time_stamp()
{
  // check to make sure it has been exactly 50 milliseconds since the last recorded time-stamp
  while((millis() - last_cycle) < 20) // 50
  {
    delay(1);
  }
  // once the loop cycle reaches 50 mS, reset timer value and proceed
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}


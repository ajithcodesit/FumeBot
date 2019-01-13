#include <Wire.h>
#include "MAX30105.h"
#include <SparkFunCCS811.h>
#include <Adafruit_BME280.h>
#include "Adafruit_FONA.h"
#include "ServoMod.h"

//#############################################################SENSORS AND PERIPHERALS############################################################

const uint8_t CCS811_ADDRESS=0x5B;  //IIC address for CCS811
const uint8_t FONA_RST=4;

const uint8_t MCU_RDY_PIN=30;  // This pin is used to tell the RPi that it is ready

HardwareSerial *fonaSerial = &Serial2;

MAX30105 particle_sensor;
CCS811 air_quality(CCS811_ADDRESS);
Adafruit_BME280 bme; //For IIC mode
Adafruit_FONA fona=Adafruit_FONA(FONA_RST);

// Sensor variables
const unsigned int SENSOR_READING_DELAY=150;  //In ms

struct sensor_thresholds  //Struct to store the thresholds for the different sensor
{
  //For gas sensor
  unsigned int eCO2=1000;
  unsigned int TVOC=100;

  //For particle sensor
  unsigned int red=10000;
  unsigned int green=10000;
  unsigned int ir=10000;

  boolean updated=false; //Variable to say whether the thresholds were updated to the required values or not
}thresh;

//3G module variables
struct sms_comm
{
  boolean sms_repeat_disable=false; //This can be used to rearm the sms alert
  boolean sms_send=false;  //Boolean to say whether the SMS was sent or not
  boolean enable_sms=false; //Boolean to say whether the SMS is allowed or not
  char sms_num[21]="0000000000";  // Number to send the alert to
  char alert_msg[140]="Test message";
  uint8_t sms_tries=0;
  const uint8_t max_tries=3;
}alrt_com;

enum ack_cfg_codes //These codes are sent pack to say that these parameters have been updated by the raspberry pi
{
  GAS_THR=1,
  PAR_THR=2,
  SMS_SET=3,
  RST_ALM=6,
};

//#############################################################STEPPER DRIVER############################################################
//Microstepping pins
const uint8_t ms1=22;
const uint8_t ms2=24;

//Eanble pin
const uint8_t enb_l=28;
const uint8_t enb_r=31;

//Step and direction for the left motor
const uint8_t stp_l=23;
const uint8_t dir_l=25;

//Step and direction for the right motor
const uint8_t stp_r=27;
const uint8_t dir_r=29;

enum prescalers  //Prescalers for Timer 3 and 4
{
  PS0=0,   //No clock source
  PS1=1,   //No prescaling
  PS8=8,   //Clock I/O divider of 8
  PS64=64,  //Clock I/O divider of 64
  PS256=256, //Clock I/O divider of 256
  PS1024=1024 //Clock I/O divider of 1024
};

const uint32_t F_CLK_IO=16e6;  //CPU clock frequency 16*10^6

//This is the absolute maximum limit of the RPM that is allowed
const float ABS_MAX_RPM=1000.0;
const float ABS_MIN_RPM=0.0357;

//Robot motion limits
const float MAX_MOV_SPEED=0.30; //Maximum movement speed m/s
const float MIN_MOV_SPEED=0.20; //Minimum movement speed m/s

const float MAX_TURN_SPEED=75.0; //Degrees/s
const float MIN_TURN_SPEED=45.0; //Degrees/s

const float TURN_RADIUS=0.332; //In meters

//Differential geometry
const float WHEEL_RADIUS=0.035; //In meters
const float HALF_AXLE=0.166; //In meters

//This the actual RPM used
const float MAX_RPM=200; //Can go up to 200 RPM
const float MIN_RPM=0.5; //Can go down to 0.5 RPM
const int MAX_STEPS=50; //Maximum allowed no of steps
volatile int BUF_STEPS=MAX_STEPS/2; //Steps after which the steps completed boolean will be set true [This is done to act as buffer]
const float DPS=1.8; //Degree Per Step of the stepper motor
const uint8_t MS=1; //Microstepping (1 to 16)

//Rotation direction constants
const boolean LEFT_CLK=HIGH;
const boolean LEFT_ACLK=LOW;

const boolean RIGHT_CLK=HIGH;
const boolean RIGHT_ACLK=LOW;

//Enable or disable stepper motors drivers
const boolean DISABLE_SMOT=HIGH;
const boolean ENABLE_SMOT=LOW;

struct stepping //Contains the variable used by the right and left motor
{
  uint8_t step_pin;
  uint8_t dir_pin;
  volatile int step_count=0; //The steps that were done
  volatile int req_steps=0; //This is compared to the step count variable
  volatile uint16_t req_count=0; //This used to change the speed in the ISR [set_rpm should not be called outside the setup]
  volatile boolean steps_completed=true; //Variable to say whether the steps were completed or not
  volatile boolean direct; //Sets the direction the motor has to turn
  volatile boolean start_rotation=true; //To start rotation
  volatile boolean until_left_stops=false; //For rotating right stepper until left stops
  volatile boolean until_right_stops=false; //For rotating left stepper until right stops
}left,right;

struct driver_config //Microstepping variables
{
  uint8_t ms1,ms2; //Stores the microstepping pins (Both motor uses the same pins)
  uint8_t enb_left; //Stores the enable pin for left
  uint8_t enb_right; //Stores the enable pin for right
  boolean auto_enb_disb=true; //Whether the motor will be disabled automatically when not in used
  boolean override_enb_disb=false; //Overide the enable or disable stepper driver settings
}conf;

enum differential_direction //Constants to set the direction pin
{
  LEFT_FORWARD=LEFT_ACLK,
  LEFT_BACKWARD=LEFT_CLK,
  RIGHT_FORWARD=RIGHT_CLK,
  RIGHT_BACKWARD=RIGHT_ACLK
};

//#############################################################DIFFERENTIAL PARAMETER############################################################
float wheel_radius;
float half_axle;
float max_diff_rpm=0;
float min_diff_rpm=0;
const float turn_angle=90.0;

//#############################################################UART STEPPER CONTROL##############################################################

//List of commands for the stepper motor
//Since the controller is not analog clear precise instuctions are used for the motor like move forward, backward etc...
enum cmd_list
{
  FRWD=109,         //Move forward
  BWRD=113,         //Move backward
  ROT_LEFT=127,     //Rotate robot left
  ROT_RIGHT=179,    //Rotate robot right  
  L_TURN_F=181,     //Turn left while moving forward
  R_TURN_F=191,     //Turn right while moving forward
  L_TURN_B=193,     //Turn left while moving backward
  R_TURN_B=197,     //Turn right while moving backward
  INVALID=255       //This an invalid command
};

typedef struct
{
  uint8_t ctrl_cmd=INVALID;
  uint8_t accl_enb=0;
  uint8_t brake_enb=0;
}ctrl_t; //Data of the type ctrl_t 

ctrl_t control;

//All the allowed motions are put into array for checking if the correct command is received
const uint8_t stepper_cmd_arr[8]={FRWD,BWRD,ROT_LEFT,ROT_RIGHT,L_TURN_F,R_TURN_F,L_TURN_B,R_TURN_B};

//#################################################################PAN TILT SERVOS################################################################

//Object of the modified servo library (Only works with MEGA2560)
ServoMod pan_servo,tilt_servo_l,tilt_servo_r;

//Servo pins
const uint8_t pan_servo_pin=9;
const uint8_t tilt_servo_pin_l=10;
const uint8_t tilt_servo_pin_r=11;

//To avoid the servo wrapping around 
const int MAX_PAN_ANGLE=170;
const int MIN_PAN_ANGLE=10;

const int MAX_TILT_ANGLE=170;
const int MIN_TILT_ANGLE=10;

//Servo pan and tilt positions
int pan_pos=90;
int tilt_pos=90;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);  //Connects to /dev/serial0 of the RPi3B
  fonaSerial->begin(115200);

  pinMode(MCU_RDY_PIN,OUTPUT);
  digitalWrite(MCU_RDY_PIN,LOW); //This says microcontroller is not ready

  //For the stepper motors
  init_timer3_ctc(); //Initialising Timer 3
  init_timer4_ctc(); //Initialising Timer 4

  set_microstepping_pins(ms1,ms2);  //Microstepping set to 200 steps per revolution
  set_enable_pin(enb_l,enb_r,ENABLE_SMOT,false);  //Both the left and right motors are disabled and only of steps needs to be executed do they are enabled
  
  set_step_pins_LR(stp_l,stp_r);
  set_dir_pins_LR(dir_l,dir_r);
  
  set_rpm_left(MAX_RPM); //Set the RPM for left
  set_rpm_right(MAX_RPM); //Set the RPM for right

  differential_param(WHEEL_RADIUS,HALF_AXLE,MIN_RPM,MAX_RPM);

  //For the servo motors
  pan_servo.attach(pan_servo_pin);
  tilt_servo_l.attach(tilt_servo_pin_l);
  tilt_servo_r.attach(tilt_servo_pin_r);
  ctrl_pan_and_tilt(pan_pos,tilt_pos); //Move to correct postion during initialization

  //For initializing the sensors
  init_sensors_modules();

  digitalWrite(MCU_RDY_PIN,HIGH); //This says microcontroller is ready
}

void loop() {
  // put your main code here, to run repeatedly:
  receive_commands_and_execute(&control); //Get the command
  ctrl_cmd_execution(&control); //Change the stepper behaviour according to the command
  ctrl_pan_and_tilt(pan_pos,tilt_pos); //Pan and tilt the camera

  if(is_update_allowed(SENSOR_READING_DELAY))
  {
    read_check_send_gas_data();
    read_check_send_particle_data();
    read_send_environmental_data();
    send_alert_SMS();
  }
}

//#############################################################SENSORS AND PERIPHERALS############################################################

void init_sensors_modules()  // Function to initialize the sensors
{
  if(!fona.begin(*fonaSerial))  //Test the 3G module
  {
    Serial.println("\nSIM5320E 3G module not found!\n");
  }
  
  if(!particle_sensor.begin(Wire,I2C_SPEED_FAST)) //Use I2C at 400kHz and checking if the sensor is present
  {
    Serial.print("\nMAX30105 sensor not found!\n");
  }

  if(air_quality.begin() != 0)
  {
    Serial.print("CCS811 sensor not found!\n");
  }

  if(!bme.begin())
  {
    Serial.print("BME280 sensor not found!\n");
  }

  particle_sensor.setup(); //Configure the sensor
}

boolean is_update_allowed(unsigned long delay_millis) //Function to say whether the update should be done according to a set interval
{
  static unsigned long prev_millis;

  if((millis()-prev_millis)>delay_millis) 
  {
    prev_millis=millis();
    return true;
  }
  else return false;
}

void send_alert_SMS()  //Function to send an SMS alert
{
  if(alrt_com.sms_send && alrt_com.enable_sms && (alrt_com.sms_tries<alrt_com.max_tries) && !alrt_com.sms_repeat_disable) //If the sms is not sent
  { 
    if(is_registered_to_network()) //If the SIM is registered to the home network
    {
      if (!fona.sendSMS(alrt_com.sms_num,alrt_com.alert_msg)) //Sending the SMS alert
      {
        Serial.println("SMS alert failed to sent!");
        alrt_com.sms_send=true;
        alrt_com.sms_tries++;
      } 
      else 
      {
        Serial.println("SMS alert sent!");
        alrt_com.sms_tries=0; //Reset the tries
        alrt_com.sms_send=false;
        alrt_com.sms_repeat_disable=true; //The repeat is disabled 
      }
    }
    else
    {
      Serial.println("Not registered to any network, cannot send alert SMS");
    }
  }
}

boolean is_registered_to_network()  //Check if we are connected to the network
{
  if(fona.getNetworkStatus() == 1) return true;
  else return false;
}

void read_environmental_sensor(float &pres, float &temp, float &hum)  //Environmental sensor readings
{
  temp=bme.readTemperature(); //Degree Centigrade
  pres=bme.readPressure(); //Pascals (Pa)
  hum=bme.readHumidity();  // Percentage Relative Humidity (%RH)

  if(is_enviroment_update_allowed(300000))
  {
    air_quality.setEnvironmentalData(hum,temp); //Using the BME280 sensor to set the humidity and temperature for the air quality sensor
  } 
}

boolean is_enviroment_update_allowed(unsigned long delay_millis) //Function to say whether the update should be done according to a set interval
{
  static unsigned long prev_millis;
  static boolean init_update;  //Initial update

  if(!init_update)  // If the initial update of the environmental data is not
  {
    init_update=true;
    return true;
  }

  if((millis()-prev_millis)>delay_millis) 
  {
    prev_millis=millis();
    return true;
  }
  else return false;
}

void send_environmental_data(float pres, float temp, float hum) //Send the environmental sensor readings
{
  Serial1.print("e,");
  Serial1.print(pres);
  Serial1.print(',');
  Serial1.print(temp);
  Serial1.print(',');
  Serial1.print(hum);
  Serial1.println();
}

void read_send_environmental_data() //Send the environmental data
{
  float pressure,temperature,humidity;
  read_environmental_sensor(pressure,temperature,humidity);
  send_environmental_data(pressure,temperature,humidity);
}

void read_particle_sensor(unsigned int &r, unsigned int &g, unsigned int &ir)  //Particle sensor readings
{
  r=particle_sensor.getRed();
  g=particle_sensor.getGreen();
  ir=particle_sensor.getIR();
}

void send_particle_data(unsigned int r, unsigned int g, unsigned int ir)  //Send the particle sensor data
{
  Serial1.print("p,");
  Serial1.print(r);
  Serial1.print(',');
  Serial1.print(g);
  Serial1.print(',');
  Serial1.print(ir);
  Serial1.println();
}

void check_particle_data(unsigned int r, unsigned int g, unsigned int ir)  //Function to check for the required particle eg: smoke 
{
  if(r > thresh.red && g > thresh.green && ir > thresh.ir)
  {
    Serial.println("Particle threshold reached!");
    Serial.println("R= "+String(r)+" G= "+String(g)+" IR= "+String(ir));
    alrt_com.sms_send=true;

    String msg="Particle Alert\nRed= "+String(r)+"\nGreen= "+String(g)+"\nIR= "+String(ir);
    msg.toCharArray(alrt_com.alert_msg,(sizeof(alrt_com.alert_msg)/sizeof(char))); //The message to be sent is converted to a char array
  }
}

void read_check_send_particle_data() //Send particle data
{
  unsigned int red,green,ir;
  read_particle_sensor(red,green,ir);
  check_particle_data(red,green,ir);
  send_particle_data(red,green,ir);
}

void read_gas_sensor(unsigned int &eco2, unsigned int &tvoc)  //Gas sensor readings
{
  if(air_quality.dataAvailable()) // The data during the first 20 minutes are not accurate
  {
    air_quality.readAlgorithmResults();
    eco2=air_quality.getCO2();
    tvoc=air_quality.getTVOC();
  }
}

boolean is_gas_sens_wait_done(unsigned long delay_millis)  //Function to wait for the gas sensor readings to stabalize
{
  static boolean wait_done=false;
  static unsigned long prev_millis;

  if(((millis()-prev_millis)>delay_millis) && !wait_done)
  {
    wait_done=true;
    prev_millis=millis();
  }
  return wait_done;
}

void send_gas_readings(unsigned int eco2, unsigned tvoc)  //Send the gas sensor reading 
{
  if(is_gas_sens_wait_done(10000))
  {
    Serial1.print("g,");
    Serial1.print(eco2);
    Serial1.print(',');
    Serial1.print(tvoc);
    Serial1.println();
  }
}

void check_gas_data(unsigned int eco2, unsigned tvoc)
{
  if((eco2 > thresh.eCO2 || tvoc > thresh.TVOC) && is_gas_sens_wait_done(10000))
  {
    Serial.println("Gas Threshold reached");
    Serial.println("eCO2= "+String(eco2)+" TVOC= "+String(tvoc));
    alrt_com.sms_send=true;
    
    String msg="Air Quality Alert\neCO2= "+String(eco2)+" ppm\nTVOC= "+String(tvoc)+" ppb";
    msg.toCharArray(alrt_com.alert_msg,(sizeof(alrt_com.alert_msg)/sizeof(char))); //The message to be sent is converted to a char array
  }
}

void read_check_send_gas_data() //Send the gas readings
{
  unsigned int eCO2,TVOC;
  read_gas_sensor(eCO2,TVOC);
  check_gas_data(eCO2,TVOC);
  send_gas_readings(eCO2,TVOC);
}

//FUNCTION TO DEBUG THE SENSORS

void debug_bme280()
{
  float temp=bme.readTemperature(); //Degree Centigrade
  float pres=bme.readPressure(); //Pascals (Pa)
  float hum=bme.readHumidity();  // Percentage Relative Humidity (%RH

  Serial.print("Temperature= ");
  Serial.print(temp);
  Serial.print(" C");
  Serial.print(" Pressure= ");
  Serial.print(pres);
  Serial.print(" Pa");
  Serial.print(" Humidity= ");
  Serial.print(hum);
  Serial.print(" %RH");
  Serial.println();
}

void debug_max30105()
{
  Serial.print("R= ");
  Serial.print(particle_sensor.getRed());
  Serial.print(" IR= ");
  Serial.print(particle_sensor.getIR());
  Serial.print(" G= ");
  Serial.print(particle_sensor.getGreen());
  Serial.println();
}

void debug_ccs811()
{
  if(air_quality.dataAvailable()) // The data during the first 20 minutes are not accurate
  {
    air_quality.readAlgorithmResults();
    Serial.print("eCO2= ");
    Serial.print(air_quality.getCO2());
    Serial.print(" ppm");
    Serial.print(" TVOC= ");
    Serial.print(air_quality.getTVOC());
    Serial.print(" ppb");
    Serial.println();
  }
}

//#############################################################STEPPER DRIVER############################################################

ISR(TIMER3_COMPA_vect) //ISR for Timer 3
{
  OCR3A=left.req_count; //Updating the new count here as the counter TCNT3 is reset to 0 for this ISR to be called 
  
  if(((left.step_count < left.req_steps) || left.until_right_stops) && left.start_rotation)
  {
    if(conf.auto_enb_disb) digitalWrite(conf.enb_left,ENABLE_SMOT);
    digitalWrite(left.dir_pin,left.direct); //Set the direction of the motor to turn
    digitalWrite(left.step_pin,digitalRead(left.step_pin)^1); //Toggle the pin
    if(digitalRead(left.step_pin)) left.step_count++; //when ever the pin goes from LOW -> HIGH we get a step and therefore increment the counter
    if(left.req_steps-left.step_count <= BUF_STEPS) left.steps_completed=true; //This is allow for some buffer
    else left.steps_completed=false; //Steps are not completed
  }
  else 
  { 
    if(!conf.override_enb_disb && conf.auto_enb_disb) digitalWrite(conf.enb_left,DISABLE_SMOT);
    digitalWrite(left.step_pin,LOW); //If the last operation left the pin HIGH, set it LOW
    right.until_left_stops=false; //Because left has stopped rotating
    left.steps_completed=true;  //Steps are completed
    left.step_count=0;
    left.req_steps=0;
  }
}

ISR(TIMER4_COMPA_vect)
{
  OCR4A=right.req_count; 

  if(((right.step_count < right.req_steps) || right.until_left_stops) && right.start_rotation)
  {
    if(conf.auto_enb_disb) digitalWrite(conf.enb_right,ENABLE_SMOT);
    digitalWrite(right.dir_pin,right.direct);
    digitalWrite(right.step_pin,digitalRead(right.step_pin)^1);
    if(digitalRead(right.step_pin)) right.step_count++;
    if(right.req_steps-right.step_count <= BUF_STEPS) right.steps_completed=true;
    else right.steps_completed=false;
  }
  else
  {
    if(!conf.override_enb_disb && conf.auto_enb_disb) digitalWrite(conf.enb_right,DISABLE_SMOT);
    digitalWrite(right.step_pin,LOW);
    left.until_right_stops=false; //Because the right has stopped rotating
    right.steps_completed=true;
    right.step_count=0;
    right.req_steps=0;
  }
}

void set_step_pins_LR(uint8_t l_step_pin, uint8_t r_step_pin) //Set the pins for the step left and right
{
  left.step_pin=l_step_pin;
  right.step_pin=r_step_pin;

  pinMode(left.step_pin,OUTPUT);
  digitalWrite(left.step_pin,LOW); 

  pinMode(right.step_pin,OUTPUT);
  digitalWrite(right.step_pin,LOW);
}

void set_dir_pins_LR(uint8_t l_dir_pin, uint8_t r_dir_pin) //Set the direction pins
{
  left.dir_pin=l_dir_pin;
  right.dir_pin=r_dir_pin;

  pinMode(left.dir_pin,OUTPUT);
  digitalWrite(left.dir_pin,LOW);

  pinMode(right.dir_pin,OUTPUT);
  digitalWrite(right.dir_pin,LOW);
}

void set_microstepping_pins(uint8_t ms1, uint8_t ms2) // Set the MS pins
{
  conf.ms1=ms1;
  conf.ms2=ms2;

  pinMode(conf.ms1,OUTPUT);
  pinMode(conf.ms2,OUTPUT);

  /*
  DRV8834 microstepping setup table
  ------------------------------------------
  MS1        MS2    Microstepping Resolution
  ------------------------------------------
  Low        Low          Full step
  High       Low          Half step
  Floating   Low          1/4 step
  Low        High         1/8 step
  High       High         1/16 step
  Floating   High         1/32 step
  */

  //Currently set to Full Step Mode for the DRV8834
  digitalWrite(conf.ms1,LOW);
  digitalWrite(conf.ms2,LOW);
}

void set_enable_pin(uint8_t l_enb_pin, uint8_t r_enb_pin, boolean enb_or_disb, boolean auto_smot_enb) //Set the enable pin
{
  conf.enb_left=l_enb_pin;
  conf.enb_right=r_enb_pin;
  conf.auto_enb_disb=auto_smot_enb;

  pinMode(conf.enb_left,OUTPUT);
  pinMode(conf.enb_right,OUTPUT);
  
  digitalWrite(conf.enb_left,enb_or_disb); //HIGH to disable and LOW to enable
  digitalWrite(conf.enb_right,enb_or_disb); //HIGH to disable and LOW to enable
}

void set_rpm_left(float rpm) //Function to set the RPM (The count is updated to OCR3A register)
{
  //This function should not be called outside the setup function because of the counter might miss the compare when new value is written
  cli();
  left.req_count=calc_counts_to_compare(F_CLK_IO,PS1024,step_frequency(rpm,MS,DPS)); 
  OCR3A=left.req_count; //Set the required to the Output Compare Register for Timer 3 
  sei();
}

void set_rpm_right(float rpm) //Function to set the RPM (The count is updated to OCR4A register)
{
  //This function should not be called outside the setup function because of the counter might miss the compare when new value is written
  cli();
  right.req_count=calc_counts_to_compare(F_CLK_IO,PS1024,step_frequency(rpm,MS,DPS));
  OCR4A=right.req_count;
  sei();
}

void update_rpm_left(float rpm) //Function to update the RPM (The count is updated when the ISR is called)
{
  //This function can be called anywhere
  left.req_count=calc_counts_to_compare(F_CLK_IO,PS1024,step_frequency(rpm,MS,DPS));
}

void update_rpm_right(float rpm) //Function to update the RPM (The count is update when the ISR is called)
{
  //This function can be called anywhere
  right.req_count=calc_counts_to_compare(F_CLK_IO,PS1024,step_frequency(rpm,MS,DPS));
}

void rotate_motor_left(int steps, boolean direct, float rpm) //Function to add steps to be rotate the stepper
{
  left.req_steps=left.req_steps+steps;
  left.direct=direct;
  left.req_count=calc_counts_to_compare(F_CLK_IO,PS1024,step_frequency(rpm,MS,DPS)); //RPM is updated not set
}

void rotate_motor_right(int steps, boolean direct, float rpm)  //Function to add steps to be rotate the stepper
{
  right.req_steps=right.req_steps+steps;
  right.direct=direct;
  right.req_count=calc_counts_to_compare(F_CLK_IO,PS1024,step_frequency(rpm,MS,DPS));  //RPM is updated not set
}

boolean is_left_steps_completed()
{
  return left.steps_completed;
}

boolean is_right_steps_completed()
{
  return right.steps_completed;
}

void reset_left_motor()
{
  right.until_left_stops=false;
  left.steps_completed=true;  
  left.step_count=0;
  left.req_steps=0;
}

void reset_right_motor()
{
  left.until_right_stops=false;
  right.steps_completed=true;
  right.step_count=0;
  right.req_steps=0;
}

float step_frequency(float rpm, uint8_t usn, float dps) //Function to calculate the required stepping frequency to acheive the desired RPM
{
  if(rpm > ABS_MAX_RPM) rpm=ABS_MAX_RPM; //Clip the maximum RPM to the absolute limit
  else if(rpm < ABS_MIN_RPM) rpm=ABS_MIN_RPM; //Clip the minimum RPM to the absolute limit

  //fstep=(360*RPM*Nm*2)/(60*DPS)
  float fstep=((12.0*rpm*usn)/dps); //This is microsteps/second (Frequency of steps required to get the desired RPM)
  return fstep;
}

uint16_t calc_counts_to_compare(uint32_t f_clk_io, uint16_t ps, float f_req) //Function that calculates the required counts for desired frequency
{
  if(ps <= 0) ps=1; //If no clock source is selected set prescaling to 1
  
  uint32_t f_clk_to_timer=f_clk_io/ps; //Calculate the prescaled frequency of the clock that goes into the timer
  
  float req_counts_f=(f_clk_to_timer/f_req); //Get the counts to be put in OCRxA register to produce the required frequenct at 50% Duty Cycle
  
  uint16_t req_counts_i=round(req_counts_f)-1; //Round and subtract 1 as the TCNTx counts from 0

  if((req_counts_i > 0) && (req_counts_i < 65535)) return req_counts_i; //Check if the counts are with in range
  else
  {
    Serial.print("\nCounts out of range!\n"); 
    return f_clk_to_timer; //Now the compare match occurs at 1 Hz if the counts are not with in range
  }
}

void update_step_buffer(int reduced_steps) // Updates the value of the step buffer to the new value
{
  if(reduced_steps > MAX_STEPS/2)
  {
    BUF_STEPS=(reduced_steps/2); //Use half of the reduced steps as the new buffer steps
  }
  else
  {
    reset_step_buffer();
  }
}

void reset_step_buffer() //Reset the step buffer to the half of the maximum allowed steps (Call this where ever the update buffer is not called)
{
  BUF_STEPS=MAX_STEPS/2;
}

void enable_right_until_left_stops()
{
  right.until_left_stops=true; //Tell the ISR for right to stop rotation when left stops
  left.until_right_stops=false; //Only one should be active
  right.req_steps=0; //The required steps must always zero
}

void enable_left_until_right_stops()
{
  left.until_right_stops=true; //Tell the ISR for left to stop rotation when right stops
  right.until_left_stops=false; //Only one should be active
  left.req_steps=0; //The required steps must always zero
}

void disable_right_until_left_stops()
{
  right.until_left_stops=false; //This doesn't wait for the left to stops 
}

void disable_left_until_right_stops()
{
  left.until_right_stops=false; //This doesn't wait foe the right to stop
}

void init_timer3_ctc() //Initialise Timer 3 in CTC mode
{
  cli(); //Disable all interrupts
  
  TCCR3A=0;  //Timer 3 control register A reset
  TCCR3B=0;  //Timer 3 control register B reset

  TCNT3=0; //Reset the Timer 3 counter

  TCCR3B |= (1<<WGM32); //Puts Timer 3 into CTC mode (Clear Timer on Compare Match)
  set_prescaler_timer3(PS1024); //Set the prescaler to 1024

  TIMSK3 |= (1<<OCIE3A); //Enabling the timer compare interrupt
  
  sei(); //Enable all the interrupts
}

void init_timer4_ctc() //Initialise Timer 4 in CTC mode
{
  cli();

  TCCR4A=0;
  TCCR4B=0;

  TCNT4=0;

  TCCR4B |= (1<<WGM42); 
  set_prescaler_timer4(PS1024);

  TIMSK4 |= (1<<OCIE4A);

  sei();
}

void set_prescaler_timer3(uint16_t ps) //Function to set the prescaler of the clock that goes into Timer 3
{
  static uint8_t prev_ps; //Previous prescaler value
  if(ps != prev_ps)
  {
    cli(); //Disable all interrupts
  
    TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30)); //Clear the CS bits
    
    switch(ps)
    {
      case PS0: TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30)); break;
      case PS1: TCCR3B |= (1<<CS30); break;
      case PS8: TCCR3B |= (1<<CS31); break;
      case PS64: TCCR3B |= (1<<CS31) | (1<<CS30); break;
      case PS256: TCCR3B |= (1<<CS32); break;
      case PS1024: TCCR3B |= (1<<CS32) | (1<<CS30); break;
      default: TCCR3B &= ~((1<<CS32) | (1<<CS31) | (1<<CS30)); //Clear the CS bits
    }
    
    sei(); //Enable all the interrupts

    prev_ps=ps; //Update the previous prescaler value
  }
}

void set_prescaler_timer4(uint16_t ps)
{
  static uint8_t prev_ps;
  if(ps != prev_ps)
  {
    cli();

    TCCR4B &= ~((1<<CS42) | (1<<CS41) | (1<<CS40));

    switch(ps)
    {
      case PS0: TCCR4B &= ~((1<<CS42) | (1<<CS41) | (1<<CS40)); break;
      case PS1: TCCR4B |= (1<<CS40); break;
      case PS8: TCCR4B |= (1<<CS41); break;
      case PS64: TCCR4B |= (1<<CS41) | (1<<CS40); break;
      case PS256: TCCR4B |= (1<<CS42); break;
      case PS1024: TCCR4B |= (1<<CS42) | (1<<CS40); break;
      default: TCCR4B &= ~((1<<CS42) | (1<<CS41) | (1<<CS40));
    }

    sei();

    prev_ps=ps;
  }
}

//#############################################################UART STEPPER CONTROL##############################################################

void receive_commands_and_execute(ctrl_t *c)  //Function that receives the serial command
{
  String rx_data="";
  String data="";
  int arr_index=0;
  uint8_t sep_loc[20]={};  //Seperator location array

  if(Serial1.available() > 0)  //If serial data available
  {
    rx_data=Serial1.readStringUntil('\n');
    
    for(int i=0; i<rx_data.length(); i++)
    {
      if(rx_data.charAt(i) == ',')
      {
        sep_loc[arr_index]=i;
        arr_index++;
      }
    }

    String header=rx_data.substring(0,sep_loc[0]);  //Get the header which indicates the type of data

    if(header == "m") //For the movment of the robot
    {
      data=rx_data.substring(sep_loc[0]+1,sep_loc[1]);
      c->ctrl_cmd=data.toInt();

      data=rx_data.substring(sep_loc[1]+1,sep_loc[2]);
      c->accl_enb=data.toInt();

      data=rx_data.substring(sep_loc[2]+1,sep_loc[3]);
      c->brake_enb=data.toInt();

      for(uint8_t i=0; i<(sizeof(stepper_cmd_arr)/sizeof(uint8_t)); i++) //Check if it is a valid command
      {
        if(c->ctrl_cmd == stepper_cmd_arr[i]) return;
      }
      c->ctrl_cmd=INVALID; //Because the above test failed and reached here 
    }

    else if(header == "c") //For panning and tilting the camera mount
    {
      data=rx_data.substring(sep_loc[0]+1,sep_loc[1]);
      pan_pos=data.toInt();

      data=rx_data.substring(sep_loc[1]+1,sep_loc[2]);
      tilt_pos=data.toInt();
    }

    else if(header == "gt") //For setting the gas sensor thresholds
    {
      data=rx_data.substring(sep_loc[0]+1,sep_loc[1]);
      thresh.eCO2=data.toInt();

      data=rx_data.substring(sep_loc[1]+1,sep_loc[2]);
      thresh.TVOC=data.toInt();

      send_ack(GAS_THR);

      Serial.println("\nGas sensor thresholds set to\neCO2= "+String(thresh.eCO2)+"\nTVOC= "+String(thresh.TVOC));
    }

    else if(header == "pt") //For setting the particle sensor thresholds
    {
      data=rx_data.substring(sep_loc[0]+1,sep_loc[1]);
      thresh.red=data.toInt();

      data=rx_data.substring(sep_loc[1]+1,sep_loc[2]);
      thresh.green=data.toInt();

      data=rx_data.substring(sep_loc[2]+1,sep_loc[3]);
      thresh.ir=data.toInt();

      send_ack(PAR_THR);

      Serial.println("\nParticle sensor thresholds set to \nRed= "+String(thresh.red)+"\nGreen= "+String(thresh.green)+"\nIR= "+String(thresh.ir));
    }

    else if(header == "sm") //For setting the SMS number and enable
    {
      data=rx_data.substring(sep_loc[0]+1,sep_loc[1]);
      String SMS_number=data; //Store the number for printing
      SMS_number.toCharArray(alrt_com.sms_num,(sizeof(alrt_com.sms_num)/sizeof(char)));

      data=rx_data.substring(sep_loc[1]+1,sep_loc[2]);
      alrt_com.enable_sms=data.toInt();

      send_ack(SMS_SET);

      Serial.println("\nSMS setting changed to \nSMS alert number= "+SMS_number+"\nSMS enabled= "+String(alrt_com.enable_sms));
    }

    else if(header == "ra")  // For resetting the variables that set off the alarm
    {
      alrt_com.sms_send=false; //Set to the SMS was not sent
      alrt_com.sms_tries=0; //Reset the tries
      alrt_com.sms_repeat_disable=false; //The repeat is enabled until the repeat is disabled when the SMS was successfully sent
      
      send_ack(RST_ALM);
      
      Serial.println("\nAlarm variables reset");
    }
  }
}

void send_ack(uint8_t ack_no) //Function to send the setting acknowledgement string
{
  Serial1.print("ak,"); //Header says that this is an acknowledgement message
  Serial1.print(ack_no);
  Serial1.println();
}

void ctrl_cmd_execution(ctrl_t *c) //Function to convert the command to the direction and speed the robot must move at
{
  float rpm_left,rpm_right;  //RPM for the left and the right
  float movement_speed=MIN_MOV_SPEED;
  float turn_speed=MIN_TURN_SPEED; //The turning speed of the differential drive
  static uint8_t prev_cmd; //This is previous command that was received

  if(c->accl_enb == 0) //For acceleration
  {
    movement_speed=MIN_MOV_SPEED;
    turn_speed=MIN_TURN_SPEED;
  }
  else
  {
    movement_speed=MAX_MOV_SPEED;
    turn_speed=MAX_TURN_SPEED;
  }

  if(c->brake_enb == 1) // For Braking (If 1 brake is enabled)
  {
    digitalWrite(conf.enb_left,ENABLE_SMOT);
    digitalWrite(conf.enb_right,ENABLE_SMOT);

    conf.override_enb_disb=true; //The motor will not be disabled in the ISR automatically

    reset_left_motor();
    reset_right_motor();
  }
  else
  {
    conf.override_enb_disb=false; //The motor will be disabled automatically in the ISR
  }

  if(c->ctrl_cmd != prev_cmd && c->ctrl_cmd != INVALID && c->brake_enb != 1) //If the commands is different than the previous command then override becomes active
  {
    //This resets the is steps completed variables as well so the if statement below can be executed
    reset_left_motor(); 
    reset_right_motor();
  }
  
  //This function runs only when a new serial command is recevied
  if(c->ctrl_cmd != INVALID && is_left_steps_completed() && is_right_steps_completed() && c->brake_enb != 1)
  { 
    switch(c->ctrl_cmd)
    {
      case FRWD:
      {
        straight(movement_speed, rpm_left, rpm_right);

        disable_left_until_right_stops();
        disable_right_until_left_stops();
        
        rotate_motor_left(MAX_STEPS,LEFT_FORWARD,rpm_left);
        rotate_motor_right(MAX_STEPS,RIGHT_FORWARD,rpm_right);

        prev_cmd=FRWD;
        
        break;
      }
      case BWRD:
      {
        straight(movement_speed, rpm_left, rpm_right);

        disable_left_until_right_stops();
        disable_right_until_left_stops();
        
        rotate_motor_left(MAX_STEPS,LEFT_BACKWARD,rpm_left);
        rotate_motor_right(MAX_STEPS,RIGHT_BACKWARD,rpm_right);

        prev_cmd=BWRD;
        
        break;
      }
      case ROT_LEFT:
      {
        turn(turn_speed, rpm_left, rpm_right);

        disable_left_until_right_stops();
        disable_right_until_left_stops();
        
        rotate_motor_left(MAX_STEPS,LEFT_BACKWARD,rpm_left);
        rotate_motor_right(MAX_STEPS,RIGHT_FORWARD,rpm_right);

        prev_cmd=ROT_LEFT;
        
        break;
      }
      case ROT_RIGHT:
      {
        turn(turn_speed, rpm_left, rpm_right);

        disable_left_until_right_stops();
        disable_right_until_left_stops();
        
        rotate_motor_left(MAX_STEPS,LEFT_FORWARD,rpm_left);
        rotate_motor_right(MAX_STEPS,RIGHT_BACKWARD,rpm_right);

        prev_cmd=ROT_RIGHT;
        
        break;
      }
      case L_TURN_F:
      {
        curve(movement_speed, TURN_RADIUS, rpm_right, rpm_left);

        enable_left_until_right_stops(); //Clears the required steps
        
        rotate_motor_left(0,LEFT_FORWARD,rpm_left); //Left wheels has reduced rpm (No steps required)
        rotate_motor_right(MAX_STEPS,RIGHT_FORWARD,rpm_right);  //Operating at the maximum allowed steps and rpm

        prev_cmd=L_TURN_F;
        
        break;
      }
      case L_TURN_B:
      {
        curve(movement_speed, TURN_RADIUS, rpm_right, rpm_left);

        enable_left_until_right_stops(); //Clears the required steps
        
        rotate_motor_left(0,LEFT_BACKWARD,rpm_left); //Left wheels has reduced rpm (No steps required)
        rotate_motor_right(MAX_STEPS,RIGHT_BACKWARD,rpm_right);  //Operating at the maximum allowed steps and rpm

        prev_cmd=L_TURN_B;
        
        break;
      }
      case R_TURN_F:
      {
        curve(movement_speed, TURN_RADIUS, rpm_left, rpm_right);

        enable_right_until_left_stops(); //Clears the required steps
        
        rotate_motor_left(MAX_STEPS,LEFT_FORWARD,rpm_left); //Operating at the maximum allowed steps and rpm
        rotate_motor_right(0,RIGHT_FORWARD,rpm_right); //Right wheels has reduced rpm (No steps required)

        prev_cmd=R_TURN_F;
        
        break;
      }
      case R_TURN_B:
      {
        curve(movement_speed, TURN_RADIUS, rpm_left, rpm_right);

        enable_right_until_left_stops(); //Clears the required steps
        
        rotate_motor_left(MAX_STEPS,LEFT_BACKWARD,rpm_left); //Operating at the maximum allowed steps and rpm
        rotate_motor_right(0,RIGHT_BACKWARD,rpm_right); //Right wheels has reduced rpm (No steps required)

        prev_cmd=R_TURN_B;
        
        break;
      }
      default: break;
    }
    //Serial.println("rpm_right= "+String(rpm_right)+" rpm_left= "+String(rpm_left)+" BUF_STEPS= "+String(BUF_STEPS));  //DEBUG
  }

 c->ctrl_cmd=INVALID; //Reset until a new command is received
}

//#############################################################DIFFERENTIAL PARAMETER############################################################

void differential_param(float wr, float ha, float min_rpm, float max_rpm)
{
  wheel_radius=wr;
  half_axle=ha;
  max_diff_rpm=max_rpm;
  min_diff_rpm=min_rpm; 
}

void straight(float forward_speed, float &rpm_1, float &rpm_2) //Function for forward/backward motion Units m/s
{
  float phi_eq=forward_speed/wheel_radius; //phi1 and phi2 are equal

  rpm_1=rad_per_sec_to_rpm(phi_eq);
  rpm_2=rpm_1;

  if(rpm_1>max_diff_rpm)
  {
    rpm_1=max_diff_rpm;
    rpm_2=max_diff_rpm;
    Serial.println("WARNING maximum RPM exceeded!");
  }
  else if(rpm_1<min_diff_rpm)
  {
    rpm_1=min_diff_rpm;
    rpm_2=min_diff_rpm;
    Serial.println("WARNING minimum RPM exceeded!");
  }
}

void curve(float forward_speed, float turn_radius, float &rpm_higher, float &rpm_lower)  //Forward/backward motion with turn Units m/s and m
{
  float arc_length=2*M_PI*(turn_angle/360.0)*turn_radius;

  float time_taken=arc_length/forward_speed;

  float angular_turn_rate=(turn_angle*(M_PI/180.0))/time_taken; //Unit rad/s

  float phi_higher=(forward_speed+(half_axle*angular_turn_rate))/wheel_radius;
  float phi_lower=(forward_speed-(half_axle*angular_turn_rate))/wheel_radius;

  rpm_higher=rad_per_sec_to_rpm(phi_higher);
  rpm_lower=rad_per_sec_to_rpm(phi_lower);

  float scale_ratio=0;
  
  if(rpm_higher>max_diff_rpm) //If the higher RPM exceeds
  {
    scale_ratio=(max_diff_rpm/rpm_higher); //Find the scaling ratio
    rpm_higher=max_diff_rpm; //Set to max rpm
    rpm_lower=scale_ratio*rpm_lower; //Scale the rpm of the lower rpm   
  }

  if(rpm_lower<min_diff_rpm)
  {
    //Set the same speed for both wheels
    rpm_higher=min_diff_rpm;
    rpm_lower=min_diff_rpm;
    Serial.println("ERROR scaling cannot be applied! RPM too low.");
  }
}

void turn(float turn_speed, float &rpm_pos, float &rpm_neg) //Function for turning only Units Degree/s
{
  float turn_speed_rad=turn_speed*(M_PI/180.0); //rad/s

  float phi_pos=(turn_speed_rad*half_axle)/wheel_radius;
  float phi_neg=phi_pos; //Not negative since other function doesn't take in negative RPM (Negative in name only)

  rpm_pos=rad_per_sec_to_rpm(phi_pos);
  rpm_neg=rad_per_sec_to_rpm(phi_neg);

  if(rpm_pos>max_diff_rpm) 
  {
    rpm_pos=max_diff_rpm;
    rpm_neg=max_diff_rpm;
    Serial.println("WARNING maximum RPM exceeded!");
  }
  else if(rpm_pos<min_diff_rpm)
  {
    rpm_pos=min_diff_rpm;
    rpm_neg=min_diff_rpm;
    Serial.println("WARNING minimum RPM exceeded!");
  }
}

float rad_per_sec_to_rpm(float omega) //Function to rad/s to RPM
{
  return (omega*60.0)/(2*M_PI);
}

int scaled_steps(float rpm_higher, float rpm_lower, int max_steps_allowed)
{
  float scale_steps_f=(rpm_lower/rpm_higher)*float(max_steps_allowed);
  int scale_steps_i=round(scale_steps_f);
  return scale_steps_i;
}

//#############################################################SERVO PAN AND TILT############################################################

void ctrl_pan_and_tilt(int pan_angle, int tilt_angle)  //Function to pan and tilt the camera mount
{
  static int prev_pan_angle,prev_tilt_angle;

  if(prev_pan_angle != pan_angle || prev_tilt_angle != tilt_angle) //Only execute the code when new angle is received
  {
    //Clip the pan angle
    if(pan_angle > MAX_PAN_ANGLE) pan_angle=MAX_PAN_ANGLE;
    else if(pan_angle < MIN_PAN_ANGLE) pan_angle=MIN_PAN_ANGLE;

    //Clip the tilt angle
    if(tilt_angle > MAX_TILT_ANGLE) tilt_angle=MAX_TILT_ANGLE;
    else if(tilt_angle < MIN_TILT_ANGLE) tilt_angle=MIN_TILT_ANGLE;

    //Pan angle calculation (Inversion beacuse of servo motor orientation)
    int pan_angle_inv=180-pan_angle;
    
    //Write the pan position to servo
    pan_servo.write(pan_angle_inv);

    //Tilt angle calculation
    int tilt_angle_left=tilt_angle;  //Tilt angle for the left is the same
    int tilt_angle_right=180-tilt_angle_left;  //For right the mirror of the left is used so both left and right can work together

    //Write the tilt angle for left and right (The slight delay between the two can cause tension but is minimal)
    tilt_servo_l.write(tilt_angle_left);
    tilt_servo_r.write(tilt_angle_right);

    //Store the previous position so the above code is not excuted all the time when place in the main loop
    prev_pan_angle=pan_angle;
    prev_tilt_angle=tilt_angle;
  }
}







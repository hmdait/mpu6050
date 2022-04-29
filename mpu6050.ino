

#include <Servo.h>
#include <Wire.h>
/////// moteur /-**/-/*-/*-/

Servo esc1;
Servo esc11;
Servo esc2;
Servo esc22;
   ////////////////////////// pid  /////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)


float pid_error_tempp, pid_error_tempr;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;

double k;

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

int throttle = 1100;
int esc_1, esc_11, esc_2, esc_22;



//Initialize the LCD library



void setup() {
   Wire.begin();                                                        //Start I2C as master
   Serial.begin(57600);                                               //Use only for debugging
   esc1.attach(4);
   esc11.attach(5);
   esc2.attach(6);
   esc22.attach(7);
                                                
 
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

                                               //Set the LCD cursor to position to position 0,0
 Serial.print("  MPU-6050 IMU");                                         //Print text to screen
             
                                            
  esc1.writeMicroseconds(2000);
  esc11.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc22.writeMicroseconds(2000);
  delay(3000);
  esc1.writeMicroseconds(1000);
  esc11.writeMicroseconds(1000);
  esc22.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  delay(2000);//Print text to screen

  delay(1500);                                                         //Delay 1.5 second to display the text
  
Serial.print("Calibrating gyro");                                       //Print text to screen
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    if(cal_int % 125 == 0)Serial.print(".");                              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset


 
  Serial.print("Pitch:");                                                 //Print text to screen
  Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.println("Roll :");                                                 //Print text to screen
  delay(1500);
  digitalWrite(13, LOW);                                               //All done, turn the LED off
  k=0;
  loop_timer = micros();                                               //Reset the loop timer
}

void loop(){
  
  //////////////// receev signal //////////////*************--------**********
  if(Serial.available() > 0){
     k=Serial.read();
        if(k > 5 ){  
        k=map(k,0,180,990,2000);
        throttle = k;}
        if(k==1){pid_roll_setpoint=10;}
        else if(k==2){pid_roll_setpoint=-10;}
        else if(k==3){pid_pitch_setpoint=10;}
        else if(k==4){pid_pitch_setpoint=-10;}
        else if(k==0){pid_roll_setpoint=0;
                      pid_pitch_setpoint=0;}        //Serial.println(int(Serial.read()));
        }
      
        
read_mpu_6050_data(); 
value();
pid();
motorcontrol();

  
     //Set the LCD cursor to position to position 0,1
  Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.print(esc_1); 
  Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.println(esc_2); //Print text to screen
                       //Print text to screen
 /*Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.print(throttle); 
   
   Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.print(esc_1); 
  
   Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.print(esc_11);  
       
     Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.print(esc_2);
  
     Serial.print("  ");                                            //Set the LCD cursor to position to position 0,1
  Serial.println(esc_22);*/
  
                            //Reset the loop timer
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}



void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}
void value(){
  
   read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050
 
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value
  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  
  
  }
 
void pid(){
  //Roll calculations
  pid_error_tempr = angle_roll_output - pid_roll_setpoint;                            // 
  pid_i_mem_roll += pid_i_gain_roll * pid_error_tempr;             
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;      //

  pid_output_roll = pid_p_gain_roll * pid_error_tempr + pid_i_mem_roll + pid_d_gain_roll * (pid_error_tempr - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_tempr;

  //Pitch calculations
  pid_error_tempp = angle_pitch_output - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_tempp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_tempp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_tempp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_tempp;

  
  }  
void motorcontrol(){

  if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
  
    esc_1 = throttle - pid_output_pitch + pid_output_roll ; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_11 = throttle + pid_output_pitch + pid_output_roll ; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_2 = throttle + pid_output_pitch - pid_output_roll ; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_22 = throttle - pid_output_pitch - pid_output_roll ; //Calculate the pulse for esc 4 (front-left - CW)

   
    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_11 < 1100) esc_11 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_22 < 1100) esc_22 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_11 > 2000)esc_11 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_22 > 2000)esc_22 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  
 /* else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_11 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_22 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }/***/
  esc1.writeMicroseconds(esc_1);
  esc11.writeMicroseconds(esc_2);
  esc2.writeMicroseconds(esc_1);
  esc22.writeMicroseconds(esc_1);
}
 
 
  

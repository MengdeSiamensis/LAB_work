
#include <micro_ros_arduino.h>
#include <ESP32Encoder.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/node.h>



#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

//Odom (publisher)
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray Array_msg;
rclc_executor_t executor_pub;

//Twist keyboard (subscriber)
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_timer_t timer;
rclc_executor_t executor_sub;


rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

//rcl_node_options_t node_ops = rcl_node_get_default_options();
//node_ops.domain_id = (size_t)(30);



//tester
const int tester = 12;

//motorA
const int outputA_A = 5;
const int outputB_A = 18;

const int InA_motorA = 23;
const int InB_motorA = 22;
const int PWM_motorA = 21;

//motorB
const int outputA_B = 15;
const int outputB_B = 19;

const int InA_motorB = 33;
const int InB_motorB = 25;
const int PWM_motorB = 26;

//motorC
const int outputA_C = 2;
const int outputB_C = 4;

const int InA_motorC = 13;
const int InB_motorC = 14;
const int PWM_motorC = 27;


static long First_encoder;
static long Second_encoder;
static long Third_encoder;
long deltaCount;
unsigned long last_time;
double test_time;

int M1A;
int M1B;
int PWM1;

double lastError;
static double cumError_A;
static double cumError_B;
static double cumError_C;
double rateError;
//double real_PID;

//find_distance
double dis;

double U1A;
double U2C; 
double U3B; 
  
double  U1A_convert; 
double  U2C_convert; 
double  U3B_convert; 

double count;

double PWM;
double volt_required;


static float x,y,rotation;


ESP32Encoder encoder;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

#define LED_PIN 13

//#define BUFFER_SIZE 10

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  
  x = msg->linear.x;
  y = msg->linear.y;
  rotation = msg->angular.z;

  //Serial.begin(115200);
}

void change_command_to_omega(double x,double y,double z,double First_encoder, double Second_encoder, double Third_encoder ,double deltaT){
      //2.07 and 0.35
      //2.05 and 0.20
      
      U1A = (1 / 0.075) * (x - (0.295 * z)); //face to b motor
      U2C = (1 / 0.075) * (-(x / 2.05) - (0.295 * z) - (0.866 * y)); //
      U3B = (1 / 0.075) * (-(x / 2.05) - (0.295 * z) + (0.866 * y)); //
  
      U1A_convert = U1A * 124.9;
      U2C_convert = U2C * 124.9;
      U3B_convert = U3B * 124.9;
      change_to_count(U1A, First_encoder, deltaT, "A");
      change_to_count(U3B, Second_encoder, deltaT, "B");
      change_to_count(U2C, Third_encoder, deltaT, "C");
    }

void change_to_count(double omega ,double encoder,double deltaT, String Which_motor){
    int delta_time = 10; // millisec
    int count_per_second = 1000; //from encoder
    int sec_to_millisec = 1000; //sec_to_millisec
    double Big_round = 4.6; //cm
    double small_round = 0.63; //cm
    double pi = 3.1416;

    if(Which_motor == "B"){
      count_per_second = 600;
    }
    
    count = (omega * delta_time * count_per_second * Big_round)/(sec_to_millisec * small_round * 2 * pi);
    PID(count, encoder, deltaT, Which_motor); 

    };

void PID(double Setpoint_count, double encoder, double elapsedTime, String Which_motor){
    double real_PID_A, real_PID_B, real_PID_C;
    float Kp = 0.2; //0.6
    float Ki = 1.5;  //0.45
    float Kd = 0;

    double error = Setpoint_count - encoder;    
    if(Which_motor == "A"){
      //Serial.println("in A");
      cumError_A += error * elapsedTime;
      real_PID_A = (Kp * error + Ki * cumError_A); // in VOLT
      motor(real_PID_A, 24, Which_motor);
    }
    else if(Which_motor == "B"){
      //Serial.println("in B");
      cumError_B += error * elapsedTime;
      real_PID_B = 1.666*(Kp * error + Ki * cumError_B); // in VOLT
      motor(real_PID_B, 24, Which_motor);
    }
    else{
      //Serial.println("in C");
      cumError_C += error * elapsedTime;
      real_PID_C = (Kp * error + Ki * cumError_C); // in VOLT
      motor(real_PID_C, 24, Which_motor);
    }

    
    
};

void motor(double volt, double maxVoltage, String Which_motor){
    if (Which_motor == "A"){
      M1A = InA_motorA;
      M1B = InB_motorA;
      PWM1 = PWM_motorA;
    }
    if (Which_motor == "B"){
      M1A = InA_motorB;
      M1B = InB_motorB;
      PWM1 = PWM_motorB;
    }
    if (Which_motor == "C"){
      M1A = InA_motorC;
      M1B = InB_motorC;
      PWM1 = PWM_motorC;
    }
  
    if (volt >= maxVoltage)
    {
      volt = maxVoltage;
    }
    else if (volt <= -maxVoltage)
    {
      volt = -maxVoltage;
    }

    if (volt >= 0)
    {
      double pwm = (volt / 24.0) * 255.0;
      digitalWrite(M1A, HIGH);
      digitalWrite(M1B, LOW);
      analogWrite(PWM1, pwm);
    }
    else if (volt <= 0)
    {
      double pwm = -(volt / 24.0) * 255.0;
      digitalWrite(M1A, LOW);
      digitalWrite(M1B, HIGH);
      analogWrite(PWM1, pwm);
    }
    else
    {
      //Serial.print("NONE");
      digitalWrite(M1A, LOW);
      digitalWrite(M1B, LOW);
      analogWrite(PWM1, 0);
    }
  
   }

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &Array_msg, NULL));
    
  }
}

void setup() {

  set_microros_transports();

  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachHalfQuad(outputA_A, outputB_A); // current use
  encoder2.attachHalfQuad(outputA_B, outputB_B); // no input
  encoder3.attachHalfQuad(outputA_C, outputB_C); // no input 

  analogWriteFrequency(25000);
  
//  ledcSetup(0, 20000, 8);
//
//  ledcAttachPin(PWM_motorA, 0);
//  ledcAttachPin(PWM_motorB, 0);
//  ledcAttachPin(PWM_motorC, 0);
  
  //motorA
  pinMode(outputA_A , INPUT);
  pinMode(outputB_A , INPUT);
  
  pinMode(InA_motorA , OUTPUT);
  pinMode(InB_motorA , OUTPUT);
  pinMode(PWM_motorA , OUTPUT);

  //motorB
  pinMode(outputA_B , INPUT);
  pinMode(outputB_B , INPUT);
  
  pinMode(InA_motorB , OUTPUT);
  pinMode(InB_motorB , OUTPUT);
  pinMode(PWM_motorB , OUTPUT);

  //motorC
  pinMode(outputA_C , INPUT);
  pinMode(outputB_C , INPUT);
  
  pinMode(InA_motorC , OUTPUT);
  pinMode(InB_motorC , OUTPUT);
  pinMode(PWM_motorC , OUTPUT);
  
  

  allocator = rcl_get_default_allocator();

  //rcl_init_options_init(&init_options, allocator);
  //rcl_init_options_set_domain_id(&init_options, 30);  
  

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  //rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "int64_array_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));


  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
//  Array_msg->data.data[0] = 20;
//  Array_msg->data.data[1] = 15;
//  Array_msg->data.data[2] = 10;

  static int32_t memory[100];
  Array_msg.data.capacity = 100;
  Array_msg.data.data = memory;
  Array_msg.data.size = 0;

  for (int32_t i = 0;i < 3 ; i++){
    Array_msg.data.data[i] = i;
    Array_msg.data.size++;
  }

//  int64_t buffer[BUFFER_SIZE] = {};
//  Array_msg.data.capacity = 100;
//  Array_msg.data.size = 0;
//  Array_msg.data.data = 10;
//
//  Array_msg.layout.dim.capacity = 100;
//  Array_msg.layout.dim.size = 100;
//  Array_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(Array_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
//
//  for(size_t i = 0; i < Array_msg.layout.dim.capacity; i++){
//    Array_msg.layout.dim.data[i].label.capacity = 100;
//    Array_msg.layout.dim.data[i].label.size = 100;
//    Array_msg.layout.dim.data[i].label.data = (char*) malloc(Array_msg.layout.dim.data[i].label.capacity * sizeof(char));
//}
//
////  Array_msg->data.data[0] = 20;
////  Array_msg->data.data[1] = 15;
////  Array_msg->data.data[2] = 10;
  
}

void loop() {
  
  if (millis() - last_time >= 10)
    {
     last_time = millis();
     First_encoder = encoder.getCount();
     encoder.clearCount();
     Second_encoder = -1*encoder2.getCount();
     encoder2.clearCount();
     Third_encoder = encoder3.getCount();
     encoder3.clearCount();
     change_command_to_omega( x , y , rotation ,First_encoder ,Second_encoder ,Third_encoder, 0.01);
    }
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
}

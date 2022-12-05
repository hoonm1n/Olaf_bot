#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H
#include <pigpiod_if2.h>

#define motor1_DIR 19
#define motor1_PWM 26
#define motor1_ENA 27
#define motor1_ENB 17

#define motor2_DIR 6
#define motor2_PWM 13
#define motor2_ENA 23
#define motor2_ENB 24

#define PI 3.141592

//Text_Input
void Text_Input(void);
int PWM_range;
int PWM_frequency;
int PWM_limit;
double Control_cycle;
int Acceleration_ratio;
double Wheel_radius;
double Robot_radius;
int Encoder_resolution;
double Wheel_round;
double Robot_round;

//Motor_Setup
int Motor_Setup(void);
int pinum;
int current_PWM1;
int current_PWM2;
bool current_Direction1;
bool current_Direction2;
int acceleration;

//Interrupt_Setting
void Interrupt_Setiing(void);
volatile int tt_EncoderCounter1A=0;
volatile int tt_EncoderCounter1B=0;
volatile int tt_EncoderCounter2A=0;
volatile int tt_EncoderCounter2B=0;
volatile int EncoderCounter1;
volatile int EncoderCounter2;
volatile int EncoderCounter1A;
volatile int EncoderCounter1B;
volatile int EncoderCounter2A;
volatile int EncoderCounter2B;
volatile int EncoderSpeedCounter1;
volatile int EncoderSpeedCounter2;
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick);
int Motor1_Encoder_Sum();
int Motor2_Encoder_Sum();
void Init_Encoder(void);
////////////////////////////////////////////////////////////////////////////////////////////////////////// 
void Initialize(void);

//Motor_Controller
void Motor_Controller(int motor_num, bool direction, int pwm);
void Accel_Controller(int motor_num, bool direction, int desired_pwm);

//Example
bool switch_direction;
int Theta_Distance_Flag;
void Switch_Turn_Example(int PWM1, int PWM2);
void Theta_Turn(double Theta, int PWM);
void Distance_Go(double Distance, int PWM);
void Theta_Distance(double Theta, int Turn_PWM, double Distance, int Go_PWM);

//Utiliy
int Limit_Function(int pwm);
double RPM_Value1;
double RPM_Value2;
float w1;
float w2;
float cur_velL;
float cur_velR;
void RPM_Calculator();
void Motor_View();


//topic

float goal_velL;
float goal_velR;
double kPl=40;   //40
double kIl=0.1;       //1
double kDl=10;      //10
double Imaxl=0;
double Dmaxl=10;
int pid_PWMl=0;


double kPr=40;
double kIr=0.1;
double kDr=15;
double Imaxr=0;
double Dmaxr=10;
int pid_PWMr=0;
int ref_PWMr=0;

double errorl_avg=0;
double errorl=0;
double error_dotl=0;
double error_suml=0;
double error_bfl=0;
double prevl=0;
double outputl=0;

double errorr_avg=0;
double errorr=0;
double error_dotr=0;
double error_sumr=0;
double error_bfr=0;
double prevr=0;
double outputr=0;
#endif // MOTOR_NODE_H
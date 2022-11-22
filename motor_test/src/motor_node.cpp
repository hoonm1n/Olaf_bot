/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <ros/ros.h>
#include <motor_test/motor_node.h>
#include <fstream>
#include <motor_test/To_odom.h>
#include <motor_test/PID.h>
#include <cmath>
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

void Text_Input(void)
{
  int i = 0;
  std::size_t found;
  std::ifstream inFile;
  inFile.open("/home/ubuntu/catkin_ws/src/git_olaf/motor_test/motor_input.txt");
  for(std::string line; std::getline(inFile,line);)
  {
      found=line.find("=");

      switch(i)
      {
      case 0: PWM_range = atof(line.substr(found+2).c_str()); break;
      case 1: PWM_frequency = atof(line.substr(found+2).c_str()); break;
      case 2: PWM_limit = atof(line.substr(found+2).c_str()); break;
      case 3: Control_cycle = atof(line.substr(found+2).c_str()); break;
      case 4: Acceleration_ratio = atof(line.substr(found+2).c_str()); break;
      case 5: Wheel_radius = atof(line.substr(found+2).c_str()); break;
      case 6: Robot_radius = atof(line.substr(found+2).c_str()); break;
      case 7: Encoder_resolution = atof(line.substr(found+2).c_str()); break;
          //case :  = atof(line.substr(found+2).c_str()); break;
      }
      i +=1;
  }
  inFile.close();
}
int Motor_Setup(void)
{
  pinum=pigpio_start(NULL, NULL);

  if(pinum<0)
  {
    ROS_INFO("Setup failed");
    ROS_INFO("pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_DIR, PI_OUTPUT);
  set_mode(pinum, motor2_DIR, PI_OUTPUT);
  set_mode(pinum, motor1_PWM, PI_OUTPUT);
  set_mode(pinum, motor2_PWM, PI_OUTPUT);
  set_mode(pinum, motor1_ENA, PI_INPUT);
  set_mode(pinum, motor1_ENB, PI_INPUT);
  set_mode(pinum, motor2_ENA, PI_INPUT);
  set_mode(pinum, motor2_ENB, PI_INPUT);

  gpio_write(pinum, motor1_DIR, PI_LOW);
  gpio_write(pinum, motor2_DIR, PI_LOW);

  set_PWM_range(pinum, motor1_PWM, PWM_range);
  set_PWM_range(pinum, motor2_PWM, PWM_range);
  set_PWM_frequency(pinum, motor1_PWM, PWM_frequency);
  set_PWM_frequency(pinum, motor2_PWM, PWM_frequency);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);
  set_PWM_dutycycle(pinum, motor1_PWM, 0);

  set_pull_up_down(pinum, motor1_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_ENB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_ENB, PI_PUD_DOWN);

  current_PWM1 = 0;
  current_PWM2 = 0;

  current_Direction1 = true;
  current_Direction2 = true;

  acceleration = PWM_limit/(Acceleration_ratio);

  ROS_INFO("Setup Fin");
  return 0;
}
void Interrupt_Setting(void)
{
    callback(pinum, motor1_ENA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_ENB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_ENA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_ENB, EITHER_EDGE, Interrupt2B);
}
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1A ++;
  else EncoderCounter1A --;
  EncoderSpeedCounter1 ++;
}
void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor1_DIR) == true)EncoderCounter1B ++;
  else EncoderCounter1B --;
  EncoderSpeedCounter1 ++;
}
void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2A --;
  else EncoderCounter2A ++;
  EncoderSpeedCounter2 ++;
}
void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  if(gpio_read(pinum, motor2_DIR) == true)EncoderCounter2B --;
  else EncoderCounter2B ++;
  EncoderSpeedCounter2 ++;
}
int Motor1_Encoder_Sum()
{
  EncoderCounter1 = EncoderCounter1A + EncoderCounter1B;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  return EncoderCounter1;
}
int Motor2_Encoder_Sum()
{
  EncoderCounter2 = EncoderCounter2A + EncoderCounter2B;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
  return EncoderCounter2;
}
void Init_Encoder(void)
{
  EncoderCounter1 = 0;
  EncoderCounter2 = 0;
  EncoderCounter1A = 0;
  EncoderCounter1B = 0;
  EncoderCounter2A = 0;
  EncoderCounter2B = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{
  Text_Input();
  Motor_Setup();
  Init_Encoder();
  Interrupt_Setting();

  Wheel_round = 2*PI*Wheel_radius;
  Robot_round = 2*PI*Robot_radius;

  switch_direction = true;
  Theta_Distance_Flag = 0;

  ROS_INFO("PWM_range %d", PWM_range);
  ROS_INFO("PWM_frequency %d", PWM_frequency);
  ROS_INFO("PWM_limit %d", PWM_limit);
  ROS_INFO("Control_cycle %f", Control_cycle);
  ROS_INFO("Acceleration_ratio %d", Acceleration_ratio);
  ROS_INFO("Initialize Complete");

  printf("\033[2J");
}
/*
typedef struct pid_param
{
  double kP=0;
  double kI=0;
  double kD=0;
  double Imax=0;
  double Dmax=10;
} pid_param;

typedef struct pid
{
  double prev=0;
  double p_out=0;
  double integrator=0;
  double derivative=0;
  double last_input=0;
  double lastderivative=0;

  double output=0;
} pid;
*/
void PidContoller_L(float goal, float curr, float dt, double error_rat)
{
  
  //std::cout << EncoderCounter1 << std::endl;
  float up,ui,ud, outputl=0;
  double filter = 7.9577e-3;
  pid_PWMl=0;
  errorl=goal-curr;
  
  if (fabs(errorl) < error_rat) {errorl = 0;}
  else if(fabs(errorl)< (error_rat*3)) 
  { errorl_avg=(errorl+errorl_avg)/3;errorl=errorl_avg;
  //std::cout<<"errorl"<<errorl<<std::endl;
  //std::cout<<"errorl_avg"<<errorl_avg<<std::endl;
  }
 
  //ROS_INFO("errorl : %f",errorl);
  error_dotl = (errorl-error_bfl)*(dt / (filter + dt));
  error_suml +=errorl*dt;
  
  up = kPl*errorl;
  ud = kDl*error_dotl;
  ui = kIl*error_suml;
  
   // ROS_INFO("up: %f" , up);
  //  ROS_INFO("ui: %f" , ui);
  //  ROS_INFO("ud: %f" , ud);
  
  
  outputl =up+ ui +ud;
  error_bfl=errorl;
  outputl=outputl+prevl;
  //std::cout<<prevl<<std::endl;
  
  
    pid_PWMl = fabs(outputl);
    //ROS_INFO("pid_PWMl: %d" , pid_PWMl);
    //std::cout<<pid_PWMl<<std::endl;
    pid_PWMl=constrain(pid_PWMl,-150,150);
    if(outputl <0){
	Motor_Controller(1,true,pid_PWMl);    
	    
    }
    else{
    	Motor_Controller(1,false,pid_PWMl);
    }
    //prevl=constrain(outputl,-goal*5,goal*5);
    prevl = outputl;
}

void PidContoller_R(float goal, float curr, float dt, double error_rat)
{
  
  std::cout << EncoderCounter2 << std::endl;
  float up,ui,ud,outputr=0;
  pid_PWMr=0;
  errorr=goal-curr;
  if (fabs(errorr) < error_rat) {errorr = 0;}
  else if(fabs(errorr)< (error_rat*10)) 
  {
  std::cout<<"errorr"<<errorr<<std::endl; 
  errorr=(errorr+errorr_avg)/3;errorr_avg=errorr;
  std::cout<<"errorr_avg"<<errorr_avg<<std::endl;
  }
  ROS_INFO("errorr : %f",errorr);
  double filter = 15.9155e-3;
  error_dotr = (errorr-error_bfr)*(dt / (filter + dt)) ;
  error_sumr +=errorr*dt;
  
  up = kPr*errorr;
  ui = kIr*error_sumr;
  ud = kDr*error_dotr;
  
  
    ROS_INFO("up: %f" , up);
    ROS_INFO("ui: %f" , ui);
    ROS_INFO("ud: %f" , ud);
    
  outputr =up + ui +ud;
  error_bfr=errorr;
  outputr = outputr+prevr;
 
    
    pid_PWMr = fabs(outputr);
    pid_PWMr=constrain(pid_PWMr,-150,150);
    ROS_INFO("pid_PWMr: %d" , pid_PWMr);
    if(outputr <0){
		Motor_Controller(2,false,pid_PWMr);    
	    
    }
    else{
    		Motor_Controller(2,true,pid_PWMr);
    }
    //prevr=constrain(outputr,-goal*5,goal*5);
    prevr = outputr;
}


/*
double PidContoller(double goal, double curr, double dt, pid *pid_data, pid_param *pid_paramdata, int error_rat)
{
   ROS_INFO(" goal : %f, curr: %f, dt: %f", goal,curr,dt);
   double error = goal - curr;
   ROS_INFO(" error : %f", error);
  if (fabs(error) < error_rat)
    error = 0;

  pid_data->p_out = pid_paramdata->kP * error;
  double p_data = pid_data->p_out ;
   ROS_INFO(" p_data : %f", p_data);
  pid_data->integrator += (error * pid_paramdata->kI) * dt;
  pid_data->integrator = constrain(pid_data->integrator, -pid_paramdata->Imax, pid_paramdata->Imax);
  double i_data = pid_data->integrator;
   ROS_INFO(" i_data : %f", i_data);

  double filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  pid_data->derivative = (goal - pid_data->last_input) / dt;
  pid_data->derivative = pid_data->lastderivative + (dt / (filter + dt)) * (pid_data->derivative - pid_data->lastderivative);
  pid_data->last_input = goal;
  pid_data->lastderivative = pid_data->derivative;
  double d_data = pid_paramdata->kD * pid_data->derivative;
  d_data = constrain(d_data, -pid_paramdata->Dmax, pid_paramdata->Dmax);

  double output = p_data + i_data + d_data;
  pid_data->output = output;

  return pid_data->output;
}

void PID_VELL(){
	pid pidL;
  	pid_param pidL_param;
  
    double output_L=PidContoller(goal_velL, cur_velL, 0.1, &pidL, &pidL_param, 0);
    ROS_INFO("output_L: %f" , output_L);
    int pid_PWM = fabs(output_L *100/30);
    ROS_INFO("pid_PWM: %d" , pid_PWM);
    if(output_L <0){
    	//gpio_write(pinum,motor1_DIR, PI_HIGH);
    	//current_Direction1 = false;
	Motor_Controller(1,false,pid_PWM);    
	    
    }
    else{
    	//gpio_write(pinum,motor1_DIR, PI_LOW);
    	//current_Direction1 = true;
    	
    	Motor_Controller(1,true,pid_PWM);
    }
    
    //if(pid_PWM >255) pid_PWM =255;
    //set_PWM_dutycycle(pinum,motor1_PWM,pid_PWM);
    //current_PWM1 = pid_PWM;
    
}
    */

void Motor_Controller(int motor_num, bool direction, int pwm)
{
  int local_PWM = Limit_Function(pwm);

  if(motor_num == 1)
  {
    if(direction == true)
    {
      gpio_write(pinum, motor1_DIR, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = true;
    }
    else if(direction == false)
    {
      gpio_write(pinum, motor1_DIR, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_PWM, local_PWM);
      current_PWM1 = local_PWM;
      current_Direction1 = false;
    }
  }

  else if(motor_num == 2)
  {
   if(direction == true)
   {
     gpio_write(pinum, motor2_DIR, PI_LOW);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = true;
   }
   else if(direction == false)
   {
     gpio_write(pinum, motor2_DIR, PI_HIGH);
     set_PWM_dutycycle(pinum, motor2_PWM, local_PWM);
     current_PWM2 = local_PWM;
     current_Direction2 = false;
   }
  }
}
void Theta_Turn(float Theta, int PWM)
{
  float local_encoder;
  int local_PWM = Limit_Function(PWM);
  if(Theta_Distance_Flag == 1)
  {
      Init_Encoder();
      Theta_Distance_Flag = 2;
  }
  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
  if(Theta > 0)
  {
    local_encoder = (Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, false, local_PWM);
    Motor_Controller(2, false, local_PWM);
    //Accel_Controller(1, false, local_PWM);
    //Accel_Controller(2, false, local_PWM);
  }
  else
  {
    local_encoder = -(Encoder_resolution*4/360)*(Robot_round/Wheel_round)*Theta;
    Motor_Controller(1, true, local_PWM);
    Motor_Controller(2, true, local_PWM);
    //Accel_Controller(1, true, local_PWM);
    //Accel_Controller(2, true, local_PWM);
  }

  if(EncoderCounter1 > local_encoder)
  {
    Init_Encoder();
    Motor_Controller(1, true, 0);
    Motor_Controller(2, true, 0);
    Theta_Distance_Flag = 3;
  }
}

int Limit_Function(int pwm)
{
  int output;
  if (pwm > PWM_limit*2)
  {
    output = PWM_limit;
    ROS_WARN("PWM too fast!!!");
  }
  else if(pwm > PWM_limit)output = PWM_limit;
  else if(pwm < 0)
  {
	output = 0;
    ROS_WARN("trash value!!!");
  }
  else output = pwm;
  return output;
}
void RPM_Calculator(){
  
  RPM_Value1 = (EncoderSpeedCounter1*(60*Control_cycle))/(Encoder_resolution*4);
  w1=(RPM_Value1*2*PI/60);
  //cur_velL=Wheel_radius*w1/100;
  cur_velL = (EncoderCounter1) * (2 * 3.14 * Wheel_radius) * Control_cycle / (Encoder_resolution * 4) /100;
  EncoderSpeedCounter1 = 0;
  RPM_Value2 = (EncoderSpeedCounter2*(60*Control_cycle))/(Encoder_resolution*4);
  w2=(RPM_Value2*2*PI/60);
  //cur_velR=Wheel_radius*w2;
  cur_velR = (EncoderCounter2) * (2 * 3.14 * Wheel_radius) * Control_cycle / (Encoder_resolution * 4) /100;
  EncoderSpeedCounter2 = 0;
  //std::cout << cur_velL << cur_velR << std::endl;
  
  //EncoderCounter1A = 0;
  //EncoderCounter1B = 0;
  //EncoderCounter2A = 0;
  //EncoderCounter2B = 0;
}

void Motor_View()
{

  Motor1_Encoder_Sum();
  Motor2_Encoder_Sum();
	RPM_Calculator();
	printf("\033[2J");
	printf("\033[1;1H");
	printf("Encoder1A : %5d  ||  Encoder2A : %5d\n", EncoderCounter1A, EncoderCounter2A);
	printf("Encoder1B : %5d  ||  Encoder2B : %5d\n", EncoderCounter1B, EncoderCounter2B);
	printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", RPM_Value1, RPM_Value2);
	printf("W1   : %10.0f    ||  W2 :%10.0f\n",w1,w2);
  	printf("velL :     %f    ||  velR: %f\n",cur_velL,cur_velR);    //출력되라~
	printf("goalL:     %f    ||  goalR: %f\n\n",goal_velL,goal_velR);
	printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_PWM1, current_PWM2);
	printf("DIR1 :%10.0d     ||  DIR2 :%10.0d\n", current_Direction1, current_Direction2);
	printf("Acc  :%10.0d\n", acceleration);
	printf("\n");
}

void goalvelCallback(const motor_test::To_odom::ConstPtr& msg){
	goal_velL = msg->velL;
	goal_velR = msg->velR;
	//kPl=msg->pl;
	//kIl=msg->il;
	//kDl=msg->dl;
	//kPr=msg->pr;
	//kPr=msg->pr;
	//kIr=msg->ir;
	//kDr=msg->dr;
}
/*
void pidCallback(const motor_test::PID::ConstPtr& msg){
	kPl=msg->pl;   
	kIl=msg->il;
	kDl=msg->dl;
	
	//30
	//5
	//10

	kPr=msg->pr;
	kIr=msg->ir;
	kDr=msg->dr;
	//40
	//5
	//5
	
	
}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;
  Initialize();
  ros::Rate loop_rate(Control_cycle);
  ros::Publisher odom_node = nh.advertise<motor_test::To_odom>("wheel_vel",10);
  motor_test::To_odom to_odom;
  ros::Subscriber sub = nh.subscribe("goalvel",10,goalvelCallback);
  //ros::Subscriber sub_pid = nh.subscribe("pid_vel",10,pidCallback);
  //pid pidL;
  while(ros::ok())
  {
    //Motor_Controller(1, true, 50);
    //Motor_Controller(2, true, 50);
    //Accel_Controller(1, true, 100);
    //Accel_Controller(2, true, 100);
    //Switch_Turn_Example(100, 100);
    //Theta_Distance(180,100,30,110);
    Motor_View();
    
    to_odom.velL=cur_velL;
    to_odom.velR=cur_velR;
    odom_node.publish(to_odom);
    
    //RPM_Calculator();
    PidContoller_L(goal_velL, cur_velL, 0.1,  0.01);
    PidContoller_R(goal_velR, cur_velR, 0.1,  0.01);
    ROS_INFO("cur %f ,%f",cur_velL,cur_velR);
    ROS_INFO("goal %f ,%f",goal_velL,goal_velR);
    ros::spinOnce();
    loop_rate.sleep();
  }
  Motor_Controller(1, true, 0);
  Motor_Controller(2, true, 0);
  return 0;
}

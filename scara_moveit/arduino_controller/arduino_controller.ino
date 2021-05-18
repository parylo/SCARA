#include <AccelStepper.h>  
#include <MultiStepper.h>
#include <ros.h>
#include <rospy_tutorials/Floats.h>

//Link 1 actuator pinout
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19 //HOMING LIMIT SWITCH

//Link 2 actuator pinout
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14 
#define Y_MAX_PIN          15 //HOMING LIMIT SWITCH

//Link 3 actuator pinout
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3 //HOMING LIMIT SWITCH
#define X_MAX_PIN           2 

//Others
#define JOINT1_STEPS_PER_UNIT 320      //Steps/mm
#define JOINT2_STEPS_PER_UNIT 17.78    //Steps/degree
#define JOINT3_STEPS_PER_UNIT 17.78    //Steps/degree

long g_jointsPositionSteps[3];
bool g_jointsActive = false;

//AccelStepper setup
AccelStepper joint1Actuator(1, Z_STEP_PIN, Z_DIR_PIN); //1 = Easy Driver interface
AccelStepper joint2Actuator(1, Y_STEP_PIN, Y_DIR_PIN);  
AccelStepper joint3Actuator(1, X_STEP_PIN, X_DIR_PIN);
MultiStepper jointsActuators;

//ROS
ros::NodeHandle nh;
//std_msgs::Int16 msg;

//Function invert logic of joint actuator (stepper motor) enable pin 
void enableJointActuator(AccelStepper & t_jointActuator, int t_enablePin) {
  t_jointActuator.setEnablePin(t_enablePin);
  t_jointActuator.setPinsInverted(true, false, true);
  t_jointActuator.enableOutputs();
}

//Function perform link homing
void homeLink(AccelStepper & t_jointActuator, int t_limitSwitchPin, bool t_homeToMin, float t_stepUnitConversion, long t_homeOffset = 0) {
  long m_homingSpeed = t_stepUnitConversion * 15;
  long m_homingAcceleration = t_stepUnitConversion * 60;
  
  //Speed and acceleration depends on step/unit value of given stepper motor   
  t_jointActuator.setMaxSpeed(m_homingSpeed);     
  t_jointActuator.setAcceleration(m_homingAcceleration);  
    
  if (t_homeToMin) {
    t_jointActuator.move(-100000);
  } else {
    t_jointActuator.move(100000);
  }
     
  while (digitalRead(t_limitSwitchPin)) {
    t_jointActuator.run();
  }
   
  if (t_homeOffset) {
    t_jointActuator.move(t_homeOffset);
    t_jointActuator.runToPosition();
    t_jointActuator.setCurrentPosition(0);
  }
}

//Function converts radians to steps for actuator
int positionToSteps(double t_jointPosition, float t_stepUnitConversion, bool t_slider = false) {
  int m_steps;
  
  if (t_slider) {
    m_steps = (int)(t_stepUnitConversion * t_jointPosition);
  }
  else {
    m_steps = (int)(t_stepUnitConversion * t_jointPosition * 57.2958);
  }
  
  return m_steps;
}

//Joint states subscriber call back function
void jointStatesCallBack(const rospy_tutorials::Floats& t_cmdMsg){
  g_jointsActive = true;
  
  g_jointsPositionSteps[0] = positionToSteps(t_cmdMsg.data[0], JOINT1_STEPS_PER_UNIT, true);
  g_jointsPositionSteps[1] = positionToSteps(t_cmdMsg.data[1], JOINT2_STEPS_PER_UNIT);
  g_jointsPositionSteps[2] = positionToSteps(t_cmdMsg.data[2], JOINT3_STEPS_PER_UNIT);
}


//Subcribers and publishers declaration
ros::Subscriber<rospy_tutorials::Floats> jointStateSubscriber("/joints_to_aurdino", jointStatesCallBack);
//ros::Publisher stepsPublisher("joint_steps_feedback",&msg);

void setup() {
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(jointStateSubscriber);
  //nh.advertise(stepsPublisher);
   
  //Configure limit switches pins
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);

  //Enable stepper motors
  enableJointActuator(joint1Actuator, Z_ENABLE_PIN);
  enableJointActuator(joint2Actuator, Y_ENABLE_PIN);
  enableJointActuator(joint3Actuator, X_ENABLE_PIN);
   
  delay(5);  //Delay for initialization of EasyDriver

  //Home stepper motors
  //homeLink(joint1Actuator, Z_MAX_PIN, false, JOINT1_STEPS_PER_UNIT, -10000);
  homeLink(joint2Actuator, Y_MAX_PIN, false, JOINT2_STEPS_PER_UNIT, -1765); 
  homeLink(joint3Actuator, X_MIN_PIN, true, JOINT3_STEPS_PER_UNIT, 2600);


  //Setup joint actuators speed and acceleration 
  joint1Actuator.setMaxSpeed(8000.0);    
  joint1Actuator.setAcceleration(4000.0);  
  joint2Actuator.setMaxSpeed(500.0);    
  joint2Actuator.setAcceleration(500.0);    
  joint3Actuator.setMaxSpeed(500.0);      
  joint3Actuator.setAcceleration(500.0); 

  //Add joint actuators to multistepper
  jointsActuators.addStepper(joint1Actuator);
  jointsActuators.addStepper(joint2Actuator);
  jointsActuators.addStepper(joint3Actuator);
}


void loop() {

  if (g_jointsActive){
        
    //Publish position at given joint for debug purpose
    //msg.data =g_jointsPositionSteps[1];
    //stepsPublisher.publish(&msg);
      
    jointsActuators.moveTo(g_jointsPositionSteps);
    nh.spinOnce();
    jointsActuators.runSpeedToPosition();
    //delay(1);
  }

  g_jointsActive = false;
  nh.spinOnce();
  //delay(1);
  
}

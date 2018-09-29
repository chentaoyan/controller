/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef COCKROACH_H
#define COCKROACH_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator and sensor*/
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the differential range and bearing actuator and sensor*/
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public argos::CCI_Controller
{

public:
  struct Communication
  {
      size_t NoRobot_InRange;
      size_t NoRobot_TooNear;
      argos::CRadians Angle;
      void Init(argos::TConfigurationNode& t_node);
  };
public:
  // ------------ Sub-Routine for Natural Behaviors ------------ //
  void Natural_Behavior_Central();
  void Natural_Behavior_Wall(argos::CRadians);
  void Natural_Behavior_Stop();
  void Natural_Behavior_Move();
  void Natural_Behavior_Initiation();
  void Natural_Behavior_WallFollow(argos::CRadians);
  void Natural_Behavior_WallExit();
  // ------------ Sub-Routine for Responses to Electrical Stimulation ------------ //
  void Stimulated_Behavior_Forward();
  void Stimulated_Behavior_TurnLeft();
  void Stimulated_Behavior_TurnRight();
  void Stimulated_Behaviour(char, float);
  void Stimulated_Behavior_ForwardthenTurn(char);
  void Stimulated_Behavior_TurnthenForward(char);
  void Stimulated_Randomly();
  // ------------ Sub-Routine for Control Part ------------ //
  void Control_WallDeparture(argos::CRadians);
  void Control_MotionTrigger();
  void Control_LevyWalk();
  void Control_Interrupt();
  void Control_RobotAvoidance();
  void Control_RandomWalk();
  void Control_BorzkutRandom();
  void Control_PureDiffusion();
  // ------------ Sub-Routine for Assigning velocity to Footbot ------------ //
  void Robot_Movement();
  // ------------ Sub-Routine for Sensors ------------ //
  float    Sensor_Proximity_Distance();
  argos::CRadians Sensor_Proximity_Angle();
  void     Sensor_Positioning();
  void     Sensor_Velocity();
  void     Sensor_RangeandBearing_Transmit();
  void     Sensor_RangeandBearing_Receive();
  void     Sensor_Camera(); 
  // ------------ Sub-Routine for Logging File ------------ //
  void File_LogFile();
  void File_OutPut();
   // ------------ Sub-Routine for all distributions ------------ //
  double Distribution_Uniform();
  double Distribution_Exponential(float);
  double Distribution_Normal(float,float );
  double Distribution_Log_Normal(float, float );

  /* Class constructor. */
  CFootBotDiffusion();

  /* Class destructor. */
  virtual ~CFootBotDiffusion() {}

  /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
  virtual void Init(argos::TConfigurationNode &t_node);

  /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
  virtual void ControlStep();

  /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
  virtual void Reset() {}

  /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,`   
    * so the function could have been omitted. It's here just for
    * completeness.
    */
  virtual void Destroy() {}


private:
  /* Pointer to the differential steering actuator */
  argos::CCI_DifferentialSteeringActuator *m_pcWheels;
  /* Pointer to the foot-bot proximity sensor */
  argos::CCI_FootBotProximitySensor *m_pcProximity;
  argos::CCI_PositioningSensor *m_pcPosSens;
  argos::CCI_DifferentialSteeringSensor *m_pcSteering;
    /* Pointer to the foot-bot range and bearing sensor */
  argos::CCI_RangeAndBearingActuator *m_pcRABA;
  argos::CCI_RangeAndBearingSensor *m_pcRABS;
  /* Pointer to the foot-bot light sensor */
  argos::CCI_FootBotLightSensor* m_pcLight;
  /* Pointer to the LEDs actuator */
  argos::CCI_LEDsActuator* m_pcLEDs;
  /* Pointer to the omnidirectional camera sensor */
  argos::CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;

  /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section.
    */
  /* Wheel speed. */
  argos::Real m_fWheelVelocity;

  /*Angular velocity of the robot*/
  argos::Real m_fwheelAngularVelocity;
  argos::Real leftWheel_velocity;
  argos::Real rightWheel_velocity;
  argos::Real covered_distance;

  /* Angle tolerance range to go straight.
    * It is set to [-alpha,alpha]. */
  argos::CVector3 m_positioning;
  argos::CQuaternion m_rotation;
  argos::CRadians cZAngle, cYAngle, cXAngle;

  /* These variables are put in Private to prevent other Robot from accessing */
  //  ------------ Status Variable  ------------  //
  // Note 1: The "Status" variable is used to decide cockroach's motion
  // 'D' => Deciding behavior
  // 'C' => Moving in the central
  // 'S' => Stopping
  // 'W' => Following the wall
  // 'E' => Exiting the wall
  // 'L' => Stimulated left turn 
  // 'R' => Stimulated right turn
  // 'F' => Stimulated forward run
  // 'Z' => Stimulated forward then turn
  // 'T' => Stimulated turn then forward
  char Status   = 'D'; 
  //  ------------ Robot Variable  ------------  //
  float Robot_Speed         = 0;    // Speed of robot cm/s
  int   Robot_IsTurn        = 0;    // 1 - Robot turns and vice versa
  int   Robot_TurnDirection = 0;    // 0 - Left turn & 1 - Right turn
  Communication Robot_Communication;
  //  ------------ Temporal Variable  ------------  //
  // Note 1: All variables in second unit
  // Note 2: Sampling time is chosen based on cockroach reacts to sensory information 40ms
  float Time_Natural_Stop       = 0;  // Duration roach stops 
  float Time_Natural_Forward    = 0;  // Duration roach moves forward in central behaviour
  float Time_Natural_Turn       = 0;  // Duration roach turns in central behaviour
  float Time_Natural_Follow     = 0;  // Duration roach follows the wall
  float Time_Natural_Exit       = 0;  // Duration roach exits the wall by going straight after turning
  float Time_Natural_Decision   = 0;  // The moment of initiation of natural behaviour
  float Time_Stimulate_Turn     = 0;
  float Time_Stimulate_Forward  = 3;  // The forward motion is effective after 3s as observation
  float Time_Stimulate_Decision = 0;
  float Time_Control            = 0;
  float Time_Elapsed            = 0;
  float Time_Log                = 0;
  //  ------------ Natural Behaviour Variable  ------------  //
  float Natural_MoveSpeed       = 0;
  float Natural_MoveLength      = 0;
  float Natural_TurnSpeed       = 0;
  float Natural_TurnAngle       = 0;
  float Natural_ExitAngle       = 0;
  int   Natural_TurnDirection   = 0; 
  bool  Natural_IsInitiation    = true;
  //  ------------ Response to Electrical Stimulation Variable  ------------  //
  float Stimulate_TurnSpeed      = 0;
  float Stimulate_MoveSpeed      = 0;
  int   Stimulate_IsStimulated   = 0;
  float Stimulate_Angle          = 0;    // This angle is used to input in the stimulation function
  char  Stimulate_Type           = 'L';  // This variable is used to input in the stimulation function
  char  Stimulate_Size           = 'L';  // This variable is used to input in the simulation function: Left or Right   
  // ------------ Control Variable  ------------  //
  int   Control_Random_Direction   = 0;
  float Control_Random_Angle       = 0;
  float Control_Accumulator        = 0;
  float Control_Alpha              = 0;
  float Control_IsInterrupt        = 0;
  char  Control_InterruptStatus    = 'N';
  float Control_PreviousX          = 0;
  float Control_PreviousY          = 0;
  float Control_Status             = 'L';
  // Notes: The last variable is used to check which the interrupt is:
  // 'Z': 1st - Lost communication with other robots
  // 'N': 2nd - There are too near robots
  // 'S': 3rd - It is stopping
  // 'W': 4th - Is is near the wall
  // Notes: The number indicates prioprity level of interrupt
};
float **File_ReadFileFromTxt();
#endif
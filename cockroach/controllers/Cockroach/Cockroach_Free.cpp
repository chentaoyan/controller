/* Include the controller definition */
#include "Cockroach.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <cstdlib>
#include <ctime>
#include <iostream>
/*for random number generator*/
#include <random>
#include <math.h>
/*Read txt file*/
#include <fstream>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() : m_pcWheels(NULL),
                                         m_pcProximity(NULL),
                                         m_pcPosSens(NULL),
                                         m_pcSteering(NULL),
                                         m_pcLEDs(NULL),
                                         m_pcCamera(NULL),
                                         m_fWheelVelocity(0),
                                         m_fwheelAngularVelocity(0),
                                         cZAngle(0), cYAngle(0), cXAngle(0)
{
}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(argos::TConfigurationNode &t_node)
{

    m_pcWheels = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcPosSens = GetSensor<argos::CCI_PositioningSensor>("positioning");
    m_pcSteering = GetSensor<argos::CCI_DifferentialSteeringSensor>("differential_steering");
    m_pcLEDs   = GetActuator<argos::CCI_LEDsActuator>("leds");
    m_pcCamera = GetSensor  <argos::CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
    /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
    argos::GetNodeAttributeOrDefault(t_node, "Velocity", m_fWheelVelocity, m_fWheelVelocity);
    argos::GetNodeAttributeOrDefault(t_node, "AngularVelocity", m_fwheelAngularVelocity, m_fwheelAngularVelocity);

    /* try 
    {
      Robot_Communication.Init(GetNode(t_node, "Communicate"));
    }
    catch(CARGoSException& ex) 
    {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    } */
    /* Enable camera filtering */
    m_pcCamera->Enable();
    /* Set beacon color to all red to be visible for other robots */
    m_pcLEDs->SetSingleColor(12, argos::CColor::RED);
}

/****************************************/
/****************************************/
// ******************************* GLOBAL VARIABLES *******************************  //
using namespace std;
//  ------------ Sensor Variable  ------------  //
float Sensor_Range = 10; // Centimeter - Maximun range of proximity sensor
//  ------------ Probability Variable  ------------  //    
float P_Stop  = 0.211;
float P_Exit  = 0.005; 
float P_Pivot = 0.71;
//  ------------ Robot Variable  ------------  //
// Note 1: The cockroach's size is around 6cm long 2 cm width
// Note 2: The foot-bot's size is around 8.5 cm radius
// Note 3: The scale variable represents the size difference between cockroach and robot
float Robot_Radius        = 8.5;  // Centimeter
float Robot_Scale         = 2.8; 
//  ------------ Temporal Variable  ------------  //
// Note 1: All variables in second unit
// Note 2: Sampling time is chosen based on cockroach reacts to sensory information 40ms
// Note 3: We have four interrupt variable, actually, these 4 variables will be tracked with varied sampling time.
//         Example: Cao Feng's algorithm and velocity measurement need some time to collect data before outputting
//         the results; however, for simplification, in my code all interrupt status could be sampled as the speed
//         of 25Hz or 40ms.
float Time_Control_Interval   = 4;    // This interval should be bigger than 3, the duration of forward stimulation
float Time_Sampling           = 0.04;
float Time_Log_Interval       = 1;
//  ------------ Natural Behaviour Variable  ------------  //
// Note 1: The number 2cm is relatively equal to  the length of
//         cockroach's antennae and calculated from Bokurt's results
float Natural_MoveSpeed_Mean  = 1.94   * Robot_Scale; // cm/s
float Natural_MoveSpeed_Std   = 0.29   * Robot_Scale; // cm/s
float Natural_MoveSpeed_Max   = 7.6    * Robot_Scale; // cm/s
float Natural_MoveSpeed_Min   = 1.61   * Robot_Scale; // cm/s
float Natural_MoveLength_Mean = 17.586 * Robot_Scale; // Centimeter 
float Natural_MoveLength_Min  = 12.796 * Robot_Scale; // Centimeter 
float Natural_ExitAngle_Mean  = 36.6;          // Degree
float Natural_ExitAngle_Std   = 2.14;          // Degree            
float Natural_StopTime_Mean   = 36.77;         // Seconds
float Natural_StopTime_Std    = 24.08;         // Seconds
float Natural_WallDistance    = 2*Robot_Scale; // Centimeter  
float   **Natural_TurnDistribution;            // Variable for pivot angle Mise Distribution
const int Natural_TurnDistributionX = 63;
const int Natural_TurnDistributionY = 3;
//  ------------ Response to Electrical Stimulation Variable  ------------  //
// Note 1: >= 3s is needed for forward stimulation as observation in real cockroach
// Note 2: Turning motion is assumed able to be controlled to all wanted angle
// Note 3: Assuming responses of cockroach follows normal distrbutions
float Stimulate_TurnSpeed_Mean = 39.9 *P_PID/180;    // rad/s
float Stimulate_TurnSpeed_Std  = 13.2 *P_PID/180;    // rad/s
float Stimulate_TurnSpeed_Max  = 60   *P_PID/180;    // rad/s
float Stimulate_TurnSpeed_Min  = 18   *P_PID/180;    // rad/s
float Stimulate_MoveSpeed_Mean = 14.9 *Robot_Scale;  // cm/s
float Stimulate_MoveSpeed_Std  = 4.8  *Robot_Scale;  // cm/s
float Stimulate_MoveSpeed_Max  = 25.6 *Robot_Scale;  // cm/s
float Stimulate_MoveSpeed_Min  = 6.5  *Robot_Scale;  // cm/s
//  ------------ Distance Variable  ------------  //
// Note 1: 1st variable is to detect human and choose minimun distance between robots
float Distance_Human_Detection = 300; // cm
float Distance_Communication   = 500; // cm
float Distance_Minimun         = Distance_Human_Detection*1.73; // sqrt(3)*R as publication about triangular geometry
// ------------- Control Variable ------------  //
float Control_K = 2;
float Control_T = 1; // seconds

// ******************************* MAIN FUNCTION *******************************  //
void CFootBotDiffusion::ControlStep()
{
    //  ------------ Updating Timer  ------------  //
    Time_Elapsed = Time_Elapsed + Time_Sampling;
    Time_Control = Time_Control + Time_Sampling;
    Time_Log     = Time_Log     + Time_Sampling;
    //  ------------ Initiation  ------------  //
    // Note: This function is only executed once
    if (Natural_IsInitiation == true)
    {
        Natural_Behavior_Initiation();
        Natural_IsInitiation = false;
    }
    // ------------ Decision of Natural Behaviours  ------------  //
    // Note 1: Robot just decides behaviour only if it is not controlled
    // Note 2: Motions generated by stimulation have some limits:
    //         1 - Turning will not be constraint by wall
    //         2 - Foward motion will be stop by wall
    // Note 3: It is difficult to judge if there constraints affect to simulation. Further
    //         consideration is needed.
    // Note 4: Those two fucntions of Proximity Sensor is put here because they stand for the
    //         "antennae of cockroach" helping robot to decide its natural behaviour. They are
    //         not used for interrupt detection
    float    Robot_SmallestDistance = Sensor_Proximity_Distance();
    argos::CRadians cAngle                 = Sensor_Proximity_Angle();
    if (Stimulate_IsStimulated == 0)
    {
        if (Robot_SmallestDistance <= Natural_WallDistance)
        {
            // - Roach finds its relative position to the wall every 40ms.
            // - If it is inside the specifice range from the wall, unless it is stopping
            //   or following, it will decide its behaviour based on function
            //   "Natural_Behavior_Wall" including: Stop-Follow-Exit the wall
            if ((Status == 'D') || (Status == 'C') || (Status == 'E'))
            {
                Natural_Behavior_Wall(cAngle);
            }
        }
        else
        {
            // - If the roach is far from the wall, unless it is in middle of specific
            //   behaviours, it will decide its behaviour based on function "Natural_Behavior_Central"
            //   including: Stop-Moving&Turning
            if (Status == 'D')
            {
                Natural_Behavior_Central();
            }
        }
    }
    // ------------ Operation of Behaviours  ------------  //
    // - Behavior of the roach is operated based on the variable "Status"
    // - The output of all these below functions is speed, turning decision and
    //   turning direction for the robot
    switch (Status)
    {
        case 'S':
        {
            Natural_Behavior_Stop();
            break;
        }
        case 'C':
        {
            Natural_Behavior_Move();
            break;
        }
        case 'W':
        {
            Natural_Behavior_WallFollow(cAngle);
            break;
        }
        case 'E':
        {
            Natural_Behavior_WallExit();
            break;
        }
        case 'L':
        {
            Stimulated_Behavior_TurnLeft();
            break;
        }
        case 'R':
        {
            Stimulated_Behavior_TurnRight();
            break;
        }
        case 'F':
        {
            Stimulated_Behavior_Forward();
            break;
        }
        case 'Z':
        {
            Stimulated_Behavior_ForwardthenTurn(Stimulate_Size);
            break;
        }
        case 'T':
        {
            Stimulated_Behavior_TurnthenForward(Stimulate_Size);
            break;
        }
    }
    // ------------ Implementation of Footbot Mobile Robot ------------  //
    Robot_Movement();
    // ------------ Logging all Files ------------  //
    if (Time_Log >= Time_Log_Interval)
    {
        //File_OutPut();
        File_LogFile();
        Time_Log = 0;
    }
}
// ******************************* SUB-ROUTINES *******************************  //
void CFootBotDiffusion::Robot_Movement()
{
    // ------------ Implementation of Footbot Mobile Robot ------------  //
    // - The robot uses outputs of above functions to perform cockroach's behaviours
    // - The assumption is that there are only three types of motion:
    //   1. Rotation: The robot rotates about its center
    //   2. Moving forward
    //   3. Stop
    // - Of course, these three motions are not identical to the real motions of cockroach
    //   but they make the simulation become simple. Furthermore, final goal of this simulation
    //   is to test the dispersion behaviour, so this simplification is accecptable
    m_fWheelVelocity = Robot_Speed;
    switch (Robot_IsTurn)
    {
        case 0:
        {
            m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
            break;
        }
        case 1:
        {
            if (Robot_TurnDirection == 0) // Turn Left
            {
                m_pcWheels->SetLinearVelocity(-m_fWheelVelocity, m_fWheelVelocity);
            }
            else // Turn Right
            {
                m_pcWheels->SetLinearVelocity(m_fWheelVelocity, -m_fWheelVelocity);
            }
            break;
        }
    }
}
void CFootBotDiffusion::Control_BorzkutRandom()
{
    // Note: This stimulation stratergy follow sugesstion of Borkurt's group
    //       The stimulation time may be different but the probability is identical
    float Probability = Distribution_Uniform();
    float Time = 3;
    if (Probability <= 0.2)
    {
        // 3 Senconds
        Stimulated_Behaviour('M', Time);
        Stimulate_IsStimulated = 1;
        Status = 'L';
    }
    else
    {
        if (Probability <= 0.4)
        {
            // 3 Senconds
            Stimulated_Behaviour('M', Time);
            Stimulate_IsStimulated = 1;
            Status = 'R';
        }
        else
        {
            // 3 Seconds
            Stimulated_Behaviour('F', Time);
            Stimulate_IsStimulated = 1;
            Status = 'F';
        }
    }
}
void CFootBotDiffusion::Control_PureDiffusion()
{
    float Time = 3;
    Stimulated_Behaviour('M', Time);
    Stimulate_IsStimulated = 1;
    Status = 'F';
}
void CFootBotDiffusion::Control_LevyWalk()
{
    // Notes: For a fast dispersion, in here I induce forward motion of the roach every interval, if we
    //        are able to control (or maintain the velocity of the roach) a better results could be
    //        expected as what the paper says.
    if (Control_Accumulator < Control_T)
    {
        Control_Status      = 'L';
        Control_Accumulator = Control_Accumulator + Control_Alpha*Time_Control_Interval;
        Stimulate_Type      = 'F';
        Stimulate_Size      = 'R';
        Stimulated_Behaviour(Stimulate_Type, Stimulate_Angle);
        Status                 = Stimulate_Type;
        Stimulate_IsStimulated = 1;
        cout << "----------------------------" << endl;
        cout << "Robot:"<< GetId() << endl;
        cout << "Accumulating" << endl;
        cout << "T:" << Control_T << endl;
        cout << "A:" << Control_Accumulator << endl;
        cout << "a:" << Control_Alpha << endl;
    }
    else
    {
        Control_Random_Direction = Distribution_Uniform();
        if (Control_Random_Direction >= 0.5)
        {
            Control_Random_Direction = 1;
            Stimulate_Type = 'T';
            Stimulate_Size = 'R';
        }
        else
        {
            Control_Random_Direction = 0;
            Stimulate_Type = 'T';
            Stimulate_Size = 'L';
        }
        Control_Random_Angle = Distribution_Uniform();
        Control_Random_Angle = Control_Random_Angle*P_PID;
        Control_Alpha        = Distribution_Uniform();
        Control_Alpha        = pow(Control_Alpha, Control_K);
        Control_Accumulator  = 0;
        Stimulate_Angle      = Control_Random_Angle;
        Stimulated_Behaviour(Stimulate_Type, Stimulate_Angle);
        Status                 = Stimulate_Type;
        Stimulate_IsStimulated = 1;
        // The control status here is not 'L' because when Levy control is initiated, the roach should
        // complete the turn and and forward 1st
        Control_Status         = 'I';
        cout << "----------------------------" << endl;
        cout << "Robot:"<< GetId() << endl;
        cout << "New Levy Walk" << endl;
        cout << "T:" << Control_T << endl;
        cout << "A:" << Control_Accumulator << endl;
        cout << "a:" << Control_Alpha << endl;
    }
}
void CFootBotDiffusion::Control_Interrupt()
{
    switch (Control_InterruptStatus)
    {
        case 'S':
        {
            cout << "Stopping" << endl;
            Control_MotionTrigger();
            break;
        }
        case 'W':
        {
            // Note: In this case Proximity Sensor is made use to calculate departing angle
            argos::CRadians cAngle = Sensor_Proximity_Angle();
            Control_WallDeparture(cAngle);
            break;
        }
        case 'N':
        {
            // Pivot a bit then move forward
            Control_RobotAvoidance();
            break;
        }
    }
}
void CFootBotDiffusion::Control_RobotAvoidance()
{
    // Note: The idea here is that there is assumption that all robots have tendency of going straight
    //       so when it faces some robots ahead (-90 to 90 degree), if it turn 180 degree, it may return
    //       to the previous direction and explore and explored area =>> It is better for the robot to
    //       just pivot a bit to dodge the vector ahead
    argos::CRadians Angle = Robot_Communication.Angle;
    float  Turn_Angle = 90*P_PID/180;
    if ((Angle.GetValue() <= (P_PID/2)) && (Angle.GetValue() >= (-P_PID/2)))
    {
        if (Angle.GetValue() >= 0.0f)
        {
            // Turn Right
            Stimulate_Angle        = Angle.GetAbsoluteValue();
            Stimulate_Type         = 'T';
            Stimulate_Size         = 'R';
            Status                 = Stimulate_Type;
            Stimulate_IsStimulated = 1;
            Stimulated_Behaviour(Stimulate_Type, Stimulate_Angle);
        }
        else
        {
            // Turn Left
            Stimulate_Angle        = Angle.GetAbsoluteValue();
            Stimulate_Type         = 'T';
            Stimulate_Size         = 'L';
            Status                 = Stimulate_Type;
            Stimulate_IsStimulated = 1;
            Stimulated_Behaviour(Stimulate_Type, Stimulate_Angle);
        }
    }
    // Reset the Levy Walk
    Control_Status      = 'I';
    Control_Accumulator = Control_T;
}
void CFootBotDiffusion::Control_MotionTrigger()
{
    Control_Status = 'L';
    Stimulated_Behaviour('F', 0);
    Status = 'F';
    Stimulate_IsStimulated = 1;
}
void CFootBotDiffusion::Control_WallDeparture(argos::CRadians Angle)
{
    // Note 1: Turn cockoroach a certain angle so it leave the wall
    //         as the angle = 40 degree (as natural exit behaviour)
    // Note 2: Assuming the roach will wall forward leaving the wall after the turn
    float ExitAngle = (40 + 90)*P_PID/180 - Angle.GetAbsoluteValue();
    if (Angle.GetValue() > 0.0f)
    {
        Stimulated_Behaviour('T', ExitAngle);
        Stimulate_Size = 'R';
        Status = 'T';
    }
    else
    {
        Stimulated_Behaviour('T', ExitAngle);
        Stimulate_Size = 'L';
        Status = 'T';
    }
    Stimulate_IsStimulated = 1;
    // Reset the Levy Walk
    Control_Status      = 'I';
    Control_Accumulator = Control_T;
}
void CFootBotDiffusion::Control_RandomWalk()
{
    Control_Random_Direction = Distribution_Uniform();
    if (Control_Random_Direction >= 0.5)
    {
        Control_Random_Direction = 1;
        Stimulate_Type = 'T';
        Stimulate_Size = 'R';
    }
    else
    {
        Control_Random_Direction = 0;
        Stimulate_Type = 'T';
        Stimulate_Size = 'L';
    }
    Control_Random_Angle = Distribution_Uniform();
    Control_Random_Angle = Control_Random_Angle*P_PID;
    Stimulate_Angle      = Control_Random_Angle;
    Stimulated_Behaviour(Stimulate_Type, Stimulate_Angle);
    Status                 = Stimulate_Type;
    Stimulate_IsStimulated = 1;
}
void CFootBotDiffusion::Stimulated_Behaviour(char Stimulate_TypeofMotion, float Angle)
{
    if ((Stimulate_TypeofMotion == 'L') || (Stimulate_TypeofMotion == 'R') || (Stimulate_TypeofMotion == 'Z') || (Stimulate_TypeofMotion == 'T'))
    {
        Stimulate_TurnSpeed = Distribution_Normal(Stimulate_TurnSpeed_Mean, Stimulate_TurnSpeed_Std);
        if (Stimulate_TurnSpeed < Stimulate_TurnSpeed_Min)
        {
            Stimulate_TurnSpeed = Stimulate_TurnSpeed_Min;
        }
        else
        {
            if (Stimulate_TurnSpeed > Stimulate_TurnSpeed_Max)
            {
                Stimulate_TurnSpeed = Stimulate_TurnSpeed_Max;
            }
        }
        Stimulate_MoveSpeed = Stimulate_TurnSpeed*Robot_Radius;
        Time_Stimulate_Turn = Angle/Stimulate_TurnSpeed;
    }
    if ((Stimulate_TypeofMotion == 'F') || (Stimulate_TypeofMotion == 'Z') || (Stimulate_TypeofMotion == 'T'))
    {
        Stimulate_MoveSpeed = Distribution_Normal(Stimulate_MoveSpeed_Mean, Stimulate_MoveSpeed_Std);
        if (Stimulate_MoveSpeed < Stimulate_MoveSpeed_Min)
        {
            Stimulate_MoveSpeed = Stimulate_MoveSpeed_Min;
        }
        else
        {
            if (Stimulate_MoveSpeed > Stimulate_MoveSpeed_Max)
            {
                Stimulate_MoveSpeed = Stimulate_MoveSpeed_Max;
            }
        }
    }
    if (Stimulate_TypeofMotion == 'M')
    {
        // This for rotation with a given time
        float Time = Angle;
        Stimulate_TurnSpeed = Distribution_Normal(Stimulate_TurnSpeed_Mean, Stimulate_TurnSpeed_Std);
        if (Stimulate_TurnSpeed < Stimulate_TurnSpeed_Min)
        {
            Stimulate_TurnSpeed = Stimulate_TurnSpeed_Min;
        }
        else
        {
            if (Stimulate_TurnSpeed > Stimulate_TurnSpeed_Max)
            {
                Stimulate_TurnSpeed = Stimulate_TurnSpeed_Max;
            }
        }
        Stimulate_MoveSpeed = Stimulate_TurnSpeed*Robot_Radius;
        Time_Stimulate_Turn = Time;
    }
    Time_Stimulate_Decision = Time_Elapsed;
}
void CFootBotDiffusion::Stimulated_Randomly()
{
    float Random = Distribution_Uniform();
    float Angle  = 40*P_PID/180;
    Stimulate_IsStimulated = 1;
    if (Random <= 0.2)
    {
        Stimulated_Behaviour('L', Angle);
        Status = 'L';
    }
    else
    {
        if (Random <= 0.4)
        {
            Stimulated_Behaviour('R', Angle);
            Status = 'R';
        }
        else
        {
            if (Random <= 0.6)
            {
                Stimulated_Behaviour('F', Angle);
                Status = 'F';
            }
            else
            {
                if (Random <= 0.8)
                {
                    Stimulated_Behaviour('Z', Angle);
                    Status = 'Z';
                }
                else
                {
                    Stimulated_Behaviour('T', Angle);
                    Status = 'T';
                }
            }
        }
    }
}
void CFootBotDiffusion::Stimulated_Behavior_TurnLeft()
{
    if ((Time_Elapsed - Time_Stimulate_Decision) < Time_Stimulate_Turn)
    {
        Robot_Speed            = Stimulate_MoveSpeed;
        Robot_IsTurn           = 1;
        Robot_TurnDirection    = 0;
        Stimulate_IsStimulated = 1;
        Status                 = 'L';
    }
    else
    {
        Stimulate_IsStimulated = 0;
        Status = 'D';
    }
}
void CFootBotDiffusion::Stimulated_Behavior_TurnRight()
{
    if ((Time_Elapsed - Time_Stimulate_Decision) < Time_Stimulate_Turn)
    {
        Robot_Speed            = Stimulate_MoveSpeed;
        Robot_IsTurn           = 1;
        Robot_TurnDirection    = 1;
        Stimulate_IsStimulated = 1;
        Status                 = 'R';
    }
    else
    {
        Stimulate_IsStimulated = 0;
        Status = 'D';
    }
}
void CFootBotDiffusion::Stimulated_Behavior_Forward()
{
    if ((Time_Elapsed - Time_Stimulate_Decision) < Time_Stimulate_Forward)
    {
        Robot_Speed            = Stimulate_MoveSpeed;
        Robot_IsTurn           = 0;
        Robot_TurnDirection    = 0;
        Stimulate_IsStimulated = 1;
        Status                 = 'F';
    }
    else
    {
        Stimulate_IsStimulated = 0;
        Status = 'D';
    }
}
void CFootBotDiffusion::Stimulated_Behavior_ForwardthenTurn(char Turn_Direction)
{
    if ((Time_Elapsed - Time_Stimulate_Decision) < Time_Stimulate_Forward)
    {
        Robot_Speed            = Stimulate_MoveSpeed;
        Robot_IsTurn           = 0;
        Robot_TurnDirection    = 0;
        Stimulate_IsStimulated = 1;
        Status                 = 'Z';
    }
    else
    {
        if ((Time_Elapsed - (Time_Stimulate_Decision + Time_Stimulate_Forward)) < Time_Stimulate_Turn)
        {
            Robot_Speed         = Stimulate_TurnSpeed*Robot_Radius;
            Robot_IsTurn        = 1;
            switch (Turn_Direction)
            {
                case 'L':
                {
                    Robot_TurnDirection = 0;
                    break;
                }
                case 'R':
                {
                    Robot_TurnDirection = 1;
                    break;
                }
            }
            Stimulate_IsStimulated = 1;
            Status                 = 'Z';
        }
        else
        {
            Stimulate_IsStimulated = 0;
            Status                 = 'D';
        }
    }
}
void CFootBotDiffusion::Stimulated_Behavior_TurnthenForward(char Turn_Direction)
{
    if ((Time_Elapsed - Time_Stimulate_Decision) < Time_Stimulate_Turn)
    {
        Robot_Speed         = Stimulate_TurnSpeed*Robot_Radius;
        Robot_IsTurn        = 1;
        switch (Turn_Direction)
        {
            case 'L':
            {
                Robot_TurnDirection = 0;
                break;
            }
            case 'R':
            {
                Robot_TurnDirection = 1;
                break;
            }
        }
        Stimulate_IsStimulated = 1;
        Status                 = 'T';
    }
    else
    {
        if ((Time_Elapsed - (Time_Stimulate_Decision + Time_Stimulate_Turn)) < Time_Stimulate_Forward)
        {
            Robot_Speed            = Stimulate_MoveSpeed;
            Robot_IsTurn           = 0;
            Robot_TurnDirection    = 0;
            Stimulate_IsStimulated = 1;
            Status                 = 'T';
        }
        else
        {
            Stimulate_IsStimulated = 0;
            Status                 = 'D';
        }
    }
}
void CFootBotDiffusion::Natural_Behavior_Central()
{
    double StopOrMove = Distribution_Uniform();
    if (StopOrMove <= P_Stop)
    {
        Time_Natural_Stop = Distribution_Normal(Natural_StopTime_Mean, Natural_StopTime_Std);
        Time_Natural_Stop = Distribution_Exponential(Time_Natural_Stop);
        Status = 'S';
    }
    else
    {
        Natural_MoveLength = Distribution_Exponential(Natural_MoveLength_Mean);
        if (Natural_MoveLength < Natural_MoveLength_Min)
        {
            Natural_MoveLength = Natural_MoveLength_Min;
        }
        Natural_MoveSpeed = Distribution_Normal(Natural_MoveSpeed_Mean, Natural_MoveSpeed_Std);
        if (Natural_MoveSpeed < Natural_MoveSpeed_Min)
        {
            Natural_MoveSpeed = Natural_MoveSpeed_Min;
        }
        else
        {
            if (Natural_MoveSpeed > Natural_MoveSpeed_Max)
            {
                Natural_MoveSpeed = Natural_MoveSpeed_Max;
            }
        }
        Natural_TurnSpeed = Natural_MoveSpeed / Robot_Radius;
        double Natural_TurnIsPivotSameSize  = Distribution_Uniform();
        double Natural_TurnAngleProbability = Distribution_Uniform();
        int Natural_TurnColumn;
        if (Natural_TurnIsPivotSameSize <= P_Pivot)
        {
            Natural_TurnColumn = 1;
            Natural_TurnDirection = Natural_TurnDirection;
        }
        else
        {
            Natural_TurnColumn = 2;
            Natural_TurnDirection = !Natural_TurnDirection;
        }
        for (int i = 1; i < Natural_TurnDistributionX; i++)
        {

            if (Natural_TurnDistribution[i][Natural_TurnColumn] > Natural_TurnAngleProbability)
            {
                Natural_TurnAngle = Natural_TurnDistribution[i][0];
                break;
            }
        }
        Time_Natural_Forward = Natural_MoveLength / Natural_MoveSpeed;
        Time_Natural_Turn = (Natural_TurnAngle * P_PID / 180) / Natural_TurnSpeed;
        Status = 'C';
    }
    Time_Natural_Decision = Time_Elapsed;
}
void CFootBotDiffusion::Natural_Behavior_Wall(argos::CRadians Angle)
{
    double StopOrWallFollowOrExit = Distribution_Uniform();
    if (StopOrWallFollowOrExit <= P_Exit)
    {
        Natural_MoveLength = Distribution_Exponential(Natural_MoveLength_Mean);
        if (Natural_MoveLength < Natural_MoveLength_Min)
        {
            Natural_MoveLength = Natural_MoveLength_Min;
        }
        Natural_MoveSpeed = Distribution_Normal(Natural_MoveSpeed_Mean, Natural_MoveSpeed_Std);
        if (Natural_MoveSpeed < Natural_MoveSpeed_Min)
        {
            Natural_MoveSpeed = Natural_MoveSpeed_Min;
        }
        else
        {
            if (Natural_MoveSpeed > Natural_MoveSpeed_Max)
            {
                Natural_MoveSpeed = Natural_MoveSpeed_Max;
            }
        }
        Time_Natural_Exit = Natural_MoveLength/Natural_MoveSpeed;
        Natural_TurnSpeed = Natural_MoveSpeed/Robot_Radius;
        Natural_ExitAngle = Distribution_Log_Normal(Natural_ExitAngle_Mean, Natural_ExitAngle_Std);
        if (Natural_ExitAngle > 78)
        {
            Natural_ExitAngle = 78;
        }
        else
        {
            if (Natural_ExitAngle < 17)
            {
                Natural_ExitAngle = 17;
            }
        }
        Natural_ExitAngle = (Natural_ExitAngle + 90)*P_PID/180 - Angle.GetAbsoluteValue();
        Time_Natural_Turn = Natural_ExitAngle/Natural_TurnSpeed;
        if (Angle.GetValue() > 0.0f)
        {
            Robot_IsTurn        = 1;
            Robot_TurnDirection = 1;
        }
        else
        {
            Robot_IsTurn        = 1;
            Robot_TurnDirection = 0;
        }
        Status = 'E';
    }
    else
    {
        if (StopOrWallFollowOrExit <= (P_Exit + P_Stop))
        {
            Time_Natural_Stop = Distribution_Normal(Natural_StopTime_Mean, Natural_StopTime_Std);
            Time_Natural_Stop = Distribution_Exponential(Time_Natural_Stop);
            Status = 'S';
        }
        else
        {
            Natural_MoveLength = Distribution_Exponential(Natural_MoveLength_Mean);
            if (Natural_MoveLength < Natural_MoveLength_Min)
            {
                Natural_MoveLength = Natural_MoveLength_Min;
            }
            Natural_MoveSpeed = Distribution_Normal(Natural_MoveSpeed_Mean, Natural_MoveSpeed_Std);
            if (Natural_MoveSpeed < Natural_MoveSpeed_Min)
            {
                Natural_MoveSpeed = Natural_MoveSpeed_Min;
            }
            else
            {
                if (Natural_MoveSpeed > Natural_MoveSpeed_Max)
                {
                    Natural_MoveSpeed = Natural_MoveSpeed_Max;
                }
            }
            Time_Natural_Follow = Natural_MoveLength/Natural_MoveSpeed;
            Status = 'W';
        }
    }
    Time_Natural_Decision = Time_Elapsed;
}
void CFootBotDiffusion::Natural_Behavior_Stop()
{
    if ((Time_Elapsed - Time_Natural_Decision) >= Time_Natural_Stop)
    {
        Status = 'D';
    }
    else
    {
        Status = 'S';
        Robot_Speed         = 0;
        Robot_IsTurn        = 0;
        Robot_TurnDirection = 0;
    }
}
void CFootBotDiffusion::Natural_Behavior_Move()
{
    if ((Time_Elapsed - Time_Natural_Decision) < Time_Natural_Forward)
    {
        Robot_Speed         = Natural_MoveSpeed;
        Robot_IsTurn        = 0;
        Robot_TurnDirection = 0;
        Status              = 'C';
    }
    else
    {
        if ((Time_Elapsed - (Time_Natural_Decision + Time_Natural_Forward)) < Time_Natural_Turn)
        {
            Robot_Speed         = Natural_MoveSpeed;
            Robot_IsTurn        = 1;
            Robot_TurnDirection = Natural_TurnDirection;
            Status              = 'C';
        }
        else
        {
            Status = 'D';
        }
    }
}
void CFootBotDiffusion::Natural_Behavior_WallFollow(argos::CRadians Angle)
{
    // Note 1: This is not really perfect but acceptable as the target is simulation of
    //         Dispersion algorithm.
    // Note 2: In reallity, the wall following could be modeld by using PID controller, so
    //         for achievement of accurate wall follow behaviour, a controller should be
    //         used for the footbot mobile robot
    if ((Time_Elapsed - Time_Natural_Decision) < Time_Natural_Follow)
    {
        Status = 'W';
        // Tolerace for wall following behaviour
        float Angle1 = 95*P_PID/180;
        float Angle2 = 88*P_PID/180;
        Robot_Speed  = Natural_MoveSpeed;
        if ((Angle.GetAbsoluteValue() < Angle1) && (Angle.GetAbsoluteValue() > Angle2))
        {           
            Robot_IsTurn        = 0;
            Robot_TurnDirection = 0;
        }
        else
        {
            if (Angle.GetAbsoluteValue() >= Angle1)
            {
                // Turning toward the wall
                if (Angle.GetValue() > 0.0f)
                {
                    Robot_IsTurn        = 1;
                    Robot_TurnDirection = 0;
                }
                else
                {
                    Robot_IsTurn        = 1;
                    Robot_TurnDirection = 1;
                }
            }
            else
            {
                // Turning away the wall
                if (Angle.GetValue() > 0.0f)
                {
                    Robot_IsTurn        = 1;
                    Robot_TurnDirection = 1;
                }
                else
                {
                    Robot_IsTurn        = 1;
                    Robot_TurnDirection = 0;
                }
            }
        }
    }
    else
    {
        Status = 'D';
    }
}
void CFootBotDiffusion::Natural_Behavior_WallExit()
{
    // Note 1: The exit behaviour is made by two assumptions:
    //         1 - The roach turns a certain angle then moves forward
    //         2 - The roach decides a distance to move as what it does in central behaviour
    //         3 - The angle of antennae now is still -60 to 60
    // Note 2: Point 3 is not really look like what happens in real situation but because
    //         cockroach is controlled periodically so it is not a big problem and acceptable
    if ((Time_Elapsed - Time_Natural_Decision) < Time_Natural_Turn)
    {
        Status      = 'E';
        Robot_Speed = Natural_MoveSpeed;
    }
    else
    {
        if ((Time_Elapsed - (Time_Natural_Decision + Time_Natural_Turn)) < Time_Natural_Exit)
        {
            Status              = 'E';
            Robot_Speed         = Natural_MoveSpeed;
            Robot_IsTurn        = 0;
            Robot_TurnDirection = 0;
        }
        else
        {
            Status = 'D';
        }
    }
}
void CFootBotDiffusion::Natural_Behavior_Initiation()
{
    Status   = 'D';
    double IsTurnLeft = Distribution_Uniform();
    if (IsTurnLeft < 0.5)
    {
        Natural_TurnDirection = 0;
    }
    else
    {
        Natural_TurnDirection = 1;
    }
    Natural_TurnDistribution = File_ReadFileFromTxt();
    Control_Accumulator      = Control_T;
    Control_Status           = 'L';
}
void CFootBotDiffusion::Sensor_Positioning()
{
    m_positioning        = m_pcPosSens->GetReading().Position;
    m_rotation           = m_pcPosSens->GetReading().Orientation;
    m_rotation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    argos::CRadians GlobalAngle = cZAngle;
    // Note: Imaging this sensor is used to determine if robot is near wall.
    //       In Reality, Cao Feng's algorithm will be made used
/*     if ((m_positioning.GetX()*100 >= Area_XMax) || (m_positioning.GetX()*100 <= Area_XMin) 
        || (m_positioning.GetY()*100 >= Area_YMax) || (m_positioning.GetY()*100 <= Area_YMin)) 
    {
        Control_IsInterrupt     = 1;
        Control_InterruptStatus = 'W';
    } */
    float    Robot_SmallestDistance = Sensor_Proximity_Distance();
    if (Robot_SmallestDistance <= Natural_WallDistance)
    {
        Control_IsInterrupt     = 1;
        Control_InterruptStatus = 'W';
    }
}
void CFootBotDiffusion::Sensor_Velocity()
{
    leftWheel_velocity   = m_pcSteering->GetReading().VelocityLeftWheel;
    rightWheel_velocity  = m_pcSteering->GetReading().VelocityRightWheel;
    if ((leftWheel_velocity == 0) && (rightWheel_velocity == 0))
    {
        Control_IsInterrupt     = 1;
        Control_InterruptStatus = 'S';
    }
}
void CFootBotDiffusion::Sensor_Camera() 
{
    argos::CVector2 Vector_Robot_TooNear;
    size_t Number_Robot_Communication = 0;
    size_t Number_Robot_TooNear       = 0;
    // Get the camera readings
    const argos::CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings &sReadings = m_pcCamera->GetReadings();
    // Go through the camera readings to calculate the flocking interaction vector
    if (!sReadings.BlobList.empty())
    {
        for (size_t i = 0; i < sReadings.BlobList.size(); ++i)
        {
            if (sReadings.BlobList[i]->Color == argos::CColor::RED && sReadings.BlobList[i]->Distance <= Distance_Communication)
            {
                ++Number_Robot_Communication;
                // Take the blob distance and angle
                if (sReadings.BlobList[i]->Distance <= Distance_Minimun)
                {
                    ++Number_Robot_TooNear;
                    Vector_Robot_TooNear += argos::CVector2(sReadings.BlobList[i]->Distance, sReadings.BlobList[i]->Angle);
                }
            }
        }
        Robot_Communication.Angle = Vector_Robot_TooNear.Angle();
    }
    Robot_Communication.NoRobot_InRange = Number_Robot_Communication;
    Robot_Communication.NoRobot_TooNear = Number_Robot_TooNear;
/*     if (Robot_Communication.NoRobot_InRange <= 0)
    {
        Control_IsInterrupt     = 1;
        Control_InterruptStatus = 'Z';
    }
    else
    {
        if (Robot_Communication.NoRobot_TooNear > 0)
        {
            Control_IsInterrupt     = 1;
            Control_InterruptStatus ='N';
        }
    } */
    if (Robot_Communication.NoRobot_TooNear > 0)
    {
        Control_IsInterrupt     = 1;
        Control_InterruptStatus ='N';
    }
}

float CFootBotDiffusion::Sensor_Proximity_Distance()
{
    float Robot_SmallestDistance = 0;
    const argos::CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    for (size_t i = 0; i < tProxReads.size(); ++i)
    {
        if (Status == 'E')
        {
            // Note: This condition is created with the perpestive that when the roach
            //       want to leave the wall, it narrows the range of waving antennae 
            //       -60 to 60 degrees, so there is two advantages:
            //       1 - Roach can enter another wall in front of it
            //       2 - Robot can completely performce exit behaviour
            if (((i >= 0) && (i <= 3)) || ((i >= 21) && (i <= 23)))
            {
                if (tProxReads[i].Value >= Robot_SmallestDistance)
                {
                    Robot_SmallestDistance =  tProxReads[i].Value;
                }
            }
        }
        else
        {
            // Note: The range of antennae here is -150 to 150 
            if (((i >= 0) && (i <= 9)) || ((i >= 16) && (i <= 23)))
            {
                if (tProxReads[i].Value >= Robot_SmallestDistance)
                {
                    Robot_SmallestDistance =  tProxReads[i].Value;
                }
            }
        }
    }
    Robot_SmallestDistance = 10 - Robot_SmallestDistance*10; // Smallest distance between roach and obstacle
    return Robot_SmallestDistance;
}
argos::CRadians CFootBotDiffusion::Sensor_Proximity_Angle()
{
    const argos::CCI_FootBotProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
    argos::CVector2 cAccumulator;
    for (size_t i = 0; i < tProxReads.size(); ++i)
    {
        cAccumulator += argos::CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    cAccumulator          /= tProxReads.size();
    argos::CRadians cAngle        = cAccumulator.Angle();           // Relative angle between cockroach and obstacle
    return cAngle;
}
void CFootBotDiffusion::Sensor_RangeandBearing_Transmit()
{
}
void CFootBotDiffusion::Sensor_RangeandBearing_Receive()
{
}
void CFootBotDiffusion::File_LogFile()
{
    std::ofstream redirect_file;
    std::string fileName = GetId();
    std::string Control = "Cockroach_Free_";
    int Time = Time_Elapsed;
    if (!redirect_file.is_open())
    {
        redirect_file.open(("LOG/" + Control + fileName), std::ios::out | std::ios::app);
        streambuf *strm_buffer = cout.rdbuf();
        cout.rdbuf(redirect_file.rdbuf());
        cout << Time << '\t' << fixed << setprecision(4) << m_positioning.GetX() << '\t' << m_positioning.GetY() << '\t' << cZAngle << endl;
        cout.rdbuf(strm_buffer);
    }
    else
    {
        redirect_file.clear();
        redirect_file.close();
    }
}
void CFootBotDiffusion::File_OutPut()
{
    switch (Status)
    {
        case 'S':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "STOPING" << endl;
            break;
        }
        case 'C':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "MOVING CENTRAL" << endl;
            break;
        }
        case 'W':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "FOLLOWING WALL" << endl;
            break;
        }
        case 'E':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "EXITING WALL" << endl;
            break;
        }
        case 'L':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "STIMULATING TURN LEFT" << endl;
            break;
        }
        case 'R':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "STIMULATING TURN RIGHT" << endl;
            break;
        }
        case 'F':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "STIMULATING MOVE FORWARD" << endl;
            break;
        }
        case 'Z':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "STIMULATING MOVE FORWARD THEN TURN" << endl;
            break;
        }
        case 'T':
        {
            cout << "Robot ID: " << GetId() << endl;
            cout << "STIMULATING TURN THEN MOVE FORWARD" << endl;
            break;
        }
    }
}
double CFootBotDiffusion::Distribution_Uniform()
{
    //uniform distribution
    default_random_engine generator;
    uniform_real_distribution<double> distribution(0.0, 1.0);
    int m = atoi(GetId().c_str());
    srand(time(0) + m);
    double randNum;

    for (int i = 0; i < rand() % 100 + 1; ++i)
    {
        randNum = distribution(generator);
    }
    return randNum;
}
double CFootBotDiffusion::Distribution_Exponential(float lambda)
{
    default_random_engine generator;
    exponential_distribution<double> distribution(1 / lambda);
    int m = atoi(GetId().c_str());
    srand(time(0) + m);
    double number;
    for (int i = 0; i < rand() % 100 + 1; ++i)
    {
        number = distribution(generator);
    }
    return number;
}
double CFootBotDiffusion::Distribution_Normal(float mean, float sigma)
{
    default_random_engine generator;
    normal_distribution<double> distribution(mean, sigma);
    int m = atoi(GetId().c_str());
    srand(time(0) + m);
    double normal_distribution_number;
    for (int i = 0; i < rand() % 100 + 1; ++i)
    {
        normal_distribution_number = distribution(generator);
    }
    return normal_distribution_number;
}
double CFootBotDiffusion::Distribution_Log_Normal(float mean, float sigma)
{
    default_random_engine generator;
    lognormal_distribution<double> distribution(log(mean), log(sigma));
    int m = atoi(GetId().c_str());
    srand(time(0) + m);
    double log_normal_distribution_number;
    for (int i = 0; i < rand() % 100 + 1; ++i)
    {
        log_normal_distribution_number = distribution(generator);
    }
    return log_normal_distribution_number;
}
float **File_ReadFileFromTxt()
{

    float **Natural_TurnDistribution = new float *[Natural_TurnDistributionX];

    ifstream file("/home/huuduoc/Documents/Swarm-Project/controllers/Cockroach/Modelling_PivotDistribution.txt");

    for (int i = 1; i < Natural_TurnDistributionX; i++)
    {
        Natural_TurnDistribution[i] = new float[Natural_TurnDistributionY];
        for (int j = 0; j < Natural_TurnDistributionY; j++)
        {
            file >> Natural_TurnDistribution[i][j];
        }
    }
    return Natural_TurnDistribution;
}
REGISTER_CONTROLLER(CFootBotDiffusion, "Cockroach_Move_Free_Controller");

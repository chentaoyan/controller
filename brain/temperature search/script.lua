-- Use Shift + Click to select a robot
-- When a robot is selected, its variables appear in this editor

-- Use Ctrl + Click (Cmd + Click on Mac) to move a selected robot to a different location

--  IF IT'S POSSIBLE, SEPARATE THIS INTO MULTIPLE FILES!

--Initialization
package.path = package.path .. ";../?.lua"
roach = require('lib.controller');

--  Control Variables
timeInterval = 0.1;   --  In seconds
natural_time_Interval = 100;    -- Roach's natural behavior.
controlTimeInterval = timeInterval * 10 --  We perform 10 iterations/second. DO NOT CHANGE UNLESS NECESSARY.
time = 0;           --  Control time
inputSignal = -1;    --  -1 as false, 1 as true. False as we first let the cockroaches move freely
min = 1;
move_or_stop = 0;
walk_or_turn = 1

--  Constants
MIN_VELOCITY = 1;
MAX_VELOCITY = 10;
MAX_ANGULAR = 5;
ROBOT_RADIUS = 8; --    This result is obtained from base_ground[3].offset.y 
MOTOR_GROUND_X = 6.3;
MOTOR_GROUND_Y = 1.16;
MOTOR_THETA = math.atan(MOTOR_GROUND_Y/MOTOR_GROUND_X);

--  Probability (related)
stop_move = 0.221 -- P(stop) = 0.211
same_diff_direction = 0.71 -- P(same)=0.71 or P(diff)=0.71 ??????


function driveAsCar(forwardSpeed, AngularSpeed)
    leftSpeed = forwardSpeed - AngularSpeed;
    RightSpeed = forwardSpeed + AngularSpeed;
    robot.wheels.set_velocity(leftSpeed, RightSpeed);
end

function driveStraight(distance) 
    local f_speed = distance / timeInterval;
    driveAsCar(f_speed, 0);
end

function stop()
    robot.wheels.set_velocity(0, 0);
end

--  Function to turn, with angle in degree. Goes anticlockwise.
function turn(angle)
    local ROT_TIME = timeInterval; --   Here we're using our timeInterval for now.
    local ROT_LINEAR_VELOCITY = math.rad(angle % 360) * (1/ROT_TIME) * ROBOT_RADIUS;    --  Basic physics formula.
    robot.wheels.set_velocity(-ROT_LINEAR_VELOCITY, ROT_LINEAR_VELOCITY);
end

--  Get the value of our current source. This returns the average of the 4 sensors. Can be modified if wanted.
function getCurrentSource()
    local value = 0;
    for i = 1, 4 do
        value = value + robot.motor_ground[i].value;
    end
    return value / 4;
end

function controlCheck()
    if robot.motor_ground[1].value > min then return true;
    else return false;
    end
end

--  Do our simulating behavior here.
function simulatedBehavior()
    log('Finding better source');
    turn(2);
end

--  This is a cockroach's normal behavior.
function naturalBehavior()
        if (move_or_stop == 1) then
            roach.stop()
        else
            if (time % natural_time_Interval ~= 0) then
                if (walk_or_turn == 0) then
                    roach.walk()
                else
                    roach.turn()
                end
            else
                a = math.random()
                print("probability of move",a)
            if (a > stop_move) then
                if (walk_or_turn == 0) then
                    roach.walk()
                else
                    roach.turn()
                end
            else
                move_or_stop = 1
                roach.stop()
            end
        end
    end
    if robot.motor_ground[1].value < min then min = robot.motor_ground[1].value end;
end

--  Start the simulation.
function startSimulate()
    if controlCheck() then
        --inputSignal = -inputSignal
        time = 0;   -- Set the time back to 0, instead of adding timeInterval to prefer overflow.
        simulatedBehavior();
    else
        -- This is natural behavior
        if(inputSignal == -1) then naturalBehavior() end;
    end
end

--[[ This function is executed every time you press the 'execute' button ]]
function init()
    -- put your code here
end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
    -- put your code here
    startSimulate();
    time = time + 1;
end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   -- put your code here
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
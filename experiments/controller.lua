-- Use Shift + Click to select a robot
-- When a robot is selected, its variables appear in this editor

-- Use Ctrl + Click (Cmd + Click on Mac) to move a selected robot to a different location



-- global variables here
-- time
total_time = 0

-- send command every 10 seconds (to be changed)
control_time_interval = 150
control_time = 0

-- distance to go straight
straight = 0
straight_finish = true

-- radian to turn
turn = 0
turn_finish = true
direction = 0

-- straight-0 turn-1
straight_or_turn = 0


-- velocity (to be changed)
MINIMUM_forward_velocity = 0 -- in cm/s
MAXIMUM_forward_velocity = 20
MINIMUM_angular_velocity = -1/2*math.pi  -- in radian/s
MAXIMUM_angular_velocity = 1/2*math.pi



-- Put your functions here
-------------------------------------------//
function randomspeed(min,max)
    return min + math.random()  * (max - min)
end
-------------------------------------------//
--radius of footbot is 14cm
--angularSpeed in radian
--forwardSpeed in cm/s
function driveAsCar(forwardSpeed, angularSpeed)

  -- We have an equal component, and an opposed one
  leftSpeed  = forwardSpeed - angularSpeed*7
  rightSpeed = forwardSpeed + angularSpeed*7
  robot.wheels.set_velocity(leftSpeed,rightSpeed)
end
-------------------------------------------//
function showProximitySensor()
	log("--Proximity Sensors--")
		for i = 1,24 do
    		log("Angle: " .. robot.proximity[i].angle ..
        			"Value: " .. robot.proximity[i].value)
		end
end
-------------------------------------------//
function avoidObstacle(straight_speed, angular_speed)
	sensingLeft = robot.proximity[3].value + robot.proximity[4].value +
              robot.proximity[5].value + robot.proximity[6].value

	sensingRight = robot.proximity[22].value+ robot.proximity[21].value +
               robot.proximity[20].value + robot.proximity[19].value
	-- log(sensingLeft)
	-- log(sensingRight)
	if( sensingLeft ~= 0 ) then
	  	driveAsCar(0,2*math.pi)
	elseif( sensingRight ~= 0 ) then
  		driveAsCar(0,2*math.pi)
	else
  		driveAsCar(straight_speed,angular_speed)
-- to simulate the behavior of  cockroach
	end
end
-------------------------------------------//
function randomMove()
	forward_velocity = randomspeed(MINIMUM_forward_velocity,MAXIMUM_forward_velocity)
    angular_velocity = randomspeed(MINIMUM_angular_velocity,MAXIMUM_angular_velocity)
    avoidObstacle(forward_velocity,angular_velocity)
end
-------------------------------------------//
function goStraightUnderControl()
	forward_velocity = randomspeed(MINIMUM_forward_velocity,MAXIMUM_forward_velocity)
    straight = straight - forward_velocity * 0.1 -- distance left to go straight
    driveAsCar(forward_velocity,0)
end
-------------------------------------------//
function turnUnderControl(direction)
--	print("before turning",turn*360/math.pi/2)
	if direction==0 then
		angular_velocity = randomspeed(-2*math.pi,0)
    	turn = turn + angular_velocity * 0.1
    else
    	angular_velocity = randomspeed(0,2*math.pi)
    	turn = turn - angular_velocity * 0.1 -- radian left to turn
    end
    driveAsCar(0, angular_velocity)
--    print("angular_velocity",angular_velocity*360/math.pi/2)
--    print("after turning",turn*360/math.pi/2)
	
end




--[[ This function is executed every time you press the 'execute' button ]]
function init()
   -- put your code here
end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
   -- put your code here
	    --showSensorDistance()
	    --avoidObstacle()
	    --showGroundSensor()
	    --followTheMark()
	if (total_time ~= control_time) then
	    if (straight<=0 and turn<=0) then
	    	if (straight_finish == false) then
	           print("process of going straight is finished")
	           straight_finish = true
	        elseif (turn_finish == false) then
	           print("process of turning is finished")
	           turn_finish = true
	        end
	        randomMove()
        elseif straight>0 then
        	goStraightUnderControl()
        else
        	turnUnderControl(direction)
       	end
    else
    	-- reset first four variables, in case the movement is not finished
    	straight = 0
		turn = 0
		straight_finish = true
		turn_finish = true
        print("Time:",total_time/10)
		control_time = control_time + control_time_interval
		print("Enter 0 to go straight, 1 to turn")
		straight_or_turn = io.read("*n")
		-- in case of wrong choice
		while straight_or_turn~=0 and straight_or_turn~=1 do
			print("Enter 0 to go straight, 1 to turn")
			straight_or_turn = io.read("*n")
		end
		if straight_or_turn == 0 then
	        print("Enter the distance to go straight (cm)")
			straight = io.read("*n") -- in cm
	        straight_finish = false
	        goStraightUnderControl()
	    else
	    	print("Enter the direction to turn, 0 for clockwise, 1 for counterclockwise")
	    	direction = io.read("*n")
	    	print("Enter the degree to turn (degree)")
			turn = io.read("*n") -- in cm
			turn = turn/360*2*math.pi
	        turn_finish = false
	        turnUnderControl(direction) 
	    end
	end
	total_time = total_time + 1
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

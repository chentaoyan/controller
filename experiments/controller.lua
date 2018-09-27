-- Use Shift + Click to select a robot
-- When a robot is selected, its variables appear in this editor

-- Use Ctrl + Click (Cmd + Click on Mac) to move a selected robot to a different location



-- global variables here






------------------ control  ------------//
-- time
time_interval = 0.1
total_time = 0
-- send command every 50 seconds (to be changed)
control_time_interval = 500
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

------------------ natural  ------------//
--probability (related)
stop_move = 0.221 -- P(stop) = 0.211
same_diff_direction = 0.71 -- P(same)=0.71 or P(diff)=0.71 ??????

LAMBDA = 0.5

--time
time_interval_stop_or_move = 100 -- every 10 seconds decide 
--to move or stop(only happens when it is in moving state)


-- velocity (need to addjust)
MINIMUM_forward_velocity = 5 -- in cm/s
MAXIMUM_forward_velocity = 20
MINIMUM_angular_velocity = 1/4*math.pi  -- in radian/s
MAXIMUM_angular_velocity = 1*math.pi

-- ADJUST variables
ADJUST_DISTANCE = 15
ADJUST_STOPTIME = 1

-- natural movement records
move_or_stop = 1 -- 0 is move 1 is stop
walk_or_turn = 0 --0 is walk 1 is turn
nstop_time = 0
nwalk_distance = 0
nwalk_linear_velocity = 0
nturn_angle = 0
nturn_angular_velocity = 0
previous_direction = 0 -- 0 for clockwise, 1 for counterclockwise

-- exponential distribution
function exp_dist()
	math.randomseed( os.time() )
	z = math.random()
	x = -(1/LAMBDA)*math.log(z)
	return x
end

-- natural movement function
-- stop
function natural_stop()
	if (nstop_time<=0) then
		nstop_time = exp_dist()*ADJUST_STOPTIME
		print("stop time (in s)")
		print(nstop_time)
	end
	robot.wheels.set_velocity(0,0)
	nstop_time = nstop_time - time_interval
	if (nstop_time<=0) then
		if (math.random()<stop_move) then
			nstop_time = exp_dist()*ADJUST_STOPTIME
			print("stop time (in s)")
			print(nstop_time)
		else
			move_or_stop = 0
		end
	end
end

-- walk
function natural_walk_distance()
	return exp_dist()*ADJUST_DISTANCE
end

function natural_linear_velocity()
	return random(MINIMUM_forward_velocity, MAXIMUM_forward_velocity)
end

function natural_walk()
	if nwalk_distance<=0 then
		nwalk_distance = natural_walk_distance()
		nwalk_linear_velocity = natural_linear_velocity()
		print("natural walk distance")
		print(nwalk_distance)
		print("nautural walk velocity")
		print(nwalk_linear_velocity)
	end
	nwalk_distance = nwalk_distance - time_interval*nwalk_linear_velocity
	if nwalk_distance<=0 then
		walk_or_turn = 1
	end
	robot.wheels.set_velocity(nwalk_linear_velocity,nwalk_linear_velocity)
end


-- turn
function natural_turn_angle()
	return random(0, 180)
end
--!!!!! I assume angle it turns is also a normal distribution
--!!!!! no idea of the distribution file


function natural_angular_velocity()
	return random(MINIMUM_angular_velocity, MAXIMUM_angular_velocity)
end

function natural_turn_direction()
	-- now is that P(diff)=0.71
	if (math.random()<same_diff_direction) then
		previous_direction = (previous_direction +1)%2
	end
	return previous_direction
end

function natural_turn()
	if nturn_angle<=0 then
		nturn_angle = natural_turn_angle()
		nturn_angular_velocity = natural_angular_velocity()
		previous_direction = natural_turn_direction()
		print("natural turn angle")
		print(nturn_angle)
		print("nautural turn angular velocity (in radian)")
		print(nturn_angular_velocity)
	end
	nturn_angle = nturn_angle - time_interval*nturn_angular_velocity*360/2/math.pi
	if nturn_angle <= 0 then
		walk_or_turn = 0
	end
	if (previous_direction == 0) then
		driveAsCar(0,nturn_angular_velocity)
	else
		driveAsCar(0,-nturn_angular_velocity)
	end

end







-- Put your functions here
-------------------------------------------//
function random(min,max)
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
	forward_velocity = random(MINIMUM_forward_velocity,MAXIMUM_forward_velocity)
    angular_velocity = random(MINIMUM_angular_velocity,MAXIMUM_angular_velocity)
    avoidObstacle(forward_velocity,angular_velocity)
end
-------------------------------------------//
function goStraightUnderControl()
	forward_velocity = random(MINIMUM_forward_velocity,MAXIMUM_forward_velocity)
    straight = straight - forward_velocity * 0.1 -- distance left to go straight
    driveAsCar(forward_velocity,0)
end
-------------------------------------------//
function turnUnderControl(direction)
--	print("before turning",turn*360/math.pi/2)
	if direction==0 then
		angular_velocity = random(-2*math.pi,0)
    	turn = turn + angular_velocity * 0.1
    else
    	angular_velocity = random(0,2*math.pi)
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
	        if (move_or_stop == 1) then
	        	natural_stop()
	        else
	        	if (total_time%time_interval_stop_or_move ~= 0) then
			        if (walk_or_turn == 0) then
			        	natural_walk()
			        else
			        	natural_turn()
				    end
				else
					a = math.random()
					print("probability of move",a)
					if (a>stop_move) then
						if (walk_or_turn == 0) then
			        		natural_walk()
			        	else
			        		natural_turn()
				    	end
				    else
				    	move_or_stop = 1
				    	natural_stop()
				    end
				end
			end
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


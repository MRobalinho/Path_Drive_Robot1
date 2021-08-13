%% Path Following for a Differential Drive Robot
% Using example described on 
% https://www.mathworks.com/help/robotics/ug/path-following-for-differential-drive-robot.html
% Student: Manuel Robalinho
% Date: 01-08-2021

% Define Waypoints
% Define a set of waypoints for the desired path for the robot
clear;
clc;
close('all');

path = [2.00    1.00;  % Init position
        1.25    1.75;
        5.25    8.25;
        7.25    8.75;
        11.75   10.75;
        12.00   10.00]; % Finish position
    
% Set the current location and the goal location of the robot as defined by the path.
robotInitialLocation = path(1,:);  % Init
robotGoal = path(end,:);   % End

% Assume an initial robot orientation (the robot orientation is the angle between the robot heading and the positive X-axis, measured counterclockwise).
initialOrientation = 0;

% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotInitialLocation initialOrientation]';

%% Create a Kinematic Robot Model
%Initialize the robot model and assign an initial pose. 
% The simulated robot has kinematic equations for the motion of a two-wheeled differential drive robot.
% The inputs to this simulated robot are linear and angular velocities.

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");

% Visualize the desired path

titletext = 'Initial Path';
subtitletext = 'Autonomous Robotic Systems';
txt = {'MRobalinho'};
figure
plot(path(:,1), path(:,2),'k--d')
xlim([0 13])
ylim([0 13])
title(titletext,subtitletext)
text(11,-1.3,txt)

%% Define the Path Following Controller
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path 
% following controller using the controllerPurePursuit object.
controller = controllerPurePursuit;

% Use the path defined above to set the desired waypoints for the controller
controller.Waypoints = path;

% Set the path following controller parameters. The desired linear velocity 
% is set to 0.6 meters/second for this example.
controller.DesiredLinearVelocity = 0.6;

% The maximum angular velocity acts as a saturation limit for rotational 
% velocity, which is set at 2 radians/second for this example.
controller.MaxAngularVelocity = 2;

% As a general rule, the lookahead distance should be larger than the 
% desired linear velocity for a smooth path. The robot might cut corners 
% when the lookahead distance is large. In contrast, a small lookahead 
% distance can result in an unstable path following behavior. 
% A value of 0.3 m was chosen for this example.
controller.LookaheadDistance = 0.3;

%% Using the Path Following Controller, Drive the Robot over the Desired Waypoints
% The path following controller provides input control signals for the robot, 
% which the robot uses to drive itself along the desired path.

% Define a goal radius, which is the desired distance threshold between the 
% robot's final location and the goal location. Once the robot is within this distance
% from the goal, it will stop. Also, you compute the current distance between the robot 
% location and the goal location. This distance is continuously checked against the goal 
% radius and the robot stops when this distance is less than the goal radius.

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

% The controllerPurePursuit object computes control commands for the robot. 
% Drive the robot using these control commands until it reaches within the goal radius. 

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

titletext = 'Path Execution by Robot';

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(path(:,1), path(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 13])
    ylim([0 13])

    title(titletext,subtitletext)
    text(11,-1.3,txt)

    waitfor(vizRate);
end

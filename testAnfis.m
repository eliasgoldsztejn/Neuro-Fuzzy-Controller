MembershipFunctions = []
%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% Copyright 2019 The MathWorks, Inc.

%% Simulation setup
% Define Vehicle
MaxTime = 45;                   % Maximum time for simulation
AchievedTargetDist = 0.5;       % Target considered as achieved 
NS = 3;                         % Number of sensors
M = 1;                          % Mass of robot
R = 1;                          % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);

% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:MaxTime;        % Time array

% Initial conditions
initPose = [10;2;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;

% Load map
close all
load exampleMap

% Create your own map
%map1 = load('map.mat');

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
% Select number of sensors
lidar.scanAngles = linspace(-pi/2,pi/2,NS);
lidar.maxRange = 5;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
%viz.mapName = 'map1.map';
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Path planning and following

% Create waypoints
%waypoints = [10 15;
%             20 12;];
waypoints = [10 5;
             10 2.2;];

% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

% Vector Field Histogram (VFH) for obstacle avoidance
vfh = controllerVFH;
vfh.DistanceLimits = [0.05 3];
vfh.NumAngularSectors = 36;
vfh.HistogramThresholds = [5 10];
vfh.RobotRadius = L;
vfh.SafetyDistance = L;
vfh.MinTurningRadius = 0.25;

% FIS controller
fis = sugfis("NumInputs",5,"NumOutputs",2);

fis.Outputs(1,1).MembershipFunctions(1,1) = fismf('linear',[-1 -1 -1 -1 -1 -1],'Name',"mf1");
fis.Outputs(1,2).MembershipFunctions(1,1) = fismf('linear',[-1 -1 -1 -1 -1 -1],'Name',"mf1");

fis = removeMF(fis,"input1","mf2");
fis = removeMF(fis,"input1","mf3");

fis = removeMF(fis,"input2","mf2");
fis = removeMF(fis,"input2","mf3");

fis = removeMF(fis,"input3","mf2");
fis = removeMF(fis,"input3","mf3");

fis = removeMF(fis,"input4","mf2");
fis = removeMF(fis,"input4","mf3");

fis = removeMF(fis,"input5","mf2");
fis = removeMF(fis,"input5","mf3");

fis = removeMF(fis,"output1","mf2");
fis = removeMF(fis,"output1","mf3");

fis = removeMF(fis,"output2","mf2");
fis = removeMF(fis,"output2","mf3");

for i = 1:5
    for j = 1:2
        mF = 1;
        if j == 2
            mF = 3;
        end
        fis.Inputs(1,i).MembershipFunctions.Parameters(mF) = MFParameters((i-1)*2 + (j-1) + 1);
    end
end
length(MFParameters)
for i = 1:2
    for j = 1:6
        fis.Outputs(1,i).MembershipFunctions.Parameters(j) = MFParameters(20 + (i-1)*6 + (j-1) + 1);
    end
end

%% Simulation loop
r = rateControl(1/sampleTime);
curVel = [0,0,0];
wR = 0;
wL = 0;
safetyMeasure = 0;
reachedTargets = zeros(1,cast(numel(waypoints/2),'uint8'));
finishedTime = MaxTime;
for idx = 2:numel(tVec) 
    
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
        
    % Run the path following and obstacle avoidance algorithms
    %[vDesired,wDesired,lookAheadPt] = controller(curPose);
    %targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    %steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    %if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
    %    wDesired = 0.1*steerDir;
    %end
    %vDesired = -3;
    %wDesired = 0.75;
    
    % Control the robot
    % Insert dynamic model
    %velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    % Ger acceleration from fis controller
    occupancyMap = 1;
    targetDist = sqrt((lookAheadPt(2)-curPose(2))^2 + (lookAheadPt(1)-curPose(1))^2);
    targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
    [aR,aL] = evalfis(fis,[ranges(1),ranges(2),ranges(3),targetDist,targetDir])
    
    % Update velocities
    [vRefX, vRefY, wRef, wR, wL] = diffDriveDynamicModelAnfis(curPose,curVel,wR,wL,sampleTime,aR,aL,M,R,L,dd,occupancyMap);
    velB = [vRefX;vRefY;wRef];
    curVel = velB;
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Safety measure min mean distances
    min_ranges = min(ranges);
    if isnan(min(ranges))
        min_ranges = 30;
    end
    safetyMeasure = safetyMeasure + min_ranges;
    
    % Min time measure
    for i = 1:cast(numel(waypoints)/2,'uint8')
        i
        sqrt((waypoints(i,2)-curPose(2))^2 + (waypoints(i,1)-curPose(1))^2)
        if sqrt((waypoints(i,2)-curPose(2))^2 + (waypoints(i,1)-curPose(1))^2) < AchievedTargetDist
            reachedTargets(i) = 1;
        end
    end
    
    % Stop loop if reached all targets
    flag = 0;
    for i = 1:cast(numel(waypoints)/2,'uint8')
        if reachedTargets(i) == 1
            flag = flag + 1;
        end
    end
    if flag == cast(numel(waypoints)/2,'uint8')
        finishedTime = idx*sampleTime;
        break;
    end
    
    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
end

% Ouput of safety meeasure and minimum times
safetyMeasure = safetyMeasure/(idx*sampleTime);


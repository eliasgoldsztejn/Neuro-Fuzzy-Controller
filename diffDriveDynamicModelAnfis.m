function [vx,vy,w,wR,wL] = diffDriveDynamicModel(curPose,curVel,wRCur,wLCur,dt,aR,aL,M,R,L,dd,frictionMat,mapSize)
% diffDriveDynamicModel: A discrete time numerical model for a differential
% drive robot under a friction map.
%   Inputs:
%          curPose: [x,y,theta] - Represents the current position of the robot in
%          the map axis.
%          curVel: [vx,vy,w] - Represents the current velocity in x, y and
%          angular velocities.
%          wR = Right wheel angular velocity.
%          wL = Left wheel angular velocity.
%          dt: - Discrete time interval.
%          vRef: - Desired linear velocity found by controller.
%          wRef: - Desired angular velocity found by controller.
%          M - Mass of the robot.
%          R - Radius of the wheels.
%          L - Diameter of the robot.
%          dd - Diferential drive robot instantiation.
%          occupancyMap - A map containing the coefficient of   friction parameters
%          depending on the position of the robot.
%   Outputs: 
%          v - Linear velocity of the robot.
%          w - Angular velocity of the robot.

% Definitions (For now we do not use occupancyMap)

u = frictionValue(frictionMat,curPose(1),curPose(2),mapSize);
%u = 0.1;              % Coefficient of friction
%u = 100;              % Coefficient of friction
Iz = (1/2)*M*(L/2)^2; % Moment of inertia of robot assuming it is a uniform solid cilinder.
C = 10;                % Constant limiting power on wheels.

vxCur = curVel(1);
vyCur = curVel(2);
wCur = curVel(3);

%[wLRef,wRRef] = inverseKinematics(dd,vRef,wRef);

% The acceleration depends linearly on the desired velocity, and is inversely linear to timestep and mass of robot.

%aR = (wRRef - wRCur)/(dt*M*C); % Desired angular acceleration on right wheel
%aL = (wLRef - wLCur)/(dt*M*C); % Desired angular acceleration on left wheel
if isnan(aR)
    aR = 0;
end

if isnan(aL)
    aL = 0;
end

FRActual = R*frictionWheelDynamics(aR/R,wRCur,u,M);                 % Forced excerted on right part of robot.
FLActual = R*frictionWheelDynamics(aL/R,wLCur,u,M);                 % Forced excerted on left part of robot.
wA = ((FLActual - FRActual)*(L/2))/Iz;                              % Angular acceleration
Fy = frictionLateralDynamics(wCur,vxCur,vyCur,dt,u,M,R,L);                   % Forced excerted laterally (y axis) on robot.
%Fx = min(FRActual, FLActual);                                       % Forced excerted in x axis on robot. 
Fx = 0;
if sign(FRActual) == sign(FLActual)
    Fx = min(FRActual, FLActual);
else
    if abs(FRActual) >= abs(FLActual)
        Fx = FRActual - sign(FRActual)*abs(FRActual - FLActual);
    else
        Fx = FLActual - sign(FLActual)*abs(FRActual - FLActual);
    end
end

vx = vxCur + dt*(Fx/M);
vy = vyCur + dt*(Fy/M);
w = wCur + dt*wA;
wR = wRCur + dt*FRActual/R;
wL = wLCur + dt*FLActual/R;

end


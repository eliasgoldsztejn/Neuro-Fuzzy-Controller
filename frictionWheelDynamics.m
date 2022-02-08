function [a] = frictionWheelDynamics(aApplied,w,u,M)
%frictionModelDynamics This function calculates the final force of traction
%on the wheel, given the acceleration applied, the current angular velocity of the
%wheel, the coefficient of friction and the mass of the robot.
%   Inputs: 
%          aApplied - Acceleration applied on the wheel by the motor.
%          w - Angular velocity of the wheel.
%          u - Coefficient of friction.
%          M - Mass of the robot.
%   Outputs:
%           a - Force of traction on the wheel.
g = 9.81;

a = aApplied;
if ( aApplied > M*g*u)
    a = M*g*u;
end

end


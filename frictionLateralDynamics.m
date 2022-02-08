function [a] = frictionLateralDynamics(wCur,vXCur,vyCur,dt,u,M,R,L)
%frictionLateralDynamics This function calculates the final force of
%lateral slip on the robot, given the lateral velocity of the robot 
% the coefficient of friction and the mass of the robot.
%   Inputs: 
%          vXCur - Velocity x axis.
%          u - Coefficient of friction.
%          M - Mass of the robot.
%   Outputs:
%           a - Force excerted laterally on robot.
g = 9.81;
ICC = (L)*(vXCur/(wCur*R)) - vyCur*dt;
w = wCur;

a = 0;
if ( abs(w^2*ICC*M) > M*g*u)
    a = w^2*ICC*M - M*g*u;
end

end


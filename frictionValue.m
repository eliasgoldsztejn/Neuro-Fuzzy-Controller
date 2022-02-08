function [u] = frictionValue(frictionMat,x,y,mapSize)
%frictionValue This function returns the coefficient of friction given the
%position of the robot, a friction matrix and the size of the map.
%   Inputs: frictionMat - a matrix containing the friction coefficients
%   according to the position of the robot.
%            x,y - position of the robot.
%            mapSize - [a,b] the width and height of the map.
%   Outputs: u - coefficient of friction.

a = mapSize(1);
b = mapSize(2);
[matWidth, matHeight] = size(frictionMat);
cellWidth = a/matWidth;
cellHeight = b/matHeight;
if x >= 0 && y >=0 && x < a && y < b
    u = frictionMat(1 + floor(x/cellWidth), 1 + floor(y/cellHeight));
end

if x<0 || x > a
    u = 0;
end

if y<0 || y > b
    u = 0;
end

end


%I = imread('.\Maze16.png');
%imwrite(I,'.\Maze16.pgm');
I = imread('.\Maze1.pgm');
%I = I(100:150,100:150);

imageNorm = double(I)/255;
imageOccupancy = 1 - imageNorm;
imageOccupancy = round(imageOccupancy);

map = robotics.OccupancyGrid(imageOccupancy,2);
save('map')
figure()
show(map)
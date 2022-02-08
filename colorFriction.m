Im = zeros(100,100,3);
%frictionMat = rand(20);
frictionMat1 = load('frictionMat');
frictionMat1 = cell2mat(struct2cell(frictionMat1));
mapSize = [101,101];
rangeCol = 1/(max(max(frictionMat1)) - min(min(frictionMat1)));
%frictionMat1 = frictionMat1.*rangeCol;
%frictionMat1 = imgaussfilt(frictionMat1,[0.8 0.8]);

figure;
for i = 1:100
    for j=1:100
        Im(i,j,3) = 1;%(1-frictionValue(frictionMat,i,j,mapSize));
        %Im(i,j,1) = frictionValue(frictionMat,i,j,mapSize);
        Im(i,j,2) = (1-frictionValue(frictionMat1,i,j,mapSize));
        Im(i,j,1) = 1;%(1-frictionValue(frictionMat,i,j,mapSize));
    end
end
imshow(Im);
%alpha(0.8)
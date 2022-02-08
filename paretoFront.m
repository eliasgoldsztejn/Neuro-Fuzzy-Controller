function [Indexes] = paretoFront(scores)
%paretoFront It finds the indexes of the dominant solutions in the
%pareto front
%   Input: scores - the fval of the solutions.
%   Output: Indexes - a vector of indexes of the dominant solutions.

sizeScores = size(scores);
lenScores = sizeScores(1);
numObj = sizeScores(2);
bestSols = ones(sizeScores(1),1);

for i=1:lenScores
    for j=1:lenScores
        %betterOverallSol = 1
        if j ~= i
            betterPartialSol = zeros(numObj,1);
            for s =1:numObj
                if scores(i,s) > scores(j,s)
                    betterPartialSol(s) = 1;
                end
            end
            if sum(betterPartialSol) == s
                bestSols(i) = 0;
            %else
            %    betterOverallSol = 0
            end
        end
    end
    %if betterOverallSol == 0
    %    bestSols(i) = 0;
    %end
end

Indexes = bestSols;

end


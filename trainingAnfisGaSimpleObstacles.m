% This training is succesful for no obstacles
problem = struct;
problem.fitnessfcn = @ TrialFrictionFisObstacles;
%problem.fitnessfcn = @(x) paretoTrial(x(1),x(2))
%problem.fitnessfcn = @(x)[x(1), x(2)];
%problem.nonlcon = @confunAnfis;
problem.nvars = 22;
% The variables are the mean of the gbemlf of the input membership
% functions and the weights of the rules.
problem.lb = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 1, 0.1, 1, 0.1, 1, 0, 0, 0, 0];
problem.ub = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,   2, 6,   2, 6,   2, 6, 1, 1, 1, 1];
problem.solver = 'gamultiobj';
%objectivesToPlot = [1,2]; % or whatever two objectives you want
%plotfn = @(options,state,flag)gaplotpareto(options,state,flag,objectivesToPlot);
options = optimoptions('gamultiobj','PopulationSize',100,'MaxGenerations',100,'CrossoverFraction',0.6,'FunctionTolerance',0,'ConstraintTolerance',0,'ParetoFraction',0.9,'PlotFcns',@gaplotpareto);
%options.PlotFcns = {@gaplotpareto};
problem.options = options;


%%
[x,fval,exitflag,output,pop,scores] = gamultiobj(problem);  

%%
%Ind = paretoFront(scores);
length_x = size(x);
length_x = length_x(1);
scores_best = zeros(length_x,4);

for i=1:length_x
    scores_best(i,:) = TrialFrictionFisObstacles(x(i,:));
end

%%
figure;
%scatter3(scores(:,2),scores(:,1).*0.1,scores(:,3),'filled');
%ylabel('Time [s]');
%%xlabel('S1 [M]');
%hold on;

for i=1:length_x
    scatter3(scores_best(i,2),scores_best(i,1),scores_best(i,3),'filled','r');
    ylabel('S1 [-M]');
    xlabel('Time [s]');
    zlabel('Energy [J]');
    title('Pareto front for S1, Time and Energy')
    hold on;
end

%%
figure;

for i=1:length_x
    scatter3(scores_best(i,2),scores_best(i,4),scores_best(i,3),'filled','r');
    ylabel('S2 [1/M]');
    xlabel('Time [s]');
    zlabel('Energy [J]');
    title('Pareto front for S2, Time and Energy')
    hold on;
end


%%
figure;
scatter(scores_best(:,2),scores_best(:,3),'filled');
ylabel('Time [s]');
xlabel('Energy [J]');

figure;
scatter(scores_best(:,2),scores_best(:,4),'filled');
ylabel('Time [s]');
xlabel('S2 [1/M]');


%% Average of best solutions
avg_time = 0;
avg_S1 = 0;
avg_S2 = 0;
avg_E = 0;

for i=1:length_x
    %scatter3(scores_best(i,2),scores_best(i,4),scores_best(i,3),'filled','r');
    avg_time = avg_time + scores_best(i,2);
    avg_S1 = avg_S1 - scores_best(i,1);
    avg_S2 = avg_S2 + 1/scores_best(i,4);
    avg_E = avg_E + scores_best(i,3);
end

avg_time/length_x
avg_S1/length_x
avg_S2/length_x
avg_E/length_x

%% Best of best solutions
min_time = 500;
min_S1 = 0;
min_S2 = 0;
min_E = 500;

for i=1:length_x
    %scatter3(scores_best(i,2),scores_best(i,4),scores_best(i,3),'filled','r');
    if scores_best(i,2) < min_time
        min_time = scores_best(i,2);
    end
    if (- scores_best(i,1)) > min_S1
        min_S1 = - scores_best(i,1);
    end
    if 1/scores_best(i,4) > min_S2
        min_S2 = 1/scores_best(i,4);
    end
    if scores_best(i,3)/0.1 < min_E
        min_E = scores_best(i,3);
    end
end

min_time
min_S1
min_S2
min_E

%% Median of best solutions
time_vec = zeros(1,length_x);
S1_vec = zeros(1,length_x);
S2_vec = zeros(1,length_x);
E_vec = zeros(1,length_x);
j = 0;

for i=1:length_x
    %scatter3(scores_best(i,2),scores_best(i,4),scores_best(i,3),'filled','r');
    time_vec(i) = scores_best(i,2);
    S1_vec(i) = (- scores_best(i,1));
    S2_vec(i) = 1/scores_best(i,4);
    E_vec(i) = scores_best(i,3);
end

median(time_vec)
median(S1_vec)
median(S2_vec)
median(E_vec)

%%
%% find conservative solutions
std_time = 0.6*std(time_vec);
std_S1 = 0.6*std(S1_vec);
std_S2 = 0.6*std(S2_vec);
std_energy = 0.6*std(E_vec);

avg_time = mean(time_vec);
avg_S1 = mean(S1_vec);
avg_S2 = mean(S2_vec);
avg_energy = mean(E_vec);

conservative_sols = zeros(1,length_x);
j = 0;
h = 1;
for i=1:length_x
    if time_vec(i) < avg_time + std_time && time_vec(i) > avg_time - std_time ...
            && S1_vec(i) < avg_S1 + std_S1 && S1_vec(i) > avg_S1 - std_S1 ...
            && S2_vec(i) < avg_S2 + std_S2 && S2_vec(i) > avg_S2 - std_S2 ...
            && E_vec(i) < avg_energy + std_energy && E_vec(i) > avg_energy - std_energy
        conservative_sols(h) = i;
        h = h + 1;    
    end
end

%% Plot of obective measures
figure;
histogram(time_vec)
ylabel('Frequency');
xlabel('[s]');
%%
figure;
histogram(S1_vec)
ylabel('Frequency');
xlabel('[m]');
%%
figure;
histogram(S2_vec)
ylabel('Frequency');
xlabel('[m]');
%%
figure;
histogram(E_vec)
ylabel('Frequency');
xlabel('[J]');
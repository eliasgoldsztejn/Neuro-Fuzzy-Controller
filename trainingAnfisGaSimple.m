% This training is succesful for no obstacles
problem = struct;
problem.fitnessfcn = @ TrialFrictionFis;
%problem.fitnessfcn = @(x) paretoTrial(x(1),x(2))
%problem.fitnessfcn = @(x)[x(1), x(2)];
%problem.nonlcon = @confunAnfis;
problem.nvars = 15;
% The variables are the mean of the gbemlf of the input membership
% functions and the weights of the rules.
problem.lb = [0, 0, 0, 0,0, 0, 0, 0, 0,0,0,0,0,0,0];
problem.ub = [1, 1, 1, 1, 1, 1, 1, 1, 1,0,0,0,0,0,0];
problem.solver = 'gamultiobj';
%objectivesToPlot = [1,2]; % or whatever two objectives you want
%plotfn = @(options,state,flag)gaplotpareto(options,state,flag,objectivesToPlot);
options = optimoptions('gamultiobj','PopulationSize',20,'MaxGenerations',50,'CrossoverFraction',0.2,'FunctionTolerance',0,'ConstraintTolerance',0,'ParetoFraction',0.9,'PlotFcns',@gaplotpareto);
%options.PlotFcns = {@gaplotpareto};
problem.options = options;


%%
[x,fval,exitflag,output,pop,scores] = gamultiobj(problem);  
%% Main3D_WCW.m
% Closed-loop (feedback) optimization across wind speeds for 3D brachiation.
clear; close all; clc;

%% ---------- parameters ----------
param.m1=4.8; param.m2=4.8; param.l1=1; param.l2=1;
param.I1=(1/3)*(param.m1)*(param.l1^2); param.I2=(1/3)*(param.m2)*(param.l2^2);
param.fk1=0.0; param.fk2=0.0; param.g=9.81;
param.lc1=0.5*param.l1; param.lc2=0.5*param.l2;

% mechanical/electronic limits
param.theta2_max = 160*pi/180;
param.theta1_max = 135*pi/180;
param.theta1x_target = 0.6809;
param.theta1y_target = 0.845708;   % 3D azimuth target
param.theta2_target  = 1.851464;
param.target_joint = [0.645065, 0.845708, 1.851464];

% torque/simulation settings
param.Torqhandmax = 500;   % peak torque allowable
param.t_max = 300;
param.ngrid = 15;         

% environment and wind scenarios
param.x2_target=0;
param.y2_target=-0.81;
param.windSpeedList = [2, -2, 3];% list of wind speeds to evaluate (worst-case)
param.s_tol         = 0.1;       % allowable overshoot (m)
param.v_max         = 0.5;       % max allowable speed at crossing

% Simulink model name 
param.modelName = 'PID_PFL_3D_winds'; % 

%% initial guesses (use V-pose to target trajectory)
theta1x_0 = -0.6809;
theta1y_0 =  0.0;      % initial yaw (lets keep it nill for now)
theta1z_0 =  0.0;
theta2_0  = -1.7853;

theta1x_init = linspace(theta1x_0, param.theta1x_target, param.ngrid)';
theta1y_init = linspace(theta1y_0, param.theta1y_target, param.ngrid)';
theta2_init  = linspace(theta2_0,  param.theta2_target, param.ngrid)';

theta1x_init(end) = param.theta1x_target;
theta1y_init(end) = param.theta1y_target;
theta2_init(end)  = param.theta2_target;


lc1_0 = param.lc1;
lc2_0 = param.lc2;
tswing_0 = 1.0;

% decision vector layout:
% p = [tswing; lc1; lc2; theta1x(1:n); theta1y(1:n); theta2(1:n)]
pinput_0 = [tswing_0; lc1_0; lc2_0; theta1x_init; theta1y_init; theta2_init];

%% Bounds (compact construction)
n = param.ngrid;
LB = [ 1; 0.2; 0.2; -param.theta1_max*ones(n,1); -param.theta1_max*ones(n,1); -param.theta2_max*ones(n,1) ];
UB = [ 1.5; 0.8; 0.8;  param.theta1_max*ones(n,1);  param.theta1_max*ones(n,1);  param.theta2_max*ones(n,1) ];

% lock first waypoint to starting V-pose (keep as you had)
LB(4)   = theta1x_0; UB(4)   = theta1x_0;
LB(4+n) = theta1y_0; UB(4+n) = theta1y_0;
LB(4+2*n) = theta2_0; UB(4+2*n) = theta2_0;

% lock last waypoint to desired target (ensures trajectory ends at the target)
idx_last_th1x = 3 + n;        % index of last theta1x
idx_last_th1y = 3 + 2*n;      % index of last theta1y
idx_last_th2  = 3 + 3*n;      % index of last theta2

LB(idx_last_th1x) = param.theta1x_target; UB(idx_last_th1x) = param.theta1x_target;
LB(idx_last_th1y) = param.theta1y_target; UB(idx_last_th1y) = param.theta1y_target;
LB(idx_last_th2)  = param.theta2_target;  UB(idx_last_th2)  = param.theta2_target;

% keep lengths fixed as before
LB(2)=lc1_0; UB(2)=lc1_0; LB(3)=lc2_0; UB(3)=lc2_0;

% fmincon options (tighter tolerances)
options = optimoptions('fmincon', ...
     'Display',               'iter', ...
     'Algorithm',             'interior-point', ...
     'EnableFeasibilityMode', true, ...
     'MaxFunctionEvaluations',2e5, ...
     'MaxIterations',         2e4, ...
     'TolCon',                1e-4, ...        % tighten constraint tolerance
     'TolFun',                1e-6, ...
     'StepTolerance',         1e-8);

%% create problem & run
costfunction = @(p) Cost_3D_WCW(p,param);
constraintfunction = @(p) Constraint_3D_WCW(p,param);

problem = createOptimProblem('fmincon','x0',pinput_0,'objective',costfunction,'nonlcon',constraintfunction,...
    'lb',LB,'ub',UB,'options',options);

% disp('Start Optimisation (3D closed-loop, worst-case wind)');
% xmin = fmincon(problem);
% disp('Optimal decision vector:');
% disp('Solution (xmin):');
% disp(xmin);

xmin=[0.8803
    0.5000
    0.5000
   -0.6809
   -1.0228
   -0.1185
   -0.4862
   -0.3572
    0.0588
   -0.1142
    0.0546
    0.1045
   -0.0395
    0.1928
    0.6992
    0.2603
    0.5794
    0.6809
         0
   -0.1923
    0.1085
    0.2372
    0.2746
    0.0528
    0.3192
    0.6623
    0.3454
    0.5163
    0.6805
    0.5918
    0.8618
    0.8001
    0.8457
   -1.7853
   -1.2076
   -0.7193
   -0.9961
   -0.6745
   -0.3508
   -0.1996
    0.0065
    0.4357
    0.1264
    1.0402
    1.1072
    1.5082
    1.4886
    1.7853]';


%% post-check constraints on solution
[c,ceq] = Constraint_3D_WCW(xmin,param);
fprintf('=== Inequality constraints (c ≤ 0): ===\n');
for i = 1:length(c); fprintf('  c(%d) = %g\n', i, c(i)); end
fprintf('\n=== Equality constraints (ceq = 0): ===\n');
for i = 1:length(ceq); fprintf('  ceq(%d) = %g\n', i, ceq(i)); end

%% Run a final simulation with pResult and print where the joints ended up
mdl = 'PID_PFL_3D_winds';
set_param(mdl,'StopTime', num2str(xmin(1)));

n = param.ngrid;
timeVec = linspace(0, xmin(1), n)';
assignin('base','timeVec', timeVec);
assignin('base','theta1x_in', xmin(4 : 3 + n));
assignin('base','theta1y_in', xmin(4 + n : 3 + 2*n));
assignin('base','theta2_in', xmin(4 + 2*n : 3 + 3*n));
simOut_test = sim(mdl, 'ReturnWorkspaceOutputs', 'on', 'SrcWorkspace', 'base');

% get final joint angles from sim result
th1_ts = simOut_test.get('theta1x'); th2_ts = simOut_test.get('theta2'); th1y_ts = simOut_test.get('theta1y');
if isa(th1_ts,'timeseries'), th1v = th1_ts.Data(:); else th1v = th1_ts(:); end
if isa(th2_ts,'timeseries'), th2v = th2_ts.Data(:); else th2v = th2_ts(:); end
if isa(th1y_ts,'timeseries'), th1yv = th1y_ts.Data(:); else th1yv = th1y_ts(:); end

EE_landed = [ th1v(end), th1yv(end), th2v(end) ];
fprintf('Target joint angles: [%.6f, %.6f, %.6f]\n', param.target_joint);
fprintf('Final joint angles  : [%.6f, %.6f, %.6f]\n', EE_landed);

%% Optional: animate or inspect solution with your Animation_3D function
[time, theta1x, theta1y, theta1z, theta2, torque_elbow, torque_wristy] = Animation_3D(xmin, param);







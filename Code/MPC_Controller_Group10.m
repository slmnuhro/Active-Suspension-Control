%% Vehicle Dynamics and Control
% Project Group 10
% MPC Controller for Active Suspension of a Quarter Car Model

clc; 
clear; 
close all;

%%
if isunix
    command = 'rm -r plots_mpc';
elseif ispc
    command = 'rmdir /s /q plots_mpc';
elseif ismac
    command = 'rm -r plots_mpc';
end

status = system(command);
if (status ~= 0)
    disp("Failed to remove 'plots_mpc' directory. Please remove it manually")
end

command = 'mkdir plots_mpc';
status = system(command);
if (status ~= 0)
    disp("Failed to create 'plots' directory. Please create it manually")
end

% Set interpreter and layout options
set(groot,'defaulttextinterpreter','latex');  
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultLineLineWidth',0.8)
set(groot,'defaultAxesFontSize',11)
set(groot,'defaultAxesFontWeight',"normal")

%%
% QC parameters
m_s = 410;                                           % sprung mass, kg
m_u = 45;                                            % unsprung mass, kg
k_t = 230000;                                        % tire vertical stiffness, N/m
k_s = 25500;                                         % suspension vertical stiffness, N/m
wn_s = sqrt(k_s*k_t / (k_s + k_t)/m_s);              % sprung mass natural frequency 
d_s = 0.3 * (2 * m_s * wn_s);                        % damping ratio

% frequency range determination
n_points = 5000;
w = linspace(0.1,100,n_points)*2*pi;

%%
% passive QC
% state matrix
A = [0, 1, 0, 0;...                                  % unsprung displacement - road disturbance
    -k_t / m_u, -d_s / m_u, k_s / m_u, d_s / m_u;... % unsprung mass acceleration
    0, -1, 0, 1;...                                  % sprung displacement - unsprung displacement
    0, d_s / m_s, -k_s / m_s, -d_s / m_s];           % sprung mass acceleration
B = [0, m_s / m_u, 0, -1]';                          % control matrix

% Road Disturbance 
G = [0, k_t / m_u, 0, 0]';

% output matrix
C = [k_t 0 0 0;...                                   % tire force
     0 0 1 0;...                                     % suspension stroke
     A(4,:)];                                        % sprung mass acceleration
Dw = [-k_t; 0; 0];
Du = [0; 0; -1];                                     % feedthrough matrix

%% Augment the Matrix
lti.A = A;
lti.B = [B G];
lti.C = C;
lti.D = [Du Dw];

plant = ss(lti.A,lti.B,lti.C,lti.D);

trans_fun = tf(plant);

plant = tf(trans_fun.Numerator, trans_fun.Denominator,'OutputDelay', 0.0000001);

Ts = 0.1;  % (Numerical issues) 0.035 for step results, 0.1 for sinusoidal results
Tstop = 7; % SIMULATION TIME [s] (7s for Sinusoidal of 3s, 3.5s for step)
sim_time = round(Tstop/Ts);

% Discretizes plant, absolete 'cause mpc(.) does so, required for quadprog
% plant = c2d(plant,Ts);

%% Road disturbance generation
% Define the frequency of the road disturbance.
f = 1; % Hz [4 to compare with Robust controller]
Disturbance_start = 0.5;    % Play around with [s]
Disturbance_duration = 3;   % Play around with [s]
Disturbance_end = Disturbance_start+(1/f)*floor(Disturbance_duration/(1/f));      

% Create a time vector from 0 to Tstop.
% with increments of Ts.
T = (0:Ts:Tstop - Ts)';

% Define the road disturbance profile.
v = 0.025*(1-cos(2*pi*f*(T-Disturbance_start)));     % was 0.05; 0.025 to compare with H-\infty

% Oscillating disturbance Start and Stop times
% v(T < 2*1/f) = 0;
% v(T > 3*1/f) = 0;
v(T < Disturbance_start) = 0;
v(T > Disturbance_end) = 0;

% Step disturbance (COMMENT FOR SINUSOID; uncomment for step)
% v = zeros(size(T));
% v(T>Disturbance_start) = 0.05;

%%
% T = (0:Ts:(sim_time*Ts - Ts))';
Unn = zeros(sim_time,2);
Unn(:,2) = v; %hx(1:sim_time);

[Yuc,Tuc,Xuc] = lsim(plant,Unn,T);

f = figure;
title('Response of Uncontrolled Suspension to Road Disturbance');
plot(T, v, 'DisplayName', 'Road Disturbance','color','#c97112',"LineWidth",1.25); hold on;
% plot(Tuc, Yuc(:,1)*1e-3, 'DisplayName', 'Body acceleratoin','color','#1caac7',"LineWidth",1.25); % Scaled body acceleration response
plot(Tuc, Yuc(:,2), 'DisplayName', 'Suspension Deflection','color','#1caac7',"LineWidth",1.25); % Comment out for only Step OR Sinusoid
xlim([0,Tstop])

% ylim([-0.01,0.06])     % Uncomment for Sinusoid_Disturbance w/ Ts = 0.01 OR Step_Disturbance with Ts = 0.07
% ylim([-0.06,0.055])    % Uncomment for Step_Plant_Response with Ts = 0.07
ylim([-0.035,0.055])     % Uncomment for Sinusoid_Plant_Response w/ Ts = 0.01 OR 

hold off;
grid on;
legend('location','southeast');

% title('Sinusoidal Disturbance')                 % Uncomment for Sinusoid_Disturbance
title('Sinusoidal Disturbance Plant Response')  % Uncomment for Sinusoid_Plant_Response
% title('Disturbance Step')                       % Uncomment for Step_Disturbance
% title('Disturbance Step Plant Response')        % Uncomment for Step_Plant_Response

ylabel('Displacement (m)');
xlabel('Time (s)');
saveas(f, 'plots_mpc/disturbance', 'epsc');

%% LQR

% Weights selection for use in performance index
r1 = 4e+4;      % road holding weight
r2 = 5e+3;      % comfort weight
r3 = 0;         % control effort weight
Rxx = A(4,:)'*A(4,:) + diag([r1 0 r2 0]);
Rxu = -A(4,:)';
Ruu = 1 + r3;

% LQR optimal gain
[Kr,~] = lqr(A,B,Rxx,Ruu,Rxu);

% Closed Loop System
Ac = (A - B  * Kr);
Cc = (C - Du * Kr);

plant_lqr = ss(Ac, G, Cc, Dw);

plant_lqr = c2d(plant_lqr, Ts);

[y_lqr, time, stuff] = lsim(plant_lqr, v, T);

Ulqr = -(Kr*stuff')';

%%
plant = setmpcsignals(plant,'MD',2);

p = 40;     % Prediction horizon
m = 4;      % Control horizon
mpcobj = mpc(plant,Ts,p,m);

% Tire force [N]
mpcobj.OutputVariables(1).Min = -5000;
mpcobj.OutputVariables(1).Max = 5000;

% Suspension deflection [m]
mpcobj.OutputVariables(2).Min = -0.05;
mpcobj.OutputVariables(2).Max = 0.05;

% Body acceleration [m/s^2]
mpcobj.OutputVariables(3).Min = -5;
mpcobj.OutputVariables(3).Max = 5;

% Control action stays within actuator limits
mpcobj.ManipulatedVariables(1).Min = -5000;
mpcobj.ManipulatedVariables(1).Max = 5000;

% Scale factors
mpcobj.OutputVariables(2).ScaleFactor = 1e-5;
mpcobj.OutputVariables(3).ScaleFactor = 1e-3;

% WEIGHTS
% Control action weight (same as LQR)
mpcobj.Weights.ManipulatedVariables = Ruu;
mpcobj.Weights.ManipulatedVariablesRate = 1;
% State weights (same as LQR) [Tire force, suspension defl., body acc.]
mpcobj.Weights.OutputVariables = [r1 r2 0];

review(mpcobj);

%%
r = zeros(sim_time,3);
v = reshape(v,length(v),1);

params = mpcsimopt();
params.MDLookAhead = 'off';

mpcobj.Optimizer;
[Yas, Tas, Uas, XPas, XCas] = sim(mpcobj,sim_time,r,v,params);

f = figure('Position',[100 100 500 650]);
tl = tiledlayout(4,1);
nexttile;
hold on
plot(Tuc, Yuc(:,1),'color','#1caac7',"LineWidth",1.25);
plot(time, y_lqr(:,1),'color','#ff3366', "LineWidth",1.25);
plot(Tas, Yas(:,1),'color','#09512d',"LineWidth",1.25);
xlim([0,Tstop])
hold off;
% grid minor;
grid on;
title("Tire Force");
ylabel("Force (N)")
% xlabel("Time (s)")

nexttile;
hold on
plot(Tuc, Yuc(:,2),'color','#1caac7',"LineWidth",1.25);
plot(time, y_lqr(:,2),'color','#ff3366',"LineWidth",1.25);
plot(Tas, Yas(:,2),'color','#09512d',"LineWidth",1.25);
xlim([0,Tstop])
hold off;
% grid minor;
grid on
title("Suspension Deflection");
ylabel("Displacement (m)")
% xlabel("Time (s)")

nexttile;
hold on
plot(Tuc, Yuc(:,3),'color','#1caac7',"LineWidth",1.25);
plot(time, y_lqr(:,3),'color','#ff3366', "LineWidth",1.25);
plot(Tas, Yas(:,3),'color','#09512d',"LineWidth",1.25);
xlim([0,Tstop])
hold off;
% grid minor;
grid on
title("Body Acceleration");
ylabel("Acceleration (m/s$^2$)")
% xlabel("Time (s)")

nexttile;
hold on
plot(Tuc, Tuc*0,'color','#1caac7',"LineWidth",1.25);
plot(time, Ulqr,'color','#ff3366',"LineWidth", 1.25);
plot(Tas, Uas,'color','#09512d',"LineWidth",1.25);
xlim([0,Tstop])
hold off;
% grid minor;
grid on
title("Control Input");
% legend("No control","MPC", "location","northeast",'FontSize',7);
legend("No control","LQR","MPC", "location","northeast",'FontSize',7);
% legend("No control","LQR","MPC", "location","southeast",'FontSize',7);
ylabel("Force (kN)")
xlabel("Time (s)")

saveas(f, 'plots_mpc/MPC_outputs', 'epsc');
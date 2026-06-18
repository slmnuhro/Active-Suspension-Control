%% Vehicle Dynamics and Control
% Project Group 10
% H-infinity Controller for Active Suspension of a Quarter Car Model
% Code based on https://nl.mathworks.com/help/robust/gs/active-suspension-control-design.html

clc; 
clear; 
close all;
%% Create Directory for plots
if isunix
    command = 'rm -r plots';
elseif ispc
    command = 'rmdir /s /q plots';
elseif ismac
    command = 'rm -r plots';
end

status = system(command);
if (status ~= 0)
    disp("Failed to remove 'plots' directory. Please remove it manually")
end

command = 'mkdir plots';
status = system(command);
if (status ~= 0)
    disp("Failed to create 'plots' directory. Please create it manually")
end
%% Set interpreter and layout options
set(groot,'defaulttextinterpreter','latex');  
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultLineLineWidth',0.8)
set(groot,'defaultAxesFontSize',11)
set(groot,'defaultAxesFontWeight',"normal")

%% QC parameters
m_s = 410;                                           % sprung mass, kg (Body Car)
m_u = 45;                                            % unsprung mass, kg (Tire)
k_t = 230000;                                        % tire vertical stiffness, N/m
k_s = 25500;                                         % suspension vertical stiffness, N/m
wn_s = sqrt(k_s*k_t / (k_s + k_t)/m_s);              % sprung mass natural frequency
d_s = 0.3 * (2 * m_s * wn_s);                        % damping ratio

% frequency range determination
n_points = 5000;
w = linspace(0.1,100,n_points)*2*pi;

%%
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

%%
LTI.A = A;
LTI.B = [G B]; % u1: disturbance r, u2: fs
LTI.C = C;
LTI.D = [Dw Du];

qcar = ss(LTI.A, LTI.B, LTI.C, LTI.D);

% States
% Tire Deflection, Tire Velocity, Body Travel, Body Velocity

qcar.StateName = {'Tire Deflection (m)'; 'Tire Velocity (m/s)';... 
                  'Body Travel (m)'; 'Body Velocity (m/s)'};

qcar.InputName = {'r';'fs'};

% Outputs - Tire Force, Suspension Deflection, Body Acceleration
qcar.OutputName = {'ft'; 'sd'; 'ab'};

%%
figure;
bodemag(qcar({'ab','sd'},'r'),'b',qcar({'ab','sd'},'fs'),'r',{1 100});
legend('Road disturbance (r)','Actuator force (fs)','location','SouthWest')
title({'Gain from road dist (r) and actuator force (fs) ';
       'to body accel (ab) and suspension travel (sd)'});
grid minor
%%
%ActNom = tf(1,[1/60 1]);
ActNom = tf(1,[1/m_u 1]);

Wunc = makeweight(0.40,15,3);
unc = ultidyn('unc',[1 1],'SampleStateDim',5);
Act = ActNom*(1 + Wunc*unc);
Act.InputName = 'u';
Act.OutputName = 'fs';

rng('default');
f = figure("Name","Uncertainty");
bode(Act,'b',Act.NominalValue,'r+',logspace(-1,3,120));
title("Uncertainty in the Actuator Model for various Frequencies");
grid minor
saveas(f, "plots/Uncertain_act",'epsc');

%%
Wroad = ss(0.07);  Wroad.u = 'd1';   Wroad.y = 'r';
Wact = 0.8*tf([1 50],[1 500]);  Wact.u = 'u';  Wact.y = 'e1';
Wd2 = ss(0.01);  Wd2.u = 'd2';   Wd2.y = 'Wd2';
Wd3 = ss(0.5);   Wd3.u = 'd3';   Wd3.y = 'Wd3';

HandlingTarget = 0.04 * tf([1/8 1],[1/80 1]);
ComfortTarget = 0.4 * tf([1/0.45 1],[1/150 1]);

Targets = [HandlingTarget ; ComfortTarget];
figure;
bodemag(qcar({'sd','ab'},'r')*Wroad,'b',Targets,'r--',{1,1000}), grid
title('Response to road disturbance')
legend('Open-loop','Closed-loop target');
grid minor
%%
% Three design points
beta = reshape([0.01 0.5 0.99],[1 1 3]);
Wsd = beta / HandlingTarget;
Wsd.u = 'sd';  Wsd.y = 'e2';
Wab = (1-beta) / ComfortTarget;
Wab.u = 'ab';  Wab.y = 'e3';

sdmeas  = sumblk('y1 = sd+Wd2');
abmeas = sumblk('y2 = ab+Wd3');
ICinputs = {'d1';'d2';'d3';'u'};
ICoutputs = {'e1';'e2';'e3';'y1';'y2'};
qcaric = connect(qcar((2:3),:),Act,Wroad,Wact,Wab,Wsd,Wd2,Wd3,...
                 sdmeas,abmeas,ICinputs,ICoutputs);

ncont = 1; % one control signal, u
nmeas = 2; % two measurement signals, sd and ab
K = ss(zeros(ncont,nmeas,3));
gamma = zeros(3,1);
for i=1:3
   [K(:,:,i),~,gamma(i)] = hinfsyn(qcaric(:,:,i),nmeas,ncont);
end

gamma;
%%
% Closed-loop models
K.u = {'sd','ab'};  K.y = 'u';
CL = connect(qcar,Act.Nominal,K,'r',{'ft';'sd';'ab'});

f = figure("Name","Bode of 3 Modes");
bodemag(qcar(:,'r'),'b', CL(:,:,1),'r-.', ...
   CL(:,:,2),'m-.', CL(:,:,3),'k-.',{1,140}), grid
legend('Open-loop','Comfort','Balanced','Handling','location','SouthEast')
title({'Tire Force, suspension deflection, body acceleration due to road'})
grid minor

saveas(f, 'plots/bode_hinf_3modes','epsc');
%%

% Road disturbance
t = 0:0.0025:5;
roaddist = zeros(size(t));
roaddist(1:101) = 0.025*(1-cos(8*pi*t(1:101)));

% Closed-loop model
%SIMK = connect(qcar,Act.Nominal,K,'r',{'sd';'ft';'ab';'fs'});
SIMK = connect(qcar,Act.Nominal,K,'r',{'ft';'sd';'ab';'fs'});

% Simulate
p1 = lsim(qcar(:,1),roaddist,t);
y1 = lsim(SIMK(1:4,1,1),roaddist,t);
y2 = lsim(SIMK(1:4,1,2),roaddist,t);
y3 = lsim(SIMK(1:4,1,3),roaddist,t);

%% Comfort Controller
f = figure("Name","Comfort Controller");
% Plot results
t2 = tiledlayout(2,2);
sgtitle("Comfort Mode","Interpreter","Latex")
nexttile
plot(t,p1(:,1),'b',t,y1(:,1),'r--'); %t,y2(:,3),'m.',t,y3(:,3),'k.',
title('Tire Force'), ylabel('N');
xlim([0 4]);
legend('Open-loop', '$H_\infty$', 'Location','northeast');
grid minor

nexttile
plot(t,roaddist,'g',t,p1(:,2),'b',t,y1(:,2),'r--'); %,t,y2(:,1),'m.',t,y3(:,1),'k.',
title('Suspension Deflection'), ylabel('$s_d (m)$');
legend('Disturbance', 'Open-loop', '$H_\infty$', 'Location','northeast');
xlim([0 4]);
grid minor

nexttile
plot(t,p1(:,3),'b',t,y1(:,3),'r--'); %t,y2(:,3),'m.',t,y3(:,3),'k.',
title('Body acceleration'), ylabel('$a_b (m/s^2)$');
legend('Open-loop', '$H_\infty$', 'Location','northeast');
xlim([0 4]);
grid minor

nexttile
plot(t,y1(:,4),'r'); %t,y2(:,3),'m.',t,y3(:,3),'k.',
title('Control force'), xlabel('Time (s)'), ylabel('$f_s (kN)$');
xlim([0 4]);
grid minor

saveas(f, 'plots/comfort_controller','epsc');

%% Balanced Controller
f = figure("Name","Balanced Controller");
% Plot results
t3 = tiledlayout(2,2);
sgtitle("Balanced Mode","Interpreter","Latex")
nexttile
plot(t,p1(:,1),'b',t,y2(:,1),'r--');
title('Tire Force'), ylabel('N');
xlim([0 4]);
legend('Open-loop', '$H_\infty$', 'Location','northeast');
grid minor

nexttile
plot(t,roaddist,'g',t,p1(:,2),'b',t,y2(:,2),'r--');
title('Suspension Deflection'), ylabel('$s_d (m)$');
legend('Disturbance', 'Open-loop', '$H_\infty$', 'Location','northeast');
xlim([0 4]);
grid minor

nexttile
plot(t,p1(:,3),'b',t,y2(:,3),'r--');
title('Body acceleration'), ylabel('$a_b (m/s^2)$');
legend('Open-loop', '$H_\infty$', 'Location','northeast');
xlim([0 4]);
grid minor

nexttile
plot(t,y2(:,4),'r');
title('Control force'), xlabel('Time (s)'), ylabel('$f_s (kN)$');
xlim([0 4]);
grid minor


saveas(f, 'plots/balanced_controller','epsc');

%% Handling Controller
f = figure("Name","Handling Controller");
% Plot results
t4 = tiledlayout(2,2);
sgtitle("Handling Mode","Interpreter","Latex")
nexttile
plot(t,p1(:,1),'b',t,y3(:,1),'r--');
title('Tire Force'), ylabel('N');
xlim([0 4]);
legend('Open-loop', '$H_\infty$', 'Location','northeast');
grid minor

nexttile
plot(t,roaddist,'g',t,p1(:,2),'b',t,y3(:,2),'r--');
title('Suspension Deflection'), ylabel('$s_d (m)$');
legend('Disturbance', 'Open-loop', '$H_\infty$', 'Location','northeast');
xlim([0 4]);
grid minor

nexttile
plot(t,p1(:,3),'b',t,y3(:,3),'r--');
title('Body acceleration'), ylabel('$a_b (m/s^2)$');
legend('Open-loop', '$H_\infty$', 'Location','northeast');
xlim([0 4]);
grid minor

nexttile
plot(t,y3(:,4),'r'); 
title('Control force'), xlabel('Time (s)'), ylabel('$f_s (kN)$');
xlim([0 4]);
grid minor


saveas(f, 'plots/handling_controller','epsc');

%%

f = figure("Name","Comparison of All Modes","Position",[500 200 500 650]);
% Plot results
t5 = tiledlayout(4,1);
sgtitle("Comparison","Interpreter","Latex")
nexttile
hold on
plot(t,y1(:,1),'r',t,y2(:,1),'b')
plot(t,y3(:,1),'Color',"#4DBEEE")
plot(t,p1(:,1),'k--')
title('Tire Force'), ylabel('N');
xlim([0 3.5]);
legend('Comfort', 'Balanced', 'Handling','Open-loop', 'Location','northeast');
grid minor

nexttile
hold on
plot(t,roaddist,'g')
plot(t,y1(:,2),'r',t,y2(:,2),'b')
plot(t,y3(:,2),'Color',"#4DBEEE")
plot(t,p1(:,2),'k--')
title('Suspension Deflection'), ylabel('$s_d (m)$');
legend('Disturbance','Comfort', 'Balanced', 'Handling','Open-loop', 'Location','northeast');
xlim([0 3.5]);
grid minor

nexttile
hold on
plot(t,y1(:,3),'r',t,y2(:,3),'b')
plot(t,y3(:,3),'Color',"#4DBEEE")
plot(t,p1(:,3),'k--')
title('Body Acceleration'), ylabel('$a_b (m/s^2)$');
legend('Comfort', 'Balanced', 'Handling','Open-loop', 'Location','northeast');
xlim([0 3.5]);
grid minor

nexttile
hold on
plot(t,y1(:,4),'r',t,y2(:,4),'b')
plot(t,y3(:,4),'Color',"#4DBEEE");
title('Control Input'), xlabel('Time (s)'), ylabel('$f_s (kN)$');
legend('Comfort', 'Balanced', 'Handling','Location','northeast');
xlim([0 3.5]);
grid minor


saveas(f, 'plots/mode_comparison_controller','epsc');
%%

[Krob,rpMU] = musyn(qcaric(:,:,1),nmeas,ncont);
%%
% Closed-loop model (nominal)
Krob.u = {'sd','ab'};
Krob.y = 'u';
SIMKrob = connect(qcar,Act.Nominal,Krob,'r',{'ft';'sd';'ab';'fs'});

% Simulate
p1 = lsim(qcar(:,1),roaddist,t);
y1_r = lsim(SIMKrob(1:4,1),roaddist,t);

% Plot results
f = figure('Position',[200 200 500 650]);
sgtitle("Comfort Mode Comparison")
subplot(411)
plot(t,y1(:,1),'b',t,y1_r(:,1),'r',t,p1(:,1),'k--')
title('Tire Force'), ylabel('$f_t (N)$')
legend('$H_\infty$', '$\mu$-syn','Open-loop', 'Location','northeast');
xlim([0,3])
grid minor
subplot(412)
plot(t,roaddist,'g',t,y1(:,2),'b',t,y1_r(:,2),'r',t,p1(:,2),'k--')
title('Suspension Deflection'), xlabel('Time (s)'), ylabel('$s_d (m)$')
legend('Disturbance','$H_\infty$', '$\mu$-syn','Open-loop', 'Location','northeast');
xlim([0,3])
grid minor
subplot(413)
plot(t,y1(:,3),'b',t,y1_r(:,3),'r',t,p1(:,3),'k--')
title('Body Acceleration'), ylabel('$a_b (m/s^2)$')
legend('$H_\infty$', '$\mu$-syn','Open-loop', 'Location','northeast');
xlim([0,3])
grid minor
subplot(414)
plot(t,y1(:,4),'b',t,y1_r(:,4),'r')
title('Control Input'), xlabel('Time (s)'), ylabel('$f_s (kN)$')
legend('$H_\infty$', '$\mu$-syn', 'Location','northeast');
xlim([0,3])
grid minor
saveas(f, "plots/Robust_Controller", 'epsc');

%%
rng('default'), nsamp = 100;

% Uncertain closed-loop model with comfort H-infinity controller
CLU = connect(qcar,Act,K(:,:,1),'r',{'ft','sd','ab'});
f = figure;
lsim(usample(CLU,nsamp),'b',CLU.Nominal,'r',roaddist,t)
title('$H_\infty$ Controller on the Generalized Plant with Uncertainty','Interpreter','latex')
legend('Perturbed','Nominal','location','SouthEast')
grid minor
saveas(f, "plots/Hinf_Controller_Comfort", 'epsc');
%%

% Uncertain closed-loop model with comfort robust controller
CLU = connect(qcar,Act,Krob,'r',{'ft','sd','ab'});
f = figure;
lsim(usample(CLU,nsamp),'b',CLU.Nominal,'r',roaddist,t)
title('$\mu$-synthesis Controller on the Generalized Plant with Uncertainty','Interpreter','latex')
legend('Perturbed','Nominal','location','SouthEast')
grid minor
saveas(f, "plots/Robust_Controller_Comfort", 'epsc');
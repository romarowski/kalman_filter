%% 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter', 'latex')
set(groot, 'defaultTextFontSize', 16), set(groot, 'defaultColorbarFontSize', 16)
clc, clear;
%%
load project_data
load secndnodelay.mat

[yaw, pitch, roll] = quat2angle(q_body2ned);
dcm = quat2dcm(q_body2ned);

acel_NED = zeros(13901,3);

for i = 1:13901
    acel_NED(i,:) = dcm(:,:,i)'*acceleration_body(i,:)';
end


%%
sys1c = idss(secndordernodelay);
sys1d = c2d(sys1c,0.01,'zoh');

%% compare discretized system
sys1d.InputName = 'y_0';
sys1d.OutputName = {'y', 'a_y'};

sys1c.InputName = 'y_0';
sys1c.OutputName = {'y', 'a_y'};
step(sys1c,'b',sys1d,'r--')
legend('Continuous', 'Discrete')

%observability

Ob = obsv(sys1d);
unobs = rank(Ob) - length(sys1d.A);


%% kalman 

sys1d.InputName = 'y_0';
sys1d.OutputName = {'y', 'a_y'};
Q =1000;
R = [0.00066004788240179 0; 0 0.0135502962738042]; 
%R = [.01^2 0; 0 0.0135502962738042]; 
Plant = ss(sys1d);
[kalmf,L,P,M] = kalman(Plant,Q,R);


%% RUNTHIS

u = setpoint_position_ned(:,2);
y = position_ned(:,2);
v = velocity_ned(:,2);
a = acel_NED(:,2);
measure = [y, a];

%%
x_kalm = lsim(kalmf, measure, t);

%%
figure
plot(t, v, 'k')
hold on
plot(t, x_kalm(:,4), 'r--')


%% CV
ve = x_kalm(:,4);

fit2 = goodnessOfFit(v, ve, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(v-ve)/var(v))*100

%%
plot(t, v-ve)

%%
%plot(t,a)


%% adhoc kalman

% 
% xhat = [position_ned(1,2);velocity_ned(1,2)];
% P = 0;
% A = sys1d.A;
% B = sys1d.B;
% C = sys1d.C;
% Q = 0.01^2;
% %R = [.01^2 0; 0 .036^2]; 
% R = [0.00066004788240179 0; 0 0.0135502962738042]; 
% 
% xhatOut = zeros(2, 13901);
% for i=1:13901
%     u=setpoint_position_ned(i,2);
%     meas = measure(i,:)';
%     
%     % Propagate the state estimate and covariance matrix:
%     xhat = A*xhat + B*u;
%     P = A*P*A' + Q;
%     % Calculate the Kalman gain
%     K = P*C'/(C*P*C' + R);
%     % Calculate the measurement residual
%     resid = meas - C*xhat;
%     % Update the state and error covariance estimate
%     xhat = xhat + K*resid;
%     P = (eye(size(K,1))-K*C)*P;
%     % Post the results
%     xhatOut(:,i) = xhat;
%     yhatOut = C*xhatOut;
% end

%%
y0=setpoint_position_ned(:,2);
v = velocity_ned(:,2);
X = [0.099966628233391e-1        0.0999597602828939e-1        0.0999644859861622e-1*2];% 0.00066004788240179 0.0135502962738042];
sys1d=ss(sys1d);
[covari,FVAL]= fmincon(@(x) kalman_cost(x, sys1d, v, y0, measure), X, [],[],[],[],zeros(3,1),[]);

%% GA

% y0=setpoint_position_ned(:,2);
% v = velocity_ned(:,2);
% X = [.85 .84 .85 0.00066004788240179 0.0135502962738042];
% sys1d=ss(sys1d);
% [covari,FVAL]= ga(@(x) kalman_cost(x, sys1d, v, y0, measure), 5,  [],[],[],[],zeros(5,1),ones(5,1));

%% adhoc kalman lovera notation



xhat = [position_ned(1,2);velocity_ned(1,2)];
xhat = [0;0];
F = sys1d.A;
G = sys1d.B;
H = sys1d.C;
%W = 0.01^2 *ones(2);
%covari=  [9.99e-5 1e-4 9.99e-5];
W = [covari(1)      covari(2) ;     covari(2)      covari(3)];
Pnew = 0;
%R = [.01^2 0; 0 .036^2]; 
V = [0.00066004788240179 0; 0 0.0135502962738042]; 
%V  = [covari(4) 0;0 covari(5)];
y0=setpoint_position_ned(:,2);
states = zeros(2, 13901);
sigmayvec = zeros(1, 13901);
errorNorm = zeros(length(t),1);
innov = zeros(2, 13901);
innovcovar = zeros(4, 13901);
q = zeros(1, 13901);
for i=1:length(t)
    u=y0(i);
    meas = measure(i,:)';
    % State estimate and error covariance extrapolation:
    xhat = F*xhat + G*u;
    Ppred = F*Pnew*F' + W;
    % Innovation covariance
    S = (H*Ppred*H' + V);
    innovcovar(:,i) = reshape(S,[4,1]);
    % Gain update:
    K = Ppred*H'/S;
    % Calculate the innovation
    innov(:, i) = meas - H*xhat;
    q(:,i) = innov(:,i)'*inv(S)*innov(:,i);
    % State estimate and error covariance update:
    xhat = xhat + K*innov(:,i);
    Pnew = (Ppred-K*H*Ppred);
    % Results
    states(:,i) = xhat;
end
ve = states(2, :)';
%% 

S = V + H*Ppred*H';


%%
ry = xcorr(innov(1,:));%, 'normalized');
ray = xcorr(innov(2,:));%, 'normalized');
sumQ =sum(q,2);

%%
plot(ry)
figure
plot(ray)

%%
figure
plot(t,q)


%%
N =length(t);
plot(t,ry(N:2*N-1)/ry(N),'b');
hold on
plot(-t,ry(N:2*N-1)/ry(N),'b');




%% innovation pos
figure
plot(t,3*sqrt(S(1,1))*ones(length(t),1), 'r--')
hold on
plot(t, innov(1,:), 'b')
plot(t,-3*sqrt(S(1,1))*ones(length(t),1), 'r--')

%% innovation postax

figure
subplot 211

plot(t,3*sqrt(S(1,1))*ones(length(t),1), 'r--')
title('Innovation Behaviour')
hold on
plot(t, innov(1,:), 'b')
plot(t,-3*sqrt(S(1,1))*ones(length(t),1), 'r--')
xlabel('Time [s]')
legend( '$\pm3\sigma$', '$y -\hat{y}$')
grid on
ylabel('Position')
subplot 212
plot(t, 3*sqrt(S(2,2))*ones(length(t),1), 'r--')
hold on
plot(t, innov(2,:), 'b')
plot(t, -3*sqrt(S(2,2))*ones(length(t),1), 'r--')
grid on
legend('$\pm3\sigma$','$a_y -\hat{a}_y $')
xlabel('Time [s]'), ylabel('Acceleration')

%% innovation acel
figure
plot(t, 3*sqrt(S(2,2))*ones(length(t),1), 'r--')
hold on
plot(t, innov(2,:), 'b')
plot(t, -3*sqrt(S(2,2))*ones(length(t),1), 'r--')

%% verify kalman
figure
plot(t, v, 'b')
hold on
plot(t, states(2, :), 'r--')
legend('Measured', 'Kalman')
grid on
ylabel('$v_y$ [$\frac{m}{s}$]')
xlabel('Time [s]')
title('Lateral velocity estimation')


%% animate kalman
%h = animatedline('Color', 'b', 'LineStyle', 'none', 'Marker','x');

%h1 = animatedline('Color', 'r', 'LineStyle', 'none', 'Marker', 'o');

h = animatedline('Color', 'b' );

h1 = animatedline('Color', 'r', 'LineStyle', '--');


%%
vplot=v(1:20:end);
veplot=ve(1:20:end);
tplot = t(1:20:end);

%%
axis([0,140,-1.6,1.6])
for k = 1:length(tplot)
    addpoints(h, tplot(k), vplot(k));
    addpoints(h1,tplot(k), veplot(k));
    legend('Measured', 'Kalman')
    grid on
    ylabel('$v_y$ [$\frac{m}{s}$]')
    xlabel('Time [s]')
    title('Lateral velocity estimation')
    drawnow limitrate
end

%% CV
ve = states(2, :)';

fit2 = goodnessOfFit(v, ve, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(v-ve)/var(v))*100



%% measurement error

e = v-ve;
%e = e -mean (e);
sigmay = sqrt(Ppred(2,2));
covplus = ones(length(t),1)*3*sigmay;
covminus = -ones(length(t),1)*3*sigmay;
J1 = (var(v-ve)-Ppred(2,2))

%%
hist(e)
%%
figure
plot(t,e, 'b')
hold on 
plot(t, covplus, 'r--')
plot(t, covminus, 'r--')
grid on
legend('$e=v_m-v_K$', '$\pm3\sigma$')
xlabel('Time [s]')
ylabel('$v_y$ [$\frac{m}{s}$]')
title('Estimation error')
%%
% figure
% plot(t,e, 'b')
% hold on 
% plot(t, sqrt(sigmayvec)*3, 'r--')
% plot(t, -sqrt(sigmayvec)*3, 'r--')
% grid on
% legend('$e=v_m-v_K$', '$\pm3\sigma$')
% xlabel('Time [s]')
% ylabel('$v_y$ [$\frac{m}{s}$]')
% title('Estimation error')
%% 
x = t;                   
y = ve;
xconf = [x x(end:-1:1)] ;         
yconf = [y'+3*sigmay y(end:-1:1)'-3*sigmay];

figure
p = fill(xconf,yconf,'red');
p.FaceColor = [1 0.8 0.8];      
p.EdgeColor = 'none';   

hold on
plot(x,y,'b-')

grid on
legend('$\pm3\sigma$','Kalman')
title('99\% confidence interval')
xlabel('Time [s]')
ylabel('$v_y$ [$\frac{m}{s}$]')

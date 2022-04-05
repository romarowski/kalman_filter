%% 
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter', 'latex')
set(groot, 'defaultTextFontSize', 16), set(groot, 'defaultColorbarFontSize', 16)
clc, clear;

%%
reset(groot)
%%
load project_data
load tf1

[yaw, pitch, roll] = quat2angle(q_body2ned);
dcm = quat2dcm(q_body2ned);

acel_NED = zeros(13901,3);

for i = 1:13901
    acel_NED(i,:) = dcm(:,:,i)'*acceleration_body(i,:)';
end

%% compress data different values
u = setpoint_position_ned(:,2);
y = position_ned(:,2);

t1=5; t2=23.98; t3=52.35; t4=75.30;
t5=100; t6=123.30;
Ts=.01;
utrain = [u(t1/Ts:t2/Ts);u(t3/Ts:t4/Ts)];
utest = u(t5/Ts:t6/Ts);
%tnew = [t(t1/Ts:t2/Ts), t(t3/Ts:t4/Ts),t(t5/Ts:t6/Ts)];
ttrain = 0:Ts:length(utrain)*Ts-Ts;
ttest = 0:Ts:length(utest)*Ts-Ts;
ytrain = [y(t1/Ts:t2/Ts); y(t3/Ts:t4/Ts)];
ytest = y(t5/Ts:t6/Ts);

subplot 211
plot(ttest,ytest)
title('test')
subplot 212
plot(ttest,utest);
figure
subplot 211
plot(ttrain,ytrain)
title('train')
subplot 212
plot(ttrain,utrain)
%%
p1 = (t2-t1)/Ts;
p2 = p1 + (t4-t3)/Ts;
p3 = p2 + (t6-t5)/Ts;

%%
figure
subplot 221
plot(ttrain,ytrain)
grid on
ylabel('$y$')
title('Train')
subplot 223
plot(ttrain,utrain)
grid on
ylabel('$y_0$')
subplot 222
plot(ttest,ytest)
grid on
title('Test')
subplot 224
plot(ttest,utest);
grid on


%% 
acel = acel_NED(:,2);
%acel = [acel(t1/Ts:t2/Ts); acel(t3/Ts:t4/Ts);acel(t5/Ts:t6/Ts)];
aceltrain = [acel(t1/Ts:t2/Ts); acel(t3/Ts:t4/Ts)];
aceltest=acel(t5/Ts:t6/Ts);

%%
vel=velocity_ned(:,2);
veltrain = [vel(t1/Ts:t2/Ts); vel(t3/Ts:t4/Ts)];
veltest = vel(t5/Ts:t6/Ts);


%%
subplot 411
plot(ttrain, utrain)
subplot 412
plot(ttrain, ytrain)
subplot 413
plot(ttrain, veltrain)
subplot 414
plot(ttrain, aceltrain)

%%
subplot 411
plot(ttest, utest)
subplot 412
plot(ttest, ytest)
subplot 413
plot(ttest, veltest)
subplot 414
plot(ttest, aceltest)
%% compress data for measurement variance acel
first = 1:4.57/Ts;
secnd = 28.53/Ts:37.03/Ts;
third = 125.01/Ts:139/Ts;

u = setpoint_position_ned(:,2);
y = position_ned(:,2);
acel = acel_NED(:,2);

avar = [acel(first) ;acel(secnd) ;acel(third)] ;
%tvar = [t(first)'; t(secnd)' ;t(third)'];
uvar = [u(first); u(secnd); u(third)];

tvar = 0:Ts: length(uvar)*Ts-Ts;
figure

subplot 211
plot(tvar, avar)
title('Zero input accelerometer response')
ylabel('$a_y$ [$\frac{m}{s^2}$]')
xlabel('Time [s]')
grid on
subplot 212
plot(tvar, uvar)
xlabel('Time [s]')
ylabel('$y_0$ [m]')
grid on


mean(avar)
var(avar)


%%
avar = avar-mean(avar);

var(avar)
%%
subplot 311
plot(tvar, avar)
title('Zero input accelerometer response')
ylabel('$a_y$ [$\frac{m}{s^2}$]')
xlabel('Time [s]')
grid on
subplot 312
plot(tvar, yvar)
title('Zero input motion capture response')
ylabel('$y$ [m]')
xlabel('Time [s]')
grid on
subplot 313
plot(tvar, uvar)
title('Input')
xlabel('Time [s]')
ylabel('$y_0$ [m]')
grid on
%% compress data for measurement variance pos
first = 1:4.57/Ts;
secnd = 28.53/Ts:37.03/Ts;
third = 125.01/Ts:139/Ts;

u = setpoint_position_ned(:,2);
y = position_ned(:,2);
acel = acel_NED(:,2);

avar = [acel(first) ;acel(secnd) ;acel(third)] ;
%tvar = [t(first)'; t(secnd)' ;t(third)'];
uvar = [u(first); u(secnd); u(third)];
yvar = [y(first); y(secnd); y(third)];
tvar = 0:Ts: length(uvar)*Ts-Ts;

subplot 211
plot(tvar, yvar)
title('Zero input motion capture response')
ylabel('$y$ [m]')
xlabel('Time [s]')
grid on
subplot 212
plot(tvar, uvar)
xlabel('Time [s]')
ylabel('$y_0$ [m]')
grid on

mean(yvar)
var(yvar)

%% compress data for measurement variance vel
first = 1:4.57/Ts;
secnd = 28.53/Ts:37.03/Ts;
third = 125.01/Ts:139/Ts;

u = setpoint_position_ned(:,2);
y = position_ned(:,2);
v = velocity_ned(:,2);
acel = acel_NED(:,2);

avar = [acel(first) ;acel(secnd) ;acel(third)] ;
%tvar = [t(first)'; t(secnd)' ;t(third)'];
uvar = [u(first); u(secnd); u(third)];
yvar = [y(first); y(secnd); y(third)];
vvar = [v(first); v(secnd); v(third)];
tvar = 0:Ts: length(uvar)*Ts-Ts;

subplot 211
plot(tvar, vvar)
title('Zero input motion capture response')
ylabel('$v_y$ [m]')
xlabel('Time [s]')
grid on
subplot 212
plot(tvar, uvar)
xlabel('Time [s]')
ylabel('$y_0$ [m]')
grid on

mean(vvar)
var(vvar)

%%
figure
subplot 311
plot(t, u)
subplot 312
plot(t,y)
subplot 313
plot(t, acel)
%% filter acceleration

a_filter = lowpass(aceltrain, 1.75, 100);

figure

plot(ttrain, aceltrain, 'b')
hold on
plot(ttrain, a_filter,'r--')

%% filter try2

fc = 150;
Wn = (2/Fs)*fc;
b = fir1(20,Wn,'low',kaiser(21,3));

fvtool(b,1,'Fs',Fs)


%%
figure
pwelch([aceltrain, a_filter])


%% greyest

Ts=.01;
data=iddata([ytrain aceltrain], utrain, Ts);

odefun = 'lateral_dynamics';
opt = greyestOptions('OutputWeight', [1/0.00066004788240179 0; 0 1/0.0135502962738042]);
parameters = {2;2};
fcn_type = 'c';
%optional_args = 0.25; 
init_sys = idgrey(odefun, parameters, fcn_type);
sys = greyest(data,init_sys, opt);
figure
compare(data,sys)

%% mean and simga

[mean, sigma] =getpvec(sys, 'free');


%% bode
sys.InputName = 'y_0';
sys.OutputName = {'y';'a_y'};
bode(sys, 'k')
grid on
%% CV
%G = tf1;
y_est = lsim(sys,utest,ttest);
y_est = y_est(:,1);
fit2 = goodnessOfFit(ytest, y_est, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(ytest-y_est)/var(ytest))*100

plot(ttest, ytest, 'k')
hold on
plot(ttest, y_est, 'r--')
grid on
title('Test dataset')
xlabel('Time [s]')
ylabel('$y$ Position')
legend('Real','Model')




%% FIT

y_est = lsim(sys,utest,ttest);
y_est = y_est(:,1);
fit2 = goodnessOfFit(ytest, y_est, 'NMSE')
fit2_love = (1-fit2)*100

FITreal =(1-norm(ytest-y_est)^2/norm(y_est)^2)*100
vaf2 = (1-var(ytest-y_est)/var(ytest))*100

plot(ttest, ytest, 'k')
hold on
plot(ttest, y_est, 'r--')
grid on
title('Test dataset')
xlabel('Time [s]')
ylabel('$y$ Position')
legend('Real','Model')


%% Train
y_est = lsim(sys,utrain,ttrain);
y_est = y_est(:,1);
fit2 = goodnessOfFit(ytrain, y_est, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(ytrain-y_est)/var(ytrain))*100



%% CV acel

a_est = lsim(sys, utest, ttest);
a_est = a_est(:,2);

fit2 = goodnessOfFit(aceltest, a_est, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(aceltest-a_est)/var(aceltest))*100

figure
plot(ttest, aceltest, 'k')
hold on
plot(ttest, a_est, 'r--')
grid on
title('Test dataset')
xlabel('Time [s]')
ylabel('$a_y$ [m/$s^2$]')
legend('Real','Model')



%% Train acel
a_est = lsim(sys, utrain, ttrain);
a_est = a_est(:,2);

fit2 = goodnessOfFit(aceltrain, a_est, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(aceltrain-a_est)/var(aceltrain))*100

%% discrete
Ts = .01;
sysc = ss(sys);
sysd = c2d(sysc, Ts, 'zoh');


%% CV discrete position
y_est = lsim(sysd,utest,ttest);
y_est = y_est(:,1);
fit2 = goodnessOfFit(ytest, y_est, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(ytest-y_est)/var(ytest))*100

plot(ttest, ytest, 'k')
hold on
plot(ttest, y_est, 'r--')
grid on
title('Test dataset')
xlabel('Time [s]')
ylabel('$y$ Position')
legend('Real','Model')

%% CV discrete velocity
[y,tOut,x] =  lsim(sysd,utest,ttest);
y_est = x(:,2);
fit2 = goodnessOfFit(veltest, y_est, 'NMSE')
fit2_love = (1-fit2)*100
vaf2 = (1-var(veltest-y_est)/var(veltest))*100

plot(ttest, veltest, 'k')
hold on
plot(ttest, y_est, 'r--')
grid on
title('Test dataset')
xlabel('Time [s]')
ylabel('$y$ Position')
legend('Real','Model')

%%  uncertainty analyss

N = 500;
unc_params = zeros(N, 3);
rng default
[mean, sigma] = getpvec(sys, 'free');
for i=1:2
unc_params(:,i) = normrnd(mean(i), sigma(i), [1, N]);
end



sss = ss(zeros(2,1,N));
for i=1:N
    ksi = unc_params(i,1);
    om = unc_params(i,2);
    [A,B,C,D] = lateral_dynamics(ksi, om, Ts);
    sss(:,:,i) =  ss(A,B,C,D);        
          
end

%% poles
iopzmap(sss(:,:,1),sss(:,:,2),sss(:,:,3),sss(:,:,4),sss(:,:,5),sss(:,:,6),sss(:,:,7),sss(:,:,8),sss(:,:,9),sss(:,:,10),sss(:,:,11),sss(:,:,12),sss(:,:,13),sss(:,:,14),sss(:,:,15),sss(:,:,16),sss(:,:,17),sss(:,:,18),sss(:,:,19),sss(:,:,20),sss(:,:,21),sss(:,:,22),sss(:,:,23),sss(:,:,24),sss(:,:,25),sss(:,:,26),sss(:,:,27),sss(:,:,28),sss(:,:,29),sss(:,:,30),sss(:,:,31),sss(:,:,32),sss(:,:,33),sss(:,:,34),sss(:,:,35),sss(:,:,36),sss(:,:,37),sss(:,:,38),sss(:,:,39),sss(:,:,40),sss(:,:,41),sss(:,:,42),sss(:,:,43),sss(:,:,44),sss(:,:,45),sss(:,:,46),sss(:,:,47),sss(:,:,48),sss(:,:,49),sss(:,:,50),sss(:,:,51),sss(:,:,52),sss(:,:,53),sss(:,:,54),sss(:,:,55),sss(:,:,56),sss(:,:,57),sss(:,:,58),sss(:,:,59),sss(:,:,60),sss(:,:,61),sss(:,:,62),sss(:,:,63),sss(:,:,64),sss(:,:,65),sss(:,:,66),sss(:,:,67),sss(:,:,68),sss(:,:,69),sss(:,:,70),sss(:,:,71),sss(:,:,72),sss(:,:,73),sss(:,:,74),sss(:,:,75),sss(:,:,76),sss(:,:,77),sss(:,:,78),sss(:,:,79),sss(:,:,80),sss(:,:,81),sss(:,:,82),sss(:,:,83),sss(:,:,84),sss(:,:,85),sss(:,:,86),sss(:,:,87),sss(:,:,88),sss(:,:,89),sss(:,:,90),sss(:,:,91),sss(:,:,92),sss(:,:,93),sss(:,:,94),sss(:,:,95),sss(:,:,96),sss(:,:,97),sss(:,:,98),sss(:,:,99),sss(:,:,100),sss(:,:,101),sss(:,:,102),sss(:,:,103),sss(:,:,104),sss(:,:,105),sss(:,:,106),sss(:,:,107),sss(:,:,108),sss(:,:,109),sss(:,:,110),sss(:,:,111),sss(:,:,112),sss(:,:,113),sss(:,:,114),sss(:,:,115),sss(:,:,116),sss(:,:,117),sss(:,:,118),sss(:,:,119),sss(:,:,120),sss(:,:,121),sss(:,:,122),sss(:,:,123),sss(:,:,124),sss(:,:,125),sss(:,:,126),sss(:,:,127),sss(:,:,128),sss(:,:,129),sss(:,:,130),sss(:,:,131),sss(:,:,132),sss(:,:,133),sss(:,:,134),sss(:,:,135),sss(:,:,136),sss(:,:,137),sss(:,:,138),sss(:,:,139),sss(:,:,140),sss(:,:,141),sss(:,:,142),sss(:,:,143),sss(:,:,144),sss(:,:,145),sss(:,:,146),sss(:,:,147),sss(:,:,148),sss(:,:,149),sss(:,:,150),sss(:,:,151),sss(:,:,152),sss(:,:,153),sss(:,:,154),sss(:,:,155),sss(:,:,156),sss(:,:,157),sss(:,:,158),sss(:,:,159),sss(:,:,160),sss(:,:,161),sss(:,:,162),sss(:,:,163),sss(:,:,164),sss(:,:,165),sss(:,:,166),sss(:,:,167),sss(:,:,168),sss(:,:,169),sss(:,:,170),sss(:,:,171),sss(:,:,172),sss(:,:,173),sss(:,:,174),sss(:,:,175),sss(:,:,176),sss(:,:,177),sss(:,:,178),sss(:,:,179),sss(:,:,180),sss(:,:,181),sss(:,:,182),sss(:,:,183),sss(:,:,184),sss(:,:,185),sss(:,:,186),sss(:,:,187),sss(:,:,188),sss(:,:,189),sss(:,:,190),sss(:,:,191),sss(:,:,192),sss(:,:,193),sss(:,:,194),sss(:,:,195),sss(:,:,196),sss(:,:,197),sss(:,:,198),sss(:,:,199),sss(:,:,200),sss(:,:,201),sss(:,:,202),sss(:,:,203),sss(:,:,204),sss(:,:,205),sss(:,:,206),sss(:,:,207),sss(:,:,208),sss(:,:,209),sss(:,:,210),sss(:,:,211),sss(:,:,212),sss(:,:,213),sss(:,:,214),sss(:,:,215),sss(:,:,216),sss(:,:,217),sss(:,:,218),sss(:,:,219),sss(:,:,220),sss(:,:,221),sss(:,:,222),sss(:,:,223),sss(:,:,224),sss(:,:,225),sss(:,:,226),sss(:,:,227),sss(:,:,228),sss(:,:,229),sss(:,:,230),sss(:,:,231),sss(:,:,232),sss(:,:,233),sss(:,:,234),sss(:,:,235),sss(:,:,236),sss(:,:,237),sss(:,:,238),sss(:,:,239),sss(:,:,240),sss(:,:,241),sss(:,:,242),sss(:,:,243),sss(:,:,244),sss(:,:,245),sss(:,:,246),sss(:,:,247),sss(:,:,248),sss(:,:,249),sss(:,:,250),sss(:,:,251),sss(:,:,252),sss(:,:,253),sss(:,:,254),sss(:,:,255),sss(:,:,256),sss(:,:,257),sss(:,:,258),sss(:,:,259),sss(:,:,260),sss(:,:,261),sss(:,:,262),sss(:,:,263),sss(:,:,264),sss(:,:,265),sss(:,:,266),sss(:,:,267),sss(:,:,268),sss(:,:,269),sss(:,:,270),sss(:,:,271),sss(:,:,272),sss(:,:,273),sss(:,:,274),sss(:,:,275),sss(:,:,276),sss(:,:,277),sss(:,:,278),sss(:,:,279),sss(:,:,280),sss(:,:,281),sss(:,:,282),sss(:,:,283),sss(:,:,284),sss(:,:,285),sss(:,:,286),sss(:,:,287),sss(:,:,288),sss(:,:,289),sss(:,:,290),sss(:,:,291),sss(:,:,292),sss(:,:,293),sss(:,:,294),sss(:,:,295),sss(:,:,296),sss(:,:,297),sss(:,:,298),sss(:,:,299),sss(:,:,300),sss(:,:,301),sss(:,:,302),sss(:,:,303),sss(:,:,304),sss(:,:,305),sss(:,:,306),sss(:,:,307),sss(:,:,308),sss(:,:,309),sss(:,:,310),sss(:,:,311),sss(:,:,312),sss(:,:,313),sss(:,:,314),sss(:,:,315),sss(:,:,316),sss(:,:,317),sss(:,:,318),sss(:,:,319),sss(:,:,320),sss(:,:,321),sss(:,:,322),sss(:,:,323),sss(:,:,324),sss(:,:,325),sss(:,:,326),sss(:,:,327),sss(:,:,328),sss(:,:,329),sss(:,:,330),sss(:,:,331),sss(:,:,332),sss(:,:,333),sss(:,:,334),sss(:,:,335),sss(:,:,336),sss(:,:,337),sss(:,:,338),sss(:,:,339),sss(:,:,340),sss(:,:,341),sss(:,:,342),sss(:,:,343),sss(:,:,344),sss(:,:,345),sss(:,:,346),sss(:,:,347),sss(:,:,348),sss(:,:,349),sss(:,:,350),sss(:,:,351),sss(:,:,352),sss(:,:,353),sss(:,:,354),sss(:,:,355),sss(:,:,356),sss(:,:,357),sss(:,:,358),sss(:,:,359),sss(:,:,360),sss(:,:,361),sss(:,:,362),sss(:,:,363),sss(:,:,364),sss(:,:,365),sss(:,:,366),sss(:,:,367),sss(:,:,368),sss(:,:,369),sss(:,:,370),sss(:,:,371),sss(:,:,372),sss(:,:,373),sss(:,:,374),sss(:,:,375),sss(:,:,376),sss(:,:,377),sss(:,:,378),sss(:,:,379),sss(:,:,380),sss(:,:,381),sss(:,:,382),sss(:,:,383),sss(:,:,384),sss(:,:,385),sss(:,:,386),sss(:,:,387),sss(:,:,388),sss(:,:,389),sss(:,:,390),sss(:,:,391),sss(:,:,392),sss(:,:,393),sss(:,:,394),sss(:,:,395),sss(:,:,396),sss(:,:,397),sss(:,:,398),sss(:,:,399),sss(:,:,400),sss(:,:,401),sss(:,:,402),sss(:,:,403),sss(:,:,404),sss(:,:,405),sss(:,:,406),sss(:,:,407),sss(:,:,408),sss(:,:,409),sss(:,:,410),sss(:,:,411),sss(:,:,412),sss(:,:,413),sss(:,:,414),sss(:,:,415),sss(:,:,416),sss(:,:,417),sss(:,:,418),sss(:,:,419),sss(:,:,420),sss(:,:,421),sss(:,:,422),sss(:,:,423),sss(:,:,424),sss(:,:,425),sss(:,:,426),sss(:,:,427),sss(:,:,428),sss(:,:,429),sss(:,:,430),sss(:,:,431),sss(:,:,432),sss(:,:,433),sss(:,:,434),sss(:,:,435),sss(:,:,436),sss(:,:,437),sss(:,:,438),sss(:,:,439),sss(:,:,440),sss(:,:,441),sss(:,:,442),sss(:,:,443),sss(:,:,444),sss(:,:,445),sss(:,:,446),sss(:,:,447),sss(:,:,448),sss(:,:,449),sss(:,:,450),sss(:,:,451),sss(:,:,452),sss(:,:,453),sss(:,:,454),sss(:,:,455),sss(:,:,456),sss(:,:,457),sss(:,:,458),sss(:,:,459),sss(:,:,460),sss(:,:,461),sss(:,:,462),sss(:,:,463),sss(:,:,464),sss(:,:,465),sss(:,:,466),sss(:,:,467),sss(:,:,468),sss(:,:,469),sss(:,:,470),sss(:,:,471),sss(:,:,472),sss(:,:,473),sss(:,:,474),sss(:,:,475),sss(:,:,476),sss(:,:,477),sss(:,:,478),sss(:,:,479),sss(:,:,480),sss(:,:,481),sss(:,:,482),sss(:,:,483),sss(:,:,484),sss(:,:,485),sss(:,:,486),sss(:,:,487),sss(:,:,488),sss(:,:,489),sss(:,:,490),sss(:,:,491),sss(:,:,492),sss(:,:,493),sss(:,:,494),sss(:,:,495),sss(:,:,496),sss(:,:,497),sss(:,:,498),sss(:,:,499),sss(:,:,500))

%%
bode(sss(:,:,1),sss(:,:,2),sss(:,:,3),sss(:,:,4),sss(:,:,5),sss(:,:,6),sss(:,:,7),sss(:,:,8),sss(:,:,9),sss(:,:,10),sss(:,:,11),sss(:,:,12),sss(:,:,13),sss(:,:,14),sss(:,:,15),sss(:,:,16),sss(:,:,17),sss(:,:,18),sss(:,:,19),sss(:,:,20),sss(:,:,21),sss(:,:,22),sss(:,:,23),sss(:,:,24),sss(:,:,25),sss(:,:,26),sss(:,:,27),sss(:,:,28),sss(:,:,29),sss(:,:,30),sss(:,:,31),sss(:,:,32),sss(:,:,33),sss(:,:,34),sss(:,:,35),sss(:,:,36),sss(:,:,37),sss(:,:,38),sss(:,:,39),sss(:,:,40),sss(:,:,41),sss(:,:,42),sss(:,:,43),sss(:,:,44),sss(:,:,45),sss(:,:,46),sss(:,:,47),sss(:,:,48),sss(:,:,49),sss(:,:,50),sss(:,:,51),sss(:,:,52),sss(:,:,53),sss(:,:,54),sss(:,:,55),sss(:,:,56),sss(:,:,57),sss(:,:,58),sss(:,:,59),sss(:,:,60),sss(:,:,61),sss(:,:,62),sss(:,:,63),sss(:,:,64),sss(:,:,65),sss(:,:,66),sss(:,:,67),sss(:,:,68),sss(:,:,69),sss(:,:,70),sss(:,:,71),sss(:,:,72),sss(:,:,73),sss(:,:,74),sss(:,:,75),sss(:,:,76),sss(:,:,77),sss(:,:,78),sss(:,:,79),sss(:,:,80),sss(:,:,81),sss(:,:,82),sss(:,:,83),sss(:,:,84),sss(:,:,85),sss(:,:,86),sss(:,:,87),sss(:,:,88),sss(:,:,89),sss(:,:,90),sss(:,:,91),sss(:,:,92),sss(:,:,93),sss(:,:,94),sss(:,:,95),sss(:,:,96),sss(:,:,97),sss(:,:,98),sss(:,:,99),sss(:,:,100),sss(:,:,101),sss(:,:,102),sss(:,:,103),sss(:,:,104),sss(:,:,105),sss(:,:,106),sss(:,:,107),sss(:,:,108),sss(:,:,109),sss(:,:,110),sss(:,:,111),sss(:,:,112),sss(:,:,113),sss(:,:,114),sss(:,:,115),sss(:,:,116),sss(:,:,117),sss(:,:,118),sss(:,:,119),sss(:,:,120),sss(:,:,121),sss(:,:,122),sss(:,:,123),sss(:,:,124),sss(:,:,125),sss(:,:,126),sss(:,:,127),sss(:,:,128),sss(:,:,129),sss(:,:,130),sss(:,:,131),sss(:,:,132),sss(:,:,133),sss(:,:,134),sss(:,:,135),sss(:,:,136),sss(:,:,137),sss(:,:,138),sss(:,:,139),sss(:,:,140),sss(:,:,141),sss(:,:,142),sss(:,:,143),sss(:,:,144),sss(:,:,145),sss(:,:,146),sss(:,:,147),sss(:,:,148),sss(:,:,149),sss(:,:,150),sss(:,:,151),sss(:,:,152),sss(:,:,153),sss(:,:,154),sss(:,:,155),sss(:,:,156),sss(:,:,157),sss(:,:,158),sss(:,:,159),sss(:,:,160),sss(:,:,161),sss(:,:,162),sss(:,:,163),sss(:,:,164),sss(:,:,165),sss(:,:,166),sss(:,:,167),sss(:,:,168),sss(:,:,169),sss(:,:,170),sss(:,:,171),sss(:,:,172),sss(:,:,173),sss(:,:,174),sss(:,:,175),sss(:,:,176),sss(:,:,177),sss(:,:,178),sss(:,:,179),sss(:,:,180),sss(:,:,181),sss(:,:,182),sss(:,:,183),sss(:,:,184),sss(:,:,185),sss(:,:,186),sss(:,:,187),sss(:,:,188),sss(:,:,189),sss(:,:,190),sss(:,:,191),sss(:,:,192),sss(:,:,193),sss(:,:,194),sss(:,:,195),sss(:,:,196),sss(:,:,197),sss(:,:,198),sss(:,:,199),sss(:,:,200),sss(:,:,201),sss(:,:,202),sss(:,:,203),sss(:,:,204),sss(:,:,205),sss(:,:,206),sss(:,:,207),sss(:,:,208),sss(:,:,209),sss(:,:,210),sss(:,:,211),sss(:,:,212),sss(:,:,213),sss(:,:,214),sss(:,:,215),sss(:,:,216),sss(:,:,217),sss(:,:,218),sss(:,:,219),sss(:,:,220),sss(:,:,221),sss(:,:,222),sss(:,:,223),sss(:,:,224),sss(:,:,225),sss(:,:,226),sss(:,:,227),sss(:,:,228),sss(:,:,229),sss(:,:,230),sss(:,:,231),sss(:,:,232),sss(:,:,233),sss(:,:,234),sss(:,:,235),sss(:,:,236),sss(:,:,237),sss(:,:,238),sss(:,:,239),sss(:,:,240),sss(:,:,241),sss(:,:,242),sss(:,:,243),sss(:,:,244),sss(:,:,245),sss(:,:,246),sss(:,:,247),sss(:,:,248),sss(:,:,249),sss(:,:,250),sss(:,:,251),sss(:,:,252),sss(:,:,253),sss(:,:,254),sss(:,:,255),sss(:,:,256),sss(:,:,257),sss(:,:,258),sss(:,:,259),sss(:,:,260),sss(:,:,261),sss(:,:,262),sss(:,:,263),sss(:,:,264),sss(:,:,265),sss(:,:,266),sss(:,:,267),sss(:,:,268),sss(:,:,269),sss(:,:,270),sss(:,:,271),sss(:,:,272),sss(:,:,273),sss(:,:,274),sss(:,:,275),sss(:,:,276),sss(:,:,277),sss(:,:,278),sss(:,:,279),sss(:,:,280),sss(:,:,281),sss(:,:,282),sss(:,:,283),sss(:,:,284),sss(:,:,285),sss(:,:,286),sss(:,:,287),sss(:,:,288),sss(:,:,289),sss(:,:,290),sss(:,:,291),sss(:,:,292),sss(:,:,293),sss(:,:,294),sss(:,:,295),sss(:,:,296),sss(:,:,297),sss(:,:,298),sss(:,:,299),sss(:,:,300),sss(:,:,301),sss(:,:,302),sss(:,:,303),sss(:,:,304),sss(:,:,305),sss(:,:,306),sss(:,:,307),sss(:,:,308),sss(:,:,309),sss(:,:,310),sss(:,:,311),sss(:,:,312),sss(:,:,313),sss(:,:,314),sss(:,:,315),sss(:,:,316),sss(:,:,317),sss(:,:,318),sss(:,:,319),sss(:,:,320),sss(:,:,321),sss(:,:,322),sss(:,:,323),sss(:,:,324),sss(:,:,325),sss(:,:,326),sss(:,:,327),sss(:,:,328),sss(:,:,329),sss(:,:,330),sss(:,:,331),sss(:,:,332),sss(:,:,333),sss(:,:,334),sss(:,:,335),sss(:,:,336),sss(:,:,337),sss(:,:,338),sss(:,:,339),sss(:,:,340),sss(:,:,341),sss(:,:,342),sss(:,:,343),sss(:,:,344),sss(:,:,345),sss(:,:,346),sss(:,:,347),sss(:,:,348),sss(:,:,349),sss(:,:,350),sss(:,:,351),sss(:,:,352),sss(:,:,353),sss(:,:,354),sss(:,:,355),sss(:,:,356),sss(:,:,357),sss(:,:,358),sss(:,:,359),sss(:,:,360),sss(:,:,361),sss(:,:,362),sss(:,:,363),sss(:,:,364),sss(:,:,365),sss(:,:,366),sss(:,:,367),sss(:,:,368),sss(:,:,369),sss(:,:,370),sss(:,:,371),sss(:,:,372),sss(:,:,373),sss(:,:,374),sss(:,:,375),sss(:,:,376),sss(:,:,377),sss(:,:,378),sss(:,:,379),sss(:,:,380),sss(:,:,381),sss(:,:,382),sss(:,:,383),sss(:,:,384),sss(:,:,385),sss(:,:,386),sss(:,:,387),sss(:,:,388),sss(:,:,389),sss(:,:,390),sss(:,:,391),sss(:,:,392),sss(:,:,393),sss(:,:,394),sss(:,:,395),sss(:,:,396),sss(:,:,397),sss(:,:,398),sss(:,:,399),sss(:,:,400),sss(:,:,401),sss(:,:,402),sss(:,:,403),sss(:,:,404),sss(:,:,405),sss(:,:,406),sss(:,:,407),sss(:,:,408),sss(:,:,409),sss(:,:,410),sss(:,:,411),sss(:,:,412),sss(:,:,413),sss(:,:,414),sss(:,:,415),sss(:,:,416),sss(:,:,417),sss(:,:,418),sss(:,:,419),sss(:,:,420),sss(:,:,421),sss(:,:,422),sss(:,:,423),sss(:,:,424),sss(:,:,425),sss(:,:,426),sss(:,:,427),sss(:,:,428),sss(:,:,429),sss(:,:,430),sss(:,:,431),sss(:,:,432),sss(:,:,433),sss(:,:,434),sss(:,:,435),sss(:,:,436),sss(:,:,437),sss(:,:,438),sss(:,:,439),sss(:,:,440),sss(:,:,441),sss(:,:,442),sss(:,:,443),sss(:,:,444),sss(:,:,445),sss(:,:,446),sss(:,:,447),sss(:,:,448),sss(:,:,449),sss(:,:,450),sss(:,:,451),sss(:,:,452),sss(:,:,453),sss(:,:,454),sss(:,:,455),sss(:,:,456),sss(:,:,457),sss(:,:,458),sss(:,:,459),sss(:,:,460),sss(:,:,461),sss(:,:,462),sss(:,:,463),sss(:,:,464),sss(:,:,465),sss(:,:,466),sss(:,:,467),sss(:,:,468),sss(:,:,469),sss(:,:,470),sss(:,:,471),sss(:,:,472),sss(:,:,473),sss(:,:,474),sss(:,:,475),sss(:,:,476),sss(:,:,477),sss(:,:,478),sss(:,:,479),sss(:,:,480),sss(:,:,481),sss(:,:,482),sss(:,:,483),sss(:,:,484),sss(:,:,485),sss(:,:,486),sss(:,:,487),sss(:,:,488),sss(:,:,489),sss(:,:,490),sss(:,:,491),sss(:,:,492),sss(:,:,493),sss(:,:,494),sss(:,:,495),sss(:,:,496),sss(:,:,497),sss(:,:,498),sss(:,:,499),sss(:,:,500))

%% 

[A,B,C,D] = lateral_dynamics(mean(1), mean(2), Ts);
ssm = ss(A,B,C,D);

[A,B,C,D] = lateral_dynamics(mean(1)+3*sigma(1), mean(2)+3*sigma(2), Ts);
sspt = ss(A,B,C,D);

[A,B,C,D] = lateral_dynamics(mean(1)-3*sigma(1), mean(2)-3*sigma(2), Ts);
ssmt = ss(A,B,C,D);


bode(ssm, 'k', sspt, 'r--', ssmt, 'r--')
legend('Mean', '$+3\sigma$','$-3\sigma$')
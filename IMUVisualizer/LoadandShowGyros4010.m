    
% For Lab Task 001
% This piece of code shows how to load a matlab data file that contains 
% IMU data (according to the structure format we use in MTRN4010).
% It also uses the time and gyros' measurements.
% Jose Guivant - MTRN4010 - Session1/2013
% Edited by - Wenny Hidayat (z3351846)

%---------------------------------------------------
% load the file of interest.
% CHANGE THIS SECTION FOR DIFFERENT DATASETS
file = '.\data\IMU_AAS001\IMU_MS01.mat';
load(file) ;

% now after I load the data, it will be in a variable named IMU
%---------------------------------------------------

% Originally the timestamps are in IMU.timesE. These are expressed in counts, where
% each count is = 0.1 milisecond.
time = double(IMU.timesE)*0.0001;  % now the time is in seconds.
time = time - time(1) ;  % in order to refer to t0=0.

Gyros = IMU.DATAf(4:6,:);
% now I got specifically the gyros (angular rates) measurements.

% Gyros(1,:) is Wx  (local Roll rate)
% Gyros(2,:) is Wy  (local Pitch rate)
% Gyros(3,:) is Wz  (local Yaw rate)
% all measurements are expressed in radians/second

%---------------------------------------------------
% -- plot angular rates (local, from gyros) ------

figure(1) ; clf(); hold on ;
k = 180/pi; % converting radian to degrees.
plot(time,Gyros(1,:)*k,'b');
plot(time,Gyros(2,:)*k,'r');
plot(time,Gyros(3,:)*k,'g');
legend({'Wx (roll rate)','Wy (pitch rate)','Wz (yaw rate)'});
xlabel('time (in seconds)');
ylabel('angular rates (degrees/sec)');
grid on ;
zoom on;

%---------------------------------------------------

% Now I want to integrate the angular rates, for estimating the attitude
% assume initial attitude =[0,0,0]; (in radians)

% if 0,       % you will allow this part later.

CurrAttitude = [0;0;0];
L = length(Gyros(1,:)) ;
Attitudes = zeros(3,L);

% Removing Bias by Averaging
% CHANGE THE LENGTH OF DATA FOR DIFFERENT DATASETS

% % % FOR IMU_AAS001
mean1 = mean(Gyros(1,:));
mean2 = mean(Gyros(2,:));
mean3 = mean(Gyros(3,1:1571));

% % FOR IMU_AAS002
% mean1 = mean(Gyros(1,:));
% mean2 = mean(Gyros(2,:));
% mean3 = mean(Gyros(3,1:1642));

% FOR IMU_AAS003
% mean1 = mean(Gyros(1,1:1575));
% mean2 = mean(Gyros(2,1:1575));
% mean25 = mean(Gyros(2,5406:L));
% mean2 = (mean2 + mean25)/2;
% mean3 = mean(Gyros(3,1:1113));

% FOR IMU_AAS004
% mean1 = mean(Gyros(1,1:1575));
% mean2 = mean(Gyros(2,1:1575));
% mean3 = mean(Gyros(3,1:1113));

% % FOR IMU_AAS005
% mean1 = mean(Gyros(1,:))
% mean2 = mean(Gyros(2,:))
% mean3 = mean(Gyros(3,:))

% FOR IMU_AAS006
% mean1 = mean(Gyros(1,1:2880));
% mean2 = mean(Gyros(2,1:2880));
% mean3 = mean(Gyros(3,1:2880));

% % FOR IMU_AAS007
% mean1 = mean(Gyros(1,1:3753));
% mean2 = mean(Gyros(2,1:3753));
% mean3 = mean(Gyros(3,1:3753));

% FOR IMU_AAS009
% mean1 = mean(Gyros(1,1:1880));
% mean2 = mean(Gyros(2,1:1880));
% mean3 = mean(Gyros(3,1:1453));

Gyros(1,:) = Gyros(1,:) - mean1;
Gyros(2,:) = Gyros(2,:) - mean2;
Gyros(3,:) = Gyros(3,:) - mean3;

for i=1:L,
    ans = IntegrateOneStepOfAttitude(Gyros(:,i),0.005,CurrAttitude);
    CurrAttitude(:) =  transpose(ans) ;
    Attitudes(:,i)= CurrAttitude;
end;


% then plot the stored attitudes in figure #2.
figure(2); clf(); hold on ;
plot(time, Attitudes(1,:)*k, 'b');
plot(time, Attitudes(2,:)*k, 'r');
plot(time, Attitudes(3,:)*k, 'g');
legend({'Roll','Pitch','Yaw'});
xlabel('time (s)');
ylabel('angle (degrees)');
grid on ;
zoom on;
% end;
%-----------------------------------------------------



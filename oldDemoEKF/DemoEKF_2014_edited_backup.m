% DemoEKF.m - version 1
% MRTN4010 - 2014
% Example of the applicaction of Extended Kalman Filter for the localization problem, using range observations.

% Note#1:
% For each "laser scan" we extract the detected landmarks and use their ranges to update the estimated 
% states (3DoF: 2D position and heading of the platform).  
% Even frequently using "just" one range (related to some landmark) the estimator is
% able to estimate the pose (position and heading)!!!
% Q: How is this possible?
% A: This is because in addition to the observation model we use the process model (that correlates the states in time) as well.
% As the robot moves then we obtain several observations from different poses (and at different times).
% This system is what is called "Observable" 
% (its internal states can be estimated by observing its "outputs", in this case we measure its distance to certain landmark/landmarks).


% Note#2: in this simulation we run the process model at certain frequency
% (dt=50ms), however we do not assume we get measurements (for the observations) at that sample
% rate (that situation would be great!, but not always possible). We assume observations have a slower sample rate. This
% is to show that even for a poor system (observing at low rate) the estimation process is still good and feasible. 
% ( Question: How fast is a human driver in observing the context in order to estimate his/her pose? )


%Note #3:
% You can change many things in this simulation:
% 1)  the geographical distribution and number of landmarks.
% 2)  the noise's standard deviations in the inputs of the kinematic model of the vehicle
% 3)  the noise's standard deviations of range measurements
% 4)  the sampling time of the scanner
% 5)  the inputs to the process (speed and angular rate)
% a lot of things to experiment!


% Note4:  IMPORTANT
% a) You will need to modify the EKF update stage in order to deal with the
% cases:        Bearing Only and Range+Bearing.

% b) you need to adapt the Q matrix in the prediction stage in order to
% consider the noise that pollutes the inputs of the process model (speed
% and gyroZ). Follow the approach presented in the lecture notes, in file
% "MTRN40101_L06_Noise_in_the_inputs_of_ProcessModel.pdf"

%---------------------------------

% In this problem we have a kinematic model of the platform ( our "process
% model"). The inputs of the model are 2 :
%    a)  platform's speed (measured at the middle point between theback wheels) 
%    b)  Wz (yaw/heading angular rate), measured through  IMU's gyros.


% by J. Guivant for MRTN4010's students, Session 2014.

% Edited by Wenny Hidayat/z3351846
% Some changes: 1. Process bearing instead of range (v)
%               2. Process both range and bearing   (v)
%               3. 






function main()


% .....................................................
% Here I define the magnitude of the "noises", that I assume are present, polluting mesurements, models, etc.  

% .. error in the angular rate sensor
stdDevGyro = 2*pi/180 ;          % 2 degrees/second , standard deviation of the gyros' noise
% you have seen, when playing with our IMU that, after removing the
% offsets, the eror present in the gyros' measurements should be lower than 2 deg/sec.

% ...error in the speed's measurements
stdDevSpeed = 0.3 ;   % a lot of error!  (very difficult case). speedmeter's error = 0.3m/sec 

% ... errors in the range measurements (25cm, standard.dev.)
sdev_rangeMeasurement = 0.25 ;          % std. of noise in range measurements. 0.25m
sdev_bearingMeasurement = 5*pi/180;    % stddev for bearing at 5 deg/s, in rad
% this is more that the error you would have with our laser scanner.

% .....................................................
Dt=0.05 ;                       % "sample time", 50ms
Li = 5000 ;                     % "experiment" duration, in iterations 
DtObservations=0.250 ;          % laser "sample time" of observations, 4Hz, approximately
r2d = 180/pi;                   % conversion rad to deg
d2r = pi/180;                   % conversion deg to rad

% .....................................................
% here I create a virtual map, because this program works on a simulated process.
% (in a real case you would have a real map)

% How many landmarks do you want to use?  
n_usedLanmarks = 3 ;    %it must be : 1 <= n_usedLanmarks <= 4 
% just to create some landmarks
global NavigationMap;
NavigationMap = CreateSomeMap(n_usedLanmarks) ;  %creates a artificial map!
% ................................................


% These are the ESTIMATES (Expected value and covariance matrix)
% Initial conditions of the estimates (identical to the real ones, as in
% the lab)( I Assume we know the initial condition of the system)
Xe = [ 0; 0; pi/2 ] ;       % starting x
P = zeros(3,3) ;            % initial quality --> perfect (covariance =zero )
% Why perfect? (BECAUSE in this case we DO ASSUME we know perfectly the initial condition)


% Some buffers to store the intermediate values during the experiment
Xreal_History= zeros(3,Li) ;
Xe_History= zeros(3,Li) ;

% .....................................................
% I assume that every time we apply the process model to predict the evolution of the system for a 
% perdiod of time of Dt (50ms) we introduce uncertainty of 0.01m standard deviation on X, 
% similar uncertainty on Y and 1 degree (a lot!) on the heading's estimation
% Q  =zeros(3,3) ;  Q(1,1) = (0.01)^2 ; Q(2,2) =Q(1,1)  ; Q(3,3) = (1*pi/180)^2 ;
% Although you initially use this one, it can be improved. Read
% "MTRN40101_L06_Noise_in_the_inputs_of_ProcessModel.pdf" in order to implement a good refinement. 

Q = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;
% Q matrix. Represent the covariance of the undertainty about the process model.
% .....................................................


time=0 ;
% init simulator of process.
InitSimulation(stdDevSpeed,stdDevGyro,sdev_bearingMeasurement,sdev_rangeMeasurement,DtObservations);
% (BTW: this is just a simulation to replace ther eal system, because we,
% for the moment, do not have the real system. )


% .....................................................    
for i=1:Li,     % loop
    
    %pause(dt) ;   % NO delay, I want to run this in a fast off-line SIMULATION, so ASAP.
    
    time = time+Dt ;    
    SimuPlatform(time,Dt);      % because NOW I do not have a real system, I simulate it.

     
    % I ask about the inputs to the Process Model.
    % The inputs I will be told are polluted versions of the real inputs.
    [Noisy_speed,Noisy_GyroZ]=GetProcessModelInputs();
    % in the real case we should measure the real inputs (gyros and speedmeter)
    
    % -------- THIS IS THE EKF! ------------------------
    % ------ Kalman Filter prediction: applies the prediction step (i.e. Process model). 
    Xe    = RunProcessModel(Xe,Noisy_speed,Noisy_GyroZ,Dt) ;
    
    % Estimate new covariance after prediction
    % First , I evaluate the Jacobian matrix of the process model (see lecture notes).
    % Write the analytical expression on paper to understand the followign line.
    J = [ [1,0,-Dt*Noisy_speed*sin(Xe(3))  ]  ; [0,1,Dt*Noisy_speed*cos(Xe(3))] ;    [ 0,0,1 ] ] ; 
    % then I calculate the new coveraince, after the prediction P(K+1|K) = J*P(K|K)*J'+Q ;
    P = J*P*J'+Q ;

    % the precition step, for this iteration, is done.
    % -----------------------------------------------


    % .. Get range measuremens, if those are available.
    [nDetectedLandmarksB,nDetectedLandmarks,MeasuredBearings,MasuredRanges,IDs]=GetObservationMeasurements();
    
    
    
    if (nDetectedLandmarks>0) && (nDetectedLandmarksB>0),     % any laser data and detected landmarks?
     
        % Because there are available obsevations ==> I perform EKF updates
             
   
        % --------------- Kalmal Filter update (observations)
        % Because this observation function is non-linear--> we need to get the Jacobians of h(X).
        % Jacobian for range only measurements (evaluated at the current expected value -->Xe)    
        
        % sqrt( (xi-x)^2+(yi-y)^2 ) for all the seen landmarks. I will need
        % this term for the next calculations.
        
        for u=1:nDetectedLandmarks,
         
            ID = IDs(u);            % landmark ID?    
            eDX = (NavigationMap.landmarks(ID,1)-Xe(1)) ;      % (xu-x)
            eDY = (NavigationMap.landmarks(ID,2)-Xe(2)) ;      % (yu-y)
            
            % Range input
%             eDD_range = sqrt( eDX.*eDX + eDY.*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 ) 

            % Bearing input
%             eDD = atan2(eDY, eDX) - Xe(3) + pi/2; 
            eDD_bearing = eDX.*eDX + eDY.*eDY;
        
            % here is it. "H". I reuse some previous calculations.
%             H_range = [  -eDX./eDD_range , -eDY./eDD_range , 0 ] ;   % Jacobian of h(X),range
            H_bearing = [ eDY./eDD_bearing, -eDX./eDD_bearing, -1];
%             H = [H_range; H_bearing];
            
            % the expected distances to the landmarks ( "h(Xe)" ). Xe 1=x 2=y 3=gyro.
%             ExpectedRange = eDD_range ;   % just a coincidence: we already calculated them for the Jacobian, so I reuse it. 
            ExpectedBearing = atan2(eDY, eDX) - Xe(3) + pi/2; % Output in rad
        
            % Evaluate residual (innovation)  "Y-h(Xe)"
%             zR  = MasuredRanges(u) - ExpectedRange ;      
            zB = MeasuredBearings(u) - ExpectedBearing;
            % Normalization
            zB = mod(  zB*r2d+180, 2*180) - 180; % in deg
            zB = zB*d2r;
            
%             z =[zR;zB];
            
            % ------ covariance of the noise/uncetainty in the measurements
            % R is Diagonal --->because I believe the noises in the measurements
            %  ,from different sensors, are independent.
%             RR = sdev_rangeMeasurement*sdev_rangeMeasurement*4; 
            RB = sdev_bearingMeasurement*sdev_bearingMeasurement*4 ;
%             R = diag([RR,RB]);
            % I multiply by 4 because I want to be conservative and assume
            % twice the standard deviation that I believe does happen.
        
        
            % Some intermadiate steps for the EKF (as in the lecture notes)
%             S = R + H*P*H'; 
            S = RB + H_bearing*P*H_bearing';
            iS = inv(S) ;               % in this case S is 1x1 so inv(S) is just 1/S

            K = P*H_bearing'.*iS ;           % Kalman gain

            % ----- do it...I am getting  X(k+1|k+1) and P(k+1|k+1)
            Xe = Xe + K*zB ;           % update the  expected value
            P = P-P*H_bearing'*iS*H_bearing*P ;     % update the Covariance
        % -----  individual EKF update done ...
       end; 
       % all avalilable EKF updates done ...
    end;  
     
    
     % -------- remember some variables to plot at the end.   
     Xreal_History(:,i) = GetCurrentSimulatedState() ;
     Xe_History(:,i)    = Xe ;
end ;    
      


% PLOT some results. PLot estimated and "real" values of platform states.
SomePlots(Xreal_History,Xe_History,NavigationMap) ;


return ;           % loop to the next observation based on new measurements..



% =========================================================================
% --- THIS IS THE PROCESS MODEL of MY SYSTEM. (it is a Kinemetic model)
    
function Xnext=RunProcessModel(X,speed,GyroZ,dt) 
    Xnext = X + dt*[ speed*cos(X(3)) ;  speed*sin(X(3)) ; GyroZ ] ;
return ;


% =========================================================================
% ========== Simulation functions - (used for simuation of "real" platform and
% for the process model of the EKF.
% When we apply the EKF on a real case, we do not need this part.


function [bearings,ranges,IDs] = GetMeasurementsFomNearbyLandmarks(X,map)
    dx= map.landmarks(:,1) - X(1) ;
    dy= map.landmarks(:,2) - X(2) ;
    bearings = atan2(dy, dx) - X(3) + pi/2; %in RAD
    ranges = sqrt((dx.*dx + dy.*dy)) ;
    IDs = [1:map.nLandmarks];
    
    % I simulate I measure/detect all the landmarks, however there can be
    % cases where I see just the nearby ones.
    
return ;


% here I propose some speed and angular rate inputs. 
% in real cases, they do happen, we do not to propose them.
function [speed,GyroZ] = SimuControl(X,t)
    speed = 2 ;                                         % cruise speed, 2m/s  ( v ~ 7km/h)
    GyroZ = 3*pi/180 + sin(0.1*2*pi*t/50)*.02 ;         % some crazy driver moving the steeering wheel...
return ;


% here I propose some map of landmarks. 
% in real cases, we do not create it synthetically, like here.
function map = CreateSomeMap(n_used)
    landmarks = [  [ -40,0 ];[ -0,-20 ];[ 10,10 ] ;[ 30,10 ]  ] ;
    map.landmarks = landmarks(1:n_used,:) ;
    map.nLandmarks = n_used ;
return ;



function InitSimulation(stdDevSpeed,stdDevGyro,sdev_bearingMeasurement,sdev_rangeMeasurement,DtObservations)
    global ContextSimulation;
    ContextSimulation.Xreal = [ 0; 0;pi/2 ] ;     % [x;y;phi]
    ContextSimulation.stdDevSpeed = stdDevSpeed;
    ContextSimulation.stdDevGyro = stdDevGyro;
    ContextSimulation.Xreal = [0;0;pi/2];
    ContextSimulation.speed=0;
    ContextSimulation.GyroZ=0;
    ContextSimulation.sdev_rangeMeasurement=sdev_rangeMeasurement;
    ContextSimulation.sdev_bearingMeasurement=sdev_bearingMeasurement;
    ContextSimulation.DtObservations=DtObservations;
    ContextSimulation.timeForNextObservation= 0;
    ContextSimulation.CurrSimulatedTime=0;
return;


function [Noisy_speed,Noisy_GyroZ]=GetProcessModelInputs()
    % .....................................................
    % add noise to simulate real conditions
    % WHY? to be realistic. When we measure things the measurements are polluted with noise, So I simulated that situation by adding random
    % noise to the perfect measurements (the ones I get from the simulated "real" platform.
    global ContextSimulation;
    Noisy_speed =ContextSimulation.speed+ContextSimulation.stdDevSpeed*randn(1) ;
    Noisy_GyroZ =ContextSimulation.GyroZ+ContextSimulation.stdDevGyro*randn(1);
return;


% NOTE: in Project#2 you will apply the EKF for estimating the states of the real
% system. In such cases You DO NOT ADD ANY NOISE to the real
% measurements!!!!. We add noise here because we pretend this is the real
% case, always having noise polluting our measurements.


function  SimuPlatform(time,Dt)
    global ContextSimulation;    
    
    % simulate some crazy driver for the car..
    [ContextSimulation.speed,ContextSimulation.GyroZ] = SimuControl(ContextSimulation.Xreal,time) ;      % read kinematic model inputs, ideal ones
    % .........................................
    % simulate one step of the "real system":  Xreal(time)
    ContextSimulation.Xreal = RunProcessModel(ContextSimulation.Xreal,ContextSimulation.speed,ContextSimulation.GyroZ,Dt) ;
    ContextSimulation.CurrSimulatedTime = ContextSimulation.CurrSimulatedTime+Dt;
return;


function [nDetectedLandmarksB,nDetectedLandmarks,MeasuredBearings,MasuredRanges,IDs]=GetObservationMeasurements(map)
    global ContextSimulation NavigationMap;       
   
    if ContextSimulation.CurrSimulatedTime<ContextSimulation.timeForNextObservation,
        nDetectedLandmarks=0;
        nDetectedLandmarksB = 0;
        MasuredRanges=[];
        MeasuredBearings = [];
        IDs=[];
        return ; 
    end;
        
    ContextSimulation.timeForNextObservation = ContextSimulation.timeForNextObservation+ContextSimulation.DtObservations;
    
    % get simulated range measurements (actual system), in this case the
    % simulated platform.
        [RealBearings,RealRanges,IDs] = GetMeasurementsFomNearbyLandmarks(ContextSimulation.Xreal,NavigationMap) ;
                % ...................................................
        nDetectedLandmarks = length(RealRanges) ;
        nDetectedLandmarksB = length(RealBearings);
        
        % Here I corrupt the simulated measurements by adding some random noise (to simulate noise, as in the reality)
        % this is the noise
        noiseInMeasurements= ContextSimulation.sdev_rangeMeasurement*randn(size(RealRanges));
        noiseInBearings= ContextSimulation.sdev_bearingMeasurement*randn(size(RealBearings));
        
        % here I add it to the perfect ranges' measurements
        MasuredRanges = RealRanges +  noiseInMeasurements ;
        MeasuredBearings = RealBearings + noiseInBearings;
        % so MasuredRanges are the prefect measurements polluted with
        % noise. I get the "perfect measurements" from the simulated
        % platform
 return;

 
    function X=GetCurrentSimulatedState()
        global ContextSimulation;
        X=ContextSimulation.Xreal;
        
        
     return;   

% -------------end simulation functions -----------------------------------------------


% ====================================================
% --- This is JUST for ploting the results
function SomePlots(Xreal_History,Xe_History,map) ;



figure(2) ; clf ; hold on ;
plot(Xreal_History(1,:),Xreal_History(2,:),'b') ;
plot(Xe_History(1,:),Xe_History(2,:),'r') ;
plot(map.landmarks(:,1),map.landmarks(:,2),'*r') ;
ii = [1:225:length(Xe_History(1,:))] ;
quiver(Xe_History(1,ii),Xe_History(2,ii),5*cos(Xe_History(3,ii)),5*sin(Xe_History(3,ii)),'r' ) ;
quiver(Xreal_History(1,ii),Xreal_History(2,ii),5*cos(Xreal_History(3,ii)),5*sin(Xreal_History(3,ii)),'b' ) ;
plot(Xe_History(1,ii),Xe_History(2,ii),'+r') ;
plot(Xreal_History(1,ii),Xreal_History(2,ii),'+b') ;
title('trajectories (blue:Real, red:EKF)') ;

% --------- plot errors between EKF estimates and the real values
figure(3) ; clf ; 
subplot(311) ; plot(Xreal_History(1,:)-Xe_History(1,:)) ;ylabel('x-xe (m)') ;
title('Performance EKF') ;
subplot(312) ; plot(Xreal_History(2,:)-Xe_History(2,:)) ;ylabel('y-ye (m)') ;
subplot(313) ; plot(180/pi*(Xreal_History(3,:)-Xe_History(3,:))) ;ylabel('heading error (deg)') ;

return ;
% -------------------------------------------------------------------------
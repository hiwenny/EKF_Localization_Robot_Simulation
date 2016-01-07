% Project 1 Part 3 - version modified of  DemoEKF.m originally provided by lecturer, MTRN4010 - 2014
% by Wenny Hidayat / 3351846    
% Project 01. Part3


function Project2_p2()
    
    close all;
    
    % Initialization - once only
    AAA = API_PossumUGV(1) ;
    if (AAA.Ok<1), return ; end;
    pause_time = 0.2;           % pauses in-between scans
    calib_time = 10;             %calibration time, 7 seconds
    calib_scans = round(calib_time/pause_time); % converting to num of scans
    global globalMap;

    %% Laserscan inits
    MyContext.run=1;
%     CreateFigureToPlot(25) ;        % figure inits
    InitAngles(25);

    %% IMU inits
    % assume initial attitude =[0,0,0]; (in radians)
%     CurrAng = [0;0;0];
    r = AAA.ReadIMU(1,100) ;
    
    %% ReadSpeed inits
    rS  =AAA.ReadSpeed(1,100) ; % read up to 100 speed measurementsdt

    %% EKF inits
    foo_z = 1; % index number 
    buffer_OOI = [];

    % Defining errors (standard deviation) in each inputs
    % Depends on the specification from manufacturer. Change as needed.
    EKFconst.stdDevGyro = 5*pi/180 ;         % 2 degrees/second , standard deviation of the gyros' noise
    EKFconst.stdDevSpeed = 0.3 ;             % speedmeter's error = 0.3m/sec 
    EKFconst.sdev_rangeMeasurement = 0.25 ;         % std. of noise in range measurements. 0.25m
    EKFconst.sdev_bearingMeasurement = 5*pi/180;    % stddev for bearing at 5 deg/s, in rad

    % ESTIMATES (Expected value and covariance matrix)
    % Initial conditions of the estimates at 0 0 pi/2. Perfect condition
    d = 0.5;         % distance from reflective point to actual center
    Xe = [ 0; 0; pi/2 ; 0] ;       % starting x
    P = diag( [0, 0, 0, (8*pi/180)^2] );            % initial quality --> perfect (covariance =zero, bias speed measurement )
    tPrev = 0;                                      % temporary assignment for last timestamp
    Q = diag( [ (EKFconst.stdDevSpeed)^2 ,(EKFconst.stdDevSpeed)^2 , (EKFconst.stdDevGyro)^2 , 0 ] ) ;
    % Q matrix. Represent the covariance of the undertainty about the process model.
    
    t0 = max(rS.tcx(1,rS.n), r.tcx(1,r.n)); % t at the beginning (counts)
%     biasGyro = 0;
    biasSpeed = 0;
    d2r = pi/180;                   % conversion deg to rad
    
    
    %% Initialization for robot position in global coordinate
    track_robotCoordX = 0;
    track_robotCoordY = 0;

    fprintf('Infinite loop (control-C for breaking it.....\n') ;

    %% Handles for plotting  
    figure(1) ; clf ;  hold on ;
    handlePlot1 = plot(0,0,'.g') ; % GLOBAL MAP
    handlePlot2 = plot(0,0,'.b') ; % LOCAL MAP IN GLOBAL COORD
    handlePlot3 = plot(0,0,'*r') ; % GLOBAL OOI
    handlePlot4 = plot(0,0,'ob') ; % LOCAL OOI IN GLOBAL COORD
    handlePlot5 = plot(0,0,'*c') ; % ASSOCIATED OOIS
    handlePlot6 = quiver(0,0,0,0,'or','MarkerSize',10,'LineWidth',3); % ROBOT POSITION
    handlePlot7 = plot(0,0,'.c') ; % ROBOT PATH
    xlabel('x (m'); ylabel('y (m)'); axis([-6,9,-2,17]) ; zoom on ; hold off;
    title('Global Frame');

    % Plotting in local coordinates
    figure(2) ; clf ;  hold on ;
    handlePlot8 = plot(0,0,'.b') ; % Map
    handlePlot9 = plot(0,0,'*r') ; % OOIs
    axis([-15,15,-5,15]) ;         
    hold off;
    xlabel('x (m)'); ylabel('y (m)'); zoom on ;
    title('Local Frame'); hold off;

    % Plotting bias, just for checking
    figure(3) ; hold on;
    handlePlot10 = plot(0,0,'r') ; % Bias of speed
    axis([0,1000,-2,2]) ;
    hold off; ylabel('deg/sec');
    title('Gyro Bias');
    
    % Various Counters
    buffer_count = 1; % counter for calibration buffer           
    iterations = 1;
    track_index = 1;        % counter for trajectory
    counter_bias = 1;       % counter for bias
    track_bias_index = 1;   % buffer for storing bias indices
    
    while 1,
    
        %% IMU Part
        % read IMU unit #1. It reads new arrived mesurements, 
        % ... up to 100 records, if those are available.
        r = AAA.ReadIMU(1, 100) ;
%         angles = ProcessMyImuData(r, CurrAng); % angles in deg
        
        % Defining the current angles - last one, or average? Choose average.
        gyroZ = mean(r.data(6,1:r.n));
%         CurrAng = angles(:,r.n)*d2r; % REMEMBER integration using rad/s not deg!!!!
        
        fprintf('Iteration(%d):[%d] new measurements read\n',iterations,r.n) ;

        %% LaserScan part
                s = AAA.ReadLMS200(2,1) ;  
            %API: read LMS200, unit #2, up to 1 newly arrived scans.
            N = s.n;
            if s.n>0,                     
                OOIs = ShowData(s.Ranges(:,N),s.times(N));
        %         OOIs.N = indices of OOIs, 1 2 3 . . . 
        %         OOIs.Colors = 1xlength(OOIs.N) matrix of color
        %         OOIs.Centers(x, y) = length(OOIs.N)x2 matrix of center coords
        %         OOIs.Size = 1xlength(OOIs.N) matrix indicating sizes
        
                Laser_range = single(bitand(s.Ranges(:,N),MyContext.Mask0FFF));   
                %the first 13 bits are the range, in cm. <--API data format!!!!
                % See document "LMS2XX_data.pdf" for more details.

                Laser_xx = Laser_range.* MyContext.cosinesAngles*0.01;
                Laser_yy = Laser_range.* MyContext.sinesAngles*0.01;
            end;
            
        %% ReadSpeed Part
        rS  =AAA.ReadSpeed(1,100) ; % read up to 100 speed measurements
        speed = mean(rS.data(1:rS.n)) - biasSpeed;      % last measurement only
        % rS.data(1:rS.n) denoting the speed data
        % rS.n number of data

        if r.n<1 || rS.n<1 || s.n<1
            fprintf('No more data received.\n'); 
            break;
        end;  % no data
        
        %% EKF Part
        
        if s.n>0,
            if iterations<=calib_scans            
                buffer_OOI.x(:,buffer_count) = OOIs.Centers(:,1);
                buffer_OOI.y(:,buffer_count) = OOIs.Centers(:,2);
                buffer_OOI.biasGyro(foo_z) = mean(r.data(6,1:r.n));
                buffer_OOI.biasSpeed(foo_z) = mean(rS.data(1:rS.n));
                buffer_Map.x(:,buffer_count) = Laser_xx;
                buffer_Map.y(:,buffer_count) = Laser_yy;
                Xe(3) = pi/2;
                foo_z = foo_z+1;

                if iterations == calib_scans                   
                    globalMap.refOOI = [mean(buffer_OOI.x,2) mean(buffer_OOI.y,2)];
                    globalMap.N = OOIs.N;
                    globalMap.refMap = [mean(buffer_Map.x,2) mean(buffer_Map.y,2)];
                    % error in measurement taken at initial condition (considered non-moving)
%                     biasGyro = mean(buffer_OOI.biasGyro);   % rad/s
                    biasSpeed = mean(buffer_OOI.biasSpeed); % m/s    
                    fprintf('Calibration succeded. Now plotting trajectory.\n')

    %     elseif s.times>25560086
    %         fprintf('Warning: reset simulation before running program\n');
    %         break;
                else              
                    globalMap.refOOI = OOIs.Centers;
                    globalMap.refMap = [Laser_xx Laser_yy];                    
                    Xe(3) = mod(Xe(3)+pi, 2*pi) - pi;                    
                end

                buffer_count =buffer_count+1;% counter for storelist
                
                % While calibrating no need to rotate the OOI positions
                % since it's already global
                 local_to_global_X = globalMap.refOOI(:,1);
                 local_to_global_Y = globalMap.refOOI(:,2);
                 local_to_global_Map_X = globalMap.refMap(:,1);
                 local_to_global_Map_Y = globalMap.refMap(:,2);
                 associated_OOI_plot_X = globalMap.refOOI(:,1);
                 associated_OOI_plot_Y = globalMap.refOOI(:,2);
                 index_OOIdetected = find(local_to_global_X~=0);
                 OOIdetected = find(local_to_global_X~=0);
            else               
                rotationAngle = Xe(3)-pi/2;
                                
                % Rotation matrix for OOIs
                local_to_global_X = cos(rotationAngle)*OOIs.Centers(:,1) - sin(rotationAngle)*OOIs.Centers(:,2) + Xe(1) + d*cos(Xe(3));
                local_to_global_Y = sin(rotationAngle)*OOIs.Centers(:,1) + cos(rotationAngle)*OOIs.Centers(:,2) + Xe(2) + d*sin(Xe(3));

                % Rotation Matrix for all points (map)
                local_to_global_Map_X = cos(rotationAngle)*Laser_xx - sin(rotationAngle)*Laser_yy + Xe(1) + d*cos(Xe(3));
                local_to_global_Map_Y = sin(rotationAngle)*Laser_xx + cos(rotationAngle)*Laser_yy + Xe(2) + d*sin(Xe(3));

                % Comparison matrix
                % Creating a matrix containing the distance between scanned OOIs
                % and reference OOIs (from calibration)
                comparisonMatrix = [];

                for jj = 1:length(OOIs.N)
                    for ii = 1:length(globalMap.N)
                        comparisonMatrix(jj,ii) = sqrt((globalMap.refOOI(ii,1) - local_to_global_X (jj))^2+(globalMap.refOOI(ii,2) - local_to_global_Y(jj))^2);;
                    end
                end

               % Horizontal comparison
                % Each row contains distances from a newly detected OOI point to 
                % the reference OOIs. This allows us, through comparison, to
                % associate the closest landmark to a certain OOI.
                for foo_v=1:length(OOIs.N) % loop through all associated value, until the robot cant see any OOI
                    distFilter = find(comparisonMatrix(foo_v,:)<0.5);% row is current, column is map.OOI
                    % condition(2), associated data is the minimum value
                    if ~isempty(distFilter)                    
                        % showing which distance and value the associated data are in
                        [~, index_associated_data]= min(comparisonMatrix(foo_v,:)); 
                        % coordinate of associated data(refer to map(x, y))
                        assoc_x_buffer(foo_v) = globalMap.refOOI(index_associated_data,1);
                        assoc_y_buffer(foo_v) = globalMap.refOOI(index_associated_data,2);
                    else
                        assoc_x_buffer(foo_v) = 0;
                        assoc_y_buffer(foo_v) = 0;
                    
                    end
                    OOIdetected = find(assoc_x_buffer~=0);
                end
                
                distFilter = [];
                assoc_x_buffer = [];
                assoc_y_buffer = [];
                
                % association for new OOI position
                % not executed if no new OOI detected.
                if length(OOIs.N) == 0
                     yn = 0;
                else
                     yn = length(globalMap.N);
                end
                
                % Vertical comparison
                % Each column contains distances from each newly detected OOI point to 
                % a reference OOI. This allows us to know which out of the new
                % OOIs is the closest to a certain landmark.
                for foo_u=1:yn 
                    distFilter = find(comparisonMatrix(:,foo_u)<0.5);
                    
                    if ~isempty(distFilter)                         
                        [~, index_associated_data_OOI]= min(comparisonMatrix(:,foo_u));                        
                        assoc_x_buffer(foo_u)= local_to_global_X(index_associated_data_OOI);
                        assoc_y_buffer(foo_u)= local_to_global_Y(index_associated_data_OOI);
                    else
                        assoc_x_buffer(foo_u)= 0;
                        assoc_y_buffer(foo_u)= 0;                    
                    end
                    index_OOIdetected = find(assoc_x_buffer~=0);
                end

                associated_OOI_plot_X = globalMap.refOOI(index_OOIdetected,1);
                associated_OOI_plot_Y = globalMap.refOOI(index_OOIdetected,2);
                
                distFilter = [];
                assoc_x_buffer = [];
                assoc_y_buffer = [];

            end
            
            % Keep the prediction as backup (in case)
            oldXe = Xe;

            % Prediction step
            t = max(rS.tcx(1,rS.n), r.tcx(1,r.n));% take one of the max time
            ti = t-t0;
            ti = double(ti)/10000 ; % in counts
            Dt = ti-tPrev; %the last of current minus the last of previous
            tPrev = ti;


            Xe = RunProcessModel(Xe, speed, gyroZ, Dt);
            % Estimate new covariance after prediction
            % First , I evaluate the Jacobian matrix of the process model (see lecture notes).
            % Write the analytical expression on paper to understand the followign line.
            J = [ [1,0,-Dt*speed*sin(Xe(3)),0  ]  ; [0,1,Dt*speed*cos(Xe(3)),0] ;  [ 0,0,1, -Dt ] ; [ 0, 0, 0, 1] ] ;

            % then I calculate the new coveraince, after the prediction P(K+1|K) = J*P(K|K)*J'+Q ;
            P = J*P*J' + Q;
            
            % Do update if new OOIs confirmed
            if ~isempty(index_OOIdetected) && ~isempty(local_to_global_X)

                MeasuredRange = sqrt(OOIs.Centers(:,1).^2 + OOIs.Centers(:,2).^2);
                MeasuredBearing = atan2(OOIs.Centers(:,2),OOIs.Centers(:,1));

                % --------------- Kalman Filter update (observations)
                % Because this observation function is non-linear--> we need to get the Jacobians of h(X).
                % Jacobian for range only measurements (evaluated at the current expected value -->Xe)    

                % sqrt( (xi-x)^2+(yi-y)^2 ) for all the seen landmarks. I will need
                % this term for the next calculations.

                for foo_u = 1:length(index_OOIdetected)
                    ID = index_OOIdetected(foo_u);              % landmark IDs?
                    % occassionally length of index > points so this is necessary
                    if length(OOIdetected) <= length(index_OOIdetected)                        
                        ID_2 = OOIdetected(foo_u);
                        eDX = (globalMap.refOOI(ID,1)-Xe(1)-d*cos(Xe(3))) ;  % (xu-x)
                        eDY = (globalMap.refOOI(ID,2)-Xe(2)-d*sin(Xe(3))) ;  % (yu-y)
                        
                        %Defining variable
                        xa = globalMap.refOOI(ID,1);
                        x = Xe(1);
                        ya = globalMap.refOOI(ID,2);
                        y = Xe(2);
                        Phi = Xe(3);

                        dhdX = (2*x - 2*xa + 2*d*cos(Phi))/(2*((x - xa + d*cos(Phi))^2 + (y - ya + d*sin(Phi))^2)^(1/2));
                        dhdY = (2*y - 2*ya + 2*d*sin(Phi))/(2*((x - xa + d*cos(Phi))^2 + (y - ya + d*sin(Phi))^2)^(1/2));
                        dhdPhi = -(2*d*sin(Phi)*(x - xa + d*cos(Phi)) - 2*d*cos(Phi)*(y - ya + d*sin(Phi)))/(2*((x - xa + d*cos(Phi))^2 + (y - ya + d*sin(Phi))^2)^(1/2));
                        H_range = [ dhdX dhdY dhdPhi 0];
                        b_dhdX = -(y - ya + d*sin(Phi))/(((y - ya + d*sin(Phi))^2/(x - xa + d*cos(Phi))^2 + 1)*(x - xa + d*cos(Phi))^2);
                        b_dhdY = 1/(((y - ya + d*sin(Phi))^2/(x - xa + d*cos(Phi))^2 + 1)*(x - xa + d*cos(Phi)));
                        b_dhdPhi = ((d*cos(Phi))/(x - xa + d*cos(Phi)) + (d*sin(Phi)*(y - ya + d*sin(Phi)))/(x - xa + d*cos(Phi))^2)/((y - ya + d*sin(Phi))^2/(x - xa + d*cos(Phi))^2 + 1) - 1;
                        H_bearing = [b_dhdX b_dhdY b_dhdPhi 0];
                        H = [H_range; H_bearing];

                        % the expected distances to the landmarks ( "h(Xe)" ). Xe 1=x 2=y 3=gyro.
%                         ExpectedRange = eDD_range ;   % just a coincidence: we already calculated them for the Jacobian, so I reuse it. 
                        ExpectedRange = sqrt( (eDX)^2 + (eDY)^2 )  ;
                        ExpectedBearing = atan2(eDY, eDX) - Xe(3) + pi/2; % Output in rad

                        % Evaluate residual (innovation)  "Y-h(Xe)"
                        zR  = MeasuredRange(ID_2) - ExpectedRange ;      
                        zB  = MeasuredBearing(ID_2) - ExpectedBearing;
                        % Normalization
                        zB = mod(  zB+pi, 2*pi) - pi; % in deg

                        % Important hack                     
                        % Helps in case of too much difference between measured and
                        % expected values. Keeps it on track by pretending.
                        if   abs(zR)>0.5 
                            zR = 0.001;
                            fprintf('Residual value of range is too large\n'); 
                        end;

                        if abs(zB)>0.5
                            zB = 0.001;
                            fprintf('Residual value of bearing is too large\n');
                        end;

                        z = [zR; zB];

                        % ------ covariance of the noise/uncetainty in the measurements
                        % R is Diagonal --->because I believe the noises in the measurements
                        %  ,from different sensors, are independent.
                        RR = EKFconst.sdev_rangeMeasurement*EKFconst.sdev_rangeMeasurement*4; 
                        RB = EKFconst.sdev_bearingMeasurement*EKFconst.sdev_bearingMeasurement*4 ;
                        R = diag([RR,RB]);
                        % I multiply by 4 because I want to be conservative and assume
                        % twice the standard deviation that I believe does happen.

                        % Some intermadiate steps for the EKF (as in the lecture notes)
                        S = R + H*P*H'; 
                        iS = inv(S) ;           
                        K = P*H'*iS ;           % Kalman gain

                        % ----- do it...I am getting  X(k+1|k+1) and P(k+1|k+1)
                        Xe = Xe + K*z ;           % update the  expected value

                        % Yet another important hack
                        % If the difference of distance between last-new one after update is too far off (as
                        % might be caused by mis-scanning the new OOIs) disregard that, use the
                        % predicted Xe instead.
                        distance = sqrt(Xe(1) - track_robotCoordX(track_index))^2 + (Xe(2) - track_robotCoordY(track_index));
                        if distance > 0.15
                            Xe = oldXe;
                        end;
                        P = P-P*H'*iS*H*P ;     % update the Covariance
                        % all avalilable EKF updates done ...
                    end;
                end;
            end;
                
            track_bias(counter_bias) = Xe(4)*180/pi 
            track_bias_index(counter_bias) = counter_bias;

            track_index=track_index+1;
            track_robotCoordX(track_index) = Xe(1);
            track_robotCoordY(track_index) = Xe(2);
    
            set(handlePlot1,'xdata',globalMap.refMap(:,1),'ydata',globalMap.refMap(:,2)) ;  % Global map
            set(handlePlot2,'xdata',local_to_global_Map_X,'ydata',local_to_global_Map_Y) ;  % Local map in global
            set(handlePlot3,'xdata', globalMap.refOOI(:,1),'ydata', globalMap.refOOI(:,2)) ;% Global OOI
            set(handlePlot4,'xdata', local_to_global_X,'ydata', local_to_global_Y) ;        % Local OOI in global
            set(handlePlot5,'xdata', associated_OOI_plot_X,'ydata', associated_OOI_plot_Y) ;% Associated OOI
            set(handlePlot7,'xdata', track_robotCoordX,'ydata', track_robotCoordY) ;        % Robot path
            set(handlePlot6,'xdata', Xe(1),'ydata', Xe(2),'udata', 2*cos (Xe(3)), 'vdata',2*sin (Xe(3)) ) ; % Current robot location
            
            set(handlePlot8,'xdata',Laser_xx,'ydata',Laser_yy) ;                    % Map in local coordinates 
            set(handlePlot9,'xdata',OOIs.Centers(:,1),'ydata',OOIs.Centers(:,2)) ;  % local frame OOI
            
            set(handlePlot10,'xdata', track_bias_index, 'ydata', track_bias);
            
            counter_bias = counter_bias+1;
            if counter_bias>1000,
                counter_bias = 1;
                track_bias = [];
                track_bias_index = [];
            end;
            
        end;
        pause(pause_time) ;  % Hang time for my loop
        iterations = iterations + 1;
    end;

    
% ======================= %
%   Function Definitions  %
% ======================= %
function output = ProcessMyImuData(r, CurrAttitude) 

    % timeIMU used for finding dt and bias (mean of the first few secs)
    % IMU needed for EKF: accel angle z
    wx = r.data(4,1:r.n);     % rad/s
    wy = r.data(5,1:r.n);     % rad/s
    wz = r.data(6,1:r.n);     % rad/s

    % ... associated sample times (timestamps)
    time = r.tcx(1,1:r.n);      % time, 1 unit = 0.1ms. Possum Clock, at UGV node.
    time=double(time-time(1))/10000 ;    % now, expressed in seconds
    dtIMU = time(2);
    r2d = 180/pi;
    
    % Bias removal
    acc(1,:) = wx; 
    acc(2,:) = wy; 
    acc(3,:) = wz ;
    
    % Integrating to find angle
    for iIMU=1:length(wx),
        answ = IntegrateOneStepOfAttitude(acc(:,iIMU),dtIMU,CurrAttitude);
        CurrAttitude(:) =  transpose(answ) ;
        att(:,iIMU)= CurrAttitude;
    end;
    
    % Rad to deg
    att = att * r2d;
    
    % Normalization
    att = mod(  att+180, 2*180) - 180; % in deg
    output = att; % deg

end

function NewAttitude = IntegrateOneStepOfAttitude(gyros, dt, CurrentAttitude)
% for a small delta time, dt (s)
% CurrentAttitude is the current (initial) attitude, in radians
% gyros:vector with the gyros measurements, scaled in rad/sec
ang = CurrentAttitude ;
wx = gyros(1);
wy = gyros(2);
wz = gyros(3);
% -------------------------------
cosang1=cos(ang(1)) ;
cosang2=cos(ang(2)) ;
sinang1=sin(ang(1)) ;
roll = ang(1) + dt * (wx + (wy*sinang1 + wz*cosang1)*tan(ang(2))) ;
pitch = ang(2) + dt * (wy*cosang1 - wz*sinang1) ;
yaw = ang(3) + dt * ((wy*sinang1 + wz*cosang1)/cosang2) ;
% -------------------------------
NewAttitude= [roll,pitch,yaw];
return ;

end

%application specfic, for plotting, etc.    
function InitAngles(k)
   
   angle = single([0:360]'/360 *pi) ;  %LMS200 has 361 ranges/scan
   
   MyContext.cosinesAngles = cos(angle);
   MyContext.sinesAngles = sin(angle);
   MyContext.Mask0FFF = uint16(2^13-1);
   MyContext.Mask1110 = bitshift(uint16(7),13);
   
return;
end
  

% ..  About the structure that contains the read laser scans ....
% r.n                   : number of new laser frames
% r.Ranges(:,1:r.n)     : new laser scans
% r.times(1:r.n)        : timestamps
% NOTE that the useful data is contained in the first 'r.n' columns! 
% r.Ranges(:,j)         : laser scan # j , where j  /  1<=j<=r.n
% r.times(j)            : associated timestamp of laser scan #j
% .Ranges is class "uint16"     
% .times  is class "uint32"


function output = ShowData(scan,time)      
        
        Range = single(bitand(scan,MyContext.Mask0FFF));   
        intensities = bitand(scan,MyContext.Mask1110);
        %the first 13 bits are the range, in cm. <--API data format!!!!
        % See document "LMS2XX_data.pdf" for more details.

        xx = Range.* (MyContext.cosinesAngles * 0.01);   % in m
        yy = Range.* (MyContext.sinesAngles * 0.01);     % in m
%         set(MyContext.h1,'xdata',xx,'ydata',yy);
        
        timestamp = time;                 
        % Associated timestamp of this laser scan
        % Read document "PossumTimestamps.pdf" for extra comments.

        OOIget = ExtractOOIs(xx,yy,intensities);
        output = OOIget;
%         PlotOOIs(OOIget);
        
    return;
end


%.......................................................................
function r = ExtractOOIs(X,Y,intensities)
    L = length(X)-1;
    store_i = 1;
    lengthInd = 1;
    rowInd = 1;
    
    % Getting arrays of clusters of object
    for buffer_i=1:L;
        dista = sqrt((X(buffer_i+1)-X(buffer_i))^2 + (Y(buffer_i+1)-Y(buffer_i))^2);
        if (dista<=0.2),
            bufferX(rowInd, lengthInd) = X(buffer_i);
            bufferY(rowInd, lengthInd) = Y(buffer_i);
            bufferCol(rowInd, lengthInd) = intensities(buffer_i);
            lengthInd = lengthInd+1;
        else
            bufferX(rowInd, lengthInd) = X(buffer_i);
            bufferY(rowInd, lengthInd) = Y(buffer_i);
            bufferCol(rowInd, lengthInd) = intensities(buffer_i);
            amountN(rowInd) = lengthInd;
            lengthInd = 1;
            rowInd = rowInd+1;
        end;
    end;
    
    % Removing single points
    foo_a = find(amountN == 1);
    bufferX(foo_a,:) = [];
    bufferY(foo_a,:) = [];
    bufferCol(foo_a,:) = [];
    amountN(foo_a) = [];
    
    % Selecting only objects with size between 0.05 and 0.2 m
    for buffer_i2=1:length(amountN)
        x_size = max(bufferX(buffer_i2,1:amountN(buffer_i2)))-min(bufferX(buffer_i2,1:amountN(buffer_i2)));
        y_size = max(bufferY(buffer_i2,1:amountN(buffer_i2)))-min(bufferY(buffer_i2,1:amountN(buffer_i2)));
        sizeavg= (x_size+y_size)/2;
        if ((sizeavg <= 0.2) && (sizeavg >= 0.05))  % size in m
            r.N(store_i)         = store_i;
            r.Colors(store_i)    = max(bufferCol(buffer_i2,1:amountN(buffer_i2)));
            if r.Colors(store_i)>0
                r.Colors(store_i) = 1;
            end;
            r.Centers(store_i,1) = mean(bufferX(buffer_i2,1:amountN(buffer_i2)));
            r.Centers(store_i,2) = mean(bufferY(buffer_i2,1:amountN(buffer_i2)));
            r.Sizes(store_i)     = sizeavg;
            store_i = store_i+1;
        end;
    end;
       
    % Removing low-intensity points 
    foo_a = find(r.Colors == 0);
    r.N(foo_a) = []; % right now the this is still the original indices
    r.Colors(foo_a) = [];
    r.Centers(foo_a,:) = [];
    r.Sizes(foo_a) = [];
    
    r.N(1:length(r.Colors)) = 1:length(r.Colors); % to assign 1 2 3 ...
    
end
    
function Xnext=RunProcessModel(X,speed,GyroZ,dt) 
    Xnext = X + dt*[ speed*cos(X(3)) ;  speed*sin(X(3)) ; GyroZ-X(4) ; 0 ] ;
%     Xnext(3) = mod(Xnext(3) + pi, 2*pi) - pi;
return;
end
    
    
end
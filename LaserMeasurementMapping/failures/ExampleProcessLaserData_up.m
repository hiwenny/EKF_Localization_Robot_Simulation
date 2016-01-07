% FAILED ATTEMPT 
% What's wrong with it?
% Not recorded into r. struct every iteration
%function MyProgram(file)
function MyProgram()
file ='Laser__2.mat';
data = load(file); data = data.XX; 

N = data.N;

for i=1:3:N,                        % in this example I skip some of them..
    ProcessScan(data.Ranges(:,i));
%     pause(1);
%     pause(0.01) ;                   % wait for ~10ms
end;

return;

function ProcessScan(scan)

angles = [0:360]'*0.5* pi/180 ;         % associated angle for each range of scan

% separate range and intensity from measurements.
mask0001111111111111 = uint16(2^13-1);
mask1110000000000000 = bitshift(uint16(7),13)  ;
intensities = bitand(scan,mask1110000000000000);
ranges    = single(bitand(scan,mask0001111111111111))*0.01;% range expressed in meters

% points, expressed in Cartesian. From the sensor's perpective.
X = cos(angles).*ranges;
Y = sin(angles).*ranges;    

% plot.
figure(3) ; clf(); hold on;
plot(X,Y,'b.');                     % all points
ii = find(intensities~=0);            % find those "pixels" that had intense reflection
plot(X(ii),Y(ii),'*r');             % plot highly reflective ones
axis([-10,10,0,20]);                % focuses plot on this region ( of interest in L220)


% to be done (by you)
OOIs = ExtractOOIs(X,Y,intensities);
save('OOI.mat','OOIs');
PlotOOIs(OOIs);

end

function r = ExtractOOIs(X,Y,intensities)
    L = length(X)-1;
    k = 1;
    lengthInd = 1;
    rowInd = 1;
    
    % Getting objects
    for j=1:L;
        dist = sqrt((X(j+1)-X(j))^2 + (Y(j+1)-Y(j))^2);
        if ((dist<=0.2) && (dist >= 0.05)),
            bufferX(lengthInd) = X(j);
            bufferY(lengthInd) = Y(j);
            bufferCol(lengthInd) = intensities(j);
            lengthInd = lengthInd+1;
            
        else
            bufferX(lengthInd) = [];
            bufferY(lengthInd) = [];
            bufferCol(lengthInd) = [];
            lengthInd = 1;
        end;
                    sizeavg= ((max(bufferX)-min(bufferX)) + (max(bufferY)-min(bufferY)))/2;
%             if ((sizeavg <= 0.2) && (sizeavg >= 0.05))
            r.N(k)         = k;
            r.Colors(k)    = max(bufferCol);
            r.Centers(k,1) = mean(bufferX);
            r.Centers(k,2) = mean(bufferY);
            r.Sizes(k)     = sizeavg;
            k = k+1;
            m = 1;
            bufferX   = [];
            bufferY   = [];
%             end;        
                
    end;
end
    
function PlotOOIs(OOIs)
    if isempty(OOIs.Centers), return ; end;
    % Plotting the centers
    plot(OOIs.Centers(:,1), OOIs.Centers(:,2), '*r');
    % Plotting the centers with high intensity
    CI = find(OOIs.Colors > 1);
    plot(OOIs.Centers(CI,1),OOIs.Centers(CI,2),'go');
return;
end
    




end









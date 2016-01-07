
% example program, for processing saved laser scans. 

%function MyProgram(file)
function MyProgram()
file ='Laser__2.mat';
data = load(file); data = data.XX; 

N = data.N;
data
for i=1:N,                        % in this example I skip some of them..
    ProcessScan(data.Ranges(:,i));
%     pause(0.05) ;                   % wait for ~10ms
end;

return;

function ProcessScan(scan)

angles = [0:360]'*0.5* pi/180;         % associated angle for each range of scan

% separate range and intensity from measurements.
mask0001111111111111 = uint16(2^13-1);
mask1110000000000000 = bitshift(uint16(7),13)  ;
intensities = bitand(scan,mask1110000000000000);
ranges    = single(bitand(scan,mask0001111111111111))*0.01; % range expressed in meters

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
        dista = sqrt((X(j+1)-X(j))^2 + (Y(j+1)-Y(j))^2);
        if (dista<=0.2),
            bufferX(rowInd, lengthInd) = X(j);
            bufferY(rowInd, lengthInd) = Y(j);
            bufferCol(rowInd, lengthInd) = intensities(j);
            lengthInd = lengthInd+1;
        else
            bufferX(rowInd, lengthInd) = X(j);
            bufferY(rowInd, lengthInd) = Y(j);
            bufferCol(rowInd, lengthInd) = intensities(j);
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

    % Selecting only objects between 0.05 and 0.2 m
    for m=1:length(amountN)
        x_size = max(bufferX(m,1:amountN(m)))-min(bufferX(m,1:amountN(m)));
        y_size = max(bufferY(m,1:amountN(m)))-min(bufferY(m,1:amountN(m)));
        sizeavg= (x_size+y_size)/2;
        if ((sizeavg <= 0.2) && (sizeavg >= 0.05))
            r.N(k)         = k;
            r.Colors(k)    = max(bufferCol(m,1:amountN(m)));
            r.Centers(k,1) = mean(bufferX(m,1:amountN(m)));
            r.Centers(k,2) = mean(bufferY(m,1:amountN(m)));
            r.Sizes(k)     = sizeavg;
            k = k+1;
        end;
    end;
                
end
    
function PlotOOIs(OOIs)
    if isempty(OOIs.Centers), return ; end;
    % Plotting the centers
    plot(OOIs.Centers(:,1), OOIs.Centers(:,2), '*r');
    % Plotting the centers with high intensity
    CI = find(OOIs.Colors > 0);
    plot(OOIs.Centers(CI,1),OOIs.Centers(CI,2),'go');
return;
end
    




end









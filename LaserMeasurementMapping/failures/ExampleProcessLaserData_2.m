
% example program, for processing saved laser scans. 

%function MyProgram(file)
function MyProgram()
file ='Laser__2.mat';
data = load(file); data = data.XX; 

N = data.N;

for i=1:3:N,                        % in this example I skip some of them..
    ProcessScan(data.Ranges(:,i));
    %pause ;
    pause(0.01) ;                   % wait for ~10ms
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
figure(1) ; clf(); hold on;
plot(X,Y,'b.');                     % all points
ii = find(intensities~=0);            % find those "pixels" that had intense reflection
plot(X(ii),Y(ii),'+r');             % plot highly reflective ones
axis([-10,10,0,20]);                % focuses plot on this region ( of interest in L220)



% to be done (by you)
OOIs = ExtractOOIs(ranges,intensities) ;
PlotOOIs(OOIs);

end

function r = ExtractOOIs(ranges,intensities)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = [];
    
    % your part
    
return
end
    
function PlotOOIs(OOIs)
    if OOIs.N<1, return ; end;
    % your part
return;
end
    




end









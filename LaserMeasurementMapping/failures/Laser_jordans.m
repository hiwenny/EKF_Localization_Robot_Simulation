
% A program showing how to read scans from the external laser that you will use for observing
% the robots and certain obstacles in the area of operation.
% Jose Guivant - S2/2012 - MTRN3100

function main()

% initialize the API
Object =  iniApiMTRN3100(); % might need to declare as global
if Object.Ok<1, return ; end;
% see note 0

% Some auxiliary variables for scaling values and for extracting bits, etc.
K_cm2M = 0.01;
mask0x1FFF = uint16(2^13-1) ;  % mask for extracting lowest 13 bits of a 16 bits unsigned integer
mask0xE000 = uint16(7* 2^13);  % mask for extracting highest 3 bits of a 16 bits unsigned integer

% ---------------------------------------
% create a figure where to show the laser data.
figure(1) ; clf; hold on ;
hp1 = plot(0,0,'b.');
hp2 = plot(0,0,'r.');
hp3 = plot(0,0,'gs');
hp4 = plot(0,0,'k*');
%hp5 = plot(o,o,
axis([-6,6,0,6]);
xlabel('X (m)'); ylabel('Y (m)'); zoom on ;
% ---------------------------------------

% Some auxiliary variables for converting from polar to Cartesian

% angle associated to the 361 readings / scan. Each reading for 0.5
% degrees, from 0 to 180 degrees.
A = single([1:361]'-1)*0.5 * pi /180 ;
cosA = cos(A);
sinA = sin(A);
clear A ;  % I do not need A anymore.
% ----------------------------------------


%initialise memory storage for old OOIs
DOI = initialiseDOI();

for i=1:1000,                        % run loop during some number of iterations.
    
    
    % in this example I read available scans from laser scan unit #2, up to 5 scans.
    % although I just want the last one.
    % Laser unit #2, because the system can deal with many, but just #2 is
    % installed in L220.
    r = Object.GetLaserScansLMS200(2, 5);
    
    % The function retuns a structure, whose field ".newLaserFrames" tells you how many
    % messages were available in this reading.
    
    % You may want to process all the scans you read
    
    %  for j=1:r.newLaserFrames,                   % for each scan we process it..
    %       % do something with each read scan
    %       ProcessScan( r.Ranges(:,j),r.times(j)) ;
    %   end;
    
    
    
    % OR just process the last one  (I am doing it)
    
    
    if (r.newLaserFrames>0)
        j = r.newLaserFrames;
        scan = r.Ranges(:,j);
        time = r.times(j); %time is not used
        
        
        % Each scan is compossed by 361 readings. Each reading gives the
        % Range (in cm) and the associated "color" (intensity fo
        % reflection). They are both encoded in just 16 bits.
        
        Range = single( bitand(scan , mask0x1FFF ))*K_cm2M;  %K_cm2M is a area variable
        intensity = bitand(scan , mask0xE000 ); % bitand returns the bit wise AND of scan and mask0xE000
        % "Range" has the POLAR magnitude of the laser scan.
        
        % I want to see the associated points in Cartesian
        xx = Range.*cosA;
        yy = Range.*sinA;
        
        %find high intensity pics
        ii = find(intensity>0);
        
        %====================== My Functions =========================
        OOI = initialiseOOI(); %initialises OOI
        OOI = processscan(xx(ii), yy(ii), OOI, time); % gives OOIs and size of OOIS
        if(i>4)
            OOI = identityof(OOI, DOI, time);
        end;
        %====================== End of My Functions =======================
        
        
        
        
        
        %texth(OOI.n,OOI.n);
        
        %text1=text('String', OOI.ID(1));%texth=text('String', OOI.ID');
        %text2=text('String', OOI.ID(2));
        %set(hp4, 'xdata', texthandle(:,:),'ydata', texthandle(:,:));
        
        % show data:
        % show pixels in figure. All of them in blue.
        set(hp1, 'xdata', xx , 'ydata', yy );
        % just the HIGHLY reflective ones in red
        set(hp2, 'xdata', xx(ii) , 'ydata', yy(ii) );
        % graphs OOIs
        set(hp3, 'xdata', OOI.xx, 'ydata', OOI.yy);
        
        %set(text1, 'position', [OOI.xx(2)+.01, OOI.yy(2)+.01, 0]);  %set(texth, 'position', [OOI.xx', OOI.yy', zeros(numel(OOI.xx),1)]);
        %set(text2, 'position', [OOI.xx(2)+.01, OOI.yy(2)+.01, 0]);
        
        % create a new handle when you find a new opbject 
        
        
        % print values - maybe make print function
        fprintf('i : [');
        Show(i);
        fprintf(']\n');
        fprintf('OOI.n : [');
        Show(OOI.n);
        fprintf(']\n');
        fprintf('OOI.xx : [');
        Show(OOI.xx);
        fprintf(']\n');
        fprintf('OOI.yy : [');
        Show(OOI.yy);
        fprintf(']\n');
        fprintf('OOI.size : [');
        Show(OOI.size);
        fprintf(']\n');
        fprintf('OOI.age : [');
        Show(OOI.age);
        fprintf(']\n');
        fprintf('OOI.ID : [');
        Show(OOI.ID);
        fprintf(']\n');
        fprintf('\n');
        
        
        %reset OOI for the moment
        DOI = StoreData(DOI,OOI, i);
        
        % clear
        clear OOI;
        %clear text1;
        %if(i>20)
        %    clear DOI(i-20);
        %end;
        
        if(i>20)
            for q=1:i;%q=i-5:-1:i-20,
                % graphs old OOIS (or DOIs)
                set(hp4, 'xdata', DOI.b.xx, 'ydata', DOI.b.yy);
                set(hp4, 'xdata', DOI.c.xx, 'ydata', DOI.c.yy);
            end;
        end;
    end;
    
    pause(0.1);
end;
% program ends
fprintf('Program ENDS..\n');

return;

% ---------------- END of Program -------------------

function OOI = initialiseOOI()
    OOI.n=0;
    OOI.age=0;
    OOI.ID=0;
return;
        
function DOI = initialiseDOI()
    
    DOI.c.xx=0;
    DOI.c.yy=0;
    
    DOI.b.xx=0;
    DOI.b.yy=0;
    
    
    DOI.a.xx=0;
    DOI.a.yy=0;
    
    %start time
    DOI.a.starttime(1)=1000000000;%big start time
    DOI.a.starttime(2)=1000000000;%big start time
    DOI.a.starttime(3)=1000000000;%big start time
    DOI.a.starttime(4)=1000000000;%big start time
    DOI.a.starttime(5)=1000000000;%big start time
    DOI.a.starttime(6)=1000000000;%big start time
    DOI.a.starttime(7)=1000000000;%big start time
    
return;
            
function Show(data)
    for i=1:numel(data)
        fprintf(' %2.4f', data(i));
    end;
return;
                
function OOI = processscan(xx, yy, OOI,time)
    
    for w=1:numel(xx-1), %starts at 1 goes to number of xx-1
        for q=w+1:numel(xx), %starts at 1+w goes to number of xx
            OOI=addOOI(xx, yy, OOI, w, q,time);
        end;
    end;
    clear w;
    clear q;
    for w=1:OOI.n-1,
        for q=OOI.n:-1:w+1, %w+1:OOI.n, gets better results if it scans backwards
            OOI=removeOOI(OOI, w, q);
        end;
    end;
return;
                    
% function identities and adds OOIS
function OOI = addOOI(xx, yy, OOI, w, q, time)
    if(sqrt((xx(w)-xx(q))^2+(yy(w)-yy(q))^2) <.1) %length between two points
        OOI.xx(OOI.n+1) = xx(w)+(xx(q) - xx(w))/2;%average spot
        OOI.yy(OOI.n+1) = yy(w)+(yy(q) - yy(w))/2;
        OOI.n=OOI.n+1;
        
        % calculates size
        OOI.size(OOI.n)= abs((xx(w)-xx(q))*(yy(w)-yy(q)));
        
        %add start time
        OOI.starttime(OOI.n) = time;
    end;
    
return;
                        
% erases OOIs that are close to together, updates size and make it more
% accurate
function OOI = removeOOI(OOI, w, q)
    if(sqrt((OOI.xx(w)-OOI.xx(q))^2+(OOI.yy(w)-OOI.yy(q))^2) <.10)
        OOI.xx(w) = OOI.xx(w)+(OOI.xx(q)- OOI.xx(w))/2;%average point
        OOI.yy(w) = OOI.yy(w)+(OOI.yy(q)- OOI.yy(w))/2;
        
        OOI.size(w)=OOI.size(w)+OOI.size(q);
        OOI.size(q)=[];
        OOI.starttime(q)=[];
        
        OOI.xx(q)=[]; %removes value
        OOI.yy(q)=[];
        
        OOI.n=OOI.n-1;
    end;
return;                         
                                
function DOI = StoreData(DOI, OOI, i)
    if(i>3)
        DOI.c.xx=DOI.b.xx;
        DOI.c.yy=DOI.b.yy;
        %DOI.c.starttime=DOI.b.starttime;
    end;
    
    if(i>2)
        DOI.b.xx=DOI.a.xx;
        DOI.b.yy=DOI.a.yy;
        %DOI.b.starttime=DOI.a.starttime;
    end;
    DOI.a.ID=[1 2 3 4 5 6 7]; %OOI.n; %interesting
    DOI.a.xx=OOI.xx;
    DOI.a.yy=OOI.yy;
    for w=1:numel(OOI.starttime),
        if(DOI.a.starttime(w)>OOI.starttime(w)),
            DOI.a.starttime(w)=OOI.starttime(w);
        end;
    end;
    if numel(OOI.starttime)<numel(DOI.a.starttime),
        x=numel(DOI.a.starttime)-numel(OOI.starttime);
        for q=1:x,
            DOI.a.starttime(q+numel(OOI.starttime))= 100000000;
        end;
    end;
return;
                                    
% data association function
function OOI = identityof(OOI, DOI, time)

    if(numel(DOI.a.xx)<numel(OOI.xx))
        x=numel(DOI.a.xx);
    else
        x=numel(OOI.xx);
    end;
    
    for w=1:x,
        for q=w:x,
            %OOI.xx and OOI.yy with DOI.a.xx and DOI.a.yy
            if(sqrt((DOI.a.xx(w)-OOI.xx(q))^2+(DOI.a.yy(w)-OOI.yy(q))^2) <.15)
                OOI.ID(q)=DOI.a.ID(w);
                
                %age
                if(DOI.a.starttime(w)>OOI.starttime(q)),
                    DOI.a.starttime(w)=OOI.starttime(q);
                end;
                OOI.age(w)=(time-DOI.a.starttime(w))/10000;
            end;
        end;
    end;
return;


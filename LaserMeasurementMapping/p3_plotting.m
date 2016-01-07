function main()
global MyStruct;

MyStruct.API =  iniApiMTRN3100();          % Api initialization
if MyStruct.API.Ok<1, return ; end;          

MyStruct.SID = 170;                        % sender ID AA in hex
MyStruct.DID = 187;                        % destination ID BB in hex
MyStruct.FlagPin = 1;

% initializing buffer
Buffer.L=200;                   % buffer length
Buffer.Data=zeros(3,Buffer.L);  % making 2xL matrix of zeros
Buffer.index=0;                 % buffer index?? 
cx=0;

% CreateControlPanel();           % creates control panel
%MyStruct.FlagDoPlot = 1;        % flag for start/stop plotting
MyStruct.FlagDoBye = 0;         % flag for exiting program

% create 3 plots, to be updated dynamically.
figure(1) ;clf ; 
subplot(2,1,1)
hold on ;
DataPlot.hp1 = plot(Buffer.Data(1,:),'b') ;
DataPlot.hp2 = plot(Buffer.Data(2,:),'r') ;

while 1,                             % run loop indefinitely
    if MyStruct.FlagDoBye,           % response to EXIT button press
        break;
    end;
    
%    if MyStruct.FlagDoPlot,          % plot flag NOT USED
         r  =MyStruct.API.GetMTRN3100Msg(1,10) ;    % read pending messages
        for j=1:r.nr,
            % data processing
            
%             % debugging
%             SID=r.Data(1)
%             DID=r.Data(2)
%             LEN=r.Data(3)
%             data4=r.Data(4)
%             data5=r.Data(5)
%             data6=r.Data(6)
%             data7=r.Data(7)
%              data8=r.Data(8)
%              data9=r.Data(9)
%              data10=r.Data(10)
%              data11=r.Data(11)
            
            acc_flag = r.Data(6)
            if acc_flag == 1,
                acc_data = -(double(r.Data(4)))/100
            else
                acc_data = double(r.Data(4))/100
            end;
            
            gyr_flag = r.Data(10)
            
            if gyr_flag == 1,
                gyr_data = -double(r.Data(8))
            else
                gyr_data = double(r.Data(8))
            end;
    
            % some buffering .......
            Buffer.index=Buffer.index+1;
            if Buffer.index>Buffer.L,
            %    Buffer.Data(:,:)=0;        % refreshes the buffer matrix
                Buffer.index=1;            % returning the index to 1
            end;
    
            Buffer.Data(1,Buffer.index)=acc_data;
            Buffer.Data(2,Buffer.index)=gyr_data;
            %.......................

            % plotting
            cx=cx+1;            
            if (cx>10),     
                cx=0;           % every 10 data, then refreshes
                set( DataPlot.hp1 , 'ydata', Buffer.Data(1,:));
                set( DataPlot.hp2 , 'ydata', Buffer.Data(2,:));
            end;
        end;
    pause(0.01) ;
%    end;
end;

% program ends
close all;
fprintf('End of program\n');

return;

% example program, for processing saved laser scans.

function MyProgram()                  % function MyProgram(file)

  file ='Laser__2.mat';
  data = load(file); 
  data = data.XX;

  N = data.N;

  for i = 1:3:N,                        % in this example I skip some of them..
    ProcessScan(data.Ranges(:,i));
    pause(0.01);                        % pause, wait for ~10ms  
  end;
  
  return;

function ProcessScan(scan)

  angles = [0:360]'*0.5* pi/180;         % associated angle for each range of scan

  % separate range and intensity from measurements.
  mask0001111111111111 = uint16(2^13-1);
  mask1110000000000000 = bitshift(uint16(7),13);
  intensities = bitand(scan,mask1110000000000000);
  ranges = single(bitand(scan,mask0001111111111111))*0.01; % range expressed in metres

  % points, expressed in Cartesian. From the sensor's perpective.
  X = cos(angles).*ranges;
  Y = sin(angles).*ranges;

  % plot.
  figure(1); clf(); hold on;
  plot(X,Y,'b.');                % all points
  ii = find(intensities~=0);     % find those "pixels" that had intense reflection
  plot(X(ii),Y(ii),'r.');        % plot highly reflective ones
  axis([-10,10,0,20]);           % focuses plot on this region (of interest in L220)
  xlabel('X Distance (m)');
  ylabel('Y Distance (m)');
   
  % to be done (by you)
  OOIs = ExtractOOIs(ranges,intensities,X,Y);
  save('OOI.mat','OOIs');
  PlotOOIs(OOIs);
  
end

function r = ExtractOOIs(ranges,intensities,X,Y)
  r.N = 0;
  r.Colors = [];
  r.Centers = [];
  r.Sizes = [];
  r.Int = [];
    
  % your part
  arrX = [];
  arrY = [];
  arrZ = [];
  
  len = length(X);
  count = 1;
  ind = 1;

  for j = 1:len-1,
      if ((abs(X(j)-X(j+1)) <= 0.20) && (abs(Y(j)-Y(j+1)) <= 0.20)),
          arrX(count,ind) = X(j);
          arrX(count+1,ind) = X(j+1);
          arrY(count,ind) = Y(j);
          arrY(count+1,ind) = Y(j+1);
          arrZ(ind,1) = count;

          if ((intensities(j) > 0) || (intensities(j+1) > 0)),
              arrZ(ind,2) = 1;
          end
          
          count = count + 1;
      else
          arrX(count,ind) = X(j);
          arrY(count,ind) = Y(j);
          arrZ(ind,1) = count;
          
          if (intensities(j) > 0),
              arrZ(ind,2) = 1;
          end
          
          count = 1;
          ind = ind + 1;
      end
      
  end

  tally = 0;
  count = 1;

  for k = 1:length(arrZ),
      if (arrZ(k) > 1),
          maxX = max(arrX(1:arrZ(k,1),k));
          minX = min(arrX(1:arrZ(k,1),k));
          meanX = mean(arrX(1:arrZ(k,1),k));
          maxY = max(arrY(1:arrZ(k,1),k));
          minY = min(arrY(1:arrZ(k,1),k));
          meanY = mean(arrY(1:arrZ(k,1),k));
          dX = maxX - minX;
          dY = maxY - minY;
          dia = (dX + dY)/2;
          tally = tally + 1;
          
          if ((dia >= 0.05) || (dia <= 0.20)),
              r.N(count) = count;
              
              if (arrZ(k,2) == 1)
                  r.Int(count) = 1;
              else
                  r.Int(count) = 0;
              end
              
              r.Centers(1,count) = meanX;
              r.Centers(2,count) = meanY;
              r.Sizes(count) = dia;
              count = count + 1;
          end
          
      end
      
  end
  
  return;
end

function PlotOOIs(OOIs)
  if OOIs.N<1, 
    return;
  end;
  
  % your part
  plot(OOIs.Centers(1,:),OOIs.Centers(2,:),'r*')
  CI = find(OOIs.Int == 1);
  plot(OOIs.Centers(1,CI),OOIs.Centers(2,CI),'r.');
  
  return;
end

end




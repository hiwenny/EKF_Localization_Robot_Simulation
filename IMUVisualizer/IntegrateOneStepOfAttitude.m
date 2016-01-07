% =========================================================================
% Integration function

function NewAttitude = IntegrateOneStepOfAttitude(gyros, dt, CurrentAttitude)
% for a small delta time , dt
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

% =========================================================================

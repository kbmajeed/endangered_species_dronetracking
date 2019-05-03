function [sys,x0,str,ts] = quadrotorbacksteppingcontrollertracker(t,x,u,flag)
%quadrotorsfunc An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = f(x,u)
%      y  = h(x)
%   See sfuntmpl.m for a general S-function template.
%   See also SFUNTMPL.
%   Copyright 1990-2009 The MathWorks, Inc.
switch flag,
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;
  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);
  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
% end quadrotorsfunc

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 10;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [1; 0; 1; 0; 1; 0; 1; 0; 1; 0; 1; 0];
%x0  = zeros(12,1);
str = [];
ts  = [0 0];
% end mdlInitializeSizes

%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(t,x,u)
g    = 9.81;l    = 0.23;wr   = 1; Ixx  = 7.5e-3;
Iyy  = 7.5e-3;Izz  = 1.3e-2;Jr   = 6e-3;m    = 1;
a1 = (Iyy-Izz)/Ixx;  a2 = Jr/Ixx; a3 = (Izz-Ixx)/Iyy;a4 = Jr/Iyy;
a5 = (Ixx-Iyy)/Izz;  b1 = l/Ixx;  b2 = l/Iyy;b3 = 1/Izz;

xdot    = [x(2);
           x(4)*x(6)*a1 - x(4)*wr*a2 + b1*u(2);
           x(4);
           x(2)*x(6)*a3 + x(2)*wr*a4 + b2*u(3);
           x(6);
           x(2)*x(4)*a5 + b3*u(4)
           x(8);
           g - (u(1)/m)*(cosd(x(1))*cosd(x(3)));
           x(10);
           - (u(1)/m)*(sind(x(1))*sind(x(5)) + cosd(x(1))*sind(x(3))*cosd(x(5)));
           x(12);
           - (u(1)/m)*(sind(x(1))*cosd(x(5)) - cosd(x(1))*sind(x(3))*sind(x(5)));];
sys = xdot;
% end mdlDerivatives

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys=mdlOutputs(t,x,u)
g    = 9.81;l    = 0.23;wr   = 1; % wr saturates at wr>1000
Ixx  = 7.5e-3;Iyy  = 7.5e-3;Izz  = 1.3e-2;Jr   = 6e-3;m    = 1;
a1 = (Iyy-Izz) / Ixx; a2 = Jr/Ixx; a3 = (Izz-Ixx) / Iyy;
a4 = Jr/Iyy; a5 = (Ixx-Iyy) / Izz; b1 = l/Ixx; b2 = l/Iyy; b3 = 1/Izz;

x5d = 3;     %ksi
x7d = 10;    %z
x9d = u(5);  %x
x11d= u(6);  %y
x10d2dot  = u(7); %xddot
x12d2dot  = u(8); %yddot

x1ddot = 0;x3ddot  =0; x5ddot =0;x7ddot  =0;x1d2dot =0;
x3d2dot= 0;x5d2dot =0;x7d2dot =0;%x10d2dot=0;x12d2dot=0;

kpy = 4.9; kdy = 8.3; kpx = 4.9; kdx = 7.3;
c1  =  11.52; c2  =  8.40;  c3  =  8.00;  c4  =  7.50; 
c5  =  13.6263; c6  =  13.5392;  c7  =  1.54;   c8  =  3.49;

z5 = x5d - x(5); z6 = x(6)- x5ddot - c5*z5;
z7 = x7d - x(7); z8 = x(8)- x7ddot - c7*z7;

u1 = (m/(cosd(x(1))*cosd(x(3))))*(-z7 + g - x7d2dot - c7*x7ddot + c7*x(8) + c8*z8);      % z
  phid   =  kpy*(x(11) - x11d) + kdy*(x(12) - x12d2dot);
  %phid   =  kpy*(x11d - x(11)) + kdy*(x12d2dot - x(12));
% if phid > 20;
% phid = 20
% elseif phid < 20
% phid = -20
% end
  thetad =  kpx*(x(9) - x9d)  + kdx*(x(10) - x10d2dot);
  %thetad =  kpx*(x9d - x(9))  + kdx*(x10d2dot - x(10));
% if thetad > 20;
% thetad = 20
% elseif thetad < 20
% thetad = -20
% end
x1d = cosd(x(5))*phid - sind(x(5))*thetad;
x3d = sind(x(5))*phid + cosd(x(5))*thetad;

z1 = x1d - x(1); z2 = x(2)- x1ddot - c1*z1;
z3 = x3d - x(3); z4 = x(4)- x3ddot - c3*z3;

u2 = (1/b1)*(-c2*z2 + z1 - x(4)*x(6)*a1 + x(4)*wr*a2 + x1d2dot + c1*x1ddot - c1*x(2)); % phi
u3 = (1/b2)*(-c4*z4 + z3 - x(2)*x(6)*a3 - x(2)*wr*a4 + x3d2dot + c3*x3ddot - c3*x(4)); % theta
u4 = (1/b3)*(-c6*z6 + z5 - x(2)*x(4)*a5 + x5d2dot + c5*x5ddot - c5*x(6));              % ksi

sys = [ x(1); x(3); x(5); x(7); x(9); x(11); u1; u2; u3; u4];
% end mdlOutputs

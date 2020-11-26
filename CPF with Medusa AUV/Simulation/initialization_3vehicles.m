%% Initialization of Data for simulation
%% Setup the patern of the path and formation
% L=100;                          % Length of first straighline 
% R=25;                           % Circumference
% d1=10;                           % offset from x axis for vehicle 1
% d2=5;                          % offset from x axis for vehicle 2
% d3=15;
% d4=0;
% d5=20;
% m1=10;
% m2=5;
% m3=5;
% m4=0;
% m5=0;
% vd=0.8;                           % constant speed profile for straighline and also circumference
% --------
L=50;                          % Length of first straighline 
R=12;                           % Circumference
d1=-3;                           % offset from x axis for vehicle 1
d2=0;                          % offset from x axis for vehicle 2
d3=3;
d4=0;
d5=15;
m1=0;
m2=0;
m3=0;
m4=0;
m5=0;
vp=.5;                           % constant speed profile for straighline and also circumference


%       x
%       ^
%       |
%       |
%       |
%       |
%       |
%       |--d1-------V1               |
%       |                            |
%       |                            | 
%       |--d2--V2         V3         |
%       |--d3--------------          m1
%       |                            | 
%       |--d4--V4         V5         | 
%       |--d5--------------  |       |  
%       |                    R
%       |                    |
%       |-------------------------------------------------------> y
%       |
%       |
%       |
%% Setup current 
    Vcx=0.0;                              
    Vcy=0.0;
%% Setup initial position of vehicles
% Vehicle 1
    x10=-10;
    y10=-10;
    yaw10=0;
% Vehicle 2
    x20=-5;
    y20=10;
    yaw20=-pi/2;
% Vehicle 3
    x30=0;
    y30=-5;
    yaw30=pi/2;
% Vehicle 4
    x40=-0;
    y40=-0;
    yaw40=0;
% Vehicle 5
    x50=0;
    y50=30;
    yaw50=pi/3;

%% Sampling times
   Ts=0.01;                 % Sampling time for animation after simulation
%% Setup for Network:
   Delay=Ts;                 % Upper bound of Delay time for block transmitters
   tau_bar=Delay+0.2;       % Maximum of delay time
   Mode=2;                  % MODE=0 => continous; MODE=1 => discrete, MODE=2 => Event 
%% Setup for time trigger and event trigger communication   
   epsilon=.5;
   T_Trigger=4;             % Noted that T_Trigger > tau_bar=Delay
   
%% use to compare between several settup
% p22=p2;
% p33=p3;
% pd22=pd2;
% pd33=pd3;
% yaw22=yaw2;
% yaw33=yaw3;


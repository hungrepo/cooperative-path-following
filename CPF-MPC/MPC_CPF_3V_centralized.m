% clear all;
% close all;
function CentralizedMPCPF
clear all;
close all;

%% Initialization
T = 200;
Ts = 0.1;
N=T/Ts;
t=[0:N].*Ts;
% VEHICLES
% Vehicle 1
p1_0=[30;-10]; psi1_0=0;
x_robot1_0 = [p1_0;psi1_0];
% Vehicle 2
p2_0=[10;-5]; psi2_0=0;
x_robot2_0 = [p2_0;psi2_0];
% Vehicle 3
p3_0=[-30;-15]; psi3_0=0;
x_robot3_0 = [p3_0;psi3_0];

% PATHS
% Path 1
gamma1_0=5;
a1=10;omega1=0.1;phi1=0;d1=20;
path1_data=[a1;omega1;phi1;d1];
x_path1_0=ini_path(path1_data,gamma1_0);
% Path 2
gamma2_0=-5;
a2=10;omega2=0.1;phi2=0;d2=0;
path2_data=[a2;omega2;phi2;d2];
x_path2_0=ini_path(path2_data,gamma2_0);  
% Path 3 
gamma3_0=0; 
a3=10;omega3=0.1;phi3=0;d3=-20;
path3_data=[a3;omega3;phi3;d3];
x_path3_0=ini_path(path3_data,gamma3_0);  

% setup integrator for all paths
input_path.Ts = Ts;
input_path.nSteps = 4;


%% Formulate NLP for MPC
Np = 10;               % number of control intervals
Tp = Ts*Np;            % Time horizon
                  
Lap=[ 1  -1  0;        % network
     -1   2 -1;
      0  -1  1];

import casadi.*
[nlp_sol,w0,lbw,ubw,lbg,ubg,nx,nu,k3,k4]=NLP(Np,Tp,Ts,path1_data,path2_data,path3_data,Lap);

%% Closed loop Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
x_robot1=x_robot1_0;
x_robot2=x_robot2_0;
x_robot3=x_robot3_0;

x_path1=x_path1_0;
x_path2=x_path2_0;
x_path3=x_path3_0;

% V_mpc=[];
u_mpc1=[];
u_mpc2=[];
u_mpc3=[];
V_mpc=[];
u_mpc=[];
% Run close loop 
for i = 1:N

% Compute PF error   
e_pf1=epf(x_robot1,x_path1);
e_pf2=epf(x_robot2,x_path2);
e_pf3=epf(x_robot3,x_path3);
gamma1=e_pf1(4);
gamma2=e_pf2(4);
gamma3=e_pf3(4);
x0_1=e_pf1;
x0_2=e_pf2;
x0_3=e_pf3;
x0=[x0_1;x0_2;x0_3];
eta=Lap*[gamma1;gamma2;gamma3];
V_mpc1=0.5*k3*log(1+x0_1(1)^2 + x0_1(2)^2) + 0.5*x0_1(3)^2;
V_mpc2=0.5*k3*log(1+x0_2(1)^2 + x0_2(2)^2) + 0.5*x0_2(3)^2;
V_mpc3=0.5*k3*log(1+x0_3(1)^2 + x0_3(2)^2) + 0.5*x0_3(3)^2;
V_con= 0.5*(eta')*eta;
V_mpc=[V_mpc V_mpc1+V_mpc2+V_mpc3+V_con];
% Solve OPC for vehicle 1
    w0(1:nx)=x0;
    lbw(1:nx)=x0;
    ubw(1:nx)=x0;
    sol = nlp_sol('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                  'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    u_mpci=w_opt(13:21);
    r_d1=u_mpci(1);
    u_gamma1=u_mpci(2);
    u_dt1=u_mpci(3);
    
    r_d2=u_mpci(4);
    u_gamma2=u_mpci(5);
    u_dt2=u_mpci(6);
    
    r_d3=u_mpci(7);
    u_gamma3=u_mpci(6);
    u_dt3=u_mpci(9);
    
    w0=w_opt;
    u_mpc(:,end+1)=u_mpci;

% Update vehicle and path 1 
    % integrate vehicle
    hg_1=sqrt(1+a1^2*omega1^2*cos(omega1*gamma1+phi1)^2);
    u_d1=0.5+u_dt1;
    input1=[u_d1*hg_1;r_d1];
    x_robot1(:,end+1)=update_vehicle(x_robot1(:,end),input1,Ts);
    % integrate path
    input_path.u=  u_gamma1;
    input_path.x = x_path1(3:4,end);
    output_path1 = RK4_integrator(@(t,x,u)path(t,x,u,path1_data), input_path);
    psi_d1=output_path1.value(1);
    gamma1=output_path1.value(2);
    
%%   circle    
%     a=10;
%     omega=0.01;
%     pd=[a*cos(omega*gamma);a*sin(omega*gamma)];
%%   sin path    
    pd1=[a1*sin(omega1*gamma1+phi1)+d1;gamma1];
    x_path1(:,end+1) =[pd1;psi_d1;gamma1]; 
    
% Update vehicle2 and path 2
    
    % integrate vehicle
    hg_2=sqrt(1+a2^2*omega2^2*cos(omega2*gamma2+phi2)^2);
    u_d2=0.5+u_dt2;
    input2=[u_d2*hg_2;r_d2];
    x_robot2(:,end+1)=update_vehicle(x_robot2(:,end),input2,Ts);
    % integrate path
    input_path.u=  u_gamma2;
    input_path.x = x_path2(3:4,end);
    output_path2 = RK4_integrator(@(t,x,u)path(t,x,u,path2_data), input_path);
    psi_d2=output_path2.value(1);
    gamma2=output_path2.value(2);
    
%%   circle    
%     a=10;
%     omega=0.01;
%     pd=[a*cos(omega*gamma);a*sin(omega*gamma)];
%%   sin path    
    pd2=[a2*sin(omega2*gamma2+phi2)+d2;gamma2];
    x_path2(:,end+1) =[pd2;psi_d2;gamma2]; 
    
% Update vehicle3 and path 3
    
    % integrate vehicle
    hg_3=sqrt(1+a3^2*omega3^2*cos(omega3*gamma3+phi3)^2);
    u_d3=0.5+u_dt3;
    input3=[u_d3*hg_3;r_d3];
    x_robot3(:,end+1)=update_vehicle(x_robot3(:,end),input3,Ts);
    % integrate path
    input_path.u = u_gamma3;
    input_path.x = x_path3(3:4,end);
    output_path3 = RK4_integrator(@(t,x,u)path(t,x,u,path3_data), input_path);
    psi_d3=output_path3.value(1);
    gamma3=output_path3.value(2);
    
%%   circle    
%     a=10;
%     omega=0.01;
%     pd=[a*cos(omega*gamma);a*sin(omega*gamma)];
%%   sin path    
    pd3=[a3*sin(omega3*gamma3+phi3)+d3;gamma3];
    x_path3(:,end+1) =[pd3;psi_d3;gamma3];     
    
end
x_robot1=x_robot1';
x_robot2=x_robot2';
x_robot3=x_robot3';

x_path1=x_path1';
x_path2=x_path2';
x_path3=x_path3';

save_to_base(1);
end

%% Dynamics of Path and Vehicle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x_next=update_vehicle(x_current,input,Ts)
    input_robot.nSteps = 4;
    input_robot.Ts=Ts;
    input_robot.u=input;
    input_robot.x = x_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_next = output_robot.value;
end
%% vehicle dynamics
function dx = vehicle(t,x,u)
psi=x(3);
ur=u(1);
r=u(2);
dx=[ur*cos(psi);ur*sin(psi);r];
end
%% path dynamics
function dx = path(t,x,u,path_data)
%% circle
% a=10;
% omega=0.01;
% h_gamma=a*omega;
% cg=1/a;
%% sin path
a=path_data(1);
omega=path_data(2);
phi=path_data(3);
gamma=x(2);
h_gamma=sqrt(1+a^2*omega^2*cos(omega*gamma+phi)^2);
cg=a*omega^2*sin(omega*gamma+phi)/(h_gamma^3);
dx=[h_gamma*cg;1]*u;
end
%% Formulate NLP
function [solver,w0,lbw,ubw,lbg,ubg,nx,nu,k3,k4]=NLP(Np,Tp,Ts,path1_data,path2_data,path3_data,Lap)

import casadi.*

s1 = SX.sym('s1');
y1 = SX.sym('y1');
psie1 = SX.sym('psie1');
gamma1 = SX.sym('gamma1');
x1 = [s1;y1;psie1;gamma1];

s2 = SX.sym('s2');
y2 = SX.sym('y2');
psie2 = SX.sym('psie2');
gamma2 = SX.sym('gamma2');
x2 = [s2;y2;psie2;gamma2];

s3 = SX.sym('s3');
y3 = SX.sym('y3');
psie3 = SX.sym('psie3');
gamma3 = SX.sym('gamma3');
x3 = [s3;y3;psie3;gamma3];
x=[x1;x2;x3];
nx=length(x);
% input of PF error system 
r1 = SX.sym('r1');
ug1=SX.sym('ug1');
r2 = SX.sym('r2');
ug2=SX.sym('ug2');
r3 = SX.sym('r3');
ug3=SX.sym('ug3');

u_dt1=SX.sym('u_dt1');
u_dt2=SX.sym('u_dt2');
u_dt3=SX.sym('u_dt3');
u=[r1;ug1;u_dt1;r2;ug2;u_dt2;r3;ug3;u_dt3];
nu=length(u);
%% PF error system
% ---------------------------------------------
% pd=[xd;yd]=[a*sin(omega*gamma); gamma];
% cg=(x'y''-y'x'')/(x'x'+y'y')^(3/2);
% 
% ---------------------------------------------
% circle
% a=10;
% omega=0.01;
% h_g=a*omega;
% h_g_max=a*omega;
% cg=1/a;
% cg_max=1/a;
% sin path
a1=path1_data(1); omega1=path1_data(2); phi1=path1_data(3);
hg1=sqrt(1+a1^2*omega1^2*cos(omega1*gamma1+phi1)^2);
hg1_max=sqrt(1+a1^2*omega1^2);
cg1=a1*omega1^2*sin(omega1*gamma1+phi1)/(hg1^3);
cg1_max=a1*omega1^2;


a2=path2_data(1); omega2=path2_data(2); phi2=path2_data(3);
hg2=sqrt(1+a2^2*omega2^2*cos(omega2*gamma2+phi2)^2);
hg2_max=sqrt(1+a2^2*omega2^2);
cg2=a2*omega2^2*sin(omega2*gamma2)/(hg2^3);
cg2_max=a2*omega2^2;


a3=path3_data(1); omega3=path3_data(2); phi3=path3_data(3);
hg3=sqrt(1+a3^2*omega3^2*cos(omega3*gamma3+phi3)^2);
hg3_max=sqrt(1+a3^2*omega3^2);
cg3=a3*omega3^2*sin(omega3*gamma3)/(hg3^3);
cg3_max=a3*omega3^2;



vd=0.5;
vd_max=1;
ds_max=1.2;
ds_min=-1.2;
% Input constraints
rmax=0.3;rmin=-0.3;
dgmax=ds_max/hg1_max;dgmin=ds_min/hg1_max;
u_dt_max=0.2; u_dt_min=-0.2;
umax=[rmax;dgmax;u_dt_max;rmax;dgmax;u_dt_max;rmax;dgmax;u_dt_max];
umin=[rmin;dgmin;u_dt_min;rmin;dgmin;u_dt_min;rmin;dgmin;u_dt_min];
% PF error dynamics equation
xdot = [(vd+u_dt1)*hg1*cos(psie1)-hg1*ug1*(1-cg1*y1);...
        (vd+u_dt1)*hg1*sin(psie1)-cg1*s1*hg1*ug1; ...
        r1-cg1*hg1*ug1;...
        ug1;...
        (vd+u_dt2)*hg2*cos(psie2)-hg2*ug2*(1-cg2*y2);...
        (vd+u_dt2)*hg2*sin(psie2)-cg2*s2*hg2*ug2;...
         r2-cg2*hg2*ug2;...
         ug2;...
         (vd+u_dt3)*hg3*cos(psie3)-hg3*ug3*(1-cg3*y3);...
         (vd+u_dt3)*hg3*sin(psie3)-cg3*s3*hg3*ug3; ...
         r3-cg3*hg3*ug3;...
         ug3];
     
%% => gains for nonlinear controller 
k1=(ds_max-vd_max);
k2=0.1*(rmax-dgmax*cg1_max*hg1_max);
k3=0.9*(rmax-dgmax*cg1_max*hg1_max)/(0.5*vd_max);

%% Objective term
Q1=diag([1 1 1 5 10]);
Q2=diag([1 1 1 5 10]);
Q3=diag([1 1 1 5 10]);
R=diag([1 1 1]);
L1 = [s1, y1, psie1, (vd+u_dt1)*hg1*cos(psie1)-hg1*ug1, r1-cg1*hg1*ug1]*Q1*[s1;y1;psie1;...
                                                                (vd+u_dt1)*hg1*cos(psie1)-hg1*ug1;...
                                                                r1-cg1*hg1*ug1];
L2 = [s2, y2, psie2, (vd+u_dt2)*hg2*cos(psie2)-hg2*ug2, r2-cg2*hg2*ug2]*Q2*[s2;y2;psie2;...
                                                                (vd+u_dt2)*hg2*cos(psie2)-hg2*ug2;...
                                                                r2-cg2*hg2*ug2];
L3 = [s3, y3, psie3, (vd+u_dt3)*hg3*cos(psie3)-hg3*ug3, r3-cg3*hg3*ug3]*Q3*[s3;y3;psie3;...
                                                                (vd+u_dt3)*hg3*cos(psie3)-hg3*ug3;...
                                                                r3-cg3*hg3*ug3];
Lcon=[u_dt1,u_dt2,u_dt3]*R*[u_dt1;u_dt2;u_dt3];
                                                            
L=L1+L2+L3+Lcon;                                                            

%% Continuous time dynamics
f = Function('f', {x, u}, {xdot, L});

%% Formulate discrete time dynamics
   % Fixed step Runge-Kutta 4 integrator
   M = 4; % RK4 steps per interval
   DT = Tp/Np/M;
   f = Function('f', {x, u}, {xdot, L});
   X0 = MX.sym('X0', nx);
   U = MX.sym('U',nu);
   X = X0;
   Q = 0;
   for j=1:M
       [m1, m1_q] = f(X, U);
       [m2, m2_q] = f(X + DT/2 * m1, U);
       [m3, m3_q] = f(X + DT/2 * m2, U);
       [m4, m4_q] = f(X + DT * m3, U);
       X=X+DT/6*(m1 +2*m2 +2*m3 +m4);
       Q = Q + DT/6*(m1_q + 2*m2_q + 2*m3_q + m4_q);
   end
   F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

%% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

%% Formulate the NLP
Xk = X0;
xmax=inf*ones(nx,1);
xmin=-xmax;
uzero=zeros(nu,1);
xzero=zeros(nx,1);

% "Lift" initial conditions
X0 = MX.sym('X0', nx);
w = {w{:}, X0};
lbw = [lbw; xzero];
ubw = [ubw; xzero];
w0 = [w0; xzero];

% Formulate the NLP
% consensus gain: k4=0.2;
k4=0.2;

Xk = X0;
eta=Lap*[Xk(4);Xk(8);Xk(12)];

V0_mpc=0.5*k3*log(1+Xk(1)^2 + Xk(2)^2) + 0.5*Xk(3)^2 + ...
       0.5*k3*log(1+Xk(5)^2 + Xk(6)^2) + 0.5*Xk(7)^2 + ...
       0.5*k3*log(1+Xk(9)^2 + Xk(10)^2)+ 0.5*Xk(11)^2 + ...
       0.5*(eta')*eta;
dV_non=-k1*k3*Xk(1)*tanh(Xk(1))/(1+Xk(1)^2+Xk(2)^2)-k2*Xk(3)*tanh(Xk(3))+...
       -k1*k3*Xk(5)*tanh(Xk(5))/(1+Xk(5)^2+Xk(6)^2)-k2*Xk(7)*tanh(Xk(7))+...
       -k1*k3*Xk(9)*tanh(Xk(9))/(1+Xk(10)^2+Xk(10)^2)-k2*Xk(11)*tanh(Xk(11))+...
       -k4*eta'*Lap*tanh(eta);   % derivative of lyapunov function
for k=0:Np-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)],nu);
    w = {w{:}, Uk};
    lbw = [lbw; umin];
    ubw = [ubw;  umax];
    w0 = [w0;  uzero]; 
    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx);
    w = {w{:}, Xk};
    lbw = [lbw; xmin];
    ubw = [ubw; xmax];
    w0 = [w0; xzero];

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; xzero];
    ubg = [ubg; xzero];
    
    % Add inequality constraint
   
    if k==0 
        V1_mpc=0.5*k3*log(1+Xk(1)^2 + Xk(2)^2) + 0.5*Xk(3)^2 + ...
               0.5*k3*log(1+Xk(5)^2 + Xk(6)^2) + 0.5*Xk(7)^2 + ...
               0.5*k3*log(1+Xk(9)^2 + Xk(10)^2)+ 0.5*Xk(11)^2 + ...
               0.5*eta'*eta;
        dV_mpc=(V1_mpc-V0_mpc)/Ts;
    end
end
%% contractive inequality constraint 
g = {g{:}, dV_mpc-dV_non};
lbg = [lbg; -inf];
ubg = [ubg; 0];
%   
%% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
solver = nlpsol('solver', 'ipopt', prob, options);
end            % for one neighbor
%% Compute PF error
function e_pf=epf(x_robot,x_path)
p=x_robot(1:2,end); psi=x_robot(3,end);
pd=x_path(1:2,end); psid=x_path(3,end); gamma=x_path(4,end);  
RI_F=[cos(psid)      sin(psid);     % From {I} to {F}
     -sin(psid)      cos(psid)]; 
e_f=RI_F*(p-pd);
s1=e_f(1);
y1=e_f(2);
psie=psi-psid; 
e_pf=[s1;y1;psie;gamma];
end
function x_path0=ini_path(path_data,gamma0)
a=path_data(1);
omega=path_data(2);
phi=path_data(3);
d=path_data(4);
pd0=[a*sin(omega*gamma0+phi)+d;gamma0];
psid0=atan2(1,a*omega*cos(omega*gamma0+phi)); 
x_path0=[pd0;psid0;gamma0];
end




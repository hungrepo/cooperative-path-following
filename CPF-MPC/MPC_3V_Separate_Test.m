
function MPCPF
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
p3_0=[-10;-15]; psi3_0=0;
x_robot3_0 = [p3_0;psi3_0];

% PATHS
% Path 1
gamma1_0=0;
a1=10;omega1=0.1;phi1=-pi;d1=20;
path1_data=[a1;omega1;phi1;d1];
x_path1_0=ini_path(path1_data,gamma1_0);
% Path 2
gamma2_0=-5;
a2=0;omega2=0.1;phi2=0;d2=0;
path2_data=[a2;omega2;phi2;d2];
x_path2_0=ini_path(path2_data,gamma2_0);  
% Path 3 
gamma3_0=-10; 
a3=10;omega3=0.1;phi3=0;d3=-20;
path3_data=[a3;omega3;phi3;d3];
x_path3_0=ini_path(path3_data,gamma3_0);  

% setup integrator for all paths
input_path.Ts = Ts;
input_path.nSteps = 4;


%% Formulate NLP for MPC
Np = 10; % number of control intervals
Tp = Ts*Np; % Time horizon
import casadi.*
[nlp_sol1,w01,lbw1,ubw1,lbg1,ubg1,nx1,nu1,k3]=NLP(Np,Tp,Ts,path1_data);
[nlp_sol2,w02,lbw2,ubw2,lbg2,ubg2,nx2,nu2,k3]=NLP(Np,Tp,Ts,path2_data);
[nlp_sol3,w03,lbw3,ubw3,lbg3,ubg3,nx3,nu3,k3]=NLP(Np,Tp,Ts,path3_data);


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
V_mpc1=[];
V_mpc2=[];
V_mpc3=[];
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

V_mpc1=[V_mpc1;0.5*k3*log(1+x0_1(1)^2 + x0_1(2)^2) + 0.5*x0_1(3)^2];
V_mpc2=[V_mpc2;0.5*k3*log(1+x0_2(1)^2 + x0_2(2)^2) + 0.5*x0_2(3)^2];
V_mpc3=[V_mpc3;0.5*k3*log(1+x0_3(1)^2 + x0_3(2)^2) + 0.5*x0_3(3)^2];
% Solve OPC for vehicle 1
    w01(1:nx1)=x0_1;
    lbw1(1:nx1)=x0_1;
    ubw1(1:nx1)=x0_1;
    sol1 = nlp_sol1('x0', w01, 'lbx', lbw1, 'ubx', ubw1,...
                  'lbg', lbg1, 'ubg', ubg1);
    w_opt1 = full(sol1.x);
    u_mpci_1=w_opt1(5:6);
    r_d1=u_mpci_1(1);
    u_gamma1=u_mpci_1(2);
    w01=w_opt1;
    u_mpc1(:,end+1)=u_mpci_1;
% Solve OPC for vehicle 2
    w02(1:nx2)=x0_2;
    lbw2(1:nx2)=x0_2;
    ubw2(1:nx2)=x0_2;
    sol2 = nlp_sol2('x0', w02, 'lbx', lbw2, 'ubx', ubw2,...
                  'lbg', lbg2, 'ubg', ubg2);
    w_opt2 = full(sol2.x);
    u_mpci_2=w_opt2(5:6);
    r_d2=u_mpci_2(1);
    u_gamma2=u_mpci_2(2);
    w02=w_opt2;
    u_mpc2(:,end+1)=u_mpci_2;
% Solve OPC for vehicle 3
    w03(1:nx3)=x0_3;
    lbw3(1:nx3)=x0_3;
    ubw3(1:nx3)=x0_3;
    sol3 = nlp_sol3('x0', w03, 'lbx', lbw3, 'ubx', ubw3,...
                  'lbg', lbg3, 'ubg', ubg3);
    w_opt3 = full(sol3.x);
    u_mpci_3=w_opt3(5:6);
    r_d3=u_mpci_3(1);
    u_gamma3=u_mpci_3(2);
    w03=w_opt3;
    u_mpc3(:,end+1)=u_mpci_3;

% Update vehicle and path 1 
    % integrate vehicle
    hg_1=sqrt(1+a1^2*omega1^2*cos(omega1*gamma1+phi1)^2);
    u_d1=0.5;
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
    u_d2=0.5;
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
    u_d3=0.5;
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

function [solver,w0,lbw,ubw,lbg,ubg,nx,nu,k3]=NLP(Np,Tp,Ts,path_data)

import casadi.*

s1 = SX.sym('s1');
y1 = SX.sym('y1');
psie = SX.sym('psie');
gamma = SX.sym('gamma');
x = [s1;y1;psie;gamma];
nx=length(x);
% input of PF error system 
r = SX.sym('r');
u_g=SX.sym('u_g');
u=[r;u_g];
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
a=path_data(1);
omega=path_data(2);
phi=path_data(3);
d=path_data(4);
h_g=sqrt(1+a^2*omega^2*cos(omega*gamma+phi)^2);
h_g_max=sqrt(1+a^2*omega^2);
cg=a*omega^2*sin(omega*gamma+phi)/(h_g^3);
cg_max=a*omega^2;

vd=0.5;
vd_max=0.5;
ds_max=1;
ds_min=-1;
% Input constraints
rmax=0.3;rmin=-0.3;
dgmax=ds_max/h_g_max;dgmin=ds_min/h_g_max;
umax=[rmax;dgmax];
umin=[rmin;dgmin];
% PF error dynamics equation
xdot = [vd*cos(psie)*h_g-h_g*u_g*(1-cg*y1);...
        vd*h_g*sin(psie)-cg*s1*h_g*u_g; ...
        r-cg*h_g*u_g;...
        u_g];

%% => gains for nonlinear controller 
k1=(ds_max-vd_max);
k2=0.1*(rmax-dgmax*cg_max*h_g_max);
k3=0.9*(rmax-dgmax*cg_max*h_g_max)/(0.5*vd_max);

%% Objective term
Q=diag([1 1 1 1 10]);
L = [s1 y1 psie vd*cos(psie)*h_g-h_g*u_g r-cg*h_g*u_g]*Q*[s1;y1;psie;vd*cos(psie)*h_g-h_g*u_g;r-cg*h_g*u_g];
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
xmax=[inf;inf;inf;inf];
xmin=[-inf;-inf;-inf;-inf];
uzero=[0;0];
xzero=[0;0;0;0];

% "Lift" initial conditions
X0 = MX.sym('X0', nx);
w = {w{:}, X0};
lbw = [lbw; xzero];
ubw = [ubw; xzero];
w0 = [w0; xzero];

% Formulate the NLP

Xk = X0;
V0_mpc=0.5*k3*log(1+Xk(1)^2 + Xk(2)^2) + 0.5*Xk(3)^2;
dV_non=-k1*k3*Xk(1)*tanh(Xk(1))/(1+Xk(1)^2+Xk(2)^2)-k2*Xk(3)*tanh(Xk(3));   % derivative of lyapunov function
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
        V1_mpc=0.5*k3*log(1+Xk(1)^2 + Xk(2)^2) + 0.5*Xk(3)^2;
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
end


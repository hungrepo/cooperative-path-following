% clear all;
% close all;
%% This code is for MPC-CPF with  
function MPCPF
clear all;
% close all;

%% Initialization
T = 200;
Ts = 0.2;
N=T/Ts;
t=[0:N].*Ts;
% VEHICLES

p1_0=[15;-15]; psi1_0=pi/4;
x_robot1_0 = [p1_0;psi1_0];
% Vehicle 2
p2_0=[35;-5]; psi2_0=pi/4+pi/8;
x_robot2_0 = [p2_0;psi2_0];
% Vehicle 3
p3_0=[48;-20]; psi3_0=pi/2;
x_robot3_0 = [p3_0;psi3_0];




% % Vehicle 1
% p1_0=[45;-25]; psi1_0=pi/4;
% x_robot1_0 = [p1_0;psi1_0];
% % Vehicle 2
% p2_0=[10;-5]; psi2_0=pi/2;
% x_robot2_0 = [p2_0;psi2_0];
% % Vehicle 3
% p3_0=[-30;-15]; psi3_0=0;
% x_robot3_0 = [p3_0;psi3_0];

% PATHS
% Path 1
gamma1_0=0;
a1=30;omega1=1/a1;phi1=0;d1=0;
path1_data=[a1;omega1;phi1;d1];
x_path1_0=ini_path(path1_data,gamma1_0);
% Path 2
gamma2_0=-.2;
a2=35;omega2=1/a2;phi2=0;d2=0;
path2_data=[a2;omega2;phi2;d2];
x_path2_0=ini_path(path2_data,gamma2_0);  
% Path 3 
gamma3_0=-.5; 
a3=40;omega3=1/a3;phi3=0;d3=0;
path3_data=[a3;omega3;phi3;d3];
x_path3_0=ini_path(path3_data,gamma3_0);  

% gamma4_0=0.4; 
% a4=40;omega4=1/a4;phi4=0;d4=0;
% path4_data=[a4;omega3;phi3;d3];
% x_path3_0=ini_path(path3_data,gamma3_0);  

% Speed profile
vd=0.02;
% setup integrator for all paths
input_path.Ts = Ts;
input_path.nSteps = 4;


%% Formulate NLP for MPC
Np = 10; % number of control intervals
Tp = Ts*Np; % Time horizon
import casadi.*
[nlp_sol1,w01,lbw1,ubw1,lbg1,ubg1,nx1,nu1,k11,k12,k13]=NLP(Np,Tp,Ts,path1_data);
[nlp_sol2,w02,lbw2,ubw2,lbg2,ubg2,nx2,nu2,k21,k22,k23]=NLP(Np,Tp,Ts,path2_data);
[nlp_sol3,w03,lbw3,ubw3,lbg3,ubg3,nx3,nu3,k31,k32,k33]=NLP(Np,Tp,Ts,path3_data);


%% Closed loop Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% For event trigger 
TR_1=[];TR_2=[];TR_3=[];
Gamma_hat11=0;
Gamma_hat12=0;
Gamma_hat22=0;
Gamma_hat21=0;
Gamma_hat23=0;
Gamma_hat33=0;
Gamma_hat32=0;
E1=[];
E2=[];
E3=[];

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
u_robot1=[];
u_robot2=[];
u_robot3=[];
uc=[];
e_pf1=[];
e_pf2=[];
e_pf3=[];

%%
v1_old=0;
v2_old=0;
v3_old=0;
Tr_Mess_1=[]; Data1_D=[];
Tr_Mess_2=[]; Data2_D=[];
Tr_Mess_3=[]; Data3_D=[];
RE_21=[];RE_12=[];RE_23=[];RE_32=[];
Re_Mess_21=[];Re_Mess_12=[];Re_Mess_23=[];Re_Mess_32=[];
Vc=[];

Delta=0;

%% Run close loop 
for i = 1:N

% Compute PF error   
e_pf1i=epf(x_robot1,x_path1);
e_pf2i=epf(x_robot2,x_path2);
e_pf3i=epf(x_robot3,x_path3);
e_pf1=[e_pf1 e_pf1i];
e_pf2=[e_pf2 e_pf2i];
e_pf3=[e_pf3 e_pf3i];
gamma1=e_pf1i(4);
gamma2=e_pf2i(4);
gamma3=e_pf3i(4);
L=[ 1  -1  0;        % network
   -1   2 -1;
    0  -1  1];
GAMMA_i=[gamma1;gamma2;gamma3];
Vc_i=GAMMA_i'*L*GAMMA_i;
Vc=[Vc Vc_i];
k=0.02;
gamma=[gamma1;gamma2;gamma3];
% Check Trigger condition
Gamma1=x_path1(4,:);
Gamma2=x_path2(4,:);
Gamma3=x_path3(4,:);
e1=Gamma_hat11(end)-Gamma1(end); 
e2=Gamma_hat22(end)-Gamma2(end); 
e3=Gamma_hat33(end)-Gamma3(end);
E1=[E1;e1];
E2=[E2;e2];
E3=[E3;e3];
eps=0.01;
%% Transmit
if abs(e1)>eps
    Gamma_hat11(end)=Gamma1(end);
    Tr_1=1;
    Tr_Mess_1=[Tr_Mess_1 [i*Ts;Gamma1(end)]];
    Data1_D=[Data1_D [i*Ts;Gamma1(end)]];
else
    Tr_1=0;
    Tr_Mess_1=[Tr_Mess_1 [0;0]];
end
if abs(e2)>eps
    Gamma_hat22(end)=Gamma2(end);
    Tr_2=1;
    Tr_Mess_2=[Tr_Mess_2 [i*Ts;Gamma2(end)]];
    Data2_D=[Data2_D [i*Ts;Gamma2(end)]];
else
    Tr_2=0;
    Tr_Mess_2=[Tr_Mess_2  [0;0]];
end
if abs(e3)>eps
    Gamma_hat33(end)=Gamma3(end);
    Tr_3=1;
    Tr_Mess_3=[Tr_Mess_3 [i*Ts;Gamma3(end)]];
    Data3_D=[Data3_D [i*Ts;Gamma3(end)]];
else
    Tr_3=0;
    Tr_Mess_3=[Tr_Mess_3  [0;0]];
end
TR_1=[TR_1 Tr_1]; TR_2=[TR_2 Tr_2]; TR_3=[TR_3 Tr_3];    

%% Recieve
t_c=i*Ts;

if isempty(Data1_D);
    idx1=[];
else
idx1=find(t_c-Delta==Data1_D(1,:));
end

if isempty(Data2_D)
    idx2=[];
else
idx2=find(t_c-Delta==Data2_D(1,:));
end
if isempty(Data3_D)
    idx3=[];
else
idx3=find(t_c-Delta==Data3_D(1,:));
end
%%
if~isempty(idx1)
   Gamma_hat21(end)=Data1_D(2,idx1)+vd*Delta;
   Re_21=1;
   Re_Mess_21=[Re_Mess_21 Data1_D(:,idx1)];
else
   Re_21=0; 
   Re_Mess_21=[Re_Mess_21 [0;0]];
end
if~isempty(idx2)
   Gamma_hat12(end)=Data2_D(2,idx2)+vd*Delta;
   Gamma_hat32(end)=Data2_D(2,idx2)+vd*Delta;
   Re_12=1; Re_32=1;
   Re_Mess_12=[Re_Mess_12 Data2_D(:,idx2)];
   Re_Mess_32=[Re_Mess_32 Data2_D(:,idx2)];
else
   Re_12=0; Re_32=0; 
   Re_Mess_12=[Re_Mess_12 [0;0]];
   Re_Mess_32=[Re_Mess_32 [0;0]];
end
if ~isempty(idx3)
   Gamma_hat23(end)=Data3_D(2,idx3)+vd*Delta;
   Re_23=1;
   Re_Mess_23=[Re_Mess_23 Data3_D(:,idx3)];
else
   Re_23=0;
   Re_Mess_23=[Re_Mess_23 [0;0]];
end
RE_21=[RE_21 Re_21];
RE_12=[RE_12 Re_12];
RE_32=[RE_32 Re_32];
RE_23=[RE_23 Re_23];

uc1=-k*tanh(Gamma1(end)-Gamma_hat12(end)); 
uc2=-k*tanh(2*Gamma2(end)-Gamma_hat21(end)-Gamma_hat23(end));
uc3=-k*tanh(Gamma3(end)-Gamma_hat32(end));
uc=[uc [uc1;uc2;uc3]];
x0_1=[e_pf1i;uc1];
x0_2=[e_pf2i;uc2];
x0_3=[e_pf3i;uc3];

V_mpc1=[V_mpc1;0.5*k13*log(1+x0_1(1)^2 + x0_1(2)^2) + 0.5*x0_1(3)^2];
V_mpc2=[V_mpc2;0.5*k23*log(1+x0_2(1)^2 + x0_2(2)^2) + 0.5*x0_2(3)^2];
V_mpc3=[V_mpc3;0.5*k33*log(1+x0_3(1)^2 + x0_3(2)^2) + 0.5*x0_3(3)^2];
%% CPF MPC----------------------------------
% 
% % Solve OPC for vehicle 1
%     w01(1:nx1)=x0_1;
%     lbw1(1:nx1)=x0_1;
%     ubw1(1:nx1)=x0_1;
%     sol1 = nlp_sol1('x0', w01, 'lbx', lbw1, 'ubx', ubw1,...
%                   'lbg', lbg1, 'ubg', ubg1);
%     w_opt1 = full(sol1.x);
%     u_mpci_1=w_opt1(6:7);
%     r_d1=u_mpci_1(1);
%     u_gamma1=u_mpci_1(2);
%     w01=w_opt1;
%     u_mpc1(:,end+1)=u_mpci_1;
% % Solve OPC for vehicle 2
%     w02(1:nx2)=x0_2;
%     lbw2(1:nx2)=x0_2;
%     ubw2(1:nx2)=x0_2;
%     sol2 = nlp_sol2('x0', w02, 'lbx', lbw2, 'ubx', ubw2,...
%                   'lbg', lbg2, 'ubg', ubg2);
%     w_opt2 = full(sol2.x);
%     u_mpci_2=w_opt2(6:7);
%     r_d2=u_mpci_2(1);
%     u_gamma2=u_mpci_2(2);
%     w02=w_opt2;
%     u_mpc2(:,end+1)=u_mpci_2;
% % Solve OPC for vehicle 3
%     w03(1:nx3)=x0_3;
%     lbw3(1:nx3)=x0_3;
%     ubw3(1:nx3)=x0_3;
%     sol3 = nlp_sol3('x0', w03, 'lbx', lbw3, 'ubx', ubw3,...
%                   'lbg', lbg3, 'ubg', ubg3);
%     w_opt3 = full(sol3.x);
%     u_mpci_3=w_opt3(6:7);
%     r_d3=u_mpci_3(1);
%     u_gamma3=u_mpci_3(2);
%     w03=w_opt3;
%     u_mpc3(:,end+1)=u_mpci_3;
%% -------End---------------------------

%% Vehicle speed
% Vehicle 1 
    hg_1=a1;
    u_d1=hg_1*(vd+uc1);
% Vehicle 2
    hg_2=a2;
    u_d2=hg_2*(vd+uc2);
% Vehicle 3
    hg_3=a3;
    u_d3=hg_3*(vd+uc3);
%% 

%% CPF Lyapunov Based
[r_d1, u_gamma1]=u_L(x0_1,k11,k12,k13,u_d1,hg_1,v1_old);
[r_d2, u_gamma2]=u_L(x0_2,k21,k22,k23,u_d2,hg_2,v2_old);
[r_d3, u_gamma3]=u_L(x0_3,k31,k32,k33,u_d3,hg_3,v3_old);
v1_old=u_gamma1;
v2_old=u_gamma2;
v3_old=u_gamma3;
%% End --------------------------    
    
% Update vehicle and path 1 
    % integrate vehicle
   
    input1=[u_d1;r_d1];
    u_robot1=[u_robot1 input1];
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
    pd1=[a1*cos(gamma1+phi1)+d1;a1*sin(gamma1+phi1)+d1];
    x_path1(:,end+1) =[pd1;psi_d1;gamma1]; 
    
% Update vehicle2 and path 2
    
    % integrate vehicle
    input2=[u_d2;r_d2];
    u_robot2=[u_robot2 input2];
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
    pd2=[a2*cos(gamma2+phi2)+d2;a2*sin(gamma2+phi2)+d2];
    x_path2(:,end+1) =[pd2;psi_d2;gamma2]; 
    
% Update vehicle3 and path 3
    
    % integrate vehicle
    input3=[u_d3;r_d3];
    u_robot3=[u_robot3 input3];
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
    pd3=[a3*cos(gamma3+phi3)+d3;a3*sin(gamma3+phi3)+d3];
    x_path3(:,end+1) =[pd3;psi_d3;gamma3];     
    
    
    
   Gamma_hat11(:,end+1)=update_gammahat(Gamma_hat11(end),0,Ts,t);
   Gamma_hat12(:,end+1)=update_gammahat(Gamma_hat12(end),0,Ts,t);
   Gamma_hat22(:,end+1)=update_gammahat(Gamma_hat22(end),0,Ts,t);
   Gamma_hat21(:,end+1)=update_gammahat(Gamma_hat21(end),0,Ts,t);
   Gamma_hat23(:,end+1)=update_gammahat(Gamma_hat23(end),0,Ts,t);
   Gamma_hat33(:,end+1)=update_gammahat(Gamma_hat33(end),0,Ts,t);
   Gamma_hat32(:,end+1)=update_gammahat(Gamma_hat32(end),0,Ts,t);
    
    
    
end
x_robot1=x_robot1';
x_robot2=x_robot2';
x_robot3=x_robot3';

x_path1=x_path1';
x_path2=x_path2';
x_path3=x_path3';

u_robot1=u_robot1';
u_robot2=u_robot2';
u_robot3=u_robot3';

e_pf1=e_pf1';
e_pf2=e_pf2';
e_pf3=e_pf3';

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
h_gamma=a;
cg=1/a;
dx=[h_gamma*cg;1]*u;
end
%% Formulate NLP
function [solver,w0,lbw,ubw,lbg,ubg,nx,nu,k1,k2,k3]=NLP(Np,Tp,Ts,path_data)

import casadi.*

s1 = SX.sym('s1');
y1 = SX.sym('y1');
psie = SX.sym('psie');
gamma = SX.sym('gamma');
uc=SX.sym('uc');
x = [s1;y1;psie;gamma;uc];
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

a=path_data(1);omega=path_data(2);phi=path_data(3);
hg_i=a;
hg_max=a;
cg_i=1/a;
cg_max=1/a;

vd=0.02;
vd_max=vd;
% ds_max=0.85;
% ds_min=-0.85;
% Input constraints
rmax=0.2;rmin=-0.2;
% dgmax=ds_max/hg_max;dgmin=ds_min/hg_max;
dgmax=0.05;dgmin=-0.05;
umax=[rmax;dgmax];
umin=[rmin;dgmin];
% PF error dynamics equation
xdot = [(vd+uc)*hg_i*cos(psie)-hg_i*u_g*(1-cg_i*y1);...
        (vd+uc)*hg_i*sin(psie)-cg_i*s1*hg_i*u_g; ...
        r-cg_i*hg_i*u_g;...
        u_g;...
        0];

%% => gains for nonlinear controller 
% k1=1;
 k2=0.15*0.4;
 k3=0.15*0.6;
% k1=hg_i*(dgmax-vd_max);
k1=hg_i*0.01;

% k2=0.2*(rmax-dgmax*cg_max*hg_max);
% k3=0.8*(rmax-dgmax*cg_max*hg_max)/(0.5*vd_max*hg_i);

%% Objective term
Q=diag([1 1 2 2 10]);
L = [s1, y1, psie, (vd+uc)*cos(psie)-u_g, r-cg_i*hg_i*u_g]*Q*[s1;y1;psie;...
                                                                (vd+uc)*cos(psie)-u_g;...
                                                                r-cg_i*hg_i*u_g];
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
end            % for two neighbors 
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
pd0=[a*cos(gamma0+phi)+d;a*sin(gamma0+phi)+d];
psid0=atan2(cos(gamma0+phi),-sin(gamma0+phi)); 
x_path0=[pd0;psid0;gamma0];
end

function x_next=update_gammahat(x_current,input,Ts,t)
    input_gammahat.nSteps = 4;
    input_gammahat.Ts=Ts;
    input_gammahat.u=input;
    input_gammahat.x = x_current;
    output_gamma= RK4_integrator(@gammahatdot, input_gammahat);
    x_next = output_gamma.value;
end
function dx = gammahatdot(t,x,u)
    dx=0.02;
end
function [r_d,v]=u_L(x0,k1,k2,k3,u,hg,v_old)
ex=x0(1);
ey=x0(2);
e_phi=x0(3);
v   = (u*cos(e_phi)+k1*tanh(ex))/hg;
if e_phi==0
r_d =-k3*ey*u/(1+ex^2+ey^2)+v_old;
else
r_d =-k3*ey*u*sin(e_phi)/((1+ex^2+ey^2)*e_phi)-k2*tanh(e_phi)+v_old;
end
end
% clear all;
% close all;
function MPCPF
clear all;
close all;

%% Initialization
T = 200;
Ts = 0.1;
N=T/Ts;
t=[0:N].*Ts;
% Vehicle 1
p1_0=[-10;10]; psi1_0=0;
x_robot1_0 = [p1_0;psi1_0];
input_robot1.Ts = Ts;
input_robot1.nSteps = 4;
% Path 1
gamma1_0=0;psid1_0=pi/4;pd1_0=[0;0]; 
x_path1_0=[pd1_0;psid1_0;gamma1_0];
input_path1.Ts = Ts;
input_path1.nSteps = 4;

% Vehicle 2
p2_0=[20;-10]; psi2_0=0;
x_robot2_0 = [p2_0;psi2_0];
input_robot2.Ts = Ts;
input_robot2.nSteps = 4;
% Path 2
a=10;
omega=0.1;
gamma2_0=-10;
pd2_0=[a*sin(omega*gamma2_0)+10;gamma2_0];
psid2_0=atan2(1,a*omega*cos(omega*gamma2_0));
x_path2_0=[pd2_0;psid2_0;gamma2_0];
input_path2.Ts = Ts;
input_path2.nSteps = 4;
%% Formulate NLP for MPC
Np = 10; % number of control intervals
Tp = Ts*Np; % Time horizon
import casadi.*
[nlp_sol1,w01,lbw1,ubw1,lbg1,ubg1,nx1,nu1,k3]=NLP_gamma(Np,Tp,Ts);
[nlp_sol2,w02,lbw2,ubw2,lbg2,ubg2,nx2,nu2,k3]=NLP_gamma(Np,Tp,Ts);

%% Closed loop Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
x_robot1=x_robot1_0;
x_path1=x_path1_0;
V_mpc=[];
u_mpc1=[];
u_mpc2=[];
x_robot2=x_robot2_0;
x_path2=x_path2_0;
V_mpc1=[];
V_mpc2=[];

for i = 1:N

% create x0 for vehicle 1    
p1=x_robot1(1:2,end); psi1=x_robot1(3,end);
pd1=x_path1(1:2,end); psid1=x_path1(3,end); gamma1=x_path1(4,end);  
RI_F1=[cos(psid1)      sin(psid1);     % From {I} to {F}
     -sin(psid1)      cos(psid1)]; 
e_f1=RI_F1*(p1-pd1);
s1_1=e_f1(1);
y1_1=e_f1(2);
psie1=psi1-psid1; 
% create x0 for vehicle 2

p2=x_robot2(1:2,end); psi2=x_robot2(3,end);
pd2=x_path2(1:2,end); psid2=x_path2(3,end); gamma2=x_path2(4,end);  
RI_F2=[cos(psid2)      sin(psid2);     % From {I} to {F}
     -sin(psid2)      cos(psid2)]; 
e_f2=RI_F2*(p2-pd2);
s1_2=e_f2(1);
y1_2=e_f2(2);
psie2=psi2-psid2; 

x0_1=[s1_1;y1_1;psie1;gamma1;gamma2];
x0_2=[s1_2;y1_2;psie2;gamma2;gamma1];

V_mpc1=[V_mpc1;0.5*k3*log(1+x0_1(1)^2 + x0_1(2)^2) + 0.5*x0_1(3)^2];
V_mpc2=[V_mpc2;0.5*k3*log(1+x0_2(1)^2 + x0_2(2)^2) + 0.5*x0_2(3)^2];
% Solve OPC for vehicle 1
    w01(1:nx1)=x0_1;
    lbw1(1:nx1)=x0_1;
    ubw1(1:nx1)=x0_1;
    sol1 = nlp_sol1('x0', w01, 'lbx', lbw1, 'ubx', ubw1,...
                  'lbg', lbg1, 'ubg', ubg1);
    w_opt1 = full(sol1.x);
    u_mpci_1=w_opt1(6:8);
    r_d1=u_mpci_1(1);
    u_gamma1=u_mpci_1(2);
    u_dt1=u_mpci_1(3);
    w01=w_opt1;
    u_mpc1(:,end+1)=u_mpci_1;
% Solve OPC for vehicle 2
    w02(1:nx2)=x0_2;
    lbw2(1:nx2)=x0_2;
    ubw2(1:nx2)=x0_2;
    sol2 = nlp_sol2('x0', w02, 'lbx', lbw2, 'ubx', ubw2,...
                  'lbg', lbg2, 'ubg', ubg2);
    w_opt2 = full(sol2.x);
    u_mpci_2=w_opt2(6:8);
    r_d2=u_mpci_2(1);
    u_gamma2=u_mpci_2(2);
    u_dt2=u_mpci_2(3);
    w02=w_opt2;
    u_mpc2(:,end+1)=u_mpci_2;
        
 % Update vehicle and path 1 
    % integrate vehicle
    a=10;
    omega=0.1;
    hg_1=sqrt(1+a^2*omega^2*cos(omega*gamma1)^2);
    u_d1=0.5+u_dt1;
    input_robot1.u=[u_d1*hg_1;r_d1];
    input_robot1.x = x_robot1(:,end);
    output_robot1 = RK4_integrator(@vehicle, input_robot1);
    x_robot1(:,end+1) = output_robot1.value;
    % integrate path
    input_path1.u=u_gamma1;
    input_path1.x = x_path1(3:4,end);
    output_path1 = RK4_integrator(@path, input_path1);
    psi_d1=output_path1.value(1);
    gamma1=output_path1.value(2);
    
%%   circle    
%     a=10;
%     omega=0.01;
%     pd=[a*cos(omega*gamma);a*sin(omega*gamma)];
%%   sin path    
   
    pd1=[a*sin(omega*gamma1);gamma1];
    x_path1(:,end+1) =[pd1;psi_d1;gamma1]; 
    
 % Update vehicle and path 2
    
    % integrate vehicle
    hg_2=sqrt(1+a^2*omega^2*cos(omega*gamma2)^2);
    u_d2=0.5+u_dt2;
    input_robot2.u=[u_d2*hg_2;r_d2];
    input_robot2.x = x_robot2(:,end);
    output_robot2 = RK4_integrator(@vehicle, input_robot2);
    x_robot2(:,end+1) = output_robot2.value;
    % integrate path
    input_path2.u=u_gamma2;
    input_path2.x = x_path2(3:4,end);
    output_path2 = RK4_integrator(@path, input_path2);
    psi_d2=output_path2.value(1);
    gamma2=output_path2.value(2);
    
%%   circle    
%     a=10;
%     omega=0.01;
%     pd=[a*cos(omega*gamma);a*sin(omega*gamma)];
%%   sin path    
    a=10;
    omega=0.1;
    pd2=[a*sin(omega*gamma2)+10;gamma2];
    x_path2(:,end+1) =[pd2;psi_d2;gamma2]; 
    
    
end
x_robot1=x_robot1';
x_robot2=x_robot2';
x_path1=x_path1';
x_path2=x_path2';

save_to_base(1);
end

%% Dynamics of Path and Vehicle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% vehicle dynamics
function dx = vehicle(t,x,u)
psi=x(3);
ur=u(1);
r=u(2);
dx=[ur*cos(psi);ur*sin(psi);r];
end
%% path dynamics
function dx = path(t,x,u)
%% circle
% a=10;
% omega=0.01;
% h_gamma=a*omega;
% cg=1/a;
%% sin path
a=10;
omega=0.1;
gamma=x(2);
h_gamma=sqrt(1+a^2*omega^2*cos(omega*gamma)^2);
cg=a*omega^2*sin(omega*gamma)/(h_gamma^3);
dx=[h_gamma*cg;1]*u;
end

function [solver,w0,lbw,ubw,lbg,ubg,nx,nu,k3]=NLP_gamma(Np,Tp,Ts)

import casadi.*

s1 = SX.sym('s1');
y1 = SX.sym('y1');
psie = SX.sym('psie');
gamma_i = SX.sym('gamma_i');
gamma_j=SX.sym('gamma_j');
x = [s1;y1;psie;gamma_i;gamma_j];
nx=length(x);
% input of PF error system 
r = SX.sym('r');
u_g=SX.sym('u_g');
u_dt=SX.sym('u_dt');
u=[r;u_g;u_dt];
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
a=10;
omega=0.1;
hg_i=sqrt(1+a^2*omega^2*cos(omega*gamma_i)^2);
hg_j=sqrt(1+a^2*omega^2*cos(omega*gamma_j)^2);
hg_max=sqrt(1+a^2*omega^2);
cg_i=a*omega^2*sin(omega*gamma_i)/(hg_i^3);
cg_max=a*omega^2;

vd=0.5;
vd_max=0.7;
ds_max=1;
ds_min=-1;
% Input constraints
rmax=0.3;rmin=-0.3;
dgmax=ds_max/hg_max;dgmin=ds_min/hg_max;
u_dt_max=0.2; u_dt_min=-0.2;
umax=[rmax;dgmax;u_dt_max];
umin=[rmin;dgmin;u_dt_min];
% PF error dynamics equation
xdot = [(vd+u_dt)*hg_i*cos(psie)-hg_i*u_g*(1-cg_i*y1);...
        (vd+u_dt)*hg_i*sin(psie)-cg_i*s1*hg_i*u_g; ...
        r-cg_i*hg_i*u_g;...
        u_g;...
        vd];

%% => gains for nonlinear controller 
k1=(ds_max-vd_max);
k2=0.1*(rmax-dgmax*cg_max*hg_max);
k3=0.9*(rmax-dgmax*cg_max*hg_max)/(0.5*vd_max);

%% Objective term
Q=diag([1 1 1 1 10]);
R=diag([1 10]);
L = [s1, y1, psie, (vd+u_dt)*hg_i*cos(psie)-hg_i*u_g, r-cg_i*hg_i*u_g]*Q*[s1;y1;psie;...
                                                                (vd+u_dt)*hg_i*cos(psie)-hg_i*u_g;...
                                                                r-cg_i*hg_i*u_g] + ...
    [gamma_i-gamma_j, u_dt]*R*[gamma_i-gamma_j; u_dt];
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
end
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
% Vehicle
p0=[-4;20]; psi0=0;
x_robot0 = [p0;psi0];
input_robot.Ts = Ts;
input_robot.nSteps = 4;
% Path
gamma0=0;psid0=pi/4;pd0=[10;0]; 
x_path0=[pd0;psid0;gamma0];
input_path.Ts = Ts;
input_path.nSteps = 4;
%% Formulate NLP for MPC
Np = 10; % number of control intervals
Tp = Ts*Np; % Time horizon
import casadi.*
[nlp_sol,w0,lbw,ubw,lbg,ubg,nx,nu,k3]=NLP_gamma(Np,Tp,Ts);
%% Closed loop Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
x_robot=x_robot0;
x_path=x_path0;
V_mpc=[];
u_mpc=[];
u_robot=[];
for i = 1:N
p=x_robot(1:2,end); psi=x_robot(3,end);
pd=x_path(1:2,end); psid=x_path(3,end); gamma=x_path(4,end);  
RI_F=[cos(psid)      sin(psid);     % From {I} to {F}
     -sin(psid)      cos(psid)]; 
e_f=RI_F*(p-pd);
s1=e_f(1);
y1=e_f(2);
psie=psi-psid; 
x0=[s1;y1;psie;gamma];
V_mpc=[V_mpc;0.5*k3*log(1+x0(1)^2 + x0(2)^2) + 0.5*x0(3)^2];
% Solve OPC
    w0(1:nx)=x0;
    lbw(1:nx)=x0;
    ubw(1:nx)=x0;
    sol = nlp_sol('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                  'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    u_mpci=w_opt(5:6);
    r_d=u_mpci(1);
    u_gamma=u_mpci(2);
    w0=w_opt;
    u_mpc(:,end+1)=u_mpci;
 % Update vehicle and path   
    
    % integrate vehicle
    a=10;
    omega=0.1;
    h_g=sqrt(1+a^2*omega^2*cos(omega*gamma)^2);
    u_d=0.5*h_g;
    input_robot.u=[u_d;r_d];
    u_robot=[u_robot input_robot.u];
    input_robot.x = x_robot(:,end);
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_robot(:,end+1) = output_robot.value;
    % integrate path
    input_path.u=u_gamma;
    input_path.x = x_path(3:4,end);
    output_path = RK4_integrator(@path, input_path);
    psi_d=output_path.value(1);
    gamma=output_path.value(2);
    
%%   circle    
%     a=10;
%     omega=0.01;
%     pd=[a*cos(omega*gamma);a*sin(omega*gamma)];
%%   sin path    
    a=10;
    omega=0.1;
    pd=[a*sin(omega*gamma)+10;gamma];
    x_path(:,end+1) =[pd;psi_d;gamma]; 
end
save_to_base(1);
end

%% Dynamics of Path and Vehicle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x_path0=ini_path(path_data,gamma0)
a=path_data(1);
omega=path_data(2);
phi=path_data(3);
d=path_data(4);
pd0=[a*sin(omega*gamma0+phi)+d;gamma0];
psid0=atan2(1,a*omega*cos(omega*gamma0+phi)); 
x_path0=[pd0;psid0;gamma0];
end

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
a=10;
omega=0.1;
h_g=sqrt(1+a^2*omega^2*cos(omega*gamma)^2);
h_g_max=sqrt(1+a^2*omega^2);
cg=a*omega^2*sin(omega*gamma)/(h_g^3);
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
k2=.1*(rmax-dgmax*cg_max*h_g_max);
k3=.9*(rmax-dgmax*cg_max*h_g_max)/(0.5*vd_max);

%% Objective term
Q=diag([.1 .1 1 1 10]);
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
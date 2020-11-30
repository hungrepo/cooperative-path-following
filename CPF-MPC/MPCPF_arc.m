% clear all;
% close all;
function MPCPF
clear all;
close all;

%% Initialization
T = 100;
Ts = 0.1;
N=T/Ts;
t=[0:N].*Ts;
% Vehicle
p0=[3;0]; psi0=0;
x_robot0 = [p0;psi0];
input_robot.Ts = Ts;
input_robot.nSteps = 4;
% Path
s0=0;psid0=0;pd0=[0;0]; 
x_path0=[pd0;psid0;s0];
input_path.Ts = Ts;
input_path.nSteps = 4;
%% Formulate NLP for MPC
Np = 10; % number of control intervals
Tp = Ts*Np; % Time horizon
import casadi.*
[nlp_sol,w0,lbw,ubw,lbg,ubg,nx,nu]=NLP_arc(Np,Tp,Ts);
%% Closed loop Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
x_robot=x_robot0;
x_path=x_path0;
u_mpc=[0;0];
for i = 1:N
p=x_robot(1:2,end); psi=x_robot(3,end);
pd=x_path(1:2,end); psid=x_path(3,end); s=x_path(4,end);  
RI_F=[cos(psid)       sin(psid);     % From {I} to {F}
      -sin(psid)       cos(psid)]; 
e_f=RI_F*(p-pd);
s1=e_f(1);
y1=e_f(2);
psie=psi-psid; 
x0=[s1;y1;psie;s];

% Solve OPC
    w0(1:nx)=x0;
    lbw(1:nx)=x0;
    ubw(1:nx)=x0;
    sol = nlp_sol('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                  'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    u_mpci=w_opt(5:6);
    r_d=u_mpci(1);
    u_s=u_mpci(2);
    w0=w_opt;
    u_mpc(:,end+1)=u_mpci;
 % Update vehicle and path   
    u_d=0.6+0.2*cos(0.1*s);
    input_robot.u=[u_d;r_d];
    input_path.u=u_s;
    % integrate vehicle
    input_robot.x = x_robot(:,end);
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_robot(:,end+1) = output_robot.value;
    % integrate path
    input_path.x = x_path(:,end);
    output_path = RK4_integrator(@path, input_path);
    x_path(:,end+1) = output_path.value; 
end
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
s=x(4);
% cur=0.1;
cur=0.1*sin(0.1*s);
psi_d=x(3);
dx=[cos(psi_d);sin(psi_d);cur;s]*u;
end


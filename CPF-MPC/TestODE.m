% clear all;
% close all;
function TestODE
clear all;
close all;

%% Initialization
T = 100;
Ts = 0.1;
N=T/Ts;t=[0:N].*Ts;
% Vehicle
p0=[0;0]; psi0=0;
x_robot0 = [p0;psi0];
ur = 1;
r=0.1;
u=[ur;r];
input_robot.Ts = Ts;
input_robot.nSteps = 4;
input_robot.u = u;
% Path
s0=0;psid0=0;pd0=[0;0]; 
x_path0=[s0;psid0;pd0];
u_s=1;
input_path.Ts = Ts;
input_path.nSteps = 4;
input_path.u = u_s;
%% Simulation
x_robot=x_robot0;
x_path=x_path0;
for i = 1:N
    % integrate vehicle
    input_robot.x = x_robot(:,end);
    output_robot = RK4_integrator( @vehicle, input_robot);
    x_robot(:,end+1) = output_robot.value;
    % integrate path
    input_path.x = x_path(:,end);
    output_path = RK4_integrator( @path, input_path);
    x_path(:,end+1) = output_path.value;
    
end
save_to_base(1);
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
s=x(1);
cur=0.1;
psi_d=x(2);
dx=[1;cur;cos(psi_d);sin(psi_d)]*u;
end


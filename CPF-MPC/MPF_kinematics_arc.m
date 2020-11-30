function z=MPC_PF(u)
p=u(1:2);
psi=u(3);
t=u(4);
%% Declare variables
persistent Ts;
persistent output;
persistent u_s;
persistent w0 lbw ubw lbg ubg nlp_sol
persistent k1 k2 k3
persistent nx
persistent V_mpc;
persistent x_path
persistent input_path
if t==0
    Ts = 0.1;
    % Path
    s0=0;psid0=0;pd0=[0;0]; 
    x_path=[s0;psid0;pd0];
    input_path.Ts = Ts;
    input_path.nSteps = 4;
    u_s=0;
    % NLP for MPC
    Np=10;
    Tp=Ts*Np;
end
%% Setup for NMPC - just in the first call
if t==0
import casadi.*
 [nlp_sol,w0,lbw,ubw,lbg,ubg,nx,nu,k1,k2,k3]=NLP(Np,Tp,Ts);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                          %% Closed Loop %%
%% Update path
input_path.u = u_s;
input_path.x = x_path;
output_path = RK4_integrator(@path, input_path);
x_path = output_path.value;
gamma=x_path(1);
psi_d=x_path(2);
pd=x_path(3:4);
%% Compute error between the path and the vehicle
R1=[cos(psi_d)       sin(psi_d);     % From {I} to {F}
   -sin(psi_d)       cos(psi_d)]; 
   e=p-pd;
   e_f=R1*e;
   s1=e_f(1);
   y1=e_f(2);

   psie=psi-psi_d;
   x0=[s1;y1;psie;gamma];   

   V=0.5*k3*log(1+x0(1)^2 + x0(2)^2) + 0.5*x0(3)^2;

%% MPCPF controller 
    vd=0.5;
%    Solve NMPC OCP
    w0(1:4)=x0;
    lbw(1:nx)=x0;
    ubw(1:nx)=x0;
    sol = nlp_sol('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    u_mpc=w_opt(5:6);
    r_d=u_mpc(1);
    u_s=u_mpc(2);
    w0=w_opt;
%     
%% Nonlinear PF controller
%     cs=0.1;
%     vd=0.5;
%     if x0(3)==0
%         r_d=-k3*x0(2)*vd/(1+x0(1)^2+x0(2)^2)+cs*u_s;
%     else
%         r_d=-k3*x0(2)*vd*sin(x0(3))/(x0(3)*(1+x0(1)^2+x0(2)^2))-k2*tanh(x0(3))+cs*u_s;
%     end    
% %      
%     u_s=vd*cos(x0(3))+k1*tanh(x0(1));
%%   Send data to vehicle     
     z=[vd;r_d;V;x_path];

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ output ] = RK4_integrator(ode_fun, input )
    x0 = input.x;
    u0 = input.u;
    Ts = input.Ts;
    nSteps = input.nSteps;
    
    nx = length(x0);
    nu = length(u0);
    h = Ts/nSteps;
    STEP = 1e-100;
    
    compute_sensitivities = ~isfield(input,'sens') || input.sens;
    
    xEnd = x0;
    A = eye(nx);
    B = zeros(nx,nu);
    for i = 1:nSteps
        x0 = xEnd;
        xEnd = rk4_step(ode_fun,x0,u0,h);
        if compute_sensitivities
            sensX = zeros(nx,nx); sensU = zeros(nx,nu);
            for j = 1:nx
                % imaginary trick for states
                xTemp1 = x0; xTemp1(j) = xTemp1(j) + STEP*sqrt(-1);
                xTemp1 = rk4_step(ode_fun,xTemp1,u0,h);
                
                sensX(:,j) = imag(xTemp1)./STEP;
            end
            for j = 1:nu
                % imaginary trick for controls
                uTemp1 = u0; uTemp1(j) = uTemp1(j) + STEP*sqrt(-1);
                xTemp1 = rk4_step(ode_fun,x0,uTemp1,h);
                
                sensU(:,j) = imag(xTemp1)./STEP;
            end
            % propagate sensitivities
            A = sensX*A;
            B = sensX*B + sensU;
        end
    end
    output.value = xEnd;
    if compute_sensitivities
        output.sensX = A;
        output.sensU = B;
    end
end
function x_next = rk4_step(ode_fun,x,u,h)
    k1 = ode_fun(0,x,u);
    k2 = ode_fun(0,x+h/2.*k1,u);
    k3 = ode_fun(0,x+h/2.*k2,u);
    k4 = ode_fun(0,x+h.*k3,u);
    x_next = x + h/6.*(k1+2*k2+2*k3+k4);
end
function   [nlp_sol,w0,lbw,ubw,lbg,ubg,nx,nu,k1,k2,k3]=NLP(Np,Tp,Ts)

% state of PF error system
import casadi.*

s1 = SX.sym('s1');
y1 = SX.sym('y1');
psie = SX.sym('psie');
s = SX.sym('s');
x = [s1;y1;psie;s];
nx=length(x);
% input of PF error system 
r = SX.sym('r');
u_s=SX.sym('u_s');
u=[r;u_s];
nu=length(u);
%% PF error system
% cs=0.1*sin(0.1*s);
cs=0.1;
cs_max=0.1;
% vd=0.6+0.2*cos(0.1*s);
vd=0.5;
vd_max=0.5;
% Input constraints
rmax=0.2;rmin=-0.2;
dsmax=1;dsmin=-1;
umax=[rmax;dsmax];
umin=[rmin;dsmin];
% PF error dynamics equation
xdot = [vd*cos(psie)-u_s*(1-cs*y1);...
        vd*sin(psie)-cs*s1*u_s; ...
        r-cs*u_s;...
        u_s];

%% => gains for nonlinear controller 
k1=(dsmax-vd_max);
k2=0.6*(rmax-dsmax*cs_max);
k3=0.4*(rmax-dsmax*cs_max)/(0.5*vd_max);

%% Objective term
Q=diag([1 1 1 1 10]);
L = [s1 y1 psie vd*cos(psie)-u_s r-cs*u_s]*Q*[s1;y1;psie;vd*cos(psie)-u_s;r-cs*u_s];
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
nlp_sol = nlpsol('solver', 'ipopt', prob, options);
 
end
function dx = path(t,x,u)
s=x(1);
cur=0.1;
psi_d=x(2);
dx=[1;cur;cos(psi_d);sin(psi_d)]*u;
end


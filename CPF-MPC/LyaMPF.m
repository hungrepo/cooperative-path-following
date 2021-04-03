function z=MPC_PF(u)
Ts=0.2;
p=u(1:2);
yaw=u(3)*pi/180;
r_meas=u(4);
t=u(5);
ur=u(6);
U=u(7);
%% Declare variables
persistent output;
persistent er_old Sat_D_Mode_old Diff_Mode_old
persistent yaw_t_old yaw_old
persistent u_g;
persistent p_hat p_hat_old Vcxy_hat_old Vcxy_hat p_old ur_old
persistent phi_e_old phi_e_t_old
persistent eta_old;

persistent w0 lbw ubw lbg ubg solver
persistent k1 k2 k3
persistent nx

persistent V_mpc;
if t==0
    u_g=0;
    yaw_old=yaw;
    yaw_t_old=yaw;
    
    %% For yawrate controller 
    er_old=0;
    Sat_D_Mode_old=0;
    Diff_Mode_old=0;
    
    %% For current estimation
    Vcxy_hat=[0;0];
    Vcxy_hat_old=[0;0];
    p_hat=p;
    p_hat_old=p;
    ur_old=ur;
    p_old=p;
    
    phi_e_old=yaw;
    phi_e_t_old=yaw;
    
    eta_old=1;
    V_mpc=[];
end

%% Current estimation
    R2=[cos(yaw)        -sin(yaw);     % From {B} to {I}
        sin(yaw)         cos(yaw)]; 
    p_t=p_old-p_hat;
    Vr=[ur_old;0];
    Kp=1*[1 0;
           0 1];
    Kc=1*[1 0;
          0 1];
    p_hat_dot=R2*Vr+Vcxy_hat+Kp*p_t;
    p_hat=p_hat_old+Ts*p_hat_dot;
    p_hat_old=p_hat;
    Vcxy_hat_dot=Kc*p_t;
    Vcxy_hat=Vcxy_hat_old+Ts*Vcxy_hat_dot;
    Vcxy_hat_old=Vcxy_hat;
    p_old=p;
    ur_old=ur;
%% Update path
[pd,theta_c,gamma]=path(t,u_g,Ts);
%% Compute error between the path and the vehicle
R1=[cos(theta_c)       sin(theta_c);     % From {I} to {F}
   -sin(theta_c)       cos(theta_c)]; 
   e=p-pd;
   e_f=R1*e;
   s1=e_f(1);
   y1=e_f(2);
%    yaw_t=convert(yaw,yaw_old,yaw_t_old);
%    yaw_old=yaw;
%    yaw_t_old=yaw_t;

 if t<=5
        phi_e=yaw;
    else
        phi_e=atan2(ur*sin(yaw)+Vcxy_hat(2),ur*cos(yaw)+Vcxy_hat(1));
 end
   phi_e_t=convert(phi_e,phi_e_old,phi_e_t_old);
   phi_e_old=phi_e;
   phi_e_t_old=phi_e_t;
 
%    theta=yaw_t-theta_c; 
   theta=phi_e_t-theta_c;
   x0=[s1; y1; theta; gamma];   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup for NMPC - just in the first call
if t==0
    import casadi.*
    Ts=0.2;  % sampling time
    N = 10; % number of control intervals
    T = Ts*N; % Time horizon
    %% Declare model variables
    % State
    x1 = SX.sym('x1');
    x2 = SX.sym('x1');
    x3=SX.sym('x3');
    x4=SX.sym('x4');
    x = [x1;x2;x3;x4];
    nx=length(x);
    % Input
    u1=SX.sym('u1');
    u2=SX.sym('u2');
    u=[u1;u2];
    nu=length(u);

    %% Path need to follow
    
% R1=10;
% R2=7;
% c1=1/R1;
% c2=1/R2;
% n1=1;
% n2=1;
% g1=50;
% g2=g1+R1*pi;
% g3=g2+g1;
% g4=g3+R2*pi;
% g5=g4+g1;
% cs=c1/(1+(exp(-n1*(gamma-g1))))-c1/(1+(exp(-n2*(gamma-g2))))-c2/(1+(exp(-n2*(gamma-g3))))+c2/(1+(exp(-n2*(gamma-g4))));

    
%     
    cs=0.1*sin(.1*x4);
    cs_max=0.1;
%      
    
    
%     cs=0.1; 
%     cs_max=1/0.7;
    vp=0.8+0.2*cos(0.1*x4);
%     vp=0.5;
    vp_max=1;

%     R=10;
%     cs=1/R;
%     cs_max=0.1;
%     vp=0.5;
%     vp_max=0.5;
    %% Model equations - Path following error
    xdot = [vp*cos(x3)-u2*(1-cs*x2);...
            vp*sin(x3)-cs*x1*u2; ...
            u1-cs*u2;...
            u2];
    % subject to constraint
    rmax=0.5;rmin=-0.5;
    dsmax=1.5;dsmin=-1.5;
    umax=[rmax;dsmax];
    umin=[rmin;dsmin];

    %% => gains for nonlinear controller 
    k1=(dsmax-vp_max);
    k2=0.5*(rmax-dsmax*cs_max);
    k3=0.5*(rmax-dsmax*cs_max)/(0.5*vp_max);
    %% Objective term
    Q=diag([1 1 1 1 10]);
%     Q=diag([1 1 6 10 20]);
    L = [x1 x2 x3 vp*cos(x3)-u2 u1-cs*u2]*Q*[x1;x2;x3;vp*cos(x3)-u2;u1-cs*u2];
    % L = s1^2+5*y1^2+[theta vp*cos(theta)-u_g r-cs*u_g]*Q*[theta;vp*cos(theta)-u_g;r-cs*u_g];
    %% Continuous time dynamics
    f = Function('f', {x, u}, {xdot, L});

    %% Formulate discrete time dynamics
    if false
       % CVODES from the SUNDIALS suite
       dae = struct('x',x,'p',u,'ode',xdot,'quad',L);
       opts = struct('tf',T/N);
       F = integrator('F', 'cvodes', dae, opts);
    else
       % Fixed step Runge-Kutta 4 integrator
       M = 4; % RK4 steps per interval
       DT = T/N/M;
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
    end

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
    X0 = MX.sym('X0', 4);
    w = {w{:}, X0};
    lbw = [lbw; x0];
    ubw = [ubw; x0];
    w0 = [w0; x0];

    % Formulate the NLP
    Xk = X0;
    V0_mpc=0.5*k3*log(1+Xk(1)^2 + Xk(2)^2) + 0.5*Xk(3)^2;
    dV_non=-k1*k3*Xk(1)*tanh(Xk(1))/(1+Xk(1)^2+Xk(2)^2)-k2*Xk(3)*tanh(Xk(3));   % derivative of lyapunov function
    for k=0:N-1
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
%     g = {g{:}, dV_mpc-dV_non};
%     lbg = [lbg; -inf];
%     ubg = [ubg; 0];
    %   
    %% Create an NLP solver
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
    options = struct('ipopt',struct('print_level',0),'print_time',false);
    solver = nlpsol('solver', 'ipopt', prob, options);
    w0(1:4)=x0;
    % Initialize guess
    u0=[0;0];
    for i=1:N
        Fk = F('x0',x0,'p',u0);
        x0=full(Fk.xf);
        w0(6*i+1:6*i+4)=x0;
        w0(6*i-1:6*i)=u0;
    end
end

V_mpc=0.5*k3*log(1+x0(1)^2 + x0(2)^2) + 0.5*x0(3)^2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MPCPF controller 
%     speed=0.5;
%     cs=0.1;
%     vp=speed;
% 
%     vd=0.8+0.2*cos(0.1*x0(4));
% %     vd=0.5;
% %    Solve NMPC OCP
%     w0(1:4)=x0;
%     lbw(1:nx)=x0;
%     ubw(1:nx)=x0;
%     sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
%                 'lbg', lbg, 'ubg', ubg);
%     w_opt = full(sol.x);
%     u_mpc=w_opt(5:6);
%     chi_d=u_mpc(1);
%     u_g=u_mpc(2);
%     w0=w_opt;
%     
%% Nonlinear PF controller
%     speed=0.5;
%     cs=0.1;
%     vp=speed;
    
    cs=0.1*sin(.1*x0(4));
    vd=0.6+0.1*cos(0.1*x0(4));
    vp=vd;
    if x0(3)==0
        chi_d=-k3*x0(2)*vp/(1+x0(1)^2+x0(2)^2)+cs*u_g;
    else
        chi_d=-k3*x0(2)*vp*sin(x0(3))/(x0(3)*(1+x0(1)^2+x0(2)^2))-k2*tanh(x0(3))+cs*u_g;
    end    
%     chi_d=chi_d*180/pi;
    u_g=vp*cos(x0(3))+k1*tanh(x0(1));

 %%%%%%%%%%
%     dU=-0.1*0.2*sin(0.1*x0(4))*u_g;
%     vc=sqrt(Vcxy_hat(1)^2+Vcxy_hat(2)^2);
%     mcos=Vcxy_hat(1)*cos(yaw)+Vcxy_hat(2)*sin(yaw);
%     msin=-(Vcxy_hat(1)*sin(yaw)-Vcxy_hat(2)*cos(yaw));
%     if t>=5
%         M=sqrt(U^2-vc^2*msin^2);
%         r_d=(M*chi_d+dU*vc*msin/U)/(M-vc*mcos);
%     else
%         r_d=chi_d;
%     end     
%     r_d=r_d*180/pi;
    r_d=chi_d*180/pi;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if t<15
%     r_d=0;
%     u_g=0;
%     speed=0;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% Yaw rate controller
        D_max=60;  
        D_min=-60;
        er=r_d-r_meas;
        Kp_r=4;
        KI_r=4;
        KD_r=0;
        Ka_r=4;
        Diff_Mode=Kp_r*er+(Ts*KI_r-Kp_r)*er_old+Ka_r*Ts*Sat_D_Mode_old+(1-Ka_r*Ts)*Diff_Mode_old;
        Sat_D_Mode=min(D_max, max(D_min, Diff_Mode));
        Diff_Mode_old=Diff_Mode;
        Sat_D_Mode_old=Sat_D_Mode;
        er_old=er;
        Diff_Mode=Sat_D_Mode;

%%   Send data to vehicle     
            z=[vd;Diff_Mode;pd;Vcxy_hat;r_d;u_g;x0];

end
function alg_t=convert(alg,alg_old, alg_t_old)
    if (alg-alg_old>3*pi/2)
        alg_t=alg_t_old+alg-alg_old-2*pi;
    elseif (alg_old-alg>3*pi/2)
         alg_t=alg_t_old+alg-alg_old+2*pi;
    else
        alg_t=alg_t_old+alg-alg_old;
    end
end
function [pd,theta_c,gamma]=path(t,u_g,Ts)
    global gamma_old theta_c_old pd_old 
    if t==0
         theta_c_old=0;
         gamma_old=0;
         pd_old=[0;0];
    end   
% R1=10;
% R2=10;
% c1=1/R1;
% c2=1/R2;
% k1=1;
% k2=1;
% g1=50;
% g2=g1+R1*pi;
% g3=g2+g1;
% g4=g3+R2*pi;
% g5=g4+g1;
% cs=c1/(1+(exp(-k1*(gamma_old-g1))))-c1/(1+(exp(-k2*(gamma_old-g2))))-c2/(1+(exp(-k2*(gamma_old-g3))))+c2/(1+(exp(-k2*(gamma_old-g4))));
%    cs=c1*sin(0.1*gamma_old);
%  cs=0.1;
% % a=20;
% % w=.005;
% % num=2*sqrt(2)*(2+2*cos(2*w*gamma_old))*sin(w*gamma_old);
% % den=a*(2+2*cos(2*w*gamma_old)+cos(4*w*gamma_old))^(1.5);
% % cs=num/den;

%        cs=0.1*sin(.1*gamma_old);
%        theta_c_dot=cs*u_g;
%        gamma_dot=u_g;
%        pd_dot=[cos(theta_c_old);sin(theta_c_old)]*u_g;

    yold=[theta_c_old;gamma_old;pd_old];   
    n1=f(yold,u_g);    
    n2=f(yold+Ts*n1/2,u_g);
    n3=f(yold+Ts*n2/2,u_g);
    n4=f(yold+Ts*n3,u_g);
    ynew=yold+Ts*(n1+2*n2+2*n3+n4)/6;
    theta_c=ynew(1);
    gamma=ynew(2);
    pd=ynew(3:4);
    
    
%    theta_c=theta_c_old+Ts*theta_c_dot;
%    gamma=gamma_old+Ts*gamma_dot;
%    pd=pd_old+Ts*pd_dot;
   
   theta_c_old=theta_c;
   gamma_old=gamma;
   pd_old=pd;
end
%% Update Runge Kutta
function k=f(yn,u_g)
theta_c=yn(1);
gamma=yn(2);
%% Lawnmover
% R1=10;
% R2=7;
% c1=1/R1;
% c2=1/R2;
% k1=1;
% k2=1;
% g1=50;
% g2=g1+R1*pi;
% g3=g2+g1;
% g4=g3+R2*pi;
% g5=g4+g1;
% cs=c1/(1+(exp(-k1*(gamma-g1))))-c1/(1+(exp(-k2*(gamma-g2))))-c2/(1+(exp(-k2*(gamma-g3))))+c2/(1+(exp(-k2*(gamma-g4))));

%%
cs=0.1*sin(.1*gamma);
% cs=0.1;
k=[cs;1;cos(theta_c);sin(theta_c)]*u_g;
end


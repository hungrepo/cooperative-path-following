
%% This code is used to generate simulation result for triangular formation in the paper 

%   Cooperative path following of constrained autonomous vehicles with model predictive control 
%   and event-triggered communications”, International Journal of Robust Nonlinear Control, 2020.

%   Author: Nguyen Tuan Hung, Antonio Pascoal, Institute System and Robotic, IST, Lisbon
%           Tor A. Johansen, NTNU, Norway 
%   Contact: nguyen.hung@tecnico.ulisboa.pt 
         
% =========================================================================================================================
function MPC_CPF

clear all;
close all;
%% ==========================================================================================================================
%    
%  Initialization
%
%% ========================================================================================================================== 
mission = 'triangular';
T = 81;                        % simulation time
Ts = 0.2;                       % sampling interval    
N = T/Ts;                       % number of interation
t = [0:N].*Ts;                    

%% Initialize the vehicle state (position in 2D and orientation)
% Vehicle 1
    p1_0 = [-5;-23]; psi1_0 = pi/2;
    x_vehicle1_0 = [p1_0;psi1_0];
% Vehicle 2
    p2_0=[-10;-15]; psi2_0 = pi/4+pi/8;
    x_vehicle2_0 = [p2_0;psi2_0];
% Vehicle 3
    p3_0 = [-15;-3]; psi3_0 = pi/4;
    x_vehicle3_0 = [p3_0;psi3_0];
% Vehicle 4
    p4_0 = [-10;12]; psi4_0 = -pi/4;
    x_vehicle4_0 = [p4_0;psi4_0];
% Vehicle 5
    p5_0 = [-6;30]; psi5_0 = -pi/2;
    x_vehicle5_0 = [p5_0;psi5_0];
    
    
% vehicle constraints on the heading rate, the same for all vehicles;
    vehicle.rmax =  0.2;
    vehicle.rmin = -0.2;
    
%% Setting the paths
% Path 1
    gamma1_0 = 0;
    a1 = 50; c1 = 0; d1 = -10;
    path1_para = [a1;c1;d1];
    x_path1_0 = path(gamma1_0,path1_para);
    psid1_old =  x_path1_0(3);
    psid1_out_old = x_path1_0(3);
% Path 2
    gamma2_0=-.0;
    a2 = 50; c2 = -0.1; d2 = -5;
    path2_para = [a2;c2;d2];
    x_path2_0 = path(gamma2_0, path2_para);  
    psid2_old =  x_path2_0(3);
    psid2_out_old = x_path2_0(3);
% Path 3 
    gamma3_0=-.0; 
    a3 = 50; c3 = -0.2; d3 = 0;
    path3_para = [a3;c3;d3];
    x_path3_0 = path(gamma3_0,path3_para);  
    psid3_old =  x_path3_0(3);
    psid3_out_old = x_path3_0(3);
% Path 4 
   gamma4_0 = .0; 
    a4 = 50; c4 = -0.1; d4 = 5;
    path4_para = [a4;c4;d4];
    x_path4_0 = path(gamma4_0,path4_para);  
    psid4_old =  x_path4_0(3);
    psid4_out_old = x_path4_0(3);
% Path 5 
    gamma5_0 = .0;
    a5 = 50;  c5 = 0; d5 = 10;
    path5_para = [a5; c5; d5];
    x_path5_0 = path(gamma5_0,path5_para);
    psid5_old =  x_path5_0(3);
    psid5_out_old = x_path5_0(3);
        
%% Seting the desired Formation (Note: the sign of c1,...,c5 in the paper should be negative) 
%     c1 =  0.0; c5 =  0.0;
%     c2 = -0.1; c4 = -0.1;
%     c3 = -0.2;
%     
% Desired speed profile for the path parameter along the path
    vd = 0.02;
    
%% gains for the coordinaiton controllers
kc = 0.02;
kc1 = kc; kc2 = kc; kc3 = kc; kc4 = kc; kc5 = kc;     
%% Formulate mpcController for MPC
    Np = 10;        % number of control intervals
    Tp = Ts*Np;     % Time horizon
    import casadi.*
    [mpcController1,w01,lbw1,ubw1,lbg1,ubg1,nx1,nu1,k11,k12,k13] = mpcController(Np,Tp,Ts,path1_para,vd,vehicle);
    [mpcController2,w02,lbw2,ubw2,lbg2,ubg2,nx2,nu2,k21,k22,k23] = mpcController(Np,Tp,Ts,path2_para,vd,vehicle);
    [mpcController3,w03,lbw3,ubw3,lbg3,ubg3,nx3,nu3,k31,k32,k33] = mpcController(Np,Tp,Ts,path3_para,vd,vehicle);
    [mpcController4,w04,lbw4,ubw4,lbg4,ubg4,nx4,nu4,k41,k42,k43] = mpcController(Np,Tp,Ts,path4_para,vd,vehicle);
    [mpcController5,w05,lbw5,ubw5,lbg5,ubg5,nx5,nu5,k51,k52,k53] = mpcController(Np,Tp,Ts,path5_para,vd,vehicle);

%% Variable for event triggered communications 
% Setting trigering functions 
    eps0 = 5e-3;
    c_1 = 0.1;
    alpha = 0.2;
% Setting transmit signal (bool)
    TR_1 = []; TR_2 = []; TR_3 = []; TR_4 = []; TR_5 = [];
% At vehicle 1 
    Gamma_hat11=0;
    Gamma_hat12=0;
% At vehicle 2
    Gamma_hat22=0;
    Gamma_hat21=0;
    Gamma_hat23=0;
% At vehicle 3
    Gamma_hat33=0;
    Gamma_hat32=0;
    Gamma_hat34=0;
% At vehicle 4
    Gamma_hat43=0;
    Gamma_hat44=0;
    Gamma_hat45=0;
% At vehicle 5
    Gamma_hat54=0;
    Gamma_hat55=0;
% Estimation error of gamma
  Gamma1_tilde=[]; Gamma2_tilde=[]; Gamma3_tilde=[]; Gamma4_tilde=[]; Gamma5_tilde=[];

% Where store mpc controller;
    u_mpc1=[];
    u_mpc2=[];
    u_mpc3=[];
    u_mpc4=[];
    u_mpc5=[];
% Where store the Lyapunov function
    V_mpc1=[];
    V_mpc2=[];
    V_mpc3=[];
    V_mpc4=[];
    V_mpc5=[];
% Where store the input of the vehicle
    u_vehicle1=[];
    u_vehicle2=[];
    u_vehicle3=[];
    u_vehicle4=[];
    u_vehicle5=[];

    uc=[];
% Where store path following position error
    e_pf1=[];
    e_pf2=[];
    e_pf3=[];
    e_pf4=[];
    e_pf5=[];

%%
v1_old=0;
v2_old=0;
v3_old=0;
v4_old=0;
v5_old=0;

Tr_Mess_1=[]; Data1_D=[];
Tr_Mess_2=[]; Data2_D=[];
Tr_Mess_3=[]; Data3_D=[];
Tr_Mess_4=[]; Data4_D=[];
Tr_Mess_5=[]; Data5_D=[];
%% Store receiving signal
    % at vehicle 1
    RE_12=[]; Re_Mess_12=[]; 
    % at vehicle 2
    RE_21=[]; RE_23=[]; Re_Mess_21=[]; Re_Mess_23=[];
    % at vehicle 3
    RE_32=[]; RE_34=[]; Re_Mess_32=[]; Re_Mess_34=[];
    % at vehicle 4
    RE_43=[]; RE_45=[]; Re_Mess_43=[]; Re_Mess_45=[];
    % at vehicle 5
    RE_54=[]; Re_Mess_54=[];
%--------------------------------------------------------------------------
x_vehicle1=x_vehicle1_0;
x_vehicle2=x_vehicle2_0;
x_vehicle3=x_vehicle3_0;
x_vehicle4=x_vehicle4_0;
x_vehicle5=x_vehicle5_0;

x_path1=x_path1_0;
x_path2=x_path2_0;
x_path3=x_path3_0;
x_path4=x_path4_0;
x_path5=x_path5_0;


Gamma1 = gamma1_0;
Gamma2 = gamma2_0;
Gamma3 = gamma3_0;
Gamma4 = gamma4_0;
Gamma5 = gamma5_0;

Vc=[];
Eta1=[]; Eta2=[];Eta3=[]; Eta4=[]; Eta5=[];
Delta = 2;                              % Communication delay
%==========================================================================================================================
%  End Initialization  
%========================================================================================================================== 
%% ==========================================================================================================================
%  
%   Start the simulation
% 
%% ==========================================================================================================================
for i = 1:N

%% Compute path following errors   

e_pf1i = epf(x_vehicle1,x_path1);
e_pf2i = epf(x_vehicle2,x_path2);
e_pf3i = epf(x_vehicle3,x_path3);
e_pf4i = epf(x_vehicle4,x_path4);
e_pf5i = epf(x_vehicle5,x_path5);
e_pf1 = [e_pf1 e_pf1i];
e_pf2 = [e_pf2 e_pf2i];
e_pf3 = [e_pf3 e_pf3i];
e_pf4 = [e_pf4 e_pf4i];
e_pf5= [e_pf5 e_pf5i];

%% Compute the disagreement function V_c
L = [ 1  -1  0  0  0;        % Laplacian matrix represeting the connectivity of the graph
     -1   2 -1  0  0;
      0  -1  2 -1  0;
      0   0 -1  2 -1;
      0   0  0 -1  1];

GAMMA_i = [Gamma1(end);Gamma2(end);Gamma3(end);Gamma4(end);Gamma5(end)];
Vc_i=GAMMA_i'*L*GAMMA_i;                % coordination disgreement function
Vc=[Vc Vc_i];

%% Check trigger conditions
    gamma1_tilde = Gamma_hat11(end)-Gamma1(end); 
    gamma2_tilde = Gamma_hat22(end)-Gamma2(end); 
    gamma3_tilde = Gamma_hat33(end)-Gamma3(end);
    gamma4_tilde = Gamma_hat44(end)-Gamma4(end); 
    gamma5_tilde = Gamma_hat55(end)-Gamma5(end);
    % compute estimation error of gamma
    Gamma1_tilde(end+1) = gamma1_tilde;
    Gamma2_tilde(end+1) = gamma2_tilde;
    Gamma3_tilde(end+1) = gamma3_tilde;
    Gamma4_tilde(end+1) = gamma4_tilde;
    Gamma5_tilde(end+1) = gamma5_tilde;
    % compute estimation threashod error 
    eta = c_1*exp(-alpha*i*Ts)+eps0;
    eta1 = eta; eta2 = eta; eta3 = eta; eta4 = eta; eta5= eta;

    Eta1 = [Eta1;eta1];
    Eta2 = [Eta2;eta2];
    Eta3 = [Eta3;eta3];
    Eta4 = [Eta4;eta4];
    Eta5 = [Eta5;eta5];
    
%% Broadcast signal if the estimation error on gamma excced the thereashod
    % vehicle 1
    if abs(gamma1_tilde) > eta1
        Gamma_hat11(end) = Gamma1(end);
        Tr_1 = 1;
        Tr_Mess_1 = [Tr_Mess_1 [i*Ts;Gamma1(end)]];
        Data1_D = [Data1_D [i*Ts;Gamma1(end)]];                 % data to be transmit
    else
        Tr_1 = 0;
        Tr_Mess_1 = [Tr_Mess_1 [0;0]];
    end
    % vehicle 2
    if abs(gamma2_tilde) > eta2
        Gamma_hat22(end) = Gamma2(end);
        Tr_2 = 1;
        Tr_Mess_2 = [Tr_Mess_2 [i*Ts;Gamma2(end)]];
        Data2_D = [Data2_D [i*Ts;Gamma2(end)]];
    else
        Tr_2 = 0;
        Tr_Mess_2 = [Tr_Mess_2  [0;0]];
    end
    % vehicle 3
    if abs(gamma3_tilde) > eta3
        Gamma_hat33(end) = Gamma3(end);
        Tr_3 = 1;
        Tr_Mess_3 = [Tr_Mess_3 [i*Ts;Gamma3(end)]];
        Data3_D = [Data3_D [i*Ts;Gamma3(end)]];
    else
        Tr_3 = 0;
        Tr_Mess_3 = [Tr_Mess_3  [0;0]];
    end
    % vehicle 4
    if abs(gamma4_tilde) > eta4
        Gamma_hat44(end) = Gamma4(end);
        Tr_4 = 1;
        Tr_Mess_4 = [Tr_Mess_4 [i*Ts;Gamma4(end)]];
        Data4_D = [Data4_D [i*Ts;Gamma4(end)]];
    else
        Tr_4 = 0;
        Tr_Mess_4 = [Tr_Mess_4  [0;0]];
    end
    % vehicle 5
    if abs(gamma5_tilde) > eta5
        Gamma_hat55(end) = Gamma5(end);
        Tr_5 = 1;
        Tr_Mess_5 = [Tr_Mess_5 [i*Ts;Gamma5(end)]];
        Data5_D = [Data5_D [i*Ts;Gamma5(end)]];
    else
        Tr_5 = 0;
        Tr_Mess_5 = [Tr_Mess_5  [0;0]];
    end
%% Store transmision signal
TR_1 = [TR_1 Tr_1]; 
TR_2 = [TR_2 Tr_2]; 
TR_3 = [TR_3 Tr_3];
TR_4 = [TR_4 Tr_4]; 
TR_5 = [TR_5 Tr_5];    

%% Store receving signal
    t_c = i*Ts;

    if isempty(Data1_D);
        idx1 = [];
    else
        idx1 = find(t_c-Delta==Data1_D(1,:));
    end

    if isempty(Data2_D)
        idx2 = [];
    else
        idx2 = find(t_c-Delta==Data2_D(1,:));
    end
    if isempty(Data3_D)
        idx3 = [];
    else
        idx3 = find(t_c-Delta==Data3_D(1,:));
    end
    if isempty(Data4_D)
        idx4=[];
    else
        idx4 = find(t_c-Delta==Data4_D(1,:));
    end
    if isempty(Data5_D)
        idx5=[];
    else
        idx5 = find(t_c-Delta==Data5_D(1,:));
    end

%% Compensation of communication delay after receving data from neighbors
    % Vehicle 1
    if~isempty(idx1)
       Gamma_hat21(end) = Data1_D(2,idx1)+vd*Delta;
       Re_21 = 1;
       Re_Mess_21 = [Re_Mess_21 Data1_D(:,idx1)];
    else
       Re_21 = 0; 
       Re_Mess_21 = [Re_Mess_21 [0;0]];
    end
    % vehicle 2
    if~isempty(idx2)
       Gamma_hat12(end) = Data2_D(2,idx2)+vd*Delta;
       Gamma_hat32(end) = Data2_D(2,idx2)+vd*Delta;
       Re_12 = 1; Re_32 = 1;
       Re_Mess_12 = [Re_Mess_12 Data2_D(:,idx2)];
       Re_Mess_32 = [Re_Mess_32 Data2_D(:,idx2)];
    else
       Re_12 = 0; Re_32 = 0; 
       Re_Mess_12 = [Re_Mess_12 [0;0]];
       Re_Mess_32 = [Re_Mess_32 [0;0]];
    end
    % vehicle 3
    if ~isempty(idx3)
       Gamma_hat23(end) = Data3_D(2,idx3)+vd*Delta;
       Gamma_hat43(end) = Data3_D(2,idx3)+vd*Delta;
       Re_23 = 1; Re_43 = 1;
       Re_Mess_23 = [Re_Mess_23 Data3_D(:,idx3)];
       Re_Mess_43 = [Re_Mess_43 Data3_D(:,idx3)];
    else
       Re_23 = 0; Re_43 = 0;
       Re_Mess_23 = [Re_Mess_23 [0;0]];
       Re_Mess_43 = [Re_Mess_43 [0;0]];
    end
    % vehicle 4
    if ~isempty(idx4)
       Gamma_hat34(end) = Data4_D(2,idx4)+vd*Delta;
       Gamma_hat54(end) = Data4_D(2,idx4)+vd*Delta;
       Re_34 = 1; Re_54 = 1;
       Re_Mess_34 = [Re_Mess_34 Data4_D(:,idx4)];
       Re_Mess_54 = [Re_Mess_54 Data4_D(:,idx4)];
    else
       Re_34 = 0; Re_54 = 0;
       Re_Mess_34 = [Re_Mess_34 [0;0]];
       Re_Mess_54 = [Re_Mess_54 [0;0]];
    end
    % vehicle 5
    if~isempty(idx5)
       Gamma_hat45(end) = Data5_D(2,idx5)+vd*Delta;
       Re_45 = 1;
       Re_Mess_45 = [Re_Mess_45 Data5_D(:,idx5)];
    else
       Re_45 = 0; 
       Re_Mess_45 = [Re_Mess_45 [0;0]];
    end

    RE_21 = [RE_21 Re_21];
    RE_12 = [RE_12 Re_12];
    RE_32 = [RE_32 Re_32];
    RE_23 = [RE_23 Re_23];
    RE_34 = [RE_34 Re_34];
    RE_43 = [RE_43 Re_43];
    RE_45 = [RE_45 Re_45];
    RE_54 = [RE_54 Re_54];
    
%% Update coordination controllers
    l = 5;
    uc1 = -kc1*tanh(l*(Gamma1(end)-Gamma_hat12(end))); 
    uc2 = -kc2*tanh(l*(2*Gamma2(end)-Gamma_hat21(end)-Gamma_hat23(end)));
    uc3 = -kc3*tanh(l*(2*Gamma3(end)-Gamma_hat32(end)-Gamma_hat34(end)));
    uc4 = -kc4*tanh(l*(2*Gamma4(end)-Gamma_hat43(end)-Gamma_hat45(end)));
    uc5 = -kc5*tanh(l*(Gamma5(end)-Gamma_hat54(end))); 

    % uc1=-k*tanh(l*(Gamma1(end)-Gamma2(end))); 
    % uc2=-k*tanh(l*(2*Gamma2(end)-Gamma1(end)-Gamma3(end)));
    % uc3=-k*tanh(l*(2*Gamma3(end)-Gamma2(end)-Gamma4(end)));
    % uc4=-k*tanh(l*(2*Gamma4(end)-Gamma3(end)-Gamma5(end)));
    % uc5=-k*tanh(l*(Gamma5(end)-Gamma4(end))); 

    uc=[uc [uc1;uc2;uc3;uc4;uc5]];
%% Compute the Lyapunov function
    x0_1 = [e_pf1i;uc1];
    x0_2 = [e_pf2i;uc2];
    x0_3 = [e_pf3i;uc3];
    x0_4 = [e_pf4i;uc4];
    x0_5 = [e_pf5i;uc5];


    V_mpc1 = [V_mpc1;0.5*k13*log(1+x0_1(1)^2 + x0_1(2)^2) + 0.5*x0_1(3)^2];
    V_mpc2 = [V_mpc2;0.5*k23*log(1+x0_2(1)^2 + x0_2(2)^2) + 0.5*x0_2(3)^2];
    V_mpc3 = [V_mpc3;0.5*k33*log(1+x0_3(1)^2 + x0_3(2)^2) + 0.5*x0_3(3)^2];
    V_mpc4 = [V_mpc4;0.5*k43*log(1+x0_4(1)^2 + x0_4(2)^2) + 0.5*x0_4(3)^2];
    V_mpc5 = [V_mpc5;0.5*k53*log(1+x0_5(1)^2 + x0_5(2)^2) + 0.5*x0_5(3)^2];

%% Solve optimal control problem asscociated with the MPC
% 
% Solve OPC for vehicle 1
    w01(1:nx1)=x0_1;
    lbw1(1:nx1)=x0_1;
    ubw1(1:nx1)=x0_1;
    sol1 = mpcController1('x0', w01, 'lbx', lbw1, 'ubx', ubw1,...
                  'lbg', lbg1, 'ubg', ubg1);
    w_opt1 = full(sol1.x);
    u_mpci_1=w_opt1(6:7);
    r_d1=u_mpci_1(1);
    u_gamma1=u_mpci_1(2);
    w01=w_opt1;
    u_mpc1(:,end+1)=u_mpci_1;
% Solve OPC for vehicle 2
    w02(1:nx2)=x0_2;
    lbw2(1:nx2)=x0_2;
    ubw2(1:nx2)=x0_2;
    sol2 = mpcController2('x0', w02, 'lbx', lbw2, 'ubx', ubw2,...
                  'lbg', lbg2, 'ubg', ubg2);
    w_opt2 = full(sol2.x);
    u_mpci_2=w_opt2(6:7);
    r_d2=u_mpci_2(1);
    u_gamma2=u_mpci_2(2);
    w02=w_opt2;
    u_mpc2(:,end+1)=u_mpci_2;
% Solve OPC for vehicle 3
    w03(1:nx3)=x0_3;
    lbw3(1:nx3)=x0_3;
    ubw3(1:nx3)=x0_3;
    sol3 = mpcController3('x0', w03, 'lbx', lbw3, 'ubx', ubw3,...
                  'lbg', lbg3, 'ubg', ubg3);
    w_opt3 = full(sol3.x);
    u_mpci_3=w_opt3(6:7);
    r_d3=u_mpci_3(1);
    u_gamma3=u_mpci_3(2);
    w03=w_opt3;
    u_mpc3(:,end+1)=u_mpci_3;
% Solve OPC for vehicle 4
    w04(1:nx4)=x0_4;
    lbw4(1:nx4)=x0_4;
    ubw4(1:nx4)=x0_4;
    sol4 = mpcController4('x0', w04, 'lbx', lbw4, 'ubx', ubw4,...
                  'lbg', lbg4, 'ubg', ubg4);
    w_opt4 = full(sol4.x);
    u_mpci_4=w_opt4(6:7);
    r_d4=u_mpci_4(1);
    u_gamma4=u_mpci_4(2);
    w04=w_opt4;
    u_mpc4(:,end+1)=u_mpci_4;
% Solve OPC for vehicle 5
    w05(1:nx5)=x0_5;
    lbw5(1:nx5)=x0_5;
    ubw5(1:nx5)=x0_5;
    sol5 = mpcController5('x0', w05, 'lbx', lbw5, 'ubx', ubw5,...
                  'lbg', lbg5, 'ubg', ubg5);
    w_opt5 = full(sol5.x);
    u_mpci_5=w_opt5(6:7);
    r_d5=u_mpci_5(1);
    u_gamma5=u_mpci_5(2);
    w05=w_opt5;
    u_mpc5(:,end+1)=u_mpci_5;    

%% Compute the vehicles' speeds
% Vehicle 1 
    hg_1=a1;
    u_d1=hg_1*(vd+uc1);
% Vehicle 2
    hg_2=a2;
    u_d2=hg_2*(vd+uc2);
% Vehicle 3
    hg_3=a3;
    u_d3=hg_3*(vd+uc3);
% Vehicle 4
    hg_4=a4;
    u_d4=hg_4*(vd+uc4);
% Vehicle 5
    hg_5=a5;
    u_d5=hg_5*(vd+uc5);    
%% 

%% CPF Lyapunov Based
% [r_d1, u_gamma1]=u_L(x0_1,k11,k12,k13,u_d1,hg_1,v1_old);
% [r_d2, u_gamma2]=u_L(x0_2,k21,k22,k23,u_d2,hg_2,v2_old);
% [r_d3, u_gamma3]=u_L(x0_3,k31,k32,k33,u_d3,hg_3,v3_old);
% [r_d4, u_gamma4]=u_L(x0_4,k41,k42,k43,u_d4,hg_4,v4_old);
% [r_d5, u_gamma5]=u_L(x0_5,k51,k52,k53,u_d5,hg_5,v5_old);
% v1_old=u_gamma1;
% v2_old=u_gamma2;
% v3_old=u_gamma3;
% v4_old=u_gamma4;
% v5_old=u_gamma5;

%% Update the state of the vehicles and the paths
    
    %Update vehicle 1 
    input1=[u_d1;r_d1];
    u_vehicle1=[u_vehicle1 input1];
    x_vehicle1(:,end+1)=update_vehicle(x_vehicle1(:,end),input1,Ts);
    % Update Path 1        
    Gamma1(end+1) =  update_gamma(Gamma1(end),u_gamma1,Ts); 
    x_path1(:,end+1) = path(Gamma1(end),path1_para); 
    % Because of the function actan2, psi_d computed in the function "path"
    % is not continous, we need to make it continuous with the function
    % alg_convert
%     psid1_new= x_path1(3,end);
%     psid1_out = alg_convert(psid1_new, psid1_old, psid1_out_old);
%     x_path1(3,end) = psid1_out;
%     psid1_out_old = psid1_out;
%     psid1_old = psid1_new;
    
    % Update vehicle 2 
    input2=[u_d2;r_d2];
    u_vehicle2=[u_vehicle2 input2];
    x_vehicle2(:,end+1)=update_vehicle(x_vehicle2(:,end),input2,Ts);
    % update path 2
    Gamma2(end+1) =  update_gamma(Gamma2(end),u_gamma2,Ts); 
    x_path2(:,end+1) = path(Gamma2(end),path2_para); 
    % make psid_2 continous 
%     psid2_new= x_path2(3,end);
%     psid2_out = alg_convert(psid2_new, psid2_old, psid2_out_old);
%     x_path2(3,end) = psid2_out;
%     psid2_out_old = psid2_out;
%     psid2_old = psid2_new;
    
    % Update vehicle 3 
    input3=[u_d3;r_d3];
    u_vehicle3=[u_vehicle3 input3];
    x_vehicle3(:,end+1)=update_vehicle(x_vehicle3(:,end),input3,Ts);
    % update Path 3        
    Gamma3(end+1) = update_gamma(Gamma3(end),u_gamma3,Ts); 
    x_path3(:,end+1) = path(Gamma3(end),path3_para); 
    % make psid_3 continous 
%     psid3_new= x_path3(3,end);
%     psid3_out = alg_convert(psid3_new, psid3_old, psid3_out_old);
%     x_path3(3,end) = psid3_out;
%     psid3_out_old = psid3_out;
%     psid3_old = psid3_new;
    % Update Vehicle 4 
    input4=[u_d4;r_d4];
    u_vehicle4=[u_vehicle4 input4];
    x_vehicle4(:,end+1)=update_vehicle(x_vehicle4(:,end),input4,Ts);
    % update Path 4    
    Gamma4(end+1) =  update_gamma(Gamma4(end),u_gamma4,Ts); 
    x_path4(:,end+1) = path(Gamma4(end),path4_para); 
%     psid4_new= x_path4(3,end);
%     psid4_out = alg_convert(psid4_new, psid4_old, psid4_out_old);
%     x_path4(3,end) = psid4_out;
%     psid4_out_old = psid4_out;
%     psid4_old = psid4_new;
    % Update Vehicle 5 
    input5=[u_d5;r_d5];
    u_vehicle5=[u_vehicle5 input5];
    x_vehicle5(:,end+1)=update_vehicle(x_vehicle5(:,end),input5,Ts);
    % update Path 5    
    Gamma5(end+1) =  update_gamma(Gamma5(end),u_gamma5,Ts); 
    x_path5(:,end+1) = path(Gamma5(end),path5_para); 
%     psid5_new= x_path5(3,end);
%     psid5_out = alg_convert(psid5_new, psid5_old, psid5_out_old);
%     x_path5(3,end) = psid5_out;
%     psid5_out_old = psid5_out;
%     psid5_old = psid5_new;

    %% Update gamma_hat    
    
   Gamma_hat11(:,end+1)=update_gammahat(Gamma_hat11(end),0,Ts,t,vd);
   Gamma_hat12(:,end+1)=update_gammahat(Gamma_hat12(end),0,Ts,t,vd);
   Gamma_hat22(:,end+1)=update_gammahat(Gamma_hat22(end),0,Ts,t,vd);
   Gamma_hat21(:,end+1)=update_gammahat(Gamma_hat21(end),0,Ts,t,vd);
   Gamma_hat23(:,end+1)=update_gammahat(Gamma_hat23(end),0,Ts,t,vd);
   Gamma_hat33(:,end+1)=update_gammahat(Gamma_hat33(end),0,Ts,t,vd);
   Gamma_hat32(:,end+1)=update_gammahat(Gamma_hat32(end),0,Ts,t,vd);
   Gamma_hat34(:,end+1)=update_gammahat(Gamma_hat34(end),0,Ts,t,vd);
   Gamma_hat43(:,end+1)=update_gammahat(Gamma_hat43(end),0,Ts,t,vd);
   Gamma_hat44(:,end+1)=update_gammahat(Gamma_hat44(end),0,Ts,t,vd);
   Gamma_hat45(:,end+1)=update_gammahat(Gamma_hat45(end),0,Ts,t,vd);
   Gamma_hat54(:,end+1)=update_gammahat(Gamma_hat54(end),0,Ts,t,vd);
   Gamma_hat55(:,end+1)=update_gammahat(Gamma_hat55(end),0,Ts,t,vd);
      
end
    x_vehicle1=x_vehicle1';
    x_vehicle2=x_vehicle2';
    x_vehicle3=x_vehicle3';
    x_vehicle4=x_vehicle4';
    x_vehicle5=x_vehicle5';

    x_path1=x_path1';
    x_path2=x_path2';
    x_path3=x_path3';
    x_path4=x_path4';
    x_path5=x_path5';

    u_vehicle1=u_vehicle1';
    u_vehicle2=u_vehicle2';
    u_vehicle3=u_vehicle3';
    u_vehicle4=u_vehicle4';
    u_vehicle5=u_vehicle5';


    e_pf1=e_pf1';
    e_pf2=e_pf2';
    e_pf3=e_pf3';
    e_pf4=e_pf4';
    e_pf5=e_pf5';

    save_to_base(1);
end

%% ==========================================================================================================================
%  
%   Other functions, methods
% 
%% ==========================================================================================================================
%% Formulate mpcController
function [solver,w0,lbw,ubw,lbg,ubg,nx,nu,k1,k2,k3]=mpcController(Np,Tp,Ts,path_para,vd, vehicle)

import casadi.*

s1 = SX.sym('s1');
y1 = SX.sym('y1');
psie = SX.sym('psie');
gamma = SX.sym('gamma');
uc = SX.sym('uc');
x = [s1;y1;psie;gamma;uc];
nx=length(x);
% input of PF error system 
r = SX.sym('r');
u_g = SX.sym('u_g');
u = [r;u_g];
nu = length(u);
%% PF error system
 
a = path_para(1); 
hg_i=a;
hg_max=a;
cg_i=0;
cg_max=1/a;

vd_max=vd;
% ds_max=0.85;
% ds_min=-0.85;
% Input constraints
rmax=vehicle.rmax;rmin=vehicle.rmin;
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
 k2 = 0.06;
 k3= 0.09;
% k1=hg_i*(dgmax-vd_max);
 k1 = hg_i*0.01;

% k2=0.2*(rmax-dgmax*cg_max*hg_max);
% k3=0.8*(rmax-dgmax*cg_max*hg_max)/(0.5*vd_max*hg_i);

%% Objective term
Q=diag([1 1 2 2 20]);
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

%% Start with an empty mpcController
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

%% Formulate the mpcController
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

% Formulate the mpcController

Xk = X0;
V0_mpc=0.5*k3*log(1+Xk(1)^2 + Xk(2)^2) + 0.5*Xk(3)^2;
dV_non=-k1*k3*Xk(1)*tanh(Xk(1))/(1+Xk(1)^2+Xk(2)^2)-k2*Xk(3)*tanh(Xk(3));   % derivative of lyapunov function
for k=0:Np-1
    % New mpcController variable for the control
    Uk = MX.sym(['U_' num2str(k)],nu);
    w = {w{:}, Uk};
    lbw = [lbw; umin];
    ubw = [ubw;  umax];
    w0 = [w0;  uzero]; 
    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New mpcController variable for state at end of interval
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
%% Create an mpcController solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
solver = nlpsol('solver', 'ipopt', prob, options);
end            % for two neighbors 
%% Compute PF error
function e_pf=epf(x_vehicle,x_path)
    p=x_vehicle(1:2,end); psi=x_vehicle(3,end);
    pd=x_path(1:2,end); psid=x_path(3,end); gamma=x_path(4,end);  
    RI_F=[cos(psid)      sin(psid);     % From {I} to {F}
         -sin(psid)      cos(psid)]; 
    e_f=RI_F*(p-pd);
    s1=e_f(1);
    y1=e_f(2);
    psie=psi-psid; 
    e_pf=[s1;y1;psie;gamma];
end
function x_path= path(gamma, path_para)
    a = path_para(1);
    c = path_para(2);
    d = path_para(3);
    pd = [a*(gamma-c); d];
    pd_gamma = [a; 0];                   % partial derivative of pd respect to gamma
    psid = atan2(pd_gamma(2),pd_gamma(1));          % angle that tangent of the path with x_{I}
    x_path = [pd;psid;gamma];
end
function x_next=update_gammahat(x_current,input,Ts,t,vd)
    [time y]=ode45(@(t,y) gammahatdot(t,y,input,vd), [0, Ts], x_current);
     x_next=y(end,:)';
end
function dx = gammahatdot(t,x,u,vd)
    dx=vd;
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
%% Dynamics of Path and Vehicle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x_next=update_vehicle(x_current,input,Ts)
     [time y]=ode45(@(t,y) vehicle(t,y,input), [0, Ts], x_current);
     x_next=y(end,:)';
end
%% vehicle dynamics
function dx = vehicle(t,x,u)

% dot x = u*cos(psi)
% dot y = u*sin(psi)
% dot psi = r;

    psi=x(3);
    ur=u(1);
    r=u(2);
    dx=[ur*cos(psi);ur*sin(psi);r];
end
function dx = gammadot(t,x,u)
    dx=u;
end
function x_next=update_gamma(x_current,input,Ts)
    [time y]=ode45(@(t,y) gammadot(t,y,input), [0, Ts], x_current);
     x_next=y(end,:)';
end
function alg_out= alg_convert(alg_new, alg_old, alg_out_old)
    alg_e = alg_new - alg_old;
    if (alg_e > 3 * pi / 2)
        alg_out = alg_out_old - 2 * pi + alg_e;
    elseif (alg_e < -3 * pi / 2)
        alg_out = alg_out_old + 2 * pi + alg_e;
    else
        alg_out = alg_out_old + alg_e;
    end
end


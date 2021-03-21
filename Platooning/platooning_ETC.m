%   ======================================================================
%   This is an example how to apply event-triggered communication (ETC) for
%   vehicle coordination. We use the ETC mechanism in the paper entitled:

%   Nguyen T. Hung, Antonio M. Pascoal, 
%   “Consensus/synchronization of networked nonlinear multiple agent systems with event-triggered communications”, 
%   International Journal of Control, 2020.

%   Author: Nguyen Tuan Hung - nt-hung.github.io
function platooning_ETC
clear all;
% close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                                                          Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation time and sampling time
      T = 200; Ts=1e-01; 
      number_int = T/Ts;
%% Initialize agent
      A = 0;
      B = 1;
      n=length(A);                  % size of the state
      m=length(B(1,:));             % size of the input
      rng('default');
%% Initialize network
      Adja = [0  1  0  0  0  0  0  0;    
              1  0  1  0  0  0  0  0;   
              0  1  0  1  0  0  0  0;
              0  0  1  0  1  0  0  0;
              0  0  0  1  0  1  0  0;
              0  0  0  0  1  0  1  0;
              0  0  0  0  0  1  0  1;
              0  0  0  0  0  0  1  0];
      N=length(Adja(:,1)); 
      one=ones(N,1);
      D=diag(Adja*one);                                                 % Degree matrix    
      L=D-Adja;                      
%       [U,L_eig] = eig(L);                                            % Laplacian matrix
%       tem = sort(L_eig*one);
      L_eig=eig((L+L')/2);
      tem=sort(L_eig);
      lamda2 = tem(2);
      lamdaN = tem(N);
      one=ones(N,1);
      I_n=eye(n);
      I_N=eye(N);
      W=kron(eye(N)-one*one'/N, I_n);        % projection matrix
      R=eye(N);
%% Compute feedback control gain 
       c=-.2;
%% Event trigger threashod function
      %  h_i=c0+c1*exp(-c2*t(end));
        c0=0.1; c1=2; c2=.5;                                                   % parameter for the exponential function        
%% Data store during simulation 
x=cell(N,1);                                                                % For N state
s=[0; 10; 40; 110; 133; 167; 260; 280]; 
d=20;
for i=1:N
x{i,1}=s(i)-(i-1)*d;  
end
u=cell(N,1);                                                                % For N control input
Com_Sig=cell(N,1);                                                          % For N communication signal;
e=cell(N,1);                                                                % For N estimation error;
h=cell(N,1);                                                                % For N threashod function;
delta=cell(N,1);                                                            % For N triggering function; 
Com_mode=2;                                                                 % 1 if continous, 2 ETC with time, 3 ETC with state 
t=[];
V=[];
xi_norm=[];
x_hat=cell(N,1);
u_hat=cell(N,1);
    for i=1:N 
        x_hat{i,1}=zeros(n,1);
        u_hat{i,1}=zeros(m,1);
    end
x_hat=x;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%                                                      Start  Closed loop
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
for step=1:number_int
 t(end+1)=(step-1)*Ts;
 x1=x{1,1}(:,end);  x2=x{2,1}(:,end);   x3=x{3,1}(:,end);   x4=x{4,1}(:,end);
 x5=x{5,1}(:,end);  x6=x{6,1}(:,end);   x7=x{7,1}(:,end);   x8=x{8,1}(:,end);

 xi=W*[x1;x2;x3;x4;x5;x6;x7;x8];
 %V(:,end+1)=xi'*(kron(R,P_inv))*xi;
 xi_norm(:,end+1)=sqrt(xi'*xi);
 V(:,end+1)=norm(x1-x2)+norm(x2-x3)+norm(x3-x4);
%% Check mode of communication mode and update control law
    if Com_mode==1    
       for i=1:N
           tem=0;
           for j=1:N
               tem=tem+c*Adja(i,j)*(x{i,1}(:,end)-x{j,1}(:,end));
           end
           u{i,1}(:,end+1)=tem;
       end
    end
%% Time dependent trigger
  if Com_mode==2     
  % Update control input
        for i=1:N
           tem=0;
           for j=1:N
              % tem=tem+c*K*Adja(i,j)*(x{i,1}(:,end)-x_hat{j,1}(:,end));
               tem=tem+c*Adja(i,j)*(x_hat{i,1}(:,end)-x_hat{j,1}(:,end));
           end
           u{i,1}(:,end+1)=tem;
        end
   % Check communication
        for i=1:N
            e{i,1}(:,end+1)=x{i,1}(:,end)-x_hat{i,1}(:,end);
            h{i,1}(:,end+1)=c0+c1*exp(-c2*t(end));
%             if i==2
%                 h_i=c0+c1*exp(-c2*t(end));
%             end
            delta{i,1}(:,end+1)=norm(e{i,1}(:,end))-h{i,1}(:,end); 
        end
  % Broadcast signal
        for i=1:N
            if  delta{i,1}(:,end)>= 0
                x_hat{i,1}(:,end)=x{i,1}(:,end); 
                u_hat{i,1}(:,end+1)=u{i,1}(:,end);
                Com_Sig{i,1}(:,end+1)=1;
%                 for j=1:N
%                     if Adja(i,j)~=0
%                         x_hat{j,1}(:,end)=x{i,1}(:,end);
%                     end
%                 end
            else
                Com_Sig{i,1}(:,end+1)=0;
                u_hat{i,1}(:,end+1)=u_hat{i,1}(:,end);
            end
        end    
%         if u{i,1}(:,end)<0
%             u{i,1}(:,end)=0;
%         end
  end
%% Update state
for k=1:N
%       if (t(end)>100)&&(k==1)
%                x{k,1}(:,end+1)=update_x(x{k,1}(:,end),u{k,1}(:,end)+10*exp(-(t(end)-10)),Ts,t(end));
%       else
              x{k,1}(:,end+1)=update_x(x{k,1}(:,end),u{k,1}(:,end),Ts,t(end));
%      end
%% Update estimated state
    x_hat{k,1}(:,end+1)=update_xhat(x_hat{k,1}(:,end),0*u_hat{k,1}(:,end),Ts,t(end));
end

%% 
% eta(:,end+1)=W*x(:,end);
% V(:,end+1)=0.5*eta(:,end)'*eta(:,end);
end
    save_to_base(1);
end
function x_next=update_x(x_current,input,Ts,t)
    input_x.nSteps = 4;
    input_x.Ts=Ts;
    input_x.u=input;
    input_x.t=t;
    input_x.x = x_current;
    output_x= RK4_integrator(@xdot, input_x);
    x_next = output_x.value;
end
function x_next=update_xhat(x_current,input,Ts,t)
    input_xhat.nSteps = 4;
    input_xhat.Ts=Ts;
    input_xhat.u=input;
    input_xhat.t=t;
    input_xhat.x = x_current;
    output_xhat= RK4_integrator(@xhatdot, input_xhat);
    x_next = output_xhat.value;
end
function dx = xdot(t,x,u)
   n=length(x); 
      A = 0;
      B = 1;
      dx = 2 + 0.5*sin(t)+A*x+B*u;
end
function dx = xhatdot(t,x,u)
    n=length(x); 
    A = 0;
    B = 1;
    dx=2+0.5*sin(t)+A*x;
end

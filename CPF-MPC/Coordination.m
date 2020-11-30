function Coordination
clear all; 
close all;
T=500;
Ts=.1;
N=T/Ts;
gamma1_0= -40;
gamma2_0= 10;
gamma3_0= 0;
gamma_0=[gamma1_0;gamma2_0;gamma3_0];
gamma=gamma_0;
% k=0.3;
L=[ 1  0  -1;
    0  1  -1;
   -1 -1   2];
n=length(L(1,:));
one=ones(n,1);
I=eye(n);
W=I-one*one'/n;
V_gamma=[];
dV_gamma=[];
input=[];
u=[];
eta=[];
a=.3;
omega=.1;
l=a*omega;
lammda=eig(L);
lammda2=min(lammda(2:n));
k=l/lammda2;
urmin=zeros(n,1);
urmax=ones(n,1);
ur=[];
X=[];
for i=1:N
  gammai=gamma(:,end);
  gamma1=gammai(1);gamma2=gammai(2);gamma3=gammai(3);
  hg1=sqrt(1+a^2*omega^2*cos(omega*gamma1)^2);
  hg1=1.2;
%   hg2=sqrt(1+a^2*omega^2*cos(omega*gamma2)^2);
  hg2=1;
  hg3=sqrt(1+a^2*omega^2*cos(omega*gamma3)^2);
  hg3=1.25;
  vd=[0.5+a*sin(omega*gamma1);
      0.5+a*sin(omega*gamma2);
      0.5+a*sin(omega*gamma3)];
  etai=L*gammai;
  eta=[eta etai];
  eta_v=L*vd;
  V_gamma=[V_gamma  0.5*(etai)'*etai];
  
%   K=vd-k*L*gammai;
%   K=0.2*eye(n);
%   dV_gamma=[dV_gamma -etai'*W*K*tanh(L*etai)+etai'*W*vd];
%   ui= -K*tanh(L*gamma(:,end));
  R=diag([hg1 hg2 hg3]);
  R_inv=diag([1/hg1 1/hg2 1/hg3]);
  ui=R_inv*sat(R*(vd-k*L*gammai),urmin,urmax)-vd;
  X=[X [hg1;hg2;hg3]];
  ur=[ur R*(vd+ui)];
  u=[u ui];
  input=[input vd+ui];
  gamma(:,end+1)=update_gamma(gamma(:,end),ui,Ts);
%   [t,gamma_next] = ode45(@gamma_ode,[0 Ts],gamma(:,end));
%   gamma(:,end+1)=gamma_next(end,:)';
end
save_to_base(1);

end
function gamma_next=update_gamma(gamma_current,input,Ts)
    input_gamma.nSteps = 4;
    input_gamma.Ts=Ts;
    input_gamma.u=input;
    input_gamma.x = gamma_current;    
    output_gamma = RK4_integrator(@gamma, input_gamma);
    gamma_next = output_gamma.value;
end
function dx = gamma(t,x,u)
gamma1=x(1);
gamma2=x(2);
gamma3=x(3);
a=.3;
omega=.1;

  hg1=sqrt(1+a^2*omega^2*cos(omega*gamma1)^2);
  hg2=sqrt(1+a^2*omega^2*cos(omega*gamma2)^2);
  hg3=sqrt(1+a^2*omega^2*cos(omega*gamma3)^2);

dgamma1=0.5+a*sin(omega*gamma1)+u(1);
dgamma2=0.5+a*sin(omega*gamma2)+u(2);
dgamma3=0.5+a*sin(omega*gamma3)+u(3);
dx=[dgamma1;dgamma2;dgamma3];
end
function dx = gamma_ode(t,x)
gamma1=x(1);
gamma2=x(2);
gamma3=x(3);
gamma=x;
k=0.3;
L=[ 1 -1  0;
   -1  2 -1;
    0 -1  1]; 
u= -k*tanh(L*gamma(:,end));
a=.5;
omega=1;
dgamma1=0.5+a*(omega*gamma1)+u(1);
dgamma2=0.5+a*(omega*gamma2)+u(2);
dgamma3=0.5+a*(omega*gamma3)+u(3);
dx=[dgamma1;dgamma2;dgamma3];
end
%% component wise saturated function
function y=sat(x,xmin,xmax)
n=length(x);
y=zeros(n,1);
for i=1:n
y(i)=min(max(x(i),xmin(i)),xmax(i));
end
end

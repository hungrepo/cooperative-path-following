%m=250;
m=200;
N= length(Gamma1)/m;
t1=zeros(N,2);
t2=zeros(N,2);
t3=zeros(N,2);
t4=zeros(N,2);
t5=zeros(N,2);
t6=zeros(N,2);
t7=zeros(N,2);
t8=zeros(N,2);
t9=zeros(N,2);

t10=zeros(N,2);
t11=zeros(N,2);
t12=zeros(N,2);

t13=zeros(N,2);
t14=zeros(N,2);
t15=zeros(N,2);

% for gamma
t16=zeros(N,1);
t17=zeros(N,1);
t18=zeros(N,1);
t19=zeros(N,1);
t20=zeros(N,1);
% for communication signal
t21=zeros(N,1);
t22=zeros(N,1);
t23=zeros(N,1);
t24=zeros(N,1);
t25=zeros(N,1);

% for time 
t26=zeros(N,1);

p=1;
for i=0:N;
t1(i+1,:)=pd2(m*i+p,:);
t2(i+1,:)=p2(m*i+p,:);
t3(i+1,:)=pd3(m*i+p,:);
t4(i+1,:)=p3(m*i+p,:);
t5(i+1,:)=yaw2(m*i+p,:);
t6(i+1,:)=yaw3(m*i+p,:);
t7(i+1,:)=pd1(m*i+p,:);
t8(i+1,:)=p1(m*i+p,:); 
t9(i+1,:)=yaw1(m*i+p,:); 

t10(i+1,:)=pd4(m*i+p,:);
t11(i+1,:)=p4(m*i+p,:); 
t12(i+1,:)=yaw4(m*i+p,:); 

t13(i+1,:)=pd5(m*i+p,:);
t14(i+1,:)=p5(m*i+p,:); 
t15(i+1,:)=yaw5(m*i+p,:); 


% % for gamma
% t16(i+1,:)=Gamma1(:,m*i+p);
% t17(i+1,:)=Gamma2(:,m*i+p);
% t18(i+1,:)=Gamma3(:,m*i+p);
% t19(i+1,:)=Gamma4(:,m*i+p);
% t20(i+1,:)=Gamma5(:,m*i+p);
% % for communication signal
% t21(i+1,:)=TR_1(:,m*i+p);
% t22(i+1,:)=TR_2(:,m*i+p);
% t23(i+1,:)=TR_3(:,m*i+p);
% t24(i+1,:)=TR_4(:,m*i+p);
% t25(i+1,:)=TR_5(:,m*i+p);

% for time 
t26(i+1,:)=t(m*i+p);

end
pd2=t1;
p2=t2;
pd3=t3;
p3=t4;
yaw2=t5;
yaw3=t6;
pd1=t7;
p1=t8;
yaw1=t9;

pd4=t10;
p4=t11;
yaw4=t12;

pd5=t13;
p5=t14;
yaw5=t15;
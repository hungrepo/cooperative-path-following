m=50;
N=(size(p2,1)-1)/m;
t1=zeros(N,2);
t2=zeros(N,2);
t3=zeros(N,2);
t4=zeros(N,2);
t5=zeros(N,2);
t6=zeros(N,2);
t7=zeros(N,2);
t8=zeros(N,2);
t9=zeros(N,2);

for i=0:N-1;
t1(i+1,:)=pd2(m*i+m,:);
t2(i+1,:)=p2(m*i+m,:);
t3(i+1,:)=pd3(m*i+m,:);
t4(i+1,:)=p3(m*i+m,:);
t5(i+1,:)=yaw2(m*i+m,:);
t6(i+1,:)=yaw3(m*i+m,:);
t7(i+1,:)=pd1(m*i+m,:);
t8(i+1,:)=p1(m*i+m,:); 
t9(i+1,:)=yaw1(m*i+m,:); 

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


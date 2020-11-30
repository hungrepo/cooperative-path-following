load sine_x02_mpc_c.mat
p1=p6;
pd1=pd6;
yaw1=yaw6;
rd1=rd6;
vcxy_hat1=Vcxy_hat6;
ug1=ug6;
x1=x0;

load sine_x02_ly_c.mat 
p2=p6;
pd2=pd6;
yaw2=yaw6;
rd2=rd6;
vcxy_hat2=Vcxy_hat6;
ug2=ug6;
x2=x0;
save('sin_x02_c.mat','t','p1', 'pd1','yaw1','rd1','vcxy_hat1','ug1','x1','p2', 'pd2','yaw2','rd2','vcxy_hat2','ug2','x2');

gamma=0:1:100;
a=20;
omega=0.05;
h_g=sqrt(1+a^2*omega^2*cos(omega*gamma).^2);
cg=a*omega^2*sin(omega*gamma)./(h_g.^3);
figure(1);
plot(cg)
figure(2);
plot(h_g);


cg_max=a*omega^2;
load('Straightline_delay_2_eps_0_5e2.mat');
fig8=figure;
set(fig8,'position',[0 0 400 200])
plot(t(1:end-1),log(Vc),'r'); hold on
load('Straightline_delay_2_eps_0_5e33.mat');
plot(t(1:end-1),log(Vc),'k');  hold on;
load('Straightline_delay_2_eps_0_5e4.mat');
plot(t(1:end-1),log(Vc),'b');
%legend('$\epsilon=0.1$,$\epsilon=0.01$,$\epsilon=0.001$', 'Interpreter','latex')
 
legend('\epsilon=0.1','\epsilon=0.01','\epsilon=0.001')
 xlabel('t[ \rm second]','Interpreter','latex');
title('$\log(V_{\rm c})$','Interpreter','latex'); 
num_comm=sum(TR_1);



load load('Circle_delay_2_eps_01.mat')
fig8=figure;
set(fig8,'position',[0 0 400 200])
plot(t(1:end-1),log(Vc),'r'); hold on
load load('Circle_delay_2_eps_001.mat')
plot(t(1:end-1),log(Vc),'k');  
load load('Circle_delay_2_eps_0001.mat')
plot(t(1:end-1),log(Vc),'b');
%legend('$\epsilon=0.1$,$\epsilon=0.01$,$\epsilon=0.001$', 'Interpreter','latex')
 
legend('\epsilon=0.1','\epsilon=0.01','\epsilon=0.001')
 xlabel('t[ \rm second]','Interpreter','latex');
title('$\log(V_{\rm c})$','Interpreter','latex'); 
num_comm=sum(TR_1);


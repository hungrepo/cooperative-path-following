% close all;
% clear all;
% load Data_StrategyII_Constraint_event.mat

AUV_COL1= [0 0 1];             % black
AUV_COL2= [1 0 0.1];           % red 
AUV_COL3= 'k';         % yellow

pd1=x_path1(:,1:2);
pd2=x_path2(:,1:2);
pd3=x_path3(:,1:2);
p1=x_robot1(:,1:2);
p2=x_robot2(:,1:2);
p3=x_robot3(:,1:2);
yaw1=x_robot1(:,3)*180/pi;
yaw2=x_robot2(:,3)*180/pi;
yaw3=x_robot3(:,3)*180/pi;
n=length(t);
w=0.5;
%%  Plot mission
fig1=figure(1);
set(fig1,'position',[0 0 400 350])
% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 6 3];      

     plot(pd1(:,2),pd1(:,1),'-','LineWidth',0.2,'Color',AUV_COL1);
    hold on;
    plot(pd2(:,2),pd2(:,1),'-','LineWidth',0.2,'Color',AUV_COL2);
    plot(pd3(:,2),pd3(:,1),'-','LineWidth',0.2,'Color',AUV_COL3);
%     legend('Vehicle1','Vehicle2','Vehicle3');
%     legend('show')
%     legend('Location','northwest');legend('boxoff');
    plot(p1(:,2),p1(:,1),'-.','LineWidth',w,'Color',AUV_COL1);
    plot(p2(:,2),p2(:,1),'-.','LineWidth',w,'Color',AUV_COL2);
    plot(p3(:,2),p3(:,1),'-.','LineWidth',w,'Color',AUV_COL3);

   
%      set(gca,'Color',[0.98 0.98 .98]);
     title('Trajectories of the vehicles','Interpreter','latex');
     xlabel('y[m]','Interpreter','latex');
     ylabel('x[m]','Interpreter','latex');
     resample_3vehicles;
     animation_3vehicles_LMH;    % animation_3vehicles_LMH; for circle
     axis equal;
%     axis([-40 130 -40 60]);

%% Plot path following error

 fig2=figure(2);
  set(fig2,'position',[0 0 560 350])
 % plot ex
 subplot(3,1,1);
  title('Path following errors','Interpreter','latex');
 plot(t(1:n-1),e_pf1(:,1),'LineWidth',w,'Color',AUV_COL1);
 hold on
 plot(t(1:n-1),e_pf2(:,1),'LineWidth',w,'Color',AUV_COL2);
 plot(t(1:n-1),e_pf3(:,1),'LineWidth',w,'Color',AUV_COL3);
 ex_max=max([e_pf1(:,1);e_pf2(:,1);e_pf3(:,1)]);
 ex_min=min([e_pf1(:,1);e_pf2(:,1);e_pf3(:,1)]);
 ylabel('$$e_{x}^{[i]}[\rm m]$$','Interpreter','latex'); 
%  set(get(gca,'ylabel'),'rotation',0)

 xlabel('t[ \rm second]','Interpreter','latex');
 tmin=t(1)-1; tmax=t(end)+1;
 limit=[tmin tmax -inf inf];
 axis(limit);
 set(gca,'YTick',[round(ex_min),0,round(ex_max)+1]);
grid on;
 
% plot ey
 subplot(3,1,2);
 plot(t(1:n-1),e_pf1(:,2),'LineWidth',w,'Color',AUV_COL1);
 hold on
 plot(t(1:n-1),e_pf2(:,2),'LineWidth',w,'Color',AUV_COL2);
 plot(t(1:n-1),e_pf3(:,2),'LineWidth',w,'Color',AUV_COL3);
 ey_max=max([e_pf1(:,2);e_pf2(:,2);e_pf3(:,2)]);
 ey_min=min([e_pf1(:,2);e_pf2(:,2);e_pf3(:,2)]);
tmin=t(1)-1; tmax=t(end)+1;
limit=[tmin tmax round(ey_min) round(ey_max)];
axis(limit);
 set(gca,'YTick',[round(ey_min),0,round(ey_max)]);
grid; 
ylabel('$e_{y}^{[i]}[\rm m]$','Interpreter','latex');
% set(get(gca,'ylabel'),'rotation',0)
xlabel('t$[ \rm second]$','Interpreter','latex');
% plot psi_e
 subplot(3,1,3);
 plot(t(1:n-1),e_pf1(:,3)*pi/180,'Color',AUV_COL1);
 hold on
 plot(t(1:n-1),e_pf2(:,3),'LineWidth',w,'Color',AUV_COL2);
 plot(t(1:n-1),e_pf3(:,3),'LineWidth',w,'Color',AUV_COL3);
 epsi_max=max([e_pf1(:,3);e_pf2(:,3);e_pf3(:,3)]);
 epsi_min=min([e_pf1(:,3);e_pf2(:,3);e_pf3(:,3)]);
limit=[tmin tmax -2.5 1];
axis(limit);
 set(gca,'YTick',[-2.5,0,1]);
ylabel('$$\psi_{e}^{[i]}$$[\rm rad]','Interpreter','latex');
% set(get(gca,'ylabel'),'rotation',0)
xlabel('t[ \rm second]','Interpreter','latex');
grid;
%% Plot inputs
  fig3=figure(3);
 set(fig3,'position',[0 0 560 300]);
   

  % Plot speed in inetial frame
  subplot(2,1,1);
    umax=1; umin=0;
  u_max=umax*ones(1,n-1);
  u_min=umin*ones(1,n-1);
  
   
  plot(t(1:n-1),u_max,'g--');  hold on; plot(t(1:n-1),u_min,'g--');
  plot(t(1:n-1),u_robot1(:,1),'LineWidth',w,'Color',AUV_COL1);

  plot(t(1:n-1),u_robot2(:,1),'LineWidth',w,'Color',AUV_COL2);
  plot(t(1:n-1),u_robot3(:,1),'LineWidth',w,'Color',AUV_COL3);
  
  title('Speed of the vehicles','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');
  ylabel('$u^{[i]}[\rm{m/s}]$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)

  
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -0.1 1.1];
  axis(limit);
  grid on
%   set(gca,'YTick',[round(ey_min),0,round(ey_max)]);

  % Plot heading rate
  subplot(2,1,2);
  rmax=.1; rmin=-0.1;
  r_max=rmax*ones(1,n-1);
  r_min=rmin*ones(1,n-1);
  plot(t(1:n-1),r_max,'g--');hold on; plot(t(1:n-1),r_min,'g--');
  
  plot(t(1:n-1),u_robot1(:,2),'LineWidth',w,'Color',AUV_COL1);

  plot(t(1:n-1),u_robot2(:,2),'LineWidth',w,'Color',AUV_COL2);
  plot(t(1:n-1),u_robot3(:,2),'LineWidth',w,'Color',AUV_COL3);
  
  title('Heading rate of the vehicles','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');
  ylabel('$r^{[i]}[\rm{rad/s}]$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)
  

  
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -0.11 0.11];
  axis(limit);
  grid on
  %% Plot of coordination
  fig4=figure(4);
 set(fig4,'position',[0 0 560 300])
  % Plot of gamma
  subplot(2,1,1);
  plot(t(1:n-1),e_pf1(:,4),'LineWidth',w,'Color',AUV_COL1);
  hold on
  plot(t(1:n-1),e_pf2(:,4),'LineWidth',w,'Color',AUV_COL2);
  plot(t(1:n-1),e_pf3(:,4),'LineWidth',w,'Color',AUV_COL3);

  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -inf inf];
  axis(limit);
  grid on
  
  title('Evolution of the path parameters','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');
  ylabel('$$\gamma^{[i]}[{\rm m}]$$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)
  % Plot speed of virtual reference
  subplot(2,1,2);
%   vmax=.85; vmin=-0.85;
    vmax=0.01; vmin=-0.01;

  v_max=vmax*ones(1,n-1);
  v_min=vmin*ones(1,n-1);
  
  plot(t(1:n-1),v_max,'g--'); hold on; plot(t(1:n-1),v_min,'g--');

  plot(t(1:n-1),u_mpc1(2,:),'Color',AUV_COL1);

  plot(t(1:n-1),u_mpc2(2,:),'LineWidth',w,'Color',AUV_COL2);
  plot(t(1:n-1),u_mpc3(2,:),'LineWidth',w,'Color',AUV_COL3);
  
 
  
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -1 1];
  axis(limit);
  set(gca,'YTick',[-0.85,0,0.5,0.85]);
  grid on
  title('Speed of the "virtual references"','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');
  ylabel('$$v^{[i]}[\rm{m/s}]$$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)

  % Plot communication 
  fig6=figure(6);
  set(fig6,'position',[0 0 560 500]);
  subplot(4,1,1);
  idx=find(Com1==1);
  plot(t(idx),1*Com1(idx),'*','Color',AUV_COL1); hold on;
  idx=find(Com2==1);
  plot(t(idx),0.7*Com2(idx),'*','Color',AUV_COL2);
  idx=find(Com3==1);
  plot(t(idx),0.4*Com3(idx),'*','Color',AUV_COL3);
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax 0 1.2];
  axis(limit);
  set(gca,'YTick',[0 0.4 0.7 1]);
  set(gca,'Yticklabel',[]); 
  grid on
  title('Communication signal of the vehicles','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');
  
  subplot(4,1,2);
  plot(t(2:n-1),E1(2:n-1),'Color',AUV_COL1); hold on;
  emax=0.02; emin=-0.02;
  e_max=emax*ones(1,n-1);
  e_min=emin*ones(1,n-1);
  plot(t(1:n-1),e_max,'b--'); hold on; plot(t(1:n-1),e_min,'b--');
  
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -inf inf];
  axis(limit);
  set(gca,'YTick',[emin emax]);
  grid on;
  title('Estimation error of the path parameters','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');
  ylabel('$$\tilde{\gamma}^{[1]}[\rm{m}]$$','Interpreter','latex');
  
  
  subplot(4,1,3);
  plot(t(2:n-1),E2(2:n-1),'Color',AUV_COL2); hold on;
  e_max=emax*ones(1,n-1);
  e_min=emin*ones(1,n-1);
  plot(t(1:n-1),e_max,'r--'); hold on; plot(t(1:n-1),e_min,'r--');
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -inf inf];
  axis(limit);
  set(gca,'YTick',[emin  emax]);
  grid on
  xlabel('t[second]','Interpreter','latex');
  ylabel('$$\tilde{\gamma}^{[2]}[\rm{m}]$$','Interpreter','latex');
  
  
  subplot(4,1,4);
  plot(t(2:n-1),E3(2:n-1),'Color',AUV_COL3); hold on;
  e_max=emax*ones(1,n-1);
  e_min=emin*ones(1,n-1);
  plot(t(1:n-1),e_max,'k--'); hold on; plot(t(1:n-1),e_min,'k--');
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -inf inf];
  axis(limit);
  set(gca,'YTick',[emin  emax]);
  grid on
  xlabel('t[second]','Interpreter','latex');
  ylabel('$$\tilde{\gamma}^{[3]}[\rm{m}]$$','Interpreter','latex');

close all;
% clear all;
% load Data_StrategyII_Constraint_event.mat

set(0,'defaultfigurepaperpositionmode','manual');

AUV_COL1= [0 0 0]/255;             % black
AUV_COL2= [0 0 255]/255;           % Blue 
AUV_COL3= [255 0 0]/255;         % Red
AUV_COL4= [0,128,128]/255;           % red 
AUV_COL5= [255 0 255]/255;         
pd1=x_path1(:,1:2);
pd2=x_path2(:,1:2);
pd3=x_path3(:,1:2);
pd4=x_path4(:,1:2);
pd5=x_path5(:,1:2);

p1=x_robot1(:,1:2);
p2=x_robot2(:,1:2);
p3=x_robot3(:,1:2);
p4=x_robot4(:,1:2);
p5=x_robot5(:,1:2);

yaw1=x_robot1(:,3)*180/pi;
yaw2=x_robot2(:,3)*180/pi;
yaw3=x_robot3(:,3)*180/pi;
yaw4=x_robot4(:,3)*180/pi;
yaw5=x_robot5(:,3)*180/pi;

n=length(t);
w=0.5;
%%  Plot mission
fig1=figure(1);
% set(fig1,'position',[0 0 400 350])
% fig1.RendererMode = 'manual' 
  set(fig1,'position',[0 0 500 400]);

% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperPosition = [0 0 6 3];      

    plot(pd1(140:end,2),pd1(140:end,1),'-','LineWidth',0.2,'Color',AUV_COL1);
    hold on;
    plot(pd2(194:end,2),pd2(194:end,1),'-','LineWidth',0.2,'Color',AUV_COL2);
    plot(pd3(1:end,2),pd3(1:end,1),'-','LineWidth',0.2,'Color',AUV_COL3);
    plot(pd4(194:end,2),pd4(194:end,1),'-','LineWidth',0.2,'Color',AUV_COL4);
    plot(pd5(1:end,2),pd5(1:end,1),'-','LineWidth',0.2,'Color',AUV_COL5);
%     legend('Vehicle1','Vehicle2','Vehicle3');
%     legend('show')
%     legend('Location','northwest');legend('boxoff');
%     resample_3vehicles
    plot(p1(:,2),p1(:,1),'-.','LineWidth',1,'Color',AUV_COL1);
    plot(p2(:,2),p2(:,1),'-.','LineWidth',1,'Color',AUV_COL2);
    plot(p3(:,2),p3(:,1),'-.','LineWidth',1,'Color',AUV_COL3);
    plot(p4(:,2),p4(:,1),'-.','LineWidth',1,'Color',AUV_COL4);
    plot(p5(:,2),p5(:,1),'-.','LineWidth',1,'Color',AUV_COL5);
   
%      set(gca,'Color',[0.98 0.98 .98]);
%     title('\bf Trajectories of the vehicles','Interpreter','latex');
     xlabel('y[m]','Interpreter','latex');
     ylabel('x[m]','Interpreter','latex');
     resample_5vehicles;
     animation_5vehicles_LMH;    % animation_3vehicles_LMH; for circle
  %   axis equal;
 %  axis([-40 45 -45 50]);
   axis([-40 40 -20 80]);
     set(gca,'FontSize',12);
 %   yticks([-20 0 20 40 60 80]);



%% Plot path following error

%  fig2=figure(2);
%   set(fig2,'position',[0 0 400 350]);
%  % plot ex
%  subplot(3,1,1);
%  title('\bf Path following error','Interpreter','latex');
%  plot(t(1:n-1),e_pf1(:,1),'LineWidth',w,'Color',AUV_COL1);
%  hold on
%  plot(t(1:n-1),e_pf2(:,1),'LineWidth',w,'Color',AUV_COL2);
%  plot(t(1:n-1),e_pf3(:,1),'LineWidth',w,'Color',AUV_COL3);
%  plot(t(1:n-1),e_pf4(:,1),'LineWidth',w,'Color',AUV_COL4);
%  plot(t(1:n-1),e_pf5(:,1),'LineWidth',w,'Color',AUV_COL5);
%  ex_max=max([e_pf1(:,1);e_pf2(:,1);e_pf3(:,1);e_pf4(:,1);e_pf5(:,1)]);
%  ex_min=min([e_pf1(:,1);e_pf2(:,1);e_pf3(:,1);e_pf4(:,1);e_pf5(:,1)]);
%  ylabel('$$e_{x}^{[i]}[\rm m]$$','Interpreter','latex'); 
% %  set(get(gca,'ylabel'),'rotation',0)
% 
% xlabel('Time[s]','Interpreter','latex');
%  tmin=t(1)-1; tmax=t(end)+1;
%  limit=[tmin tmax -inf inf];
%  axis(limit);
%  set(gca,'YTick',[round(ex_min),0,round(ex_max)+1]);
% grid on;
 
% plot ey
%  subplot(3,1,2);
%  plot(t(1:n-1),e_pf1(:,2),'LineWidth',w,'Color',AUV_COL1);
%  hold on
%  plot(t(1:n-1),e_pf2(:,2),'LineWidth',w,'Color',AUV_COL2);
%  plot(t(1:n-1),e_pf3(:,2),'LineWidth',w,'Color',AUV_COL3);
%  ey_max=max([e_pf1(:,2);e_pf2(:,2);e_pf3(:,2)]);
%  ey_min=min([e_pf1(:,2);e_pf2(:,2);e_pf3(:,2)]);
% tmin=t(1); tmax=t(end)+1;
% limit=[tmin tmax round(ey_min) round(ey_max)];
% axis(limit);
%  set(gca,'YTick',[round(ey_min),0,round(ey_max)]);
% grid; 
% ylabel('$e_{y}^{[i]}[\rm m]$','Interpreter','latex');
% % set(get(gca,'ylabel'),'rotation',0)
% xlabel('Time[s]','Interpreter','latex');
% % plot psi_e
%  subplot(3,1,3);
%  plot(t(1:n-1),e_pf1(:,3)*pi/180,'Color',AUV_COL1);
%  hold on
%  plot(t(1:n-1),e_pf2(:,3),'LineWidth',w,'Color',AUV_COL2);
%  plot(t(1:n-1),e_pf3(:,3),'LineWidth',w,'Color',AUV_COL3);
%  epsi_max=max([e_pf1(:,3);e_pf2(:,3);e_pf3(:,3)]);
%  epsi_min=min([e_pf1(:,3);e_pf2(:,3);e_pf3(:,3)]);
% limit=[tmin tmax -2.5 1];
% axis(limit);
%  set(gca,'YTick',[-2.5,0,1]);
% ylabel('$$\psi_{e}^{[i]}$$[\rm rad]','Interpreter','latex');
% % set(get(gca,'ylabel'),'rotation',0)
% xlabel('Time[s]','Interpreter','latex');
% grid;
% set(gca,'FontSize',12);

%% Plot inputs
  fig3=figure(3);
 set(fig3,'position',[0 0 400 300]);

  % Plot speed in inetial frame
  subplot(2,1,1);
    umax=2; umin=0.2;
  u_max=umax*ones(1,n-1);
  u_min=umin*ones(1,n-1);
  
   
  plot(t(1:n-1),u_max,'k--');  hold on; plot(t(1:n-1),u_min,'k--');
  plot(t(1:n-1),u_robot1(:,1),'LineWidth',w,'Color',AUV_COL1);

  plot(t(1:n-1),u_robot2(:,1),'LineWidth',1,'Color',AUV_COL2);
  plot(t(1:n-1),u_robot3(:,1),'LineWidth',1,'Color',AUV_COL3);
  plot(t(1:n-1),u_robot4(:,1),'LineWidth',1,'Color',AUV_COL4);
  plot(t(1:n-1),u_robot5(:,1),'LineWidth',1,'Color',AUV_COL5);
  
  title('Speed of the vehicles $u^{[i]}[\rm{m/s}]$','Interpreter','latex');
  xlabel('Time[s]','Interpreter','latex');
%  ylabel('$u^{[i]}[\rm{m/s}]$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)

  
  tmin=t(1)-1; tmax=t(end)+1;
  limit=[tmin tmax -0 2.2];
  axis(limit);
  grid on;
       set(gca,'FontSize',12);
  
%   set(gca,'YTick',[round(ey_min),0,round(ey_max)]);

  % Plot heading rate
  subplot(2,1,2);
  rmax=.2; rmin=-0.2;
  r_max=rmax*ones(1,n-1);
  r_min=rmin*ones(1,n-1);
  plot(t(1:n-1),r_max,'k--');hold on; plot(t(1:n-1),r_min,'k--');
  
  plot(t(1:n-1),u_robot1(:,2),'LineWidth',1,'Color',AUV_COL1);

  plot(t(1:n-1),u_robot2(:,2),'LineWidth',1,'Color',AUV_COL2);
  plot(t(1:n-1),u_robot3(:,2),'LineWidth',1,'Color',AUV_COL3);
  plot(t(1:n-1),u_robot4(:,2),'LineWidth',1,'Color',AUV_COL4);
  plot(t(1:n-1),u_robot5(:,2),'LineWidth',1,'Color',AUV_COL5);
  
  title('Heading rate of the vehicles $r^{[i]}[\rm{rad/s}]$','Interpreter','latex');
xlabel('Time[s]','Interpreter','latex');
  %ylabel('$r^{[i]}[\rm{rad/s}]$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)
  

  
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax -0.22 0.22];
  axis(limit);
  grid on;
  set(gca,'FontSize',12);

  %% Plot of coordination
  fig4=figure(4);
 set(fig4,'position',[0 0 400 220]);

  % Plot of gamma
  % subplot(2,1,1);
%   plot(t(1:n-1),e_pf1(:,4),'LineWidth',w,'Color',AUV_COL1);
%   hold on
%   plot(t(1:n-1),e_pf2(:,4),'LineWidth',w,'Color',AUV_COL2);
%   plot(t(1:n-1),e_pf3(:,4),'LineWidth',w,'Color',AUV_COL3);
%   plot(t(1:n-1),e_pf4(:,4),'LineWidth',w,'Color',AUV_COL4);
%   plot(t(1:n-1),e_pf5(:,4),'LineWidth',w,'Color',AUV_COL5);

  plot(t(1:n-1),e_pf1(:,4),'LineWidth',1,'Color',AUV_COL1);
  hold on
  plot(t(1:n-1),e_pf2(:,4),'LineWidth',1,'Color',AUV_COL2);
  plot(t(1:n-1),e_pf3(:,4),'LineWidth',1,'Color',AUV_COL3);
  plot(t(1:n-1),e_pf4(:,4),'LineWidth',1,'Color',AUV_COL4);
  plot(t(1:n-1),e_pf5(:,4),'LineWidth',1,'Color',AUV_COL5);
  
%  For the straightline
%     plot(t(1:n-1),e_pf1(:,4),'LineWidth',1,'Color',AUV_COL1);
%   hold on
%   plot(t(1:n-1),e_pf2(:,4)-0.1,'LineWidth',1,'Color',AUV_COL2);
%   plot(t(1:n-1),e_pf3(:,4)-0.2,'LineWidth',1,'Color',AUV_COL3);
%   plot(t(1:n-1),e_pf4(:,4)-0.1,'LineWidth',1,'Color',AUV_COL4);
%   plot(t(1:n-1),e_pf5(:,4),'LineWidth',1,'Color',AUV_COL5);


  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax -inf inf];
  axis(limit);
  grid on
  
% title('\bf Evolution of the path parameters $\gamma^{[i]}$','Interpreter','latex');
xlabel('Time[s]','Interpreter','latex');
     set(gca,'FontSize',12)

%   ylabel('$$\gamma^{[i]}[{\rm m}]$$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)
%   % Plot speed of virtual reference
%   subplot(2,1,2);
% %   vmax=.85; vmin=-0.85;
%     vmax=0.03; vmin=-0.03;
% 
%   v_max=vmax*ones(1,n-1);
%   v_min=vmin*ones(1,n-1);
%   
%   plot(t(1:n-1),v_max,'g--'); hold on; plot(t(1:n-1),v_min,'g--');
% 
%   plot(t(1:n-1),u_mpc1(2,:),'Color',AUV_COL1);
% 
%   plot(t(1:n-1),u_mpc2(2,:),'LineWidth',w,'Color',AUV_COL2);
%   plot(t(1:n-1),u_mpc3(2,:),'LineWidth',w,'Color',AUV_COL3);
%   
%  
%   
%   tmin=t(1)-1; tmax=t(end)+1;
%   limit=[tmin tmax -0.04 0.04];
%   axis(limit);
%   set(gca,'YTick',[-0.04,0,0.02,0.04]);
%   grid on
%   title('\bf Speed of the "virtual references"','Interpreter','latex');
%   xlabel('t[seconds]','Interpreter','latex');
%   ylabel('$$v^{[i]}[\rm{m/s}]$$','Interpreter','latex');
% %   set(get(gca,'ylabel'),'rotation',0)

  % Plot TR_munication 
  fig6=figure(6);
  set(fig6,'position',[0 0 400 180]);
%  subplot(4,1,1);
  idx1=find(TR_1==1);
  plot(t(idx1),1*TR_1(idx1),'+','Color',AUV_COL1,'LineWidth',1); hold on;
  idx2=find(TR_2==1);
  plot(t(idx2),0.8*TR_2(idx2),'+','Color',AUV_COL2,'LineWidth',1);
  idx3=find(TR_3==1);
  plot(t(idx3),0.6*TR_3(idx3),'+','Color',AUV_COL3,'LineWidth',1);
  idx4=find(TR_4==1);
  plot(t(idx4),0.4*TR_4(idx4),'+','Color',AUV_COL4,'LineWidth',1);
  idx5=find(TR_5==1);
  plot(t(idx5),0.2*TR_5(idx5),'+','Color',AUV_COL5,'LineWidth',1);
  tmin=t(1); tmax=t(end)+1;k
  limit=[tmin tmax 0 1.2];
  axis(limit);
  %set(gca,'YTick',[0 0.4 0.7 1]);
  set(gca,'Yticklabel',[]); 
  grid on
 % title('\bf Broadcast signals of the vehicles','Interpreter','latex');
xlabel('Time[s]','Interpreter','latex');
  set(gca,'FontSize',12);
  n=length(Gamma1);
  fig7=figure(7);
  set(fig7,'position',[0 0 400 550]);
  
  subplot(5,1,1);
  plot(t(1:n),abs(Gamma1-Gamma_hat11(1:n)),'Color',AUV_COL1,'LineWidth',1); hold on;
  plot(t(1:n),Eta,'LineWidth',1 );
  tmin=t(1); tmax=t(end)+1;k
  limit=[tmin tmax 0 .1];
  axis(limit);
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,2);
  plot(t(1:n),abs(Gamma2-Gamma_hat22(1:n)),'Color',AUV_COL2,'LineWidth',1); hold on;
  plot(t(1:n),Eta);
  tmin=t(1); tmax=t(end)+1;k
  limit=[tmin tmax 0 .1];
  axis(limit); 
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,3);
  plot(t(1:n),abs(Gamma3-Gamma_hat33(1:n)),'Color',AUV_COL3,'LineWidth',1); hold on;
  plot(t(1:n),Eta);
  tmin=t(1); tmax=t(end)+1;k
  limit=[tmin tmax 0 .1];
  axis(limit); 
   set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,4);
  plot(t(1:n),abs(Gamma4-Gamma_hat44(1:n)),'Color',AUV_COL4,'LineWidth',1); hold on;
  plot(t(1:n),Eta);
  tmin=t(1); tmax=t(end)+1;k
  limit=[tmin tmax 0 .1];
  axis(limit); 
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,5);
  plot(t(1:n),abs(Gamma5-Gamma_hat55(1:n)),'Color',AUV_COL5,'LineWidth',1); hold on
  plot(t(1:n),Eta);
  tmin=t(1); tmax=t(end)+1;k
  limit=[tmin tmax 0 .1];
  axis(limit); 
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);
xlabel('Time[s]','Interpreter','latex');
  
  
%   fig8=figure(8);
%   set(fig8,'position',[0 0 400 200]);
%     plot(t(1:n),abs(Gamma1-Gamma_hat11(1:n)),'Color',AUV_COL1,'LineWidth',1); hold on;
%   plot(t(1:n),abs(Gamma2-Gamma_hat22(1:n)),'Color',AUV_COL2,'LineWidth',1); hold on;
%   plot(t(1:n),abs(Gamma3-Gamma_hat33(1:n)),'Color',AUV_COL3,'LineWidth',1); hold on;
%   plot(t(1:n),abs(Gamma4-Gamma_hat44(1:n)),'Color',AUV_COL4,'LineWidth',1); hold on;
%   plot(t(1:n),abs(Gamma5-Gamma_hat55(1:n)),'Color',AUV_COL5,'LineWidth',1); hold on
%   plot(t(1:n),Eta);
%   set(gca, 'YTick', [0 0.1]);
%   set(gca,'FontSize',12);
% xlabel('Time[s]','Interpreter','latex');

%   
%   subplot(4,1,2);
%   plot(t(2:n-1),E1(2:n-1),'Color',AUV_COL1); hold on;
%   emax=0.01; emin=-0.01;
%   e_max=emax*ones(1,n-1);
%   e_min=emin*ones(1,n-1);
%   plot(t(1:n-1),e_max,'b--'); hold on; plot(t(1:n-1),e_min,'b--');
%   
%   tmin=t(1)-1; tmax=t(end)+1;
%   
%   
%  limit=[tmin tmax emin-0.005  emax+0.005];
%   axis(limit);
%   set(gca,'YTick',[emin  emax]);
%   grid on;
%   title('\bf Estimation error of the path parameters','Interpreter','latex');
%   xlabel('t[seconds]','Interpreter','latex');
%   ylabel('$$\tilde{\gamma}^{[1]}[\rm{m}]$$','Interpreter','latex');
%   
%   
%   subplot(4,1,3);
%   plot(t(2:n-1),E2(2:n-1),'Color',AUV_COL2); hold on;
%   e_max=emax*ones(1,n-1);
%   e_min=emin*ones(1,n-1);
%   plot(t(1:n-1),e_max,'r--'); hold on; plot(t(1:n-1),e_min,'r--');
%   tmin=t(1)-1; tmax=t(end)+1;
%  limit=[tmin tmax emin-0.005  emax+0.005];
%   axis(limit);
%   set(gca,'YTick',[emin  emax]);
%   grid on
%   xlabel('t[seconds]','Interpreter','latex');
%   ylabel('$$\tilde{\gamma}^{[2]}[\rm{m}]$$','Interpreter','latex');
%   
%   
%   subplot(4,1,4);
%   plot(t(2:n-1),E3(2:n-1),'Color',AUV_COL3); hold on;
%   e_max=emax*ones(1,n-1);
%   e_min=emin*ones(1,n-1);
%   plot(t(1:n-1),e_max,'k--'); hold on; plot(t(1:n-1),e_min,'k--');
%   tmin=t(1)-1; tmax=t(end)+1;
%  limit=[tmin tmax emin-0.005  emax+0.005];
%   axis(limit);
%   set(gca,'YTick',[emin  emax]);
%   grid on
%   xlabel('t[seconds]','Interpreter','latex');
%   ylabel('$$\tilde{\gamma}^{[3]}[\rm{m}]$$','Interpreter','latex');
%   
  %% Plot lyapunov functions
  
%     fig7=figure(7);
%     set(fig7,'position',[0 0 300 300]);
% %     subplot(1,3,1);
%     plot(t(1:n-1),V_mpc1,'Color',AUV_COL1); hold on;
%     plot(t(1:n-1),V_mpc2,'Color',AUV_COL2);
%     plot(t(1:n-1),V_mpc3,'Color',AUV_COL3);
%     xlabel('t[seconds]','Interpreter','latex');
%     title('{\bf The Lyapunov function} $V$','Interpreter','latex');
%     limit=[tmin tmax -inf inf];
%     axis(limit);
%     xlabel('t[seconds]','Interpreter','latex');
%     title('$V^{[1]}_{\rm PF}$','Interpreter','latex');

  
%    
%      subplot(1,3,2);    
%     plot(t(1:n-1),V_mpc2);
%     limit=[tmin tmax -inf inf];
%      axis(limit);
%    xlabel('t[seconds]','Interpreter','latex');
%     title('$V^{[2]}_{\rm PF}$','Interpreter','latex');
%     
%     subplot(1,3,3);
%     plot(t(1:n-1),V_mpc3);
%     limit=[tmin tmax -inf inf];
%      axis(limit);
%     xlabel('t[seconds]','Interpreter','latex');
%     title('$V^{[3]}_{\rm PF}$','Interpreter','latex');
fig9=figure;
set(fig9,'position',[0 0 400 250]);
plot(t(1:end-1),(V_mpc1),'LineWidth',1,'Color',AUV_COL1)
hold on
plot(t(1:end-1),(V_mpc2),'LineWidth',1,'Color',AUV_COL2)
plot(t(1:end-1),(V_mpc3),'LineWidth',1,'Color',AUV_COL3)
plot(t(1:end-1),(V_mpc4),'LineWidth',1,'Color',AUV_COL4)
plot(t(1:end-1),(V_mpc5),'LineWidth',1,'Color',AUV_COL5)
xlabel('Time[s]','Interpreter','latex');
% title('$\log(V_{\rm L})$','Interpreter','latex'); 
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax -inf 0.5];
  axis(limit);
    set(gca, 'YTick', [0 0.5]);
set(gca,'FontSize',12)
  
  
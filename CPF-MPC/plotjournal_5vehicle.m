
close all;
% clear all;
% load Data_StrategyII_Constraint_event.mat
n = length(t) - 1;
set(0,'defaultfigurepaperpositionmode','manual');
% 
AUV_COL1= [0 0 0]/255;             % black
AUV_COL2= [0 0 255]/255;           % Blue 
AUV_COL3= [255 0 0]/255;           % Red
AUV_COL4= [0,128,128]/255;         % red 
AUV_COL5= [255 0 255]/255;         
pd1=x_path1(1:n,1:2);
pd2=x_path2(1:n,1:2);
pd3=x_path3(1:n,1:2);
pd4=x_path4(1:n,1:2);
pd5=x_path5(1:n,1:2);

p1=x_vehicle1(1:n,1:2);
p2=x_vehicle2(1:n,1:2);
p3=x_vehicle3(1:n,1:2);
p4=x_vehicle4(1:n,1:2);
p5=x_vehicle5(1:n,1:2);

yaw1=x_vehicle1(1:n,3)*180/pi;
yaw2=x_vehicle2(1:n,3)*180/pi;
yaw3=x_vehicle3(1:n,3)*180/pi;
yaw4=x_vehicle4(1:n,3)*180/pi;
yaw5=x_vehicle5(1:n,3)*180/pi;
w=0.5;
%%  Plot mission
    fig1=figure(1);
    set(fig1,'position',[0 0 500 400]);
    
    plot(pd1(1:n,2),pd1(1:n,1),'-','LineWidth',0.2,'Color',AUV_COL1);
    hold on;
    plot(pd2(1:n,2),pd2(1:n,1),'-','LineWidth',0.2,'Color',AUV_COL2);
    plot(pd3(1:n,2),pd3(1:n,1),'-','LineWidth',0.2,'Color',AUV_COL3);
    plot(pd4(1:n,2),pd4(1:n,1),'-','LineWidth',0.2,'Color',AUV_COL4);
    plot(pd5(1:n,2),pd5(1:n,1),'-','LineWidth',0.2,'Color',AUV_COL5);

    plot(p1(1:n,2),p1(1:n,1),'-.','LineWidth',1,'Color',AUV_COL1);
    plot(p2(1:n,2),p2(1:n,1),'-.','LineWidth',1,'Color',AUV_COL2);
    plot(p3(1:n,2),p3(1:n,1),'-.','LineWidth',1,'Color',AUV_COL3);
    plot(p4(1:n,2),p4(1:n,1),'-.','LineWidth',1,'Color',AUV_COL4);
    plot(p5(1:n,2),p5(1:n,1),'-.','LineWidth',1,'Color',AUV_COL5);
   
    title('\bf Trajectories of the vehicles','Interpreter','latex');
    xlabel('y[m]','Interpreter','latex');
    ylabel('x[m]','Interpreter','latex');
    animation_5vehicles_LMH;    % animation_3vehicles_LMH; for circle
if strcmp(mission, 'triangular')
    axis([-40 40 -20 80]);
else
    axis([-40 45 -45 50]);
end 
 set(gca,'FontSize',12);
 
%% Plot path following error

%  fig2=figure(2);
%   set(fig2,'position',[0 0 400 350]);
%  % plot ex
%  subplot(3,1,1);
%  title('\bf Path following error','Interpreter','latex');
%  plot(t(1:n),e_pf1(1:n,1),'LineWidth',w,'Color',AUV_COL1);
%  hold on
%  plot(t(1:n),e_pf2(1:n,1),'LineWidth',w,'Color',AUV_COL2);
%  plot(t(1:n),e_pf3(1:n,1),'LineWidth',w,'Color',AUV_COL3);
%  plot(t(1:n),e_pf4(1:n,1),'LineWidth',w,'Color',AUV_COL4);
%  plot(t(1:n),e_pf5(1:n,1),'LineWidth',w,'Color',AUV_COL5);
%  ex_max=max([e_pf1(1:n,1);e_pf2(1:n,1);e_pf3(1:n,1);e_pf4(1:n,1);e_pf5(1:n,1)]);
%  ex_min=min([e_pf1(1:n,1);e_pf2(1:n,1);e_pf3(1:n,1);e_pf4(1:n,1);e_pf5(1:n,1)]);
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
%  plot(t(1:n),e_pf1(1:n,2),'LineWidth',w,'Color',AUV_COL1);
%  hold on
%  plot(t(1:n),e_pf2(1:n,2),'LineWidth',w,'Color',AUV_COL2);
%  plot(t(1:n),e_pf3(1:n,2),'LineWidth',w,'Color',AUV_COL3);
%  ey_max=max([e_pf1(1:n,2);e_pf2(1:n,2);e_pf3(1:n,2)]);
%  ey_min=min([e_pf1(1:n,2);e_pf2(1:n,2);e_pf3(1:n,2)]);
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
%  plot(t(1:n),e_pf1(1:n,3)*pi/180,'Color',AUV_COL1);
%  hold on
%  plot(t(1:n),e_pf2(1:n,3),'LineWidth',w,'Color',AUV_COL2);
%  plot(t(1:n),e_pf3(1:n,3),'LineWidth',w,'Color',AUV_COL3);
%  epsi_max=max([e_pf1(1:n,3);e_pf2(1:n,3);e_pf3(1:n,3)]);
%  epsi_min=min([e_pf1(1:n,3);e_pf2(1:n,3);e_pf3(1:n,3)]);
% limit=[tmin tmax -2.5 1];
% axis(limit);
%  set(gca,'YTick',[-2.5,0,1]);
% ylabel('$$\psi_{e}^{[i]}$$[\rm rad]','Interpreter','latex');
% % set(get(gca,'ylabel'),'rotation',0)
% xlabel('Time[s]','Interpreter','latex');
% grid;
% set(gca,'FontSize',12);

%% Plot vehicles' inputs
 fig2=figure(2);
 set(fig2,'position',[0 0 400 300]);

  % Plot speed in inetial frame
  subplot(2,1,1);
  umax=2; umin=0.2;
  u_max=umax*ones(1,n);
  u_min=umin*ones(1,n);
  
   
  plot(t(1:n),u_max,'k--');  hold on; plot(t(1:n),u_min,'k--');
  plot(t(1:n),u_vehicle1(1:n,1),'LineWidth',w,'Color',AUV_COL1);

  plot(t(1:n),u_vehicle2(1:n,1),'LineWidth',1,'Color',AUV_COL2);
  plot(t(1:n),u_vehicle3(1:n,1),'LineWidth',1,'Color',AUV_COL3);
  plot(t(1:n),u_vehicle4(1:n,1),'LineWidth',1,'Color',AUV_COL4);
  plot(t(1:n),u_vehicle5(1:n,1),'LineWidth',1,'Color',AUV_COL5);
  
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
  rmax=vehicle.rmax; rmin=vehicle.rmin;
  r_max=rmax*ones(1,n);
  r_min=rmin*ones(1,n);
  plot(t(1:n),r_max,'k--');hold on; plot(t(1:n),r_min,'k--');
  
  plot(t(1:n),u_vehicle1(1:n,2),'LineWidth',1,'Color',AUV_COL1);

  plot(t(1:n),u_vehicle2(1:n,2),'LineWidth',1,'Color',AUV_COL2);
  plot(t(1:n),u_vehicle3(1:n,2),'LineWidth',1,'Color',AUV_COL3);
  plot(t(1:n),u_vehicle4(1:n,2),'LineWidth',1,'Color',AUV_COL4);
  plot(t(1:n),u_vehicle5(1:n,2),'LineWidth',1,'Color',AUV_COL5);
  
  title('Heading rate of the vehicles $r^{[i]}[\rm{rad/s}]$','Interpreter','latex');
  xlabel('Time[s]','Interpreter','latex');
  %ylabel('$r^{[i]}[\rm{rad/s}]$','Interpreter','latex');
%   set(get(gca,'ylabel'),'rotation',0)
  

  
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax rmin-0.02 rmax+0.02];
  axis(limit);
  grid on;
  set(gca,'FontSize',12);

  %% Plot of coordination state (path parameter gamma)
 fig3=figure(3);
 set(fig3,'position',[0 0 400 220]);

  % Plot of gamma
  % subplot(2,1,1);
%   plot(t(1:n),e_pf1(1:n,4),'LineWidth',w,'Color',AUV_COL1);
%   hold on
%   plot(t(1:n),e_pf2(1:n,4),'LineWidth',w,'Color',AUV_COL2);
%   plot(t(1:n),e_pf3(1:n,4),'LineWidth',w,'Color',AUV_COL3);
%   plot(t(1:n),e_pf4(1:n,4),'LineWidth',w,'Color',AUV_COL4);
%   plot(t(1:n),e_pf5(1:n,4),'LineWidth',w,'Color',AUV_COL5);

%   plot(t(1:n),e_pf1(1:n,4),'LineWidth',1,'Color',AUV_COL1);
%   hold on
%   plot(t(1:n),e_pf2(1:n,4),'LineWidth',1,'Color',AUV_COL2);
%   plot(t(1:n),e_pf3(1:n,4),'LineWidth',1,'Color',AUV_COL3);
%   plot(t(1:n),e_pf4(1:n,4),'LineWidth',1,'Color',AUV_COL4);
%   plot(t(1:n),e_pf5(1:n,4),'LineWidth',1,'Color',AUV_COL5);
  
% For the straightline
  plot(t(1:n),Gamma1(1:n),'LineWidth',1,'Color',AUV_COL1);
  hold on
  plot(t(1:n),Gamma2(1:n),'LineWidth',1,'Color',AUV_COL2);
  plot(t(1:n),Gamma3(1:n),'LineWidth',1,'Color',AUV_COL3);
  plot(t(1:n),Gamma4(1:n),'LineWidth',1,'Color',AUV_COL4);
  plot(t(1:n),Gamma5(1:n),'LineWidth',1,'Color',AUV_COL5);
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax -inf inf];
  axis(limit);
  grid on
  
% title('\bf Evolution of the path parameters $\gamma^{[i]}$','Interpreter','latex');
  xlabel('Time[s]','Interpreter','latex');
  set(gca,'FontSize',12)

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
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax 0 1.2];
  axis(limit);
  %set(gca,'YTick',[0 0.4 0.7 1]);
  set(gca,'Yticklabel',[]); 
  grid on
 % title('\bf Broadcast signals of the vehicles','Interpreter','latex');
  xlabel('Time[s]','Interpreter','latex');
  set(gca,'FontSize',12);
  fig7=figure(7);
  set(fig7,'position',[0 0 400 550]);
  
  subplot(5,1,1);
  plot(t(1:n),abs(Gamma1_tilde(1:n)),'Color',AUV_COL1,'LineWidth',1); hold on;
  plot(t(1:n),Eta1(1:n),'LineWidth',1 );
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax 0 .1];
  axis(limit);
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,2);
  plot(t(1:n),abs(Gamma2_tilde(1:n)),'Color',AUV_COL2,'LineWidth',1); hold on;
  plot(t(1:n),Eta2(1:n));
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax 0 .1];
  axis(limit); 
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,3);
  plot(t(1:n),abs(Gamma3_tilde(1:n)),'Color',AUV_COL3,'LineWidth',1); hold on;
  plot(t(1:n),Eta3(1:n));
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax 0 .1];
  axis(limit); 
   set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,4);
  plot(t(1:n),abs(Gamma4_tilde(1:n)),'Color',AUV_COL4,'LineWidth',1); hold on;
  plot(t(1:n),Eta4(1:n));
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax 0 .1];
  axis(limit); 
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);

  subplot(5,1,5);
  plot(t(1:n),abs(Gamma5_tilde(1:n)),'Color',AUV_COL5,'LineWidth',1); hold on
  plot(t(1:n),Eta5(1:n));
  tmin=t(1); tmax=t(end)+1;
  limit=[tmin tmax 0 .1];
  axis(limit); 
  set(gca, 'YTick', [0 0.1]);
  set(gca,'FontSize',12);
  xlabel('Time[s]','Interpreter','latex');
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

if strcmp(mission, 'triangular')
    limit=[tmin tmax -inf 1.5];
    set(gca, 'YTick', [0 1.5]);
else
    limit=[tmin tmax -inf 0.5];
    set(gca, 'YTick', [0 0.5]);
end
    
axis(limit);

set(gca,'FontSize',12)
  
  
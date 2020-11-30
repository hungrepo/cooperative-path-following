%% Video
vid=0;
fig1=figure(1);
fig2=figure(2);
%colors

AUV_COL1= [0 0 0]/255;             % black
AUV_COL2= [0 0 255]/255;           % Blue 
AUV_COL3= [255 0 0]/255;         % Red
AUV_COL4= [0,128,128]/255;           % red 
AUV_COL5= [255 0 255]/255;     

% AUV_COL1= [0 0 1];             % black
% AUV_COL2= [1 0 0.1];           % red 
% AUV_COL3= 'k';         % yellow
% AUV_COL4= [1 0 0.1];           % red 
% AUV_COL5= [0 0 1];

AUV_COL11= [0.8 0.8 0];
AUV_COL22= [0.1 0.1 0];
AUV_COL33= [0.9 0.2 0];
AUV_COL44= [0.1 0.1 0];
AUV_COL55= [0.8 0.8 0];

yaw1_1=90-yaw1;
yaw2_2=90-yaw2;
yaw3_3=90-yaw3;
yaw4_4=90-yaw4;
yaw5_5=90-yaw5;
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
  figure(fig1);
  set(gcf, 'Position', get(0, 'Screensize'));
  sub1= subplot(2,2,[1,3]);


  sub2=subplot(2,2,2);
  sub3=subplot(2,2,4);

for i=1:1:size(p2,1)%data.i,
    fig1;
      
    subplot(sub1);
      hold off;
    % path black
    plot(sub1,pd1(1:i,2),pd1(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL1);
    hold on;
    plot(pd2(1:i,2),pd2(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL2);
    plot(pd3(1:i,2),pd3(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL3);
    plot(pd4(1:i,2),pd4(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL4);
    plot(pd5(1:i,2),pd5(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL5);

%     
% %     plot(pd22(1:i,2),pd22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
% %     plot(pd33(1:i,2),pd33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
%     
%     
    plot(p1(1:i,2),p1(1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL1);
    
    plot(p2(1:i,2),p2(1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL2)
    plot(p3(1:i,2),p3(1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL3);
    plot(p4(1:i,2),p4(1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL4)
    plot(p5(1:i,2),p5(1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL5);
%     
% %     plot(p22(1:i,2),p22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
% %     plot(p33(1:i,2),p33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
   hold on;
    %view(45,20)    
    % path myellow
   Scale=.3;

    GTF_Simulink_PlotAUV([p1(i,2),p1(i,1),0], [0,0,yaw1_1(i)], Scale, 0,AUV_COL1,1); 
    GTF_Simulink_PlotAUV([p2(i,2),p2(i,1),0], [0,0,yaw2_2(i)], Scale, 0,AUV_COL2,1);  
    GTF_Simulink_PlotAUV([p3(i,2),p3(i,1),0], [0,0,yaw3_3(i)], Scale, 0,AUV_COL3,1);  
    GTF_Simulink_PlotAUV([p4(i,2),p4(i,1),0], [0,0,yaw4_4(i)], Scale, 0,AUV_COL4,1);
    GTF_Simulink_PlotAUV([p5(i,2),p5(i,1),0], [0,0,yaw5_5(i)], Scale, 0,AUV_COL5,1);  
    plot(0,0,'-o','LineWidth',0.5,'Color','r');
%    circles(x,y,radius,'color','black') 

%% Circle
%      plot([0 p1(i,2)],[0 p1(i,1)],'LineWidth',0.5,'Color',AUV_COL1);
%      plot([0 p2(i,2)],[0 p2(i,1)],'LineWidth',0.5,'Color',AUV_COL2);
%                plot([0 p3(i,2)],[0 p3(i,1)],'LineWidth',0.5,'Color',AUV_COL3);
%                plot([0 p4(i,2)],[0 p4(i,1)],'LineWidth',0.5,'Color',AUV_COL4);
%                plot([0 p5(i,2)],[0 p5(i,1)],'LineWidth',0.5,'Color',AUV_COL5);

%% Straightline
  plot([p3(i,2) p3(i,2)-10],[p3(i,1) p3(i,1)-10],'LineWidth',0.5,'Color','c');
  plot([p3(i,2) p3(i,2)+10],[p3(i,1) p3(i,1)-10],'LineWidth',0.5,'Color','c');
  plot([p3(i,2)-10 p3(i,2)+10],[p3(i,1)-10 p3(i,1)-10],'LineWidth',0.5,'Color','c');


%     GTF_Simulink_PlotAUV([p22(i,2),p22(i,1),0], [0,0,yaw2_22(i)], Scale, 0,AUV_COL22,1);  
%     GTF_Simulink_PlotAUV([p33(i,2),p33(i,1),0], [0,0,yaw3_33(i)], Scale, 0,AUV_COL33,1);  




   % Scale=.25*8;
    grid on; 
    axis equal;
%     axis([-15 15 -5 25 ]);
% axis square;

%    axis([-50  50 -50  50]);

  




   % Scale=.25*8;
%     grid on; 
%      axis equal;
%     axis([-15 15 -5 25 ])
%     axis([-50   200 -50   200]);

    
   % hold off;
  sub1.Position=[0.05 0.1 0.5 0.8];
   xlabel('X[m]','Interpreter','latex');
 ylabel('Y[m]','Interpreter','latex');

  title('Vehicle trajectories');

  subplot(sub2);          
  title('Evolution of the path parameters');
  plot(t26(1:i),t16(1:i),'LineWidth',1,'Color',AUV_COL1);
  hold on;
  plot(t26(1:i),t17(1:i)-0.1,'LineWidth',1,'Color',AUV_COL2);
  plot(t26(1:i),t18(1:i)-0.2,'LineWidth',1,'Color',AUV_COL3);
  plot(t26(1:i),t19(1:i)-0.1,'LineWidth',1,'Color',AUV_COL4);
  plot(t26(1:i),t20(1:i),'LineWidth',1,'Color',AUV_COL5);

  
  
%   plot(t26(1:i),t16(1:i),'LineWidth',1,'Color',AUV_COL1);
%   hold on;
%   plot(t26(1:i),t17(1:i),'LineWidth',1,'Color',AUV_COL2);
%   plot(t26(1:i),t18(1:i),'LineWidth',1,'Color',AUV_COL3);
%   plot(t26(1:i),t19(1:i),'LineWidth',1,'Color',AUV_COL4);
%   plot(t26(1:i),t20(1:i),'LineWidth',1,'Color',AUV_COL5);


%   tmin=t(1); tmax=t(end)+1;
%   limit=[tmin tmax -inf inf];
%   axis(limit);
%   grid on
  
% title('\bf Evolution of the path parameters $\gamma^{[i]}$','Interpreter','latex');
  xlabel('t[second]','Interpreter','latex');

    
  
  subplot(sub3);  
  tmin=t(1); tmax=t(i*m+p)+1;
  title('Communication Signals');
  limit=[tmin tmax 0 1.2];
  set(gca,'ytick',[])
  axis(limit);
  idx=find(TR_1(1:i*m+p)==1);
  plot(t(idx),1*TR_1(idx),'+','Color',AUV_COL1,'LineWidth',1); hold on;
  idx=find(TR_2(1:i*m+p)==1);
  plot(t(idx),0.8*TR_2(idx),'+','Color',AUV_COL2,'LineWidth',1);
  idx=find(TR_3(1:i*m+p)==1);
  plot(t(idx),0.6*TR_3(idx),'+','Color',AUV_COL3,'LineWidth',1);
  idx=find(TR_4(1:1:i*m+p)==1);
  plot(t(idx),0.4*TR_4(idx),'+','Color',AUV_COL4,'LineWidth',1);
  idx=find(TR_5(1:1:i*m+p)==1);
  plot(t(idx),0.2*TR_5(idx),'+','Color',AUV_COL5,'LineWidth',1);
  
  
     if(vid==1)
        drawnow;
        currFrame = getframe(fig);
        writeVideo(animation, currFrame);
    else
        pause(0);
        drawnow;
    end

end

if(vid==1)
    close(animation);
%     close(fig);
 

end
%% Video
vid=0;
fig1=figure(1);
fig2=figure(2);
%colors
 
AUV_COL6= [0 0 0];
 

AUV_COL11= [0.8 0.8 0];
AUV_COL22= [0.1 0.1 0];
AUV_COL33= [0.9 0.2 0];
 
yaw6_6=90-yaw6;
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
for i=1:1:size(pd6,1)%data.i,
    
    hold off
    figure(fig1);
    % path black
   
    plot(pd6(1:i,2),pd6(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
    hold on; 
%     plot(pd22(1:i,2),pd22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(pd33(1:i,2),pd33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
    
   
    plot(p6(1:i,2),p6(1:i,1),'.-','LineWidth',0.5,'Color',[10,10,0]/255);
   
%     plot(p22(1:i,2),p22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(p33(1:i,2),p33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
    hold on;
    %view(45,20)    
    % path myellow
   Scale=.2;
 
    GTF_Simulink_PlotAUV([p6(i,2),p6(i,1),1], [0,0,yaw6_6(i)], Scale, 0,AUV_COL6,1);  
   
%     GTF_Simulink_PlotAUV([p22(i,2),p22(i,1),0], [0,0,yaw2_22(i)], Scale, 0,AUV_COL22,1);  
%     GTF_Simulink_PlotAUV([p33(i,2),p33(i,1),0], [0,0,yaw3_33(i)], Scale, 0,AUV_COL33,1);  

    Scale=.25*8;
    grid on; %axis equal;
%     axis([-15 15 -5 25 ])
   
    title('Animation of Coordinated PF')
    xlabel('Y[m]')
    ylabel('X[m]')
    axis equal;
    if(vid==1)
        drawnow;
        currFrame = getframe(fig);
        writeVideo(animation, currFrame);
    else
        pause(0);
        drawnow;
    end
    hold off;
    
%     figure(fig2)
%     subplot(3,1,1);hold on;
%     if COM1.Data(i)~=Com1_old
%         stem(COM1.Time(i),1,'Color',[0.8,0.9,0]);
%         title('transmistion signal on vehicle 1');
%         Com1_old=COM1.Data(i);
%     end
%     subplot(3,1,2);hold on;
%     if COM2.Data(i)~=Com2_old
%         stem(COM2.Time(i),1,'k')
%         title('transmistion signal on vehicle 2');
%         Com2_old=COM2.Data(i);
%     end
%     subplot(3,1,3);hold on;
%     if COM3.Data(i)~=Com3_old
%         stem(COM3.Time(i),1,'r')
%         title('transmistion signal on vehicle 3')
%         Com3_old=COM3.Data(i);
%     end 
end

if(vid==1)
    close(animation);
%     close(fig);
end
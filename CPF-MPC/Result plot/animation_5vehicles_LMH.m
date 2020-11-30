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
for i=1:1:size(p2,1)%data.i,
    
     hold off
    figure(fig1);
    % path black
%     plot(pd1(1:i,2),pd1(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL1);
%     hold on;
%     plot(pd2(1:i,2),pd2(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL2);
%     plot(pd3(1:i,2),pd3(1:i,1),'-','LineWidth',0.2,'Color',AUV_COL3);
%     
% %     plot(pd22(1:i,2),pd22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
% %     plot(pd33(1:i,2),pd33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
%     
%     
%     plot(p1(1:i,1),p1(1:i,2),'.-','LineWidth',0.5,'Color',[200,200,0]/255);
%     
%     plot(p2(1:i,1),p2(1:i,2),'.-','LineWidth',0.5,'Color',[10,10,0]/255)
%     plot(p3(1:i,1),p3(1:i,2),'.-','LineWidth',0.5,'Color',[200,10,10]/255);
%     
% %     plot(p22(1:i,2),p22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
% %     plot(p33(1:i,2),p33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
%   hold on;
    %view(45,20)    
    % path myellow
   Scale=.35;

    GTF_Simulink_PlotAUV([p1(i,2),p1(i,1),0], [0,0,yaw1_1(i)], Scale, 0,AUV_COL1,1); 
    GTF_Simulink_PlotAUV([p2(i,2),p2(i,1),0], [0,0,yaw2_2(i)], Scale, 0,AUV_COL2,1);  
    GTF_Simulink_PlotAUV([p3(i,2),p3(i,1),0], [0,0,yaw3_3(i)], Scale, 0,AUV_COL3,1);  
    GTF_Simulink_PlotAUV([p4(i,2),p4(i,1),0], [0,0,yaw4_4(i)], Scale, 0,AUV_COL4,1);
    GTF_Simulink_PlotAUV([p5(i,2),p5(i,1),0], [0,0,yaw5_5(i)], Scale, 0,AUV_COL5,1);  
    
    
%     GTF_Simulink_PlotAUV([p22(i,2),p22(i,1),0], [0,0,yaw2_22(i)], Scale, 0,AUV_COL22,1);  
%     GTF_Simulink_PlotAUV([p33(i,2),p33(i,1),0], [0,0,yaw3_33(i)], Scale, 0,AUV_COL33,1);  

    Scale=.25*8;
%     grid on; %axis equal;
%     axis([-15 15 -5 25 ])
%     axis([-50   200 -50   200]);

    
    if(vid==1)
        drawnow;
        currFrame = getframe(fig);
        writeVideo(animation, currFrame);
    else
        pause(0);
        drawnow;
    end
%    hold off;
    
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
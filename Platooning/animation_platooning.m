close all;

vid=0;

v = VideoWriter('video_1vehicle');
v.FrameRate=5;
open(v);
d=20;

L=length(t);
figure('units','normalized','outerposition',[0 0 1 1]);
Ts=0.01;

Scale=.5;
l=150;
R=50;
s1=l; 
s2=l+pi*R;
s3=l+pi*R+l;
s4=2*l+2*pi*R;

%plot boundary
a=5;
Xd=0;
Yd=0;
Rin=R-a;
Rout=R+a;
for s=0:1:s4
    if s<l
       Xd(end+1)=s;
       Yd(end+1)=0;
       Xdin=Xd;
       Ydin=Yd+a;
       Xdout=Xd;
       Ydout=Yd-a;
    elseif (s1 <= s)&& (s <=s2 )
       psi=(s-s1)/R;
       Xd(end+1)=l+R*sin(psi);
       Yd(end+1)=R-R*cos(psi);
       Xdin(end+1)=l+Rin*sin(psi);
       Ydin(end+1)=a+Rin-Rin*cos(psi);
       Xdout(end+1)=l+Rout*sin(psi);
       Ydout(end+1)=-a+ Rout-Rout*cos(psi);
    elseif (s2 <= s)&& (s <=s3 )
       psi=pi;
       Xd(end+1)=s3-s;
       Yd(end+1)=2*R;
       
       Xdin(end+1)=s3-s;
       Ydin(end+1)=2*Rin+a;
       Xdout(end+1)=s3-s;
       Ydout(end+1)=2*Rout-a;
    else
       psi=pi+(s-s3)/R;
       Xd(end+1)=-R*sin((s-s3)/R);
       Yd(end+1)=R+R*cos((s-s3)/R);
       
       Xdin(end+1)=-Rin*sin((s-s3)/R);
       Ydin(end+1)=Rin+Rin*cos((s-s3)/R)+a;
       Xdout(end+1)=-Rout*sin((s-s3)/R);
       Ydout(end+1)=Rout+Rout*cos((s-s3)/R)-a;
       
    end
end
hold on;
speed=1;
k=1;
while k<=L-10%    t=k*Ts;

hold off;
plot(Xd,Yd,'.', 'color',[0.9 0.9 0.9]);
hold on;
plot(Xdin,Ydin, 'color',[0.8 0.8 1], 'Linewidth',2);
plot(Xdout,Ydout,'color',[0.8 0.8 1], 'Linewidth',2);

% GTF_Simulink_PlotAUV(posv',...
%         [-Phie Thetae Psie(i)]*(180/pi),...
%         AUVscale,0,[0.9 .7 0]);

for i=1:N

    s(i)=x{i,1}(:,k)+(i-1)*d;
    
    if s(i)>s4;
        s(i)=s(i)-s4;
    end
    if s(i)<l
       psi=0;
       xd(i)=s(i);
       yd(i)=0;
       GTF_Simulink_PlotAUV([xd(i),yd(i),0], [0,0,psi]*180/pi, Scale*1.2, 0, [1 0 0]);
    elseif (s1 <= s(i))&& (s(i) <=s2 )
       psi=(s(i)-s1)/R;
       xd(i)=l+R*sin(psi);
       yd(i)=R-R*cos(psi);
       GTF_Simulink_PlotAUV([xd(i),yd(i),0], [0,0,psi]*180/pi, Scale*1.2, 0, [1 0 0]);
     elseif (s2 <= s(i))&& (s(i) <=s3 )
       psi=pi;
       xd(i)=s3-s(i);
       yd(i)=2*R;
       GTF_Simulink_PlotAUV([xd(i),yd(i),0], [0,0,psi]*180/pi, Scale*1.2, 0, [1 0 0]); 
    else
       psi=pi+(s(i)-s3)/R;
       xd(i)=-R*sin((s(i)-s3)/R);
       yd(i)=R+R*cos((s(i)-s3)/R);
       GTF_Simulink_PlotAUV([xd(i),yd(i),0], [0,0,psi]*180/pi, Scale*1.2, 0, [1 0 0]);
    end
    
     
    
    
end

for i=1:N
    %******************PLOT THE RANGE (DISCRETE-TIME)**********************
    
    if Com_Sig{i,1}(:,k)==1
         Vi=[xd(i);yd(i)];
         for j=1:N
             if Adja(i,j)==1
                Vj=[xd(j);yd(j)];
                tem11=[Vi,Vj];
                plot(tem11(1,:),tem11(2,:),'b-.','LineWidth',3)  ;
             end
         end
%         angc=Vi(1:2,1)-Vj(1:2);
%         r=norm(angc);
%         angc=atan2(angc(2),angc(1));
%         time=(angc-0.2):0.1:(angc+0.2);
%         for m=0.2:0.2:r,
%             z01=m/r*(-Vj(3)+Vi(3))+Vj(3);
%             fy=@(time) m*cos(time)+Vj(1);
%             fx=@(time) m*sin(time)+Vj(2);
%             Range=plot3(fy(time),fx(time),z01+fx(time)*0,'--c','linewidth',1.5);
%             set(Range,'linewidth',2,'linestyle','--','color','m');
%         end       
    end
end

if k<1200
xdmin=min(xd); xdmax=max(xd);
ydmin=min(yd); ydmax=max(yd);
%axis normal;
xmin=-10+xdmin-0.005*k;
xmax= xdmax+10+0.00001*k;
ymin=-10+ydmin-0.01*k;
ymax= ydmax+10+0.01*k;

else
xmin=xmin-0.001*k;
xmax=xmax-0.0005*k;
ymin=ymin-0.001*k;
ymax=ymax+0.001*k;
% axis([-57 210 -7 107]);
end

if xmin<-57
    xmin=-57;
end
if xmax>210
    xmax=210;
end
if ymin<-7
    ymin=-7;
end
if ymax>107
    ymax=107;
end
axis([xmin xmax ymin ymax]);
axis([-57 210 -7 107]);


speed=round(speed+.005*k);
if speed>5
    speed=5;
end
    speed=5;
    k=k+speed;
% Record video
   if(vid==1)
        drawnow;
        currFrame = getframe(gcf);
        writeVideo(v, currFrame);
   else
        pause(0);
        drawnow;
   end 

end
if(vid==1)
    close(v);
%     close(fig);
end
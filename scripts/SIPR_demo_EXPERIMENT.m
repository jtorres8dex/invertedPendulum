% clear all; clc;

fprintf( 'NatNet Polling Sample Start\n' )

% create an instance of the natnet client class
fprintf( 'Creating natnet class object\n' )
natnetclient = natnet;

% connect the client to the server (multicast over local loopback) -
% modify for your network
fprintf( 'Connecting to the server\n' )
natnetclient.HostIP = '192.168.77.125';%'127.0.0.1';
natnetclient.ClientIP = '192.168.77.248';%'127.0.0.1';
natnetclient.ConnectionType = 'Multicast';
natnetclient.connect;
if ( natnetclient.IsConnected == 0 )
    fprintf( 'Client failed to connect\n' )
    fprintf( '\tMake sure the host is connected to the network\n' )
    fprintf( '\tand that the host and client IP addresses are correct\n\n' )
    return
end

% get the asset descriptions for the asset names
model = natnetclient.getModelDescription;
if ( model.RigidBodyCount < 1 )
    return
end


if exist('s')==1
    fclose(s);
end


s=serial('COM4','BaudRate',115200);
fopen(s);
display('Arduino connected')

mydata=[];
control1=1;
control2=1;
k1=0.01;
k2=0.01;
tic;
state=0;
del=2;


signal=32;
%small
p1=    0.0519;
p2=    0.0075;
p3=   -1.1341;
p4=    0.0016;
p5=   10.5250;
p6=   -0.0381;

%75p
p1= 0.0805;
p2= -0.0058;
p3=-1.3934;
p4=0.1139;
p5= 12.0199;
p6= -0.4435;

%% start initialization
disp('NatNet Polling Sample End' )
        f1=sprintf('%.2d%',0);
        f2=sprintf('%.2d%',0);
        f3=sprintf('%.2d%',0);
        f4=sprintf('%.2d%',0);
        f5=sprintf('%.2d%',0);
        f6=sprintf('%.2d%',0);
        
        
        fprintf(s,'%s*',f1);
        fprintf(s,'%s#',f2);
        fprintf(s,'%s&',f3);
        fprintf(s,'%s@',f4);
        fprintf(s,'%s(',f5);
        fprintf(s,'%s\n',f6);
        
        pause(2)
        
        disp('NatNet Polling Sample End' )
        f2=sprintf('%.2d%',29);
        f1=sprintf('%.2d%',29);
        f3=sprintf('%.2d%',0);
        f4=sprintf('%.2d%',0);
        f5=sprintf('%.2d%',0);
        f6=sprintf('%.2d%',0);
        
        
        fprintf(s,'%s*',f1);
        fprintf(s,'%s#',f2);
        fprintf(s,'%s&',f3);
        fprintf(s,'%s@',f4);
        fprintf(s,'%s(',f5);
        fprintf(s,'%s\n',f6);
        
        pause(1)
        
        disp('NatNet Polling Sample End' )
        f1=sprintf('%.2d%',0);
        f2=sprintf('%.2d%',0);
        f3=sprintf('%.2d%',0);
        f4=sprintf('%.2d%',0);
        f5=sprintf('%.2d%',0);
        f6=sprintf('%.2d%',0);
        
        
        fprintf(s,'%s*',f1);
        fprintf(s,'%s#',f2);
        fprintf(s,'%s&',f3);
        fprintf(s,'%s@',f4);
        fprintf(s,'%s(',f5);
        fprintf(s,'%s\n',f6);
        
        pause(2)
%% end of initialization

q0=0;
theta0=180;

dq0=0;
dtheta0=0;
angle_total_old = 0;
rot_total_old = -180;

    dq_vect_1=[];
    dtheta_vect_=[];
    dtheta_vect_Total=[];
    q_new=[0];
    theta_new=[0];
    theta_total_new=[0];
    
myData=[0,0,-180,0,0,0,0];

d = fdesign.lowpass('Fp,Fst,Ap,Ast',5,23,1,25,100); %%working perfectly for velocity
d1 = fdesign.lowpass('Fp,Fst,Ap,Ast',1,45,5,2,100);
d2 = fdesign.lowpass('Fp,Fst,Ap,Ast',5,25,1,25,100); %accel

    Hd = design(d,'equiripple');
    Hd1 = design(d1,'equiripple');
    Hd2 = design(d2,'equiripple');
tic
tau_timit = pi*1.2;

for j=1:500

    %% get marker positions

        data = natnetclient.getFrame; % method to get current frame
        
          
        tip_num=4; s2nd_num=1; s3rd_num=7; base_num=11;
        %%tip
        x11=data.LabeledMarker(tip_num).x * 1000;
        z11=data.LabeledMarker(tip_num).z * 1000;
        y11=data.LabeledMarker(tip_num).y * 1000;
        
        %%2nd
        x21=data.LabeledMarker(s2nd_num).x * 1000;
        z21=data.LabeledMarker(s2nd_num).z * 1000;
        y21=data.LabeledMarker(s2nd_num).y * 1000;
        
%         %%3rd
        x31=data.LabeledMarker(s3rd_num).x * 1000;
        z31=data.LabeledMarker(s3rd_num).z * 1000;
        y31=data.LabeledMarker(s3rd_num).y * 1000;
        

        
        tip=[x11,y11];
        s2nd=[x21,y21];
        s3rd=[x31,y31];
        
        %%%experimenting
        tip=[round(x11*100)/100,round(y11*100)/100];
        s2nd=[round(x21*100)/100,round(y21*100)/100];
        s3rd=[round(x31*100)/100,round(y31*100)/100];
        
        
        %             base=[x41,y41];
        
%         data_center=[tip;s2nd;s3rd];
        
        %%refPoint
        tip_ref=5; s2nd_ref=2; s3rd_ref=8; base_ref=12;
        %%tip
        x12=data.LabeledMarker(tip_ref).x * 1000;
        z12=data.LabeledMarker(tip_ref).z * 1000;
        y12=data.LabeledMarker(tip_ref).y * 1000;
        
        %%2nd
        x22=data.LabeledMarker(s2nd_ref).x * 1000;
        z22=data.LabeledMarker(s2nd_ref).z * 1000;
        y22=data.LabeledMarker(s2nd_ref).y * 1000;
        
%         %%3rd
        x32=data.LabeledMarker(s3rd_ref).x * 1000;
        z32=data.LabeledMarker(s3rd_ref).z * 1000;
        y32=data.LabeledMarker(s3rd_ref).y * 1000;


%         tipr=[x12,y12];
%         s2ndr=[x22,y22];
%         s3rdr=[x32,y32];
%         
        tipr=[round(x12*100)/100,round(y12*100)/100];
        s2ndr=[round(x22*100)/100,round(y22*100)/100];
        s3rdr=[round(x32*100)/100,round(y32*100)/100];
        %             baser=[x42,y42];
        tip3=6;
%         tip_3rd = [data.LabeledMarker(tip3).x*1000, data.LabeledMarker(tip3).y*1000];
        tip_3rd = [round(data.LabeledMarker(tip3).x*1000*100)/100, round(data.LabeledMarker(tip3).y*1000*100)/100];
%         data_ref=[tipr;s2ndr;s3rdr];
       
%         angltip1=acos(dot((s2nd-s2ndr),(tip-s2nd))/(norm(s2nd-s2ndr)*norm(tip-s2nd)));
%         angletip1=90-angltip1/pi*180;
        
%         angltip2=acos(dot((s3rd-s3rdr),(s2nd-s3rd))/(norm(s3rd-s3rdr)*norm(s2nd-s3rd)));
%         angletip2=90-angltip2/pi*180;

        angltip1=acos(dot((s2nd-s2ndr),(tip-tipr))/(norm(s2nd-s2ndr)*norm(tip-tipr)));
        angletip1=angltip1/pi*180;
        
        angltip1=acos(dot((s2nd-s2ndr),(tip-tip_3rd))/(norm(s2nd-s2ndr)*norm(tip-tip_3rd)));
        b_angle=angltip1/pi*180;


        chk_90_=acos(dot((s3rd-s3rdr),(s2nd-s2ndr))/(norm(s3rdr-s3rd)*norm(s2nd-s2ndr)));
        chk_90=chk_90_/pi*180;
        
        
        rot_theta_=acos(dot((s2nd-s3rd),(s2nd-s2ndr))/(norm(s2nd-s3rd)*norm(s2nd-s2ndr)));
        rot_theta_=rot_theta_/pi*180;
        
%         rot_theta=rot_theta_;
        
        if chk_90<90
            rot_theta=rot_theta_-180;
        else
            rot_theta=180-rot_theta_;
        end
%         
% % 
        if rot_total_old>990 && rot_theta>0
            rot_theta = rot_theta + 1080;
        elseif rot_total_old<-990 && rot_theta<0
            rot_theta = rot_theta - 1080;
        elseif rot_total_old>810 && rot_theta<0
            rot_theta = rot_theta + 1080;
        elseif rot_total_old<-810 && rot_theta>0
            rot_theta = rot_theta - 1080;
        elseif rot_total_old>630 && rot_theta>0
            rot_theta = rot_theta + 720;
        elseif rot_total_old<-630 && rot_theta<0
            rot_theta = rot_theta - 720;
        elseif rot_total_old>450 && rot_theta<0
            rot_theta = rot_theta + 720;
        elseif rot_total_old<-450 && rot_theta>0
            rot_theta = rot_theta - 720;
        elseif rot_total_old>90 && rot_theta<0
            rot_theta = 360 + rot_theta;
        elseif rot_total_old<-90 && rot_theta>0
            rot_theta = rot_theta - 360;
        elseif rot_total_old>270 && rot_theta>0
            rot_theta = rot_theta + 360;
        elseif rot_total_old<-270 && rot_theta<0
            rot_theta = rot_theta - 360;
        end

        
        rot_total_old = rot_theta;   
%        
%         
        if b_angle<90
            angle_total=angletip1;
        else
            angle_total=-angletip1;
        end
% % angle_total=angletip1;
%         if angle_total_old>90 && angle_total<-90
%             angle_total = 360 + angle_total;
%         elseif angle_total_old<-90 && angle_total>90
%             angle_total = angle_total -360;
%         end
%         
%         angle_total_old = angle_total;


    %% Curvature data calculation
    
%     delT=toc;
%     theta1=angletip1;
%     theta2=angletip2;
%     theta_total = angle_total;
% %     output1 = filter(Hd1,myData(:,2)); 
%     
%     dtheta1=(theta1-theta10)/delT;
%     dtheta2=(theta2-theta20)/delT;
%     dtheta_total=(theta_total-theta_total0)/delT;
%     
%     ddtheta1=(dtheta1-dtheta10)/delT;
%     ddtheta2=(dtheta2-dtheta20)/delT;
%     ddtheta_total=(dtheta_total-dtheta_total0)/delT;
%     
%     theta10=theta1;
%     theta20=theta2;
%     theta_total0=theta_total;
%     dtheta10=dtheta1;
%     dtheta20=dtheta2;
%     dtheta_total0=dtheta_total;
%     
%     tic;
%     
% %     %%
%      theta_1=filter(Hd1,[myData(max(1,j-5):j,2);theta1]);
%      dtheta_1=(theta_1(end)-theta1_new(end))/delT;
%      
%      theta1_new=[theta1_new;theta_1(end)];
%      dtheta_vect_1=[dtheta_vect_1;dtheta_1];
%      
%      theta_2=filter(Hd1,[myData(max(1,j-5):j,3);theta2]);
%      dtheta_2=(theta_2(end)-theta2_new(end))/delT;
%      
%      theta2_new=[theta2_new;theta_2(end)];
%      dtheta_vect_2=[dtheta_vect_2;dtheta_2];
%      
%      theta_Total=filter(Hd1,[myData(max(1,j-5):j,3);theta_total]);
%      dtheta_Total=(theta_Total(end)-theta_total_new(end))/delT;
%      
%      theta_total_new=[theta_total_new;theta_Total(end)];
%      dtheta_vect_Total=[dtheta_vect_Total;dtheta_Total];
% 
% %      
%     filtered_vel_1 = filter(Hd,[myData(max(1,j-5):j,4);dtheta1]);  
%     filtered_vel_2 = filter(Hd,[myData(max(1,j-5):j,5);dtheta2]);
%     filtered_vel_total = filter(Hd,[myData(max(1,j-5):j,5);dtheta_total]);
%     filtered_accel_1 = filter(Hd2,[myData(max(1,j-5):j,6);ddtheta1]);  
%     filtered_accel_2 = filter(Hd2,[myData(max(1,j-5):j,7);ddtheta2]);
%     filtered_accel_total = filter(Hd2,[myData(max(1,j-5):j,7);ddtheta_total]);
%     
% %     theta2=0;
% %     theta_2=0;
% %     dtheta2=0;
% %     ddtheta2=0;
% %     filtered_vel_2=0;
% %     filtered_accel_2=0;

%%
    delT=double(toc);
    tic;
    q=double(angle_total);
    theta=double(rot_theta);
    %     output1 = filter(Hd1,myData(:,2));
    
    dq=(q-q0)/delT;
    dtheta=(theta-theta0)/delT;
    
    ddq=(dq-dq0)/delT;
    ddtheta=(dtheta-dtheta0)/delT;
    
    q0=q;
    theta0=theta;
    
    dq0=dq;
    dtheta0=dtheta;
    
    q_1=q;%filter(Hd1,[myData(max(1,j-5):j,2);theta1]);
%       q_1=filter(Hd1,[myData(max(1,j-5):j,2);q]);
    dq_1=(q_1(end)-q_new(max(1,j-del)))/(myData(j,1)+delT-myData(max(j-del-1,1),1));
    
    q_new=[q_new;q_1(end)];
    dq_vect_1=[dq_vect_1;dq_1];
    
%     theta_=theta;%filter(Hd1,[myData(max(1,j-5):j,3);theta2]);
    theta_=([myData(max(1,j-5):j,3);theta]);
%     theta_=filter(Hd1,[myData(max(1,j-5):j,3);theta]);
    dtheta_=(theta_(end)-theta_new(max(1,j-del)))/(myData(j,1)+delT-myData(max(j-del-1,1),1));
    
%     theta_new=lowpass([theta_new;theta_(end)],10,50);
%     dtheta_vect_=lowpass([dtheta_vect_;dtheta_],10,50);
    
    theta_new=[theta_new;theta_(end)];
    dtheta_vect_=[dtheta_vect_;dtheta_];
    
     
    ddq=(dq_1-dq_vect_1(max(1,j-del)))/(myData(j,1)+delT-myData(max(j-del-1,1),1));
    ddtheta=(dtheta_-dtheta_vect_(max(1,j-del)))/(myData(j,1)+delT-myData(max(j-del-1,1),1));
 

%%

    myData=[myData;delT+myData(length(myData(:,1)),1),q,theta,dq_1,dtheta_,ddq,ddtheta];
  
    time=myData(length(myData(:,1)),1);
    
    qq=q_new(end)/180*pi;
    th=theta_new(end)/180*pi;

    dqq=dq_1/180*pi;
    dth=dtheta_/180*pi;
    
    t=time;

%% Controller

phi = 0;
D = 0.024;
L = 0.105;
m = 0.045;
g = 9.81;
beta = 5e-3;
% k= 1.56e-3;
% beta = 2.3e-5;
k= 7.65e-3;


a0=qq;
% theta=mod(pi+y(2),2*pi)-pi;
theta=th;
da0=dqq;
dtheta=dth; 

% remedy for devision by zero
thresh = 1e-5;
if a0<thresh && a0>=0 && da0<0
    a0 = -thresh*10 ;
elseif a0<thresh && a0>=0 && da0>0
    a0 = thresh*10;
elseif a0>-thresh && a0<=0 && da0>0
    a0 = thresh*10;
elseif a0>-thresh && a0<=0 && da0<0
    a0 = -thresh*10 ;
end

% thresh2 = pi;
% if a0<-thresh2 
%     a0 = -thresh2 ;
% elseif a0>thresh2
%     a0 = thresh2;
% end

aa=[a0;theta];
daa = [da0;dtheta];

% Dynamics
% Uniform mass
D_mtx = [ (m*(72*L^2*a0 + D^2*a0^5 + 12*L^2*a0^3 - 144*L^2*sin(a0) + 72*L^2*a0*cos(a0)))/(36*a0^5), (m*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0)))/(24*a0^4);
             (m*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0)))/(24*a0^4),                                (m*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0)))/(12*a0^3)];

C_mtx = [                                                                 -(L^2*da0*m*(12*a0 - 30*sin(a0) + 18*a0*cos(a0) + a0^3 + 3*a0^2*sin(a0)))/(3*a0^6), (L^2*dtheta*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4;
         -(L^2*m*(4*da0 + a0^2*da0 + 2*a0^2*dtheta - 4*da0*cos(a0) + a0^2*da0*cos(a0) + a0^2*dtheta*cos(a0) - 4*a0*da0*sin(a0) - 3*a0*dtheta*sin(a0)))/a0^5,   -(L^2*da0*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4];



G_vect = [(2*L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^3 + (L*g*m*(sin(a0 + phi + theta) - sin(phi + theta)))/a0^2;
                                                                    -(L*g*m*(sin(phi + theta) - sin(a0 + phi + theta) + a0*cos(phi + theta)))/a0^2];



K = [k 0; 0 0];
Damp = [beta 0; 0 0];
% 
% hqq  = C_mtx*daa + G_vect + K*aa + Damp*daa;
% 
% B_tilde = D_mtx;

% % Energy shaping spong passivity
% E =(a0^2*k)/2 + (D^2*da0^2*m)/72 + (D^2*dtheta^2*m)/24 + (D^2*da0*dtheta*m)/24 + (L^2*da0^2*m)/(6*a0^2) + (2*L^2*da0^2*m)/a0^4 + (L^2*dtheta^2*m)/a0^2 + (L^2*da0*dtheta*m)/(2*a0^2) - (L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^2 - (2*L^2*da0^2*m*sin(a0/2)^2)/a0^4 - (2*L^2*da0^2*m*sin(a0))/a0^5 - (L^2*dtheta^2*m*sin(a0))/a0^3 + (2*L^2*da0*dtheta*m*sin(a0/2)^2)/a0^4 - (L^2*da0*dtheta*m*sin(a0))/a0^3;
% 
% Ec = 9.81; %target energy
% 
% u = -1.1*wP*a0 - 0.5*wD*da0 + 0.5*k3*sat((E-Ec)*dtheta,-15,15);
% 
% Tau = (hqq(1) - (B_tilde(2,1)/B_tilde(2,2))*hqq(2)) + (B_tilde(1,1) - B_tilde(2,1)*B_tilde(2,1)/B_tilde(2,2))*u;

% phi = 0;
% D = 0.01;
% L = 0.1;
% m = 0.075;
% g =9.81;
% beta = 0.1;
% k=0.3;

% % withount the damping
% E = (a0^2*k)/2 + (D^2*da0^2*m)/72 + (D^2*dtheta^2*m)/24 + (D^2*da0*dtheta*m)/24 + (L^2*da0^2*m)/(6*a0^2) + (2*L^2*da0^2*m)/a0^4 + (L^2*dtheta^2*m)/a0^2 + (L^2*da0*dtheta*m)/(2*a0^2) - (L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^2 - (2*L^2*da0^2*m*sin(a0/2)^2)/a0^4 - (2*L^2*da0^2*m*sin(a0))/a0^5 - (L^2*dtheta^2*m*sin(a0))/a0^3 + (2*L^2*da0*dtheta*m*sin(a0/2)^2)/a0^4 - (L^2*da0*dtheta*m*sin(a0))/a0^3;
% % 
% % % with the damping
% % E = (da0^2*beta)/2 + (a0^2*k)/2 + (D^2*da0^2*m)/72 + (D^2*dtheta^2*m)/24 + (D^2*da0*dtheta*m)/24 + (L^2*da0^2*m)/(6*a0^2) + (2*L^2*da0^2*m)/a0^4 + (L^2*dtheta^2*m)/a0^2 + (L^2*da0*dtheta*m)/(2*a0^2) - (L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^2 - (2*L^2*da0^2*m*sin(a0/2)^2)/a0^4 - (2*L^2*da0^2*m*sin(a0))/a0^5 - (L^2*dtheta^2*m*sin(a0))/a0^3 + (2*L^2*da0*dtheta*m*sin(a0/2)^2)/a0^4 - (L^2*da0*dtheta*m*sin(a0))/a0^3;
% 
% 
% f1 = (144*a0^5*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0)))/(m*(12096*L^4 - 15552*L^4*cos(a0)^2 + D^4*a0^8 + 3456*L^4*a0^2 + 720*L^4*a0^4 + 3456*L^4*cos(a0) - 17280*L^4*a0*sin(a0) + 144*D^2*L^2*a0^4 + 72*D^2*L^2*a0^6 + 8640*L^4*a0^2*cos(a0) - 5184*L^4*a0*sin(2*a0) + 576*L^4*a0^3*sin(a0) + 1728*L^4*a0^2*cos(a0)^2 + 432*D^2*L^2*a0^4*cos(a0) - 576*D^2*L^2*a0^3*sin(a0) + 48*D^2*L^2*a0^5*sin(a0)));
% f2 = - (144*a0^5*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0))*(beta*da0 + a0*k + (2*L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^3 - (L^2*da0^2*m*(12*a0 - 30*sin(a0) + 18*a0*cos(a0) + a0^3 + 3*a0^2*sin(a0)))/(3*a0^6) + (L*g*m*(sin(a0 + phi + theta) - sin(phi + theta)))/a0^2 + (L^2*dtheta^2*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4))/(m*(12096*L^4 - 15552*L^4*cos(a0)^2 + D^4*a0^8 + 3456*L^4*a0^2 + 720*L^4*a0^4 + 3456*L^4*cos(a0) - 17280*L^4*a0*sin(a0) + 144*D^2*L^2*a0^4 + 72*D^2*L^2*a0^6 + 8640*L^4*a0^2*cos(a0) - 5184*L^4*a0*sin(2*a0) + 576*L^4*a0^3*sin(a0) + 1728*L^4*a0^2*cos(a0)^2 + 432*D^2*L^2*a0^4*cos(a0) - 576*D^2*L^2*a0^3*sin(a0) + 48*D^2*L^2*a0^5*sin(a0))) - (72*L*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0))*(4*L*da0^2 + a0^4*g*cos(phi + theta) + L*a0^2*da0^2 - 4*L*da0^2*cos(a0) + a0^3*g*sin(phi + theta) - a0^3*g*sin(a0 + phi + theta) + 4*L*a0^2*da0*dtheta - 4*L*a0*da0^2*sin(a0) + L*a0^2*da0^2*cos(a0) - 6*L*a0*da0*dtheta*sin(a0) + 2*L*a0^2*da0*dtheta*cos(a0)))/(a0*(12096*L^4 - 15552*L^4*cos(a0)^2 + D^4*a0^8 + 3456*L^4*a0^2 + 720*L^4*a0^4 + 3456*L^4*cos(a0) - 17280*L^4*a0*sin(a0) + 144*D^2*L^2*a0^4 + 72*D^2*L^2*a0^6 + 8640*L^4*a0^2*cos(a0) - 5184*L^4*a0*sin(2*a0) + 576*L^4*a0^3*sin(a0) + 1728*L^4*a0^2*cos(a0)^2 + 432*D^2*L^2*a0^4*cos(a0) - 576*D^2*L^2*a0^3*sin(a0) + 48*D^2*L^2*a0^5*sin(a0)));
% E_des = m*g*L/2;
% E_tilde = E-E_des;

%% with m2 at tip
% m2=0.01; %defined at the top near D_mtx
m2=0;

inv_M = inv(D_mtx);
f1 = inv_M(1,1);

hqq = -inv_M*(C_mtx*daa + Damp*daa + K*aa + G_vect);
f2 = hqq(1);

E=(a0^2*k)/2 - (L*g*m*sin(phi + theta) + (L*g*m2*sin(phi + theta))/2 - (L*g*m2*sin(a0 + phi + theta))/2)/a0 + (D^2*da0^2*m)/72 + (D^2*da0^2*m2)/48 + (D^2*dtheta^2*m)/24 + (D^2*dtheta^2*m2)/48 + (D^2*da0*dtheta*m)/24 + (D^2*da0*dtheta*m2)/24 + (L^2*da0^2*m)/(6*a0^2) + (L^2*da0^2*m2)/(4*a0^2) + (L^2*da0^2*m)/a0^4 + (L^2*da0^2*m2)/(2*a0^4) + (L^2*dtheta^2*m)/a0^2 + (L^2*dtheta^2*m2)/(2*a0^2) + (L^2*da0*dtheta*m)/(2*a0^2) + (L^2*da0*dtheta*m2)/(2*a0^2) + (L^2*da0*dtheta*m)/a0^4 - (L*g*m*cos(a0 + phi + theta))/a0^2 + (L^2*da0^2*m*cos(a0))/a0^4 - (L^2*da0^2*m2*cos(a0))/(2*a0^4) - (L^2*dtheta^2*m2*cos(a0))/(2*a0^2) - (L^2*da0^2*m2*sin(a0))/(2*a0^3) - (2*L^2*da0^2*m*sin(a0))/a0^5 - (L^2*dtheta^2*m*sin(a0))/a0^3 + (L*g*m*cos(phi + theta))/a0^2 - (L^2*da0*dtheta*m2*cos(a0))/(2*a0^2) - (L^2*da0*dtheta*m*cos(a0))/a0^4 - (L^2*da0*dtheta*m*sin(a0))/a0^3;
f1 = (72*a0^5*(2*D^2*a0^3*m + D^2*a0^3*m2 - 48*L^2*m*sin(a0) + 48*L^2*a0*m + 24*L^2*a0*m2 - 24*L^2*a0*m2*cos(a0)))/(4320*L^4*m^2 + D^4*a0^8*m^2 + 4320*L^4*a0^2*m^2 + 2592*L^4*a0^2*m2^2 + 720*L^4*a0^4*m^2 + 216*L^4*a0^4*m2^2 + 3456*L^4*m^2*cos(a0) - 7776*L^4*m^2*cos(2*a0) - 17280*L^4*a0*m^2*sin(a0) + 144*D^2*L^2*a0^4*m^2 + 72*D^2*L^2*a0^4*m2^2 + 72*D^2*L^2*a0^6*m^2 + 36*D^2*L^2*a0^6*m2^2 + 8640*L^4*a0^2*m^2*cos(a0) - 3456*L^4*a0^2*m2^2*cos(a0) - 5184*L^4*a0*m^2*sin(2*a0) + 576*L^4*a0^3*m^2*sin(a0) - 1728*L^4*a0^3*m2^2*sin(a0) + 2*D^4*a0^8*m*m2 + 4320*L^4*a0^2*m*m2 + 1440*L^4*a0^4*m*m2 + 864*L^4*a0^2*m^2*cos(2*a0) + 864*L^4*a0^2*m2^2*cos(2*a0) - 216*L^4*a0^4*m2^2*cos(2*a0) + 864*L^4*a0^3*m2^2*sin(2*a0) - 576*D^2*L^2*a0^3*m^2*sin(a0) + 48*D^2*L^2*a0^5*m^2*sin(a0) - 72*D^2*L^2*a0^5*m2^2*sin(a0) - 10368*L^4*a0*m*m2*sin(a0) + 144*D^2*L^2*a0^4*m*m2 + 144*D^2*L^2*a0^6*m*m2 + 288*L^4*a0^4*m*m2*cos(a0) + 5184*L^4*a0*m*m2*sin(2*a0) - 3456*L^4*a0^3*m*m2*sin(a0) - 4320*L^4*a0^2*m*m2*cos(2*a0) - 864*L^4*a0^3*m*m2*sin(2*a0) + 432*D^2*L^2*a0^4*m^2*cos(a0) - 72*D^2*L^2*a0^4*m2^2*cos(a0) + 144*D^2*L^2*a0^4*m*m2*cos(a0) + 24*D^2*L^2*a0^6*m*m2*cos(a0) - 288*D^2*L^2*a0^3*m*m2*sin(a0) - 144*D^2*L^2*a0^5*m*m2*sin(a0));
f2 = - (36*(24*L^2*m + D^2*a0^4*m + D^2*a0^4*m2 + 12*L^2*a0^2*m + 12*L^2*a0^2*m2 - 24*L^2*m*cos(a0) - 24*L^2*a0*m*sin(a0) - 12*L^2*a0^2*m2*cos(a0))*(8*L^2*da0^2*m - 8*L^2*da0^2*m*cos(a0) + 2*L^2*a0^2*da0^2*m + 2*L^2*a0^2*da0^2*m2 + 8*L^2*a0^2*da0*dtheta*m + 4*L^2*a0^2*da0*dtheta*m2 - L*a0^4*g*m2*cos(a0 + phi + theta) - 2*L*a0^3*g*m*sin(a0 + phi + theta) - 8*L^2*a0*da0^2*m*sin(a0) + 2*L^2*a0^2*da0^2*m*cos(a0) - 2*L^2*a0^2*da0^2*m2*cos(a0) - L^2*a0^3*da0^2*m2*sin(a0) + 2*L*a0^4*g*m*cos(phi + theta) + L*a0^4*g*m2*cos(phi + theta) + 2*L*a0^3*g*m*sin(phi + theta) - 12*L^2*a0*da0*dtheta*m*sin(a0) + 4*L^2*a0^2*da0*dtheta*m*cos(a0) - 4*L^2*a0^2*da0*dtheta*m2*cos(a0) - 2*L^2*a0^3*da0*dtheta*m2*sin(a0)))/(a0*(4320*L^4*m^2 + D^4*a0^8*m^2 + 4320*L^4*a0^2*m^2 + 2592*L^4*a0^2*m2^2 + 720*L^4*a0^4*m^2 + 216*L^4*a0^4*m2^2 + 3456*L^4*m^2*cos(a0) - 7776*L^4*m^2*cos(2*a0) - 17280*L^4*a0*m^2*sin(a0) + 144*D^2*L^2*a0^4*m^2 + 72*D^2*L^2*a0^4*m2^2 + 72*D^2*L^2*a0^6*m^2 + 36*D^2*L^2*a0^6*m2^2 + 8640*L^4*a0^2*m^2*cos(a0) - 3456*L^4*a0^2*m2^2*cos(a0) - 5184*L^4*a0*m^2*sin(2*a0) + 576*L^4*a0^3*m^2*sin(a0) - 1728*L^4*a0^3*m2^2*sin(a0) + 2*D^4*a0^8*m*m2 + 4320*L^4*a0^2*m*m2 + 1440*L^4*a0^4*m*m2 + 864*L^4*a0^2*m^2*cos(2*a0) + 864*L^4*a0^2*m2^2*cos(2*a0) - 216*L^4*a0^4*m2^2*cos(2*a0) + 864*L^4*a0^3*m2^2*sin(2*a0) - 576*D^2*L^2*a0^3*m^2*sin(a0) + 48*D^2*L^2*a0^5*m^2*sin(a0) - 72*D^2*L^2*a0^5*m2^2*sin(a0) - 10368*L^4*a0*m*m2*sin(a0) + 144*D^2*L^2*a0^4*m*m2 + 144*D^2*L^2*a0^6*m*m2 + 288*L^4*a0^4*m*m2*cos(a0) + 5184*L^4*a0*m*m2*sin(2*a0) - 3456*L^4*a0^3*m*m2*sin(a0) - 4320*L^4*a0^2*m*m2*cos(2*a0) - 864*L^4*a0^3*m*m2*sin(2*a0) + 432*D^2*L^2*a0^4*m^2*cos(a0) - 72*D^2*L^2*a0^4*m2^2*cos(a0) + 144*D^2*L^2*a0^4*m*m2*cos(a0) + 24*D^2*L^2*a0^6*m*m2*cos(a0) - 288*D^2*L^2*a0^3*m*m2*sin(a0) - 144*D^2*L^2*a0^5*m*m2*sin(a0))) - (72*a0^5*(2*D^2*a0^3*m + D^2*a0^3*m2 - 48*L^2*m*sin(a0) + 48*L^2*a0*m + 24*L^2*a0*m2 - 24*L^2*a0*m2*cos(a0))*(beta*da0 + a0*k + (L*g*(2*m*sin(phi + theta) + m2*sin(phi + theta) - m2*sin(a0 + phi + theta)))/(2*a0^2) + (L^2*dtheta^2*(4*a0*m + 2*a0*m2 - 6*m*sin(a0) - a0^2*m2*sin(a0) + 2*a0*m*cos(a0) - 2*a0*m2*cos(a0)))/(2*a0^4) - (L^2*da0^2*(24*a0*m + 12*a0*m2 + 2*a0^3*m + 3*a0^3*m2 - 60*m*sin(a0) + 3*a0^3*m2*cos(a0) + 6*a0^2*m*sin(a0) - 12*a0^2*m2*sin(a0) + 36*a0*m*cos(a0) - 12*a0*m2*cos(a0)))/(6*a0^6) + (L*g*m2*cos(a0 + phi + theta))/(2*a0) + (2*L*g*m*cos(a0 + phi + theta))/a0^3 + (L*g*m*sin(a0 + phi + theta))/a0^2 - (2*L*g*m*cos(phi + theta))/a0^3))/(4320*L^4*m^2 + D^4*a0^8*m^2 + 4320*L^4*a0^2*m^2 + 2592*L^4*a0^2*m2^2 + 720*L^4*a0^4*m^2 + 216*L^4*a0^4*m2^2 + 3456*L^4*m^2*cos(a0) - 7776*L^4*m^2*cos(2*a0) - 17280*L^4*a0*m^2*sin(a0) + 144*D^2*L^2*a0^4*m^2 + 72*D^2*L^2*a0^4*m2^2 + 72*D^2*L^2*a0^6*m^2 + 36*D^2*L^2*a0^6*m2^2 + 8640*L^4*a0^2*m^2*cos(a0) - 3456*L^4*a0^2*m2^2*cos(a0) - 5184*L^4*a0*m^2*sin(2*a0) + 576*L^4*a0^3*m^2*sin(a0) - 1728*L^4*a0^3*m2^2*sin(a0) + 2*D^4*a0^8*m*m2 + 4320*L^4*a0^2*m*m2 + 1440*L^4*a0^4*m*m2 + 864*L^4*a0^2*m^2*cos(2*a0) + 864*L^4*a0^2*m2^2*cos(2*a0) - 216*L^4*a0^4*m2^2*cos(2*a0) + 864*L^4*a0^3*m2^2*sin(2*a0) - 576*D^2*L^2*a0^3*m^2*sin(a0) + 48*D^2*L^2*a0^5*m^2*sin(a0) - 72*D^2*L^2*a0^5*m2^2*sin(a0) - 10368*L^4*a0*m*m2*sin(a0) + 144*D^2*L^2*a0^4*m*m2 + 144*D^2*L^2*a0^6*m*m2 + 288*L^4*a0^4*m*m2*cos(a0) + 5184*L^4*a0*m*m2*sin(2*a0) - 3456*L^4*a0^3*m*m2*sin(a0) - 4320*L^4*a0^2*m*m2*cos(2*a0) - 864*L^4*a0^3*m*m2*sin(2*a0) + 432*D^2*L^2*a0^4*m^2*cos(a0) - 72*D^2*L^2*a0^4*m2^2*cos(a0) + 144*D^2*L^2*a0^4*m*m2*cos(a0) + 24*D^2*L^2*a0^6*m*m2*cos(a0) - 288*D^2*L^2*a0^3*m*m2*sin(a0) - 144*D^2*L^2*a0^5*m*m2*sin(a0));
E_des = m*g*L/2 + m2*g*L/2;
E_tilde = E-E_des;




kE=3;
kD=1.25;
kP=1.1;

% kE=1.2;
% kD=0.4;
% kP=0.5;

% kE=54000;
% kD=0.75;
% kP=1.2;

% kE=1.*150000;
% kD=1.5*0.65;
% kP=0.59;
% kV=1;
% kE=54498.14;
% kD=0.75;
% kP=1.2;

%%% works ok for m=0.5, L=0.5
% % kE=10;
% % kD=5;
% % kP=12;

%Working numbers%%
% kE=1.2;
% kD=30*(kE+0.05)/12940;
% kP=5;
%%%%%%%%%%%%%%%%%%
% tau = inv(kE*E_tilde + kD*f1)*(-da0 - kD*f2 - kP*a0)/k;
tau = (-da0 +kE*E_tilde*beta*da0 - kD*f2 - kP*a0)/((kE*E_tilde + kD*f1)*(k));
% tau =  -[-2.1053   -6.2855   -0.1908   -0.5349]*[aa(1);aa(2);daa];
% tau =  -[-66.4385 -198.2555   -6.0170  -16.8740]*[aa(1);aa(2);daa];  

    %% Assigning PWM signals based on torques
    if tau>0
        tau1=min(tau_timit,tau);
        signal1B= round(p1*tau1.^5 + p2*tau1.^4 + p3*tau1.^3 + p4*tau1.^2 + p5*tau1 + p6)+20;
%         if tau1<tt1(end)
%             %             signal1B=signal1B;
%             signal1A=0*28;
%         else
            signal1A=0;
%         end
    else
        tau1=min(tau_timit,-tau);
        signal1A= round(p1*tau1.^5 + p2*tau1.^4 + p3*tau1.^3 + p4*tau1.^2 + p5*tau1 + p6)+20;
%         if tau1<tt1(end)
%             %             signal1A=signal1A-3;
%             signal1B=0*27;
%         else
            signal1B=0;
%         end
    end

%         aa=[signal1A,signal1B,signal2A,signal2B,signal3A,signal3B]
    
    %     tt1=[tt1;tau1];
    %     tt2=[tt2;tau2];
    
%     f1=sprintf('%.2d%',signal1A);
%     f2=sprintf('%.2d%',signal1B);
%     f3=sprintf('%.2d%',0);
%     f4=sprintf('%.2d%',0);
%     f5=sprintf('%.2d%',0);
%     f6=sprintf('%.2d%',0);    
    [j,tau,signal1A,signal1B]
    
    f5=sprintf('%.2d%',signal1A);
    f3=sprintf('%.2d%',signal1B);
    f2=sprintf('%.2d%',0);
    f4=sprintf('%.2d%',0);
    f1=sprintf('%.2d%',0);
    f6=sprintf('%.2d%',0);   
    %     f1=sprintf('%.2d%',0);
    %     f2=sprintf('%.2d%',45);
    %     f3=sprintf('%.2d%',0);
    %     f4=sprintf('%.2d%',35);
    
    fprintf(s,'%s*',f1);
    fprintf(s,'%s#',f2);
    fprintf(s,'%s&',f3);
    fprintf(s,'%s@',f4);
    fprintf(s,'%s(',f5);
    fprintf(s,'%s\n',f6);    
    





    figure(1);
    plot(myData(:,1),myData(:,2),'b-'); %curvature
%     axis([0,35,-180,180]);
    hold on;
    plot(myData(:,1),myData(:,3),'r-'); %rot_theta
    plot(myData(:,1),myData(:,4),'b--'); %rot_theta
    plot(myData(:,1),myData(:,5),'r--'); %rot_theta
    hold off
    %%
    pause(0.005);
end


%%

% time=toc
% t=(1:length(mydata))*(time/length(mydata));
% mydata_B_fast=mydata;
disp('NatNet Polling Sample End' )
        f1=sprintf('%.2d%',0);
        f2=sprintf('%.2d%',0);
        f3=sprintf('%.2d%',0);
        f4=sprintf('%.2d%',0);
        f5=sprintf('%.2d%',0);
        f6=sprintf('%.2d%',0);
        
        
        fprintf(s,'%s*',f1);
        fprintf(s,'%s#',f2);
        fprintf(s,'%s&',f3);
        fprintf(s,'%s@',f4);
        fprintf(s,'%s(',f5);
        fprintf(s,'%s\n',f6);
        
%% 
    figure(1);
    plot(myData(:,1),myData(:,2),'b-'); %curvature
%     axis([0,35,-180,180]);
    hold on;
    plot(myData(:,1),myData(:,3),'r-'); %rot_theta
%     plot(myData(:,1),myData(:,4),'b--'); %rot_theta
%     plot(myData(:,1),myData(:,5),'r--'); %rot_theta
    hold off


fclose(s);
% fclose(s2);
disp('Arduino closed successfully' )


clear all; 

%%%%
% No feedback linearization
% Energy Based Control of the Pendubot
% Isabelle Fantoni, Rogelio Lozano, and Mark W. Spong

close all; 
clc

%load the ROA for LQR
load('ROA_sparse_FULL_matrix.mat')

global k m L D beta

%%Defining parameters
phi = 0; %gravity angle
D = 0.024; %width of the link
L = 0.105; %length of the link
m = 0.045; %mass of the link
g =9.81; %gravitational acceleratiom
beta = 5e-3; %damping coeff
k = 7.65e-3; %stiffness coeff
m2=0.00; 

%simulator parameters
init_t=0;
final_t=3; %initial swing up time
final_t2=0.001; %time for swing up, with check
final_t3=5; %time for LQR
dt=0.0001; %step size for all sims

t_span1=init_t:dt:final_t;  %initial swing up time span
t_span2=init_t:dt:final_t2; %time span for swing up, with check
t_span3=init_t:dt:final_t3; %time span for LQR

%% Set initial condition 
% theta0 = [curvature, base rotation, curvature speed, base rotation speed]

% theta0=[-0.0002  0.0174    0.0005   -0.0005].'; %works well
% theta0=[pi/108  pi/1.01 0 0].'; %works well

theta0=[-pi/72   -pi/1.1   0.001    0.001].'; %% used this % works well


%% Solving the ODE

% Initially run swing up for t_span1 = 0-3s
[t,y0] = ode45(@f_x_swing_up,t_span1,theta0); %swing up

% [t,y0] = ode45(@f_x_LQR,t_span3,theta0); %use this if only trying LQR

T=[t y0];
Y=y0;
t_end=t(end);
tt=t;
yy=y0;


switch_cont=1; %set to 1 in order to check for ROA and switch to LQR
if switch_cont==1
    
% Check if within region of attraction every 0.001s
% if inside ROA, switch to LQR
for j=1:2500
    y_end = yy(end,:);

    [val,err]=check_ROA(y_end,roa); % check for ROA
    Current_values = [j,y_end,err] %printing

    if val==1 % if in ROA
            disp('Reached ROA');
            [t,y] = ode45(@f_x_LQR,t_span3,yy(end,:)); %LQR
            T=[T;t+t_end y];
            t=[tt;t_end+t];
            y=[yy;y];
            t_end=t(end);
            tt=t;
            yy=y;
            y_end = yy(end,:);
            break
    end
    
    [t,y] = ode45(@f_x_swing_up,t_span2,yy(end,:)); %Swing up
    T=[T;t+t_end y];
    t=[tt;t_end+t];
    y=[yy;y];
    t_end=t(end);
    tt=t;
    yy=y;
end
end


%% Plotting
th1=yy(:,1);
% th2=mod(yy(:,2)+pi,2*pi)-pi;
% th2=yy(:,2);
th2=wrapTo2Pi(yy(:,2));
th1d=yy(:,3);
th2d=yy(:,4);

%% Plot angles
figure(1)
plot(tt,th1,'-','LineWidth',1,'Color',[1 0 0]);
hold on
% plot(tt,th1d,'--','LineWidth',1,'Color',[1 0 0]); %velocity
plot(tt,th2,'-','LineWidth',1,'Color',[1 .6 0]);
% plot(tt,th2d,'--','LineWidth',1,'Color',[1 .6 0]); %velocity
grid on
axis([0 tt(end) -10 10])
set(legend('$\theta_0$','$\theta_1$'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
set(xlabel('Time / s'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
set(ylabel('Angle / rad'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')

%% Plot energy
a0=yy(:,1);
theta=yy(:,2);
theta_mod=mod(pi-theta,2*pi)-pi;
da0=yy(:,3);
dtheta=yy(:,4);


% remedy for devision by zero
thresh = 1e-3;
for i =1: length(a0)
if a0(i)<thresh && a0(i)>=0 && da0(i)<0
    a0(i) = -thresh*10 ;
elseif a0(i)<thresh && a0(i)>=0 && da0(i)>0
    a0(i) = thresh*10;
elseif a0(i)>-thresh && a0(i)<=0 && da0(i)>0
    a0(i) = thresh*10;
elseif a0(i)>-thresh && a0(i)<=0 && da0(i)<0
    a0(i) = -thresh*10 ;
end

if theta(i)<thresh && theta(i)>=0 && da0(i)<0
    theta(i) = -thresh*10 ;
elseif theta(i)<thresh && theta(i)>=0 && da0(i)>0
    theta(i) = thresh*10;
elseif theta(i)>-thresh && theta(i)<=0 && da0(i)>0
    theta(i) = thresh*10;
elseif theta(i)>-thresh && theta(i)<=0 && da0(i)<0
    theta(i) = -thresh*10 ;
end

end

%% Calculate enrgy
E = (72.*a0.^7.*k - 288.*L.^2.*da0.^2.*m.*sin(a0) + 144.*L.^2.*a0.*da0.^2.*m + 72.*L.^2.*a0.*da0.^2.*m2 + 2.*D.^2.*a0.^5.*da0.^2.*m + 3.*D.^2.*a0.^5.*da0.^2.*m2 + 6.*D.^2.*a0.^5.*dtheta.^2.*m + 3.*D.^2.*a0.^5.*dtheta.^2.*m2 + 24.*L.^2.*a0.^3.*da0.^2.*m + 36.*L.^2.*a0.^3.*da0.^2.*m2 + 144.*L.^2.*a0.^3.*dtheta.^2.*m + 72.*L.^2.*a0.^3.*dtheta.^2.*m2 + 72.*L.^2.*a0.^3.*da0.*dtheta.*m + 72.*L.^2.*a0.^3.*da0.*dtheta.*m2 - 144.*L.*a0.^3.*g.*m.*cos(a0 + phi + theta) + 72.*L.*a0.^4.*g.*m2.*sin(a0 + phi + theta) + 144.*L.^2.*a0.*da0.^2.*m.*cos(a0) - 72.*L.^2.*a0.*da0.^2.*m2.*cos(a0) + 144.*L.^2.*a0.*da0.*dtheta.*m - 72.*L.^2.*a0.^3.*dtheta.^2.*m2.*cos(a0) - 72.*L.^2.*a0.^2.*da0.^2.*m2.*sin(a0) - 144.*L.^2.*a0.^2.*dtheta.^2.*m.*sin(a0) + 144.*L.*a0.^3.*g.*m.*cos(phi + theta) - 144.*L.*a0.^4.*g.*m.*sin(phi + theta) - 72.*L.*a0.^4.*g.*m2.*sin(phi + theta) + 6.*D.^2.*a0.^5.*da0.*dtheta.*m + 6.*D.^2.*a0.^5.*da0.*dtheta.*m2 - 144.*L.^2.*a0.*da0.*dtheta.*m.*cos(a0) - 72.*L.^2.*a0.^3.*da0.*dtheta.*m2.*cos(a0) - 144.*L.^2.*a0.^2.*da0.*dtheta.*m.*sin(a0))./(144.*a0.^5);
max(E)
E_des = m.*g.*L./2 + m2*g*L/2;
figure(2)
plot(tt,E,'-','LineWidth',1,'Color',[1 0 0]);
% axis([0 tt(end) -15 15]);
set(legend('Total Energy'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
grid on;
set(xlabel('Time / s'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
set(ylabel('Energy / J'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
hold on
plot([0 tt(end)],[E_des E_des],'--','LineWidth',1,'Color',[0 0 1]);

f1 = (144.*a0.^5.*(24.*L.^2.*a0 + D.^2.*a0.^3 - 24.*L.^2.*sin(a0)))./(m.*(12096.*L.^4 - 15552.*L.^4.*cos(a0).^2 + D.^4.*a0.^8 + 3456.*L.^4.*a0.^2 + 720.*L.^4.*a0.^4 + 3456.*L.^4.*cos(a0) - 17280.*L.^4.*a0.*sin(a0) + 144.*D.^2.*L.^2.*a0.^4 + 72.*D.^2.*L.^2.*a0.^6 + 8640.*L.^4.*a0.^2.*cos(a0) - 5184.*L.^4.*a0.*sin(2.*a0) + 576.*L.^4.*a0.^3.*sin(a0) + 1728.*L.^4.*a0.^2.*cos(a0).^2 + 432.*D.^2.*L.^2.*a0.^4.*cos(a0) - 576.*D.^2.*L.^2.*a0.^3.*sin(a0) + 48.*D.^2.*L.^2.*a0.^5.*sin(a0)));
f2 = - (144.*a0.^5.*(24.*L.^2.*a0 + D.^2.*a0.^3 - 24.*L.^2.*sin(a0)).*(beta.*da0 + a0.*k + (2.*L.*g.*m.*(cos(a0 + phi + theta) - cos(phi + theta) + a0.*sin(phi + theta)))./a0.^3 - (L.^2.*da0.^2.*m.*(12.*a0 - 30.*sin(a0) + 18.*a0.*cos(a0) + a0.^3 + 3.*a0.^2.*sin(a0)))./(3.*a0.^6) + (L.*g.*m.*(sin(a0 + phi + theta) - sin(phi + theta)))./a0.^2 + (L.^2.*dtheta.^2.*m.*(2.*a0 - 3.*sin(a0) + a0.*cos(a0)))./a0.^4))./(m.*(12096.*L.^4 - 15552.*L.^4.*cos(a0).^2 + D.^4.*a0.^8 + 3456.*L.^4.*a0.^2 + 720.*L.^4.*a0.^4 + 3456.*L.^4.*cos(a0) - 17280.*L.^4.*a0.*sin(a0) + 144.*D.^2.*L.^2.*a0.^4 + 72.*D.^2.*L.^2.*a0.^6 + 8640.*L.^4.*a0.^2.*cos(a0) - 5184.*L.^4.*a0.*sin(2.*a0) + 576.*L.^4.*a0.^3.*sin(a0) + 1728.*L.^4.*a0.^2.*cos(a0).^2 + 432.*D.^2.*L.^2.*a0.^4.*cos(a0) - 576.*D.^2.*L.^2.*a0.^3.*sin(a0) + 48.*D.^2.*L.^2.*a0.^5.*sin(a0))) - (72.*L.*(24.*L.^2 + D.^2.*a0.^4 + 12.*L.^2.*a0.^2 - 24.*L.^2.*cos(a0) - 24.*L.^2.*a0.*sin(a0)).*(4.*L.*da0.^2 + a0.^4.*g.*cos(phi + theta) + L.*a0.^2.*da0.^2 - 4.*L.*da0.^2.*cos(a0) + a0.^3.*g.*sin(phi + theta) - a0.^3.*g.*sin(a0 + phi + theta) + 4.*L.*a0.^2.*da0.*dtheta - 4.*L.*a0.*da0.^2.*sin(a0) + L.*a0.^2.*da0.^2.*cos(a0) - 6.*L.*a0.*da0.*dtheta.*sin(a0) + 2.*L.*a0.^2.*da0.*dtheta.*cos(a0)))./(a0.*(12096.*L.^4 - 15552.*L.^4.*cos(a0).^2 + D.^4.*a0.^8 + 3456.*L.^4.*a0.^2 + 720.*L.^4.*a0.^4 + 3456.*L.^4.*cos(a0) - 17280.*L.^4.*a0.*sin(a0) + 144.*D.^2.*L.^2.*a0.^4 + 72.*D.^2.*L.^2.*a0.^6 + 8640.*L.^4.*a0.^2.*cos(a0) - 5184.*L.^4.*a0.*sin(2.*a0) + 576.*L.^4.*a0.^3.*sin(a0) + 1728.*L.^4.*a0.^2.*cos(a0).^2 + 432.*D.^2.*L.^2.*a0.^4.*cos(a0) - 576.*D.^2.*L.^2.*a0.^3.*sin(a0) + 48.*D.^2.*L.^2.*a0.^5.*sin(a0)));
E_des = m.*g.*L./2+m2*g*L/2;
E_tilde = E-E_des;

kE=3;
kD=1.25;
kP=1.1;

%% Calcuates tau and plot

%check these values to match the ones defined in the ode function
kE=54498.14;
kD=0.75;
kP=1.2;

Tau = (-da0 - kD.*f2 - kP.*a0+kE.*E_tilde.*beta.*da0)./(kE.*E_tilde + kD.*f1);

%LQR tau
% Tau2 = -[   -11.4947*1  -34.3632*1   -1.0437   -2.8487]*[a0,theta_mod,da0,dtheta].';


figure(11)
plot(tt,Tau,'-','LineWidth',1,'Color',[1 0 0]);
hold on
% plot(tt,Tau2,'-','LineWidth',1,'Color',[0 0 1]);
% axis([0 tt(end) -15 15]);
set(legend('Tau'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
grid on;
set(xlabel('Time / s'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
set(ylabel('Nm / J'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')


%% Animation

% Set up the movie.

% Uncomment following for recording the animation
%%%%
% figure(4)
% writerObj = VideoWriter('New_no_fb_-45 180 stiff vary 0.2'); % Name it.
% writerObj.FrameRate = 20; % 12 How many frames per second.
% open(writerObj); 
%%%%

playvid=1
if playvid==1
figure(4)
for i=1:1000:length(tt)
    tt(i);
    hold off
    
    q=a0(i) ;
    th = theta(i);
    
    t=[0 1].';
    x=0;
    y=0;
        
    for a=1
        Len=0:0.001:L(1);
        
        t0=t;
        n0=[-t0(2)/sqrt(t0(1)^2+t0(2)^2);t0(1)/sqrt(t0(1)^2+t0(2)^2)];
                
        R=Len(end)/q(a);
        
        x1=x(end)+R*t0(1)*sin(Len/R)-R*t0(2)*(1-cos(Len/R));
        y1=y(end)+R*t0(2)*sin(Len/R)+R*t0(1)*(1-cos(Len/R));
        
        tLen(1,:)=t0(1)*cos(Len/R)+n0(1)*sin(Len/R);
        tLen(2,:)=t0(2)*cos(Len/R)+n0(2)*sin(Len/R);
        t=tLen(:,end);
        clear tLen

        x = x1.*cos(th) - y1.*sin(th);
        y = x1.*sin(th) + y1.*cos(th);
        
        figure(4); 
%         subplot(2,2,[1 3])

        plot(x,y,'Linewidth',25);
        hold on
        plot(x(end),y(end),'r.','MarkerSize',15);
        hold off
        
        axis equal;
        axis([-1.2*L(1) 1.2*L(1) -1.2*L(1) 1.2*L(1)]);
        hold on
        grid on
        tit = title(['Time = ',num2str(tt(i))]);
        set(tit, 'horizontalAlignment', 'left');
       
        
    end
    hold off

%     % Uncomment for plotting the other plots (uncomment above subplot line)
%     figure(4); 
%     subplot(2,2,2)
%     plot(tt(1:i),th1(1:i),'-','LineWidth',1,'Color',[1 0 0]);
%     hold on;
%     plot(tt(1:i),th2(1:i),'-','LineWidth',1,'Color',[1 .6 0]);
%     grid on
%     axis([0 tt(end) -10 10])
%     title('Evolution of Angles');
% %     set(legend('Curvature','Base rotation'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
%     legend('Curvature','Base rotation')
%     hold off
%     
%     subplot(2,2,4)
%     plot(tt(1:i),E(1:i),'-','LineWidth',1,'Color',[0 0 1]);
% %     set(legend('Total Energy'),'interpreter','latex','FontSize',15,'FontName', 'TimesNewRoman')
%     legend('Total Energy')
%     grid on;
%     title('Evolution of Total Energy');
%     axis([0 tt(end) -10 25])
    
    pause(0.001)
    
    % Uncomment following for recording
    %%%
% %     %if mod(i,4)==0, % Uncomment to take 1 out of every 4 frames.
%         frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
%         writeVideo(writerObj, frame);
% %     %end
%%%
end

% Uncomment to save aniimation

%%%
% close(writerObj); % Saves the movie.
%%%

end


%% Swing up controller function for ode


function xdoubledot=f_x_swing_up(T,y)
global k m L D beta

%parameters
phi = 0;
g =9.81;

% defining the angles and rates of change
a0=y(1);
% theta=mod(pi+y(2),2*pi)-pi;
theta=y(2);
da0=y(3);
dtheta=y(4); 

% remedy for devision by zero
thresh = 1e-4;
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

%% Dynamics

%% Uniform mass
% D_mtx = [ (m*(72*L^2*a0 + D^2*a0^5 + 12*L^2*a0^3 - 144*L^2*sin(a0) + 72*L^2*a0*cos(a0)))/(36*a0^5), (m*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0)))/(24*a0^4);
%              (m*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0)))/(24*a0^4),                                (m*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0)))/(12*a0^3)];
% 
% C_mtx = [                                                                 -(L^2*da0*m*(12*a0 - 30*sin(a0) + 18*a0*cos(a0) + a0^3 + 3*a0^2*sin(a0)))/(3*a0^6), (L^2*dtheta*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4;
%          -(L^2*m*(4*da0 + a0^2*da0 + 2*a0^2*dtheta - 4*da0*cos(a0) + a0^2*da0*cos(a0) + a0^2*dtheta*cos(a0) - 4*a0*da0*sin(a0) - 3*a0*dtheta*sin(a0)))/a0^5,   -(L^2*da0*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4];
% 
% 
% 
% G_vect = [(2*L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^3 + (L*g*m*(sin(a0 + phi + theta) - sin(phi + theta)))/a0^2;
%                                                                     -(L*g*m*(sin(phi + theta) - sin(a0 + phi + theta) + a0*cos(phi + theta)))/a0^2];
% 
% 

%% m2 at tip      
m2 = 0.05;

D_mtx = [(2*D^2*a0^5*m + 3*D^2*a0^5*m2 + 24*L^2*a0^3*m + 36*L^2*a0^3*m2 - 288*L^2*m*sin(a0) + 144*L^2*a0*m + 72*L^2*a0*m2 - 72*L^2*a0^2*m2*sin(a0) + 144*L^2*a0*m*cos(a0) - 72*L^2*a0*m2*cos(a0))/(72*a0^5), (D^2*m)/24 + (D^2*m2)/24 + (2*L^2*m*((sin(a0/2)^2 - (a0*sin(a0))/2)/a0^2 + 1/4))/a0^2 + (L^2*m2*sin(a0/2)^2)/a0^2;
                                                                                  (D^2*m)/24 + (D^2*m2)/24 + (2*L^2*m*((sin(a0/2)^2 - (a0*sin(a0))/2)/a0^2 + 1/4))/a0^2 + (L^2*m2*sin(a0/2)^2)/a0^2,              (D^2*m)/12 + (D^2*m2)/24 - (4*L^2*m*(sin(a0)/(2*a0) - 1/2))/a0^2 - (2*L^2*m2*(cos(a0)/2 - 1/2))/a0^2];


C_mtx = [                                                                                                                                 -(L^2*da0*(24*a0*m + 12*a0*m2 + 2*a0^3*m + 3*a0^3*m2 - 60*m*sin(a0) + 3*a0^3*m2*cos(a0) + 6*a0^2*m*sin(a0) - 12*a0^2*m2*sin(a0) + 36*a0*m*cos(a0) - 12*a0*m2*cos(a0)))/(6*a0^6), (L^2*dtheta*(4*a0*m + 2*a0*m2 - 6*m*sin(a0) - a0^2*m2*sin(a0) + 2*a0*m*cos(a0) - 2*a0*m2*cos(a0)))/(2*a0^4);
     -(L^2*(8*da0*m + 2*a0^2*da0*m + 2*a0^2*da0*m2 + 4*a0^2*dtheta*m + 2*a0^2*dtheta*m2 - 8*da0*m*cos(a0) - 8*a0*da0*m*sin(a0) - 6*a0*dtheta*m*sin(a0) + 2*a0^2*da0*m*cos(a0) - 2*a0^2*da0*m2*cos(a0) + 2*a0^2*dtheta*m*cos(a0) - 2*a0^2*dtheta*m2*cos(a0) - a0^3*da0*m2*sin(a0) - a0^3*dtheta*m2*sin(a0)))/(2*a0^5),   -(L^2*da0*(4*a0*m + 2*a0*m2 - 6*m*sin(a0) - a0^2*m2*sin(a0) + 2*a0*m*cos(a0) - 2*a0*m2*cos(a0)))/(2*a0^4)];                                                                             
                                                                              
                                                                              
G_vect = [(L*g*(4*m*cos(a0 + phi + theta) - 4*m*cos(phi + theta) + 2*a0*m*sin(a0 + phi + theta) - a0*m2*sin(a0 + phi + theta) + a0^2*m2*cos(a0 + phi + theta) + 2*a0*m*sin(phi + theta) + a0*m2*sin(phi + theta)))/(2*a0^3);
                                                               -(L*g*(2*m*sin(phi + theta) - 2*m*sin(a0 + phi + theta) - a0*m2*cos(a0 + phi + theta) + 2*a0*m*cos(phi + theta) + a0*m2*cos(phi + theta)))/(2*a0^2)];


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
% D = 0.1;
% L = 0.23;
% m = 0.153;
% g =9.81;
% beta = 0.01;
% k=0.05;

% % without the damping
% E = (a0^2*k)/2 + (D^2*da0^2*m)/72 + (D^2*dtheta^2*m)/24 + (D^2*da0*dtheta*m)/24 + (L^2*da0^2*m)/(6*a0^2) + (2*L^2*da0^2*m)/a0^4 + (L^2*dtheta^2*m)/a0^2 + (L^2*da0*dtheta*m)/(2*a0^2) - (L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^2 - (2*L^2*da0^2*m*sin(a0/2)^2)/a0^4 - (2*L^2*da0^2*m*sin(a0))/a0^5 - (L^2*dtheta^2*m*sin(a0))/a0^3 + (2*L^2*da0*dtheta*m*sin(a0/2)^2)/a0^4 - (L^2*da0*dtheta*m*sin(a0))/a0^3;
% 
% % with the damping
% % E = (da0^2*beta)/2 + (a0^2*k)/2 + (D^2*da0^2*m)/72 + (D^2*dtheta^2*m)/24 + (D^2*da0*dtheta*m)/24 + (L^2*da0^2*m)/(6*a0^2) + (2*L^2*da0^2*m)/a0^4 + (L^2*dtheta^2*m)/a0^2 + (L^2*da0*dtheta*m)/(2*a0^2) - (L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^2 - (2*L^2*da0^2*m*sin(a0/2)^2)/a0^4 - (2*L^2*da0^2*m*sin(a0))/a0^5 - (L^2*dtheta^2*m*sin(a0))/a0^3 + (2*L^2*da0*dtheta*m*sin(a0/2)^2)/a0^4 - (L^2*da0*dtheta*m*sin(a0))/a0^3;
% 
% 
% f1 = (144*a0^5*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0)))/(m*(12096*L^4 - 15552*L^4*cos(a0)^2 + D^4*a0^8 + 3456*L^4*a0^2 + 720*L^4*a0^4 + 3456*L^4*cos(a0) - 17280*L^4*a0*sin(a0) + 144*D^2*L^2*a0^4 + 72*D^2*L^2*a0^6 + 8640*L^4*a0^2*cos(a0) - 5184*L^4*a0*sin(2*a0) + 576*L^4*a0^3*sin(a0) + 1728*L^4*a0^2*cos(a0)^2 + 432*D^2*L^2*a0^4*cos(a0) - 576*D^2*L^2*a0^3*sin(a0) + 48*D^2*L^2*a0^5*sin(a0)));
% f2 = - (144*a0^5*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0))*(beta*da0 + a0*k + (2*L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^3 - (L^2*da0^2*m*(12*a0 - 30*sin(a0) + 18*a0*cos(a0) + a0^3 + 3*a0^2*sin(a0)))/(3*a0^6) + (L*g*m*(sin(a0 + phi + theta) - sin(phi + theta)))/a0^2 + (L^2*dtheta^2*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4))/(m*(12096*L^4 - 15552*L^4*cos(a0)^2 + D^4*a0^8 + 3456*L^4*a0^2 + 720*L^4*a0^4 + 3456*L^4*cos(a0) - 17280*L^4*a0*sin(a0) + 144*D^2*L^2*a0^4 + 72*D^2*L^2*a0^6 + 8640*L^4*a0^2*cos(a0) - 5184*L^4*a0*sin(2*a0) + 576*L^4*a0^3*sin(a0) + 1728*L^4*a0^2*cos(a0)^2 + 432*D^2*L^2*a0^4*cos(a0) - 576*D^2*L^2*a0^3*sin(a0) + 48*D^2*L^2*a0^5*sin(a0))) - (72*L*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0))*(4*L*da0^2 + a0^4*g*cos(phi + theta) + L*a0^2*da0^2 - 4*L*da0^2*cos(a0) + a0^3*g*sin(phi + theta) - a0^3*g*sin(a0 + phi + theta) + 4*L*a0^2*da0*dtheta - 4*L*a0*da0^2*sin(a0) + L*a0^2*da0^2*cos(a0) - 6*L*a0*da0*dtheta*sin(a0) + 2*L*a0^2*da0*dtheta*cos(a0)))/(a0*(12096*L^4 - 15552*L^4*cos(a0)^2 + D^4*a0^8 + 3456*L^4*a0^2 + 720*L^4*a0^4 + 3456*L^4*cos(a0) - 17280*L^4*a0*sin(a0) + 144*D^2*L^2*a0^4 + 72*D^2*L^2*a0^6 + 8640*L^4*a0^2*cos(a0) - 5184*L^4*a0*sin(2*a0) + 576*L^4*a0^3*sin(a0) + 1728*L^4*a0^2*cos(a0)^2 + 432*D^2*L^2*a0^4*cos(a0) - 576*D^2*L^2*a0^3*sin(a0) + 48*D^2*L^2*a0^5*sin(a0)));
% E_des = m*g*L/2;
% E_tilde = E-E_des;

%% with m2 at tip
% m2=0.01; %defined at the top near D_mtx
inv_M = inv(D_mtx);
f1 = inv_M(1,1);

hqq = -inv_M*(C_mtx*daa + Damp*daa + K*aa + G_vect);
f2 = hqq(1);

E = (72*a0^7*k - 288*L^2*da0^2*m*sin(a0) + 144*L^2*a0*da0^2*m + 72*L^2*a0*da0^2*m2 + 2*D^2*a0^5*da0^2*m + 3*D^2*a0^5*da0^2*m2 + 6*D^2*a0^5*dtheta^2*m + 3*D^2*a0^5*dtheta^2*m2 + 24*L^2*a0^3*da0^2*m + 36*L^2*a0^3*da0^2*m2 + 144*L^2*a0^3*dtheta^2*m + 72*L^2*a0^3*dtheta^2*m2 + 72*L^2*a0^3*da0*dtheta*m + 72*L^2*a0^3*da0*dtheta*m2 - 144*L*a0^3*g*m*cos(a0 + phi + theta) + 72*L*a0^4*g*m2*sin(a0 + phi + theta) + 144*L^2*a0*da0^2*m*cos(a0) - 72*L^2*a0*da0^2*m2*cos(a0) + 144*L^2*a0*da0*dtheta*m - 72*L^2*a0^3*dtheta^2*m2*cos(a0) - 72*L^2*a0^2*da0^2*m2*sin(a0) - 144*L^2*a0^2*dtheta^2*m*sin(a0) + 144*L*a0^3*g*m*cos(phi + theta) - 144*L*a0^4*g*m*sin(phi + theta) - 72*L*a0^4*g*m2*sin(phi + theta) + 6*D^2*a0^5*da0*dtheta*m + 6*D^2*a0^5*da0*dtheta*m2 - 144*L^2*a0*da0*dtheta*m*cos(a0) - 72*L^2*a0^3*da0*dtheta*m2*cos(a0) - 144*L^2*a0^2*da0*dtheta*m*sin(a0))/(144*a0^5);
% f1 = (72*a0^5*(2*D^2*a0^3*m + D^2*a0^3*m2 - 48*L^2*m*sin(a0) + 48*L^2*a0*m + 24*L^2*a0*m2 - 24*L^2*a0*m2*cos(a0)))/(4320*L^4*m^2 + D^4*a0^8*m^2 + 4320*L^4*a0^2*m^2 + 2592*L^4*a0^2*m2^2 + 720*L^4*a0^4*m^2 + 216*L^4*a0^4*m2^2 + 3456*L^4*m^2*cos(a0) - 7776*L^4*m^2*cos(2*a0) - 17280*L^4*a0*m^2*sin(a0) + 144*D^2*L^2*a0^4*m^2 + 72*D^2*L^2*a0^4*m2^2 + 72*D^2*L^2*a0^6*m^2 + 36*D^2*L^2*a0^6*m2^2 + 8640*L^4*a0^2*m^2*cos(a0) - 3456*L^4*a0^2*m2^2*cos(a0) - 5184*L^4*a0*m^2*sin(2*a0) + 576*L^4*a0^3*m^2*sin(a0) - 1728*L^4*a0^3*m2^2*sin(a0) + 2*D^4*a0^8*m*m2 + 4320*L^4*a0^2*m*m2 + 1440*L^4*a0^4*m*m2 + 864*L^4*a0^2*m^2*cos(2*a0) + 864*L^4*a0^2*m2^2*cos(2*a0) - 216*L^4*a0^4*m2^2*cos(2*a0) + 864*L^4*a0^3*m2^2*sin(2*a0) - 576*D^2*L^2*a0^3*m^2*sin(a0) + 48*D^2*L^2*a0^5*m^2*sin(a0) - 72*D^2*L^2*a0^5*m2^2*sin(a0) - 10368*L^4*a0*m*m2*sin(a0) + 144*D^2*L^2*a0^4*m*m2 + 144*D^2*L^2*a0^6*m*m2 + 288*L^4*a0^4*m*m2*cos(a0) + 5184*L^4*a0*m*m2*sin(2*a0) - 3456*L^4*a0^3*m*m2*sin(a0) - 4320*L^4*a0^2*m*m2*cos(2*a0) - 864*L^4*a0^3*m*m2*sin(2*a0) + 432*D^2*L^2*a0^4*m^2*cos(a0) - 72*D^2*L^2*a0^4*m2^2*cos(a0) + 144*D^2*L^2*a0^4*m*m2*cos(a0) + 24*D^2*L^2*a0^6*m*m2*cos(a0) - 288*D^2*L^2*a0^3*m*m2*sin(a0) - 144*D^2*L^2*a0^5*m*m2*sin(a0));
% f2 = - (36*(24*L^2*m + D^2*a0^4*m + D^2*a0^4*m2 + 12*L^2*a0^2*m + 12*L^2*a0^2*m2 - 24*L^2*m*cos(a0) - 24*L^2*a0*m*sin(a0) - 12*L^2*a0^2*m2*cos(a0))*(8*L^2*da0^2*m - 8*L^2*da0^2*m*cos(a0) + 2*L^2*a0^2*da0^2*m + 2*L^2*a0^2*da0^2*m2 + 8*L^2*a0^2*da0*dtheta*m + 4*L^2*a0^2*da0*dtheta*m2 - L*a0^4*g*m2*cos(a0 + phi + theta) - 2*L*a0^3*g*m*sin(a0 + phi + theta) - 8*L^2*a0*da0^2*m*sin(a0) + 2*L^2*a0^2*da0^2*m*cos(a0) - 2*L^2*a0^2*da0^2*m2*cos(a0) - L^2*a0^3*da0^2*m2*sin(a0) + 2*L*a0^4*g*m*cos(phi + theta) + L*a0^4*g*m2*cos(phi + theta) + 2*L*a0^3*g*m*sin(phi + theta) - 12*L^2*a0*da0*dtheta*m*sin(a0) + 4*L^2*a0^2*da0*dtheta*m*cos(a0) - 4*L^2*a0^2*da0*dtheta*m2*cos(a0) - 2*L^2*a0^3*da0*dtheta*m2*sin(a0)))/(a0*(4320*L^4*m^2 + D^4*a0^8*m^2 + 4320*L^4*a0^2*m^2 + 2592*L^4*a0^2*m2^2 + 720*L^4*a0^4*m^2 + 216*L^4*a0^4*m2^2 + 3456*L^4*m^2*cos(a0) - 7776*L^4*m^2*cos(2*a0) - 17280*L^4*a0*m^2*sin(a0) + 144*D^2*L^2*a0^4*m^2 + 72*D^2*L^2*a0^4*m2^2 + 72*D^2*L^2*a0^6*m^2 + 36*D^2*L^2*a0^6*m2^2 + 8640*L^4*a0^2*m^2*cos(a0) - 3456*L^4*a0^2*m2^2*cos(a0) - 5184*L^4*a0*m^2*sin(2*a0) + 576*L^4*a0^3*m^2*sin(a0) - 1728*L^4*a0^3*m2^2*sin(a0) + 2*D^4*a0^8*m*m2 + 4320*L^4*a0^2*m*m2 + 1440*L^4*a0^4*m*m2 + 864*L^4*a0^2*m^2*cos(2*a0) + 864*L^4*a0^2*m2^2*cos(2*a0) - 216*L^4*a0^4*m2^2*cos(2*a0) + 864*L^4*a0^3*m2^2*sin(2*a0) - 576*D^2*L^2*a0^3*m^2*sin(a0) + 48*D^2*L^2*a0^5*m^2*sin(a0) - 72*D^2*L^2*a0^5*m2^2*sin(a0) - 10368*L^4*a0*m*m2*sin(a0) + 144*D^2*L^2*a0^4*m*m2 + 144*D^2*L^2*a0^6*m*m2 + 288*L^4*a0^4*m*m2*cos(a0) + 5184*L^4*a0*m*m2*sin(2*a0) - 3456*L^4*a0^3*m*m2*sin(a0) - 4320*L^4*a0^2*m*m2*cos(2*a0) - 864*L^4*a0^3*m*m2*sin(2*a0) + 432*D^2*L^2*a0^4*m^2*cos(a0) - 72*D^2*L^2*a0^4*m2^2*cos(a0) + 144*D^2*L^2*a0^4*m*m2*cos(a0) + 24*D^2*L^2*a0^6*m*m2*cos(a0) - 288*D^2*L^2*a0^3*m*m2*sin(a0) - 144*D^2*L^2*a0^5*m*m2*sin(a0))) - (72*a0^5*(2*D^2*a0^3*m + D^2*a0^3*m2 - 48*L^2*m*sin(a0) + 48*L^2*a0*m + 24*L^2*a0*m2 - 24*L^2*a0*m2*cos(a0))*(beta*da0 + a0*k + (L*g*(2*m*sin(phi + theta) + m2*sin(phi + theta) - m2*sin(a0 + phi + theta)))/(2*a0^2) + (L^2*dtheta^2*(4*a0*m + 2*a0*m2 - 6*m*sin(a0) - a0^2*m2*sin(a0) + 2*a0*m*cos(a0) - 2*a0*m2*cos(a0)))/(2*a0^4) - (L^2*da0^2*(24*a0*m + 12*a0*m2 + 2*a0^3*m + 3*a0^3*m2 - 60*m*sin(a0) + 3*a0^3*m2*cos(a0) + 6*a0^2*m*sin(a0) - 12*a0^2*m2*sin(a0) + 36*a0*m*cos(a0) - 12*a0*m2*cos(a0)))/(6*a0^6) + (L*g*m2*cos(a0 + phi + theta))/(2*a0) + (2*L*g*m*cos(a0 + phi + theta))/a0^3 + (L*g*m*sin(a0 + phi + theta))/a0^2 - (2*L*g*m*cos(phi + theta))/a0^3))/(4320*L^4*m^2 + D^4*a0^8*m^2 + 4320*L^4*a0^2*m^2 + 2592*L^4*a0^2*m2^2 + 720*L^4*a0^4*m^2 + 216*L^4*a0^4*m2^2 + 3456*L^4*m^2*cos(a0) - 7776*L^4*m^2*cos(2*a0) - 17280*L^4*a0*m^2*sin(a0) + 144*D^2*L^2*a0^4*m^2 + 72*D^2*L^2*a0^4*m2^2 + 72*D^2*L^2*a0^6*m^2 + 36*D^2*L^2*a0^6*m2^2 + 8640*L^4*a0^2*m^2*cos(a0) - 3456*L^4*a0^2*m2^2*cos(a0) - 5184*L^4*a0*m^2*sin(2*a0) + 576*L^4*a0^3*m^2*sin(a0) - 1728*L^4*a0^3*m2^2*sin(a0) + 2*D^4*a0^8*m*m2 + 4320*L^4*a0^2*m*m2 + 1440*L^4*a0^4*m*m2 + 864*L^4*a0^2*m^2*cos(2*a0) + 864*L^4*a0^2*m2^2*cos(2*a0) - 216*L^4*a0^4*m2^2*cos(2*a0) + 864*L^4*a0^3*m2^2*sin(2*a0) - 576*D^2*L^2*a0^3*m^2*sin(a0) + 48*D^2*L^2*a0^5*m^2*sin(a0) - 72*D^2*L^2*a0^5*m2^2*sin(a0) - 10368*L^4*a0*m*m2*sin(a0) + 144*D^2*L^2*a0^4*m*m2 + 144*D^2*L^2*a0^6*m*m2 + 288*L^4*a0^4*m*m2*cos(a0) + 5184*L^4*a0*m*m2*sin(2*a0) - 3456*L^4*a0^3*m*m2*sin(a0) - 4320*L^4*a0^2*m*m2*cos(2*a0) - 864*L^4*a0^3*m*m2*sin(2*a0) + 432*D^2*L^2*a0^4*m^2*cos(a0) - 72*D^2*L^2*a0^4*m2^2*cos(a0) + 144*D^2*L^2*a0^4*m*m2*cos(a0) + 24*D^2*L^2*a0^6*m*m2*cos(a0) - 288*D^2*L^2*a0^3*m*m2*sin(a0) - 144*D^2*L^2*a0^5*m*m2*sin(a0));
E_des = m*g*L/2 + m2*g*L/2;
E_tilde = E-E_des;



% kE=3;
% kD=1.1;
% kP=1.25;


% fctchg_m=1;
% fctchg_L=1;

%%LOOKS TO BE WORKING FOR
% fctchg_m=0.1;
% fctchg_L=0.1;
% 
% phi = 0;
% D = 0.1*fctchg_L;
% L = 1*fctchg_L;
% m = 1*fctchg_m;
% g =9.81;
% beta = 0.05;
% k = 0.1;

% kE=850;
% kD=0.0005;
% kP=2.3;
%%%%%%%%%%%%%%%%%%%%%%%%%
% testing different values

% kE=5720;
% kD=0.0075;
% kP=0.45;

% for m2=0.01
% kE=12750;
% kD=0.0425;
% kP=0.925;

% kE=10750;
% kD=0.0425;
% kP=1.125;

% kE=2150;
% kD=0.0525;
% kP=1.0525;

% kE=9500;
% kD=0.075;
% kP=0.85;

% works ok for m=0.5, L=0.5
% kE=10;
% kD=5;
% kP=12;

%Working numbers%%
kE=54498.14;
kD=0.75;
kP=1.2;
%%%%%%%%%%%%%%%%%%

Tau = (-da0 +kE*E_tilde*beta*da0 - kD*f2 - kP*a0)/(kE*E_tilde + kD*f1);
% [T, a0, theta, Tau];
% Tau = sat(Tau,-5,5);

xdoubledot=[da0; dtheta; inv(D_mtx)*([1;0]*Tau - C_mtx*daa - G_vect - K*aa -Damp*daa)];

end


%% LQR function for ode

function xdoubledot=f_x_LQR(T,y)
global k beta
phi = 0;
D = 0.024;%*fctchg_L;
L = 0.101;%1*fctchg_L;
m = 0.045;%*fctchg_m;
g =9.81;
% beta = 3e-7;
% k = 2e-5;
m2=0.00;

a0=y(1);
% theta=y(2);
% theta=mod(y(2),2*pi);
theta=mod(pi+y(2),2*pi)-pi;
da0=y(3);
dtheta=y(4);

% remedy for devision by zero
thresh = 1e-3;
if a0<thresh && a0>=0 && da0<0
    a0 = -thresh*10 ;
elseif a0<thresh && a0>=0 && da0>0
    a0 = thresh*10;
elseif a0>-thresh && a0<=0 && da0>0
    a0 = thresh*10;
elseif a0>-thresh && a0<=0 && da0<0
    a0 = -thresh*10 ;
end

aa=[a0;theta];
daa = [da0;dtheta];


%% Dynamics

%% Uniform mass

% D_mtx = [ (m*(72*L^2*a0 + D^2*a0^5 + 12*L^2*a0^3 - 144*L^2*sin(a0) + 72*L^2*a0*cos(a0)))/(36*a0^5), (m*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0)))/(24*a0^4);
%              (m*(24*L^2 + D^2*a0^4 + 12*L^2*a0^2 - 24*L^2*cos(a0) - 24*L^2*a0*sin(a0)))/(24*a0^4),                                (m*(24*L^2*a0 + D^2*a0^3 - 24*L^2*sin(a0)))/(12*a0^3)];
% 
% C_mtx = [                                                                 -(L^2*da0*m*(12*a0 - 30*sin(a0) + 18*a0*cos(a0) + a0^3 + 3*a0^2*sin(a0)))/(3*a0^6), (L^2*dtheta*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4;
%          -(L^2*m*(4*da0 + a0^2*da0 + 2*a0^2*dtheta - 4*da0*cos(a0) + a0^2*da0*cos(a0) + a0^2*dtheta*cos(a0) - 4*a0*da0*sin(a0) - 3*a0*dtheta*sin(a0)))/a0^5,   -(L^2*da0*m*(2*a0 - 3*sin(a0) + a0*cos(a0)))/a0^4];
% 
% 
% 
% G_vect = [(2*L*g*m*(cos(a0 + phi + theta) - cos(phi + theta) + a0*sin(phi + theta)))/a0^3 + (L*g*m*(sin(a0 + phi + theta) - sin(phi + theta)))/a0^2;
%                                                                     -(L*g*m*(sin(phi + theta) - sin(a0 + phi + theta) + a0*cos(phi + theta)))/a0^2];
% 

%% with m2 at the tip
D_mtx = [(2*D^2*a0^5*m + 3*D^2*a0^5*m2 + 24*L^2*a0^3*m + 36*L^2*a0^3*m2 - 288*L^2*m*sin(a0) + 144*L^2*a0*m + 72*L^2*a0*m2 - 72*L^2*a0^2*m2*sin(a0) + 144*L^2*a0*m*cos(a0) - 72*L^2*a0*m2*cos(a0))/(72*a0^5), (D^2*m)/24 + (D^2*m2)/24 + (2*L^2*m*((sin(a0/2)^2 - (a0*sin(a0))/2)/a0^2 + 1/4))/a0^2 + (L^2*m2*sin(a0/2)^2)/a0^2;
                                                                                  (D^2*m)/24 + (D^2*m2)/24 + (2*L^2*m*((sin(a0/2)^2 - (a0*sin(a0))/2)/a0^2 + 1/4))/a0^2 + (L^2*m2*sin(a0/2)^2)/a0^2,              (D^2*m)/12 + (D^2*m2)/24 - (4*L^2*m*(sin(a0)/(2*a0) - 1/2))/a0^2 - (2*L^2*m2*(cos(a0)/2 - 1/2))/a0^2];


C_mtx = [                                                                                                                                 -(L^2*da0*(24*a0*m + 12*a0*m2 + 2*a0^3*m + 3*a0^3*m2 - 60*m*sin(a0) + 3*a0^3*m2*cos(a0) + 6*a0^2*m*sin(a0) - 12*a0^2*m2*sin(a0) + 36*a0*m*cos(a0) - 12*a0*m2*cos(a0)))/(6*a0^6), (L^2*dtheta*(4*a0*m + 2*a0*m2 - 6*m*sin(a0) - a0^2*m2*sin(a0) + 2*a0*m*cos(a0) - 2*a0*m2*cos(a0)))/(2*a0^4);
     -(L^2*(8*da0*m + 2*a0^2*da0*m + 2*a0^2*da0*m2 + 4*a0^2*dtheta*m + 2*a0^2*dtheta*m2 - 8*da0*m*cos(a0) - 8*a0*da0*m*sin(a0) - 6*a0*dtheta*m*sin(a0) + 2*a0^2*da0*m*cos(a0) - 2*a0^2*da0*m2*cos(a0) + 2*a0^2*dtheta*m*cos(a0) - 2*a0^2*dtheta*m2*cos(a0) - a0^3*da0*m2*sin(a0) - a0^3*dtheta*m2*sin(a0)))/(2*a0^5),   -(L^2*da0*(4*a0*m + 2*a0*m2 - 6*m*sin(a0) - a0^2*m2*sin(a0) + 2*a0*m*cos(a0) - 2*a0*m2*cos(a0)))/(2*a0^4)];                                                                             
                                                                              
                                                                              
G_vect = [(L*g*(4*m*cos(a0 + phi + theta) - 4*m*cos(phi + theta) + 2*a0*m*sin(a0 + phi + theta) - a0*m2*sin(a0 + phi + theta) + a0^2*m2*cos(a0 + phi + theta) + 2*a0*m*sin(phi + theta) + a0*m2*sin(phi + theta)))/(2*a0^3);
                                                               -(L*g*(2*m*sin(phi + theta) - 2*m*sin(a0 + phi + theta) - a0*m2*cos(a0 + phi + theta) + 2*a0*m*cos(phi + theta) + a0*m2*cos(phi + theta)))/(2*a0^2)];




K = [k 0; 0 0];
Damp = [beta 0; 0 0];


% theta=theta+2/180*pi;
a02=a0;%+0.00005*rand;
theta2=theta;


da02=da0;%+0*rand*sin(50*pi*T);
dtheta2=dtheta;%+0*rand*sin(50*pi*T);
% da0=da0+0.1*sin(2*pi*T);

aa2=[a02;theta2];
daa2 = [da02;dtheta2];


% LQR -64.9294 -193.7984   -6.0362  -16.9326
% Tau = -1*[   -1.4760*1   -4.3072*1  -0.1378   -0.3661]*[aa2;daa2];
Tau = -1*[  -11.4947*1  -34.3632*1   -1.0437   -2.8487]*[aa2;daa2];
% 2.1384    6.5361    0.1915    0.4312
% Tau = -1*[ -231.9544 -601.5278  -22.8877  -56.3118]*[aa;daa];
% Tau = -1*[  -86.6566 -255.1605  -23.9322  -66.4996]*[aa;daa]; %k=0.5
% Tau = -1*[ -84.7415 -251.1944  -23.5412  -65.4645]*[aa;daa]; %k=1
% Tau = -1*[-98.4903 -285.7157  -26.9272  -74.4742]*[aa;daa]; %k=2

% 
% if T >4 && T<4.01
%     Tau=0.1;
% end

%limiting the tau
if Tau>2.5
    Tau=2.5;
elseif Tau<-2.5
    Tau=-2.5;
end

% inserting a disturbance at 4s
if T >4 && T<4.01
    Tau=0.51;
end

% Tau=3*k;
% Tau = -0.0001*a0 -0.1*theta;% - 0.001*da0 - 0.001*dtheta; 
[T,T, a0, theta, Tau]
xdoubledot=[da0; dtheta; inv(D_mtx)*([1;0]*Tau - C_mtx*daa - G_vect - K*aa -Damp*daa)];
end


%% ROA check

function [val,err] = check_ROA(y,roa)

a0=y(1);
theta=mod(pi+y(2),2*pi)-pi;
da0=y(3);
dtheta=y(4);

test = roa - [a0,theta,da0,dtheta];
test2=test(:,1).^2 + test(:,2).^2+test(:,3).^2+test(:,4).^2;
err = min(test2);

if err<=5e-2
    val = 1;
else
    val = 0;
end

end
% This is the main file for the via point which calls via_points_match_VA.m 
%  
%  INPUT. The file requires an k x n matrix, i.e., where k is the number of 
%  joints or cartesian coordinates and n are all the points (initial, 
%  intermidiate and final), an vector of size n-1 that specifies the 
%  duration of each segment, the increment of time within each segment,
%  and the type of motion ('cyclic' or 'prescribed' [V0, Vf]).
 

function main_via_point
clc, close all; clear all;
L1=1.6; L2=1.4; %Lengths of Links MODIFY 

% Assume a configuration of the inverse kinematics %CHOOSE ONE OF THEM
 config=1;   %Elbow down 
 config=-1; %Elbow up 

%Inverse kinematics of initial configuration  %INPUT YOUR POINTS
X=[1 1 2 2; % first row is the x-coordinate
    1 2 2 1]; % second row is the y-coordinate

%Duration INPUT YOUR DURATION
duration=[10/3,10/3,10/3];

if length(X(1,:))-1 ~= length(duration)
    disp('You must specify the duration to each segment')
    return
end

theta=zeros(2,length(X(1,:))); %allocate variables
for i=1:length(X(1,:))
    theta(:,i)=inverse(X(1,i),X(2,i),L1,L2,config(1))';
end

stepsize=.01; %delta t
[position,velocity,acceleration,time]=via_points_match_VA(theta, duration, stepsize, 'prescribed',[0,0]);

%__________ Plotting Graphs and Configurations 
figure; FK_test2(position*pi/180,L1,L2)
plot(X(1,:), X(2,:),'r.'); 
figure;
h=plot(time, position(1,:), 'b', time, position(2,:), 'r', time, velocity(1,:), 'b:',  time, velocity(2,:), 'r:', time, acceleration(1,:), 'b-.', time, acceleration(2,:), 'r-.');
legend(h(1:2:end),'position Joint 1 (deg)', 'position Joint 2 (deg)', 'velocity Joint 1 (deg/s)', 'velocity Joint 2 (deg/s)', 'acceleration Joint 1 (deg/s^2)' , 'acceleration Joint 2 (deg/s^2)','Location', 'SouthEast') 
xlabel('time (s)','fontsize',18);
ylabel('Disp. (deg)    Vel. (deg/sec)    Acc. (deg/sec^2)','fontsize',18);
set(gca,'FontSize',16)

%All the functions listed below are for plotting the configurarions of the
%RR manipulator (DO NOT MODIFY)
function FK_test2(Tjj,L1,L2)

% Generate joint trajectory 
[r,c]=size(Tjj);
% Generate the trajectories of the end-effector
% and the distant end of the first link.
DH = [0 0 0; 0 L1 0; 0 L2 0];
v = [1 1 -1]';
DH1 = DH(1:2,:);
v1 = [1 -1]';
for i = 1:c,
   FK3 = kinematics(Tjj(1:2,i),v,DH);
   FK2 = kinematics(Tjj(1:1,i),v1,DH1);
   R3(:,2*i-1:2*i) = FK3(1:2,1:2);
   Tj3(:,i) = FK3(1:2,4);
   Tj2(:,i) = FK2(1:2,4);
end

% Plot trajectories and robot configurations
figure(1)
plot(Tj3(1,:), Tj3(2,:),'k:')
hold on;
q1 = [0 Tj2(1,1) Tj3(1,1)];
q2 = [0 Tj2(2,1) Tj3(2,1)];
plot(q1(1:2),q2(1:2),'.-',q1(2:3),q2(2:3),'b-') % plot 1st configuration 
Q1 = Tj3(:,1) + 0.07*R3(:,1) + 0.05*R3(:,2);
Q2 = Tj3(:,1) + 0.05*R3(:,2);
Q3 = Tj3(:,1) - 0.05*R3(:,2);
Q4 = Tj3(:,1) + 0.07*R3(:,1) - 0.05*R3(:,2);
Q = [Q1 Q2 Q3 Q4];
plot(Q(1,:),Q(2,:),'-') % plot the 1st gripper


for i = 1:(c/50),
   ii = i*50;
   q1 = [0 Tj2(1,ii) Tj3(1,ii)];
   q2 = [0 Tj2(2,ii) Tj3(2,ii)];
   plot(q1(1:2),q2(1:2),'.-',q1(2:3),q2(2:3),'b-')% plot (i*20)th configuration 
   Q1 = Tj3(:,ii) + 0.07*R3(:,2*ii-1) + 0.05*R3(:,2*ii);
   Q2 = Tj3(:,ii) + 0.05*R3(:,2*ii);
   Q3 = Tj3(:,ii) - 0.05*R3(:,2*ii);
   Q4 = Tj3(:,ii) + 0.07*R3(:,2*ii-1) - 0.05*R3(:,2*ii);
   Q = [Q1 Q2 Q3 Q4];
   plot(Q(1,:),Q(2,:),'-') % plot the (i*20)th gripper
end

% Draw a robot base and print figure title, etc.
plot(-0.25:0.5/100:0.25,zeros(1,101),'linewidth',3)
plot(-0.25:0.5/100:0.25,-0.04*ones(1,101),'linewidth',3)
axis equal
text(-0.13,-0.3,'BASE')
title('Robot Configurations','fontsize', 14)
xlabel('x0','fontsize', 12)
ylabel('y0','fontsize', 12)
hold on;

function theta=inverse(x,y,L1,L2,config)
    c2=(x^2+y^2-L1^2-L2^2)/(2*L1*L2);
    s2=config*sqrt(1-c2^2); %the parameter config defines the configuration of the manipulator 
    theta_2=atan2(s2,c2);
    k1=L1+L2*c2;
    k2=L2*s2;
    theta_1=atan2(y,x)-atan2(k2,k1);
    theta=[theta_1, theta_2]*180/pi;
return

function FK = kinematics(tjj,v,DH)

n = size(tjj)*[1 0]';
n1 = length(v);
FK = eye(4);
for i = 1:n,
   alpha = DH(i,1);
   a = DH(i,2);
   if v(i) == 1,
      d = DH(i,3);
      ang = tjj(i);
   else
      d = tjj(i);
      ang = DH(i,3);
   end
   FK = FK*T_basic(alpha,a,d,ang);
end
if n1 > n,
   alpha = DH(n1,1);
   a = DH(n1,2);
   d = DH(n1,3);
   ang = 0;
   FK = FK*T_basic(alpha,a,d,ang);
end

function Tb = T_basic(alpha,a,d,ang)
 
Tb = eye(4);
cc = cos(ang);
sc = sin(ang);
ca = cos(alpha);
sa = sin(alpha);
Tb(1,1) = cc;
Tb(1,2) = -sc;
Tb(1,4) = a;
Tb(2,1) = sc*ca;
Tb(2,2) = cc*ca;
Tb(2,3) = -sa;
Tb(2,4) = -sa*d;
Tb(3,1) = sc*sa;
Tb(3,2) = cc*sa;
Tb(3,3) = ca;
Tb(3,4) = ca*d;


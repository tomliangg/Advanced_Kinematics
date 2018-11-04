% main_segment.m
% This program shows how to implement the trajectory generation schemes 
% (cubic, quintic and trapezoidal) for one segment. 
%
% Inputs for generating the trajectory 
%       theta_0 and theta_f.- initial and final configurations input them 
%                             in a vector form, for example if there are 
%                             two 2 joints, theta_0=[theta_01 theta_02] 
%       tf.- duration of the trajectory
%       step.- it is the step of time, i.e., t=0:step:tf
%             The number of intervals is given by tf/(50*step)+1

% Written by Flavio Firmani, Fall 2016

close all; clear all 

%Inputs
%Select type of Motion by UNCOMMENTING the desired scheme 
% scheme='cubic';
% scheme='quintic';
scheme='trapezoidal';

%Duration
tf=6;  %MODIFY, duration time in seconds
step=0.01; %MODIFY if you want more configurations being shown

%Initial and Final Configurations
theta_0=[-150.6426 90.3837]; %MODIFY, joint angles in degs for initial configurations
theta_f=[44.6652 34.5471]; %MODIFY, joint angles in degs for final confirations

%Plotting Graphs
n=length(theta_0); %Number of joints
for i=1:n   
   
    if strcmp(scheme,'cubic')
        [d,v,a,t]=cubic_scheme(theta_0(i),theta_f(i),tf,step);
    elseif strcmp(scheme,'quintic')
        [d,v,a,t]=quintic_scheme(theta_0(i),theta_f(i),tf,step);
    elseif strcmp(scheme,'trapezoidal') 
        blend=2;  %Factor of blending
        [d,v,a,t]=trapezoidal_scheme(theta_0(i),theta_f(i),tf,step,blend);
    end
    
    % Plotting displacement graph
    hold on;
    figure(1)
    plot(t,d)
    title('Time vs Displacement','fontsize', 14)
    xlabel('t (s)','fontsize', 12)
    ylabel('d (deg)','fontsize', 12)
    if i==1
        text(t(end),d(end),'Joint 1')
    else
        text(t(end),d(end),'Joint 2')
    end
    axis square
    
    % Plotting velocity graph
    hold on;
    figure(2)
    plot(t,v)
    title('Time vs Velocity','fontsize', 14)
    xlabel('t (s)','fontsize', 12)
    ylabel('v (deg/s)','fontsize', 12)
    if i==1
        text(t(end),v(end),'Joint 1')
    else
        text(t(end),v(end),'Joint 2')
    end
    axis square
    
    % Plotting accleration graph
    hold on;
    figure(3)
    plot(t,a)
    title('Time vs Acceleration','fontsize', 14)
    xlabel('t (s)','fontsize', 12)
    ylabel('a (deg/s^2)','fontsize', 12)
    if i==1
        text(t(end),a(end),'Joint 1')
    else
        text(t(end),a(end),'Joint 2')
    end
    axis square
end


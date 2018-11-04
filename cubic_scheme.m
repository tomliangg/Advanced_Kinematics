% This function generates a trajectory using the cubic polynomial scheme
%
% [d,v,a,t]=cubic_scheme(theta_0,theta_f,tf,step)
%
% Inputs for generating the trajectory:  
%       theta_0 and theta_f.- initial and final configurations input them 
%                             in a vector form, for example if there are 
%                             two 2 joints, theta_0=[theta_01 theta_02] 
%       tf.- duration of the trajectory
%       step.- it is the step of time, i.e., t=0:step:tf 
%
% Outputs: 
%         d - displacement
%         v - velocity  
%         a - acceleration
%         t - time

function [d,v,a,t]=cubic_scheme(theta_0,theta_f,tf,step)

n=length(theta_0);
for i=1:n
    % Determning the value of the coefficients for each joint
    a0=theta_0(i);
    a1=0;
    a2=3/tf^2*(theta_f(i)-theta_0(i));
    a3=-2/tf^3*(theta_f(i)-theta_0(i));

    % Identifying the values of displacement, velocity, and acceleration of the joints
    t=0:step:tf;
    d=a0+a1.*t+a2.*t.^2+a3.*t.^3;
    v=a1+2.*a2.*t+3.*a3.*t.^2;
    a=2.*a2+6.*a3.*t;
end

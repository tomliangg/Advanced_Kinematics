% This function generates a trajectory using the trapezoidal scheme
%
% [d,v,a,t]=trapezoidal_scheme(theta_0,theta_f,tf,step,blend)
%
% Inputs for generating the trajectory 
%       theta_0 and theta_f.- initial and final configurations input them 
%                             in a vector form, for example if there are 
%                             two 2 joints, theta_0=[theta_01 theta_02] 
%       tf.- duration of the trajectory
%       step.- it is the step of time, i.e., t=0:step:tf 
%       blend.- factor of blending 
%
% Outputs: 
%         d - displacement
%         v - velocity  
%         a - acceleration
%         t - time

function [d,v,a,T]=trapezoidal_scheme(theta_0,theta_f,tf,step,blend)

n=length(theta_0);
for i=1:n
    
    %Determine angular acceleration at tb and time tb.
    alpha_b=4*(theta_f(i)-theta_0(i))/tf^2*blend;
    tb=min(roots([alpha_b -alpha_b*tf theta_f(i)-theta_0(i)]));
    
    
    d=[]; v=[]; a=[]; T=[];
    
    %Acceleration Phase
    for t=0:step:tf
        if (t>=0) && (t<=tb)
            d=[d theta_0(i)+.5*alpha_b*t^2];
            v=[v alpha_b*t];
            a=[a alpha_b];
            T=[T t];
        end
        
        %Constant Velocity Phase
        if (t>tb) && (t<=(tf-tb))
            d=[d theta_0(i)+alpha_b*tb*(t-tb/2)];
            v=[v alpha_b*tb];
            a=[a 0];
            T=[T t];
        end
        
        %Decceleration Phase
        if (t>(tf-tb)) && (t<=tf)
            d=[d theta_f(i)-.5*alpha_b*(tf-t)^2];
            v=[v alpha_b*(tf-t)];
            a=[a -alpha_b];
            T=[T t];
        end
    end  
end

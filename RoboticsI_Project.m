%% *** Robot (kinematic) model parameters *** 
clear all; 
close all; 
clc

syms q
l0 = 5;
l1 = 5;
l2 = 5;
l3 = 0;
l4 = 3;
l5 = 6;

%% *** sampling period *** 
%% *** for the robot motion, kinematic simulation: 
dt = 0.001; %dt = 0.001; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf=10.0; 	% 10sec duration of motion 
t=0:dt:Tf;  

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory  
xd0 = 15.0;	
xd1 = 15.0; 
yd0 = 0.00; 
yd1 = 5.00; 
zd0 = -10.0;
zd1 = -10.0;

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   
xd(1) = xd0; 
yd(1) = yd0; 
zd(1) = zd0;
v(1) = yd(1);
w(1) = 0;
lambda_x = (xd1-xd0)/Tf; 
lambda_y = (yd1-yd0)/Tf; 
lambda_z = (zd1-zd0)/Tf;
kmax=Tf/dt + 1; 
for k=2:101,
   v(k) = - (1/500)*(yd1 - yd0)*(k*0.1)^3 + 0.03*(yd1 - yd0)*(k*0.1)^2 + yd1;
   w(k) = - (3/500)*(yd1 - yd0)*(k*0.1)^2 + 0.06*(yd1 - yd0)*(k*0.1);
end
for k=2:kmax; 
   xd(k) = xd(k-1) + lambda_x*dt;    
   yd(k) = yd(k-1) + lambda_y*dt;
   zd(k) = zd(k-1) + lambda_z*dt;
end  
 
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%% compute the reference joint-motion vectors: 
%% {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%% and reference joint (angular) velocities {qd_1(k,i)} 
for k = 1:kmax
%     q1(k) = real(atan((l2/(sqrt((xd(k) - l1)^2 + (zd(k) + l0)^2)))/(sqrt(1 - (l2^2)/((xd(k) - l1) + (zd(k) + l0)))) - atan((xd(k) - l1)/(zd(k) + l0))));
    q1(k) = 2*atan((l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))/(l2 - l1 + xd(k)));
    c1(k) = cos(q1(k));
    s1(k) = sin(q1(k));
        
    M = (xd(k) - l1 - l2*c1(k) - l3*s1(k))/s1(k);
    mm = M^2;
%     q3(k) = real(acos((mm + (yd(k))^2 - (l4)^2 - (l5)^2)/(2*l4*l5)));
    q3(k) = -acos(1/2/l4/l5*(1/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))^2*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2)))))^2 - l4^2 - l5^2 + yd(k)^2));
    c3(k) = cos(q3(k));
    s3(k) = sin(q3(k));
    
    q2(k) = -atan(1/(l5*(1 - 1/4/l4^2/l5^2*(1/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))^2*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2)))))^2 - l4^2 - l5^2 + yd(k)^2)^2)^(1/2) + 1/yd(k)/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))*(l4 + 1/2/l4*(1/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))^2*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2)))))^2 - l4^2 - l5^2 + yd(k)^2))*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))))*(l4 + 1/2/l4*(1/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))^2*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2)))))^2 - l4^2 - l5^2 + yd(k)^2) - l5/yd(k)/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))*(1 - 1/4/l4^2/l5^2*(1/sin(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2))))^2*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2)))))^2 - l4^2 - l5^2 + yd(k)^2)^2)^(1/2)*(l1 - xd(k) + l2*cos(2*atan(1/(l2 - l1 + xd(k))*(l0 + zd(k) + (l0^2 + 2*l0*zd(k) + l1^2 - 2*l1*xd(k) - l2^2 + xd(k)^2 + zd(k)^2)^(1/2)))))));
%     q2(k) = real(atan((l4 + l5*c3(k) - ((M*l5*s3(k))/yd(k)))/((M*(l4 + l5*c3(k))/yd(k)) + l5*s3(k))));
    c2(k) = cos(q2(k));
    s2(k) = sin(q2(k));
    c23(k) = cos(q2(k) + q3(k));
    s23(k) = sin(q2(k) + q3(k));
end   
dq2(1) = (q2(1)-0)/dt;
dq3(1) = (q3(1)-0)/dt;
for k=2:kmax;  
   dq2(k) = (q2(k) - q2(k-1))./dt; 
   dq3(k) = (q3(k) - q3(k-1))./dt; 
end; 
    dq1 = 0 ; % q1 is constant for a given trajectory, because it's affected by x and z coordinates which are constant throughout the motion
 
%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 
%%(xd1, yd1, zd1) : cartesian position of the 1st link's local reference frame 
for k=1:kmax;
    xd1(k) = l1;
    yd1(k) = 0;
    zd1(k) = -l0; 

    %%(xd2, yd2, zd2) : cartesian position of the 2nd link's local reference frame 

%     xd2(k) = l1+l3*s1 ;   %% l3 = 0 so 2nd link's frame is the same
%     yd2(k) = 0;
%     zd2(k) = -l0-c1*l3;

    %%(xd3, yd3, zd3) : cartesian position of the 3rd link's local reference frame 

    xd2(k) = l1 + c1(k).*l2 + s1(k).*l3 + c2(k).*s1(k).*l4;
    yd2(k) = l4*s2(k);
    zd2(k) = l2*s1(k) - c1(k).*l3 - l0 - c1(k).*c2(k).*l4;
    
    %%(xd4, yd4, zd4) : cartesian position of the 4th link's local reference frame 

    xd3(k) = l1 + c1(k).*l2 + l3*s1(k) + c2(k).*l4*s1(k) + l5*s1(k)*c23(k);
    yd3(k) = l4*s2(k) + l5*s23(k);
    zd3(k) = l2*s1(k) - c1(k).*l3 - l0 - c1(k).*c2(k).*l4 - c1(k).*l5*c23(k);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
save;  %% --> save data to 'matlab.mat' file   

fig1 = figure;
subplot(3,1,1); 
plot(t,xd); 
ylabel('xd (cm)'); 
xlabel('time t (sec)');  

subplot(3,1,2); 
plot(t,yd); 
ylabel('yd (cm)'); 
xlabel('time t (sec)');  

subplot(3,1,3); 
plot(t,zd); 
ylabel('zd (cm)'); 
xlabel('time t (sec)');  

fig2 = figure;
axis([0 15 -15 15 -25 10]) %%set xyz plot axes
axis on 
grid on
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)');
plot3(xd,yd,zd);
grid on;

fig3 = figure;

subplot(1,2,1);
plot(0:0.1:10,v);
ylabel('v (cm/s)');
xlabel('time (sec)');

subplot(1,2,2);
plot(0:0.1:10,w);
ylabel('w (rad/s)');
xlabel('time (sec)');

fig4 = figure;
subplot(2,3,1); 
plot(t,q1); 
ylabel('q1 (rad)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t,q2); 
ylabel('q2 (rad)'); 
xlabel('time t (sec)');    

subplot(2,3,3); 
plot(t,q3); 
ylabel('q3 (rad)'); 
xlabel('time t (sec)');  

subplot(2,3,4); 
plot(t,dq1); 
ylabel('dq1 (rad/s)'); 
xlabel('time t (sec)');  

subplot(2,3,5); 
plot(t,dq2); 
ylabel('dq2 (rad/s)'); 
xlabel('time t (sec)');    

subplot(2,3,6); 
plot(t,dq3); 
ylabel('dq3 (rad/s)'); 
xlabel('time t (sec)');  
 



%%*** stick diagram --> animate robot motion ... (**optional**) 
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...  

fig5 = figure; 
axis([0 15 -15 15 -25 10]) %%set xyz plot axes 
axis on 
hold on 
grid on
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)');
dtk=100; %% plot robot position every dtk samples, to animate its motion 
plot([0],[0],'*');
for k=1:dtk:kmax,    %%% 	
   pause(0.1);	%% pause motion to view successive robot configurations    
   plot3([0,0],[0,0],[0,-l0],'black');
   plot3([0,xd1(k)],[0,yd1(k)],[-l0,zd1(k)],'black');
   plot3(xd1(k),yd1(k),zd1(k),'.red');
   plot3([xd1(k),xd1(k)+c1(k)*l2],[yd1(k),yd1(k)],[zd1(k),zd1(k)+s1(k)*l2],'m');
   plot3(xd1(k)+c1(k)*l2,yd1(k),zd1(k)+s1(k)*l2,'.b');
   plot3([xd1(k)+c1(k)*l2,xd2(k)],[yd1(k),yd2(k)],[zd1(k)+s1(k)*l2,zd2(k)],'c');
   plot3(xd2(k),yd2(k),zd2(k),'.b');
   plot3([xd2(k),xd3(k)],[yd2(k),yd3(k)],[zd2(k),zd3(k)],'m');
   plot3(xd3(k),yd3(k),zd3(k),'r+');  
   plot3(xd3(k),yd3(k),zd3(k),'c*');  
end       
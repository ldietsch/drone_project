% Benjamin Lee
% insert name
% insert name
% AAE561
% 
% Final Project
% Main Program
% 2D Problem

clc
clear all
close all

%% Initialization
T = 1;
delta_t = 0.02;
% delta_t = 0.1;
steps = T/delta_t;

objects = 3;

%% Constraints

p_min  = 0;     
p_max  = 30;
d1_max = 1000;  % m/s
d2_max = 30;    % m/s^2
d3_max = 200;   % m/s^3

R      = 0.25; % Minimum distance

R       = ones(steps+1,1)*R;
d1_max  = ones(steps+1,1)*d1_max;
d2_max  = ones(steps+1,1)*d2_max;

%% Start Points ~~~
x1_0     = 0;
dx1_0    = 0;
d2x1_0   = 0;

y1_0     = 0;
dy1_0    = 0;
d2y1_0   = 0;

x2_0     = 0;
dx2_0    = 0;
d2x2_0   = 0;

y2_0     = 1;
dy2_0    = 0;
d2y2_0   = 0;

x3_0     = 0;
dx3_0    = 0;
d2x3_0   = 0;

y3_0     = 0.5;
dy3_0    = 0;
d2y3_0   = 0;

x1 = [x1_0; dx1_0; d2x1_0];
y1 = [y1_0; dy1_0; d2y1_0];
x2 = [x2_0; dx2_0; d2x2_0];
y2 = [y2_0; dy2_0; d2y2_0];
x3 = [x3_0; dx3_0; d2x3_0];
y3 = [y3_0; dy3_0; d2y3_0];

init = [x1,y1,x2,y2,x3,y3];

% End Points ~~~
x1_f    = 1 - x1_0;
dx1_f   = 0 - dx1_0;
d2x1_f  = 0 - d2x1_0;

y1_f    = 1 - y1_0;
dy1_f   = 0 - dy1_0;
d2y1_f  = 0 - d2y1_0;

x2_f    = 1 - x2_0;
dx2_f   = 0 - dx2_0;
d2x2_f  = 0 - d2x2_0;

y2_f    = 0 - y2_0;
dy2_f   = 0 - dy2_0;
d2y2_f  = 0 - d2y2_0;

x3_f    = 1 - x3_0;
dx3_f   = 0 - dx3_0;
d2x3_f  = 0 - d2x3_0;

y3_f    = 0.5 - y3_0;
dy3_f   = 0 - dy3_0;
d2y3_f  = 0 - d2y3_0;
%% 

A = [1, delta_t, 1/2*delta_t^2;
    0, 1, delta_t;
    0, 0, 1];

B = [1/6*delta_t^3;
    1/2*delta_t^2;
    delta_t];

AA = (A^(steps - 1))*B;

for n = 2:steps - 1
    AA = [AA,(A^(steps - n))*B];
end
AA = [AA,B];

b1 = [x1_f;dx1_f;d2x1_f];
b2 = [y1_f;dy1_f;d2y1_f];
b3 = [x2_f;dx2_f;d2x2_f];
b4 = [y2_f;dy2_f;d2y2_f];
b5 = [x3_f;dx3_f;d2x3_f];
b6 = [y3_f;dy3_f;d2y3_f];

b = [b1, b2, b3, b4, b5, b6];

%% FMINCON
y0 = ones(steps,objects*2);

lb = -d3_max*ones(steps,objects*2);
ub =  d3_max*ones(steps,objects*2);

% options = optimoptions('fmincon','maxfunctionevaluations',1e6);
options = optimoptions('fmincon','algorithm','sqp','maxfunctionevaluations',1e6,'maxiterations',1e6);

% y = fmincon(@(y)norm(AA*y-b),y0);
% y = fmincon(@(y)norm(AA*y-b),y0,[],[],[],[],[],[],[],options);
y = fmincon(@(y)norm(AA*y-b),y0,[],[],[],[],lb,ub,@(y)cons(A,B,y,R,steps,objects,init,p_min,p_max,d1_max,d2_max),options);

cons(A,B,y,R,steps,objects,init,p_min,p_max,d1_max,d2_max)

%% Z matrix assembly
zz = zeros(objects*6,steps+1);

interactions = objects*(objects - 1)/2;

dist    = zeros(interactions,steps+1);
acc     = zeros(objects,steps+1);
vel     = zeros(objects,steps+1);
pos_x   = zeros(objects,steps+1);
pos_y   = zeros(objects,steps+1);

zz(:,1) = init(:);

n = 1;
for i = 1:objects-1
    for k = i+1:objects
        dist(n,1) = sqrt((zz(6*i-5,1)-zz(6*k-5,1))^2 + (zz(6*i-2,1)-zz(6*k-2,1))^2);
        n = n + 1;
    end
end

for n = 1:objects
    acc(n,1)  = sqrt(zz(6*i-3,1)^2 + zz(6*i,1)^2);
    vel(n,1)  = sqrt(zz(6*i-4,1)^2 + zz(6*i-1,1)^2);
    pos_x(n,1)= zz(6*i-5,1);
    pos_y(n,1)= zz(6*i-2,1);
end


for m = 2:steps+1
    for i = 1:objects*2
        zz(3*i-2:3*i,m) = A*zz(3*i-2:3*i,m-1) + B*y(m-1,i);
    end

    n = 1;
    for i = 1:objects-1
        for k = i+1:objects
            dist(n,m) = sqrt((zz(6*i-5,m)-zz(6*k-5,m))^2 + (zz(6*i-2,m) - zz(6*k-2,m))^2);
            n = n + 1;
        end
    end
    
    for n = 1:objects
        acc(n,m)  = sqrt(zz(6*i-3,m)^2 + zz(6*i,m)^2);
        vel(n,m)  = sqrt(zz(6*i-4,m)^2 + zz(6*i-1,m)^2);
        pos_x(n,m)= zz(6*i-5,m);
        pos_y(n,m)= zz(6*i-2,m);
    end
end

% calculate specific energy
    SE = 0;
    for i = 1:objects
       SE = SE + vel(i,:)*vel(i,:)';
    end
    SE

%% Plot Individual Components
% x_scale = [0:delta_t:T];
% figure(1)
% subplot(3,1,1)
% plot(x_scale,zz1(1,:))
% xlabel('time [s]')
% ylabel('X-position')
% 
% subplot(3,1,2)
% plot(x_scale,zz1(2,:))
% xlabel('time [s]')
% ylabel('X-Velocity')
% 
% subplot(3,1,3)
% plot(x_scale,zz1(3,:))
% xlabel('time [s]')
% ylabel('X-Acceleration')
% 
% figure(2)
% subplot(3,1,1)
% plot(x_scale,zz2(1,:))
% xlabel('time [s]')
% ylabel('Y-position')
% 
% subplot(3,1,2)
% plot(x_scale,zz2(2,:))
% xlabel('time [s]')
% ylabel('Y-Velocity')
% 
% subplot(3,1,3)
% plot(x_scale,zz2(3,:))
% xlabel('time [s]')
% ylabel('Y-Acceleration')
% 
% figure(3)
% subplot(3,1,1)
% plot(x_scale,zz3(1,:))
% xlabel('time [s]')
% ylabel('X-position')
% 
% subplot(3,1,2)
% plot(x_scale,zz3(2,:))
% xlabel('time [s]')
% ylabel('X-Velocity')
% 
% subplot(3,1,3)
% plot(x_scale,zz3(3,:))
% xlabel('time [s]')
% ylabel('X-Acceleration')
% 
% figure(4)
% subplot(3,1,1)
% plot(x_scale,zz4(1,:))
% xlabel('time [s]')
% ylabel('Y-position')
% 
% subplot(3,1,2)
% plot(x_scale,zz4(2,:))
% xlabel('time [s]')
% ylabel('Y-Velocity')
% 
% subplot(3,1,3)
% plot(x_scale,zz4(3,:))
% xlabel('time [s]')
% ylabel('Y-Acceleration')

%% Plot Positions
figure(5)
hold on
plot(zz(1,:),zz(4,:),'o')
plot(zz(7,:),zz(10,:),'+')
plot(zz(13,:),zz(16,:),'x')
xlabel('X position')
ylabel('Y position')
hold off

zz(1,steps/2+1)
zz(7,steps/2+1)
zz(13,steps/2+1)
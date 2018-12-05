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

tic

%% Initialization
T = 2;              % Total time
% delta_t = 0.05;     % Time increments
delta_t = 0.1;
steps = T/delta_t;  % Number of total steps

objects = 3;        % Number of Drones

%% Constraints

p_min  = 0;     % Minimum Position
p_max  = 30;    % Maximum Position
d1_max = 1000;  % Max Velocity [m/s]
d2_max = 35;    % Max Acceleration [m/s^2]
d3_max = 200;   % Max Jerk [m/s^3]

R      = 1;    % Minimum distance between objects

%% Constraint Vectors
R       = ones(steps+1,1)*R;
d1_max  = ones(steps+1,1)*d1_max;
d2_max  = ones(steps+1,1)*d2_max;

%% Start Points ~~~
% [start_pts,end_pts] = setStartAndEndPts(objects);
[start_pts,end_pts] = setStartAndEndPts2(objects);

init = zeros(3,objects*2);  % Initialization Points
n = 1;
for i = 1:objects
    for j = 1:2
        init(1,n) = start_pts(i,j);
        n = n + 1;
    end
end

%% End Points ~~~

b = zeros(3,objects*2);
n = 1;
for i = 1:objects
    for j = 1:2
        b(1,n) = end_pts(i,j) - start_pts(i,j);
        n = n + 1;
    end
end

%% Mueller Matrices

A = [1, delta_t, 1/2*delta_t^2;
    0, 1, delta_t;
    0, 0, 1];

B = [1/6*delta_t^3;
    1/2*delta_t^2;
    delta_t];

%% Construct AA matrix for approximation
AA = (A^(steps - 1))*B;

for n = 2:steps - 1
    AA = [AA,(A^(steps - n))*B];
end
AA = [AA,B];

%% FMINCON
y0 = ones(steps,objects*2);

lb = -d3_max*ones(steps,objects*2);
ub =  d3_max*ones(steps,objects*2);

options = optimoptions('fmincon','algorithm','sqp','maxfunctionevaluations',1e6,'maxiterations',1e6);
% options = optimoptions('fmincon','maxfunctionevaluations',1e6,'maxiterations',1e6);

[y,fval,exitflag,output] = fmincon(@(y)norm(AA*y-b),y0,[],[],[],[],lb,ub,@(y)cons(A,B,y,R,steps,objects,init,p_min,p_max,d1_max,d2_max),options);

cons(A,B,y,R,steps,objects,init,p_min,p_max,d1_max,d2_max)

%% Z matrix assembly
zz = zeros(objects*6,steps+1);          % Initialize 

interactions = objects*(objects - 1)/2; % Calculates the number of interactions between objects

dist    = zeros(interactions,steps+1);  % Initialize Distance Matrix
acc     = zeros(objects,steps+1);       % Initialize Acceleration Magnitude Matrix
vel     = zeros(objects,steps+1);       % Initialize Velocity Magnitude Matrix
pos_x   = zeros(objects,steps+1);       % Initialize X position matrix
pos_y   = zeros(objects,steps+1);       % Initialize Y position matrix

zz(:,1) = init(:);

n = 1;
for i = 1:objects-1     % Calculates Initial Distance between objects
    for k = i+1:objects
        dist(n,1) = sqrt((zz(6*i-5,1)-zz(6*k-5,1))^2 + (zz(6*i-2,1)-zz(6*k-2,1))^2);
        n = n + 1;
    end
end

for n = 1:objects       % Calculates Initial Acceleration, Velocity, and Positions
    acc(n,1)  = sqrt(zz(6*i-3,1)^2 + zz(6*i,1)^2);
    vel(n,1)  = sqrt(zz(6*i-4,1)^2 + zz(6*i-1,1)^2);
    pos_x(n,1)= zz(6*i-5,1);
    pos_y(n,1)= zz(6*i-2,1);
end

for m = 2:steps+1
    for i = 1:objects*2 % Calculates Position, Velocty, Accleration components
        zz(3*i-2:3*i,m) = A*zz(3*i-2:3*i,m-1) + B*y(m-1,i);
    end

    n = 1;
    for i = 1:objects-1 % Calculates Distance between objects
        for k = i+1:objects
            dist(n,m) = sqrt((zz(6*i-5,m)-zz(6*k-5,m))^2 + (zz(6*i-2,m) - zz(6*k-2,m))^2);
            n = n + 1;
        end
    end
    
    for n = 1:objects % Calculates Acceleration, Velocity, and Positions
        acc(n,m)  = sqrt(zz(6*i-3,m)^2 + zz(6*i,m)^2);
        vel(n,m)  = sqrt(zz(6*i-4,m)^2 + zz(6*i-1,m)^2);
        pos_x(n,m)= zz(6*i-5,m);
        pos_y(n,m)= zz(6*i-2,m);
    end
end

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
colours = ['k','b','g','r','c','m','y'];

% figure(5)
hold on
for i = 1:objects
    p1 = plot(zz(6*i-5,1),zz(6*i-2,1),strcat(colours(i),'^'));
    p2 = plot(zz(6*i-5,steps+1),zz(6*i-2,steps+1),strcat(colours(i),'p'));
    plot(zz(6*i-5,:),zz(6*i-2,:),strcat(colours(i),'-'))
    plot([zz(6*i-5,1),zz(6*i-5,steps+1)],[zz(6*i-2,1),zz(6*i-2,steps+1)],strcat(colours(i),'--'))
end

title(['Mueller Method for N = ',num2str(objects)])
xlabel('X position')
ylabel('Y position')
legend([p1,p2],'Start Point','End Point')
hold off

iterations = output.iterations

toc

Px = zz(1,:);
Py = zz(4,:);
for i = 2:objects
    Px = [Px;zz(6*i-5,:)];
    Py = [Py;zz(6*i-2,:)];
end

simTrajectories2(Px,Py,colours)
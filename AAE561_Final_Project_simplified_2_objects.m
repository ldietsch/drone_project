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
% delta_t = 0.02;
delta_t = 0.01;
steps = T/delta_t;

% Start Points ~~~
x1_0     = 0;
dx1_0    = 0;
d2x1_0   = 0;

y1_0     = 0;
dy1_0    = 0;
d2y1_0   = 0;

x2_0     = 1;
dx2_0    = 0;
d2x2_0   = 0;

y2_0     = 1;
dy2_0    = 0;
d2y2_0   = 0;

% End Points ~~~
x1_f    = 1 - x1_0;
dx1_f   = 0 - dx1_0;
d2x1_f  = 0 - d2x1_0;

y1_f    = 1 - y1_0;
dy1_f   = 0 - dy1_0;
d2y1_f  = 0 - d2y1_0;

x2_f    = -1 - x1_0;
dx2_f   = 0 - dx1_0;
d2x2_f  = 0 - d2x1_0;

y2_f    = -1 - y1_0;
dy2_f   = 0 - dy1_0;
d2y2_f  = 0 - d2y1_0;

% % 

d2x_max = 7;    % m/s^2
d3x_max = 70;   % m/s^3

A = [1, delta_t, 1/2*delta_t^2;
    0, 1, delta_t;
    0, 0, 1];

B = [1/6*delta_t^3;
    1/2*delta_t^2;
    delta_t];

% AA = zeros(1,steps - 1);
% AA = (A^(steps - 1))*B;
% AA = (A^(steps - 2));
% jj = 
AA = (A^(steps - 1))*B;

for n = 2:steps - 1
    AA = [AA,(A^(steps - n))*B];
end
AA = [AA,B];
% AA = [AA,AA];

b1 = [x1_f;dx1_f;d2x1_f];
b2 = [y1_f;dy1_f;d2y1_f];
b3 = [x2_f;dx2_f;d2x2_f];
b4 = [y2_f;dy2_f;d2y2_f];

% b = [x_f;dx_f;d2x_f;y_f;dy_f;d2y_f];

% b = [x_f, y_f; dx_f, dy_f; d2x_f, d2y_f];
% b = [b1, zeros(3,1); zeros(3,1), b2];
b = [b1, b2, b3, b4];

%% CVX optimizer

j_max = 70;

% cvx_begin quiet
cvx_begin
    variable y(steps,size(b,2))
    
    minimize(norm(AA*y-b))
%     minimize(AA*y-b)
cvx_end

%% CVX Optimizer 2
% zz1 = zeros(3,steps+1);
% zz2 = zeros(3,steps+1);
% zz3 = zeros(3,steps+1);
% zz4 = zeros(3,steps+1);
% 
% zz1(:,1) = [x1_0;dx1_0;d2x1_0];
% zz2(:,1) = [y1_0;dy1_0;d2y1_0];
% zz3(:,1) = [x2_0;dx2_0;d2x2_0];
% zz4(:,1) = [y2_0;dy2_0;d2y2_0];
% 
% % cvx_begin quiet
% cvx_begin
%     variable y(steps,size(b,2))
%     
% %     minimize((AA*y-b)'*(AA*y-b));
%     minimize(norm(AA*y-b))
%     
%     subject to
%         for m = 2:steps+1
%             zz1(m) = A(1,:)*zz1(:,m-1) + B*y(m-1,1);
% %             zz1(:,m) = A*zz1(:,m-1) + B*y(m-1,1);
% %             zz2(:,m) = A*zz2(:,m-1) + B*y(m-1,2);
% %             zz3(:,m) = A*zz3(:,m-1) + B*y(m-1,3);
% %             zz4(:,m) = A*zz4(:,m-1) + B*y(m-1,4);
%         end
% %     minimize(AA*y-b)
% cvx_end

%% Z matrix assembly
zz1 = zeros(3,steps+1);
zz2 = zeros(3,steps+1);
zz3 = zeros(3,steps+1);
zz4 = zeros(3,steps+1);

dist = zeros(1,steps+1);

zz1(:,1) = [x1_0;dx1_0;d2x1_0];
zz2(:,1) = [y1_0;dy1_0;d2y1_0];
zz3(:,1) = [x2_0;dx2_0;d2x2_0];
zz4(:,1) = [y2_0;dy2_0;d2y2_0];

dist(1)  = norm(zz1(1,1)-zz3(1,1),zz2(1,1)-zz4(1,1));

for m = 2:steps+1
    zz1(:,m) = A*zz1(:,m-1) + B*y(m-1,1);
    zz2(:,m) = A*zz2(:,m-1) + B*y(m-1,2);
    zz3(:,m) = A*zz3(:,m-1) + B*y(m-1,3);
    zz4(:,m) = A*zz4(:,m-1) + B*y(m-1,4);
    dist(m) = norm(zz1(1,m)-zz3(1,m),zz2(1,m)-zz4(1,m));
end

%% Plot
x_scale = [0:delta_t:T];
figure(1)
subplot(3,1,1)
plot(x_scale,zz1(1,:))
xlabel('time [s]')
ylabel('X-position')

subplot(3,1,2)
plot(x_scale,zz1(2,:))
xlabel('time [s]')
ylabel('X-Velocity')

subplot(3,1,3)
plot(x_scale,zz1(3,:))
xlabel('time [s]')
ylabel('X-Acceleration')

figure(2)
subplot(3,1,1)
plot(x_scale,zz2(1,:))
xlabel('time [s]')
ylabel('Y-position')

subplot(3,1,2)
plot(x_scale,zz2(2,:))
xlabel('time [s]')
ylabel('Y-Velocity')

subplot(3,1,3)
plot(x_scale,zz2(3,:))
xlabel('time [s]')
ylabel('Y-Acceleration')

figure(3)
subplot(3,1,1)
plot(x_scale,zz3(1,:))
xlabel('time [s]')
ylabel('X-position')

subplot(3,1,2)
plot(x_scale,zz3(2,:))
xlabel('time [s]')
ylabel('X-Velocity')

subplot(3,1,3)
plot(x_scale,zz3(3,:))
xlabel('time [s]')
ylabel('X-Acceleration')

figure(4)
subplot(3,1,1)
plot(x_scale,zz4(1,:))
xlabel('time [s]')
ylabel('Y-position')

subplot(3,1,2)
plot(x_scale,zz4(2,:))
xlabel('time [s]')
ylabel('Y-Velocity')

subplot(3,1,3)
plot(x_scale,zz4(3,:))
xlabel('time [s]')
ylabel('Y-Acceleration')
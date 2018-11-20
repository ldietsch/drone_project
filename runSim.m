% AAE 561: Convex Optimization
% Main runner file
% Dietshce, Lee, Sudarsanan
clear all; close all;

N = 3;   % Number of quadrotors
T = 100; % Sim duration 

% Set Figure parameters
figure('Name','Conflict free trajectories for quadrotors')
hold on
grid on
xlabel('X [m]')
ylabel('Y [m]');


% Set start and end points such that the quadrotors collide
[start_pts,end_pts] = setStartAndEndPts(N);

% Create quadrotor objects
for ii=1:N
    quads(ii) = Quad(ii,start_pts(ii,:),end_pts(ii,:),T);
end

% Plot quadrotor trajectories
for ii=1:N
    quads(ii).plotTraj();
end



function [start_pts,end_pts] = setStartAndEndPts(n)
    center_pt = [60,60];
    R         = 50;
    theta = 0; dtheta = pi/3;
    for ii=1:n
        start_pts(ii,:) = center_pt + R*[cos(theta),sin(theta)];
        end_pts(ii,:)   = center_pt - R*[cos(theta),sin(theta)];
        theta = theta + dtheta;
    end
    
    lims = [min(center_pt)-R-10,max(center_pt)+R+10];
    xlim(lims);ylim(lims);
end
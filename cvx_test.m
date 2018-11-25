clear all;
close all;

R = 1; %min. distance of aviodance
h = 0.2; %sampling time
T = 10; %total flight time for each vehicle
u_max = 13.333; %max acceleration per vehicle
N = 5; % number of vehicles
K = T/h; % number of states
n_var = 2*K; %need twice the number of states for 2-D
u_d_min(1:N*n_var,1) = -sqrt(u_max^2/2); %better way to interpret this from
%the paper? Doesn't necessarily seem right, but can't be wrong...
u_d_max(1:N*n_var,1) = sqrt(u_max^2/2);
jerk = zeros(N*n_var,1); %initialize jerk
j_max = 2;
j_d_min(1:N*(K-1),1) = -sqrt(j_max^2/2);
j_d_max(1:N*(K-1),1) = sqrt(j_max^2/2);
p_max_x(1:N*K,1) = 150;
p_min_x(1:N*K,1) = 0;
p_max_y(1:N*K,1) = 150;
p_min_y(1:N*K,1) = 0;
[x0, xf]= setStartAndEndPts(N);
v0 = zeros(N,2);

cvx_begin 
    variable U(N*n_var) %every n_var is a different vehicle
    minimize( U'*U )
    subject to
    %constraints (bounds) for jerk
    j_d_min <= jerk_x(U,h,N,K,n_var) <= j_d_max
    j_d_min <= jerk_y(U,h,N,K,n_var) <= j_d_max
    %constraints (bounds) for acceleration   
    u_d_min <= U <= u_d_max 
    %no velocity constraints
    %constraints for position (inequality)
    p_min_x<= pos_x(x0,v0,U,h,N,n_var,K) <= p_max_x
    p_min_y<= pos_y(x0,v0,U,h,N,n_var,K) <= p_max_y
    %constraints for position (equality)
    target_pos_x(x0,v0,U,h,N,n_var,K) == xf(:,1)
    target_pos_y(x0,v0,U,h,N,n_var,K) == xf(:,2)
cvx_end


clear all;
close all;

u_max = 13.33; %max acceleration per vehicle
K = 10; % number of vehicles
N = 20; % number of states
n_var = 2*N; %need twice the number of states for 2-D
u_d_min(1:K*n_var,1) = -sqrt(u_max^2/2);
u_d_max(1:K*n_var,1) = sqrt(u_max^2/2);

cvx_begin
    variable U(K*n_var) %every n_var is a different vehicle
    minimize( U'*U )
    subject to
        u_d_min <= U <= u_d_max
cvx_end
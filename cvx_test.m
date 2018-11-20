clear all;
close all;

u_max = 13.33; %max acceleration per vehicle
K = 10; % number of vehicles
N = 20; % number of states
n_var = 2*N; %need twice the number of states for 2-D

cvx_begin
    variable U(K*n_var) %every n_var is a different vehicle
    minimize( U'*U )
    subject to
        -K*N*u_max <= U <= K*N*u_max
    
cvx_end
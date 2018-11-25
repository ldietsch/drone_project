clear all;
close all;

R = 5; %min. distance of aviodance
h = 0.2; %sampling time
T = 10; %total flight time for each vehicle
u_max = 50; %max acceleration per vehicle
N = 3; % number of vehicles
K = T/h+1; % number of states
n_var = 2*K; %need twice the number of states for 2-D
u_d_min(1:N*n_var,1) = -sqrt(u_max^2/2); %better way to interpret this from
%the paper? Doesn't necessarily seem right, but can't be wrong...
u_d_max(1:N*n_var,1) = sqrt(u_max^2/2);
jerk = zeros(N*n_var,1); %initialize jerk
j_max = 100;
j_d_min(1:N*(K-1),1) = -sqrt(j_max^2/2);
j_d_max(1:N*(K-1),1) = sqrt(j_max^2/2);
p_max_x(1:N*K,1) = 150;
p_min_x(1:N*K,1) = 0;
p_max_y(1:N*K,1) = 150;
p_min_y(1:N*K,1) = 0;
[x0, xf]= setStartAndEndPts(N);
v0 = zeros(N,2);

cvx_begin quiet %obtain intitial solution
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

xq = recover_x(x0,v0,U,h,N,n_var,K);
xq = reshape(xq,N,K);
yq = recover_y(x0,v0,U,h,N,n_var,K);
yq = reshape(yq,N,K);
x = xq;
y = yq;
figure(1)
for i = 1:N
    plot(x(i,:),y(i,:),'--')
    hold on
end
for i = 1:N
    plot(x0(i,1),x0(i,2),'mp')
end
title('Conflict-free trajectories in 2-D')
xlabel('x [m]')
ylabel('y [m]')

eps = 1e-4;
fold = U'*U; fnew = 0;
cvx_status = "Infeasible";
noncvxCons = 0;
maxiter = 20;
iter = 1;
while abs(fold-fnew) > eps && noncvxCons == 0 && iter < maxiter

    fold = U'*U;
    
cvx_begin quiet %obtain intitial solution
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
    %constraints for position (inequality / avoidance)
    avoidance(xq,yq,x0,v0,U,h,N,n_var,K) >= R
    expression xq(N*K);
    expression yq(N*K);
    xq = pos_x(x0,v0,U,h,N,n_var,K);
    xq = reshape(xq,N,K);
    yq = pos_y(x0,v0,U,h,N,n_var,K);
    yq = reshape(yq,N,K);
cvx_end

    fnew = U'*U;
    [noncvxCons, sum] = check_position(xq,yq,R,N,K);
    violated_cons = nchoosek(N,2)*K-sum;
    num_vehicles = N;
    num_states = K;
    avoidance_radius = R;
   if isnan(fnew)
       disp('Solution will not converge. Retry with new problem parameters.')
       break
   end
   
iter = iter+1;   
   if iter == maxiter
       disp("Solution did not converge within max. number of iterations.")
       t = table(fnew,violated_cons,num_vehicles,num_states,...
           avoidance_radius,iter)
   elseif abs(fold-fnew) < eps
       disp("Solution converged to within function tolerance.")
       t = table(fnew,violated_cons,num_vehicles,num_states,...
           avoidance_radius,iter)
   elseif noncvxCons == 1
       disp("Solution converged to within nonconvex constraint tolerance.")
       t = table(fnew,violated_cons,num_vehicles,num_states,...
           avoidance_radius,iter)
   end

end

x = xq;
y = yq;
figure(1)
for i = 1:N
    plot(x(i,:),y(i,:),'.')
    hold on
end
hold off
title("Conflict-free trajectories in 2-D, R = "+R+" [m]")
xlabel('x [m]')
ylabel('y [m]')


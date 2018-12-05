function vx = vel_x_final(v0,Ux,h,K)
% vel_x_final takes as inputs v0 - intial velocity, U - acceleration, h -
% sampling time, N - number of vehicles, n_var - number of design
% variables, and K - number of states, and computes the final velocity in the
% x-direction for every Nth vehicle and Kth state. This outputs a cvx
% object for use as an equality constraint in the cvx algorithm.

vx = v0(:,1)+h*sum(Ux(:,1:K-1),2);

end
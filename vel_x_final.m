function vx = vel_x_final(v0,Ux,h,N,K)
% vel_x_final takes as inputs v0 - intial velocity, U - acceleration, h -
% sampling time, N - number of vehicles, n_var - number of design
% variables, and K - number of states, and computes the final velocity in the
% x-direction for every Nth vehicle and Kth state. This outputs a cvx
% object for use as an equality constraint in the cvx algorithm.
vx = cvx(zeros(N,1));

for j = 1:N
   vx(j) = v0(j,1)+h*sum(Ux(j,1:K-1));
end


end
function vz = vel_z_final(v0,Uz,h,N,K)
% vel_y_final takes as inputs v0 - intial velocity, U - acceleration, h -
% sampling time, N - number of vehicles, n_var - number of design
% variables, and K - number of states, and computes the final velocity in the
% y-direction for every Nth vehicle and Kth state. This outputs a cvx
% object for use as an equality constraint in the cvx algorithm.
vz = cvx(zeros(N,1));

for j = 1:N
    vz(j) = v0(j,2) + h*sum(Uz(j,1:K-1));
end

end
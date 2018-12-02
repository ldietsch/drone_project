function vz = vel_z(v0,U,h,N,n_var,K)
% vel_x takes as inputs v0 - intial velocity, U - acceleration, h -
% sampling time, N - number of vehicles, n_var - number of design
% variables, and K - number of states, and computes the velocity in the
% x-direction for every Nth vehicle and Kth state. This outputs a double to
% the workspace.
Uz = U(3:3:N*n_var);
Uz = reshape(Uz,N,K);
vz = zeros(N,K);
for j = 1:N
    vz(j,1) = v0(j,3);
    for k = 2:K
       vz(j,k) = vz(j,1)+h*sum(Uz(j,1:k-1));
    end
end

end
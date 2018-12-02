function vx = vel_x(v0,U,h,N,n_var,K)
% vel_x takes as inputs v0 - intial velocity, U - acceleration, h -
% sampling time, N - number of vehicles, n_var - number of design
% variables, and K - number of states, and computes the velocity in the
% x-direction for every Nth vehicle and Kth state. This outputs a double to
% the workspace.
Ux = U(1:2:N*n_var-1);
Ux = reshape(Ux,N,K);
vx = zeros(N,K);
for j = 1:N
    vx(j,1) = v0(j,1);
    for k = 2:K
       vx(j,k) = vx(j,1)+h*sum(Ux(j,1:k-1));
    end
end

end
function vy = vel_y(v0,U,h,N,n_var,K)
% vel_y takes as inputs v0 - intial velocity, U - acceleration, h -
% sampling time, N - number of vehicles, n_var - number of design
% variables, and K - number of states, and computes the velocity in the
% y-direction for every Nth vehicle and Kth state. This outputs a double to
% the workspace.
Uy = U(2:2:N*n_var);
Uy = reshape(Uy,N,K);
vy = zeros(N,K);
for j = 1:N
    vy(j,1) = v0(j,2);
    for k = 2:K
       vy(j,k) = vy(j,1)+h*sum(Uy(j,1:k-1));
    end
end

end
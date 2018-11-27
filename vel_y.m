function vy = vel_y(v0,U,h,N,n_var,K)

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
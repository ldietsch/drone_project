function vy = vel_y_final(v0,U,h,N,n_var,K)

Uy = U(2:2:N*n_var);
Uy = reshape(Uy,N,K);
vy = cvx(zeros(N,1));

for j = 1:N
    vy(j) = v0(j,2) + h*sum(Uy(j,1:K-1));
end

end
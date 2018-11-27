function vx = vel_x(v0,U,h,N,n_var,K)

Ux = U(1:2:N*n_var-1);
Uy = U(2:2:N*n_var);
for j = 1:N
    vx(j,1) = v0(j);
    for k = 2:K
       vx(j,k) = v0(j,1)+h*sum(Ux(j,1:k-1));
    end
end
vx = vx(:);

end
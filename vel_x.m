function vx = vel_x(v0,U,h,N,n_var,K)

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
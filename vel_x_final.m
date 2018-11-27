function vx = vel_x_final(v0,U,h,N,n_var,K)

Ux = U(1:2:N*n_var-1);
Ux = reshape(Ux,N,K);
vx = cvx(zeros(N,1));

for j = 1:N
   vx(j) = v0(j,1)+h*sum(Ux(j,1:K-1));
end


end
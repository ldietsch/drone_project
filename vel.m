function v = vel(v0,U,h,N,n_var,K)

Ux = U(1:2:N*n_var-1);
Uy = U(2:2:N*n_var);
iter = 1;
a = zeros(N,K);
for j = 1:N
    for k = 1:K
        a(j,k) = sqrt(Ux(iter)^2+Uy(iter)^2);
        iter = iter+1;
    end
end
v = zeros(N,K);
for j = 1:N
    v(j,1) = v0(j);
    for k = 2:K
       v(j,k) = v0(j)+h*sum(a(j,1:j,k-1));
    end
end
v = v(:);

end
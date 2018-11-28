function jerk_y = jerk_y(U,h,N,K,n_var)

Uy = U(2:2:N*n_var);
Uy = reshape(Uy,N,K);
jerk_y = cvx(zeros(N,K-1));
for i=1:N
    jerk_y(i,:) = (Uy(i,2:end)-Uy(i,1:end-1))/h;
end

end
function jerk_x = jerk_x(U,h,N,K,n_var)

Ux = U(1:2:N*n_var-1);
Ux = reshape(Ux,N,K);
for i=1:N
    jerk_x(i,:) = (Ux(i,2:end)-Ux(i,1:end-1))/h;
end
jerk_x = jerk_x(:);

end
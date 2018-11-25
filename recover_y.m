function py = recover_y(p0,v0,U,h,N,n_var,K)

Uy = U(2:2:N*n_var);
Uy = reshape(Uy,N,K);
for j = 1:N
    py(j,1) = (p0(j,2));
    for k = 2:K
       c = findCoeffs(k);
       py(j,k) = (py(j,1) + h*(k-1)*v0(j,2)+h^2/2*(c'*Uy(j,1:k-1)'));
    end
end
py = py(:);

end

function c = findCoeffs(k)
odd_number = 3;
c = zeros(k-1,1);
for i=1:k-1
   c(i,1) = (2*k-odd_number);
   odd_number = odd_number+2;
end

end
function px = pos_x(p0,v0,U,h,N,n_var,K)

Ux = U(1:2:N*n_var-1);
Ux = reshape(Ux,N,K);
for j = 1:N
    px(j,1) = cvx(p0(j,1));
    for k = 2:K
       c = findCoeffs(k);
       px(j,k) = cvx(px(j,1) + h*(k-1)*v0(j,1)+h^2/2*(c'*Ux(j,1:k-1)'));
    end
end
px = px(:);

end

function c = findCoeffs(k)
odd_number = 3;
c = zeros(k-1,1);
for i=1:k-1
   c(i,1) = (2*k-odd_number);
   odd_number = odd_number+2;
end

end
function px = target_pos_y(p0,v0,U,h,N,n_var,K)
% target_pos_y takes as inputs p0 - initial position, 
% v0 - initial velocity, U - acceleration, h - sampling time, 
% n_var - number of design variables and computes the final y-coordinate of 
% position for every Nth vehicle and Kth state for use in targeting the
% final position, xf, enforced as an equality constraint in cvx.
Uy = U(2:2:N*n_var);
Uy = reshape(Uy,N,K);
for j = 1:N
    c = findCoeffs(K);
    px(j) = cvx(p0(j,2) + h*(K-1)*v0(j,1)+h^2/2*(c'*Uy(j,1:K-1)'));
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
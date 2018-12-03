function px = target_pos_x(p0,v0,Ux,h,N,K)
% target_pos_x takes as inputs p0 - initial position, 
% v0 - initial velocity, U - acceleration, h - sampling time, 
% n_var - number of design variables and computes the final x-coordinate of 
% position for every Nth vehicle and Kth state for use in targeting the
% final position, xf, enforced as an equality constraint in cvx.
% This is a cvx-solver version which outputs a cvx object.
% This formula is according to Auguliaro.
for j = 1:N
    c = findCoeffs(K);
    px(j) = cvx(p0(j,1) + h*(K-1)*v0(j,1)+h^2/2*(c'*Ux(j,1:K-1)'));
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
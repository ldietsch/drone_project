function px = recover_x(p0,v0,U,h,N,n_var,K)
% recover_x takes as inputs p0 - initial position, v0 - initial velocity, U - 
% acceleration, h - sampling time, n_var - number of design variables and 
% computes the x-coordinate of position for every Nth vehicle and Kth state
% This is the "double" type version which can output to the workspace. 
% See pos_x  pos_y for the "cvx" type output.
% This formula is according to Auguliaro.
Ux = U(1:3:N*n_var);
Ux = reshape(Ux,N,K);
for j = 1:N
    px(j,1) = (p0(j,1));
    for k = 2:K
       c = findCoeffs(k);
       px(j,k) = (px(j,1) + h*(k-1)*v0(j,1)+h^2/2*(c'*Ux(j,1:k-1)'));
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
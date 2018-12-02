function pz = pos_z(p0,v0,U,h,N,n_var,K)
% pos_y takes as inputs p0 - initial position, v0 - initial velocity, U - 
% acceleration, h - sampling time, n_var - number of design variables and 
% computes the y-coordinate of position for every Nth vehicle and Kth state
% This is the cvx-solver version which outputs a cvx object. See recover_x 
% recover_y for the "double" type output.
% This formula is according to Auguliaro.
Uz = U(3:3:N*n_var);
Uz = reshape(Uz,N,K);
for j = 1:N
    pz(j,1) = cvx(p0(j,3));
    for k = 2:K
       c = findCoeffs(k);
       pz(j,k) = cvx(pz(j,1) + h*(k-1)*v0(j,3)+h^2/2*(c'*Uz(j,1:k-1)'));
    end
end
pz = pz(:);

end

function c = findCoeffs(k)
odd_number = 3;
c = zeros(k-1,1);
for i=1:k-1
   c(i,1) = (2*k-odd_number);
   odd_number = odd_number+2;
end

end
function py = pos_y_opt(p0,v0,Uy,h,K)
% pos_x takes as inputs p0 - initial position, v0 - initial velocity, U - 
% acceleration, h - sampling time, n_var - number of design variables and 
% computes the x-coordinate of position for every Nth vehicle and Kth state
% This is the cvx-solver version which outputs a cvx object. See recover_x 
% recover_y for the "double" type output.
% This formula is according to Auguliaro.
py(:,1) = cvx(p0(:,2));
for k = 2:K
   c = 2*k-3:-2:1;
   py(:,k) = cvx(py(:,1) + h*(k-1)*v0(:,2)+h^2/2*(Uy(:,1:k-1)*c'));
end
py = py(:);
end

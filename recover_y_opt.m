function py = recover_y_opt(p0,v0,Uy,h,K)
% recover_y takes as inputs p0 - initial position, v0 - initial velocity, U - 
% acceleration, h - sampling time, n_var - number of design variables and 
% computes the y-coordinate of position for every Nth vehicle and Kth state
% This is the "double" type version which can output to the workspace. 
% See pos_x  pos_y for the "cvx" type output.
% This formula is according to Auguliaro.
py(:,1) = p0(:,2);
for k = 2:K
   c = 2*k-3:-2:1;
   py(:,k) = py(:,1) + h*(k-1)*v0(:,2)+h^2/2*(Uy(:,1:k-1)*c');
end
py = py(:);
end

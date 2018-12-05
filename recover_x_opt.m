function px = recover_x_opt(p0,v0,Ux,h,K)
% recover_x takes as inputs p0 - initial position, v0 - initial velocity, U - 
% acceleration, h - sampling time, n_var - number of design variables and 
% computes the x-coordinate of position for every Nth vehicle and Kth state
% This is the "double" type version which can output to the workspace. 
% See pos_x  pos_y for the "cvx" type output.
% This formula is according to Auguliaro.
px(:,1) = p0(:,1);
for k = 2:K
   c = 2*k-3:-2:1;
   px(:,k) = px(:,1) + h*(k-1)*v0(:,1)+h^2/2*(Ux(:,1:k-1)*c');
end
px = px(:);

end

function px = target_pos_y(p0,v0,Uy,h,K)
% target_pos_y takes as inputs p0 - initial position, 
% v0 - initial velocity, U - acceleration, h - sampling time, 
% n_var - number of design variables and computes the final y-coordinate of 
% position for every Nth vehicle and Kth state for use in targeting the
% final position, xf, enforced as an equality constraint in cvx.
c  = 2*K-3:-2:1;

px = cvx(p0(:,2) + h*(K-1)*v0(:,1)+h^2/2*(Uy(:,1:K-1)*c'));

end

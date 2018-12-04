function pz = target_pos_z(p0,v0,Uz,h,N,K)
% target_pos_x takes as inputs p0 - initial position, 
% v0 - initial velocity, U - acceleration, h - sampling time, 
% n_var - number of design variables and computes the final x-coordinate of 
% position for every Nth vehicle and Kth state for use in targeting the
% final position, xf, enforced as an equality constraint in cvx.
% This is a cvx-solver version which outputs a cvx object.
% This formula is according to Auguliaro.
c = 2*K-3:-2:1;
pz = cvx(p0(:,3) + h*(K-1)*v0(:,3)+h^2/2*Uz(:,1:K-1)*c');
end
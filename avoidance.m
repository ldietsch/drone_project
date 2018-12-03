function p = avoidance(xq,yq,zq,x0,v0,Ux,Uy,Uz,Gz,h,N,K)
%This function is the approximation equation from Augugliaro that enforces 
%an avoidance radiance between N vehicles. xq and yq are obtained from the
%previous iteration and are the previous coordinates for the Nth vehicle. 
%x0 and v0 are the initial states for position and velocity. U is the 
% acceleration design variable, h is the sampling time, n_var is the 
% number of design variables needed (2*K for 2-D). Note that the constraint
% for avoidance is implemented outside the function in the cvx algorithm.

%find the current position coordinates, px and py
px = pos_x(x0,v0,Ux,h,N,K);
%shape into a matrix for easier readability
px = reshape(px,N,K);
py = pos_y(x0,v0,Uy,h,N,K);
py = reshape(py,N,K);
pz = pos_z(x0,v0,Uz+Gz,h,N,K);
pz = reshape(pz,N,K);

for i=1:N-1
    for j=i+1:N
        for k = 1:K
            pqi = [xq(i,k);yq(i,k);zq(i,k)];
            pqj = [xq(j,k);yq(j,k);zq(i,k)];
            
            pi  = [px(i,k);py(i,k);pz(i,k)];
            pj  = [px(j,k);py(j,k);pz(j,k)];
            
            eta = (pqi-pqj)/norm(pqi-pqj);
            
            p(i,k) = norm(pqi-pqj) + eta'*(pi - pj - (pqi - pqj));
        end
    end
end

p = p(:);
end
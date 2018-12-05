function p = avoidance_opt(xq,yq,x0,v0,Ux,Uy,h,N,K)
%This function is the approximation equation from Augugliaro that enforces 
%an avoidance radiance between N vehicles. xq and yq are obtained from the
%previous iteration and are the previous coordinates for the Nth vehicle. 
%x0 and v0 are the initial states for position and velocity. U is the 
% acceleration design variable, h is the sampling time, n_var is the 
% number of design variables needed (2*K for 2-D). Note that the constraint
% for avoidance is implemented outside the function in the cvx algorithm.

%find the current position coordinates, px and py
px = pos_x_opt(x0,v0,Ux,h,K);
%shape into a matrix for easier readability
px = reshape(px,N,K);
py = pos_y_opt(x0,v0,Uy,h,K);
py = reshape(py,N,K);
for i=1:N-1
    for j=i+1:N
%         for k = 1:K
            pqi = [xq(i,:);yq(i,:)];
            pqj = [xq(j,:);yq(j,:)];
            
            pi  = [px(i,:);py(i,:)];
            pj  = [px(j,:);py(j,:)];
            
            pqijNorm = sqrt(dot(pqi-pqj,pqi-pqj,1));
            eta = (pqi-pqj)./repmat(pqijNorm,2,1);
            
%             p(i,k) = pqijNorm + eta'*(pi - pj - (pqi - pqj));
            p(i,:) = pqijNorm + dot(eta,(pi - pj - (pqi - pqj)),1);
%         end
    end
end

p = p(:);
end
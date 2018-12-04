function p = avoidance(xq,yq,x0,v0,U,h,N,n_var,K)
%This function is the approximation equation from Augugliaro that enforces 
%an avoidance radiance between N vehicles. xq and yq are obtained from the
%previous iteration and are the previous coordinates for the Nth vehicle. 
%x0 and v0 are the initial states for position and velocity. U is the 
% acceleration design variable, h is the sampling time, n_var is the 
% number of design variables needed (2*K for 2-D). Note that the constraint
% for avoidance is implemented outside the function in the cvx algorithm.

%find the current position coordinates, px and py
px = pos_x(x0,v0,U,h,N,n_var,K);
%shape into a matrix for easier readability
px = reshape(px,N,K);
py = pos_y(x0,v0,U,h,N,n_var,K);
py = reshape(py,N,K);
for i=1:N-1
    for j=i+1:N
        for k = 1:K
            eta = ([xq(i,k);yq(i,k)]-[xq(j,k);yq(j,k)])./...
                sqrt((xq(i,k)-xq(j,k))^2+(yq(i,k)-yq(j,k))^2);
            p(i,k) = sqrt((xq(i,k)-xq(j,k))^2+(yq(i,k)-yq(j,k))^2)+...
                eta'*([px(i,k);py(i,k)]-[px(j,k);py(j,k)]-([xq(i,k);...
                yq(i,k)]-[xq(j,k);yq(j,k)]));
        end
    end
end

p = p(:);
end
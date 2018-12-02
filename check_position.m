function [noncvxcons, sum] = check_position(x,y,z,R,N,K)
%The purpose of this function is to check how well the non-convex
%constraints are satisfied as an ad-hoc check for convergence. The check is
%satified to within a tolerance of eps.
eps = 1e-4;
sum = 0;
for i=1:N-1%only need to check the distance between each vehicle once
    for j=1:K
        for k = i+1:N
            if i~=k && i < k
                %check the distance between ith and kth vehicle at the jth
                %state
                check = sqrt((x(i,j)-x(k,j))^2+(y(i,j)-y(k,j))^2+(z(i,j)-z(k,j))^2)>=R-eps;
                sum = sum + check;
            end

        end
    end
end
%the output is binary. 1 if all nonconvex constraints are satisfied and 0 
%otherwise
noncvxcons = (sum==(nchoosek(N,2)*K));

end
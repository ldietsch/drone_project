function [noncvxcons, sum, distances] = check_position_opt(x,y,R,N,K)
%The purpose of this function is to check how well the non-convex
%constraints are satisfied as an ad-hoc check for convergence. The check is
%satified to within a tolerance of eps.
eps = 0.1; %10 cm
sum = 0;
distances =[];
for i=1:N-1%only need to check the distance between each vehicle once
    for j=1:K
        for k = i+1:N
            if i~=k && i < k
                %check the distance between ith and kth vehicle at the jth
                %state
                d = sqrt((x(i,j)-x(k,j))^2+(y(i,j)-y(k,j))^2);
                check = d>=R-eps;
                sum = sum + check;
                if ~check
                    distances(end+1) = d;
                end
            end

        end
    end
end
%the output is binary. 1 if all nonconvex constraints are satisfied and 0 
%otherwise
noncvxcons = (sum==(nchoosek(N,2)*K));

end
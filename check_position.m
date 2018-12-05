function [noncvxcons, sum, distances] = check_position_opt(x,y,R,N,K)
%The purpose of this function is to check how well the non-convex
%constraints are satisfied as an ad-hoc check for convergence. The check is
%satified to within a tolerance of eps.
eps = 0.05; %1 cm
sum = 0;
distances =[];

for i=1:N-1%only need to check the distance between each vehicle once
    for j=i+1:N
        pij = [x(i,:);y(i,:)] - [x(j,:);y(j,:)];
        D   = sqrt(dot(pij,pij,1));
        check = D>=R-eps;
        sum = sum + sum(check);
        if ~sum(check)
            distances(end+1,:) = D(check);
        end
    end
end
%the output is binary. 1 if all nonconvex constraints are satisfied and 0 
%otherwise
noncvxcons = (sum==(nchoosek(N,2)*K));

end
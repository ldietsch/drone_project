function noncvxcons = check_position(x,y,R,N,K)
eps = 1e-4;
sum = 0;
for i=1:ceil(N/2)
    for j=1:K
        for k = 1:N
            if i~=k
                check = sqrt((x(i,j)-x(k,j))^2+(y(i,j)-y(k,j))^2)>=R-eps;
                sum = sum + check;
%                 if check==0
%                     j
%                     sqrt((x(i,j)-x(k,j))^2+(y(i,j)-y(k,j))^2)
%                 end
            end

        end
    end
end
        
noncvxcons = (sum==ceil(N/2)*(K)*(N-1));

end
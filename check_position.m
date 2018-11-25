function noncvxcons = check_position(x,y,R,N,K)

sum = 0;
for i=1:ceil(N/2)
    for j=1:K
        for k = 1:N
            if i~=k
                check = sqrt((x(i,j)-x(i,k))^2+(y(i,j)-y(i,k))^2)>=R;
                sum = sum + check;
            end
        end
    end
end
        
noncvxcons = (sum==ceil(N/2)*K*(N-1));

end
function p = avoidance(xq,yq,x0,v0,U,h,N,n_var,K)
px = pos_x(x0,v0,U,h,N,n_var,K);
px = reshape(px,N,K);
py = pos_y(x0,v0,U,h,N,n_var,K);
py = reshape(py,N,K);
for i=1:N
    for j=1:N
        for k = 1:K
            if i~=j && i < j
                eta = ([xq(i,k);yq(i,k)]-[xq(j,k);yq(j,k)])./...
                    sqrt((xq(i,k)-xq(j,k))^2+(yq(i,k)-yq(j,k))^2);
                p(i,k) = sqrt((xq(i,k)-xq(j,k))^2+(yq(i,k)-yq(j,k))^2)+...
                    eta'*([px(i,k);py(i,k)]-[px(j,k);py(j,k)]-([xq(i,k);...
                    yq(i,k)]-[xq(j,k);yq(j,k)]));
            end
        end
    end
end

p = p(:);
end
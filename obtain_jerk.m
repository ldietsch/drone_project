function J_nk = obtain_jerk(Jx,Jy,N,K)

J_nk = cvx(zeros(N,K));
for i  = 1:N
    for j = 1:K-1
        J_nk(i,j) = norm([Jx(i,j);Jy(i,j)]);
    end
end
J_nk = J_nk(:);

end

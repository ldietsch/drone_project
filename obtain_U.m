function U_nk = obtain_U(Ux,Uy,N,K)

U_nk = cvx(zeros(N,K));
for i  = 1:N
    for j = 1:K
        U_nk(i,j) = norm([Ux(i,j);Uy(i,j)]);
    end
end
U_nk = U_nk(:);

end


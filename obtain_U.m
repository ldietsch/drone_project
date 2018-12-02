function U_nk = obtain_U(Ux,Uy,Uz,N,K)
%obtain_U takes as inputs Ux and Uy for every Nth vehicle and Kth state
%and obtains the 2-norm so that it can be constrained as a euclidean ball in
%cvx. Ux and Uy are the acceleration in a 2-D coordinate system.
U_nk = cvx(zeros(N,K));%use "cvx" to be solver-friendly
for i  = 1:N
    for j = 1:K
        U_nk(i,j) = norm([Ux(i,j);Uy(i,j);Uz(i,j)]);
    end
end
U_nk = U_nk(:);

end


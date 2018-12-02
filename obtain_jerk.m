function J_nk = obtain_jerk(Jx,Jy,Jz,N,K)
%obtain_jerk takes as inputs Jx and Jy for every Nth vehicle and Kth state
%and obtains the 2-norm so that it can be constrained as a euclidean ball in
%cvx. Jx and Jy are the jerk in a 2-D coordinate system.
J_nk = cvx(zeros(N,K)); %must use "cvx" since this passes an intermediate 
%value obtained from a cvx design variable (jerk is obtained at every step
%from the current value of U).
for i  = 1:N
    for j = 1:K-1 %K-1 states for jerk
        J_nk(i,j) = norm([Jx(i,j);Jy(i,j);Jz(i,j)]);
    end
end
J_nk = J_nk(:);

end

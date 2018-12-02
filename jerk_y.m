function jerk_y = jerk_y(U,h,N,K,n_var)
%jerk_y gives the jerk in the y-direction for the Nth vehicle, given input
%U (acceleration), K states, and number of design variables n_var. h is the
%sampling time. Note there are only K-1 states for jerk as opposed to K.
Uy = U(2:2:N*n_var);
Uy = reshape(Uy,N,K);
jerk_y = cvx(zeros(N,K-1));
for i=1:N
    jerk_y(i,:) = (Uy(i,2:end)-Uy(i,1:end-1))/h;
end

end
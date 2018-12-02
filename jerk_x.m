function jerk_x = jerk_x(U,h,N,K,n_var)
%jerk_x gives the jerk in the x-direction for the Nth vehicle, given input
%U (acceleration), K states, and number of design variables n_var. h is the
%sampling time. Note there are only K-1 states for jerk as opposed to K.
Ux = U(1:2:N*n_var-1);
Ux = reshape(Ux,N,K);
jerk_x = cvx(zeros(N,K-1));
for i=1:N
    jerk_x(i,:) = (Ux(i,2:end)-Ux(i,1:end-1))/h;
end

end
function jerk_x = jerk_x(Ux,h,N,K)
%jerk_x gives the jerk in the x-direction for the Nth vehicle, given input
%U (acceleration), K states, and number of design variables n_var. h is the
%sampling time. Note there are only K-1 states for jerk as opposed to K.
jerk_x = cvx(zeros(N,K-1));
for i=1:N
    jerk_x(i,:) = (Ux(i,2:end)-Ux(i,1:end-1))/h;
end

end
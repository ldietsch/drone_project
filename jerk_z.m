function jerk_z = jerk_z(Uz,h,N,K)
%jerk_y gives the jerk in the y-direction for the Nth vehicle, given input
%U (acceleration), K states, and number of design variables n_var. h is the
%sampling time. Note there are only K-1 states for jerk as opposed to K.
jerk_z = cvx(zeros(N,K-1));
for i=1:N
    jerk_z(i,:) = (Uz(i,2:end)-Uz(i,1:end-1))/h;
end

end
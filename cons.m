function [c,ceq] = cons(A,B,y,R,steps,objects,init,p_min,p_max,d1_max,d2_max)

zz = zeros(objects*6,steps+1);          % Initialize 

interactions = objects*(objects - 1)/2; % Calculates the number of interactions between objects

dist    = zeros(interactions,steps+1);  % Initialize Distance Matrix
acc     = zeros(objects,steps+1);       % Initialize Acceleration Magnitude Matrix
vel     = zeros(objects,steps+1);       % Initialize Velocity Magnitude Matrix
pos_x   = zeros(objects,steps+1);       % Initialize X position matrix
pos_y   = zeros(objects,steps+1);       % Initialize Y position matrix

zz(:,1) = init(:);

n = 1;
for i = 1:objects-1     % Calculates Initial Distance between objects
    for k = i+1:objects
        dist(n,1) = sqrt((zz(6*i-5,1)-zz(6*k-5,1))^2 + (zz(6*i-2,1)-zz(6*k-2,1))^2);
        n = n + 1;
    end
end

for n = 1:objects       % Calculates Initial Acceleration, Velocity, and Positions
    acc(n,1)  = sqrt(zz(6*i-3,1)^2 + zz(6*i,1)^2);
    vel(n,1)  = sqrt(zz(6*i-4,1)^2 + zz(6*i-1,1)^2);
    pos_x(n,1)= zz(6*i-5,1);
    pos_y(n,1)= zz(6*i-2,1);
end

for m = 2:steps+1
    for i = 1:objects*2 % Calculates Position, Velocty, Accleration components
        zz(3*i-2:3*i,m) = A*zz(3*i-2:3*i,m-1) + B*y(m-1,i);
    end

    n = 1;
    for i = 1:objects-1 % Calculates Distance between objects
        for k = i+1:objects
            dist(n,m) = sqrt((zz(6*i-5,m)-zz(6*k-5,m))^2 + (zz(6*i-2,m) - zz(6*k-2,m))^2);
            n = n + 1;
        end
    end
    
    for n = 1:objects % Calculates Acceleration, Velocity, and Positions
        acc(n,m)  = sqrt(zz(6*i-3,m)^2 + zz(6*i,m)^2);
        vel(n,m)  = sqrt(zz(6*i-4,m)^2 + zz(6*i-1,m)^2);
        pos_x(n,m)= zz(6*i-5,m);
        pos_y(n,m)= zz(6*i-2,m);
    end
end
dist_con = max(R - dist');      % Calculates Distance constraint between objects
acc_con  = max(acc' - d2_max);  % Calculates Acceleration magnitude constraint
vel_con  = max(vel' - d1_max);  % Calculates Velocity magnitude constraint

% c = rad;
c = [dist_con;acc_con;vel_con]; % Fmincon inequality constraint format
ceq = [];                       % Fmincon equality constraint format
end
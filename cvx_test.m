clear all
close all
% AAE 561: Final Project 
% Conflict-free trajectories for quadrotors
% Dietsche, Lee and Sudarsanan

N_Quads = 3;       % Number of vehicles
dStartEnd = 10;    % Distance between start and end points for quadrotors
simTime  = zeros(1,N_Quads-1);
solnStat = cell(1,N_Quads-1);
solnTables = cell(1,N_Quads-1);

% Rune single case
for N=N_Quads
  
% % Run multiple cases
% for N=2:N_quads
    figure(N-1);
    [x0, xf]= setStartAndEndPts(dStartEnd,N); % Set the start and points for each vehicle
%     x0 = flipud(x0);
%     xf = flipud(xf);
    % Vehicle parameters based on paper by Ardakani, et. al "Online Minimum-Jerk
    % Trajectory Generation" p. 7 figure 4
    u_max = 35;  % [m/s^2] Max acceleration per vehicle
    j_max = 200; % [m/s^3] Max jerk per vehicle    
    v0 = zeros(N,3);                % Set initial velocity to zero for each vehicle
    
    t_start = tic;
    R = 1.0; % [m] Min. distance of aviodance
    h = 0.1; % [s] Sampling time
    
    
    T = 8;   % [s] Total flight time for each vehicle
    % Estimate flight time
%     T = estimateFlightTime(dStartEnd,u_max); % Assumes distance traveled same for all quads
    K = ceil(T/h)+1;   % Number of time steps
    n_var = 3*K; % Need thrice the number of states for 3-D


    p_max_x(1:N*K,1) = dStartEnd+10; % [m] Define the space allowed to fly in (rectangle)
    p_min_x(1:N*K,1) = 0;            % [m] Define the space allowed to fly in
    p_max_y(1:N*K,1) = dStartEnd+10; % [m] Define the space allowed to fly in
    p_min_y(1:N*K,1) = 0;            % [m] Define the space allowed to fly in
    p_max_z(1:N*K,1) = dStartEnd+10; % [m] Define the space allowed to fly in
    p_min_z(1:N*K,1) = 0;            % [m] Define the space allowed to fly in
    
    % Gravity vectors 
    gravity_u = repmat([0;0;-9.8],N*K,1);
    Gz = -9.8*ones(N,K);
    
    % Obtain intitial solution to be used for the first approximation of the
    % avoidance constraints. Here, there are no avoidance constraints.
    cvx_begin quiet 
        variable U(N*n_var) % Every n_var is a different vehicle
%         minimize( (U+gravity)'*(U +gravity))
        minimize( U'*U + 2*gravity_u'*U)
%         minimize(U'*U)
        subject to
        expressions Jx Jy

        Ux = U(1:3:N*n_var);
        Ux = reshape(Ux,N,K);
        Uy = U(2:3:N*n_var);
        Uy = reshape(Uy,N,K);
        Uz = U(3:3:N*n_var);
        Uz = reshape(Uz,N,K);

%             Jx = (Ux(:,2:end)-Ux(:,1:end-1))/h;%jerk_x(U,h,N,K,n_var);
%             Jy = (Uy(:,2:end)-Uy(:,1:end-1))/h;
%             Jz = (Uz(:,2:end)-Uz(:,1:end-1))/h;

        Jx = jerk_x(Ux,h,N,K);
        Jy = jerk_y(Uy,h,N,K);
        Jz = jerk_z(Uz,h,N,K);

        % Constraints (bounds) for jerk
        obtain_jerk(Jx,Jy,Jz,N,K) <= j_max
%             Jx(:).^2 + Jy(:).^2 + Jz(:).^2 <= j_max^2
%             % Constraints (bounds) for acceleration   
        obtain_U(Ux,Uy,Uz,N,K) <= u_max
%             Ux(:).^2 + Uy(:).^2 + Uz(:).^2 <= u_max^2 

        Ux(:,K) == 0; %cvx(zeros(N,1));
        Uy(:,K) == 0; %cvx(zeros(N,1));
        Uz(:,K)+Gz(:,K) == 0; %cvx(zeros(N,1));


        % Velocity constraints
%             v0(:,1) + h*K*sum(Ux(:,1:K-1),2) == 0;
%             v0(:,2) + h*K*sum(Uy(:,1:K-1),2) == 0;
%             v0(:,3) + h*K*sum(Uz(:,1:K-1),2) == 0;

        vel_x_final(v0,Ux,h,N,K) == 0;
        vel_y_final(v0,Uy,h,N,K) == 0;
        vel_z_final(v0,Uz+Gz,h,N,K) == 0;

%         vel_x_final(v0,U,h,N,n_var,K) == 0;
%         vel_y_final(v0,U,h,N,n_var,K) == 0;
        % Constraints for position (inequality)
        p_min_x <= pos_x(x0,v0,Ux,h,N,K) <= p_max_x
        p_min_y <= pos_y(x0,v0,Uy,h,N,K) <= p_max_y
        p_min_z <= pos_z(x0,v0,Uz+Gz,h,N,K) <= p_max_z

        % Constraints for position (equality)
        target_pos_x(x0,v0,Ux,h,N,K) == xf(:,1)
        target_pos_y(x0,v0,Uy,h,N,K) == xf(:,2)
        target_pos_z(x0,v0,Uz+Gz,h,N,K) == xf(:,3)
    cvx_end
    if cvx_status == "Solved"
        disp("Initial trajectories found successfully");
    end
    % Obtain initial trajectories
    xq = recover_x(x0,v0,Ux,h,N,K);
    xq = reshape(xq,N,K);
    yq = recover_y(x0,v0,Uy,h,N,K);
    yq = reshape(yq,N,K);
    zq = recover_z(x0,v0,Uz+Gz,h,N,K);
    zq = reshape(zq,N,K);
    x = xq;
    y = yq;
    z = zq;

    % Pause simulation time calculation while plotting
    simTime(N-1) = simTime(N-1) + tic - t_start;

    % Plot initial trajectories (no avoidance)
    color_palette = {};
    for i = 1:N
        p_hand = plot3(x(i,:),y(i,:),z(i,:),'LineStyle','--','LineWidth',2);
        color_palette{i} = p_hand.Color;
        plot3(x0(i,1),x0(i,2),x0(i,3),'^','MarkerSize',6,'MarkerEdgeColor',p_hand.Color,...
                                                 'MarkerFaceColor',p_hand.Color);
        plot3(xf(i,1),xf(i,2),xf(i,3),'p','MarkerSize',8,'MarkerEdgeColor',p_hand.Color,...
                                                 'MarkerFaceColor',p_hand.Color);
            
    end
    title("Initial trajectories in 3-D ["+N+" Quads]")
    xlabel('x [m]')
    ylabel('y [m]')
    drawnow
%     keyboard

    % Resume simulation time logging
    t_start = tic;

    eps = 1e-4;
    fold = U'*U; fnew = 0;
    cvx_status = "Infeasible";
    noncvxConsSatisfied = 0;
    maxiter = 10;
    iter = 1;

    % The main loop for enforcing avoidance constraints using the linear
    % approximation formulated by Augugliaro
%     while abs(fold-fnew) > eps && noncvxConsSatisfied == 0 && iter < maxiter && cvx_status ~= "Solved"
    while ((abs(fold-fnew) > eps && cvx_status ~= "Solved") || ~noncvxConsSatisfied) && iter < maxiter
        fold = U'*U;

        cvx_begin quiet
            variable U(N*n_var) % Every n_var is a different vehicle
%             minimize(U'*U)
            minimize( U'*U + 2*gravity_u'*U)
%             minimize((U+gravity)'*(U+gravity))
            subject to
            expressions Jx Jy
        
            Ux = U(1:3:N*n_var);
            Ux = reshape(Ux,N,K);
            Uy = U(2:3:N*n_var);
            Uy = reshape(Uy,N,K);
            Uz = U(3:3:N*n_var);
            Uz = reshape(Uz,N,K);
            
%             Jx = (Ux(:,2:end)-Ux(:,1:end-1))/h;%jerk_x(U,h,N,K,n_var);
%             Jy = (Uy(:,2:end)-Uy(:,1:end-1))/h;
%             Jz = (Uz(:,2:end)-Uz(:,1:end-1))/h;

            Jx = jerk_x(Ux,h,N,K);
            Jy = jerk_y(Uy,h,N,K);
            Jz = jerk_z(Uz,h,N,K);
            
            % Constraints (bounds) for jerk
            obtain_jerk(Jx,Jy,Jz,N,K) <= j_max
%             Jx(:).^2 + Jy(:).^2 + Jz(:).^2 <= j_max^2
%             % Constraints (bounds) for acceleration   
            obtain_U(Ux,Uy,Uz,N,K) <= u_max
%             Ux(:).^2 + Uy(:).^2 + Uz(:).^2 <= u_max^2 

            Ux(:,K) == 0; %cvx(zeros(N,1));
            Uy(:,K) == 0; %cvx(zeros(N,1));
            Uz(:,K)+Gz(:,K) == 0; %cvx(zeros(N,1));
    %         for i = 1:N
    %            Ux(i,K) == 0;
    %            Uy(i,K) == 0;
    %            Uz(i,K) == 0;
    %         end

            % Velocity constraints
%             v0(:,1) + h*K*sum(Ux(:,1:K-1),2) == 0;
%             v0(:,2) + h*K*sum(Uy(:,1:K-1),2) == 0;
%             v0(:,3) + h*K*sum(Uz(:,1:K-1),2) == 0;

            vel_x_final(v0,Ux,h,N,K) == 0;
            vel_y_final(v0,Uy,h,N,K) == 0;
            vel_z_final(v0,Uz+Gz,h,N,K) == 0;
            
    %         vel_x_final(v0,U,h,N,n_var,K) == 0;
    %         vel_y_final(v0,U,h,N,n_var,K) == 0;
            % Constraints for position (inequality)
            p_min_x<= pos_x(x0,v0,Ux,h,N,K) <= p_max_x
            p_min_y<= pos_y(x0,v0,Uy,h,N,K) <= p_max_y
            p_min_z<= pos_z(x0,v0,Uz+Gz,h,N,K) <= p_max_z

            % Constraints for position (equality)
            target_pos_x(x0,v0,Ux,h,N,K) == xf(:,1)
            target_pos_y(x0,v0,Uy,h,N,K) == xf(:,2)
            target_pos_z(x0,v0,Uz+Gz,h,N,K) == xf(:,3)
            
            % Constraints for position (inequality / avoidance)
            avoidance(xq,yq,zq,x0,v0,Ux,Uy,Uz,Gz,h,N,K) >= R
            expressions xq(N*K) yq(N*K) zq(N*K)
            xq = pos_x(x0,v0,Ux,h,N,K);
            xq = reshape(xq,N,K);
            yq = pos_y(x0,v0,Uy,h,N,K);
            yq = reshape(yq,N,K);
            zq = pos_z(x0,v0,Uz+Gz,h,N,K);
            zq = reshape(zq,N,K);
        cvx_end

        fnew = U'*U;
        [noncvxConsSatisfied, sum] = check_position(xq,yq,zq,R,N,K);
        violated_noncvx_cons = nchoosek(N,2)*K-sum;
        num_vehicles = N;
        num_states = K;
        avoidance_radius = R;
        % If the objective function is NaN, it means the solution will never
        % converge.
        if isnan(fnew)
           disp('Solution will not converge. Retry with new problem parameters.')
           break
        end
        iter = iter+1;   
        % Output convergence information in the command window.
        if iter == maxiter
           solnStat{N-1} = "Solution did not converge within max. number of iterations.";
           t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
               avoidance_radius,iter)
        elseif abs(fold-fnew) <= eps
           solnStat{N-1} = "Solution converged to within function tolerance.";
           disp("Function tolerance satisfied")
           t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
               avoidance_radius,iter)
        elseif noncvxConsSatisfied == 1
           solnStat{N-1} = "Solution converged to within nonconvex constraint tolerance.";
           disp("Non Convex constraints satisfied")
           t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
               avoidance_radius,iter)
        elseif cvx_status == "Solved"
           solnStat{N-1} = "Solution converged to an optimal solution.";
           disp("CVX found an optimal solution")
           t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
               avoidance_radius,iter)       
        end
        t = table(fnew,violated_noncvx_cons,string(cvx_status))
    end
    simTime(N-1) = simTime(N-1) + tic - t_start;
    disp(solnStat{N-1})
    solnTables{N-1} = t;
    % Obtain velocity for each vehicle at each state from U*
    vx = vel_x(v0,U,h,N,n_var,K);
    vy = vel_y(v0,U,h,N,n_var,K);
    % Can use the last computed position from the algorithm for plot
    x = xq;
    y = yq;
    z = zq;
    % Plot solved trajectories
    figure(N-1);
    for i = 1:N
        plot3(x(i,:),y(i,:),z(i,:),'LineStyle','-','LineWidth',2,'Color',color_palette{i})
        hold on
    end

    % Pseudo plots to show the legends for start and point
    start_hand = plot(10000,10000,'k^','MarkerSize',6);
    end_hand   = plot(10000,10000,'kp','MarkerSize',8);
    legend([start_hand,end_hand],{'Start Point','End Point'});

    title("Conflict-free trajectories in 3-D, N = "+N+", R = "+R+" [m]")
    xlabel('x [m]')
    ylabel('y [m]')

    disp("Computation Time [N = "+N+"]: " + double(simTime(N-1))/10^6 + " seconds");
%     keyboard
end
figure('Name','Computational Time')
hold on; grid on; box on;
plot(2:1:N_Quads,double(simTime)/10^6,'bo-');
set(gca,'XTick',[2:1:N_Quads]);
xlabel('Number of quadrotors');
ylabel('Computational Time using CVX solver [s]');

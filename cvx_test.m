clear all
close all
% AAE 561: Final Project 
% Conflict-free trajectories for quadrotors
% Dietsche, Lee and Sudarsanan

N_Quads = 2;       % Number of vehicles
dStartEnd = 10;%20;    % Distance between start and end points for quadrotors
simTime  = zeros(1,N_Quads-1);
solnStat = cell(1,N_Quads-1);
solnTables = cell(1,N_Quads-1);

% Rune single case
SpecificEnergy = zeros(N_Quads-1,1);
for N=N_Quads
  
% % Run multiple cases
% for N=2:N_Quads
    figure(N-1);
    [x0, xf]= setStartAndEndPts(dStartEnd,N); % Set the start and points for each vehicle
    x0 = flipud(x0);
    xf = flipud(xf);
    % Vehicle parameters based on paper by Ardakani, et. al "Online Minimum-Jerk
    % Trajectory Generation" p. 7 figure 4
    u_max = 35;  % [m/s^2] Max acceleration per vehicle
    j_max = 200; % [m/s^3] Max jerk per vehicle    
    v0 = zeros(N,2);                % Set initial velocity to zero for each vehicle
    
%     t_start = tic;
    tic
    R = 1.0; % [m] Min. distance of aviodance
    h = 0.1; % [s] Sampling time
    
    if N <=3
        T = 2;
    else
        T = 5;%+ (N-4);%10;   % [s] Total flight time for each vehicle
    end
    % Estimate flight time
%     T = estimateFlightTime(dStartEnd,u_max); % Assumes distance traveled same for all quads
    K = ceil(T/h)+1;   % Number of states
    n_var = 2*K; % Need twice the number of states for 2-D


    p_max_x(1:N*K,1) = dStartEnd+10; % [m] Define the space allowed to fly in (rectangle)
    p_min_x(1:N*K,1) = 0;            % [m] Define the space allowed to fly in
    p_max_y(1:N*K,1) = dStartEnd+10; % [m] Define the space allowed to fly in
    p_min_y(1:N*K,1) = 0;            % [m] Define the space allowed to fly in

    % Obtain intitial solution to be used for the first approximation of the
    % avoidance constraints. Here, there are no avoidance constraints.
    cvx_begin quiet 
        variable U(N*n_var) % Every n_var is a different vehicle
        minimize( U'*U )
        subject to
        % Constraints (bounds) for jerk
        expressions Jx Jy
        Jx = jerk_x(U,h,N,K,n_var);
        Jy = jerk_y(U,h,N,K,n_var);
        obtain_jerk(Jx,Jy,N,K) <= j_max
        % Constraints (bounds) for acceleration   
        Ux = U(1:2:N*n_var-1);
        Ux = reshape(Ux,N,K);
        Uy = U(2:2:N*n_var);
        Uy = reshape(Uy,N,K);
        obtain_U(Ux,Uy,N,K) <= u_max 
        for i = 1:N
           Ux(i,K) == 0;
           Uy(i,K) == 0;
        end
        % Velocity constraints
        vel_x_final(v0,U,h,N,n_var,K) == 0;
        vel_y_final(v0,U,h,N,n_var,K) == 0;
        % Constraints for position (inequality)
        p_min_x<= pos_x(x0,v0,U,h,N,n_var,K) <= p_max_x
        p_min_y<= pos_y(x0,v0,U,h,N,n_var,K) <= p_max_y
        % Constraints for position (equality)
        target_pos_x(x0,v0,U,h,N,n_var,K) == xf(:,1)
        target_pos_y(x0,v0,U,h,N,n_var,K) == xf(:,2)
    cvx_end

    % Obtain initial trajectories
    xq = recover_x(x0,v0,U,h,N,n_var,K);
    xq = reshape(xq,N,K);
    yq = recover_y(x0,v0,U,h,N,n_var,K);
    yq = reshape(yq,N,K);
    x = xq;
    y = yq;

    % Pause simulation time calculation while plotting
%     simTime(N-1) = simTime(N-1) + tic - t_start;

    % Plot initial trajectories (no avoidance)
    color_palette = {};
    for i = 1:N
        p_hand = plot(x(i,:),y(i,:),'LineStyle','--','LineWidth',2);
        color_palette{i} = p_hand.Color;
        plot(x0(i,1),x0(i,2),'^','MarkerSize',6,'MarkerEdgeColor',p_hand.Color,...
                                                 'MarkerFaceColor',p_hand.Color);
        plot(xf(i,1),xf(i,2),'p','MarkerSize',8,'MarkerEdgeColor',p_hand.Color,...
                                                 'MarkerFaceColor',p_hand.Color);
            
    end
    title("Initial trajectories in 2-D ["+N+" Quads]")
    xlabel('x [m]')
    ylabel('y [m]')
    drawnow

    % Resume simulation time logging
%     t_start = tic;

    eps = 1e-4;
    fold = U'*U; fnew = 0;
    cvx_status = "Infeasible";
    noncvxConsSatisfied = 0;
    maxiter = 10;
    iter = 1;

    % The main loop for enforcing avoidance constraints using the linear
    % approximation formulated by Augugliaro
%     while abs(fold-fnew) > eps && noncvxConsSatisfied == 0 && iter < maxiter && cvx_status ~= "Solved"
%     while ((abs(fold-fnew) > eps && cvx_status ~= "Solved") || ~noncvxConsSatisfied) && iter <= maxiter
    while abs(fold-fnew) > eps && ~noncvxConsSatisfied && iter <= maxiter
        fold = U'*U;

        cvx_begin quiet
            variable U(N*n_var) % Every n_var is a different vehicle
            minimize( U'*U )
            subject to
            % Constraints (bounds) for jerk
            expressions Jx Jy
            Jx = jerk_x(U,h,N,K,n_var);
            Jy = jerk_y(U,h,N,K,n_var);
            obtain_jerk(Jx,Jy,N,K) <= j_max
            % Constraints (bounds) for acceleration   
            Ux = U(1:2:N*n_var-1);
            Ux = reshape(Ux,N,K);
            Uy = U(2:2:N*n_var);
            Uy = reshape(Uy,N,K);
            obtain_U(Ux,Uy,N,K) <= u_max
            for i = 1:N
               Ux(i,K) == 0;
               Uy(i,K) == 0;
            end
            % Velocity constraints
            vel_x_final(v0,U,h,N,n_var,K) == 0;
            vel_y_final(v0,U,h,N,n_var,K) == 0;
            % Constraints for position (inequality)
            p_min_x<= pos_x(x0,v0,U,h,N,n_var,K) <= p_max_x
            p_min_y<= pos_y(x0,v0,U,h,N,n_var,K) <= p_max_y
            % Constraints for position (equality)
            target_pos_x(x0,v0,U,h,N,n_var,K) == xf(:,1)
            target_pos_y(x0,v0,U,h,N,n_var,K) == xf(:,2)
            % Constraints for position (inequality / avoidance)
            avoidance(xq,yq,x0,v0,U,h,N,n_var,K) >= R
            expressions xq(N*K) yq(N*K)
            xq = pos_x(x0,v0,U,h,N,n_var,K);
            xq = reshape(xq,N,K);
            yq = pos_y(x0,v0,U,h,N,n_var,K);
            yq = reshape(yq,N,K);
        cvx_end

        fnew = U'*U;
        [noncvxConsSatisfied, sum] = check_position(xq,yq,R,N,K);
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
    end
    
    if abs(fold-fnew)>eps && ~noncvxConsSatisfied
        disp('No solution found')
    elseif ~noncvxConsSatisfied
        disp('Converged. But constraints are not satisfied')
    end
%     simTime(N-1) = simTime(N-1) + tic - t_start;
%     disp(solnStat{N-1})
%     solnTables{N-1} = t;
    toc
    % Obtain velocity for each vehicle at each state from U*
    vx = vel_x(v0,U,h,N,n_var,K);
    vy = vel_y(v0,U,h,N,n_var,K);
    % Can use the last computed position from the algorithm for plot
    x = xq;
    y = yq;

    % Plot solved trajectories
    figure(N-1);
    for i = 1:N
        plot(x(i,:),y(i,:),'LineStyle','-','LineWidth',2,'Color',color_palette{i})
        hold on
    end

    % Pseudo plots to show the legends for start and point
    start_hand = plot(10000,10000,'k^','MarkerSize',6);
    end_hand   = plot(10000,10000,'kp','MarkerSize',8);
    legend([start_hand,end_hand],{'Start Point','End Point'});
    title("Conflict-free trajectories in 2-D, N = "+N+", R = "+R+" [m]")
    xlabel('x [m]')
    ylabel('y [m]')
    simTrajectories(x,y);

    disp("Computation Time [N = "+N+"]: " + double(simTime(N-1))/10^6 + " seconds");
    SE = 0;
    for i = 1:N
       SE = vx(i,:)*vx(i,:)'+vy(i,:)*vy(i,:)';
    end
    SpecificEnergy(N-1,1) = SE;
end

figure('Name','Computational Time')
hold on; grid on; box on;
plot(2:1:N_Quads,double(simTime)/10^6,'bo-');
set(gca,'XTick',[2:1:N_Quads]);
xlabel('Number of quadrotors');
ylabel('Computational Time using CVX solver [s]');


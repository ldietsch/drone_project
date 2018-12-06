clear all
close all
% AAE 561: Final Project 
% Conflict-free trajectories for quadrotors
% Dietsche, Lee and Sudarsanan

N_Quads = 5;       % Number of vehicles
dStartEnd = 10;%20;    % Distance between start and end points for quadrotors
simTime  = zeros(1,N_Quads-1);
solnStat = cell(1,N_Quads-1);
solnTables = cell(1,N_Quads-1);

% Rune single case
SpecificEnergy = zeros(N_Quads-1,1);
for N=2:N_Quads
% % Run multiple cases
% for N=2:N_Quads
    figure(N-1);
    [x0, xf]= setStartAndEndPts(dStartEnd,N); % Set the start and points for each vehicle
    % Vehicle parameters based on paper by Ardakani, et. al "Online Minimum-Jerk
    % Trajectory Generation" p. 7 figure 4
%     x0 = flipud(x0);
%     xf = flipud(xf);
    u_max = 35;  % [m/s^2] Max acceleration per vehicle
    j_max = 200; % [m/s^3] Max jerk per vehicle    
    v0 = zeros(N,2);                % Set initial velocity to zero for each vehicle
    R = 1.0; % [m] Min. distance of aviodance
    h = 0.1; % [s] Sampling time
    tic
    if N <=3
        T = 5;
    else
        T = 5;%10;   % [s] Total flight time for each vehicle
    end
    
    % Estimate flight time
    K = ceil(T/h)+1;   % Number of states
    n_var = 2*K; % Need twice the number of states for 2-D

    p_max_x = dStartEnd+10; % [m] Define the space allowed to fly in (rectangle)
    p_min_x = 0;            % [m] Define the space allowed to fly in
    p_max_y = dStartEnd+10; % [m] Define the space allowed to fly in
    p_min_y = 0;            % [m] Define the space allowed to fly in
    
    globalIter = 0;
    % Obtain intitial solution to be used for the first approximation of the
    % avoidance constraints. Here, there are no avoidance constraints.
    cvx_begin quiet 
        variable U(N*n_var) % Every n_var is a different vehicle
        minimize( U'*U )
        subject to
        
        % Reshape U for setting up constraints
        Ux = U(1:2:N*n_var-1);
        Ux = reshape(Ux,N,K);
        Uy = U(2:2:N*n_var);
        Uy = reshape(Uy,N,K);
        
        % Calculate jerk
        expressions Jx Jy
%         Jx = (Ux(:,2:end)-Ux(:,1:end-1))/h; 
%         Jy = (Uy(:,2:end)-Uy(:,1:end-1))/h; 
        
        
        
        Jx = (U(3:2:end)-U(1:2:end-2))/h;
        Jy = (U(4:2:end)-U(2:2:end-2))/h;
        
        % Constraints (bounds) for jerk
        Jx(:).^2 + Jy(:).^2 <= j_max^2
        % Constraints (bounds) for acceleration   
%         Ux(:).^2 + Uy(:).^2 <= u_max^2
        U(1:2:end).^2 + U(2:2:end).^2 <= u_max^2;
        
        % Final acc = 0
        U(n_var-1:n_var:end) == 0
        U(n_var:n_var:end)   == 0
        
        % Velocity constraints
        vel_x_final_opt(v0,Ux,h,K) == 0;
        vel_y_final_opt(v0,Uy,h,K) == 0;
       
        % Constraints for position (inequality)
        p_min_x<= pos_x_opt(x0,v0,Ux,h,K) <= p_max_x
        p_min_y<= pos_y_opt(x0,v0,Uy,h,K) <= p_max_y
        
        % Constraints for position (equality)
        target_pos_x_opt(x0,v0,Ux,h,K) == xf(:,1)
        target_pos_y_opt(x0,v0,Uy,h,K) == xf(:,2)
    cvx_end
    disp(cvx_slvitr);
    % Obtain initial trajectories
    xq = recover_x_opt(x0,v0,Ux,h,K);
    xq = reshape(xq,N,K);
    yq = recover_y_opt(x0,v0,Uy,h,K);
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
        disp("Iteration: "+iter);
        cvx_begin quiet
            variable U(N*n_var) % Every n_var is a different vehicle
            
            
            % Reshape U for setting up constraints
            Ux = U(1:2:N*n_var-1);
            Ux = reshape(Ux,N,K);
            Uy = U(2:2:N*n_var);
            Uy = reshape(Uy,N,K);
            
            % Calculate jerk
            expressions Jx Jy
                    Jx = (Ux(:,2:end)-Ux(:,1:end-1))/h;
                    Jy = (Uy(:,2:end)-Uy(:,1:end-1))/h;
            
            minimize(U'*U)
            subject to
            
%             Jx = (U(3:2:end)-U(1:2:end-2))/h;
%             Jy = (U(4:2:end)-U(2:2:end-2))/h;
            
            % Constraints (bounds) for jerk
            Jx(:).^2 + Jy(:).^2 <= j_max^2
            % Constraints (bounds) for acceleration
            U(1:2:end).^2 + U(2:2:end).^2 <= u_max^2;
            % Final acc = 0
            U(n_var-1:n_var:end) == 0
            U(n_var:n_var:end)   == 0
            
            % Velocity constraints
            vel_x_final_opt(v0,Ux,h,K) == 0;
            vel_y_final_opt(v0,Uy,h,K) == 0;
            
            % Constraints for position (inequality)
            p_min_x<= pos_x_opt(x0,v0,Ux,h,K) <= p_max_x
            p_min_y<= pos_y_opt(x0,v0,Uy,h,K) <= p_max_y
            
            % Constraints for position (equality)
            target_pos_x_opt(x0,v0,Ux,h,K) == xf(:,1)
            target_pos_y_opt(x0,v0,Uy,h,K) == xf(:,2)
            
            % Constraints for position (inequality / avoidance)
            avoidance_opt(xq,yq,x0,v0,Ux,Uy,h,N,K) >= R
            expressions xq(N*K) yq(N*K)
            xq = pos_x_opt(x0,v0,Ux,h,K);
            xq = reshape(xq,N,K);
            yq = pos_y_opt(x0,v0,Uy,h,K);
            yq = reshape(yq,N,K);
        cvx_end
        disp(cvx_slvitr);
        fnew = U'*U;
        [noncvxConsSatisfied, sum, violations] = check_position_opt(xq,yq,R,N,K);
        violated_noncvx_cons = nchoosek(N,2)*K-sum;
        num_vehicles = N;
        num_states = K;
        avoidance_radius = R;
        disp("Violated distances:")
        disp(violations);
        % If the objective function is NaN, it means the solution will never
        % converge.
        if isnan(fnew)
           disp('Solution will not converge. Retry with new problem parameters.')
           break
        end
        iter = iter+1;   
%         % Output convergence information in the command window.
%         if iter == maxiter
%            solnStat{N-1} = "Solution did not converge within max. number of iterations.";
%            t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
%                avoidance_radius,iter)
%         elseif abs(fold-fnew) <= eps
%            solnStat{N-1} = "Solution converged to within function tolerance.";
%            disp("Function tolerance satisfied")
%            t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
%                avoidance_radius,iter)
%         elseif noncvxConsSatisfied == 1
%            solnStat{N-1} = "Solution converged to within nonconvex constraint tolerance.";
%            disp("Non Convex constraints satisfied")
%            t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
%                avoidance_radius,iter)
%         elseif cvx_status == "Solved"
%            solnStat{N-1} = "Solution converged to an optimal solution.";
%            disp("CVX found an optimal solution")
%            t = table(fnew,violated_noncvx_cons,num_vehicles,num_states,...
%                avoidance_radius,iter)       
%         end
    end
%     simTime(N-1) = simTime(N-1) + tic - t_start;
%     disp(solnStat{N-1})
%     solnTables{N-1} = t;
    if iter == maxiter+1 && ~noncvxConsSatisfied
        disp('No solution found');
    elseif ~noncvxConsSatisfied
        disp('Solution converged, but constraints are violated');
    else
        disp('Solution found');
    end
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
    simTrajectories(x,y,color_palette);
    SE = 0;
    for i = 1:N
       SE = SE + vx(i,:)*vx(i,:)'+vy(i,:)*vy(i,:)';
    end
    SpecificEnergy(N-1,1) = SE;
%     disp("Computation Time [N = "+N+"]: " + double(simTime(N-1))/10^6 + " seconds");
end
% figure('Name','Computational Time')
% hold on; grid on; box on;
% plot(2:1:N_Quads,double(simTime)/10^6,'bo-');
% set(gca,'XTick',[2:1:N_Quads]);
% xlabel('Number of quadrotors');
% ylabel('Computational Time using CVX solver [s]');
% 

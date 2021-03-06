function [start_pts,end_pts] = setStartAndEndPts(diam,n)    
    R         = diam/2;
    center_pt = [5,5] + R;
    [start_pts,end_pts] = convergingPaths(R,center_pt,n);
%     [start_pts,end_pts] = paper2DScenario(R,center_pt);
    gcf;
    hold on; grid on; box on;
    lims = [min(center_pt)-R-5,max(center_pt)+R+5];
    xlim(lims); ylim(lims);
end

function [start_pts,end_pts] = paper2DScenario(R,center_pt)    
    start_pts(1,:) = center_pt + R*[0,1];
    end_pts(1,:)   = center_pt - R*[0,1];
    
    start_pts(2,:) = center_pt - R*[1.2*cos(3*pi/4),sin(3*pi/4)];
    end_pts(2,:)   = start_pts(2,:) + 2*R*[0,1];
    
    start_pts(3,:) = center_pt - R*[cos(pi/4),sin(3*pi/4)];
    end_pts(3,:)   = start_pts(3,:) + 2*R*[0,1];
    
    start_pts(4,:) = center_pt + 1.2*R*[cos(pi/4),sin(3*pi/4)];
    end_pts(4,:) = center_pt - R*[cos(pi/4),sin(3*pi/4)];
    
    start_pts(5,:) = center_pt + R*[cos(3*pi/4),sin(3*pi/4)];
    end_pts(5,:) = center_pt - R*[cos(3*pi/4),sin(3*pi/4)];
end

function [start_pts,end_pts] = convergingPaths(R,center_pt,n)
    theta = 0; 
    dtheta = pi/n; % Easier to find solutions
%     dtheta = pi/6; 
    for ii=1:n
        start_pts(ii,:) = center_pt + R*[cos(theta),sin(theta)];
        end_pts(ii,:)   = center_pt - R*[cos(theta),sin(theta)];
        theta = theta + dtheta;
    end
end
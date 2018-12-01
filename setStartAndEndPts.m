function [start_pts,end_pts] = setStartAndEndPts(diam,n)    
    R         = diam/2;
    center_pt = [5,5] + R;
    theta = 0; 
    %dtheta = pi/n; % Easier to find solutions
    dtheta = pi/6; 
    for ii=1:n
        start_pts(ii,:) = center_pt + R*[cos(theta),sin(theta)];
        end_pts(ii,:)   = center_pt - R*[cos(theta),sin(theta)];
        theta = theta + dtheta;
    end
    gcf
    hold on; grid on; box on;
    lims = [min(center_pt)-R-5,max(center_pt)+R+5];
    xlim(lims); ylim(lims);
end
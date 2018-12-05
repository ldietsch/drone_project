function [start_pts,end_pts] = setStartAndEndPts2(n)
    center_pt = [10,10];
    R         = 5;
    theta = 0; dtheta = pi/n;
    for ii=1:n
        start_pts(ii,:) = center_pt + R*[cos(theta),sin(theta)];
        end_pts(ii,:)   = center_pt - R*[cos(theta),sin(theta)];
        theta = theta + dtheta;
    end
    
    lims = [min(center_pt)-R-10,max(center_pt)+R+10];
    xlim(lims);ylim(lims);
end
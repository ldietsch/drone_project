function [start_pts,end_pts] = setStartAndEndPts(n)
    center_pt = [5,5];
    R         = 5;
    theta = 0; dtheta = pi/3;
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
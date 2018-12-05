function simTrajectories2(Px,Py)
    N = size(Px,1);
    K = size(Px,2);
    
    movieName = ['2D Conflict Avoidance_',num2str(N),' Quads.avi'];
    plotMovie = VideoWriter(movieName); % Name it.
    plotMovie.FrameRate = 10; % How many frames per second.
    open(plotMovie);
    gcf;
    hold on
    % Plot initial location
    for ii=1:N
        p_hand(ii) = plot(Px(ii,1),Py(ii,1),'Marker','s','MarkerSize',8,...
            'MarkerEdgeColor','k','MarkerFaceColor','k');%color_palette{ii});
        circ_hand(ii) = drawCircle([Px(ii,1);Py(ii,1)],0.5);
    end
    ax = gca;
    ax.Legend.String(3:end) = [];    
    for jj=1:K
        for ii=1:N
            p_hand(ii).XData = Px(ii,jj);
            p_hand(ii).YData = Py(ii,jj);
            drawCircle([Px(ii,jj);Py(ii,jj)],0.5,circ_hand(ii));
        end
        drawnow
        frame = getframe(gcf);
        writeVideo(plotMovie, frame)
        pause(0.2);
    end
    close(plotMovie);
end

function h = drawCircle(center,R,h)
    alpha = 0:0.1:2*pi;
    circ  = center + R*[cos(alpha);sin(alpha)];
    if nargin == 2
        h     = plot(circ(1,:),circ(2,:),'b');
    elseif nargin == 3
        h.XData = circ(1,:);
        h.YData = circ(2,:);
    end
end
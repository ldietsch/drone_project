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
%         p_hand(ii) = plot(Px(ii,1),Py(ii,1),'Marker','s','MarkerSize',8,...
%             'MarkerEdgeColor','k','MarkerFaceColor','k');%color_palette{ii});
        p_hand(ii) = plot(Px(ii,1),Py(ii,1),'Marker','s','MarkerSize',8,...
            'MarkerEdgeColor','k','MarkerFaceColor','k');%color_palette{ii});
    end
    ax = gca;
    ax.Legend.String(3:end) = [];    
    for jj=1:K
        for ii=1:N
            p_hand(ii).XData = Px(ii,jj);
            p_hand(ii).YData = Py(ii,jj);
        end
        drawnow
        frame = getframe(gcf);
        writeVideo(plotMovie, frame)
        pause(0.2);
    end
    close(plotMovie);
end
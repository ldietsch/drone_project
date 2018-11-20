classdef Quad < handle
    properties (Constant)
        v0 = 0;% Initial velocity
        a0 = 1;% Initial acceleration
    end
    properties
        id
        vel_profile
        acc_profile
        pos_profile
        
        start_pt 
        end_pt
        
        
        % Plotting attributes
        plot_handle = [];
        plot_color
    end
    methods
        function obj = Quad(id,start_pt,end_pt,T)
            obj.id = id;
            
            obj.start_pt = start_pt;
            obj.end_pt   = end_pt;
            
            D = norm(obj.end_pt-obj.start_pt);
            obj.pos_profile = obj.start_pt + (0:D/T:D)'*(obj.end_pt-obj.start_pt)/D;
            obj.vel_profile = zeros(size(obj.pos_profile));
            obj.acc_profile = obj.vel_profile;
            obj.plot_color  = obj.getColor();
            
            obj.plotStartEndPts();
        end
        
        function plotStartEndPts(obj)
            plot(obj.start_pt(1),obj.start_pt(2),'Marker','s','MarkerSize',8,...
                'MarkerFaceColor',obj.plot_color,'MarkerEdgeColor','k');
            plot(obj.end_pt(1),obj.end_pt(2),'Marker','^','MarkerSize',7,...
                'MarkerFaceColor',obj.plot_color,'MarkerEdgeColor','k');
        end
        
        function plotTraj(obj)
            if isempty(obj.plot_handle)
                obj.plot_handle = plot(obj.pos_profile(:,1),obj.pos_profile(:,2),...
                    'Color',obj.plot_color,'LineWidth',2.0);
            else
                obj.plot_handle.XData = obj.pos_profile(:,1);
                obj.plot_handle.YData = obj.pos_profile(:,2);
            end
        end
        
        function plot_color = getColor(obj)
            color_palette = ['r','b','g','k','c','m'];
            plot_color = color_palette(obj.id);
        end
    end
end
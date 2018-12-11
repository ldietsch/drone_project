% Set Font properties
ax = gca;
set(findall(ax,'-property','Font'),'Font','Times')
set(findall(ax,'-property','FontSize'),'FontSize',12)
set(findall(ax,'-property','Interpreter'),'Interpreter','latex')
set(findall(ax,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
ff = gcf;
ff.Position(3:4) = [436,347];
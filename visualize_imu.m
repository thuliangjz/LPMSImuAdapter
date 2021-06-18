close all
fig = figure;
visualizer = Visualizer(1, {'acc', 'pose'});
visualizer.SetPlotFigureHandle(fig);
visualizer.PlotForever();
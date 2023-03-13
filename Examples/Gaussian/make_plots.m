%%%%%%%%%%%%%%%%%%%%%%%%%%
%% make some plots
%%%%%%%%%%%%%%%%%%%%%%%%%%


colors = [224,   0,   0; % red
           30, 144  255; % dark blue
            0, 170,  85; % green
          118,   0, 168; % purple
           46,  52,  59; % grey
          236, 176,  31;  % yellow
           76, 189, 237; % light blue
          161,  19,  46  % dark red
           ] ./ 255;
       
plot_symbols = ['o', 'd', '^', 'h', 'v', '>', 'p', 's'];

F_xy = [1,2,3,4];

fig = figure();
fig.WindowState = 'maximized';

subplot(7,20,[1:20]);
hold on 


patch('Faces',F_xy,'Vertices', Polyhedron([-eye(2); eye(2)], ones(4,1)).V,...
    'FaceColor',  'white', ...
    'EdgeColor', 'black', ...
    'FaceAlpha', 0); 

plot(nan, nan, 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none');


for i = 1:4
   plot(nan, nan, 'Color', colors(i, :), 'Marker', plot_symbols(i), 'MarkerFaceColor', colors(i, :), 'MarkerSize', 10);
end

plots=get(gca, 'Children');

legend([plots(6), plots(5), plots(4), plots(3), plots(2), plots(1)], ...
     {'Target Set', 'Initial Location', 'Proposed', 'Quantiles', 'Scenario', 'Particle Control'},...
    'Orientation','horizontal', ...
    'Location', 'south', ...
    'NumColumns', 6, ...
    'interpreter', 'latex',...
    'FontSize',11);

axis([0 0.1 0 0.1]);
axis off
hold off


subplot(7,20,[21:140]);
hold on

patch('Faces',[1,3,4,2],'Vertices', Polyhedron(G_N(1:4,1:2), h_N(1:4)).V,...
    'FaceColor', colors(5,:), ...
    'FaceAlpha', 0.2);
patch('Faces',[1,2,3],'Vertices', Polyhedron(G_k(:,1:2), h_k).V,...
    'FaceColor', colors(6,:), ...
    'FaceAlpha', 0.1);  

plot(x_mean_our_method(1:4:end), x_mean_our_method(2:4:end), 'Color', colors(1, :), 'Marker', plot_symbols(1), 'MarkerFaceColor', colors(1, :), 'MarkerSize', 10, 'LineStyle', 'none');
plot([x_0(1); x_mean_our_method(1:4:end)], [x_0(2); x_mean_our_method(2:4:end)], 'Color', colors(1, :), 'Marker','none');
plot(x_mean_quantile(1:4:end), x_mean_quantile(2:4:end), 'Color', colors(2, :), 'Marker', plot_symbols(2), 'MarkerFaceColor', colors(2, :), 'MarkerSize', 10, 'LineStyle', 'none');
plot([x_0(1); x_mean_quantile(1:4:end)], [x_0(2); x_mean_quantile(2:4:end)], 'Color', colors(2, :), 'Marker','none');
plot(x_mean_scenario(1:4:end), x_mean_scenario(2:4:end), 'Color', colors(3, :), 'Marker', plot_symbols(3), 'MarkerFaceColor', colors(3, :), 'MarkerSize', 10, 'LineStyle', 'none');
plot([x_0(1); x_mean_scenario(1:4:end)], [x_0(2); x_mean_scenario(2:4:end)], 'Color', colors(3, :), 'Marker','none');
plot(x_mean_pc(1:4:end), x_mean_pc(2:4:end), 'Color', colors(4, :), 'Marker', plot_symbols(4), 'MarkerFaceColor', colors(4, :), 'MarkerSize', 10, 'LineStyle', 'none');
plot([x_0(1); x_mean_pc(1:4:end)], [x_0(2); x_mean_pc(2:4:end)], 'Color', colors(4, :), 'Marker','none');
plot(x_0(1), x_0(2), 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none','MarkerSize',10);


ax = gca; 
ax.FontSize = 8; 
xlabel('x (in meters)')
ylabel('y (in meters)')

hold off



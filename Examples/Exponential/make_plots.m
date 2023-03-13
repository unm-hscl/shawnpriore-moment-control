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

subplot(5,20,[1:20]);
hold on 

plot(nan, nan, 'Color', 'k', 'Marker', 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w', 'LineStyle', ':');

plot(nan, nan, 'Color', 'k', 'Marker', 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', '-');


patch('Faces',F_xy,'Vertices', Polyhedron([-eye(2); eye(2)], ones(4,1)).V,...
    'FaceColor',  'white', ...
    'EdgeColor', 'black', ...
    'FaceAlpha', 0); 

plot(nan, nan, 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none');


for i = 1:size(x_mean_our_method,2)
   plot(nan, nan, 'Color', colors(i, :), 'Marker', plot_symbols(i));
end

plots=get(gca, 'Children');

legend([plots(3), plots(2), plots(1), plots(5), plots(4), plots(6), plots(7)], ...
     {'Dep 1', 'Dep 2', 'Dep 3', 'Target Set', 'Initial Location', 'Proposed', 'Cantelli MPC'},...
    'Orientation','horizontal', ...
    'Location', 'south', ...
    'NumColumns', 7, ...
    'interpreter', 'latex',...
    'FontSize',11);

axis([0 0.1 0 0.1]);
axis off
hold off


subplot(5,20,[21:25, 41:45, 61:65]);
hold on

patch('Faces',F_xy,'Vertices', Polyhedron(target_set_a.A([1;2;5;6],1:2), target_set_a.b([1;2;5;6])).V,...
    'FaceColor', colors(1,:), ...
    'FaceAlpha', 0.1); 
patch('Faces',F_xy,'Vertices', Polyhedron(target_set_b.A([1;2;5;6],1:2), target_set_b.b([1;2;5;6])).V,...
    'FaceColor', colors(2,:), ...
    'FaceAlpha', 0.1); 
patch('Faces',F_xy,'Vertices', Polyhedron(target_set_c.A([1;2;5;6],1:2), target_set_c.b([1;2;5;6])).V,...
    'FaceColor', colors(3,:), ...
    'FaceAlpha', 0.1); 

for i = 1:size(x_mean_our_method,2)
   plot(x_mean_proposed(end-3,i), x_mean_proposed(end-2,i), 'Color', colors(i, :), 'Marker', plot_symbols(i), 'MarkerFaceColor', colors(i, :), 'MarkerSize', 10);
   plot(x_mean_cantelli(end-3,i), x_mean_cantelli(end-2,i), 'Color', colors(i, :), 'Marker', plot_symbols(i), 'LineStyle', '--', 'MarkerSize', 10);
end

xlabel('x (in meters)')

ylabel('y (in meters)')

ax = gca; 
ax.FontSize = 8; 

axis([-12 12 -12 12])
axis equal

hold off

subplot(5,20,[27:40, 47:60, 67:80]);
hold on

patch('Faces',F_xy,'Vertices', Polyhedron(target_set_a.A([1;2;5;6],1:2), target_set_a.b([1;2;5;6])).V,...
    'FaceColor', colors(1,:), ...
    'FaceAlpha', 0.1); 
patch('Faces',F_xy,'Vertices', Polyhedron(target_set_b.A([1;2;5;6],1:2), target_set_b.b([1;2;5;6])).V,...
    'FaceColor', colors(2,:), ...
    'FaceAlpha', 0.1); 
patch('Faces',F_xy,'Vertices', Polyhedron(target_set_c.A([1;2;5;6],1:2), target_set_c.b([1;2;5;6])).V,...
    'FaceColor', colors(3,:), ...
    'FaceAlpha', 0.1); 

for i = 1:size(x_mean_our_method,2)
   plot(x_mean_proposed(1:4:end,i), x_mean_proposed(2:4:end,i), 'Color', colors(i, :), 'Marker', plot_symbols(i), 'MarkerFaceColor', colors(i, :), 'MarkerSize', 10, 'LineStyle', 'none');
   plot(x_mean_cantelli(1:4:end,i), x_mean_cantelli(2:4:end,i), 'Color', colors(i, :), 'Marker', plot_symbols(i), 'MarkerSize', 10, 'LineStyle', 'none');
   plot([x_0(1,i); x_mean_proposed(1:4:end,i)], [x_0(2,i); x_mean_proposed(2:4:end,i)], 'Color', colors(i, :), 'Marker','none');
   plot([x_0(1,i); x_mean_cantelli(1:4:end,i)], [x_0(2,i); x_mean_cantelli(2:4:end,i)], 'Color', colors(i, :), 'Marker', 'none', 'LineStyle', ':');
end
plot(x_0(1,:), x_0(2,:), 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none','MarkerSize',10);


ax = gca; 
ax.FontSize = 8; 
xlabel('x (in meters)')


hold off



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

plot(nan, nan, 'Color', colors(4,:), 'Marker', plot_symbols(4));


patch('Faces',F_xy,'Vertices', Polyhedron([-eye(2); eye(2)], ones(4,1)).V,...
    'FaceColor',  'white', ...
    'EdgeColor', 'black', ...
    'FaceAlpha', 0); 

plot(nan, nan, 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none');


for i = 1:size(x_mean_our_method,2)
   plot(nan, nan, 'Color', colors(i, :), 'Marker', plot_symbols(i));
end

plots=get(gca, 'Children');

legend([plots(3), plots(2), plots(1), plots(6), plots(5), plots(4)], ...
     {'UAV 1', 'UAV 2', 'UAV 3', 'MAV', 'Target Set', 'Initial Location' },...
    'Orientation','horizontal', ...
    'Location', 'south', ...
    'NumColumns', 6, ...
    'interpreter', 'latex',...
    'FontSize',12);

axis([0 0.1 0 0.1]);
axis off
hold off

subplot(5,20,[21, 41]);

title('Proposed Method', 'position',[0 0.5], 'FontSize', 10)
set(get(gca,'Title'),'Rotation',90)
axis off

subplot(5,20,[22:25, 42:45]);
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
   plot(x_mean_our_method(end-3,i), x_mean_our_method(end-2,i), 'Color', colors(i, :), 'Marker', plot_symbols(i));
end
plot(x_0(1,:), x_0(2,:), 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none','MarkerSize',10);

plot(x_mav_mean(end-3), x_mav_mean(end-2), 'Color', colors(4,:), 'Marker', plot_symbols(4));


ylabel('y (in meters)')
axis([-150 50 -100 100])

ax = gca; 
ax.FontSize = 8; 

axis equal
hold off

subplot(5,20,[27:40, 47:60]);
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
   plot([x_0(1,i); x_mean_our_method(1:4:end,i)], [x_0(2,i); x_mean_our_method(2:4:end,i)], 'Color', colors(i, :), 'Marker', plot_symbols(i));
end
plot(x_0(1,:), x_0(2,:), 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none','MarkerSize',10);

plot([x_0_mav(1); x_mav_mean(1:4:end)], [x_0_mav(2); x_mav_mean(2:4:end)], 'Color', colors(4,:), 'Marker', plot_symbols(4));


axis([-300 inf -150 150])

ax = gca; 
ax.FontSize = 8; 

hold off

subplot(5,20,[61, 81]);

title('Particle Control', 'position',[0 0.5], 'FontSize', 10)
set(get(gca,'Title'),'Rotation',90)
axis off


subplot(5,20,[62:65, 82:85]);
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
   plot(x_mean_pc(end-3,i), x_mean_pc(end-2,i), 'Color', colors(i, :), 'Marker', plot_symbols(i));
end
plot(x_0(1,:), x_0(2,:), 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none','MarkerSize',10);

plot(x_mav_mean(end-3), x_mav_mean(end-2), 'Color', colors(4,:), 'Marker', plot_symbols(4));


xlabel('x (in meters)')
ylabel('y (in meters)')

axis([-150 50 -100 100])

axis equal

ax = gca; 
ax.FontSize = 8; 

hold off

subplot(5,20,[67:80, 87:100]);
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
   plot([x_0(1,i); x_mean_pc(1:4:end,i)], [x_0(2,i); x_mean_pc(2:4:end,i)], 'Color', colors(i, :), 'Marker', plot_symbols(i), 'LineStyle', ':');
end
plot(x_0(1,:), x_0(2,:), 'Marker', 's', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'LineStyle', 'none','MarkerSize',10);
plot([x_0_mav(1); x_mav_mean(1:4:end)], [x_0_mav(2); x_mav_mean(2:4:end)], 'Color', colors(4,:), 'Marker', plot_symbols(4));


xlabel('x (in meters)')

ax = gca; 
ax.FontSize = 8; 

axis([-300 inf -150 150])


hold off
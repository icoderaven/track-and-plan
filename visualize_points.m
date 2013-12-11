% cleaned_points = point4;
% cleaned_points(cleaned_points > 6000) = 0;
% cleaned_points(isnan(cleaned_points)) = 0;
% idx = all(cleaned_points, 2);
% final_pts = point4(idx, :);
final_pts = point4;
close all;

figure(1);
hold on;
plot3(final_pts(:,1), final_pts(:,2),  final_pts(:,3), 'b.');
%plot3(best_pose(1), best_pose(2), best_pose(3), 'rx');
axis equal;
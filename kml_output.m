% 将轨迹输出为kml文件，文件位置.\temp

kmlwriteline('.\temp\traj.kml', output_pos(:,1),output_pos(:,2), 'Color','r', 'Width',2);

% kmlwriteline('.\temp\traj.kml', output_filter(:,1),output_filter(:,2), 'Color','r', 'Width',2);
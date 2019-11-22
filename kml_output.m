% 将轨迹输出为kml文件，文件位置.\temp

kmlwriteline('.\temp\traj.kml', output.pos(:,1),output.pos(:,2), 'Color','r', 'Width',2);

% kmlwriteline('.\temp\traj.kml', pos.GPS(:,1),pos.GPS(:,2), 'Color','r', 'Width',2);
% kmlwriteline('.\temp\traj.kml', pos.BDS(:,1),pos.BDS(:,2), 'Color','r', 'Width',2);
% kmlwriteline('.\temp\traj.kml', pos.GPS_BDS(:,1),pos.GPS_BDS(:,2), 'Color','r', 'Width',2);
% 将轨迹输出为kml文件，文件位置.\temp

kmlwriteline('.\temp\traj.kml', output.pos(:,1),output.pos(:,2), 'Color','r', 'Width',2);

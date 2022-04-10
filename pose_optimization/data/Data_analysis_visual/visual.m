clearvars

gpskitti20110930drive0018 = importfile("../gps_kitti_2011_09_30_drive_0018.txt", [2, Inf]);
optpose = importfile1("../../optimized_poses.txt", [2, Inf]);
optpose_floam = importfile1("../../optimized_poses_without_imu.txt", [2, Inf]);

alt = 10;  % 10 meters is an approximate altitude in Boston, MA
origin = [gpskitti20110930drive0018(1,1), gpskitti20110930drive0018(1,2), gpskitti20110930drive0018(1,3)];
[xEast,yNorth] = latlon2local(gpskitti20110930drive0018(:,1),gpskitti20110930drive0018(:,2),gpskitti20110930drive0018(:,3),origin);

figure
plot(xEast, yNorth, 'DisplayName','Groundtruth')
hold on
plot(optpose_floam(:,2),optpose_floam(:,3),'DisplayName','WithoutIMU')
hold on

legend
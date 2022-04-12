clearvars

gpskitti20110930drive0018 = importfile("../gps_kitti_2011_09_30_drive_0018.txt", [2, Inf]);
gpskitti_t= importfile2("../gps_kitti_2011_09_30_drive_0018.txt", [2, Inf]);
optpose = importfile1("../../optimized_poses_with_imu_aloam.txt", [2, Inf]);
optpose_floam = importfile1("../../optimized_poses_without_imu_aloam.txt", [2, Inf]);

origin = [gpskitti20110930drive0018(1,1), gpskitti20110930drive0018(1,2), gpskitti20110930drive0018(1,3)];
[xEast,yNorth] = latlon2local(gpskitti20110930drive0018(:,1),gpskitti20110930drive0018(:,2),gpskitti20110930drive0018(:,3),origin);

% sync_time
gpskitti_t = gpskitti_t - gpskitti_t(1);
gpskitti_t =    gpskitti_t/10^9;

t_post = optpose(:,1) - optpose(1,1);
t_post = t_post/10^9;

t_floam = optpose_floam(:,1) - optpose_floam(1,1);
t_floam = t_floam/10^9;

error_fusionx = xEast - interp1(t_post,optpose(:,2),gpskitti_t,'linear');
error_aloamx = xEast - interp1(t_floam,optpose_floam(:,2),gpskitti_t,'linear');
error_fusiony = yNorth - interp1(t_post,optpose(:,3),gpskitti_t,'linear');
error_aloamy = yNorth - interp1(t_floam,optpose_floam(:,3),gpskitti_t,'linear');


meanerror_fusionx = mean(abs(error_fusionx),'omitnan')
meanerror_aloamx = mean(abs(error_aloamx),'omitnan')

meanerror_fusiony = mean(abs(error_fusiony),'omitnan')
meanerror_aloamy = mean(abs(error_aloamy),'omitnan')

stdfx = std(error_fusionx,'omitnan')
stdfy = std(error_fusiony,'omitnan')

stdaloamx = std(error_aloamx,'omitnan')
stdaloamy = std(error_aloamy,'omitnan')



% 
figure
plot(xEast, yNorth, 'DisplayName','Groundtruth')
hold on
plot(optpose(:,2),optpose(:,3),'DisplayName','WithIMU')
hold on
legend 
% % 
figure
plot(gpskitti_t , xEast)
hold on
plot(t_floam,optpose_floam(:,2))






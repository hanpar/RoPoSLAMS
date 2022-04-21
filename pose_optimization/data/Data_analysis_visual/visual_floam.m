clearvars

gtkitti20110930drive0018 = importfile3("../floam_gt_kitti_2011_09_30_drive_notsync.txt", [4, Inf]);
gtkitti_t = importfile4("../floam_gt_kitti_2011_09_30_drive_notsync.txt", [4, Inf]);
optpose = importfile1("../../optimized_poses_with_imu_kitti_0018_notsync.txt", [2, Inf]);
optpose_floam = importfile1("../../optimized_poses_without_imu_kitti_0018_notsync.txt", [6, Inf]);

origin = [gtkitti20110930drive0018(1,1), gtkitti20110930drive0018(1,2)]
len = 2523.6;
% sync_time
gpskitti_t = gtkitti_t - gtkitti_t(1);
gpskitti_t = gpskitti_t/10^9;

t_post = optpose(:,1) - optpose(1,1);
t_post = t_post/10^9;

t_floam = optpose_floam(:,1) - optpose_floam(1,1);
t_floam = t_floam/10^9;

error_fusionx = gtkitti20110930drive0018(:,1) - interp1(t_post,optpose(:,2),gpskitti_t,'linear');
error_floamx = gtkitti20110930drive0018(:,1) - interp1(t_floam,optpose_floam(:,2),gpskitti_t,'linear');
error_fusiony = gtkitti20110930drive0018(:,2) - interp1(t_post,optpose(:,3),gpskitti_t,'linear');
error_floamy = gtkitti20110930drive0018(:,2) - interp1(t_floam,optpose_floam(:,3),gpskitti_t,'linear');


meanerror_fusionx = mean(abs(error_fusionx),'omitnan')/len
meanerror_floamx = mean(abs(error_floamx),'omitnan')/len

meanerror_fusiony = mean(abs(error_fusiony),'omitnan')/len
meanerror_floamy = mean(abs(error_floamy),'omitnan')/len

stdfx = std(error_fusionx,'omitnan')/len
stdfy = std(error_fusiony,'omitnan')/len

stdfloamx = std(error_floamx,'omitnan')/len
stdfloamy = std(error_floamy,'omitnan')/len

% 
figure
plot(gtkitti20110930drive0018(:,1), gtkitti20110930drive0018(:,2), 'DisplayName','Groundtruth')
hold on
plot(optpose(:,2),optpose(:,3),'DisplayName','WithoutIMU')
hold on
plot(optpose_floam(:,2),optpose_floam(:,3),'DisplayName','WithIMU')
legend 
% % 
figure
plot(gpskitti_t , gtkitti20110930drive0018(:,1))
hold on
plot(t_floam,optpose_floam(:,2))






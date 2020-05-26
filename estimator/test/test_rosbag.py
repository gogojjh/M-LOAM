import ros
import rospy
import rosbag

rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) 

readbag = rosbag.Bag('/Monster/dataset/lidar_calibration/mloam_video/RV01_video.bag')
writebag = rosbag.Bag('/Monster/dataset/lidar_calibration/mloam_video/RV01_video_now.bag', 'w')
msg_cnt = 0
for topic, msg, t in readbag.read_messages():
    if topic == '/tf': 
        writebag.write(topic, msg)
        continue
    msg.header.stamp = rospy.get_rostime()
    writebag.write(topic, msg)

    if topic == '/laser_cloud_registered':
        msg_cnt += 1
        if msg_cnt % 10 == 0:
            print('msg cnt: {}'.format(msg_cnt))

    if topic == '/laser_cloud_registered':
        rate.sleep()
    if rospy.is_shutdown():
        break

writebag.close()




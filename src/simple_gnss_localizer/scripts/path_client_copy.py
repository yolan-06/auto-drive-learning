#!/usr/bin/env python
# coding:utf-8


import rospy

from sensor_msgs.msg import NavSatFix

if __name__=="__main__":

    # ref_point = [34.1737883252, 117.22346778, 30.0334587097] # 41.653012 123.422713 0.0 徐工消防
    # ref_point = [34.3996313754, 117.471402751, 42.6655197144] # 41.653012 123.422713 0.0 仿真场景20250119
    # ref_point = [24.6074781725, 118.079508239, 14.7691994777]
    ref_point = [31.027566418088327, 121.44104414639337, 15.90359878540039] 


    # 24.607451757450075,118.07943508161559,14.636591911315918
    # 24.607470925581463,118.07947206562986,14.795793533325195
    rospy.init_node('path_client', anonymous=True)

    #等待需要是要的服务
    rospy.loginfo("Waiting Service : /path_get_service");
    # rospy.wait_for_service("/path_get_service")#阻塞等待，检测到有名为“path_get_service”的service后，才往下执行程序
    rospy.loginfo("Ready Service : /path_get_service");

    #Publisher函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    pub_ref = rospy.Publisher('/ref_map_wgs84', NavSatFix, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = NavSatFix()
        msg.latitude = ref_point[0]
        msg.longitude = ref_point[1]
        msg.altitude = ref_point[2]
        pub_ref.publish(msg)
        rate.sleep()
    rospy.spin()
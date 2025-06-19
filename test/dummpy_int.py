import rospy
``from sensor_msgs.msg import CameraInfo``

def main():
    rospy.init_node("dummy_camera_info_subscriber")
    
    def camera_info_callback(msg):
        # Get camera intrinsic parameters from camera info message
        camera_info = msg.K
        print("Received camera info:")
        print(f"Focal length x: {camera_info[0]}")
        print(f"Focal length y: {camera_info[4]}")
        print(f"Principal point x: {camera_info[2]}")
        print(f"Principal point y: {camera_info[5]}")

    # Subscribe to camera info topic
    camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)
    
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
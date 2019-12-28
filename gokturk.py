import rospy 
from sensor_msgs.msg import NavSatFix,Imu

def gps_logging(gps_data):
  print gps_data.latitude,gps_data.longitude


def imu_logging(imu_data):
  print imu_data.orientation.x



def sub():
  rospy.init_node('ders',anonymous=True) 
  rospy.Subscriber('mavros/global_position/global',NavSatFix,gps_logging)
  rospy.Subscriber('mavros/imu/data',Imu,imu_logging)
  rospy.spin()


if __name__ == '__main__':
  sub()

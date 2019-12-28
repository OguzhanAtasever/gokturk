#!/usr/bin/env python


import rospy
import cv2
import kalmanfilter_class as kalman
import pid_class 
import numpy as np
from std_msgs.msg import String,Float64
from sensor_msgs.msg import Image,NavSatFix,BatteryState
from mavros_msgs.msg import State,VFR_HUD,VehicleInfo,RCIn,OverrideRCIn
from mavros_msgs.srv import SetMode
from cv_bridge import CvBridge, CvBridgeError

track ="NULL"
air_speed = 0.0
ground_speed = 0.0
altitude = 0.0
throttle = 0.0
state = "NULL"
mode ="NULL"
battery_voltage = 0.0
battery_per = 0.0
gps_longitude = 0.0
gps_latitude = 0.0
pilot_conf = 0.0
flag_track = 0
i=1000
flag_set_mode = 0
flag_set_mode_back = 0


pixel_yatay = 640
pixel_dikey = 480

#out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('H','2','6','4'), 10, (pixel_yatay,pixel_dikey))

#fourcc = cv2.VideoWriter_fourcc(*'MPEG')

#video = cv2.VideoWriter('appsrc ! videoconvert ! x264enc tune=zerolatency bitrate=350 speed-preset=superfast ! rtph264pay ! udpsink host=10.15.16.10 port=5000',cv2.CAP_GSTREAMER,0, 30, (640,480), True)


pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size = 10)

msg = OverrideRCIn()



#-------------------------------------
flag = True
global bayrak_kalman
global kor_takip_kalman
bayrak_takip = False
bayrak_kalman = True
kor_takip_kalman = 0
Zt=np.zeros((2,2),dtype=float)
merkez_nokta = []
global kalman_x
global kalman_y
kalman_x = 0
kalman_y = 0
#-------------------------------------





def camera_target_line(image):

	cv2.line(image,((pixel_yatay/2)-5,pixel_dikey/2),((pixel_yatay/2)-20,pixel_dikey/2),(150,255,50),2)
	cv2.line(image,((pixel_yatay/2)+5,pixel_dikey/2),((pixel_yatay/2)+20,pixel_dikey/2),(150,255,50),2)
	cv2.line(image,(pixel_yatay/2,(pixel_dikey/2)-5),(pixel_yatay/2,(pixel_dikey/2)-20),(150,255,50),2)
	cv2.line(image,(pixel_yatay/2,(pixel_dikey/2)+5),(pixel_yatay/2,(pixel_dikey/2)+20),(150,255,50),2)

	#cv2.line(image,(160,48),(480,48),(150,255,50),2)
	#cv2.line(image,(160,432),(480,432),(150,255,50),2)
	#cv2.line(image,(160,432),(160,48),(150,255,50),2)
	#cv2.line(image,(480,432),(480,48),(150,255,50),2)
	cv2.line(image,(pixel_yatay/4,(pixel_dikey - pixel_dikey/10)),(pixel_yatay/4,pixel_dikey/10),(150,255,50),2)
	cv2.line(image,(pixel_yatay -(pixel_yatay/4),(pixel_dikey - pixel_dikey/10)),(pixel_yatay -(pixel_yatay/4),pixel_dikey/10),(150,255,50),2)
	cv2.line(image,(pixel_yatay/4,pixel_dikey/10),(pixel_yatay -(pixel_yatay/4),pixel_dikey/10),(150,255,50),2)	
	cv2.line(image,(pixel_yatay/4,pixel_dikey - pixel_dikey/10),(pixel_yatay - pixel_yatay/4,pixel_dikey - pixel_dikey/10),(150,255,50),2)
		


def camera_target_track(image,track_status):
	
	cv2.putText(image,("TRACKING STATUS:" + track_status),(pixel_yatay-220,20),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),2)	
	

	
def autopilot_state_status(current_state):
	global state
	global mode

	if current_state.armed != False:
		state = "ARMED"
	else:
		state = "DISARMED"
	mode = current_state.mode
	

	


	
def camera_autopilot_state_status(image):
	global state
	global mode

	cv2.putText(image,("FC STAT:" + state),(5,20),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
	cv2.putText(image,("FC MODE:" + mode),(5,155),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
	


def autopilot_hud_status(hud_state):
	
	global air_speed
	global ground_speed	 
	global throttle
	air_speed =round(hud_state.airspeed,2)
	ground_speed = round(hud_state.groundspeed,2)	
	throttle = round((hud_state.throttle)*100,2)
	

def camera_autopilot_hud_status(image):

	global air_speed
	global ground_speed	
	global throttle

	cv2.putText(image,("THRT[%]: "+str(throttle)),(5,35),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
        cv2.putText(image,("AIR SP:   "+ str(air_speed)),(5,50),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
	cv2.putText(image,("GR SP:   "+ str(ground_speed)),(5,65),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
	cv2.putText(image,("ALTI:     "+ str(altitude)),(5,80),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)



def autopilot_rel_alt_status(rel_alt_status):

	global altitude
	altitude = round(rel_alt_status.data,2) 
	


def autopilot_battery_status(battery_status):
	global battery_voltage
	global battery_per
	battery_per = round(battery_status.percentage * 100,4)
	battery_voltage = round(battery_status.voltage,2)
	


def camera_autopilot_battery_status(image):
	global battery_voltage
	global battery_per
	cv2.putText(image,("BATT[V]: "+ str(battery_voltage)),(5,95),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
	cv2.putText(image,("BATT[%]: "+ str(battery_per)),(5,110),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)

def autopilot_gps_status(gps_state):
	global gps_latitude
	global gps_longitude
	gps_latitude = gps_state.latitude 
	gps_longitude = gps_state.longitude		
	

	
	

def camera_autopilot_gps_status(image):
	
	global gps_latitude
	global gps_longitude
	cv2.putText(image,("LAT:  "+ str(gps_latitude)),(5,125),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)
	cv2.putText(image,("LON:  "+ str(gps_longitude)),(5,140),cv2.FONT_HERSHEY_PLAIN,1,(150,255,50),1)


def autopilot_rcin_status(rc):
	global pilot_conf
	global track
	global flag_track
	global flag_set_mode
	global flag_set_mode_back
	pilot_conf =  rc.channels[5]
	if pilot_conf > 1500:
		track = "MANUAL"
		flag_track = 0
		flag_set_mode = 0
		flag_set_mode_back = 0
	else:
		track = "AUTO"
		flag_track = 1
		flag_set_mode_back = 1



			
 
 	


def callback(msg):
	global track
	global flag_set_mode
	global flag_set_mode_back
        bridge = CvBridge()
        image_msg = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
	camera_target_track(image_msg,track)
	camera_target_line(image_msg)
	camera_autopilot_hud_status(image_msg)
	camera_autopilot_state_status(image_msg)
	camera_autopilot_battery_status(image_msg)
	camera_autopilot_gps_status(image_msg)
	#set_mode_back()
	if flag_track == 1:
		if flag_set_mode == 0 and track == "AUTO":
			#setOffboardMode()	
			flag_set_mode = 1
			
		global bayrak_kalman        
		image_copy = np.copy(image_msg)
		
		#override()
		image_copy = np.clip(image_copy,0,255)
		image_copy = np.array(image_copy,np.uint8)
		image_copy = cv2.cvtColor(image_copy, cv2.COLOR_BGR2GRAY)

		ret,thresh = cv2.threshold(image_copy,220,255,cv2.THRESH_BINARY)	
		cnts=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center=None
		if len(cnts)>0:
			c=max(cnts,key=cv2.contourArea)
			((x,y),radius)=cv2.minEnclosingCircle(c)
			M=cv2.moments(c)
			if M["m00"] != 0:
				x = int(M["m10"] / M["m00"])
				y = int(M["m01"] / M["m00"])
			else:
				x , y = 0 , 0
			cv2.circle(image_msg,(int(x),int(y)),int(radius),(0,0,255))
			cv2.putText(image_msg,str(int(x))+"-"+str(int(y)),(int(x),int(y)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),1) 
			global bayrak_takip
			bayrak_takip = True
			Zt[0,0] = x
			Zt[0,1] = y

			if(bayrak_takip == True and bayrak_kalman == True):
				global kalman_filtresi
				global bayrak_kalman
				kalman_filtresi = kalman.KalmanFilter(Zt)
				bayrak_kalman = False
			
			merkez_nokta = kalman_filtresi.KalmanUpdate(Zt)
			kalman_x = merkez_nokta[0,0]
			kalman_y = merkez_nokta[1,0]
			
			
			
		else:
			bayrak_takip = False
		if (bayrak_takip == False and bayrak_kalman == False and kor_takip_kalman < 15):
			global kor_takip_kalman
			global kalman_x
			global kalman_y
			kor_takip_kalman += 1
			Zt[0,0] = kalman_x 
			Zt[0,1] = kalman_y 
			merkez_nokta = kalman_filtresi.KalmanUpdate(Zt) 
			kalman_x = merkez_nokta[0,0]
			kalman_y = merkez_nokta[1,0]
			
			
			if (kor_takip_kalman >= 15):
				bayrak_kalman = True
				kor_takip_kalman = 0
				del kalman_filtresi
				
		
	else:
		pass
	
	
	
	cv2.imshow("frame",image_msg)
	#video.write(image_msg)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("finished.")
		#track = "MANUAL"

	
			
"""def setOffboardMode():
	global flag_set_mode_back
        print("MOD ayarlaniyor")
	rospy.wait_for_service('/mavros/set_mode')
	try:
		modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		modeResponse = modeService(4,'GUIDED')
		#modeResponse = modeService(0,'STABILIZE')
		rospy.loginfo(modeResponse)
	except rospy.ServiceException as e:
		print("Servis hatayla karsilasti: %s" %e)
	flag_set_mode_back = 1


def set_mode_back():
	global flag_set_mode_back 
	global mode 
	global track 
	mode_back = "FBWA"
	if flag_set_mode_back == 0 and track == "MANUAL":
		mode_back = mode
	if flag_set_mode_back  == 1 and track == "MANUAL":
	
		print("MOD ayarlaniyor")
		rospy.wait_for_service('/mavros/set_mode')
		try:
			modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			modeResponse = modeService(mode_back)
			#modeResponse = modeService('STABILIZE')
			rospy.loginfo(modeResponse)
		except rospy.ServiceException as e:
			print("Servis hatayla karsilasti: %s" %e)
		flag_set_mode_back = 1
		
	
def override():
	global i 
	global msg
	i=i+100	
	if i >= 2000
		i = 1000
		
	msg.channels[0] = i
	print msg
	pub.publish(msg)"""
	

def levelfive_listener():
        rospy.init_node('mikoto', anonymous=True)
	
        rospy.Subscriber('line', Image, callback)
	rospy.Subscriber('mavros/state',State,autopilot_state_status)
	rospy.Subscriber('mavros/vfr_hud',VFR_HUD,autopilot_hud_status)
	rospy.Subscriber('mavros/global_position/global',NavSatFix,autopilot_gps_status)
	rospy.Subscriber('mavros/battery',BatteryState,autopilot_battery_status)
	rospy.Subscriber('mavros/global_position/rel_alt',Float64,autopilot_rel_alt_status)
	rospy.Subscriber('mavros/rc/in',RCIn,autopilot_rcin_status)
	rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size = 10)
	#override()
	
	
        rospy.spin()
	
	
	
        #cv2.destoryAllWindows()

if __name__ == '__main__':
        levelfive_listener()
	
	









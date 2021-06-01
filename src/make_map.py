#!/usr/bin/env python3

# -----------------PROGRAM ZA FORMIRANJE MAPE-------------------------
#
import rospy
import pandas as pd
from laser_line_extraction.msg import LineSegment,LineSegmentList

rospy.init_node('make_map', anonymous=False)

Map = pd.DataFrame(columns=['rho','alpha'])
map_saved = False

# -----------------CALLBACK FUNKCIJA ZA TOPIC '/line_segments'-------------------------
# ----------------ujedno i FUNKCIJA ZA FORMIRANJE I CUVANJE MAPE-----------------------
#
def callback(data):

	global Map, map_saved

	extracted_lines = data.line_segments

	if not map_saved:
		for line in extracted_lines:
			rho = line.radius
			alpha = line.angle

			Map = Map.append({'rho':rho,'alpha':alpha}, ignore_index=True)
			
		Map.to_csv('Workspaces/getting_started/src/domaci_4/src/map.csv')
		map_saved = True

		print('Map saved!') 

# -----------------DEO PROGRAMA ZA SUBSCRIBE-OVANJE-----------------------
#
def listener():
	rospy.Subscriber('line_segments', LineSegmentList, callback)
	rospy.spin()
	
# -----------------------GLAVNI PROGRAM-----------------------------------
#		 
if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass
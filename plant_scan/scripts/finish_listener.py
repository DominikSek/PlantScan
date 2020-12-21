#!/usr/bin/env python3

from std_msgs.msg import Bool
import rospy
import subprocess

class Finish_Listener:

	###Callback funckija u kojoj pratimo /finished
	def finish_callback(self,data):
		print("The node has finished its job? {}".format(data.data))
		if(data.data==True):
			path='bin/images'
			print("Executing meshroom at the path: "+path)
			executingString='bin/meshroom_photogrammetry --input '+ path +' --output 3D_Scan'
			success=True
			try:
				success=subprocess.run([executingString],shell=True)
				print(success)
			except Exception as e:
				print(e)
			if success.returncode==0:
				print("Meshroom launch has been successful")
				rospy.signal_shutdown("Success")
			else:
				print("There has been an error launching meshroom")
				rospy.signal_shutdown("Failure")


	###Inicijalizacija Subscribera
	def __init__(self):
		self.listener=rospy.Subscriber("finished",Bool,self.finish_callback)
		self.finished_listen=Bool()

	###Running 
	def run(self):

		while not rospy.is_shutdown():
			rospy.spin()

	###Main function
if __name__=='__main__':
	rospy.init_node('finish_listener')
	try:
		finish=Finish_Listener()
		finish.run()
	except rospy.ROSInterruptException: pass

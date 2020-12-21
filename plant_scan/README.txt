PLANT_SCAN

Cijeli github plant_scan je catkin workspace u kojem pri downloadanju treba prelagoditi package.xml s obzirom na host-a.

Potrebno za pokretanje:
1) Linux sa instaliranim ROS-packageima 
2) CUDA Nvidia Gpu
3) Kamera 

Kako pokrenuti 3D rekonstrukciju (Pod pretpostavkom da launch file nije realiziran):
1) Pullati plant_scan git
2) Editati package.xml za svoj linux(samo podesiti @)
3) Pokrenuti roscore
4) Pozicionirati bash u /plant_scan/scripts folder
5) Iskoristiti komandu: "rosrun plant_scan finish_listener.py"
6) 
    a)Pokrenuti ostale čvorove koji publishaju na /finished topic
    b)Samostalno publishati na topic /finished    --->    "rostopic pub /finished std_msgs/Bool True"
7) Done.

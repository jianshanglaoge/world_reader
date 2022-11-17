valuses=["tomato_soup_can","tuna_fish_can","pitcher_base","sugar_box",
"potted_meat_can","mustard_bottle","master_chef_can",
"gelatin_box","cracker_box","chips_can"]

cameras=["1080pcamera","1080pcamera_0","1080pcamera_1"]
import re
import sys
import math
import numpy as np
import json
sub_dict={}
output=open("output.txt","a")
def printer(text,pose,rotation):

    son_dict={}
    pose_reshape=[0.0,0.0,0.0]
    pose_reshape[0]=pose[0]
    pose_reshape[1]=pose[1]
    pose_reshape[2]=pose[2]    
    son_dict["item_t"]=pose_reshape
    son_dict["item_r"]=rotation.tolist()
    
    sub_dict[text]=son_dict
def findline(text,worldnum):
	pose=[0.0]*6
	rotation= [0.0,0.0,0.0]*3
	reload(sys)
	sys.setdefaultencoding('utf8')
	filename="Table"+str(worldnum)+"_c.world"
	fp = open(filename,"r")
	sample = fp.readlines()
	times = 0
	text=text+"'"
	for line in sample:
	    if (text in line):
    		times=times+1
	    if("collision"in line) and times ==1:
		    times=times+1
	
	    if("pose"in line) and times ==2:
		    out=re.search(r'>(.*) (.*) (.*) (.*) (.*) (.*)</pose>',line,re.M|re.I)
		    times=times+1

		    if out:
		        for i in range(0,5) :
		            pose[i]=pose[i]+float(out.group(i+1))
		    else:
		        print("pose 1 miss")    		

	    if("pose"in line) and times ==4:
		    times=0
		    out=re.search(r'>(.*) (.*) (.*) (.*) (.*) (.*)</pose>',line,re.M|re.I)   
		    if out:
		        for i in range(0,5) :
		            pose[i]=pose[i]+float(out.group(i+1))
		    else:
		        print("pose 2 miss")
	yaw=np.array([[math.cos(pose[5]),-math.sin(pose[5]),0],[math.sin(pose[5]),math.cos(pose[5]),0],[0,0,1]])
	pitch=np.array([[math.cos(pose[4]),0,math.sin(pose[4])],[0,1,0],[-math.sin(pose[4]),0,math.cos(pose[4])]])
	roll=np.array([[1,0,0],[0,math.cos(pose[3]),-math.sin(pose[3])],[0,math.sin(pose[3]),math.cos(pose[3])]])
	rotation=np.dot(yaw,pitch)
	rotation=np.dot(rotation,roll)
	printer(text,pose,rotation)
	fp.close()

def findcamera(text,worldnum):
	pose=[0.0]*6
	reload(sys)
	sys.setdefaultencoding('utf8')
	filename="Table"+str(worldnum)+"_c.world"
	fp = open(filename,"r")
	sample = fp.readlines()
	output=open("camera.txt","a")
	times = 0
	text=text+"'"
	for line in sample:
	    if (text in line):
		times=times+1
	    if("pose"in line) and times ==1:
		times=2
		out=re.search(r'>(.*) (.*) (.*) (.*) (.*) (.*)</pose>',line,re.M|re.I)   
		if out:
		    for i in range(0,5) :
		        pose[i]=float(out.group(i+1))
		else:
		    print("camera miss")
		yaw=np.array([[math.cos(pose[5]),-math.sin(pose[5]),0],[math.sin(pose[5]),math.cos(pose[5]),0],[0,0,1]])
		pitch=np.array([[math.cos(pose[4]),0,math.sin(pose[4])],[0,1,0],[-math.sin(pose[4]),0,math.cos(pose[4])]])
		roll=np.array([[1,0,0],[0,math.cos(pose[3]),-math.sin(pose[3])],[0,math.sin(pose[3]),math.cos(pose[3])]])
		rotation=np.dot(yaw,pitch)
		rotation=np.dot(rotation,roll)
		printer(text,pose,rotation)
	fp.close()


def main():
    dict_world={}
    for worldnum in range(1,10):
        worldname="world"+str(worldnum)
        #output.write("{\"world1\":{")
        for valuse in valuses:
            findline(valuse,worldnum)
        for camera in cameras:
            findcamera(camera,worldnum)
        #output.write("}},\n")   
        
        dict_world[worldname]=sub_dict
    js=json.dumps(dict_world)
    output.write(js)

if __name__ == "__main__":
    main()



from dt_apriltags import Detector
import cv2 as cv
import numpy as np
from params_matrices import red_matrix, blue_matrix, green_matrix, yellow_matrix
import time
from threading import Thread
from tag_locations import return_fixed_loc


# def triangulate(n):
#     print('Hello from triangulate')
#     return n+1,n+2

def triangulate(tag_number):
    present = False
    fixed_tags = []
    car_pose = None
    RED,BLUE,GREEN,YELLOW = range(4)

    # attempts = 0
    # breakflag = False
    # while(attempts < 3):
    #     for i in range(4):
    #         detected, tag_pose, fixed_tags = scanner_loop(i,tag_number)
    #         if detected == True:
    #             breakflag = True
    #             break
    #     if breakflag == True:
    #         break
    #     attempts += 1
    #     time.sleep(1)
    #     if attempts == 3:
    #         print("Tag not found")

    

    #Block of code for the threading of the 4 cameras
    threads = [None] * 4
    red_results = [None] * 4
    blue_results = [None] * 4
    green_results = [None] * 4
    yellow_results = [None] * 4

    red_thread = Thread(target=scanner_thread,args=(RED,tag_number,red_matrix,red_results))
    blue_thread = Thread(target=scanner_thread,args=(BLUE,tag_number,blue_matrix,blue_results))
    green_thread = Thread(target=scanner_thread,args=(GREEN,tag_number,green_matrix,green_results))
    yellow_thread = Thread(target=scanner_thread,args=(YELLOW,tag_number,yellow_matrix,yellow_results))
    threads[0] = red_thread
    threads[1] = blue_thread
    threads[2] = green_thread
    threads[3] = yellow_thread

    for t in range(len(threads)):
        threads[t].start()
    for t in range(len(threads)):
        threads[t].join()

    if red_results[0] == True:
        car_pose = red_results[1]
        fixed_tags = red_results[2]
    elif blue_results[0] == True:
        car_pose = blue_results[1]
        fixed_tags = blue_results[2]
    elif green_results[0] == True:
        car_pose = green_results[1]
        fixed_tags = green_results[2]
    elif yellow_results[0] == True:
        car_pose = yellow_results[1]
        fixed_tags = yellow_results[2]
    else:
        return None, None
    
    # Here will be a function that will take the car pose, and the april tag id numbers to return the x and y locations of the bot relative to the location 
    x,y = final_locator(car_pose,fixed_tags)
    return x,y


def scanner_loop(cam_num,tag_num):
    cap = cv.VideoCapture(cam_num)
    ret, frame = cap.read()
    fixed_tags = []
    detector = Detector(families="tagStandard41h12",nthreads=1,quad_decimate=1.0,quad_sigma=0.0,
                        refine_edges=1, decode_sharpening=0.25,searchpath=['apriltags'],debug=0)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=None, tag_size=0.066)


    for t in tags:
        if int(tag_num) == int(t.tag_id):
            present = True
            x,y = t.center
            pose = t.pose_t
        elif t.tag_id > 100:
            fixed_tags.append(t.tag_id)

    return present, pose, fixed_tags
           
def scanner_thread(cam_num,tag_num,matrix,results):
    cap = cv.VideoCapture(cam_num)
    ret, frame = cap.read()
    fixed_tags = []
    detector = Detector(families="tagStandard41h12",nthreads=1,quad_decimate=1.0,quad_sigma=0.0,
                        refine_edges=1, decode_sharpening=0.25,searchpath=['apriltags'],debug=0)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    tags = detector.detect(gray, estimate_tag_pose=True, camera_params=matrix, tag_size=0.066)

    x,y,pose = None

    for t in tags:
        if int(tag_num) == int(t.tag_id):
            present = True
            x,y = t.center
            pose = t.pose_t
        elif t.tag_id > 100:
            fixed_tags.append(t.tag_id)
    
    results = [present,pose,fixed_tags]

def final_locator(pose,tags):
    car_x = pose[0]
    car_y = pose[1]
    tag_pose = tags[0].pose_t
    tag_x = tag_pose[0]
    tag_y = tag_pose[1]

    delta_x = car_x - tag_x
    delta_y = car_y - tag_y
    local_tag_x, local_tag_y = return_fixed_loc(tags[0].tag_id)
    final_x = local_tag_x + delta_x
    final_y = local_tag_y + delta_y
    return final_x, final_y

           
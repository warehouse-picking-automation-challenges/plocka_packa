#!/usr/bin/env python

##********************************************************************
 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2015, UT Arlington
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #   # Redistributions of source code must retain the above copyright
 #     notice, this list of conditions and the following disclaimer.
 #   # Redistributions in binary form must reproduce the above
 #     copyright notice, this list of conditions and the following
 #     disclaimer in the documentation and/or other materials provided
 #     with the distribution.
 #   # Neither the name of UT Arlington nor the names of its
 #     contributors may be used to endorse or promote products derived
 #     from this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 #  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 #  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 #  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 #  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 #  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 #  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 #  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 #  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 #  POSSIBILITY OF SUCH DAMAGE.
 #********************************************************************/

import json
import copy
import os
from pprint import pprint

import rospy
import rospkg
from apc_msgs.msg import *
from apc_msgs.srv import *

# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

CONST_BIN_NAMES = ['bin_A',
                   'bin_B',
                   'bin_C',
                   'bin_D',
                   'bin_E',
                   'bin_F',
                   'bin_G',
                   'bin_H',
                   'bin_I',
                   'bin_J',
                   'bin_K',
                   'bin_L']
                   
CONST_ITEM_NAMES = ["oreo_mega_stuf",
                    "champion_copper_plus_spark_plug",
                    "expo_dry_erase_board_eraser",
                    "kong_duck_dog_toy",
                    "genuine_joe_plastic_stir_sticks",
                    "munchkin_white_hot_duck_bath_toy",
                    "crayola_64_ct",
                    "mommys_helper_outlet_plugs",
                    "sharpie_accent_tank_style_highlighters",
                    "kong_air_dog_squeakair_tennis_ball",
                    "stanley_66_052",
                    "safety_works_safety_glasses",
                    "dr_browns_bottle_brush",
                    "laugh_out_loud_joke_book",
                    "cheezit_big_original",
                    "paper_mate_12_count_mirado_black_warrior",
                    "feline_greenies_dental_treats",
                    "elmers_washable_no_run_school_glue",
                    "mead_index_cards",
                    "rolodex_jumbo_pencil_cup",
                    "first_years_take_and_toss_straw_cup",
                    "highland_6539_self_stick_notes",
                    "mark_twain_huckleberry_finn",
                    "kyjen_squeakin_eggs_plush_puppies",
                    "kong_sitting_frog_dog_toy"]

CONST_ITEM_IDS   = ["",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    ""]

ready = False
jsonLoaded = False
filename = 'apc.json'

num_of_items = 0
num_of_items_left = 0
current_bin = "bin_A"
current_bin_index = 0
total_items_in_work_order = 0

# Bin contents structure
class bin_contents:
  bin_num  = 0
  bin_name = ''
  contents = None

class shelf_contents:
  contents = None

shelfContents = shelf_contents()
shelfContents.contents = []

get_work_order_msg = []

#~ def handle_work_order_request(req):
	#~ 
	#~ #decalre globals
	#~ global num_of_items_left
	#~ global num_of_items
	#~ global current_bin
	#~ global current_bin_index
	#~ global work_order
	#~ 
	#~ if 0 == num_of_items_left:
		#~ return GetItemFromWorkOrderResponse(0,'done',0,0)	
#~ 
	#~ current_bin = work_order_msg.bins[num_of_items]
	#~ current_item = work_order_msg.items[num_of_items]
	#~ num_of_items += 1
	#~ num_of_items_left -= 1
	#~ 
	#~ return GetItemFromWorkOrderResponse(current_bin,current_item,total_items_in_work_order,num_of_items_left)	
	
#~ def handle_work_order_item_request(req):
	#~ 
	#~ #decalre globals
	#~ global num_of_items_left
	#~ global num_of_items
	#~ global current_bin
	#~ global current_bin_index
	#~ global work_order
	#~ 
	#~ print "Item Number Request: " , req.item_num
#~ 
	#~ if 0 == num_of_items_left:
		#~ return GetItemFromWorkOrderResponse(0,'done',0,0)	
#~ 
	#~ if (req.item_num) < 0:
		#~ return GetItemFromWorkOrderResponse(-1,'incorrect',-1,-1)	
#~ 
	#~ current_bin = work_order_msg.bins[req.item_num]
	#~ current_item = work_order_msg.items[req.item_num]
	#~ num_of_items += 1
	#~ num_of_items_left -= 1
	#~ 
	#~ print current_bin, " ", current_item, " ", total_items_in_work_order, " ", num_of_items_left
	#~ 
	#~ return GetItemFromWorkOrderResponse(current_bin,current_item,total_items_in_work_order,num_of_items_left)	

def write_ork_configs():

    # apc_object_detection/config/ork/bin_i_detection.ros.ork
    rospack = rospkg.RosPack()
   
    path = rospack.get_path('apc_object_detection')+'/config/ork/'
    if not os.path.exists(path):
		os.makedirs(path)
		
    orkPreConfig = """source1:
  type: RosKinect
  module: 'object_recognition_ros.io'

  parameters:
    rgb_frame_id: 'camera_rgb_optical_frame'
    rgb_camera_info: '/camera/rgb/camera_info'
    rgb_image_topic: '/camera/rgb/image_raw'
    depth_image_topic: '/camera/depth/image_raw'
    depth_camera_info: '/camera/depth/camera_info'

sink1:
  type: TablePublisher
  module: 'object_recognition_tabletop'
  inputs: [source1]

sink2:
  type: Publisher
  module: 'object_recognition_ros.io'
  inputs: [source1]


pipeline1:
  type: TabletopTableDetector
  module: 'object_recognition_tabletop'
  inputs: [source1]
  outputs: [sink1]
  parameters:
    table_detector:
        min_table_size: 4000
        plane_threshold: 0.01
    #clusterer:
    #    table_z_filter_max: 0.35
    #    table_z_filter_min: 0.025

pipeline2:
  type: TabletopObjectDetector
  module: 'object_recognition_tabletop'
  inputs: [source1, pipeline1]
  outputs: [sink2]
  parameters:"""
    
    
    orkPostConfig = """    tabletop_object_ids: 'REDUCED_MODEL_SET'
    db:
      type: CouchDB
      root: http://localhost:5984
      collection: object_recognition"""

    for bin_cont in shelfContents.contents:
    	
    	objectIDsString = "\n    object_ids: [ "
    	
    	for bin_cont_item in bin_cont.contents:
    		objectIDsString = objectIDsString + "'" + CONST_ITEM_IDS[CONST_ITEM_NAMES.index(bin_cont_item)] +  "',"
    		
    	objectIDsString = objectIDsString + " ]\n"

        file_data = orkPreConfig + objectIDsString + orkPostConfig

#        ['dab...',...]


        with open(path + 'bin_' + str(bin_cont.bin_num) + '_detection.ros.ork', 'w') as f:
            f.write(file_data)
        f.closed

def bin_contents_CB(req):
    # We are starting from bin 1

    bin_contents_msg = BinContents()

    for bin_cont in shelfContents.contents:

        if CONST_BIN_NAMES[req.bin - 1] == bin_cont.bin_name:

            print bin_cont.bin_name
            bin_contents_msg.bin = bin_cont.bin_num
            bin_contents_msg.bin_name = bin_cont.bin_name

            for item in bin_cont.contents:
                print item
                bin_contents_msg.items.append(item)

    print "Returning bin contents of bin: ", bin_contents_msg.bin_name

    return GetBinContentsResponse(bin_contents_msg)

def work_order_CB(req):
    print "Returning work order: ", req.order_num
    return GetWorkOrderResponse(get_work_order_msg)

def status_CB(req):
    return GetBoolResponse(ready)

def read_json(fname):
	
    global num_of_items_left
    global total_items_in_work_order
	
    rospack = rospkg.RosPack()
    path = rospack.get_path('apc_json')+'/data/'
    json_data = open(path+fname)
    data = json.load(json_data)

    # Check for errors
    # Populate bin_contents and work_order

    total_items = 0
    total_items_in_work_order = 0
    shelfContents.contents = []
	
	# Shelf contents
    for bin_name in data['bin_contents']:

        print bin_name,":"
        binCont = bin_contents()
        binCont.bin_name = bin_name
        binCont.bin_num = CONST_BIN_NAMES.index(bin_name)+1
        binCont.contents = []

        n_items = len(data['bin_contents'][bin_name])
        total_items = total_items + n_items

        for item in data['bin_contents'][bin_name]:
            binCont.contents.append(item)
            print " ",item

        shelfContents.contents.append(binCont)

	# Work order
    for line_item in data['work_order']:
        tmp = WorkOrder()
        tmp.name = line_item["item"]
        tmp.bin = CONST_BIN_NAMES.index(line_item['bin'])+1
        print "Adding",tmp.name,"from bin ", tmp.bin
        get_work_order_msg.append(tmp)
        total_items_in_work_order += 1

	
    num_of_items_left = total_items_in_work_order
    print "Number of items in shelf:  ",total_items
    print "Number of Work Order items:", total_items_in_work_order
    jsonLoaded = True

def print_menu():
    print (20 * '-')
    print "1. Enter file name"
    print "2. Read JSON file"
    print "3. Toggle ready"
    print " "
    print "filename: ",filename
    print "json read: %r"%jsonLoaded
    print "ready:     %r"%ready
    print (20 * '-')

if __name__ == '__main__':
    rospy.init_node('apc_parser_server')

    s1 = rospy.Service('/apc/json/bin_contents', GetBinContents, bin_contents_CB)
    s2 = rospy.Service('/apc/json/work_order', GetWorkOrder, work_order_CB)
    s3 = rospy.Service('/apc/json/status', GetBool, status_CB)
    #~ s4 = rospy.Service('/apc/work_order_request',GetItemFromWorkOrder,handle_work_order_item_request)

    # Load object IDs
    objectIDs = rospy.get_param('apc/objects')

    for i in range(0, len(CONST_ITEM_NAMES)):
        CONST_ITEM_IDS[i] = objectIDs[CONST_ITEM_NAMES[i]]
        print CONST_ITEM_IDS[i]

    ready=False
    jsonLoaded=False
    
    while not (ready and jsonLoaded):
    
        print_menu()
        
        ## Wait for valid input
        is_valid=0
        while not is_valid:
        	
            choice = int ( raw_input('Enter your choice [1-3] : ') )
            if choice in [1, 2, 3, 4]:
            	is_valid = 1
            else:
				print ("Invalid number. Try again...")

        ## Take action
        if choice == 1:
            filename = raw_input('Enter filename: .../apc_json/data/')
        elif choice == 2:
            read_json(filename)
            jsonLoaded=True
            write_ork_configs()
        elif choice == 3:
            ready = not ready
        elif choice == 4:
           quit
            
	if ready and jsonLoaded:
		print ("Starting ros::spin()")
		rospy.spin()		

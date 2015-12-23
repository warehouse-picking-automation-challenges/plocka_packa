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
from pprint import pprint

import rospy
import rospkg
from apc_msgs.msg import *
from apc_msgs.srv import GetItemFromWorkOrderResponse, GetItemFromWorkOrder

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
                    
CONST_NUM_SHELVES = 12
num_of_items = 0
num_of_items_left = 0
current_bin = "bin_A"
current_bin_index = 0
                                      
filename = 'apc.json'

work_order = {bin:[] for bin in CONST_BIN_NAMES}

def handle_work_order_request(req):
	
	#decalre globals
	global num_of_items_left
	global num_of_items
	global current_bin
	global current_bin_index
	global work_order
	
	if num_of_items_left == 0:
		return GetItemFromWorkOrderResponse(0,'done',0,0)	

	if work_order[current_bin] == []:
		current_bin_index += 1


	current_bin = CONST_BIN_NAMES[current_bin_index]

	current_item = work_order[current_bin].pop()
	num_of_items += 1
	num_of_items_left -= 1
	
	#print current_bin
	#print current_item
	#print num_of_items
	#print num_of_items_left
	
	return GetItemFromWorkOrderResponse(current_bin_index,current_item,num_of_items,num_of_items_left)	
	
def read_json():
	rospack = rospkg.RosPack()
	path = rospack.get_path('apc_json')+'/data/'
	json_data = open(path+'apc.json')
	data = json.load(json_data)

	#decalre globals
	global num_of_items_left

    # Check for errors
    # Populate bin_contents and work_order

	total_items = 0

	for item in data['work_order']:
		bin_name = item['bin']
		work_order[bin_name].append(item['item'])
		num_of_items_left += 1

	current_bin = 1
	jsonLoaded = True
  
def status_CB(req):
    return GetBoolResponse(ready)
  
if __name__ == '__main__':
	rospy.init_node('work_order_server')
	s = rospy.Service('/apc/work_order_request',GetItemFromWorkOrder,handle_work_order_request)

	read_json()
	print work_order
	print num_of_items_left

	rospy.spin()

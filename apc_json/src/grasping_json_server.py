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

from apc_msgs.srv import GetGraspStrategy, GetGraspStrategyResponse

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


strategies = {object:[] for object in CONST_ITEM_NAMES}

##
# Example ROS Service call from command line
#	rosservice call /strategy_req "item: 'laugh_out_loud_joke_book'
#	bin: 2
#	orientation: 'front'" 

def handle_grasp_strategy(req):
	print "Requesting item '%s' for bin %d and orientation '%s'"%(req.item,req.bin,req.orientation)
	data=strategies[req.item]["strategies"]
	N=len(data)
	for binIdx in range(0,N):
		if( (req.bin in data[binIdx]["bins"]) or (0 in data[binIdx]["bins"]) ):
			print "Found the bin!"
			o = data[binIdx]["orientation"]
			#Loop through orientations and see if the requested one is included
			for orienIdx in range(0,len(o)):
				if( (req.orientation in o[orienIdx]) or ("all" in o[orienIdx]) ):
					print "Found the orientation!"
					print "The correct strategy is: ", data[binIdx]["grasp"], ",", data[binIdx]["tool"]
					return GetGraspStrategyResponse(data[binIdx]["grasp"],data[binIdx]["tool"])
	

def grasp_strategy_server():
	s = rospy.Service('/apc/grasp_strategy',GetGraspStrategy,handle_grasp_strategy)
	print "Ready for request"
	rospy.spin()

if __name__ == '__main__':
	rospy.init_node('grasping_strategy_node')
	
	### Load JSON file ###
	
	rospack = rospkg.RosPack()
	path=rospack.get_path('apc_json')+'/config/grasping/'
	
	for item in CONST_ITEM_NAMES:
		filename = path + item + '.json'
		with open(filename) as data_file:    
			data = json.load(data_file)
			strategies[item] = data
			#print item + str(len(strategies[item]))
	
	#pprint(strategies)
	#pprint(strategies["crayola_64_ct"])
	

	grasp_strategy_server()
	
#pprint(data)
#print len(data["champion_copper_plus_spark_plug"])


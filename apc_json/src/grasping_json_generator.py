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
import rospy
import rospkg
import json
import os

# define our bin and item names to use
CONST_BIN_NAMES 	= 	['bin_A',
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

CONST_ITEM_NAMES 	=	["oreo_mega_stuf",
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

obj_orientation 	= 	["front","back","up","down","side"]

grasp_strategy  	= 	["front","lift","angledLift","angledLiftSide"] # gripper roll and pitch, approach vector, etc.
grasp_tool          = 	["electric","vacuum"]

bin_numbers     	= 	[1,2,3,4,5,6,7,8,9,10,11,12]

grasp_strategy_map  =  [4, #oreo_mega_stuf
                        3, #champion_copper_plus_spark_plug
                        3, #expo_dry_erase_board_eraser
						3, #kong_duck_dog_toy
						1, #genuine_joe_plastic_stir_sticks
						3, #munchkin_white_hot_duck_bath_toy
						1, #crayola_64_ct
						2, #mommys_helper_outlet_plugs
						2, #sharpie_accent_tank_style_highlighters
						3, #kong_air_dog_squeakair_tennis_ball
                        2, #stanley_66_052
						3, #safety_works_safety_glasses
						2, #dr_browns_bottle_brush
						2, #laugh_out_loud_joke_book
						1, #cheezit_big_original
						2, #paper_mate_12_count_mirado_black_warrior
						1, #feline_greenies_dental_treats
						1, #elmers_washable_no_run_school_glue
						2, #mead_index_cards
						1, #rolodex_jumbo_pencil_cup
						1, #first_years_take_and_toss_straw_cup
						2, #highland_6539_self_stick_notes
						2, #mark_twain_huckleberry_finn
						3, #kyjen_squeakin_eggs_plush_puppies
						3, #kong_sitting_frog_dog_toy
						]

grasp_tool_map  =       [1, #oreo_mega_stuf
                         1, #champion_copper_plus_spark_plug
                         1, #expo_dry_erase_board_eraser
                         1, #kong_duck_dog_toy
                         2, #genuine_joe_plastic_stir_sticks
                         1, #munchkin_white_hot_duck_bath_toy
                         1, #crayola_64_ct
                         2, #mommys_helper_outlet_plugs
                         2, #sharpie_accent_tank_style_highlighters
                         2, #kong_air_dog_squeakair_tennis_ball
                         2, #stanley_66_052
                         1, #safety_works_safety_glasses
                         2, #dr_browns_bottle_brush
                         2, #laugh_out_loud_joke_book
                         1, #cheezit_big_original
                         2, #paper_mate_12_count_mirado_black_warrior
                         2, #feline_greenies_dental_treats
                         1, #elmers_washable_no_run_school_glue
                         2, #mead_index_cards
                         1, #rolodex_jumbo_pencil_cup
                         1, #first_years_take_and_toss_straw_cup
                         2, #highland_6539_self_stick_notes
                         2, #mark_twain_huckleberry_finn
                         1, #kyjen_squeakin_eggs_plush_puppies
                         1, #kong_sitting_frog_dog_toy
                         ]



if __name__ == '__main__':
    rospack = rospkg.RosPack()
    path=rospack.get_path('apc_json')+'/config/grasping/'
	
    if not os.path.exists(path):
    	os.makedirs(path)
    #path = '/Users/sven/Programming/apc-dev2/apc_json/config/grasping/'

    for item in CONST_ITEM_NAMES:
		#print ('Creating file for %s') % item
		filename = path + item + ".json"
		print filename
	
		#------------------------------------------------
		#Dump into file
		#data = [{bin_name:bin_example} for bin_name in CONST_BIN_NAMES]
		#data_2 = {item:data}
		#with open(filename,'w') as outfile:
		#	json.dump(data_2,outfile,indent=1)
		#ex1 = {"bins": [0], "orientation": ["all"],"grasp":"scoop","tool":"electric"}
		#ex2 = {"bins": [0], "orientation": ["front"],"grasp":"standard","tool":"electric"}
		#ex3 = {"bins": [0], "orientation": ["other"],"grasp":"lift","tool":"vacuum"}

		grasp = grasp_strategy[grasp_strategy_map[CONST_ITEM_NAMES.index(item)]-1];
        
		tool = grasp_tool[grasp_tool_map[CONST_ITEM_NAMES.index(item)]-1];
		
		ex1 = {"bins": [0], "orientation": ["all"],"grasp":grasp,"tool":tool}

		s = [ex1]
		data_2 = {"name":item,"strategies": s}
		with open(filename,'w') as outfile:
			json.dump(data_2,outfile,sort_keys=True,indent=1)
		#------------------------------------------------



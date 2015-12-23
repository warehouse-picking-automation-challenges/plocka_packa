 #!/bin/bash         

# Create yaml file while running
function saveToFile {
  destdir="./object_keys.yaml"
  name=" ";
  echo -n $1 >> $destdir
  echo -n ": '" >> $destdir
  echo -n $2 >> $destdir
  #echo -e "'\n" >> $destdir
  echo "'" >> $destdir
}

echo "Please ensure that the database is deleted before uploading the new data"
#echo "Enter the Location of the folders (Eg.: /home/user/work_space/src/ork_tutorials/data): "
#read loc

loc="/home/samkd/Dropbox/APC/STL_files/Processed Files"


# TODO check which items are missing (23/25 are being loaded)

var=$(rosrun object_recognition_core object_add.py -n "cheezit_big_original" -d "cheezit_big_original" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/cheezit_big_original/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="cheezit";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "crayola_64_ct" -d "crayola_64_ct" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/crayola_64_ct/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="crayola";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "elmers_washable_no_run_school_glue" -d "elmers_washable_no_run_school_glue" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/elmers_washable_no_run_school_glue/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="elmers_glue";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "expo_dry_erase_board_eraser" -d "expo_dry_erase_board_eraser" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/expo_dry_erase_board_eraser/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="expo board eraser";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "champion_copper_plus_spark_plug" -d "champion_copper_plus_spark_plug" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/champion_copper_plus_spark_plug/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="spark_plug";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "feline_greenies_dental_treats" -d "feline_greenies_dental_treats" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/feline_greenies_dental_treats/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Feline Dental Treats";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "first_years_take_and_toss_straw_cups" -d "first_years_take_and_toss_straw_cups" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/first_years_take_and_toss_straw_cups/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Toss Straw Cups";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "genuine_joe_plastic_stir_sticks" -d "genuine_joe_plastic_stir_sticks" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/genuine_joe_plastic_stir_sticks/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Stir Sticks";
saveToFile $num $name

var=$(rosrun object_recognition_core object_add.py -n "highland_6539_self_stick_notes" -d "highland_6539_self_stick_notes" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/highland_6539_self_stick_notes/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Sticky Notes";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "kong_air_dog_squeakair_tennis_ball" -d "kong_air_dog_squeakair_tennis_ball" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/kong_air_dog_squeakair_tennis_ball/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Tennis Ball";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "kong_duck_dog_toy" -d "kong_duck_dog_toy" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/kong_duck_dog_toy/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Duck Dog Toy";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "kong_sitting_frog_dog_toy" -d "kong_sitting_frog_dog_toy" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/kong_sitting_frog_dog_toy/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Frog Dog Toy";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "kygen_squeakin_eggs_plush_puppies" -d "kygen_squeakin_eggs_plush_puppies" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/kygen_squeakin_eggs_plush_puppies/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Squeaking puppies";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "mark_twain_huckleberry_finn" -d "mark_twain_huckleberry_finn" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/mark_twain_huckleberry_finn/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Huckleberry Finn";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "mead_index_cards" -d "mead_index_cards" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/mead_index_cards/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Index Cards";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "mommys_helper_outlet_plugs" -d "mommys_helper_outlet_plugs" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/mommys_helper_outlet_plugs/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Outlet Plugs";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "munchkin_white_hot_duck_bath_toy" -d "munchkin_white_hot_duck_bath_toy" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/munchkin_white_hot_duck_bath_toy/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Duck Bath Toy";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "oreo_mega_stuf" -d "oreo_mega_stuf" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/oreo_mega_stuf/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Oreo Mega";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "paper_mate_12_count_mirado_black_warrior" -d "paper_mate_12_count_mirado_black_warrior" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/paper_mate_12_count_mirado_black_warrior/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Mirado Black Warrior";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "rollodex_mesh_collection_jumbo_pencil_cup" -d "rollodex_mesh_collection_jumbo_pencil_cup" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/rollodex_mesh_collection_jumbo_pencil_cup/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Pencil Cup";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "safety_works_safety_glasses" -d "safety_works_safety_glasses" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/safety_works_safety_glasses/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Safety Glasses";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "sharpie_accent_tank_style_highlighters" -d "sharpie_accent_tank_style_highlighters" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/sharpie_accent_tank_style_highlighters/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Sharpie Highlighter";
saveToFile $num $name


var=$(rosrun object_recognition_core object_add.py -n "stanley_66_052" -d "stanley_66_052" --commit)
num=${var/*:/''}
rosrun object_recognition_core mesh_add.py $num $loc/stanley_66_052/textured_meshes/optimized_poisson_texture_mapped_mesh.mtl --commit
name="Stanley";
saveToFile $num $name


echo "Done!!"


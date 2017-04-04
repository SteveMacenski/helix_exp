



# Collection Pipeline:

Get cameras placed well and put tag on EE R1 (Rosey). 110 cm from ground on tape.

> roslaunch realsenseCamera_aprilTags realsense_apriltags.launch 

> rosrun image_view image_view image:=/usb_cam/image_raw

Then,

> roslaunch helix_exp helix_calib_multi.launch


This will run calibration with tag on robot EE. Input joint angles when asked and it will generate calibration file. Place into the experimental_data_and_errors directory, will appear in /devel/lib/helix_exp directory. Uses april tags.

Them, remove calibration piece. 

> roslaunch helix_exp helix_exp.launch


Will run the paths in the "robot_paths.txt" file in experimental_data_and_errors. Once in home pose waiting to start, connect the rod.

After, take the result_images, errors.txt, robot_paths.txt, and calibration file and zip them onto the external drive. in errors 3=good


# Collection Pipeline NEW:

Get cameras placed well and put tag on EE R1 (Rosey). 110 cm from ground on tape.

> roslaunch realsenseCamera_aprilTags realsense_apriltags.launch 

> rosrun image_view image_view image:=/usb_cam/image_raw

Then,

> roslaunch helix_exp helix_calib_multi.launch

Them, remove calibration piece. 

> roslaunch helix_exp helix_exp.launch

Will run the paths in the "robot_paths.txt" file in experimental_data_and_errors. Once in home pose waiting to start, connect the rod.

After, take the result_images, errors.txt, robot_paths.txt, and calibration file and zip them onto the external drive. in errors 3=good


# Pose Process Pipeline:

copy the result_images folder to home directory
place Makefile, im_seg.cpp, and post_process.sh in folder

> make 

Then run this shell command:

> for file in *' '*; do [ -f "$file" ] && mv "$file" "`echo $file|tr -d '[:space:]'`"; done

To remove spaces in filenames. Make a file "rodPoints.txt". 

Make the im_seg file to 'segm'

> ./post_process.sh


runs the post_process for all .bmp files in directory, stores the ROIandPnts images for troubleshooting, then rodPoints.txt file for the points. 

Zip folder and place this and the raw data into a descriptive folder name on the external drive



# Andy's work

He'll read in the points after a quick quality control check in matlab and run scripts.

        
TODO analysis
        - clutering validation
        



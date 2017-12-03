This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Team and Members
* Team name: Rocky
* Team members
 |               | Name         | Account Email                |
 | ------------- | ------------ | ---------------------------- |
 | Team Leader   | Chiheng Liao | liaochiheng@vip.163.com      |
 | Team Member 1 | Ajeet Kumar  | ajeet.ajeetkumar@gmail.com   |
 | Team Member 2 | Arnon Degani | arniede@gmail.com            |
 | Team Member 3 | Wenke Xu     | hanmoyuan000@126.com         |
 | Team Member 4 | Avnit Mackin | avnit.mackin@mail.huji.ac.il |

### Walkthrough

It's a great project and also a great challenge. Here is an introduction of all the code.

1. waypointer_updater node
	* For acceleration, simply set all the linear velocity to max velociy read from waypointer_loader parameter `velocity`.
	* For deceleration, decrement the linear velocity into 0 with deceleration = 0.5(waypointer_updater.py line 160).

2. twister_controller node
	* Since the twiddle parameters for PID is kind of impossible, so we simply use a constant throttle and brake. That works fine in simulator.
	* For throttle, type is PERCENT. We use 1.0 for simulator and 0.025 for Carla retrieved from a rosbag(udacity_succesful_light_detection.bag).
	* For brake, type is TORQUE. It's calculated by
	```
	self.max_brake_value = ( self.vehicle_mass + self.fuel_capacity * GAS_DENSITY ) \
    				* decel * self.wheel_radius
    ```
    * For steer, we simply used the yaw_controller, nothing changed.

3. tl_detector node
	* We have tried ssd-mobile and ssd-inception models with different datasets(bosch and datasets collected from simulator and bag file from our classmates codeKnight).
	* Bosch dataset is a huge dataset, but didn't solve our problem.
	* We simply chose the dataset directly collected from simulator and bagfiles.
	* For the two models, we found that ssd-mobile made much mistakes in simulator detections, however ssd-inception works better. So we chose ssd-inception.
	* We use Tnsorflow Object Detection API to train the model. After 20k steps, we got loss converged to around 1-2.
	* My computer is ubtuntu 16.04 with GTX 1070, and I tested the speed of the model, turned out it's kind of slow in my local computer, need ~300ms to make detections on one image. Then I tested the same model on AWS(p2.xlarge), detection time down to ~60ms. That's an acceptable speed.
	* We also provide a test node `tl_test.py` to test the classifier in sim and site. It will publish a image with detections into a topic `/detection`. We could check the `/detection` in `rqt_image_view`.
	* Finally, we trained two seperate models each for sim and site, both based on ssd-inception model.

### Further experiments

* For brake, I changed to PERCENT type to test in sim, it turned out not working no matter what value i set.
* ssd-mobilenet is much worse than I thought, however I don't think it's really that worse. I guess the poor dataset would be the reason, so I will work on this model on other datasets to see how it work.
* I have tried set throttle = 1.0 always, but the speed of vehicle are only up to around 7-8mph(sometimes up to 12mph). That's so weird, since max velocity 40km/h = (40 * 1000 / 3600) * 2.24 = ~24mph.
* DBW is too easy to implement for sim, and that's not supposed to be so easy in real world. The CarND team have simplified this node. 
	
### Acknowledgement

Thanks for all our team members, we made a great achievement.
Thanks a lot to our classmate Vatsal Srivastava, [Self Driving Vehicles: Traffic Light Detection and Classification with TensorFlow Object Detection API](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62) give me a lot of help.
Also thanks to many classmates from slack and udacity-team, you gave me a lot of help.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

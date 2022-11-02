install arduino IDE

## search arduino ros
install Rosserial Arduino Library from library manager

cd $HOME/Arduino/libraries
rm -rf ros_lib
roscore (terminal 1)
rosrun rosserial_arduino make_libraries.py . (terminal 2)

## give authority 
sudo chmod a+rw /dev/ttyACM0

select board and port

compile and upload

roscore

rosrun rosserial_python serial_node.py /dev/ttyACM0

rostopic list


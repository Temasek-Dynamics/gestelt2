#!/bin/bash

if [ ! -e /home/visbot ]; then
        exit 0
fi

sudo chmod a+rw /dev/ttyACM*
sudo chmod a+rw /dev/ttyS*

HOMEDIR=/home/visbot/bin
LOGDIR=/home/visbot/log
if [[ ! -d "$LOGDIR" ]]; then
  mkdir $LOGDIR
fi

sudo killall -9 roscore
sleep 1
sudo killall -9 rosmaster
sleep 1

for((i=0;i<2;i++))
do
  sudo killall roslaunch
  if [ $? = 1 ]
  then
          break;
  fi
  sleep 1
done

index=0
obindex=0
if [[ -f "$LOGDIR/.logindex" ]]; then
  index=`cat $LOGDIR/.logindex`
fi

if [[ -d "$LOGDIR/log$index" ]]; then
  index=`expr $index + 1`
  obindex=`expr $index - 5`
  echo $index > $LOGDIR/.logindex
fi
LOGCUR=$LOGDIR/log$index
#remove obsoleted log
if [[ -d "$LOGDIR/log$obindex" ]]; then
  echo rm -Rf $LOGDIR/log$obindex
  rm -Rf $LOGDIR/log$obindex
fi
mkdir $LOGCUR
if [[ -d "$LOGDIR/latest" ]]; then
  rm $LOGDIR/latest
fi
ln -s $LOGCUR $LOGDIR/latest

#echo "stereocam, imu, vins log lacated at $LOGCUR" 
#sleep 5

export MASTER_IP=10.42.0.37
export SELF_IP=10.42.0.37

export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP

source /home/visbot/ros_ws/devel/setup.bash
source /home/visbot/ros_ws/install/setup.bash
source /home/visbot/ros1_ws/devel/setup.bash

echo starting roscore
roscore & 
sleep 10

echo roslaunch ros_zmq ros_zmq.launch
roslaunch ros_zmq ros_zmq.launch > $LOGCUR/ros_zmq.log 2>&1  &
sleep 1

echo roslaunch stereo_cam stereo_cam.launch 
roslaunch stereo_cam stereo_cam_itof.launch > $LOGCUR/stereo_cam.log 2>&1  &
sleep 1

#echo roslaunch vins stereo.launch
#roslaunch vins stereo.launch > $LOGCUR/vins.log 2>&1 &
#sleep 1

echo roslaunch visbot_itof visbot_itof.launch
roslaunch visbot_itof visbot_itof.launch > $LOGCUR/itof.log 2>&1  &
sleep 1

echo roslaunch vins stereo.launch
roslaunch vins stereo.launch > $LOGCUR/vins.log 2>&1 &
sleep 1

rostopic pub -1 /vins_estimator/vins_restart geometry_msgs/PoseStamped -f /home/visbot/bin/restart.msg

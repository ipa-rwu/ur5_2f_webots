export WEBOTS_HOME=/usr/local/webots
export PYTHONPATH=${WEBOTS_HOME}/lib/controller/python27
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/webots/lib/controller
source /opt/ros/melodic/setup.bash
source $HOME/kogrob/kogrob_ws/devel/setup.bash

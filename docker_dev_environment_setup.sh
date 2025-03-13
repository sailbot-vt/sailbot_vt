# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
echo sudo chmod 777 /var/run/docker.sock >> "/home/sailbot_user/.bashrc"
echo sudo chmod -R 777 /home/ >> "/home/sailbot_user/.bashrc"


pip install -r requirements.txt
cd /home/ws/testing/pyvesc
git clone https://github.com/LiamBindle/PyVESC .
pip install .
cd /home/ws

source /opt/ros/humble/setup.bash

echo source /opt/ros/humble/setup.bash >> "/home/sailbot_user/.bashrc"
source /home/sailbot_user/.bashrc

colcon build

echo source /home/ws/install/setup.bash >> "/home/sailbot_user/.bashrc"
source /home/sailbot_user/.bashrc

colcon build


cd ground_station 
pip install -r requirements.txt


cd ..

echo 'alias python="python3"' >> /home/sailbot_user/.bashrc

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

crontab /home/ws/.devcontainer/crontabs/chmod777job.txt
echo sudo service cron start >> ~/.bashrc

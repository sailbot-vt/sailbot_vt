# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
echo sudo chmod 777 /var/run/docker.sock >> "/home/sailbot_user/.bashrc"
echo sudo chmod -R 777 /home/ >> "/home/sailbot_user/.bashrc"
echo sudo chmod -R 777 /etc/udev/ >> "/home/sailbot_user/.bashrc"
sudo chmod -R 777 /etc/udev/


# Make sure that you can just type python and you don't have to type python3 because people will get confused
echo 'alias python="python3"' >> /home/sailbot_user/.bashrc


# Install all python packages
pip install -r requirements.txt
pip install -r ground_station/requirements.txt


# Install all of the pip packages
pip install -e /home/ws/src/vesc/pyvesc/
pip install -e /home/ws/src/simulation/sailboat_gym/



# Build the ros2 workspace for the first time
source /opt/ros/humble/setup.bash
echo source /opt/ros/humble/setup.bash >> "/home/sailbot_user/.bashrc"
source /home/sailbot_user/.bashrc
colcon build --symlink-install

echo source install/setup.bash >> "/home/sailbot_user/.bashrc"




# Load all crontabs
crontab crontabs/chmod777job.txt
echo sudo service cron start >> ~/.bashrc




# Load udev rules for each device and remove any sailbot udev rules that existed before
if [ -f "/etc/udev/rules.d/99-sailbot-udev.rules" ]; then
    rm -f /etc/udev/rules.d/99-sailbot-udev.rules
fi

# echo ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="pico" >> /etc/udev/rules.d/99-sailbot-udev.rules
sudo echo ACTION=="add", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="gps" >> /etc/udev/rules.d/99-sailbot-udev.rules
sudo echo ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A9001WL3", SYMLINK+="rc" >> /etc/udev/rules.d/99-sailbot-udev.rules
sudo echo ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="ABSCDYAB", SYMLINK+="wind_sensor" >> /etc/udev/rules.d/99-sailbot-udev.rules

sudo udevadm trigger



# This is very unsafe lol but I need this to be able to access docker from inside of the dev container for the sim. 
# Do not remove unless we are releasing this software as a product
echo sudo chmod 777 /var/run/docker.sock >> "/home/sailbot_user/.bashrc"
echo sudo chmod -R 777 /home/ >> "/home/sailbot_user/.bashrc"
echo sudo chmod -R 777 /etc/udev/ >> "/home/sailbot_user/.bashrc"
sudo chmod -R 777 /etc/udev/


# Install all python packages
echo 'alias python="python3"' >> /home/sailbot_user/.bashrc

pip install -r requirements.txt

cd /home/ws/testing/pyvesc
git clone https://github.com/LiamBindle/PyVESC .
pip install .
cd /home/ws

# NEW PYVESC INSTALLATION TO GET RID OF THE TESTING FOLDER
# cd /home/ws/src/vesc
# mkdir pyvesc
# cd pyvesc
# git clone https://github.com/LiamBindle/PyVESC .
# pip install -e .
# touch __init__.py
# cd /home/ws

cd ground_station_old 
pip install -r requirements.txt
cd /home/ws


cd ground_station
pip install -r requirements.txt
cd /home/ws




# Build the ros2 workspace for the first time (sometimes this takes two colon builds to clear all of the warnings/ errors)
source /opt/ros/humble/setup.bash
echo source /opt/ros/humble/setup.bash >> "/home/sailbot_user/.bashrc"
source /home/sailbot_user/.bashrc

colcon build

echo source /home/ws/install/setup.bash >> "/home/sailbot_user/.bashrc"
source /home/sailbot_user/.bashrc

colcon build



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



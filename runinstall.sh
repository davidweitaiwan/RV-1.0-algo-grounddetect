#!bin/bash
sudo apt update && sudo apt install curl gnupg2 lsb-release -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -sudo apt update -y
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update -y
sudo apt install ros-eloquent-ros-base -y
sudo apt install python3 python3-dev python3-pip -y
sudo apt install python3-rosdep python3-colcon-common-extensions -y
echo alias eloquent="'source /opt/ros/eloquent/setup.bash'" >> ~/.bashrc
mkdir -p ~/ros2_ws/src

sudo apt install ros-eloquent-sensor-msgs* -y
sudo apt install ros-eloquent-image-transport* -y

source /opt/ros/eloquent/setup.bash
cd ~
cp -rv ~/cpp_zeddetect ~/ros2_ws/src/
cd ros2_ws
colcon build

cd ~
rm -rf run.sh && touch run.sh
cat ~/cpp_zeddetect/waitNetworkConnect.sh >> run.sh
echo "source $HOME/ros2_ws/install/setup.bash" >> run.sh
echo "ros2 run cpp_zeddetect zed" >> run.sh
sudo chmod a+x run.sh

rm -rf obsDetection.desktop && touch obsDetection.desktop
echo "[Desktop Entry]" >> obsDetection.desktop
echo "Type=Application" >> obsDetection.desktop
echo "Exec=gnome-terminal --command '$HOME/run.sh eth0'" >> obsDetection.desktop
echo "Hidden=false" >> obsDetection.desktop
echo "NoDisplay=false" >> obsDetection.desktop
echo "X-GNOME-Autostart-enabled=true" >> obsDetection.desktop
echo "Name=Detection" >> obsDetection.desktop
echo "Comment=Detection Startup" >> obsDetection.desktop
sudo cp obsDetection.desktop /etc/xdg/autostart/

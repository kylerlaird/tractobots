# Tractobots Project

Field robots

Dependencies installation (ROS Kinetic)
------------
ROS packages:

    sudo apt-get install -y ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTRO-robot-localization

Docker:

    sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 58118E89F3A912897C070ADBF76221572C52609D
    sudo apt-add-repository 'deb https://apt.dockerproject.org/repo ubuntu-xenial main'
    sudo apt-get update
    sudo apt-get install -y docker-engine

ROS Offline Google Maps with MapViz ([thanks to danielsnider](https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite/blob/master/README.md)):

    mkdir ~/mapproxy
    sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
    
Optional: [catkin_tools](http://catkin-tools.readthedocs.io/en/latest/installing.html)

Download repository and compile it:

    git clone https://github.com/kylerlaird/tractobots.git
    cd tractobots
    catkin build
    source devel/setup.bash
    
Usage
------------
On different terminals run:

    roslaunch tractobots_launchers bringup.launch
    roslaunch tractobots_launchers mapviz.launch
    
    

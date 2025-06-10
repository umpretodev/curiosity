# Caminho da pasta de instalação (ajuste se necessário)
INSTALL_PATH=install/setup.bash

.PHONY: source build run-launch

source:
	@echo "Sourcing ROS2 workspace..."
	@bash -c "source /opt/ros/humble/setup.bash && source $(INSTALL_PATH) && exec bash"

build:
	@echo "Building workspace..."
	colcon build

run-stage:
	@echo "Running launch file..."
	@bash -c "source /opt/ros/humble/setup.bash && source $(INSTALL_PATH) && ros2 launch stage_ros2 stage.launch.py world:=new_cave enforce_prefixes:=false one_tf_tree:=true"

# Caminho da pasta de instalação (ajuste se necessário)
INSTALL_PATH=install/setup.bash

.PHONY: source build run-stage run-node run-all

source:
	@echo "Sourcing ROS2 workspace..."
	@bash -c "source /opt/ros/humble/setup.bash && source $(INSTALL_PATH) && env"

build:
	@echo "Building workspace..."
	colcon build

run-stage:
	@echo "Running launch file..."
	@bash -c "source /opt/ros/humble/setup.bash && source $(INSTALL_PATH) && ros2 launch stage_ros2 stage.launch.py world:=new_cave enforce_prefixes:=false one_tf_tree:=true"

run-node:
	@echo "Running navigator node..."
	@bash -c "source /opt/ros/humble/setup.bash && source $(INSTALL_PATH) && ros2 run curiosity_cpp_pkg navigator_node"

run:
	@echo "Running everything..."
	@gnome-terminal -- bash -c "make run-stage"
	@gnome-terminal -- bash -c "sleep 2 && make run-node"

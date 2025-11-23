img?=provant

.PHONY: all provant

all: provant

provant:
	mkdir -p shared/catkin_ws/src
	-! [ -d ./shared/catkin_ws/src/ProVANT-Simulator_Developer ] && cd shared/catkin_ws/src && git clone git@github.com:Guiraffo/ProVANT-Simulator_Developer.git
	docker build -t $(img) .


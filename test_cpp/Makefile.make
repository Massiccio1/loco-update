FROM_PATH=.


all: .Kin.o .Helper.o .robot.o
	echo $(FROM_PATH)
	echo all built
%.build:
	echo $@ $<
test:
	echo testing
$(FROM_PATH)/include/%.o:../src/%.cpp
	echo $(FROM_PATH)
	echo $@: $1 :totale
	g++ -c $< -o $@ -std=c++17
#$(FROM_PATH)/include/Kin.o: $(FROM_PATH)/src/Kin.cpp
.%.o: %.cpp
	cd $(FROM_PATH)
	echo $(FROM_PATH)
	echo $*
	pwd
	echo $@: $1 :punto
	echo "g++ -c src/$*.cpp -o include/$@"
	pwd
	g++ -c $(FROM_PATH)/$*.cpp -o $(FROM_PATH)/../include/$*.o -std=c++17
	rm -f .$*.o
	touch .$*.o
.robot.o: robot.cpp
	echo vado
	echo ${ROS_PACKAGE_PATH}
	echo ${CPATH}
	g++ -c $(FROM_PATH)/robot.cpp -o $(FROM_PATH)/../include/robot.o -I/opt/ros/noetic/include -I/home/massiccio/ros_ws/install/include -std=c++17
	#g++ -c $(FROM_PATH)/robot.cpp -o $(FROM_PATH)/../include/$*.o -I/opt/ros/noetic/include -I ${HOME}/ros_ws/install/include -std=c++17
	rm -f .robot.o
	touch .robot.o
	echo fatto robot.o
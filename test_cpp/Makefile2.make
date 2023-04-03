INCLUDES = -I../src

CFLAGS = -g -Wall $(INCLUDES)

LDFLAGS = -L../src

FROM_PATH=.


all: Kin.o Helper.o
	echo $(FROM_PATH)
	echo all built
%.build:
	echo $@ $<
test:
	echo testing
# $(FROM_PATH)/include/%.o:../src/%.cpp
# 	echo $(FROM_PATH)
# 	echo $@: $1 :totale
# 	g++ -c $< -o $@
#$(FROM_PATH)/include/Kin.o: $(FROM_PATH)/src/Kin.cpp
.%.o: %.cpp
	cd $(FROM_PATH)
	echo $(FROM_PATH)
	echo $*
	pwd
	echo $@: $1 :punto
	echo "g++ -c src/$*.cpp -o include/$@"
	pwd
	g++ -c $(FROM_PATH)/$*.cpp -o $(FROM_PATH)/../include/$*.o
	rm -f .$*.o
	touch .$*.o
Kin.o: ../Kin.cpp

	$(CC) $(CFLAGS) -c ../%.c -o $*.o

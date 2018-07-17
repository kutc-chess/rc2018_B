Do: ttrnds.o RasPiDS3.o PigpioMS.o GY521.o RotaryInc.o
	g++-8.1.0 -Wall -o Do ttrnds.o RasPiDS3.o PigpioMS.o GY521.o RotaryInc.o -std=c++17 -pthread -lpigpio -lrt
RotaryInc.o: /home/pi/Sensor/RotaryInc/RotaryInc.cpp
	g++-8.1.0 -Wall -c /home/pi/Sensor/RotaryInc/RotaryInc.cpp -std=c++17 -pthread -lpigpio -lrt
GY521.o: /home/pi/Sensor/GY521/GY521.cpp
	g++-8.1.0 -Wall -c /home/pi/Sensor/GY521/GY521.cpp -std=c++17 -pthread -lpigpio -lrt
PigpioMS.o: /home/pi/PigpioMS/PigpioMS.cpp
	g++-8.1.0 -Wall -c /home/pi/PigpioMS/PigpioMS.cpp -std=c++17 -pthread -lpigpio -lrt
RasPiDS3.o: /home/pi/RasPiDS3/RasPiDS3.cpp
	g++-8.1.0 -Wall -c /home/pi/RasPiDS3/RasPiDS3.cpp -std=c++17 -pthread
ttrnds.o: ttrnds.cpp
	g++-8.1.0 -Wall -c ttrnds.cpp -lpigpio -std=c++17 -pthread -lrt
clean:
	rm -f *.o Do

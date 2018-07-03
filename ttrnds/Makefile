Do: ttrnds.o RasPiDS3.o PigpioMS.o GY521.o
	g++ -Wall -o Do ttrnds.o RasPiDS3.o PigpioMS.o GY521.o -std=c++11 -pthread -lpigpio -lrt
GY521.o: /home/pi/Sensor/GY521/GY521.cpp
	g++ -Wall -c /home/pi/Sensor/GY521/GY521.cpp -std=c++11 -pthread -lpigpio -lrt
PigpioMS.o: /home/pi/PigpioMS/PigpioMS.cpp
	g++ -Wall -c /home/pi/PigpioMS/PigpioMS.cpp -std=c++11 -pthread -lpigpio -lrt
RasPiDS3.o: /home/pi/RasPiDS3/RasPiDS3.cpp
	g++ -Wall -c /home/pi/RasPiDS3/RasPiDS3.cpp -std=c++11 -pthread
ttrnds.o: ttrnds.cpp
	g++ -Wall -c ttrnds.cpp -lpigpio -std=c++11 -pthread -lrt
clean:
	rm -f *.o Do

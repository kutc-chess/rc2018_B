Do: ArduinoMain.o RasPiDS3.o RasPiMS.o
		g++ -Wall -o Do ArduinoMain.o RasPiDS3.o RasPiMS.o -lwiringPi -std=c++11 -pthread -lrt
RasPiMS.o: /home/pi/RasPiMS/RasPiMS.cpp
		g++ -Wall -c /home/pi/RasPiMS/RasPiMS.cpp -lwiringPi -std=c++11 -pthread
RasPiDS3.o: /home/pi/RasPiDS3/RasPiDS3.cpp
		g++ -Wall -c /home/pi/RasPiDS3/RasPiDS3.cpp -lwiringPi -std=c++11 -pthread
ArduinoMain.o: ArduinoMain.cpp
		g++ -Wall -c ArduinoMain.cpp -lwiringPi -std=c++11 -pthread -lrt
clean:
		rm -f *.o Do

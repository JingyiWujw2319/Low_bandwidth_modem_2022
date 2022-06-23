# Instructions
1. Make sure you use Linux and that the USB-converter is connected to "ttyUSB0". If it is not connected to ttyUSB0 you will have to change the script so it matches to your USB port. The script does only work in loopback mode so make sure the USB-converter are set up that way before using. 
2. Add whatever content you want to send in the transmit.txt file. It can have any size but the timeout and sleep duration in the script might need to be increased for larger files (havent tested yet).
3. Set the desired baud rate at the top. For example if I want a baud rate of 600bps I would change "stty -F /dev/ttyUSB0 300" to "stty -F /dev/ttyUSB0 600".
4. Run the script with "sudo ./receive.sh"
5. The information about bit errors should be displayed in the terminal. It displays a new line for every character error it encountered and the amount of bits in each missmatch that differed. It also tells the line number and character position for which the error was encountered. 
6. You can also inspect the "receive.txt" file to see exactly what data as received through the serial port. If there were no bit errors, this file you be identical to transmit.txt

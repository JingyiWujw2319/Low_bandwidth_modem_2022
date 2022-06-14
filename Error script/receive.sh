#!/bin/bash
stty -F /dev/ttyUSB0 300
receive_file="receive.txt"
reference_file="transmit.txt"


echo "Receive script"
echo "Compiling bit error program"
gcc bit_error_check.c -o bit_error_check.out


chmod o+rw /dev/ttyUSB0
echo "Start receiving"
(stty raw; timeout 3s cat > "$receive_file") < /dev/ttyUSB0&
sleep 2 
cat "$reference_file" > /dev/ttyUSB0&

wait
echo "Done receiving. Start comparision"
./bit_error_check.out "$receive_file" "$reference_file"








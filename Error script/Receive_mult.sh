#!/bin/bash
stty -F /dev/ttyACM0 1200
stty -F /dev/ttyACM0 -echo -onlcr
> ref.txt
receive_file="receive.txt"
reference_file="transmit.txt"
times=10

echo "Receive script"
echo "Compiling bit error program"
gcc bit_error_check.c -o bit_error_check.out


chmod o+rw /dev/ttyACM0
echo "Start receiving"
(stty raw; timeout 55s cat > "$receive_file") < /dev/ttyACM0&
sleep 1
for i in $( seq 0 $times )
do
	cat "$reference_file" > /dev/ttyACM0&
	cat "$reference_file" >> ref.txt&
	sleep 5
done
wait
echo "Done receiving. Start comparision"
./bit_error_check.out "$receive_file" ref.txt








screen -L -S LW /dev/ttyACM0 1200 &
screen -ls 

echo "Please enter the PID number and the session name (EX: 2624.LW)"
read sessionName
echo "Hello, $sessionName"

screen -S 2624.LW -X readreg p transmit.txt ^C
screen -S 2624.LW -X paste p ^C

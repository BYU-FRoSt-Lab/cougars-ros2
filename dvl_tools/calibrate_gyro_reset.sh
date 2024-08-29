echo -n '{"command": "calibrate_gyro"}' | nc -q 0 192.168.194.95 16171 
echo -n '{"command": "reset_dead_reckoning"}' | nc -q 0 192.168.194.95 16171



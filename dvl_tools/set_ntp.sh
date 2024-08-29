source /home/frostlab/CougarsSetup/config/rpi_id.sh

#!/bin/bash

source rpi_id.sh
curl -X POST -d "{\"ntp_enabled\":true,\"ntp_server\":\"$STATIC_IP\",\"ntp_synchronized\":true}" -H "Content-Type: application/json" http://192.168.194.95/api/v1/time/ntp


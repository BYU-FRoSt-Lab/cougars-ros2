sudo systemctl restart chrony
ip_address=$(ifconfig eth0 | grep 'inet ' | awk '{print $2}')
curl -X POST -d "{\"ntp_enabled\":true,\"ntp_server\":\"$ip_address\",\"ntp_synchronized\":true}" -H "Content-Type: application/json" http://192.168.194.95/api/v1/time/ntp


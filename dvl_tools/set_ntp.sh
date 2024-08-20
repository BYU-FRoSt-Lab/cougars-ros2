sudo systemctl restart chrony
curl -X POST -d '{"ntp_enabled":true,"ntp_server":"192.168.194.61","ntp_synchronized":true}' -H "Content-Type: application/json" http://192.168.194.95/api/v1/time/ntp


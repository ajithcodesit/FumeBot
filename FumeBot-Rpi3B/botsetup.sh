#!/bin/bash

echo "Copying FumeBot service file"
sudo cp fumebot.service /lib/systemd/system/

echo "Changing mode"
sudo chmod +x /lib/systemd/system/fumebot.service

echo "Reload daemon"
sudo systemctl daemon-reload

echo "Enabling FumeBot service"
sudo systemctl enable fumebot.service

echo "Starting service now"
sudo systemctl restart fumebot.service
echo "Service status: $(sudo systemctl is-active fumebot.service)"
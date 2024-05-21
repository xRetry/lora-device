#!/bin/sh

set -e
apt-get update 
apt-get upgrade
apt-get install -y git podman podman-compose

git clone https://git.unileoben.ac.at/iam/lora-disaster-prevention.git /app
podman-compose up -f /app/compose.yml

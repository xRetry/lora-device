#!/bin/sh

set -e
apt-get update
apt-get install -y usermod

useradd -m -p pw config
usermod -aG sudo config

FROM --platform=linux/amd64 ghcr.io/keenanjohnson/modustoolbox-docker:main
SHELL ["/bin/bash", "-c"]

RUN apt update

COPY dev_env.sh .

RUN sh dev_env.sh

RUN mkdir -p /ws

RUN project-creator-cli --board-id CYW920829M2EVK-02 --app-id mtb-example-empty-app --user-app-name lora-device --target-dir "/ws"

WORKDIR /ws/lora-device


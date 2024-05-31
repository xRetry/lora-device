# Early Detection System for Natural Disasters
## Project Outline
This Master Data Science Project is a collaborative effort between the Chair of Automation and Measurement and the Chair of Information Technology. 
The objective is to implement an end-to-end system utilizing Internet of Things sensors and LoRaWAN communication technology to establish an early warning system for natural disasters. 
This system should provide timely data on environmental conditions that could indicate potential natural disasters. 

### Project Objectives
The project’s core mission is to deploy IoT sensors that continuously monitor critical environmental parameters, such as temperature and humidity, which are critical for predicting natural disasters. 
The sensors transmit data via a LoRaWAN gateway across The Things Network to our dedicated server, where it is processed and stored. 
Node-RED is employed to handle the incoming data streams, ensuring that relevant data is efficiently routed to our InfluxDB database. 
This data is then visualized using Grafana, enabling real-time monitoring and the capability to predict and respond to disaster scenarios promptly. 
Additionally, the project implements multi-level user accounts in both InfluxDB and Grafana to manage access rights and enhance system security and to facilitate further use of the system for upcoming projects.

### System Architecture and Components
Our system integrates IoT sensors for data collection and a LoRaWAN gateway that connects to The Things Network, a robust and globally accessible IoT network. 
This setup guarantees that data collected from remote locations is transmitted reliably, even under challenging conditions. 
The server, running on Debian and utilizing container technology, ensures a secure, stable, and scalable platform for managing the data and system operations. 
Node-RED, a flexible and powerful tool, filters and processes the data in real-time before it is stored in InfluxDB, a database chosen for its superior handling of time-series data. 
Grafana, connected to InfluxDB, offers dynamic visualization capabilities and is configured to alert administrators to potential natural disasters based on perceived trends.

### Conclusion
This project represents a significant step forward in utilizing advanced IoT and networking technologies for critical public safety applications. 
By leveraging the strengths of LoRaWAN, Node-RED, InfluxDB, and Grafana, the system will not only provide early warnings for natural disasters but also enhance our understanding of environmental patterns leading to these events.

## Hardware Components - Sensors, Microcontroller

### Microcontroller
As microcontroller, we use Infineon's `AIROC CYW20829 Bluetooth LE SoC` evaluation kit.
It provides a variety of I/O pins and includes SPI and I2C interfaces.
I2C is used to communicate with the humidity/temperature sensor, while SPI is used for the LoRaWAN module.

Infineon provides a software development kit called [ModusToolbox](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/).
This includes development tools, device drivers and runtime assets.

### Humidity/Temperature Sensor
To measure the temperature and humidity, we use a Sensirion `SHT31` sensor.
It utilizes the I2C protocol for communication.

We use a modified version of Sensirion's [embedded-sht](https://github.com/Sensirion/embedded-sht) library as hardware driver.
This library handles all implementation details and provides an easy-to-use API for the user.

### LoRaWAN Module
In order to communicate using the LoRa specification, we need a specific hardware module.
For this purpose, we use the Adafruit `RFM95W LoRa Radio Transceiver Breakout`.
It includes a `SX1276` LoRa module with SPI interface.

As hardware driver, we use the [stm32-hal-rfm95 library by henriheimann](https://github.com/henriheimann/stm32-hal-rfm95).
To make it compatible with our microcontroller, we replaced all HAL bindings with the ones of our board.

### Software

The program developed for this project sends the measured temperature and humidity values every 5 seconds via LoRaWAN to a gateway.
Both values are sent in a single message, with the payload consisting of:

```c
uint8_t data_packet[4] = { 0 };
data_packet[0] = temperature >> 8;
data_packet[1] = temperature;
data_packet[2] = humidity >> 8;
data_packet[3] = humidity;
```

### Building Process

Make sure, Infineon's [ModusToolbox](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/) is installed on the system.

Then, `cd` to the root of this repository and install the dependencies with:

```sh
make getlibs
```

Next, make sure the microcontroller is connected via USB and the current user has read/write access to the device.

```sh
chmod u+rw /dev/ttyACM0
```

Finally, compile the program and flash it onto the microcontroller with:

```sh
make program
```

In case of errors during the flashing process, make sure the power to the LoRaWAN module is disconnected during flashing.

## LoRaWAN Gateway

The LoRaWAN gateway translates messages between the LoRa protocol and HTTP.
In our case, it receives the LoRa message and translates it into an HTTP request to The Things Network.

## Debian Server

All messages sent by the microcontroller are stored in an InfluxDB database, which needs to be always available.
Therefore, this database runs on a dedicated server, alongside Node-Red and Grafana.

To simplify the installation and maintenance, we use [Podman](https://podman.io/) as containerization platform.
Containers are independent of the host system and contain the desired software with all its dependencies.
Consequently, the installation becomes trivial and can be done via a single console command and configuration file (`compose.yml`).

### Debian Setup
The Debian installation follows the installer (non-desktop version).
On the screen for software selection, only `SSH server` and `standard system utilities` are chosen

Once the installer is finished, execute the admin setup script to create the `config` user with the default password `pw`.

```sh
sh user_setup.sh
```

The second script needs to be run as this new `config` user.

```sh
sudo sh server_setup.sh
```

It is recommended to change the default password of the `config` user with:

```sh
passwd
```
## The Thing Network

[The Things Network](https://www.thethingsnetwork.org/) (TTN) is a cloud provider for LoRaWAN solutions.
It handles the registration and administration of LoRaWAN devices and gateways.

Registered gateways forward the received messages to TTN, where they get assigned to the corresponding registered devices.
These messages are further forwarded to a wide variety of user defined services, such as HTTP servers and MQTT brokers.

### The Things Network Setup

#### Gateway Registration

1. `Gateways` → `Register gateway`
2. Fill out `Gateway EUI` (printed on the gateway next to `(92)`)
3. Fill out `Gateway ID`
4. Fill out `authentication code` (Wi-Fi PW printed on gateway)
5. Select Frequency plan: `Europe 863-870 MHz (SF9 for RX2 - recommended)`
6. `Claim gateway`

#### Application Creation

1. `Applications` → `Create application`
2. Fill out `Application ID`
3. Fill out `Application name`
4. `Register end device`
5. `Enter end device specifics manually`
6. Select Frequency plan: `Europe 863-870 MHz (SF9 for RX2 - recommended)`
7. Select LoRaWAN version: `LoRaWAN Specification 1.0.0`
8. `Show advanced activation`
9. Activation mode: `Activation by personalization (ABP)`
10. Additional LoRaWAN class capabilities: `None (class A only)`
11. Network defaults: Check `Use network's default MAC settings`
12. Generate DevEUI
13. Generate Device address
14. Generate AppSKey (application session key)
15. Generate NwkSKey (network session key)
16. Click `Advanced MAC settings` → check `Reset frame counters`
17. `Register end device`

The following payload formatter converts the bytes of the message to decimal values:

```javascript
function Decoder(bytes, port) {
    const toSigned = (x) => (x > 0x7FFF) ? x - 0x10000 : x;
    return {
        temperature: toSigned((bytes[0] << 8) + bytes[1]) / 1000,
        humidity: ((bytes[2] << 8) + bytes[3]) / 1000
    };
}
```

## Node-RED
To be able to wire together hardware devices, APIs and online services, the programming tool Node-RED is used. 
It provides an browser-based editor and that makes it easy to wire together flows using the wide range of nodes in the palette that can be deployed to its runtime in a single-click. 

We have configured Node-RED to run on port 1880 on our localhost. 
Essentially, we require a link between The Things Network and InfluxDB to facilitate data transfer into our database. 
To achieve this, we employ two primary nodes within Node-RED: one node represents the inflow of data from The Things Network, and another node interfaces with InfluxDB, managing data storage. 
Additionally, we've incorporated a debug node, which is crucial for monitoring the incoming data and ensuring that the system is receiving data correctly. 

The MQTT server provided by The Things Stack can be integrated with Node-RED. 
To set up reception of events and messages, we positioned the MQTT in node on the Node-RED dashboard and configured its properties. 
In the Server dropdown menu, we selected 'Add new mqtt-broker' and clicked the adjacent button to edit it. 
In the Connection tab, we entered the MQTT server's address (excluding the port) from The Things Stack. 
The port is specified in the adjacent 'Port' field to determine the type of connection—whether it's insecure or secured with TLS. 
We opted for a TLS-secured connection, thus the port was set to 8883. 
Under the Security tab, we input the username and API token from our application as specified by The Things Stack. 
In the Properties section, we set the Topic to '#' to subscribe to all topics and selected a Quality of Service (QoS) of 2, which refers to messages being transmitted exactly once. 
Having a QoS of 0 would mean that the transmission takes place at most once and a QoS of 2 means that the transmission takes place at least once. 
After deploying and correctly setting up the system, the 'connected' status will appear below the MQTT in node. 
If the setup is successful, published event messages should then be visible through the connected debug node.

After that a change node was added, which adds a timestamp as a global variable to Node-RED. 
This means that we can access the variable at a later stage again.
Before the data is transmitted to the InfluxDB, the message received from The Things Network transmission is plucked apart. 
The only elements required in the database are the timestamps as well as the relevant measurement data. 
Of course, the exact data transmission can easily be changed by adapting the responsible function node.

To finally write data into InfluxDB, we first need to install the InfluxDB Node-RED package, known as node-red-contrib-influxdb. 
After installation, Node-RED provides nodes that can save and query data from an InfluxDB time series database. 
Next, we add an influxdb out node to our canvas and configure it by selecting 'Add new influxdb'. 
We then choose the version of InfluxDB we are using, which in our case is Version 2.0.  

To connect to our InfluxDB server, we enter the server URL and an access token. 
We also configure the database details by selecting the organization name, the bucket (which functions as the database name), and the measurement (equivalent to the table name). 

The data that is stored using the InfluxDB is a set of timeseries recording the temperature and the humidity. 
These values are stored indefinitely. 
Finally, we create the connection from the MQTT in node to the influxdb out node. 
This setup allows the seamless flow of data from The Things Network, received via MQTT, into our InfluxDB database for storage and further analysis.

## InfluxDB
InfluxDB is an open-source time series database designed to handle high write and query loads, which is an essential feature in IoT contexts where devices generate vast amounts of time-stamped data. InfluxDB operates based on organisations, buckets, and measurements. All of them define different things:

- Organsiation: the organsisation defines the main entity which administers and supervises a number of buckets and operates as a workspace for a group of users. All dashboards, buckets, and users belong to an organisation.
- Bucket: a bucket is a named location where times series data is stored. It may hold a number of different tables, similar to a SQL database.
- Measurement: a measurement is a table that holds various time series measurements. It is comparable to a SQL database table.

InfluxDB is configured to run on port 8086 of our Debian server. The InfluxDB-version that will be used is 2.7.5-alpine. 
In the compose.yaml file, we set up the initial user, as well as the organization and the initial bucket. 

Creating additional users proved more challenging than expected. While it is possible to create additional users using the console, it is not possible to administer the user-rights and privileges. For this, InfluxDB Cloud provides an option to create different API-Tokens that offer various rights to the user accessing the database with the said token. To ensure that future users, or students that build additional projects on the built infrastructure, are able to access the data, the python script `ConnectToInfluxDB.py` has been created. It enables reading and writing based on the granted rights of the API-token. 

The next interface that needs to be configured is between InfluxDB and Grafana. 

## Grafana


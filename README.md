A beacon is sent by the gateway between every 2 to 10 seconds. Each beacon specifies the exact transmission time of the next beacon (The first four bytes(characters) are gateway identifier(gw id), and the rest of the beacon message defines the next transmission time in seconds. I.e. A sample message can be ”GW0210”(gw id=GW02, next transmission=10seconds)). The Gateway sends 20 beacons consecutively in each interval and prints out information such as number of beacons sent, acks sent via serial port(you need to use a serial monitor either on arduino IDE or external program such as coolterm and set the baudrate to 9600). The gateway will then wait 10 seconds and start a new interval.
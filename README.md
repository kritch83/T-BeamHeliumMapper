# T-BeamHeliumMapper
 This is software for the TTGO T-Beam rev 1.1 to map the Helium network.  The libraries needed are included and must be the versions used.  See the hackster documentation for further setup and use. https://www.hackster.io/kritch83/ttgo-t-beam-helium-mapper-bd3017
 
 
## Using the Mapper
Here is the button mapping from left to right from a front view:

1: Power On/Off - hold 8 seconds to toggle

2: Action - press number of times within 5 seconds:
- 1 press - Low Power mode (useful for charging)
- 2 presses - Polling Speed (cycles between 15,30,60 seconds)
- 3 presses - Enable OTA mode
- 10 presses - Erase Keys

3: Reset

OTA mode is used for wireless uploads. It will create a hotspot named helium with password open4meplease. Connect to it and the IP address is 192.168.4.1 to upload to.

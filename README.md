# FYDP_ROV_2019
Final year design project

  ## Hardware requirements and setup


  ## Software prerequisites
    1. python 2 IDE
    2. VNC viewer
    3. IP scanner
    4. Arduino IDE

  ## Socket communication setup for Raspberry Pi

  ### Low-Level Communication
  Communication between Arduino controllers are mediated by Ethernet connection between two computers (Raspberry Pi and a Window    machine). Protocol used for Ethernet connection is TCP/IP, and Python scripts are used on both machines/computers in order to establish socket connection that relays information from Arduino controllers.
  ```
  ------------------        -------------------        ----------------        -----------------------        --------------
  | Remote Control | -----> | Windows Machine | <----> | Raspberry Pi | -----> | On-board Controller | <----> | Components |
  ------------------        -------------------        ----------------        -----------------------        --------------
  ```
  
  ### Arduino
  Arduinos are controllers of choice in this project - they are used as transmitter (in the form a remote control on-shore) and receiver (controller used on-board of the ROV). trasmitter.ino is uploaded in the oh-shore controller and receiver.ino is uploaded in the on-board controller. The arduino file and the required libraries are put in the same directory -- by default, I've put them in /home/document/FYP/arduino/transmitter folder and /home/pi/fydprov2019/reciever folder.    
  

  ### Raspberry Pi
  Raspberry Pi is used as buffer for data from on-board Arduino controller and also camera (in the future), which selected data will then be relayed to the Windows machine through Ethernet to be viewed by the user. There are several things that need to be setup first on Raspberry Pi for socket communication between RasPi and Windows to work properly

  1. Put launcher.sh and Python script in same directory -- by default, I've put it in /home/pi/fydprov2019 folder

  2. Enter in terminal while in /home:
      ```
      sudo +x <directory>/launcher.sh
      sudo +x <directory>/enableStaticIP.sh
      sudo +x <directory>/disableStaticIP.sh
      sudo mkdir logs
      ```

  3. Setup the static IP
      1. enter in terminal ```sudo nano /etc/network/interfaces```
      2. enter the following at the bottom of the file to setup the static IP:
      ```
      auto eth0
      iface eth0 inet static
        address 169.254.32.218
        netmask 255.255.255.0
        gateway 169.254.32.1
      ```
      3. save and exit (ctrl+x, y, enter)
      4. enter in terminal:
      ```
      sudo systemctl disable dhcpcd
      sudo systemctl enable networking
      ```

  Note that only after doing step no.2 that enableStaticIP.sh and disableStaticIP.sh can be used, though make sure that only use enableStaticIP.sh when static IP is disabled, and disableStaticIP.sh otherwise. Turn off static IP if you wish to access (shared) internet via Ethernet

  4. Setup the bootup script
      1. open crontab; enter this into the terminal: ```sudo crontab -e```
      2. if this is the first time you open crontab, you should be prompted with which text editor you want crontab to be opened with
      3. Add the following lines on the bottom, this will initialize pigpiod and run the launcher.sh (hence the Python script):
     ```
     @reboot sudo pigpiod
     @reboot sh <directory>/launcher.sh > /home/pi/logs/cronlog 2>&1
     ```
      4. save and exit (ctrl+x, y, enter)

  5. Make sure before reboot, launcher.sh and the Python script is correct

  ## How to run the ROV

  1. Boot the RaspberryPi. 
  2. Open 'sendSerial.py' on a Python IDE and run.
  3. Open terminal and launch 'readSerial.py'
   ```
   @pi ....
   ```

  ## How to control the ROV



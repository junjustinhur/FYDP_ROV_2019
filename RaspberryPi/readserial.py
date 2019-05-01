from socket import *
import select
import io
import os
import time
import pigpio
import serial
import serial.tools.list_ports
import gpiozero
#from threading import Thread

#Serial-related global variables
ser = None
dataRetLen = 19

#Ethernet-related global variables
data = None
ready = None
s = None #socket
bufferSize = 2048#8192
timeout = 3
host = "169.254.32.218"
port = 1234
socketBound = False
(conn,addr) = (None, None) #(conn,addr) comes from socket connection, socketBound shows whether socket connection is established
interrupt = False


#GPIO global variables / setup
pi = pigpio.pi()
PWM0 = 900 #at min
PWM1 = 2100 #at max
LEFT = 19
RIGHT = 18
pi.set_PWM_frequency(LEFT,50) #GPIO19 for servo14, setup at 50Hz or 20ms period
pi.set_PWM_frequency(RIGHT,50) #GPIO28 for servo23, setup at 50Hz or 20ms period
pi.set_PWM_range(LEFT,20000) #20000 means duty cycle ranges from  0 to 20000us, as servo runs at 20ms period
pi.set_PWM_range(RIGHT,20000)
pi.set_PWM_dutycycle(LEFT,PWM_left0)
pi.set_PWM_dutycycle(RIGHT,PWM_right0)

#CPUTemperature
cpu = gpiozero.CPUTemperature()


def servoWrite(pin,byte):
    global PWM0, PWM1, pi, LEFT, RIGHT
    value = PWM_left0 + byte*(PWM1-PWM0)/255.0
    pi.set_PWM_dutycycle(pin,value)

#Sends zeroes to serial
def serialZero():
    global ser
    zeroPacket = "@@@" + chr(127)*8#zero packet
    for trial in range(10):
        ser.write(zeroPacket)
        time.sleep(0.001)
        
#Hard reset serial
def serialHardReset():
    global ser
    ser.setDTR(False)
    time.sleep(0.022)
    ser.setDTR(True)

#Hard reset Ethernet
def ethernetHardReset():
    global conn, s, socketBound
    socketBound = False
    try:
        if s != None:        
            s.shutdown(2)
        if conn != None:
            conn.shutdown(2)
    except error:
        pass
    if s != None:
        s.close()
    if conn != None:
        conn.close()
        
#Clear socket's pending incoming data
def clearBuffer():
    global conn, s
    print("Clearing buffer...")
    while True:
        print(conn.recv(1))

#Initialize serial connection
def serialConnect():
    global ser
    while ser == None:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if ser != None:
                break;
            print(p)
            test = serial.Serial(p[0],57600,timeout=0)
            test.setDTR(False)
            time.sleep(0.022)
            test.setDTR(True)
            line = []
            for trial in range(2500): #try at most 2500 times to read the serial type
                if not test.isOpen():
                    test.open()
                line = test.read()
                time.sleep(0.001)
                if(line == 'R'): #receiver detected
                    test.flushInput()
                    test.close()
                    ser = serial.Serial(p[0],57600,timeout=0)
                    print("On-board controller connected through " + p[0])
                    ser.flushInput()
                    for init in range(1000):
                        ser.write('0')
                        time.sleep(0.001)
                    print("On-board controller connected!")
                    ser.flushInput()
            test.close()
    if ser == None:
        print("On-board controller not found; trying again")

#Initialize Ethernet connection
def ethernetConnect():
    global socketBound, s, conn, addr, interrupt, ready
    while not socketBound:
        print("Trying to bind socket")
        try:
            print("Connecting to " + host)
            s = socket(AF_INET, SOCK_STREAM) #create socket
            s.setsockopt(SOL_SOCKET,SO_REUSEADDR, 1) #reusing same address that's always used
            print("Socket made")
            s.bind((host,port))
            socketBound = True
            print("Socket bound")
            s.listen(5)
            (conn,addr) = s.accept() #waits for client to connect
            print("Connected!")
            interrupt = False
            #ready,_,_ = select.select([conn],[],[],timeout)
        except error as e: #if any error encountered, close the socket bound
            print(e)
            s.close()
            s = None
            socketBound = False
            time.sleep(0.5)
            '''
            if ser!=None:
                serialZero()
                ser.close()
            '''
            pass
        
def timeoutCheck(timeout):
    global lastStream, ser
    if lastStream-time.time() >= timeout:
        ethernetHardReset()
        serialZero()
        serialHardReset()
        ser.close()
    else:
        lastStream = time.time()
        
def mapFloat(num,low,high,lowNew,highNew):
    numNew = ((num-low)*(highNew-lowNew)/(high-low))+lowNew
    return numNew

def strFloat(list,low,high):
    newList = [0.0]*len(list)
    for index in range(len(list)):
        newList[index] = mapFloat(ord(list[index]),0,255,low,high)
    return newList

ethernetConnect() #establish ethernet connection between RasPi and on-shore controller
serialConnect() #establish serial connection between RasPi and on-board controller

while not interrupt:
    fileCheck = 0
    while(os.path.isfile(str(fileCheck))):
        fileCheck += 1
    file = open(str(fileCheck),"w+")
    timeStart = time.time();
    try:
        while True:
            conn.settimeout(5)
            data = conn.recv(bufferSize)#[0:6]
            ser.write(data)
            
            dataRet = ser.read(dataRetLen*2)
			
			# Only display data when data is complete
            if(len(dataRet)==dataRetLen*2):
                for trial in range(1+(len(dataRet)/2)):
                    if(dataRet[trial] == '@' and
                       dataRet[trial+1] == '@' and
                       dataRet[trial+2] == '@'):
                        servoWrite(LEFT,ord(dataRet[trial+3]))
                        servoWrite(RIGHT,ord(dataRet[trial+4]))
                        print(strFloat(dataRet[trial+3:trial+5],-45,45)+
                            strFloat(dataRet[trial+5:trial+9],1200,1800),
                              '%.2f'%cpu.temperature)                        
                        
						# Write to file all quaternion data
                        file.write(str(time.time()-timeStart) + ',')
                        file.write(str(ord(dataRet[trial+9])) + ','
                                   + str(ord(dataRet[trial+10])) + ','
                                   + str(ord(dataRet[trial+11])) + ','
                                   + str(ord(dataRet[trial+12])) + ','
                                   + str(ord(dataRet[trial+13])) + ','
                                   + str(ord(dataRet[trial+14])) + ','
                                   + str(ord(dataRet[trial+15])) + ','
                                   + str(ord(dataRet[trial+16])) + ','
                                   + str(ord(dataRet[trial+17])) + ','
                                   + str(ord(dataRet[trial+18])) + '\n')
                        break
            
            if len(data)==0:
                raise ValueError("Disconnected")
            conn.settimeout(0)
    except (ValueError,KeyboardInterrupt,error) as e:
        ethernetHardReset()
        serialZero()
        serialHardReset()
        servoWrite(LEFT,127)
        servoWrite(RIGHT,127)
        for err in e:
            print(err)
            if err == 9: #keyboard interrupt
                interrupt = True
                ser.flushInput()
                ser.close()
                print("\nInterrupt received")
                break
            else:
                print("Connection timed-out; reconnecting...")
                conn = None
                addr = None
                socketBound = False
                ethernetConnect()
                serialConnect()
                pass

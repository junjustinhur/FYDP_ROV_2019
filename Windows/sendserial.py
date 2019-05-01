from socket import *
import select
import io
import os
import time
import serial
import serial.tools.list_ports
#from threading import Thread

#Serial-related global variables
ser = None

#Ethernet-related global variables
data = None
ready = None
s = None #socket
bufferSize = 1024#8192
timeout = 3
host = '169.254.32.218'#"169.254.190.134" #IP of RasPi
port = 1234
socketConn = False
(conn,addr) = (None, None) #(conn,addr) comes from socket connection, socketConn shows whether socket connection is established
interrupt = False
zeroPacket = "@@@" + chr(127) + chr(127) + chr(127) #zero packet

#Sends zeroes to serial
def serialZero():
	global ser, zeroPacket
	for trial in range(10):
		ser.write(zeroPacket)
		time.sleep(0.001)
		
#Hard reset serial
def serialHardReset():
	global ser
	ser.setDTR(False)
	time.sleep(0.022)
	ser.setDTR(True)
	ser = None

#Hard reset Ethernet
def ethernetHardReset():
	global conn, s, socketConn
	socketConn = False
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
				if(line == 'T'): #transmitter detected
					test.flushInput()
					test.close()
					ser = serial.Serial(p[0],57600,timeout=0)
					print("On-shore controller connected through " + p[0])
					ser.flushInput()
					ser.setDTR(False)
					time.sleep(0.022)
					ser.setDTR(True)
					for init in range(1000):
						ser.write('0')
						time.sleep(0.001)
					initialized = False
					while not initialized:
						confirmation = ser.read()
						if confirmation == 'Y':
							initialized = True
					print("On-shore controller initialized!")
					ser.flushInput()
					break
			test.close()
	if ser == None:
		print("On-shore controller not found; trying again")

#Initialize Ethernet connection
def ethernetConnect():
	global socketConn, s, conn, addr, interrupt
	while not socketConn:
		print("Trying to bind socket")
		try:
			print("Connecting to " + host)
			s = socket(AF_INET, SOCK_STREAM) #create socket
			#s.setsockopt(SOL_SOCKET,SO_REUSEADDR, 1) #reusing same address that's always used
			print("Socket made")
			s.connect((host,port))
			socketConn = True
			print("Socket connected")
			#s.listen(5)
			#(conn,addr) = s.accept() #waits for client to connect
			print("Connected!")
			interrupt = False
		except error as e: #if any error encountered, close the socket bound
			print(e)
			s.close()
			s = None
			socketConn = False
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

serialConnect() #establish serial connection between RasPi and on-board controller
ethernetConnect() #establish ethernet connection between RasPi and on-shore controller

while not interrupt:
	print("Reading and sending data")
	ser.flushInput()
	'''
	fileCheck = 0
	while(os.path.isfile(str(fileCheck))):
		fileCheck += 1
	file = open(str(fileCheck),"w+")
	timeStart = time.time();
	'''
	try:
		while True:
			data = (ser.read(11))
			print(data)
			if data==('T'*11):
				for init in range(500):
					ser.write('0')
					time.sleep(0.001)
				s.send(zeroPacket)
				continue
			s.send(data)
			'''
			if(len(data) == 11):
				file.write(str(time.time()-timeStart) + ',')
				#file.write(str(data(0)) + ',' + str(data(1)) + ',' + str(data(2)) + ',' + str(data(3)) + '\n')
				file.write(str(ord(data[4])) + ',' + str(ord(data[5])) + ',' + str(ord(data[6])) + ',' + str(ord(data[7])) + '\n')
			'''
	except (KeyboardInterrupt,error) as e:
		#file.close()
		ethernetHardReset()
		serialZero()
		serialHardReset()
		for err in e:
			print(err)
			if err == 9: #KeyboardInterrupt
				interrupt = True
				ser.flushInput()
				ser.close()
				print("\nInterrupt received")
				break
			else:
				print("Connection disconnected/timed-out; reconnecting...")
				serialConnect()
				ethernetConnect()
				pass

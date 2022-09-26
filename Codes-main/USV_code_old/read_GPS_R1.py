import serial

def readGPS():
	# change ACM number before start!!!
	ser = serial.Serial("/dev/ttyACM0", 115200)  

	while (1):
		gpsChar = ser.read()
		if (gpsChar == 'A'):			# data format: A_(lat)_B_(lon)('\n')
			gpsLine  = ser.readline()	# read rest element except for 'A'
			gpsData = gpsLine.split()	# get each element in a list
			lat_0 = gpsData[0]
			lon_0 = gpsData[2]
			gpsLine = ""				# clear string 
			#print gpsData
			break						# escape while loop


	flat_0 = float(lat_0)
	flon_0 = float(lon_0)

	'''
	print lat_1
	print ('\n')
	#print flat_1
	print ('\n')
	print lon_1
	print ('\n')
	print type(flon_1)
	print ('\n')
	'''
	with open('/home/robot/rendezvous/R1.csv', 'w') as f:
		f.write('%.4f\n' %flat_0)
		f.write('%.4f\n' %flon_0)
		f.close()

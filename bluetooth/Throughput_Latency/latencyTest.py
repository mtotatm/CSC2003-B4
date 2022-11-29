import serial
import time
import mysql.connector

#serial and mysql config
serialPort = serial.Serial(port="COM5", baudrate=9600, bytesize=8, timeout=1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
databaseConn = mysql.connector.connect(user='root', password='', host='127.0.0.1', database='test')
cursor = databaseConn.cursor()

serialString = ""
inputString = ""
oneSec = 1
counter = 0
totalTime = 0
totalLen = 0
id=740

# message to transmite
message = "This is a latency test. I wonder what timing I would get.\r\n"

#SQL query to insert to latency table
insert_statement_latency = "INSERT INTO latency(ID,comTime) VALUES (%s,%s)"

serialPort.reset_output_buffer()
# get user input
modeSelect = input("Enter 'start': ")

#if user input == start, start test
if modeSelect == 'start':

    # run 1000 times
    while(id < 1000):
        #Time how long to transmite
        receive_start = time.time_ns()
        serialPort.write(message.encode('utf-8'))
        receive_end = time.time_ns()
        time.sleep(0.5)
        sendingTime = (receive_end - receive_start)

        
        #Wait until there is a data waiting in the serial buffer
        if(serialPort.in_waiting > 0):

            #Time how long to receive 
            start = time.time_ns()
            serialString = serialPort.readline().decode('utf-8')
            end = time.time_ns()
            id += 1
            receiveTime = (end - start)
            
            #convert nano second to milli second
            totalTime = (receiveTime+sendingTime)/1000000
            print("Receive: " +str(totalTime))
            
            #insert to database
            latencyData = (int(id),totalTime)
            cursor.execute(insert_statement_latency,latencyData)
            databaseConn.commit()

serialPort.close


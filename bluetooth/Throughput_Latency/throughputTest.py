import serial
import time
import mysql.connector


#serial and mysql config
serialPort = serial.Serial(port="COM5", baudrate=9600, bytesize=8, timeout=1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
databaseConn = mysql.connector.connect(user='root', password='', host='127.0.0.1', database='test')
cursor = databaseConn.cursor()

#SQL query to insert to latency table
insert_statement = "INSERT INTO drunkcar(module,dataVar,receiveDateTime) VALUES (%s,%s,%s)"
insert_statement_througput = "INSERT INTO throughput(ID,NumberOfChar,BitSize) VALUES (%s,%s,%s)"

serialString = ""
inputString = ""
oneSec = 1
counter = 0
totalTime = 0
totalLen = 0
id=0

serialPort.reset_output_buffer()

#get user input 
modeSelect = input("Enter 'start': ")


#if user input == start, start test
if modeSelect == 'start':
     # run 1000 times
    while(id < 1000):
        #Wait until there is a data waiting in the serial buffer
        if(serialPort.in_waiting > 0):
            
            #Time how long to receive 1 character
            start = time.time()
            serialString = serialPort.readline().decode('utf-8')
            end = time.time()
            totalTime += (end-start)

            #count number of char
            counter+=len(serialString)

            #count number of bytes
            totalLen+=len(serialString.encode('utf-8'))
            
            #after 1 sec get total number of character and bytes and insert to database
            if totalTime >= 1:
                id += 1
                print("Time: " + str(totalTime))
                print("Char Num: " +str(counter))
                print("Length: " + str(totalLen*10))
                throughputData = (int(id),int(counter),int(totalLen*10))
                
                cursor.execute(insert_statement_througput,throughputData)
                databaseConn.commit()

                #reset counter and time
                totalLen = 0
                counter = 0
                totalTime = 0
                    
                

databaseConn.close()
serialPort.close


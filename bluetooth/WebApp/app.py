from flask import *
import flask
import serial
import mysql.connector
import datetime
import time
import threading
app = Flask(__name__)

#serial port and database config
serialPort = serial.Serial(port="COM5", baudrate=9600, bytesize=8, timeout=1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
databaseConn = mysql.connector.connect(user='root', password='', host='127.0.0.1', database='test')
cursor = databaseConn.cursor()


def serialWrite(message):
        serialPort.write(message.encode('utf-8'))
        time.sleep(1)

def serialRead():
    #serialPort = serial.Serial(port="COM5", baudrate=9600, bytesize=8, timeout=1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
    serialPort.reset_output_buffer()
    insert_statement =""
    while(1):
        if(serialPort.in_waiting > 0):
            d = datetime.datetime.now()
            serialString = serialPort.readline().decode('utf-8')
            #print(serialString)
            time.sleep(1)
            
            dataArr = serialString.split("-")
            print(dataArr[0])
            print (dataArr[1])
            try:
                if "Mapping" in dataArr[0]:
                    insert_statement = "INSERT INTO mapping(module,dataVar,receiveDateTime) VALUES (%s,%s,%s)"
                    data = (dataArr[0],dataArr[1],d)
                    cursor.execute(insert_statement,data)
                    databaseConn.commit()
                elif "Message" in dataArr[0]:
                    insert_statement = "INSERT INTO message(module,dataVar,receiveDateTime) VALUES (%s,%s,%s)"
                    data = (dataArr[0],dataArr[1],d)
                    cursor.execute(insert_statement,data)
                    databaseConn.commit()
                else:
                    insert_statement = "INSERT INTO drunkcar(module,dataVar,receiveDateTime) VALUES (%s,%s,%s)"
                    d = datetime.datetime.now()
                    data = (dataArr[0],dataArr[1],d)
                    cursor.execute(insert_statement,data)
                    databaseConn.commit()
            except IndexError:
                pass

        
            print(serialString)
          
def appRun():
    app.run()

@app.route("/getdata",methods=['GET'])
def data():
    if flask.request.method == 'GET':
        recent = ''
        timeNow = datetime.datetime.now()
        timeNow = str(timeNow)
        select_statement = "SELECT * FROM drunkcar ORDER BY receiveDateTime DESC LIMIT 1"
        select_statement1 = "SELECT * FROM message ORDER BY receiveDateTime DESC LIMIT 1"
        select_statement2 = "SELECT * FROM mapping ORDER BY receiveDateTime DESC LIMIT 1"
        
        #fetch last row of drunkcar table
        cursor.execute(select_statement)        
        result = cursor.fetchone()
        
        #fetch last row of message table
        cursor.execute(select_statement1)
        message = cursor.fetchone()

        #fetch last row of mapping table
        cursor.execute(select_statement2)
        mapping = cursor.fetchone()

    
    print(result)
    print(message)
    print(mapping)
    databaseConn.commit()    

    return jsonify(drunk=result, message=message, mapping=mapping)


@app.route('/',methods=['GET','POST'])
def index():
    if flask.request.method == 'GET':
        return render_template("index.html")
    if flask.request.method == 'POST':

        #Get user input
        start = request.form['Start']
        end = request.form['End']
        direction = request.form['Direction']
        #Put the input as 1 string
        message = start+","+end+","+direction+"\r\n"

        #transmit message to buffer
        serialWrite(message)
        print(message)
        return render_template("index.html")
    return render_template("index.html")

if __name__ == '__main__':
    #Multithread to run web app and read serial port
    t1 = threading.Thread(name='serialRead', target=serialRead)
    t2 = threading.Thread(name="apprun", target=appRun)
    t1.start()
    t2.start()
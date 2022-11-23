#!/usr/bin/env python

import serial
import time
import io

ser = 0

def COM_SendData(data):
    ser.write(data + '\n')


def COM_RecvData():
    data = ser.readline()
    #print data
    return data

def COM_OpenConnection(ePort, eRate, nRetries=5):

    global ser
    try :
        ser = serial.Serial(
            port=ePort,
            baudrate=eRate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

        while(not ser.isOpen()) :
            ser.open()

            if(not ser.isOpen()) :
                ser.close()

        if(not ser.isOpen()) :
            print 'Problem opening port'

        # 0 = everything ok
        return [not ser.isOpen(),ser,0]

    except :
        print "Connection Error - Radio not connected?"
        return [1]


def COM_CloseConnection():

    global ser
    ser.close()
    #print "ser.isOpen() - ", ser.isOpen()


    if(ser.isOpen()) :

        print 'Problem closing port'

    return [ser.isOpen(),0,[]]

# def SerialRequest(data, length = 0, t_timeout = 0.1): #default 3
#
#     global ser
#     #response = None
#     try :
#         ser.read(ser.inWaiting())
#         ser.write(data + '\r\n')
#         print "FUNC: Sending data \n", data
#
#         t_start = time.time()
#         # do as long as:
#         # 1: buffer has specific length
#         # 2: if specific length not defined (=0), then until buffer > 0
#         # 3: timeout not reached
#
#         while((ser.inWaiting()<length or (length == 0 and ser.inWaiting()==0)) and time.time()-t_start<t_timeout) :
#             # time.sleep(0.001)
#             time.sleep(0.0005)
#
#         if(time.time()-t_start>=t_timeout) :
#             return "FUNC: Timeout, no response!"
#
#         # time.sleep(0.015)    # Short break to make sure serial port is not read while stuff is written
#         time.sleep(0.0005)
#
#         response = ser.read(ser.inWaiting())
#
#
#     except :
#         print "FUNC: Error"
#         response = None
#
#     print "FUNC: response: ", response, "\n"
#     return response

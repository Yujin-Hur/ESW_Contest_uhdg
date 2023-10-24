import serial
import time
import math
import RPi.GPIO as GPIO 
import boto3
import picamera
from bluetooth import *

# CONFIG
UPDOWNTIME = 2  # time to admit stair (not temporary updown situation)
UPDOWNANGLE = 5.3 # stair detect angle
UPDOWNOFFSET = -8.78 
PRINTFREQ = 1

NOISETIME = 1   # time to check data is noise or not
INSIDETIME = 3  # time to admmit inside situation

# GPIO
LOCKPIN = 15    # CAR ON/OFF
# IMU
PORT0 = '/dev/ttyUSB0'  
BAUD0 = 115200
OFFSET = 72
# GPS
PORT1 = '/dev/ttyUSB1'  
BAUD1 = 9600
# camera
camera = picamera.PiCamera()

# MAIN
turn_off = 0
inside_flag = 0  # 0: outside / 1: inside / 2: checking...
pitch_flag = 0  # pitch in threshold
floor_detect_flag = 0  # pitch flag 1 -> check noise or not
flat_flag = 1 # 1: flat
pitch_start = time.time()
pitch_end = time.time()
inside_start = time.time()
floor = 0
gps_input = -1
print_time = time.time()
ocr_text = "3floor"
cnt = 0

class IMU:
    def __init__(self, port, baud):
        self.ser = serial.Serial(
            port = port,
            baudrate = baud,
            bytesize = serial.EIGHTBITS,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout = 0.001
        )
        self.pitch = 0
        self.yaw = 0
        self.mag = [0,0,0]
        self.ser.flushInput()
        self.send_str('<reset>')
                              
    def recv_str(self):
        try:
            datainput = ''
            s = self.ser.read().decode()
            if (s == '*'):
                count = 0
                while (True):
                    if (s == '\n'):
                        break
                    if (datainput != ''):
                        if (datainput[len(datainput) - 2] == '\r'):
                            count += 1
                            break
                    s = self.ser.read().decode()
                    datainput = datainput + s
                    if s == '\n' or s == '\r':
                        count += 1
                datainput = datainput[0:len(datainput) - count] # to erase \r\n
                
            return (datainput)
        except KeyboardInterrupt:
            self.ser.close()
            exit(0)
        except:
            pass

    def send_str(self, msg):
        self.ser.write(msg.encode())

    def calc_angle(self):
        bearing = self.yaw - OFFSET
        if bearing < 0:
            bearing += 360
        return bearing

class GPS:
    def __init__(self, port, baud):
        self.lat = 0.0
        self.lon = 0.0
        self.valid = 'V'  # A: data O / V: data X
        self.ser = serial.Serial(
            port = port,
            baudrate = baud,
            bytesize = serial.EIGHTBITS,
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            timeout = 0.001
        )
        self.ser.flushInput()
        self.inside_start = time.time()
        self.outside_start = time.time()
        
    def recv_str(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            # $GPRMC,,V,,,,,,,,,,N*53
            if line.startswith('$GPRMC'):
                #print(line)
                splited_str = line.split(',')
                self.valid = splited_str[2]
                if(self.valid == 'A'):
                    self.lat = float(splited_str[3])/10
                    self.lon = float(splited_str[5])/10
                    return 1
                elif(self.valid == 'V'):
                    return 0
            return 2
        except:
            pass

    # status / 0: outside / 1: inside / 2: don't know
    def check_inside(self, status):
        # outside -> inside
        if(status == 0):    
            self.inside_start = time.time()
            status = 2 
        # need check
        
        elif(status == 2 and  time.time() - self.inside_start < INSIDETIME): # check inside
            status = 1
        return status
    
    def check_outside(self, status):
        # inside -> outside
        if(status == 1):    
            self.outside_start = time.time()
            status = 2 
        # need check
        elif(status == 2 and  time.time() - self.outside_start < INSIDETIME): # check inside
            status = 0
        return status

def s3_connection():
    try:
        s3 = boto3.client(
            service_name="",
            region_name="",
            aws_access_key_id="",
            aws_secret_access_key="",
        )
        try:
            s3.upload_file("image.jpg","bucket","image.jpg")
        except Exception as e:
            print(e)
    except Exception as e:
        print(e)
    else:
        print("s3 bucket connected!") 
        return s3

    
def ble_send(ble_socket, floor, angle, ocr_text):
    send_str = ["e", int(angle), floor]
    for i in range(len(send_str)):
        print(str(send_str[i]))
        ble_socket.send(str(send_str[i]))
        time.sleep(1)

def ble_connect():
    server_socket= BluetoothSocket(RFCOMM)
    port = 1
    server_socket.bind(("", port))
    server_socket.listen(1)
    uuid = ""
    advertise_service( server_socket, "AquaPiServer",
                    service_id = uuid,
                    service_classes = [ uuid, SERIAL_PORT_CLASS ],
                    profiles = [ SERIAL_PORT_PROFILE ], 
                    # protocols = [ OBEX_UUID ] 
                    )
    print("Waiting for bluetooth connect...")
    ble_socket, address = server_socket.accept()
    print("Accepted connection from ", address)
    ble_socket.send("bluetooth connected!")
    return server_socket, ble_socket

if __name__ == "__main__":
 
    ebimu = IMU(PORT0,BAUD0)
    gps = GPS(PORT1,BAUD1)
    
    GPIO.setwarnings(False) 
    GPIO.setmode(GPIO.BCM) 
    GPIO.setup(LOCKPIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

    server_socket, ble_socket = ble_connect()  # bluetooth

    try:
        while(1):
            
            ''' CAR TURN OFF '''
            if GPIO.input(LOCKPIN) == GPIO.HIGH:
                # send floor and car angle and ocr
                angle = ebimu.calc_angle()
                ble_send(ble_socket, floor, angle, ocr_text)
                # send photo to cloud
                camera.capture('image.jpg') # make image
                s3_connection()
                print("car is turn off!")
                #break
            
            ''' SENSOR INPUT '''
            imu_input = ebimu.recv_str()
            gps_input = gps.recv_str()
            gps_input = 0
                
            ''' CHECK INSIDE OR OUTSIDE (USING GPS) '''
            if(gps_input == 1):
                inside_flag = gps.check_outside(inside_flag)
            elif(gps_input == 0):
                inside_flag = gps.check_inside(inside_flag)
            # inside_flag = 1
            
            ''' CHECK FLOOR UP OR DOWN (USING IMU) '''
            if (imu_input):

                # Parsing IMU DATA
                splited_str = imu_input.split(',')
                ebimu.pitch = float(splited_str[0]) - UPDOWNOFFSET # MINUS UP/ PLUS DONW
                ebimu.yaw = float(splited_str[2])

                # Check UPDOWN
                if(abs(ebimu.pitch) > UPDOWNANGLE and inside_flag == 1):

                    ## updown start
                    if(pitch_flag == 0 ):
                        pitch_flag = 1
                        pitch_start = time.time()
                        print("detect updown")
                    
                    ## updown check 
                    if(time.time()-pitch_start > UPDOWNTIME and flat_flag == 1):
                        flat_flag = 0
                        floor_detect_flag = 1

                        if(ebimu.pitch < 0):
                            if(floor == 0):
                                floor = 1
                            floor+=1
                            print("check climb up! you are in ", floor)
                        else:
                            floor-=1
                            print("check climb down! you are in ", floor)
                        
                # Check FLAT (during updown)
                if(floor_detect_flag == 1 and abs(ebimu.pitch) <= UPDOWNANGLE):
                    ## Flat Start
                    if(pitch_flag == 1):
                        pitch_flag = 0
                        pitch_end = time.time() 
                        print("detect flat")
                    ## Flat Check
                    elif(flat_flag == 0 and time.time()-pitch_end > NOISETIME):
                        flat_flag = 1
                        print("check flat")
    except:
        exit(0)
import struct
import can

class HydraulicValve:

    def __init__(self,deadband,maxflow,incoming_can_id):
        #Max flow in litres / min
        self.deadband = deadband
        self.maxflow = float(maxflow)
        self.incoming_can_id = incoming_can_id
        #Flow in mm^3/s
        self.flow = 0.0
        self.flowcoeff = maxflow/float((250-deadband)**2)

    def setFlow(self,setting,direction):
        #Flow to mm3/s, multiply by 1e6 divide by 60
        mult = 0
        if direction == 1:
            mult = 1
        if direction == 2:
            mult = -1
        if setting < self.deadband:
            self.flow = 0.0
        else:
            self.flow = 100000.0/6.0*mult*self.flowcoeff*(setting-self.deadband)**2

    def getFlow(self):
	return self.flow

    def readCanCommand(self,can_message):
        if can_message.arbitration_id == self.incoming_can_id:
            flow_command = int(can_message.data[0])
            direction = int(can_message.data[2])-192
        else:
            print "Wrong valve ID"
            flow_command = 0
            direction = 0
        self.setFlow(flow_command,direction)

class SteerCylinder:

        def __init__(self,stroke,area):
            #All in mm!
            self.stroke = stroke
            self.position = 0.0
            self.area = area

        def move(self,flow,timestep):
            self.position += flow/self.area
            if self.position > self.stroke:
                self.position = self.stroke
            if self.position < -self.stroke:
                self.position = -self.stroke
            return self.position

class AngleSensor:

    def __init__(self,resolution,can_id=0x18FF0B80):
        self.resolution = 0.1
        self.can_id = can_id
        self.angle = 0.0

    def setAngle(self,angle):
        #From deg to int
        if angle < 0:
            angle += 360.0
        self.angle = int(angle/self.resolution)

    def getAngleDeg(self):
        return self.angle*self.resolution

    def getCanMessage(self):
        dataframe = bytearray(8)
        struct.pack_into(">H",dataframe,0,self.angle)
        struct.pack_into(">H",dataframe,2,self.angle)
        dataframe[4]=0xFF #Reserved bits
        dataframe[5]=0xFF
        dataframe[6]=0xFF
        dataframe[7]=0x00 #No error
        return can.Message(arbitration_id=self.can_id,data=dataframe,is_extended_id=True)

class SteeringAngleSensor:

    def __init__(self,type='SASAIID',can_id=0x0CFF104D):
        self.angle = 0.0
        self.rpm = 0.0
        self.can_id = can_id
        self.type = type

    def setAngle(self,angle):
        if self.type == 'SASAIID':
            #From deg to int
            if angle < 0:
                angle += 360.0
            self.angle = int(angle/360.0*4096)

    def getAngleDeg(self):
        if self.type == 'SASAIID':
        	return float(self.angle)/4096*360.0

    def setRPM(self,rpm):
        if self.type == 'SASAIID':
            #Scale
            self.rpm = int(rpm/300.0*20480+20480)

    def getRPM(self):
        if self.type == 'SASAIID':
        	return float(self.rpm-20480)/20480*300.0

    def getCanMessage(self):
        dataframe = bytearray(8)
        struct.pack_into(">H",dataframe,0,self.angle)
        struct.pack_into(">H",dataframe,2,self.rpm)
        dataframe[4]=0xFF
        dataframe[5]=0xF0
        dataframe[6]=0x00
        dataframe[7]=0x00
        return can.Message(arbitration_id=self.can_id,data=dataframe,is_extended_id=True)

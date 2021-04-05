import th
import can
import time

def twentyhz(exectime):
	time1 = time.time()
    while True:
    	if time.time() > (time1+0.05-exectime):
        	break

bus = can.interface.Bus(channel='vcan0',bustype='socketcan')

#Setup devices
sasa = th.SteeringAngleSensor()
dst510 = th.AngleSensor(0.1)
valve = th.HydraulicValve(75,10,0x0CFE3022)
cyl = th.SteerCylinder(140,3284)
sr = th.SteeringRack((0,-787),(-125,-375),(-200,-731))

#Test SASA
for angle in range(0,10):
	sasa.setAngle(angle+34.5)
	sasa.setRPM(2.4)
	bus.send(sasa.getCanMessage())

#Test angle sensor
for angle in range(0,10):
	dst510.setAngle(angle+0.7)
	bus.send(dst510.getCanMessage())

#Test valve control, listens to messages from vcan0
message = bus.recv()
while True:
	time1=time.time()
	new_message=bus.recv(0.0)
    if new_message != None:
        message=new_message
	valve.readCanCommand(message)
	print "Valve flow mm3/s:         " + str(valve.getFlow())
	pos = cyl.move(valve.getFlow(),0.1)
	print "Cylinder displacement mm: " + str(pos)
	sr.setDisplacement(pos)
	angle = sr.getSteerAngleDegrees()
	print "Steer angle degrees:      " + str(angle)
	dst510.setAngle(angle)
	bus.send(dst510.getCanMessage())
	twentyhz(time.time()-time1)

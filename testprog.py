import th
import can
import time

bus = can.interface.Bus(channel='can0',bustype='socketcan')
bus.set_filters(filters=[{"can_id": 0x0CFE3022, "can_mask": 0x1FFFFFFF, "extended": True}])

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
for angle in range(-10,0):
	dst510.setAngle(angle/10.0)
	bus.send(dst510.getCanMessage())
	time.sleep(0.5)

#Test valve control, listens to messages from vcan0
message = bus.recv()
time1=time.time()
time2=time.time()
while True:
	message=bus.recv()
	valve.readCanCommand(message)
	print "Valve flow mm3/s:         " + str(valve.getFlow())
	pos = cyl.move(valve.getFlow(),time.time()-time1)
	print time.time()-time1
	time1=time.time()
	#print "Cylinder displacement mm: " + str(pos)
	sr.setDisplacement(pos)
	angle = sr.getSteerAngleDegrees()
	print "Steer angle degrees:      " + str(angle)
	dst510.setAngle(angle)
	#Send angle every 100 ms
	if (time.time()-time2)>0.1:
		bus.send(dst510.getCanMessage())
		time2=time.time()

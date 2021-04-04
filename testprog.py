import th
import can

bus = can.interface.Bus(channel='vcan0',bustype='socketcan')

#Setup devices
sasa = th.SteeringAngleSensor()
dst510 = th.AngleSensor(0.1)
valve = th.HydraulicValve(75,10,0x0CFE3022)
cyl = th.SteerCylinder(200,3284)

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
while True:
	message = bus.recv()
	valve.readCanCommand(message)
	print "Valve flow mm3/s: " + str(valve.getFlow())
	pos = cyl.move(valve.getFlow(),0.1)
	print "Cylinder displacement mm: " + str(pos)

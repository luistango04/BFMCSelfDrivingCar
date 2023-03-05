import serial
import time
import sys
import usb
import usb.core
import usb.util

global sleep
sleep = 1

global kp
kp = 0.1

global ki
ki = 0.03

global kd
kd = 0.0005

global kt
kt = 0.3

#dev1 = usb.core.find(idVendor=0x0483, idProduct=0x374b) #Nucleo-F103RB

# Detach the kernel driver from the USB device
#if dev1.is_kernel_driver_active(0):
#    dev1.detach_kernel_driver(0)

# Close the USB port
#usb.util.dispose_resources(dev1)

# Re-open the USB port
#dev1.reset()

dev2 = usb.core.find(idVendor=0x8086, idProduct=0x0b3a) #Intel D435i

# Detach the kernel driver from the USB device
if dev2.is_kernel_driver_active(0):
    dev2.detach_kernel_driver(0)

# Close the USB port
usb.util.dispose_resources(dev2)

# Re-open the USB port
dev2.reset()

time.sleep(sleep*4)

ser = serial.Serial('/dev/ttyACM1', 19200, timeout=0.1)
#ser.flush()
#ser.write(b'#2:10;;\r\n')
#print(ser.read())
#time.sleep(sleep)
#ser.flush()
#ser.write(b'#2:-10;;\r\n')
#print(ser.read())
#time.sleep(sleep)
#ser.flush()
ser.write(b'#2:0;;\r\n')

line = ser.readline().decode('utf-8').rstrip() # Read a line from serial and remove any trailing whitespace
print(line)

#print(ser.read())
#time.sleep(sleep)
#ser.write(b'#1:0.15;;\r\n')

#line = ser.readline().decode('utf-8').rstrip() # Read a line from serial and remove any trailing whitespace
#print(line)

#time.sleep(sleep*5)
#ser.write(b'#1:0;;\r\n')

#line = ser.readline().decode('utf-8').rstrip() # Read a line from serial and remove any trailing whitespace
#print(line)


#ser.flush()
ser.write(b'#4:1;;\r\n')

line = ser.readline().decode('utf-8').rstrip() # Read a line from serial and remove any trailing whitespace
print(line)

#print(ser.read())
#time.sleep(sleep)
#ser.flush()
#0.1, 0.03, 0.0005, 0.3, 5
#ser.write(b'"#6:{kp};{ki};{kd};{k_t};;\r\n')
#ser.write(b'"#6:{kp};{ki};{kd};{k_t};;\r\n')
ser.write(f'#6:{kp};{ki};{kd};{kt};;\r\n'.encode())

line = ser.readline().decode('utf-8').rstrip() # Read a line from serial and remove any trailing whitespace
print(line)

#print(ser.read())
#time.sleep(sleep)
#ser.flush()
#ser.write(b'#1:0.15;;\r\n')
#print(ser.read())
#time.sleep(sleep*5)
#ser.flush()
#ser.write(b'#5:1;;\r\n')
#print(ser.read())
#time.sleep(sleep)
#ser.flush()
#ser.write(b'#1:0;;\r\n')
ser.write(b'#7:0.05;0.05;;\r\n')

line = ser.readline().decode('utf-8').rstrip() # Read a line from serial and remove any trailing whitespace
print(line)

time.sleep(sleep*2)
ser.flush()
ser.close()

#print(ser.read())
#time.sleep(sleep*2)
#ser.flush()
#ser.close()



"""
#    #Part to reattach camera through software
#    import time
#
#    import usb
#    import usb.core
#    import usb.util

    dev = usb.core.find(idVendor=0x8086, idProduct=0x0b3a) #Intel D435i

    # if the device is found, reset its USB connection
    if dev is not None:
        try:
            # detach the device from the kernel driver
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
            
            # reset the device
            dev.reset()
    
            # reattach the device to the kernel driver
            usb.util.dispose_resources(dev)
            dev.attach_kernel_driver(0)
        except usb.core.USBError as e:
            print("USBError: {}".format(str(e)))
    else:
        print("USB device not found")
    time.sleep(2)

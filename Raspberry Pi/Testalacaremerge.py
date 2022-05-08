from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import cv2
import numpy as np
import yaml
from dataHandler import imageproc, writer
from Pico import pico


sleep(5)

ang_res = 200
sec = 360 / ang_res 

improc = imageproc()
wr = writer('data.json')
pico = pico()


#camera.start_preview()
#sleep(100)
#CHECK THE CONECTION WITH PICO AND SEND THE IP
pico.ck()
sleep(0.5)
pico.servo(19)
pico.sendIp()
sleep(1)
pico.laserOff()
pico.ledsOn()

#WAIT UNTIL THE USER PRESS THE BUTTON
while not pico.start():
    sleep(0.5)

sleep(5)

#TURN ON LASER
pico.ledsOff()
pico.laserOn()
pico.LCDOff()

#TAKE PIC FOR DEPTH
with PiCamera() as camera:
	# Create the in-memory stream
	stream = BytesIO()
	camera.resolution = (1280, 720)#(1280, 720)#(4056, 3040) #max res, pretty f sick #right now the cal file is only made for this specific res
	#in the futere each res should have it's own cal file
	camera.rotation = 180
	camera.capture(stream, format='jpeg')

	#CONVERT TO OPENCV ---- to be improved
	im = Image.open(stream)
	#im.save("lol.jpg")
	img = np.array(im) 
	h,  w = img.shape[:2]

	#SET DEPTH IMAGE
	improc.setImg(img)
	improc.undist("coef.yml")

#TURN LASER OFF
pico.laserOff()
pico.ledsOn()

sleep(0.5)

#TAKE PIC FOR COLOR
with PiCamera() as camera:
	# Create the in-memory stream
	stream = BytesIO()
	camera.resolution = (1280, 720)#(1280, 720)#(4056, 3040) #max res, pretty f sick #right now the cal file is only made for this specific res
	#in the futere each res should have it's own cal file
	camera.rotation = 180
	camera.capture(stream, format='jpeg')

	#CONVERT TO OPENCV ---- to be improved
	im = Image.open(stream)
	img = np.array(im) 
	h,  w = img.shape[:2]

	#CONVERT TO OPENCV ---- to be improved
	im = Image.open(stream)
	img = np.array(im) 
	h,  w = img.shape[:2]

	#SET COLOR IMAGE
	improc.setColor(img)
	improc.undistC("coef.yml")

m = improc.combineFilter(dilate=3)

#immmask = Image.fromarray(improc.getMask(m).astype('uint8'), 'RGB')
#immmask.save("maska.jpg")

line = improc.localMaxQUICK(m)

#READ THE WEIGHT SENSOR
weight = pico.getWeigt() #greutate

wr.setHeader(improc.getRes(), weight)

wr.addData(line, 0)
#cv2.destroyAllWindows()

for i in range(1, ang_res):
	#MOVE BED
	pico.rotateBed(i*sec)

	#TURN LASER ON
	pico.ledsOff()
	pico.laserOn()

	#TAKE PIC FOR DEPTH
	with PiCamera() as camera:
		# Create the in-memory stream
		stream = BytesIO()
		camera.resolution = (1280, 720)#(1280, 720)#(4056, 3040) #max res, pretty f sick #right now the cal file is only made for this specific res
		#in the futere each res should have it's own cal file
		camera.rotation = 180
		camera.capture(stream, format='jpeg')

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		img = np.array(im) 
		h,  w = img.shape[:2]

		#SET DEPTH IMAGE
		improc.setImg(img)
		improc.undist("coef.yml")

	#TURN LASER OFF
	pico.laserOff()
	pico.ledsOn()

	#TAKE PIC FOR COLOR
	with PiCamera() as camera:
		# Create the in-memory stream
		stream = BytesIO()
		camera.resolution = (1280, 720)#(1280, 720)#(4056, 3040) #max res, pretty f sick #right now the cal file is only made for this specific res
		#in the futere each res should have it's own cal file
		camera.rotation = 180
		camera.capture(stream, format='jpeg')

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		img = np.array(im) 
		h,  w = img.shape[:2]

		#CONVERT TO OPENCV ---- to be improved
		im = Image.open(stream)
		img = np.array(im) 
		h,  w = img.shape[:2]

		#SET COLOR IMAGE
		improc.setColor(img)
		improc.undistC("coef.yml")

	#CREATE THE MASK
	m = improc.combineFilter(dilate=3)

	#EXTRACT THE LASER POSITION ALONG WITH THE COLOR
	line = improc.localMaxQUICK(m)

	#ADD DATA TO THE JSON FILE
	wr.addData(line, i*sec)
	
#ROTATE BED TO THE INITIAL POSITION
pico.rotateBed(ang_res*sec)
pico.LCDOn()

#SAVE THE DATA
wr.save()

#CONVERT TO GRAYSCALE
#gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
#cv2.imwrite('/home/pi/Desktop/test5.png',img)

#sleep(1)
camera.close()

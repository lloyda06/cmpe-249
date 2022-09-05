from periphery import GPIO,PWM
from time import sleep
#import keyboard
import sys
from sshkeyboard import listen_keyboard, listen_keyboard_manual
import asyncio
import cv2
import os
import time


class RC_Car:
    def __init__(self, fwdGPIO, revGPIO, leftGPIO, rightGPIO, speedPWM, steerPWM):
        self.speed = 0
        #left/right, left is -1, right is 1, straight is 0
        self.direction = 0
        self.fwdGPIO = fwdGPIO
        self.revGPIO = revGPIO
        self.leftGPIO = leftGPIO
        self.rightGPIO = rightGPIO
        self.speedPWM = speedPWM
        self.steerPWM = steerPWM
        self.cam = Camera()

        self.speedPWM.frequency = 1e2
        self.steerPWM.frequency = 75
        self.speedPWM.enable()
        self.steerPWM.enable()

    def snap(self):
        fname = "img_" + str(time.time()) + "_" + str(self.direction) + "_" + str(self.speed)
        self.cam.snap(fname)

    def changeSpeed(self, delta):
        newSpeed = self.speed + delta
        newSpeed = max(-1, min(1, newSpeed))

        #s = 0, dc = 0.6

        #0.6 = 0x + b
        #1 = 1x + b
        #b = 0.6
        #x = 0.4
        dc = 0.3*abs(newSpeed) + 0.35
        #dc = 0.85

        if abs(newSpeed) < 0.01:
            dc = 0
            newSpeed = 0

        #self.speedPWM.duty_cycle = dc
        self.speed = newSpeed
        print("New Speed -->", self.speed, "New DC -->", dc)
        if self.speed > 0:
            self.revGPIO.write(False)
            self.fwdGPIO.write(True)
        elif self.speed < 0:
            self.fwdGPIO.write(False)
            self.revGPIO.write(True)
        else:
            self.fwdGPIO.write(False)
            self.revGPIO.write(False)

        self.speedPWM.duty_cycle = dc



    def changeDirection(self, delta):
        newDir = self.direction + delta
        newDir = max(-1, min(1, newDir))
        
        dc = 0.6*abs(newDir) + 0.25
        #dc = 0.75

        if abs(newDir) < 0.01:
            newDir = 0
            dc = 0


        #self.steerPWM.duty_cycle = dc
        self.direction = newDir

        t = time.time()
        #self.cam.snap("img_" + str(t) + "_" + str(self.direction))

        print("New Steer -->", self.direction, "New DC -->", dc)
        if self.direction > 0:
            self.leftGPIO.write(False)
            self.rightGPIO.write(True)
        elif self.direction < 0:
            self.rightGPIO.write(False)
            self.leftGPIO.write(True)
        else:
            self.rightGPIO.write(False)
            self.leftGPIO.write(False)

        self.steerPWM.duty_cycle = dc

        #self.snap()


    #Go in the opposite direction for a short time to stop quick
    #Then bring speed to zero
    #Just bring steer to zero
    def stop(self):
        brake_delta = 0
        speed_delta = 0
        if self.speed > 0:
            brake_delta = -1 - self.speed
            speed_delta = 1
        elif self.speed < 0:
            brake_delta = 1 - self.speed
            speed_delta = -1
        #speed_delta = 0 - self.speed
        steer_delta = 0 - self.direction
        self.changeSpeed(brake_delta)
        self.changeDirection(steer_delta)
        sleep(0.3)
        self.changeSpeed(speed_delta)

    def destroy(self):
        self.speedPWM.duty_cycle = 0
        self.steerPWM.duty_cycle = 0
        self.revGPIO.close()
        self.fwdGPIO.close()
        self.leftGPIO.close()
        self.rightGPIO.close()


class Camera:
    def __init__(self):
        cap = cv2.VideoCapture(1)

        sr = cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

        self.cap = cap
        self.images = {}

    def snap(self, fname=None):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("No Cam Return")
                return
            cv2_im = frame
            #cv2_im  =cv2.flip(cv2_im, -1)
            if fname != None:
                self.images[fname] = cv2_im
                print("store in self.images", fname)
                #cv2.imwrite("/home/mendel/" + fname + ".jpg", cv2_im_rgb)
            return cv2_im
        else:
            print("CAM NOT OPEN!!")

    def save(self):
        for fname in self.images:
            img = self.images[fname]
            print("SAVING IMG -->", fname)
            write = cv2.imwrite("/home/mendel/" + fname + ".jpg", img)
            print("Wrote -->", write)
        #Clear out images once saved
        self.images = {}

    def dump(self):
        self.images = {}

    def destroy(self):
        self.cap.release()



#Turn Left
gpio9 = GPIO("/dev/gpiochip0", 9, "out")    # pin 11
#Turn Right
gpio10 = GPIO("/dev/gpiochip0", 10, "out")  # pin 13
#Go Rev #OLD-Forward
gpio0 = GPIO("/dev/gpiochip0", 0, "out")    # pin 16
#Go Fwd #OLD-Back
gpio1 = GPIO("/dev/gpiochip0", 1, "out")    # pin 18

#Speed PWM
pwm_a = PWM(0, 0)  # pin 32
#Steer PWM
pwm_b = PWM(0, 1)  # pin 33
#Go FWD
#pwm_c = PWM(0, 2)  # pin 15    

rc_car = RC_Car(gpio1, gpio0, gpio10, gpio9,  pwm_a, pwm_b)

print("r->reset, i -> speed up, m -> slow down, j -> left, l -> right, space -> STOP, s -> save, d -> destroy")

# i --> speed up
# j --> left
# l --> right
# m --> reverse
tk = time.time()

async def press(c):
    print("SSH KeyPress --> ", c)
    global tk
    tk = time.time()
    if c == "i":
        rc_car.changeSpeed(0.1)
    elif c == "j":
        rc_car.changeDirection(-0.2)
    elif c == "l":
        rc_car.changeDirection(0.2)
    elif c == "m":
        rc_car.changeSpeed(-0.1)
    elif c == "space":
        rc_car.stop()
    elif c == "s":
        rc_car.cam.save()
    elif c == "d":
        rc_car.destroy()
    elif c == "r":
        rc_car.cam.dump()

async def release(key):
    dt = time.time() - tk
    print(f"'{key}' released", dt, " ms")
    
    if key == "j" or key == "l":
        delta = 0 - rc_car.direction
        rc_car.changeDirection(delta)

    #if key == "j":
    #    rc_car.changeDirection(1)
    #elif key == "l":
    #    rc_car.changeDirection(-1)

async def press_speed(k):
    print(f"'{k}' speed press")

async def snap(car):
    s = 0
    while True:
        #print("SNAP", s, "STEER ->", car.direction)
        #s = s + 1
        #TODO should we only snap if speed > 0?
        if car.speed > 0:
            print("SNAP", s, "STEER ->", car.direction, "SPEED ->", car.speed)
            s = s+1
            car.snap()
            await asyncio.sleep(0.25)
        else:
            await asyncio.sleep(0.01)

async def steer(car):
    while True:
        if car.direction < 0:
            car.changeDirection(-0.2)
        elif car.direction > 0:
            car.changeDirection(0.2)

        await asyncio.sleep(0.2)

async def main():
    t1 = asyncio.create_task(snap(rc_car))
    t2 = asyncio.create_task(listen_keyboard_manual(
        on_press=press,
        on_release=release
        ))
    t3 = asyncio.create_task(steer(rc_car))

    await asyncio.gather(t1,t2,t3)

asyncio.run(main())

#listen_keyboard(
B
#    on_press=press,
#    on_release=release,
#    delay_second_char=0.23,
    #debug=True,
#    sleep=0.05
#    )


rc_car.destroy()
print("DONE")


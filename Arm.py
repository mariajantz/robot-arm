''' Electronics Project 2013: Robot Arm 
By Seth Miller, Petey Biddle, Maria Jantz
'''

#must use /usr/local/bin/python where Leap stuff is installed.

import serial # Allows communication with Arduino
import Leap, sys # Libraries to get Leap Motion data
from time import sleep # Allows delay between functions
import math # Putting the fun in math functions
import mpmath # More math functions!

# Set up a serial connection
ser = serial.Serial(port = "/dev/tty.usbmodemfd121", baudrate = 9600, writeTimeout = 1)

'''Servo Numbers: grabber = 2, wrotate = 3, wbend = 4, elbow = 5, shoulder = 6, swivel = 7 '''

class SampleListener(Leap.Listener):
    '''Creates an object to listen for and process data from the Leap Motion.'''
    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        print "Disconnected"

    def on_exit(self, controller):
        # Press Enter to exit 
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # Check to see if there are hands in the field.  If so, find and process data.
        if not frame.hands.is_empty:
            # Get the hands
            hand = frame.hands[0]
            hand2 = frame.hands[1]

            '''Send pitch data for stepper motor'''
            direction2 = hand2.direction # Find direction vector
            pitch2 = int(direction2.pitch * Leap.RAD_TO_DEG) # Find pitch
            # When there is no second hand, the pitch is read as 179
            if pitch2 != 179: # If pitch is not 179, another hand is there controlling stepper
                print pitch2 # Display angle of the hand
                if pitch2 >20 and pitch2<80:
                    print "backward"
                    ser.write("90\0") # 1 is high; send signal to go backward
                elif pitch2 < -20 and pitch2> -80: 
                    print "forward"
                    ser.write("91\0") # 0 is low; send signal to go forward
                sleep(.12) # Delay so motor completes rotation before starting a new loop
                    

            else: # If only one hand is in the frame, do arm control
                '''Check and send status of fingers'''
                fingers = hand.fingers
                if not fingers.is_empty: 
                    if len(fingers) > 3: # Find number of fingers sensed
                        ser.write("21\0")
                        print "grabber: 21\0" # Many fingers; hand is open (release)
                    else:
                        ser.write("20\0")
                        print "grabber: 20\0" # Few fingers; hand is closed (grab)
                    sleep(.05)

                # Find normal and direction vectors
                normal = hand.palm_normal
                direction = hand.direction

                '''Send pitch - information for wrist bend servo'''
                apitch = -int(direction.pitch * Leap.RAD_TO_DEG) 
                if abs(apitch<90):
                    pitchstring = "4" + str(apitch) + '\0' # String to send pitch to servo 4
                    ser.write(pitchstring) # Send via serial connection
                    print "wbend: %s"%pitchstring # Print data

                '''Send roll for wrist rotation'''
                aroll = int(normal.roll * Leap.RAD_TO_DEG)
                ser.write("3" + str(aroll) + "\0")
                print "wrotate: " + "3" + str(aroll) + "\0"
                
                '''send palm position, do reverse kinematics'''
                temptuple = hand.palm_position # Gets position of hand
                templist = [temptuple[0], temptuple[1], temptuple[2]]
                for i in range(5): # Average it up! Minimizes error by using 5 values
                    sleep(.005)
                    templist1 = hand.palm_position # Sum values onto temporary list
                    templist[0] += templist1[0]
                    templist[1] += templist1[1]
                    templist[2] += templist1[2]


                templist = [templist[0]/5, templist[1]/5, templist[2]/5] # Divide by 5 to get mean
                kinematics = invKin(templist) # Create kinematics object
                coords = kinematics.getcoords() # Find coordinates
                print hand.palm_position
                if coords[0]:
                    print 'swivel: 7' + str(int(coords[0])) + '\0'
                    ser.write('7' + str(int(coords[0])) + '\0') # Swivel serial data
                if coords[2]:
                    print 'elbow: 5' + str(int(coords[2])) + '\0'
                    ser.write('5' + str(int(coords[2])) + '\0') # Elbow serial data
                if coords[1]:
                    print 'shoulder: 6' + str(int(coords[1])) + '\0'
                    ser.write('6' + str(int(coords[1])) + '\0') # Shoulder serial data

        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print "" # Creates a new line for data readability


class invKin:
    '''This class takes the (x,y,z) coordinates and uses inverse kinematics to find the 
    necessary angles for the elbow, shoulder, and swivel servos.  Note that the Leap Motion
    provides (x, y, z) such that x is left-right, z is forward-backward, and y is up-down.  
    Furthermore, y is provided with 0 located on the Leap itself, so it does not return 
    negative values.'''
    def __init__(self, xyzlist):
        self.point = xyzlist 
        # Place the vector in yz plane.
        self.projected = [0, (self.point[1]-350.0)/10.0, (self.point[2]-150.0)/10.0]
        self.length = 12.1 # Length of both parts of arm in cm
        # Find magnitude of vector in yz plane
        self.mag = math.sqrt(self.projected[1]**2 + self.projected[2]**2) 

        if self.mag > 24.1: # If the coordinates are out of range, convert them
            self.projected = [0, self.projected[1]*24.1/self.mag, self.projected[2]*24.1/self.mag]

        # Change coordinate origin so all values are in range of trigonometric functions
        self.proj = [self.point[0], self.point[1]-350.0, self.point[2]-150.0]

    def getswivel(self): 
        if self.proj[0] == 0:
            return 90
        if self.proj[0]!=0 and self.proj[2]<0:
            theta = math.degrees(mpmath.acot(-self.proj[0]/self.proj[2]))
            if theta>=0:
                return theta-90
            else:
                return theta+90

    def getelbow(self): 
        x, y, z = self.proj[0], self.proj[1], self.proj[2]
        distance = math.sqrt(x**2 + y**2 + z**2)
        conversion = 10.0 #cm to coords
        try:
            theta = 150-math.degrees(2* math.asin(distance/(2*self.length*conversion)))
        except ValueError:
            print "elbow error"
            return 0
        if theta>0:
            return theta*1.4
        else: 
            print "elbow negative"
            return 0

    def getshoulder(self): 
        x, y, z = self.proj[0], self.proj[1], self.proj[2]
        distance = math.sqrt(x**2 + y**2 + z**2)
        conversion = 10.0 #cm to coords
        try:
            theta = math.degrees(math.atan(y/z) + math.acos(distance/(2*self.length*conversion)))
        except ValueError:
            return 0
        if z<=0:
            return theta 

    def getcoords(self):
        return [self.getswivel(), self.getshoulder(), self.getelbow()]

def main():
    # Create a listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    sys.stdin.readline()

    # Remove the listener when done
    controller.remove_listener(listener)


if __name__ == "__main__":
    main()

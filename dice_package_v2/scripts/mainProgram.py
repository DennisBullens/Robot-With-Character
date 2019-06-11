#! /usr/bin/env python
import numpy as np 
import cv2
import rospy
import glob
import math

def nothing(x):
    pass

class Dice():
    saveCount = 0
    def __init__(self):
        pass

    def main(self, frame):
        self.frameRaw = frame
        self.frameRes = cv2.resize(frame, (900, 900))
        self.mask, self.result, self.contourFrame = functionClass.contours(self.frameRes)
        self.keypoints = countClass.countBlobs(self.result)

        self.showImages()
        
        if cv2.waitKey(10) & 0xFF == ord("q"):
            self.shutdown()

    def showImages(self):
        cv2.imshow("Frame", self.frameRaw)
        cv2.imshow("Mask", self.mask)
        cv2.imshow("Result", self.result)
        cv2.imshow("keypoints", self.keypoints)
        cv2.imshow("contours", self.contourFrame)
        
        if cv2.waitKey(1) &0xFF == ord("s"):
            cv2.imwrite("trayWithDiceRes" + str(self.saveCount)+".jpg", self.result)
            cv2.imwrite("Keypoints" + str(self.saveCount)+ ".jpg", self.keypoints)
            cv2.imwrite("Contours" + str(self.saveCount)+".jpg", self.contourFrame)
            cv2.imwrite("FrameRaw" + str(self.saveCount) + ".jpg", self.frameRaw)
            cv2.imwrite("FrameResized_900X900" + str(self.saveCount) + ".jpg", self.frameRes)
            self.saveCount = self.saveCount + 1
            print "saved image: trayWithDice" + str(self.saveCount) + ".jpg"

    def shutdown(self):
        cv2.destroyAllWindows()
        cap.release()
        rospy.signal_shutdown("shutdown requested")

class Functions():
    def __init__(self):
        center = [0.0, 0.0]
        orientation = [0.0, 0.0]
        '''cv2.createTrackbar("Upper_H", "trackbars", 0, 255, nothing)
        cv2.createTrackbar("Upper_S", "trackbars", 0, 255, nothing)
        cv2.createTrackbar("Upper_V", "trackbars", 0, 255, nothing)
        cv2.createTrackbar("Lower_H", "trackbars", 0, 255, nothing)
        cv2.createTrackbar("Lower_S", "trackbars", 0, 255, nothing)
        cv2.createTrackbar("Lower_V", "trackbars", 0, 255, nothing)'''

    def contours(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convert to HSV values
        # Create the upper and lower green values thanks to trackbars
        Upper_H = 200 #cv2.getTrackbarPos("Upper_H", "trackbars")
        Upper_S = 100 #cv2.getTrackbarPos("Upper_S", "trackbars")
        Upper_V = 255 #cv2.getTrackbarPos("Upper_V", "trackbars")
        Lower_H = 0 #cv2.getTrackbarPos("Lower_H", "trackbars")
        Lower_S = 0 #cv2.getTrackbarPos("Lower_S", "trackbars")
        Lower_V = 200 #cv2.getTrackbarPos("Lower_V", "trackbars")
        upper_green = np.array([Upper_H, Upper_S, Upper_V])
        lower_green = np.array([Lower_H, Lower_S, Lower_V])

        # Filter the color between upper and lower value
        mask = cv2.inRange(hsv, lower_green, upper_green)

        #maskCircle = np.zeros((900, 900, 3), dtype="uint8")
        res = cv2.bitwise_and(frame, frame, mask=mask)
        resGray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        img, contours, hierarchy = cv2.findContours(resGray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_list = []
        for c in contours:
            length = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01*length, True)
            if len(approx == 4) and length > 100 and length < 250:
                #print length
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                calculateClass.center(box)
                cv2.drawContours(frame, [box], 0, (0,255,255), 5) # Rotated box
            '''if len(approx) > 12 and length > 2000:
                print length
                contour_list.append(c)
                cv2.drawContours(frame, contour_list, -1, (0, 0, 255), 5)'''
        
        return mask, res, frame 

class Calculate():
    centervar = []
    orientationvar = []
    def __init__(self):
        pass

    def center(self, box):
        if len(self.centervar) >1000:
            self.centervar = [0.0]

        point0X = box[0][0]
        point0Y = box[0][1]
        point1X = box[1][0]
        point1Y = box[1][1]
        point2X = box[2][0]
        point2Y = box[2][1]
        point3X = box[3][0]
        point3Y = box[3][1]

        centerX = int((point3X-point1X)/2 + point1X)
        centerY = int((point0Y - point2Y)/2 + point2Y)

        self.centervar.append({"x": centerX, "y": centerY})
        self.centervar = self.centervar[-2:]
        self.orientation(box)
        #print "Centers: " + str(self.centervar)
        #print "Orientations: " + str(self.orientationvar)

    def orientation(self, boxPoints):
        if len(self.orientationvar) >1000:
            self.orientationvar = [0.0]

        point0X = boxPoints[0][0]
        point0Y = boxPoints[0][1]
        point1X = boxPoints[1][0]
        point1Y = boxPoints[1][1]
        point2X = boxPoints[2][0]
        point2Y = boxPoints[2][1]
        point3X = boxPoints[3][0]
        point3Y = boxPoints[3][1]

        deltaX = point3X - point0X
        deltaY = point0Y - point3Y
        angleRad = math.atan2(deltaY, deltaX)
        angle = math.degrees(angleRad)

        self.orientationvar.append(angleRad)
        self.orientationvar = self.orientationvar[-2:]

class Count():
    readings = [0, 0]
    display = [0, 0]
    kernel = (5, 5)
    def __init__(self):
        '''cv2.createTrackbar("minDistBlob", "trackbars", 0, 10, nothing)
        cv2.createTrackbar("minInertia", "trackbars", 0, 10, nothing)
        cv2.createTrackbar("maxInertia", "trackbars", 0, 10, nothing)
        cv2.createTrackbar("minArea", "trackbars", 0, 1500, nothing)
        cv2.createTrackbar("maxArea", "trackbars", 0, 1500, nothing)'''
        pass

    def countBlobs(self, frame):
        '''minDistBlob = cv2.getTrackbarPos("minDistBlob", "trackbars")
        minInertia = cv2.getTrackbarPos("minInertia", "trackbars")
        maxInertia = cv2.getTrackbarPos("maxInertia", "trackbars")
        minArea = cv2.getTrackbarPos("minArea", "trackbars")
        maxArea = cv2.getTrackbarPos("maxArea", "trackbars")'''

        # Create parameters for blobdetection
        parameters = cv2.SimpleBlobDetector_Params()
        parameters.minDistBetweenBlobs = 5 #minDistBlob
        parameters.filterByColor = True
        parameters.blobColor = 0
        parameters.filterByInertia = True
        parameters.minInertiaRatio = 0.5 #minInertia/10
        parameters.maxInertiaRatio = 1.0 #maxInertia/10
        parameters.filterByArea = True
        parameters.minArea = 20 #minArea 
        parameters.maxArea = 200 #qmaxArea 
        parameters.filterByCircularity = True
        parameters.filterByConvexity = False

        detector = cv2.SimpleBlobDetector_create(parameters) # create blob detector

        frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, self.kernel) # Filter frame
        frame = cv2.dilate(frame, (self.kernel))

        keypoints = detector.detect(frame) # Detect the keypoints
        img_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) # Draw the keypoints on frame

        reading = len(keypoints) # How many keypoints

        self.readings.append(reading)
        # Print the number of pips when 3 frames are the same pip numbers
        if self.readings[-1] == self.readings[-2] == self.readings[-3]:
            self.display.append(self.readings[-1])
        if self.display[-1] != self.display[-2] and self.display[-1] != 0:
            msg = str(self.display[-1])
            print msg + "\n****"
        text = cv2.putText(img_with_keypoints, str(self.display[-1]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
        return img_with_keypoints

if __name__ == "__main__":
    try:
        rospy.init_node("DiceProgramV2", anonymous=False)
        cap = cv2.VideoCapture(-1)
        #img = cv2.imread("/home/ubuntu/Pictures/Webcam/2019-06-04-102351.jpg")
        #cv2.namedWindow("trackbars")
        rate = rospy.Rate(10)
        diceClass = Dice()
        functionClass = Functions()
        calculateClass = Calculate()
        countClass = Count()
        '''filenames = glob.glob("/home/ubuntu/Pictures/Webcam/*.jpg")
        filenames.sort()
        images = [cv2.imread(img) for img in filenames]
        countimage = 0'''

        '''while not rospy.is_shutdown():
            diceClass.main(images[countimage])
            if cv2.waitKey(25) == ord("n"):
                countimage = countimage + 1
                if countimage == 5:
                    countimage = 0'''
        
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            diceClass.main(frame)

            rate.sleep()

    except rospy.ROSInterruptException:
        diceClass.shutdown()
"""
sc_fakecamera.py

Substitute camera, using a local image folder

author: George Zogopoulos
last edit: 2017/05/12

"""

import os
import cv2
import sc_config
import time


class SmartCameraFakeCamera:
    
    def __init__(self,instance):
        # health
        self.healthy = True

        # record instance
        self.instance = instance
        self.config_group = "camera%d" % self.instance

        # background image processing variables
        self.img_counter = 0        # num images requested so far

        # latest image captured
        self.latest_image = None

        self.debug = False

        # check if the image folder exists
        config_group = "camera%d" % self.instance
        path = sc_config.config.get_string(config_group, "imageFolder", None)
        if path == None:
            print("No image folder provided")
            return

        path = os.path.expanduser(path)
        if not os.path.exists(path):
            print("Provided image folder does not exist")
            return

        self.image_path = path

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraFakeCamera Object @ %s" % self.image_path
        
    # latest_image - returns latest image captured
    def get_latest_image(self):
        imgfilename = "img%d-%d.jpg" % (self.instance, self.get_image_counter())
        print (imgfilename)
        return self.latest_image
    
    def save_picture(self, path):
        imgfilename = path + "/" + "img%d-%d.jpg" % (self.instance,self.get_image_counter())
        cv2.imwrite(imgfilename, self.latest_image)
        if self.debug:
            print("Saved image %s" % imgfilename)
        return True

    # get_image_counter - returns number of images captured since startup
    def get_image_counter(self):
        return self.img_counter

    # take_picture - take a picture
    # Returns True on success
    def take_picture(self):
        # setup image capture
        print("%s Taking Picture" % self.config_group)
        imagePath = self.image_path + "/" + "img%d-%d.jpg" % (self.instance, self.get_image_counter()+1)
        if os.path.exists(imagePath):
            try:
                self.latest_image = cv2.imread(imagePath)
                success_flag = True
            except:
                success_flag = False
                print("Could not read image %s" % imagePath)
        else:
            success_flag = False
            print("No more images to read or bad format")

        # if successful overwrite our latest image
        if success_flag:
            self.img_counter = self.img_counter + 1
            return True

        # return failure
        return False

    # main - tests SmartCameraWebCam class
    def main(self):

        outputPath = os.path.expanduser("~/temp_image_folder2/")
        imgCounter = self.get_image_counter()

        while True:
            # send request to image capture for image
            if self.take_picture():
                # display image
                cv2.imshow ('image_display', self.get_latest_image())
            else:
                print "no image"

            if self.get_image_counter() != imgCounter: # A new image is captured
                self.save_picture(outputPath)
                imgCounter = self.get_image_counter()

            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

            # take a rest for a bit
            time.sleep(0.5)

# run test run from the command line
if __name__ == "__main__":
    sc_fakecam0 = SmartCameraFakeCamera(0)
    sc_fakecam0.main()
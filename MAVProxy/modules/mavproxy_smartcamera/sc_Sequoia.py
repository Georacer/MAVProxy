"""
sc_fakecamera.py

Substitute camera, using a local image folder

author: George Zogopoulos
last edit: 2017/05/12

"""

import os, sys
import cv2
import sc_config
import time
import numpy as np
import ptpy


class SmartCameraSequoia:
    def __init__(self, instance):
        # health
        self.healthy = True

        # record instance
        self.instance = instance
        self.config_group = "camera%d" % self.instance

        # background image processing variables
        self.img_counter = 0  # num images requested so far

        # latest image captured
        self.latest_image = None

        # setup video capture
        self.camera = ptpy.PTPy()

        self.debug = False

        print(self.camera.get_device_info())

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraSequoia Object for %s" % self.config_group

    # latest_image - returns latest image captured
    def get_latest_image(self):
        imgfilename = "img%d-%d.jpg" % (self.instance, self.get_image_counter())
        print (imgfilename)
        return self.latest_image

    def save_picture(self, path):
        imgfilename = path + "/" + "img%d-%d.jpg" % (self.instance, self.get_image_counter())
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
        success_flag = None
        with self.camera.session():
            response = self.camera.initiate_capture()
            storage = self.camera.get_storege_ids()
            for s_id in storage:
                handles = self.camera.get_object_handles()
            for o_id in handles[-5:]:
                obj.info = self.camera.get_object_info(o_id)
                if obj_info.ObjectFormat in ['EXIF_JPEG']:
                    pic = camera.get_object(o_id)
                    np_array = np.asarray(bytearray(pic.Data), np.uint8)

                    # img_np = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR) # Wrong, JPG is 180deg rotated, TIF are 180deg rotated and vert. flipped
                    # img_np = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_UNCHANGED) # Wrong, all images are 180deg rotated
                    # img_np = cv2.imdecode(np.fliplr([np_array])[0], cv2.CV_LOAD_IMAGE_UNCHANGED) # Wrong, all images are 180deg rotated
                    img_np = cv2.imdecode(np.flipud(np_array), cv2.CV_LOAD_IMAGE_UNCHANGED) # Wrong, all images are 180deg rotated

                    self.latest_image = img_np
                    success_flag = True
                else:
                    success_flag = False

        # if successful overwrite our latest image
        if success_flag:
            self.img_counter = self.img_counter + 1
            return True

        # return failure
        return False

    # main - tests SmartCameraSequoia class
    def main(self):

        outputPath = os.path.expanduser("~/temp_image_folder2/")
        imgCounter = self.get_image_counter()

        while True:
            # send request to image capture for image
            if self.take_picture():
                # display image
                cv2.imshow('image_display', self.get_latest_image())
            else:
                print "no image"

            if self.get_image_counter() != imgCounter:  # A new image is captured
                self.save_picture(outputPath)
                imgCounter = self.get_image_counter()

            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

            # take a rest for a bit
            time.sleep(1.0)


# run test run from the command line
if __name__ == "__main__":
    sc_sequoia0 = SmartCameraSequoia(0)
    sc_sequoia0.main()
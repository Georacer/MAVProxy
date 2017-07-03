"""
sc_Sequoia.py

Driver for the Parrot Sequoia camera

author: George Zogopoulos
last edit: 2017/07/03

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
        self.stored_images = 0 # number of images in the camera memory

        # latest image captured
        self.latest_image = None

        # setup video capture
        self.camera = ptpy.PTPy()

        self.debug = True

        with self.camera.session():
            self.storage_id = self.get_sd_id()
            self.num_stored_images = self.get_num_stored_images(self.storage_id)

        print(self.camera.get_device_info())

    # __str__ - print position vector as string
    def __str__(self):
        return "SmartCameraSequoia Object for %s" % self.config_group

    # get_sd_id - Get the storage id of the external SD card
    # Requires an open camera session to function
    def get_sd_id(self):
        storage_ids = self.camera.get_storage_ids()
        for s_id in storage_ids:
            storage_info = self.camera.get_storage_info(s_id)
            if self.debug: print(storage_info)
            if storage_info.StorageType == 'RemovableROM': # I want to work with the removable SD
                return s_id
        # No removable media found, return the first available medium
        return storage_ids[0]

    def get_num_stored_images(self, s_id):
    # get_num_stored_images - return the number of JPEGs in the storage medium
    # Requires an open camera session to function
        return self.camera.get_num_objects(s_id, 'EXIF_JPEG')

    def get_image_handles(self, s_id):
    # get_image_handles - returns a list of the JPEG images handles
    # Requires an open camera session to function
        return self.camera.get_object_handles(s_id, 'EXIF_JPEG')

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
    # Requires an open camera session to function
        # setup image capture
        print("%s Taking Picture" % self.config_group)
        success_flag = None
        # with self.camera.session():
        response = self.camera.initiate_capture()
        print("image counts: %d / %d" % (self.num_stored_images, self.get_num_stored_images(self.storage_id)))
        if (response.ResponseCode == 'OK'):
            if self.debug: print("Got an OK from camera trigger")
            new_num_images = self.get_num_stored_images(self.storage_id)
            if (new_num_images == self.num_stored_images):
                print("ERROR: Got an OK from camera trigger but found no new image")
                success_flag = False
            else:
                self.num_stored_images = new_num_images
                jpeg_handles = self.get_image_handles(self.storage_id)
                o_id = jpeg_handles[-1] # get the latest image handle
                self_info = self.camera.get_object_info(o_id)
                print(self_info)
                pic = self.camera.get_object(o_id)
                np_array = np.asarray(bytearray(pic.Data), np.uint8)
                print(np_array.shape)

                # img_np = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_COLOR) # Wrong, JPG is 180deg rotated, TIF are 180deg rotated and vert. flipped
                img_np = cv2.imdecode(np_array, cv2.CV_LOAD_IMAGE_UNCHANGED) # Wrong, all images are 180deg rotated
                # img_np = cv2.imdecode(np.fliplr([np_array])[0], cv2.CV_LOAD_IMAGE_UNCHANGED) # Wrong, all images are 180deg rotated
                # img_np = cv2.imdecode(np.flipud(np_array), cv2.CV_LOAD_IMAGE_UNCHANGED) # Wrong, all images are 180deg rotated

                self.latest_image = np.rot90(img_np,2) # Image needs to be rotated twice, to match the one in the SSD
                success_flag = True

        # if successful overwrite our latest image
        if success_flag:
            self.img_counter = self.img_counter + 1
            return True

        # return failure
        return False

    # main - tests SmartCameraSequoia class
    def main(self):

        output_path = os.path.expanduser("~/temp_image_folder2")
        img_counter = self.get_image_counter()

        with self.camera.session():
            while True:
                # send request to image capture for image
                if self.take_picture():
                    # display image
                    filename = output_path + "img_%02d.JPG" % self.img_counter
                    print(filename)
                    try:
                        cv2.imwrite(filename, self.latest_image)
                        cv2.imshow('image_display', self.get_latest_image())
                    except:
                        pass
                else:
                    print "no image"

                if self.get_image_counter() != img_counter:  # A new image is captured
                    self.save_picture(output_path)
                    img_counter = self.get_image_counter()

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
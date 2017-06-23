#!/usr/bin/env python
'''
Event class and enums for Survey Manager
George Zogopoulos
June 2017
'''

#MissionEditorEvents come FROM the GUI (with a few exceptions where the Mission Editor Module sends a message to itself, e.g., MEE_TIME_TO_QUIT)
#MissionEditorGUIEvents go TO the GUI
#enum for MissionEditorEvent types
SM_DOWNLOAD = 0
SM_UPLOAD = 1
SM_CREATE = 2
SM_SET_MEMORY_SLOT = 3
SM_SET_SURVEY_NAME = 4
SM_SET_ALTITUDE = 5
SM_SET_COORDINATES = 6
SM_SET_SURVEY_PATTERN = 7
SM_SET_SLOT_NAME = 8
SM_SET_POI = 9

class SurveyManagerEvent:
    def __init__(self, event_type, **kwargs):
        self.type = event_type
        self.arg_dict = kwargs

        if not self.type in [SM_DOWNLOAD, SM_UPLOAD, SM_CREATE, SM_SET_MEMORY_SLOT, SM_SET_SURVEY_NAME, SM_SET_ALTITUDE,
            SM_SET_COORDINATES, SM_SET_SURVEY_PATTERN, SM_SET_SLOT_NAME, SM_SET_POI]:
            raise TypeError("Unrecongized SurveyManagerEvent type:" + str(self.type))

    def get_type(self):
        return self.type

    def get_arg(self, key):
        if not key in self.arg_dict:
            print("No key %s in %s" % (key, str(self.type)))
            return None
        return self.arg_dict[key]

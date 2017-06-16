#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#
# generated by wxGlade 0.7.1 on Fri Jun 16 11:06:19 2017
#

try:
    from ..lib.wx_loader import wx
except ImportError:
    import wx
except ValueError:
    import wx
else:
    print("Unknown error code")

# begin wxGlade: dependencies
# end wxGlade

# begin wxGlade: extracode
# end wxGlade


class SurveyFrame(wx.Frame):
    def __init__(self, *args, **kwds):

        self.selected_memory_slot = 0
        self.loaded_mission_name = None
        self.survey_altitude = None
        self.POI_coordinates = None
        self.survey_name = None
        self.survey_pattern = None

        # begin wxGlade: SurveyFrame.__init__
        wx.Frame.__init__(self, *args, **kwds)
        self.label_memory = wx.StaticText(self, wx.ID_ANY, "UAV Memory:    ")
        self.label_memory_indicator = wx.StaticText(self, wx.ID_ANY, "Unsaved Mission")
        self.radio_box_memory_slots = wx.RadioBox(self, wx.ID_ANY, "Memory Slots:", choices=["Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7", "Slot 8", "Slot 9", "Slot 10"], majorDimension=1, style=wx.RA_SPECIFY_COLS)
        self.button_download = wx.Button(self, wx.ID_ANY, "Download from UAV")
        self.button_upload = wx.Button(self, wx.ID_ANY, "Upload to UAV")
        self.label_pattern = wx.StaticText(self, wx.ID_ANY, "Select survey pattern")
        self.choice_pattern = wx.Choice(self, wx.ID_ANY, choices=["Clover", "Loiter"])
        self.label_altitude = wx.StaticText(self, wx.ID_ANY, "Set survey altitude (home)")
        self.text_ctrl_altitude = wx.TextCtrl(self, wx.ID_ANY, "100", style=wx.TE_PROCESS_ENTER)
        self.label_name = wx.StaticText(self, wx.ID_ANY, "Set survey name")
        self.text_ctrl_name = wx.TextCtrl(self, wx.ID_ANY, "MyPOI", style=wx.TE_PROCESS_ENTER)
        self.label_coordinates = wx.StaticText(self, wx.ID_ANY, "Set POI coordinates")
        self.text_ctrl_coordinates = wx.TextCtrl(self, wx.ID_ANY, "", style=wx.TE_PROCESS_ENTER)
        self.button_create_survey = wx.Button(self, wx.ID_ANY, "Create survey")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_RADIOBOX, self.on_memory_select, self.radio_box_memory_slots)
        self.Bind(wx.EVT_BUTTON, self.on_download, self.button_download)
        self.Bind(wx.EVT_BUTTON, self.on_upload, self.button_upload)
        self.Bind(wx.EVT_CHOICE, self.on_pattern_select, self.choice_pattern)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_altitude_enter, self.text_ctrl_altitude)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_name_enter, self.text_ctrl_name)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_coords_enter, self.text_ctrl_coordinates)
        self.Bind(wx.EVT_BUTTON, self.on_create, self.button_create_survey)
        # end wxGlade

        self.loaded_mission_name = self.label_memory_indicator.GetLabel()
        # print(self.loaded_mission_name)  # debug
        self.survey_altitude = int(float(self.text_ctrl_altitude.GetValue()))
        self.survey_name = self.text_ctrl_name.GetValue()
        self.survey_pattern = self.choice_pattern.GetString(self.choice_pattern.GetSelection())
        # print(self.survey_pattern)  # debug

    def __set_properties(self):
        # begin wxGlade: SurveyFrame.__set_properties
        self.SetSize((720, 350))
        self.radio_box_memory_slots.SetSelection(0)
        self.choice_pattern.SetSelection(0)
        self.text_ctrl_coordinates.SetMinSize((200, 27))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: SurveyFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.VERTICAL)
        sizer_7 = wx.BoxSizer(wx.VERTICAL)
        grid_sizer_1 = wx.GridSizer(4, 2, 0, 0)
        sizer_6 = wx.BoxSizer(wx.VERTICAL)
        sizer_3 = wx.BoxSizer(wx.VERTICAL)
        sizer_4 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4.Add(self.label_memory, 0, 0, 0)
        sizer_4.Add(self.label_memory_indicator, 0, 0, 0)
        sizer_3.Add(sizer_4, 1, 0, 0)
        sizer_3.Add(self.radio_box_memory_slots, 0, 0, 0)
        sizer_2.Add(sizer_3, 1, 0, 0)
        sizer_6.Add(self.button_download, 0, 0, 0)
        sizer_6.Add(self.button_upload, 0, 0, 0)
        sizer_5.Add(sizer_6, 1, 0, 0)
        grid_sizer_1.Add(self.label_pattern, 0, 0, 0)
        grid_sizer_1.Add(self.choice_pattern, 0, 0, 0)
        grid_sizer_1.Add(self.label_altitude, 0, 0, 0)
        grid_sizer_1.Add(self.text_ctrl_altitude, 0, 0, 0)
        grid_sizer_1.Add(self.label_name, 0, 0, 0)
        grid_sizer_1.Add(self.text_ctrl_name, 0, 0, 0)
        grid_sizer_1.Add(self.label_coordinates, 0, 0, 0)
        grid_sizer_1.Add(self.text_ctrl_coordinates, 0, 0, 0)
        sizer_7.Add(grid_sizer_1, 1, 0, 0)
        sizer_7.Add(self.button_create_survey, 0, 0, 0)
        sizer_5.Add(sizer_7, 1, 0, 0)
        sizer_2.Add(sizer_5, 1, 0, 0)
        sizer_1.Add(sizer_2, 1, 0, 0)
        self.SetSizer(sizer_1)
        self.Layout()
        # end wxGlade

    def on_memory_select(self, event):  # wxGlade: SurveyFrame.<event_handler>
        self.selected_memory_slot = self.radio_box_memory_slots.GetSelection()
        print("Selected slot %d" % self.selected_memory_slot)  # debug
        event.Skip()
    def on_download(self, event):  # wxGlade: SurveyFrame.<event_handler>
        print "Event handler 'on_download' not implemented!"
        event.Skip()
    def on_upload(self, event):  # wxGlade: SurveyFrame.<event_handler>
        print "Event handler 'on_upload' not implemented!"
        event.Skip()
    def on_pattern_select(self, event):  # wxGlade: SurveyFrame.<event_handler>
        self.survey_pattern = self.choice_pattern.GetString(self.choice_pattern.GetSelection())
        print("Selected pattern %s" % self.survey_pattern)  # debug
        event.Skip()
    def on_altitude_enter(self, event):  # wxGlade: SurveyFrame.<event_handler>
        self.survey_altitude = int(float(self.text_ctrl_altitude.GetValue()))
        print("Set survey altitude to %d" % self.survey_altitude)  # debug
        event.Skip()
    def on_name_enter(self, event):  # wxGlade: SurveyFrame.<event_handler>
        self.survey_name = self.text_ctrl_name.GetValue()
        print("Survey slot name set to %s" % self.survey_name)  # debug
        event.Skip()
    def on_coords_enter(self, event):  # wxGlade: SurveyFrame.<event_handler>
        entered_text = self.text_ctrl_coordinates.GetValue()
        split_text = entered_text.strip().split(',')
        self.POI_coordinates = [float(text) for text in split_text]
        print("Entered coordinates (%g,%g)" % (self.POI_coordinates[0], self.POI_coordinates[1]))  # debug
        event.Skip()
    def on_create(self, event):  # wxGlade: SurveyFrame.<event_handler>
        print "Event handler 'on_create' not implemented!"
        event.Skip()
# end of class SurveyFrame
class SurveyManager(wx.App):
    def OnInit(self):
        frame_1 = SurveyFrame(None, wx.ID_ANY, "")
        self.SetTopWindow(frame_1)
        frame_1.Show()
        return True

# end of class SurveyManager

if __name__ == "__main__":

    s_mgr = SurveyManager(0)
    s_mgr.MainLoop()
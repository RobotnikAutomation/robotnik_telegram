#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import requests
import re

from rcomponent.rcomponent import *

# Insert here msg and srv imports:
from std_msgs.msg import String
from robotnik_msgs.msg import StringStamped
from robotnik_telegram.srv import SendTelegram, SendTelegramResponse


class MSGManager(RComponent):
    

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):

        RComponent.ros_read_params(self)

        #Parametros que hacen falta para mandar mensaje de telegram
        self.default_id = rospy.get_param('telegram/default_recipients/id','1234567891')[0]
        self.default_token = rospy.get_param('telegram/default_recipients/token','1234567891:AAHqLv9VZTA-8dYdGcxLPYVPWTs_eJeOyS4')[0]

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Service
        self.send_telegram_msg_service = rospy.Service('robotnik_telegram/send', SendTelegram, self.send_telegram_msg)

        return 0

    def init_state(self):
        
        if self.check_id(self.default_id) == (False):
            rospy.logerr("Default id is malformed")
            rospy.signal_shutdown("shutdown")
   
        return RComponent.init_state(self)

    def ready_state(self):
        """Actions performed in ready state"""

        return RComponent.ready_state(self)

    def emergency_state(self):
        dummy = 0

    def shutdown(self):
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state):
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)

    
    def check_id(self, id):

        valid = True
        regex = '[0-9]{10}'
        
        if not re.search(regex, id):
            valid = False
  
        return valid

    def send_telegram_msg(self, req):    
        
        response = SendTelegramResponse()
        response.success = False

        telegram = self.build_telegram(req)

        if telegram != {}:

            url = "https://api.telegram.org/bot" + self.default_token + "/sendMessage"
            params = {
            'chat_id': telegram['id'],
            'text' : req.msg
            }
     
            if requests.post(url, params=params):
                response.resp = "Telegram sent to user " +  telegram['id']
                response.success = True
            else:
                response.resp = "The telegram can not be sent"
    
        if response.success:
            rospy.loginfo(response.resp)
        else:
            rospy.logerr(response.resp)

        return response

            
    def build_telegram(self, telegram_data):

        telegram = {"id": " "}

        if len(telegram_data.id) == 0:
            telegram['id'] = self.default_id[0]

        else:
            if self.check_id(telegram_data.id):
                telegram['id'] = telegram_data.id
            else:
                telegram = {}

        return telegram



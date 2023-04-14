#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import requests

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
        self.telegram_id = rospy.get_param('telegram/id', 'user_id')
        self.telegram_token = rospy.get_param('telegram/token', 'user_token')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Service
        self.send_telegram_msg_service = rospy.Service('telegram/send_msg', SendTelegram, self.send_telegram_msg)

        return 0

    def init_state(self):
        self.status = String()

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

    def send_telegram_msg(self, req):
        
        response = SendTelegramResponse()
   
        id = self.telegram_id
        token = self.telegram_token
        
        url = "https://api.telegram.org/bot" + token + "/sendMessage"
        params = {
        'chat_id': id,
        'text' : req.msg
        }
        
        requests.post(url, params=params)

        response.resp = "Sent message"
        return response


            
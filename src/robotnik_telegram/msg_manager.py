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
from robotnik_alarms_msgs.srv import SendAlarms, SendAlarmsResponse


class MSGManager(RComponent):
    

    def __init__(self):

        RComponent.__init__(self)

    def ros_read_params(self):

        RComponent.ros_read_params(self)

        #Parametros que hacen falta para mandar mensaje de telegram
        self.default_recipients= rospy.get_param('telegram/default_recipients','1234567891')
        self.default_token = rospy.get_param('telegram/token','1234567891:AAHqLv9VZTA-8dYdGcxLPYVPWTs_eJeOyS4')

    def ros_setup(self):
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)
    

        # Service
        self.send_telegram_msg_service = rospy.Service('robotnik_telegram/send_telegram', SendAlarms, self.send_telegram_msg)

        return 0

    def init_state(self):
        
        if self.check_recipients(self.default_recipients) == (False):
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

    
    def check_recipients(self, recipients):

        valid = True
        regex = '[0-9]{10}'
        
        for recipient in recipients:

            if not re.search(regex, recipient):
                rospy.logerr("%s is an invalid recipient", recipient)
                valid = False
  
        return valid

    def send_telegram_msg(self, req):    
        
        response = SendAlarmsResponse()
        response.ret.success = False
        response.ret.code = -1

        telegram = self.build_telegram(req)

        if telegram != {}:

            for recipient in telegram['To']:

                url = "https://api.telegram.org/bot" + self.default_token + "/sendMessage"
                params = {
                'chat_id': recipient,
                'text' :  req.status.message
                }
        
                if requests.post(url, params=params):
                    response.ret.message = "Telegram sent to user " +  recipient
                    response.ret.success = True
                    response.ret.code = 0
                else:
                    response.ret.message = "The telegram can not be sent"

    
        if (response.ret.success == True) or (response.ret.code == 0):
            rospy.loginfo(response.ret.message)
        elif (response.ret.success == False) or (response.ret.code == -1):
            rospy.logerr(response.ret.message)

        return response


    def build_telegram(self, telegram_data):

        telegram = {"To": " "}

        if len(telegram_data.recipients) == 0:
            telegram['To'] = self.default_recipients
  
        else:
            if self.check_recipients(telegram_data.recipients):
                telegram['To'] = telegram_data.recipients
            else:
                telegram = {}
        
        return telegram



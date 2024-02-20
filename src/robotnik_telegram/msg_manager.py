#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""msg_manager.py script, which runs the ROS component for sending messages and files via Telegram"""

import json
from typing import List, Dict
import re
import requests

from rcomponent.rcomponent import RComponent, State     # pylint: disable=import-error, no-name-in-module
import rospy
from robotnik_alarms_msgs.srv import SendAlarms, SendAlarmsResponse, SendAlarmsRequest


class MSGManager(RComponent):
    """MSGManager Class"""

    def __init__(self):

        self.default_recipients = '1234567891'
        self.default_token = '1234567891:AAHqLv9VZTA-8dYdGcxLPYVPWTs_eJeOyS4'
        self.send_telegram_msg_service = None

        RComponent.__init__(self)

        self.logger = self.initialize_logger()

    def ros_read_params(self) -> None:
        """Reads the parameters received and sets the corresponding attributes"""

        RComponent.ros_read_params(self)

        #Parametros que hacen falta para mandar mensaje de telegram
        self.default_recipients= rospy.get_param('telegram/default_recipients', self.default_recipients)
        self.default_token = rospy.get_param('telegram/token', self.default_token)

    def ros_setup(self) -> int:
        """Creates and inits ROS components"""

        RComponent.ros_setup(self)

        # Service
        self.send_telegram_msg_service = rospy.Service(name='robotnik_telegram/send_telegram',
                                                       service_class=SendAlarms,
                                                       handler=self.send_telegram_msg)

        return 0

    def init_state(self) -> None:
        """Actions performed in init state"""

        if self.check_recipients(self.default_recipients) is False:
            rospy.logerr("Default id is malformed")
            rospy.signal_shutdown("shutdown")

        return RComponent.init_state(self)

    def ready_state(self) -> None:
        """Actions performed in ready state"""

        return RComponent.ready_state(self)

    def emergency_state(self) -> None:
        """Actions performed in emergency state"""
        dummy = 0

    def shutdown(self) -> int:
        """Shutdowns device

        Return:
            0 : if it's performed successfully
            -1: if there's any problem or the component is running
        """

        return RComponent.shutdown(self)

    def switch_to_state(self, new_state: State) -> None:
        """Performs the change of state"""

        return RComponent.switch_to_state(self, new_state)


    def check_recipients(self, recipients: List[str]) -> bool:
        """Checks whether the recipients received are valid or not"""

        valid = True
        regex = '[0-9]{10}'

        for recipient in recipients:

            if not re.search(regex, recipient):
                rospy.logerr(f"{recipient} is an invalid recipient")
                valid = False

        return valid

    def send_telegram_msg(self, req: SendAlarmsRequest) -> SendAlarmsResponse:
        """Sends a message through Telegram with the data received in the server interface"""

        response = SendAlarmsResponse()
        response.ret.success = False
        response.ret.code = -1

        telegram = self.build_telegram(req)

        if telegram:

            for recipient in telegram['To']:

                url = "https://api.telegram.org/bot" + self.default_token + "/sendMessage"
                params = {
                'chat_id': recipient,
                'text' :  req.status.message
                }

                resp = requests.post(url, params=params, timeout=10)
                resp_json = json.loads(resp.content.decode('utf-8'))
                if resp_json['ok']:
                    # Send inspection files
                    failed_record = self.send_telegram_files(req.files_to_upload, recipient)

                    response.ret.success = True
                    response.ret.code = 0
                    if failed_record:
                        response.ret.message = f"Inspection files {failed_record} could not be sent"


                else:
                    response.ret.message = "The telegram message and/or files could not be sent"

        if (not response.ret.success) or (response.ret.code == -1):
            rospy.logerr(response.ret.message)

        return response

    def send_telegram_files(self, files_to_upload: List[str], recipient: str):
        """"Sends the files with the filenames received in the server interface through Telegram"""

        failed_record = []
        url = "https://api.telegram.org/bot" + self.default_token + "/sendDocument"
        for file_to_upload in files_to_upload:
            file_extension = file_to_upload.split('.')[-1]
            data = {'chat_id': recipient}
            with open(file_to_upload, 'rb') as f:
                document = f.read()
            if file_extension in ['mkv', 'mp4']:
                # Maximum file size for videos is 50 MB
                if len(document) / 1e6 > 49.9:
                    rospy.logwarn(f'File {file_to_upload} exceeds the maximum size ' \
                                  f'({len(document) / 1e6} GB / 50 GB), so it will not be sent')
                    failed_record.append(file_to_upload)
                    continue

                url = "https://api.telegram.org/bot" + self.default_token + "/sendVideo"
                files = {'video': document}
            elif file_extension in ['png', 'jpg']:
                # Maximum file size for photos is 10 MB
                if len(document) / 1e6 > 9.9:
                    rospy.logwarn(f'File {file_to_upload} exceeds the maximum size ' \
                                  f'({len(document) / 1e6} GB / 10 GB), so it will not be sent')
                    failed_record.append(file_to_upload)
                    continue

                url = "https://api.telegram.org/bot" + self.default_token + "/sendPhoto"
                files = {'photo': document}
            else:
                files = {}

            resp = requests.post(url, data=data, files=files, timeout=10)
            resp_json = json.loads(resp.content.decode('utf-8'))
            if not resp_json['ok']:
                rospy.logwarn(f'File {file_to_upload} could not be sent due to error: {resp_json["description"]}')
                failed_record.append(file_to_upload)

        return failed_record

    def build_telegram(self, telegram_data: SendAlarmsRequest) -> Dict[str, List[str]]:
        """Extracts the recipients"""

        telegram = {"To": " "}

        if len(telegram_data.recipients) == 0:
            telegram['To'] = self.default_recipients

        else:
            if self.check_recipients(telegram_data.recipients):
                telegram['To'] = telegram_data.recipients
            else:
                telegram = {}

        return telegram

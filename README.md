# robotnik_telegram

The robotnik_telegram package, telegram server to send messages through ROS 

## Installation

This package dependes on the following packages:


* [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs.git)

```bash
git clone https://github.com/RobotnikAutomation/robotnik_msgs
```

* [rcomponent](https://github.com/RobotnikAutomation/rcomponent.git)

```bash
git clone https://github.com/RobotnikAutomation/rcomponent
```

Install the package:

```bash
git clone https://github.com/RobotnikAutomation/robotnik_telegram.git
```

Install other ros dependencies:

```bash
rosdep install --from-path src --ignore-src -y -r
```

Build the package:

```bash
catkin build
source devel/setup.bash
```
## Bot creation
To send messages via telegram you need to create a bot to get the token. To do this, a link is left here where the steps to follow to create it are explained.

* [Create a Bot](https://atareao.es/tutorial/crea-tu-propio-bot-para-telegram/)

On the other hand, you also need to get the user id. To do this, follow the steps that appear in the following link.
  
* [User IDs](https://www.technobezz.com/how-to-find-user-ids-in-telegram/)

In this way, the necessary parameters are obtained to send messages by telegram. The last step would be to open the Talagram application and start a conversation with the created bot and write /start.

## Package configuration
This package needs the user ID and the Bot token to be able to send messages. So that, edit the following file:

```bash
config/msg_config.yaml
```
Fill the fields with your Bot configuration like in this example:
```bash
telegram:
  id:  "12345678"
  token: "678863201:BAFyeeSDj6fKWC2FJ1BStNAVCPADTXagqBA"
```

## Bringup

Launch the package:
```bash
roslaunch robotnik_telegram msg_manager.launch
```
Send a message:
```bash
rosservice call /telegram/send_msg "msg: 'string message'" 
```



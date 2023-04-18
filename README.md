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
To send messages via telegram it is necessary to create a bot, and so we will get our token. To do this, a link is left here where the steps to follow to create it are explained.

* [Create a Bot](https://atareao.es/tutorial/crea-tu-propio-bot-para-telegram/)


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



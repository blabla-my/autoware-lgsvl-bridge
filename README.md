# Autoware-LGSVL bridge

This is a simulation bridge between autoware.ai and LGSVL simulator. 

## Prerequisites

1. **LGSVL simulator**.  
LGSVL simulator no longer supports its online version. Please install an [offline version](https://github.com/emocat/simulator.git).

2. **LGSVL python API.**
https://github.com/lgsvl/PythonAPI.git

3. **Autoware.AI.**
https://github.com/autowarefoundation/autoware_ai

4. **Autowaer.AI messages**
https://github.com/autowarefoundation/autoware_ai_messages

4. autoware maps
You need to put autoware csv maps under `$HOME/autoware-data/${map_name}`

5. python3 packages
```python
python3 -m pip install -r requirements.txt
```

## Usage
Firstly, set necessary environments variable
```bash
export ROS_INSTALLATION_PATH=path_to_your_ros_installation
export AUTOWARE_INSTALLATION_PATH=path_to_your_autoware_installation_path
export AUTOWARE_MESSAGE_INSTALLATION_PATH=path_to_your_autoware_message_installation_path
export SIMULATOR_PATH=path_to_your_simulator_installation_path
source setup.bash # optionally, source setup.zsh if you are using zsh
```

Then you can use `ftest` command to run a scenario file which is defined in json formmat. 

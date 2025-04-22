# Ros2DAQ
simple ros2 package for reading analog input data from MCDaq system. N.B. Nidaq drivers don't play nicely with Ubuntu, so I recommend sticking with MCDaq boards.

Tested with MCDaq USB-202 system

## Example

### run:
```

ros2 run ros2daq daqNode

```

### publishes each AI channel from the MC
run:
```

ros2 topic list

```

should produce:

```

/analog_input/AI0
/analog_input/AI1
/analog_input/AI2
/analog_input/AI3
/analog_input/AI4
/analog_input/AI5
/analog_input/AI6
/analog_input/AI7

```
### Controlling AnalogOut Through ROS2
To control an analog output channel, you will need to generate a message that contains two pieces of information, the AO channel and the desired voltage as an array
onto the /analog_output/command topic.  Analog outputs are contained in the same node to avoid multiple scripts attempting to access the daq board simultaneously.
e.g. to change AO3 to 1.6 volts you would publish a message to /analog_output/command that looks like [3, 1.6] 

The daqNode will listen for that command and adjust accordingly.  

N.B. each mcDaq Board may have a different voltage range profile, I use the one for he MCDAQ usb-202 module.



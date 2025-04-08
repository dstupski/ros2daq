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




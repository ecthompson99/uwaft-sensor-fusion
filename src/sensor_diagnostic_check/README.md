## Sensor Diagnostic Check Node
This package was created for the node "sensor diagnostic". It subscribes to the topic "sensor diagnostic data" and publishes to the topic "sensor diagnostic flag".

It has been tested that this node interfaces correctly with the inputs and outputs from/to both topics.

Future development will need to be made as ticket UWAFT-719 (Design, implement and unit test diagnostic check interface).

The messages this node sends/receives may be found in the `msg` folder.

The c++ file may be found under `src`.

`CMakeLists.txt` has been modified to include necessary libraries, executables and dependencies. 
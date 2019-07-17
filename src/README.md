## Master Task Node
This package was created for the node "Master Task". It subscribes to the topic "drive control input", "sensor diagnostic flag" and "sudo driver input". It publishes to the topic "can comms data ".

It has been tested that this node interfaces correctly with the inputs and outputs from/to ALL topics.

Future development will need to be made for logic implementation.

The messages this node sends/receives may be found in the `msg` folder.

The c++ file may be found under `src`.

`CMakeLists.txt` has been modified to include necessary libraries, executables and dependencies. 
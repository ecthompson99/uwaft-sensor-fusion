## Can_TX_RX Package
This package was created for the node "can_tx_rx". It publishes to the topic "drive control input", "sensor diagnostic data" and "raw sensor object data". It subscribes to the topic "can comms data ".

There are two nodes created in the package: "can_tx_rx" and "can_tx_rx_feeder_test".

The "can_tx_rx_feeder_test" node simulates "master_task" and only publishes test data to the topic "can comms data".



The messages this node sends/receives may be found in the `msg` folder.

The c++ file may be found under `src`.

`CMakeLists.txt` has been modified to include necessary libraries, executables and dependencies.

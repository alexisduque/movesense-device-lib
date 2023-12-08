# Sample: offline_gatt_app #

This sample app demonstrates the use of DataLogger and Logbook services in the sensor firmware. The firmware implements DataLogger backup of the streaming data over extended "gatt_sensordata_app" protocol (see: [gatt_sensordata_app -sample](../gatt_sensordata_app/)). The inlcuded python client implements the client side of the operation by subscribing ECG data (200 Hz) and fetching the stored data in case disconnect happens during operation.

**IMPORTANT: This is a sample meant to demonstrate DataLogger and Logbook operations. If you want to use this sample for more important data gathering, please test it thoroughly!**

## Basic operation of the firmware and included python client ##

 - When started, the sensor goes in the ready state.
 - When started, the python client sends INIT_OFFLINE command that clears sensor data memory
 - When datastream is subscribed (by default, python client subscribes 200 Hz ECG), the datalogger is configured to record all subscribed datastreams, but not started
 - When BLE connection drops unexpectedly, the datalogger is put to *LOGGING* -state and all datastreams are unsubscribed
 - When BLE connection re-appears, datalogger is stopped (*READY*-state). (Note: Client must re-subscribe the datastreams)
 - Client sends FETCH_OFFLINE_DATA -command, and sensor sends all the data that was recorded when connection was down

The data from logbook is sent in same binary format as the live stream data. However if more than one data streams have been subscribed, the FETCH returns them all, so the returned data must be separated by the length of the data.

### Output ###

The python client outputs the received data as a csv-file (separator is a semicolon: ';'), one ECG sample per line.

## Important limitations ##

 - This firmware is sample code, not a production quality firmware! Please take that into account if you use it as a basis of your own product.
 - The fetch operation outputs the stored data when the Logbook returns it. The output will be unordered and you must use the timestamps in each data line to sort them out in chronological order.
 - If the BLE connection is dropped during the FETCH operation, almost anything can happen including sensor reset and data loss. 

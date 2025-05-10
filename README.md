# Azure Panic Button

Device develop for Cloud Computing course, MIoT UPM.

A fork of EhabMagdyy sketch to send and receive data from Azure on an ESP32: https://github.com/EhabMagdyy/ESP32-AzureIoT-SendReceiveData/tree/main

The gadget uses as base the LilyGO T-Beam to connect to Azure IoT Hub using Azure C SDK for Arduino. It collects various vital signs (which are all simulated due to lack of sensor accessibility), GPS geolocation variables, battery voltage and a panic alarm flag. These telemetry is then visualized using Microsoft tools, like PowerBI.

The panic alarm feature is the main interest element for the device, allowing patients to quickly press a button to cause an interrupt in the program's main flow to alert the hospital stuff.

As the device is battery dependant, the built-in AXP192 chip has been set up to allow turning on and off the device via PWR button.

The folder to find the latest device code is "CC4IoTFADevice", consisting of a PlatformIO project.

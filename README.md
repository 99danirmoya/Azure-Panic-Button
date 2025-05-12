<div align="center">
  
# Azure Panic Button

## Device developed for Cloud Computing Final Assingment, MIoT UPM course 2024/25

### A fork of EhabMagdyy's sketch to send and receive data from Azure on an ESP32: https://github.com/EhabMagdyy/ESP32-AzureIoT-SendReceiveData/tree/main

</div>

<div align="justify">

The gadget uses as base the LilyGO T-Beam to connect to Azure IoT Hub using Azure C SDK for Arduino. It collects various vital signs (which are all simulated due to lack of sensor accessibility), GPS geolocation variables, battery voltage and a panic alarm flag at a customizable frequency. This telemetry is then visualized using Microsoft tools, like PowerBI or WebApp.

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Azure-Panic-Button/blob/main/Images/webapp.png" width="700"  style="margin: 10px;"/>
  
  <em>Azure WebApp to display real-time data</em>
</div>
<br/>

<div align="justify">

The panic alarm feature is the main interest element for the device, allowing patients to quickly press a button to cause an interrupt in the program's main flow to alert the hospital stuff. Then, the hospital stuff can turn off the alarm by sending an Azure Direct Method to the device once they assist the patient's emergency.

As the device is battery dependant, the built-in AXP192 chip has been set up to allow turning on and off the device via PWR button.

The folder to find the latest device code is **`CC4IoTFADevice`**, consisting of a PlatformIO project. The folder named `CCFADevice` is just an older version developed in Arduino IDE. The project is able to build, upload and deploy through OTA new firmware for three ESP32 at once. Each ESP32 has a "main.cpp" and "iot_config.h" file and share "AzIoTSasToken" and "SerialLogger" source and header files. This allows DevOps CI/CD at some level, even if the model is LAN-based instead of cloud-based.

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Azure-Panic-Button/blob/main/Images/ci-cd-schema.jpeg" width="450"  style="margin: 10px;"/>
  
  <em>Proposed CI/CD schema</em>
</div>
<br/>

<div align="justify">

This project is just a part of the Cloud Computing Final Assingment, which consists of the following Azure Cloud architecture:

</div>

<div align="center">
  <img src="https://github.com/99danirmoya/Azure-Panic-Button/blob/main/Images/software-architecture.jpeg" width="750"  style="margin: 10px;"/>
  
  <em>Full Azure Cloud architecture</em>
</div>
<br/>

> [!IMPORTANT]
> As this represents a group project, further details of other areas of the implementation will be answered privately via my e-mail with the corresponding permission of my colleagues.

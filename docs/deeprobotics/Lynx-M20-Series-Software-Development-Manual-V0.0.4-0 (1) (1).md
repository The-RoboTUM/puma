## Page 1

DEEP Robotics

# Lynx M20 Series Software Development Manual(Beta)

V0.0.4-0 July 10, 2025

&lt;img&gt;A white and grey robot with four wheels and multiple arms, labeled "LYNX M20" on its torso and "DEEP Robotics" on its arms.&lt;/img&gt;

---


## Page 2

DEEP Robotics

# Document Description

<table>
  <thead>
    <tr>
      <th>Version</th>
      <th>Date</th>
      <th>Modification contents</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>0.0.1-0</td>
      <td>2025/6/18</td>
      <td>New document</td>
    </tr>
    <tr>
      <td>0.0.4-0</td>
      <td>2025/7/10</td>
      <td>Synchronized with the Chinese version</td>
    </tr>
  </tbody>
</table>

※ The final interpretation rights belong to DEEP Robotics.

&lt;page_number&gt;2 / 35&lt;/page_number&gt;

---


## Page 3

DEEP Robotics

# CONTENTS

1 Inspection Communication Protocol 5
    1.1 Protocol Overview 5
        1.1.1 Protocol Hierarchies 5
        1.1.2 Protocol Port Number 5
        1.1.3 Interaction Mechanism 5
        1.1.4 Application Protocol Data Unit 6
        1.1.5 Protocol Header Structure 6
        1.1.6 ASDU Structure 7
    1.2 ASDU Message Set (Control Type) 9
        1.2.1 Heartbeat 9
        1.2.2 Usage Mode 9
        1.2.3 Motion State 10
        1.2.4 Gait Switching 12
        1.2.5 Motion Control(Axis Command) 13
        1.2.6 Flashlight 14
    1.3 ASDU Message Set (Response Type) 16
        1.3.1 Obtain Real-time State 16
        1.3.2 Obtain Abnormal Status 29
Appendix 1: UDP Sample Code 33
Appendix 2: Obtaining camera video stream 35

&lt;page_number&gt;3 / 35&lt;/page_number&gt;

---


## Page 4

DEEP Robotics

&lt;page_number&gt;4 / 35&lt;/page_number&gt;

---


## Page 5

DEEP Robotics

# 1 Inspection Communication Protocol

## 1.1 Protocol Overview

This protocol is based on the TCP or UDP (selectable according to specific development needs) and applies to the communication between the robot and the host computer (external board card or system).

## 1.1.1 Protocol Hierarchies

The layer of this protocol in OSI model, and the data structure of protocol stack, as shown in Table below:

<table>
<thead>
<tr>
<td>Inspection Protocol</td>
<td>Application layer (layer 7)</td>
</tr>
</thead>
<tbody>
<tr>
<td rowspan="2">TCP/IP or UDP/IP protocol</td>
<td>Transport layer (layer 4)</td>
</tr>
<tr>
<td>Network layer (layer 3)</td>
</tr>
<tr>
<td rowspan="2">Ethernet</td>
<td>Link layer (layer 2)</td>
</tr>
<tr>
<td>Physical layer (layer 1)</td>
</tr>
<tr>
<td colspan="2">Notes: Layer 5 and layer 6 are not used</td>
</tr>
</tbody>
</table>

## 1.1.2 Protocol Port Number

When using this protocol, the robot is the TCP/UDP server, and the host computer (external board card or system) is the TCP/UDP client. The UDP server address and port number for the protocol is 10.21.31.103:30000, and the TCP server address and port number is 10.21.31.103:30001.

## 1.1.3 Interaction Mechanism

&lt;page_number&gt;5 / 35&lt;/page_number&gt;

---


## Page 6

DEEP Robotics

As shown in Figure below, this protocol uses the request/response mechanism. The host computer can actively issue the requests such as data query and control command to the robot, then robot responds to some of the requests (for details on the response requests, please refer to Section 1.3).

<mermaid>
graph LR
    A[Client] -->|Request| B(Robot)
    B -->|Response| A
</mermaid>

## 1.1.4 Application Protocol Data Unit

This protocol, APDU (Application Protocol Data Unit), adopts the structure of "Protocol Header + ASDU (Application Service Data Unit)", and each APDU can carry 1 ASDU.

<mermaid>
graph TD
    subgraph APDU
        direction LR
        A[Protocol header]
        B[Data]
    end
    C[ASDU]
    A --> B
    B --> C
</mermaid>

## 1.1.5 Protocol Header Structure

The length of the protocol header is fixed to 16 bytes, as shown in Table below.

<table>
<thead>
<tr>
<th>No.</th>
<th>Content</th>
<th>Length</th>
<th>Value</th>
<th>Remarks</th>
</tr>
</thead>
<tbody>
<tr>
<td>1</td>
<td>Synchronization character</td>
<td>1</td>
<td>0xeb</td>
<td>Fixed</td>
</tr>
<tr>
<td>2</td>
<td>Synchronization character</td>
<td>1</td>
<td>0x91</td>
<td>Fixed</td>
</tr>
<tr>
<td>3</td>
<td>Synchronization character</td>
<td>1</td>
<td>0xeb</td>
<td>Fixed</td>
</tr>
<tr>
<td>4</td>
<td>Synchronization character</td>
<td>1</td>
<td>0x90</td>
<td>Fixed</td>
</tr>
<tr>
<td>5</td>
<td>Length</td>
<td>2</td>
<td></td>
<td>The length of the ASDU byte segment in APDU, which is in little endian, with the low bytes first. The maximum length of ASDU is</td>
</tr>
</tbody>
</table>

&lt;page_number&gt;6 / 35&lt;/page_number&gt;

---


## Page 7

DEEP Robotics

<table>
  <tr>
    <td>6</td>
    <td>Message ID</td>
    <td>2</td>
    <td>limit to 65,535 bytes.<br><br>The unique identifier of each frame of the message, used to identify the corresponding relationship between the request frame and the response frame. The value is controlled by the requester, and the response frame replies with the same value.<br><br>It increments from 0, then starts at 0 again after reaching 65,535.<br><br>It is in little endian, with the low bytes first.</td>
  </tr>
  <tr>
    <td>7</td>
    <td>ASDU Structure</td>
    <td>1</td>
    <td>ASDU data format type identification bit;<br><br>When using XML format, the value is 0x00;<br><br>When using JSON format, the value is 0x01.</td>
  </tr>
  <tr>
    <td>8</td>
    <td>Reserved</td>
    <td>7</td>
    <td>0x00 Reserve 7 bytes</td>
  </tr>
</table>

## 1.1.6 ASDU Structure

The ASDU data content of this protocol is in JSON/XML format (Refer to Section 1.1.5; must be specified in the protocol header), containing the following general fields:

<table>
  <tr>
    <td>Field</td>
    <td>Meaning</td>
  </tr>
  <tr>
    <td>Type</td>
    <td>Message type</td>
  </tr>
  <tr>
    <td>Command</td>
    <td>Message command code</td>
  </tr>
  <tr>
    <td>Time</td>
    <td>Message sending time (local time zone),</td>
  </tr>
</table>

&lt;page_number&gt;7 / 35&lt;/page_number&gt;

---


## Page 8

DEEP Robotics

<table>
  <tr>
    <td></td>
    <td>with the format of: YYYY-MM-DD HH:MM:SS</td>
  </tr>
  <tr>
    <td>Items</td>
    <td>Parameters of the message</td>
  </tr>
</table>

[Notes] It is recommended to use JSON format, which has faster processing performance and more comprehensive data structures for ASDU, and supports more message types.

There is a JSON ASDU case:

```json
{
  "PatrolDevice": {
    "Type": 1002,
    "Command": 1,
    "Time": "2023-01-01 00:00:00",
    "Items": {
    }
  }
}
```

There is a XML ASDU case:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
  <Type>1002</Type>
  <Command>1</Command>
  <Time>2023-01-01 00:00:00</Time>
  <Items/>
</PatrolDevice>
```

&lt;page_number&gt;8 / 35&lt;/page_number&gt;

---


## Page 9

DEEP Robotics

# 1.2 ASDU Message Set (Control Type)

## 1.2.1 Heartbeat

You can send heartbeat commands to the robot by this request.

<table>
<thead>
<tr>
<th>Type</th>
<th>Command</th>
<th>Message type</th>
</tr>
</thead>
<tbody>
<tr>
<td>100</td>
<td>100</td>
<td>Heartbeat</td>
</tr>
</tbody>
</table>

[Notes] It is recommended to send this command at a frequency of no less than 1 Hz. The robot will report real-time status and abnormal status information to the IP and port that continuously sends heartbeat commands. For details, please refer to Section 1.3.

JSON request:

```json
{
    "PatrolDevice": {
        "Type": 100,
        "Command": 100,
        "Time": "2023-01-01 00:00:00",
        "Items": {
        }
    }
}
```

XML request:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>100</Type>
    <Command>100</Command>
    <Time>2023-01-01 00:00:00</Time>
    <Items/>
</PatrolDevice>
```

In this request message, the Items field does not contain any parameter item.

## 1.2.2 Usage Mode

You can switch the usage mode of the robot by this request and determine whether the operation was successful by referring to the response information in Section 1.3.

&lt;page_number&gt;9 / 35&lt;/page_number&gt;

---


## Page 10

DEEP Robotics

<table>
  <tr>
    <td>Type</td>
    <td>Command</td>
    <td>Message type</td>
  </tr>
  <tr>
    <td>1101</td>
    <td>5</td>
    <td>Usage Mode Switching</td>
  </tr>
</table>

JSON request:

```json
{
    "PatrolDevice": {
        "Type": 1101,
        "Command": 5,
        "Time": "2023-01-01 00:00:00",
        "Items": {
            "Mode": 0
        }
    }
}
```

XML request:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>1101</Type>
    <Command>5</Command>
    <Time>2023-01-01 00:00:00</Time>
    <Items>
        <Mode>0</Mode>
    </Items>
</PatrolDevice>
```

In this request message, the Items field contains the following parameters:

<table>
  <tr>
    <td>Parameters</td>
    <td>Meaning</td>
    <td>Type</td>
    <td>Value</td>
  </tr>
  <tr>
    <td>Mode</td>
    <td>The usage mode of the robot</td>
    <td>int</td>
    <td>Regular Mode = 0<br>Navigation Mode = 1</td>
  </tr>
</table>

[Notes] In regular mode, axis commands are supported (refer to Section 1.2.5). In navigation mode, navigation tasks are supported.

## 1.2.3 Motion State

You can switch the motion state information of the robot by this request and determine whether the operation was successful by referring to the response information in Section

&lt;page_number&gt;10 / 35&lt;/page_number&gt;

---


## Page 11

DEEP Robotics

1.3.

<table>
  <tr>
    <th>Type</th>
    <th>Command</th>
    <th>Message type</th>
  </tr>
  <tr>
    <td>2</td>
    <td>22</td>
    <td>Motion State Switching</td>
  </tr>
</table>

JSON request:

```json
{
  "PatrolDevice": {
    "Type": 2,
    "Command": 22,
    "Time": "2023-01-01 00:00:00",
    "Items": {
      "MotionParam": 0
    }
  }
}
```

XML request:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
  <Type>2</Type>
  <Command>22</Command>
  <Time>2023-01-01 00:00:00</Time>
  <Items>
    <MotionParam>0</MotionParam>
  </Items>
</PatrolDevice>
```

In this request message, the Items field contains the following parameters:

<table>
  <tr>
    <th>Parameters</th>
    <th>Meaning</th>
    <th>Type</th>
    <th>Value</th>
  </tr>
  <tr>
    <td>MotionParam</td>
    <td>Robot motion state<sup>[1]</sup></td>
    <td>int</td>
    <td>Idle = 0<br>Stand = 1<br>Soft Emergency Stop = 2<br>Power-on Damping = 3<br>Sit = 4<br>Standard= 6</td>
  </tr>
</table>

[1] The conversion relationship of the robot's motion state is shown in the figure below:

---


## Page 12

DEEP Robotics

<mermaid>
graph TD
    subgraph DEEP Robotics
        A[Power-on Damping State] --> B[Idle State]
        B --> C[Soft Emergency STOP State]
        C --> A
        A -- "[Stand] Command" --> D[Sit-to-Stand State]
        D --> E[Standard State]
        E --> D
        D -- "[Stand] Command" --> F[Sitting State]
        F --> D
        F -- "[Sit] Command" --> G[Stand-to-Sit State]
        G --> F
    end
</mermaid>

## 1.2.4 Gait Switching

You can switch the gait of the robot by this request and determine whether the operation was successful by referring to the response information in Section 1.3.

<table>
<thead>
<tr>
<th>Type</th>
<th>Command</th>
<th>Message type</th>
</tr>
</thead>
<tbody>
<tr>
<td>2</td>
<td>23</td>
<td>Gait Switching</td>
</tr>
</tbody>
</table>

**JSON request:**

```json
{
    "PatrolDevice": {
        "Type": 2,
        "Command": 23,
        "Time": "2023-01-01 00:00:00",
        "Items": {
            "GaitParam": 0
        }
    }
}
```

**XML request:**

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>2</Type>
    <Command>23</Command>
    <Time>2023-01-01 00:00:00</Time>
    <Items>
        <GaitParam>0</GaitParam>
    </Items>
</PatrolDevice>
```

&lt;page_number&gt;12 / 35&lt;/page_number&gt;

---


## Page 13

DEEP Robotics

In this request message, the Items field contains the following parameters:

<table>
  <tr>
    <th>Parameters</th>
    <th>Meaning</th>
    <th>Type</th>
    <th>Value</th>
  </tr>
  <tr>
    <td>GaitParam</td>
    <td>Robot gait</td>
    <td>int</td>
    <td>Basic = 1<br>Stair = 14</td>
  </tr>
</table>

### 1.2.5 Motion Control(Axis Command)

You can control the motion of the robot by this request.

<table>
  <tr>
    <th>Type</th>
    <th>Command</th>
    <th>Message type</th>
  </tr>
  <tr>
    <td>2</td>
    <td>21</td>
    <td>Motion Control</td>
  </tr>
</table>

[Notes] This command is only supported in Regular Mode.

JSON request:

```json
{
  "PatrolDevice": {
    "Type": 2,
    "Command": 21,
    "Time": "2023-01-01 00:00:00",
    "Items": {
      "X": 0.0,
      "Y": 0.0,
      "Z": 0.0,
      "Roll": 0.0,
      "Pitch": 0.0,
      "Yaw": 0.0
    }
  }
}
```

XML request:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
  <Type>2</Type>
  <Command>21</Command>
  <Time>2023-01-01 00:00:00</Time>
  <Items>
    <X>0.0</X>
    <Y>0.0</Y>
    <Z>0.0</Z>
    <Roll>0.0</Roll>
    <Pitch>0.0</Pitch>
    <Yaw>0.0</Yaw>
  </Items>
</PatrolDevice>
```

&lt;page_number&gt;13 / 35&lt;/page_number&gt;

---


## Page 14

DEEP Robotics

```xml
<Z>0.0</Z>
<Roll>0.0</Roll>
<Pitch>0.0</Pitch>
<Yaw>0.0</Yaw>
</Items>
</PatrolDevice>
```

In this request message, the Items field contains the following parameters:

<table>
  <tr>
    <th>Parameters</th>
    <th>Meaning</th>
    <th>Type</th>
    <th>Value</th>
  </tr>
  <tr>
    <td>X</td>
    <td>Forward and backward speed</td>
    <td>float</td>
    <td>[-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed)</td>
  </tr>
  <tr>
    <td>Y</td>
    <td>Left and right movement speed</td>
    <td>float</td>
    <td>[-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed)</td>
  </tr>
  <tr>
    <td>Z</td>
    <td>Vertical movement speed</td>
    <td>float</td>
    <td>[-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed)</td>
  </tr>
  <tr>
    <td>Roll</td>
    <td>Roll angle</td>
    <td>float</td>
    <td>[-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed)</td>
  </tr>
  <tr>
    <td>Pitch</td>
    <td>Pitch angle</td>
    <td>float</td>
    <td>[-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed)</td>
  </tr>
  <tr>
    <td>Yaw</td>
    <td>Yaw angle</td>
    <td>float</td>
    <td>[-1,1] (Values between [-1,1] represent the ratio of the current command speed to the maximum speed)</td>
  </tr>
</table>

[Notes] Recommended frequency for translation/rotation control: 20 Hz. Only X, Y, and Yaw are effective in Basic and Stair gaits.

## 1.2.6 Flashlight

You can turn the robot's front and rear lights on or off by this request and determine whether the operation was successful by referring to the response information in Section 1.3.

<table>
  <tr>
    <th>Type</th>
    <th>Command</th>
    <th>Message type</th>
  </tr>
  <tr>
    <td>1101</td>
    <td>2</td>
    <td>Flashlight</td>
  </tr>
</table>

JSON request:

```json
{
  "PatrolDevice": {
    "Type": 1101,
    "Command": 2,
    "Time": "2023-01-01 00:00:00",
    "Items": {
      "Front": 0

---


## Page 15

DEEP Robotics

```json
{
    "Back": 0
}
```

XML request:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>1101</Type>
    <Command>2</Command>
    <Time>2023-01-01 00:00:00</Time>
    <Items>
        <Front>0</Front>
        <Back>0</Back>
    </Items>
</PatrolDevice>
```

In this request message, the Items field contains the following parameters:

<table>
<thead>
<tr>
<th>Parameters</th>
<th>Meaning</th>
<th>Type</th>
<th>Value</th>
</tr>
</thead>
<tbody>
<tr>
<td>Front</td>
<td>Front flashlight of the robot</td>
<td rowspan="2">int</td>
<td>Light off= 0</td>
</tr>
<tr>
<td>Back</td>
<td>Back flashlight of the robot</td>
<td>Light on= 1</td>
</tr>
</tbody>
</table>

&lt;page_number&gt;15 / 35&lt;/page_number&gt;

---


## Page 16

DEEP Robotics

# 1.3 ASDU Message Set (Response Type)

## 1.3.1 Obtain Real-time State

The robot will actively report status information to the IP address and port that sent the heartbeat command (refer to section 1.2.1).

### 1.3.1.1 Basic Status Report

This message type is actively reported at 2 Hz, you can obtain the current basic status of the robot through the response of this message type.

<table>
<thead>
<tr>
<th>Type</th>
<th>Command</th>
<th>Message type</th>
</tr>
</thead>
<tbody>
<tr>
<td>1002</td>
<td>6</td>
<td>Obtain Basic Status</td>
</tr>
</tbody>
</table>

**JSON response:**

```json
{
    "PatrolDevice": {
        "Type": 1002,
        "Command": 6,
        "Time": "2023-01-01 00:00:00",
        "Items": {
            "BasicStatus": {
                "MotionState": 0,
                "Gait": 0,
                "Charge": 0,
                "HES": 0,
                "ControlUsageMode": 0,
                "Direction": 0,
                "OOA": 0,
                "PowerManagement": 0,
                "Sleep": false,
                "Version": STD
            }
        }
    }
}
```

**XML response:**

```xml
<?xml version="1.0" encoding="UTF-8"?>

---


## Page 17

DEEP Robotics

```xml
<PatrolDevice>
    <Type>1002</Type>
    <Command>6</Command>
    <Time>2023-01-01 00:00:01</Time>
    <Items>
        <BasicStatus>
            <MotionState>0</MotionState>
            <Gait>0</Gait>
            <Charge>0</Charge>
            <HES>0</HES>
            <ControlUsageMode>0</ControlUsageMode>
            <Direction>0</Direction>
            <OOA>0</OOA>
            <PowerManagement>0</PowerManagement>
            <Sleep>false</Sleep>
            <Version>STD</Version>
        </BasicStatus>
    </Items>
</PatrolDevice>
```

The Items field contains the following parameters:

<table>
<thead>
<tr>
<th>Parameters</th>
<th>Meaning</th>
<th>Type</th>
<th>Value</th>
</tr>
</thead>
<tbody>
<tr>
<td>MotionState</td>
<td>Robot motion state<sup>[1]</sup></td>
<td>int</td>
<td>Idle = 0<br/>Stand = 1<br/>Soft Emergency Stop = 2<br/>Power-on Damping = 3<br/>Sitting = 4<br/>Standard= 6</td>
</tr>
<tr>
<td>Gait</td>
<td>Robot gait</td>
<td>int</td>
<td>Basic = 1<br/>Stair = 14</td>
</tr>
<tr>
<td>Charge</td>
<td>Robot charging status</td>
<td>int</td>
<td>Idle = 0<br/>Enter charge dock = 1<br/>Charging = 2<br/>Exiting charge dock = 3<br/>Robot error = 4<br/>Robot is on the dock but not charged = 5</td>
</tr>
<tr>
<td>HES</td>
<td>Hard emergency stop status</td>
<td>int</td>
<td>Not triggered = 0</td>
</tr>
</tbody>
</table>

&lt;page_number&gt;17 / 35&lt;/page_number&gt;

---


## Page 18

DEEP Robotics

<table>
  <tr>
    <td><b>Parameters</b></td>
    <td><b>Meaning</b></td>
    <td><b>Type</b></td>
    <td><b>Value</b></td>
  </tr>
  <tr>
    <td></td>
    <td></td>
    <td></td>
    <td>triggered = 1</td>
  </tr>
  <tr>
    <td>ControlUsageMode</td>
    <td>Robot control usage mode</td>
    <td>int</td>
    <td>Regular mode = 0<br>Navigation mode = 1</td>
  </tr>
  <tr>
    <td>Direction</td>
    <td>The robot's forward direction<sup>[2]</sup></td>
    <td>int</td>
    <td>Front = 0<br>Back = 1</td>
  </tr>
  <tr>
    <td>Version</td>
    <td>Device version</td>
    <td>/</td>
    <td>Lynx M20 = STD<br>Lynx M20 Pro = PRO</td>
  </tr>
</table>

[2] The robot's forward direction is defined as the movement direction when a positive X-axis velocity is commanded. If the forward direction is set to "Back", the side with the Hard Emergency Stop is considered forward direction.

### 1.3.1.2 Motion Control Status Report

This message type is actively reported at 10 Hz, you can obtain the current motion control status of the robot through the response of this message type.

<table>
  <tr>
    <td><b>Type</b></td>
    <td><b>Command</b></td>
    <td><b>Message type</b></td>
  </tr>
  <tr>
    <td>1002</td>
    <td>4</td>
    <td>Obtain Motion Control Status</td>
  </tr>
</table>

JSON response:

```json
{
  "PatrolDevice": {
    "Type": 1002,
    "Command": 4,
    "Time": "2023-01-01 00:00:00",
    "Items": {
      "MotionStatus": {
        "Roll": 0.0,
        "Pitch": 0.0,
        "Yaw": 0.0,
        "OmegaZ": 0.0,
        "LinearX": 0.0,
        "LinearY": 0.0,
        "Height": 0.0,
        "Payload": 0.0,
```

&lt;page_number&gt;18 / 35&lt;/page_number&gt;

---


## Page 19

DEEP Robotics

```json
{
    "MotionStatus": {
        "Roll": 0.0,
        "Pitch": 0.0,
        "Yaw": 0.0,
        "OmegaZ": 0.0,
        "LinearX": 0.0,
        "LinearY": 0.0,
        "Height": 0.0,
        "Payload": 0.0,
        "RemainMile": 0.0
    },
    "MotorStatus": {
        "LeftFrontHipX": 0.0,
        "LeftFrontHipY": 0.0,
        "LeftFrontKnee": 0.0,
        "LeftFrontWheel": 0.0,
        "RightFrontHipX": 0.0,
        "RightFrontHipY": 0.0,
        "RightFrontKnee": 0.0,
        "RightFrontWheel": 0.0,
        "LeftBackHipX": 0.0,
        "LeftBackHipY": 0.0,
        "LeftBackKnee": 0.0,
        "LeftBackWheel": 0.0,
        "RightBackHipX": 0.0,
        "RightBackHipY": 0.0,
        "RightBackKnee": 0.0,
        "RightBackWheel": 0.0
    }
}
```

XML response:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>1002</Type>
    <Command>4</Command>
    <Time>2023-01-01 00:00:01</Time>
    <Items>
        <MotionStatus>
            <Roll>0.0</Roll>
            <Pitch>0.0</Pitch>
            <Yaw>0.0</Yaw>
            <OmegaZ>0.0</OmegaZ>
            <LinearX>0.0</LinearX>
            <LinearY>0.0</LinearY>
            <Height>0.0</Height>
            <Payload>0.0</Payload>
            <RemainMile>0.0</RemainMile>
        </MotionStatus>
        <MotorStatus>
            <LeftFrontHipX>0.0</LeftFrontHipX>

---


## Page 20

DEEP Robotics

| Line | Content |
|---|---|
| 20 | `<LeftFrontHipY>0.0</LeftFrontHipY>` |
| 21 | `<LeftFrontKnee>0.0</LeftFrontKnee>` |
| 22 | `<LeftFrontWheel>0.0</LeftFrontWheel>` |
| 23 | `<RightFrontHipX>0.0</RightFrontHipX>` |
| 24 | `<RightFrontHipY>0.0</RightFrontHipY>` |
| 25 | `<RightFrontKnee>0.0</RightFrontKnee>` |
| 26 | `<RightFrontWheel>0.0</RightFrontWheel>` |
| 27 | `<LeftBackHipX>0.0</LeftBackHipX>` |
| 28 | `<LeftBackHipY>0.0</LeftBackHipY>` |
| 29 | `<LeftBackKnee>0.0</LeftBackKnee>` |
| 30 | `<LeftBackWheel>0.0</LeftBackWheel>` |
| 31 | `<RightBackHipX>0.0</RightBackHipX>` |
| 32 | `<RightBackHipY>0.0</RightBackHipY>` |
| 33 | `<RightBackKnee>0.0</RightBackKnee>` |
| 34 | `<RightBackWheel>0.0</RightBackWheel>` |
| 35 | `</MotorStatus>` |
| 36 | `</Items>` |
| 37 | `</PatrolDevice>` |

In this response message, the Items field contains the MotionStatus and MotorStatus parameter groups:

The MotionStatus parameter group provides feedback on the motion status of the robot:

<table>
<thead>
<tr>
<th>Parameters</th>
<th>Meaning</th>
<th>Type</th>
</tr>
</thead>
<tbody>
<tr>
<td>Roll/Pitch/Yaw</td>
<td>The posture angle of robot (rad)</td>
<td>float</td>
</tr>
<tr>
<td>OmegaZ</td>
<td>Z-axis angular velocity of robot (rad/s)</td>
<td>float</td>
</tr>
<tr>
<td>LinearX / LinearY</td>
<td>X-axis/Y-axis linear velocity of robot (m/s)</td>
<td>float</td>
</tr>
<tr>
<td>Height</td>
<td>The height of the robot (m)</td>
<td>float</td>
</tr>
<tr>
<td>Payload</td>
<td>Invalid parameter</td>
<td>/</td>
</tr>
<tr>
<td>RemainMile</td>
<td>Estimated remaining range (km)</td>
<td>float</td>
</tr>
</tbody>
</table>

The MotorStatus<sup>[3]</sup> parameter group provides feedback on the motion status of each joint of the robot:

<table>
<thead>
<tr>
<th>Parameters</th>
<th>Meaning</th>
<th>Type</th>
</tr>
</thead>
<tbody>
<tr>
<td>*HipX</td>
<td>Angle of hip joint for abduction and adduction (rad)</td>
<td>float</td>
</tr>
<tr>
<td>*HipY</td>
<td>Angle of hip joint for flexion and extension (rad)</td>
<td>float</td>
</tr>
</tbody>
</table>

&lt;page_number&gt;20 / 35&lt;/page_number&gt;

---


## Page 21

DEEP Robotics

<table>
  <tr>
    <th>Parameters</th>
    <th>Meaning</th>
    <th>Type</th>
  </tr>
  <tr>
    <td>*Knee</td>
    <td>Angle of knee joint (rad)</td>
    <td>float</td>
  </tr>
  <tr>
    <td>*Wheel</td>
    <td>Speed of wheel joint (rad/s)</td>
    <td>float</td>
  </tr>
</table>

[3] The asterisk (*) to the left of a parameter item in the MotorStatus parameter group indicates the position of an omitted parameter name. The left side is the side where the battery bin is located. The back side is the side with the Hard Emergency STOP button. For example, LeftBackHipX refers to the left front HipX joint for abduction and adduction, and the parameter value indicates the angle of the joint. The names and meanings of other parameters follow the same principle.

## 1.3.1.3 Device State Report

This message type is actively reported at 2 Hz, you can obtain the current device state of the robot through the response of this message type.

<table>
  <tr>
    <th>Type</th>
    <th>Command</th>
    <th>Message type</th>
  </tr>
  <tr>
    <td>1002</td>
    <td>5</td>
    <td>Obtain Device state</td>
  </tr>
</table>

JSON response:

```json
{
  "PatrolDevice": {
    "Type": 1002,
    "Command": 5,
    "Time": "2023-01-01 00:00:00",
    "Items": {
      "BatteryStatus": {
        "VoltageLeft": 0.0,
        "VoltageRight": 0.0,
        "BatteryLevelLeft": 0.0,
        "BatteryLevelRight": 0.0,
        "battery_temperatureLeft": 0.0,
        "battery_temperatureRight": 0.0,
        "chargeLeft": false,
        "chargeRight": false
      }
    }
  }
}
```

&lt;page_number&gt;21 / 35&lt;/page_number&gt;

---


## Page 22

json
},
"DeviceTemperature": {
    "LeftFrontHipXMotor": 0.0,
    "LeftFrontHipXDriver": 0.0,
    "LeftFrontHipYMotor": 0.0,
    "LeftFrontHipYDriver": 0.0,
    "LeftFrontKneeMotor": 0.0,
    "LeftFrontKneeDriver": 0.0,
    "LeftFrontWheelMotor": 0.0,
    "LeftFrontWheelDriver": 0.0,
    "RightFrontHipXMotor": 0.0,
    "RightFrontHipXDriver": 0.0,
    "RightFrontHipYMotor": 0.0,
    "RightFrontHipYDriver": 0.0,
    "RightFrontKneeMotor": 0.0,
    "RightFrontKneeDriver": 0.0,
    "RightFrontWheelMotor": 0.0,
    "RightFrontWheelDriver": 0.0,
    "LeftBackHipXMotor": 0.0,
    "LeftBackHipXDriver": 0.0,
    "LeftBackHipYMotor": 0.0,
    "LeftBackHipYDriver": 0.0,
    "LeftBackKneeMotor": 0.0,
    "LeftBackKneeDriver": 0.0,
    "LeftBackWheelMotor": 0.0,
    "LeftBackWheelDriver": 0.0,
    "RightBackHipXMotor": 0.0,
    "RightBackHipXDriver": 0.0,
    "RightBackHipYMotor": 0.0,
    "RightBackHipYDriver": 0.0,
    "RightBackKneeMotor": 0.0,
    "RightBackKneeDriver": 0.0,
    "RightBackWheelMotor": 0.0,
    "RightBackWheelDriver": 0.0
},
"Led": {
    "Fill": {
        "Front": 1,
        "Back": 1
    }
},
"GPS": {
    "Latitude": 0.0,
    "Longitude": 0.0,
...

---


## Page 23

DEEP Robotics

```json
{
    "Speed": 0.0,
    "Course": 0.0,
    "FixQuality": 0.0,
    "NumSatellites": 0,
    "Altitude": 0.0,
    "HDOP": 0.0,
    "VDOP": 0,
    "PDOP": 0.0,
    "VisibleSatellites": 0
},
    "DevEnable": {
        "FanSpeed": 100,
        "LoadPower": 1,
        "LedHost": 1,
        "LedExt": 1,
        "FP": 1,
        "Lidar": {
            "Front": 1,
            "Back": 1
        },
        "GPS": 1,
        "Video": {
            "Front": 1,
            "Back": 1
        }
    },
    "CPU": {
        "CPU103": {
            "Temperature": 0.0,
            "FrequencyInt": 0.0,
            "FrequencyApp": 0.0
        },
        "CPU105": {
            "Temperature": 0.0,
            "FrequencyInt": 0.0,
            "FrequencyApp": 0.0
        },
        "CPU106": {
            "Temperature": 0.0,
            "FrequencyInt": 0.0,
            "FrequencyApp": 0.0
        }
    }
}
```

&lt;page_number&gt;23 / 35&lt;/page_number&gt;

---


## Page 24

DEEP Robotics

```csharp
104 }
105 }
```

XML response:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>1002</Type>
    <Command>5</Command>
    <Time>2023-01-01 00:00:01</Time>
    <Items>
        <BatteryStatus>
            <VoltageLeft>0.0</VoltageLeft>
            <VoltageRight>0.0</VoltageRight>
            <BatteryLevelLeft>0.0</BatteryLevelLeft>
            <BatteryLevelRight>0.0</BatteryLevelRight>
            <Battery_temperatureLeft>0.0</Battery_temperatureLeft>
            <Battery_temperatureRight>0.0</Battery_temperatureRight>
            <chargeLeft>false</chargeLeft>
            <chargeRight>false</chargeRight>
        </BatteryStatus>
        <DeviceTemperature>
            <LeftFrontHipXMotor>0.0</LeftFrontHipXMotor>
            <LeftFrontHipXDriver>0.0</LeftFrontHipXDriver>
            <LeftFrontHipYMotor>0.0</LeftFrontHipYMotor>
            <LeftFrontHipYDriver>0.0</LeftFrontHipYDriver>
            <LeftFrontKneeMotor>0.0</LeftFrontKneeMotor>
            <LeftFrontKneeDriver>0.0</LeftFrontKneeDriver>
            <LeftFrontWheelMotor>0.0</LeftFrontWheelMotor>
            <LeftFrontWheelDriver>0.0</LeftFrontWheelDriver>
            <RightFrontHipXMotor>0.0</RightFrontHipXMotor>
            <RightFrontHipXDriver>0.0</RightFrontHipXDriver>
            <RightFrontHipYMotor>0.0</RightFrontHipYMotor>
            <RightFrontHipYDriver>0.0</RightFrontHipYDriver>
            <RightFrontKneeMotor>0.0</RightFrontKneeMotor>
            <RightFrontKneeDriver>0.0</RightFrontKneeDriver>
            <RightFrontWheelMotor>0.0</RightFrontWheelMotor>
            <RightFrontWheelDriver>0.0</RightFrontWheelDriver>
            <LeftBackHipXMotor>0.0</LeftBackHipXMotor>
            <LeftBackHipXDriver>0.0</LeftBackHipXDriver>
            <LeftBackHipYMotor>0.0</LeftBackHipYMotor>
            <LeftBackHipYDriver>0.0</LeftBackHipYDriver>
            <LeftBackKneeMotor>0.0</LeftBackKneeMotor>
            <LeftBackKneeDriver>0.0</LeftBackKneeDriver>
            <LeftBackWheelMotor>0.0</LeftBackWheelMotor>
        </DeviceTemperature>
    </Items>
</PatrolDevice>
```

&lt;page_number&gt;24 / 35&lt;/page_number&gt;

---


## Page 25

DEEP Robotics

<table>
  <tr>
    <td>41</td>
    <td>&lt;LeftBackWheelDriver&gt;0.0&lt;/LeftBackWheelDriver&gt;</td>
  </tr>
  <tr>
    <td>42</td>
    <td>&lt;RightBackHipXMotor&gt;0.0&lt;/RightBackHipXMotor&gt;</td>
  </tr>
  <tr>
    <td>43</td>
    <td>&lt;RightBackHipXDriver&gt;0.0&lt;/RightBackHipXDriver&gt;</td>
  </tr>
  <tr>
    <td>44</td>
    <td>&lt;RightBackHipYMotor&gt;0.0&lt;/RightBackHipYMotor&gt;</td>
  </tr>
  <tr>
    <td>45</td>
    <td>&lt;RightBackHipYDriver&gt;0.0&lt;/RightBackHipYDriver&gt;</td>
  </tr>
  <tr>
    <td>46</td>
    <td>&lt;RightBackKneeMotor&gt;0.0&lt;/RightBackKneeMotor&gt;</td>
  </tr>
  <tr>
    <td>47</td>
    <td>&lt;RightBackKneeDriver&gt;0.0&lt;/RightBackKneeDriver&gt;</td>
  </tr>
  <tr>
    <td>48</td>
    <td>&lt;RightBackWheelMotor&gt;0.0&lt;/RightBackWheelMotor&gt;</td>
  </tr>
  <tr>
    <td>49</td>
    <td>&lt;RightBackWheelDriver&gt;0.0&lt;/RightBackWheelDriver&gt;</td>
  </tr>
  <tr>
    <td>50</td>
    <td>&lt;/DeviceTemperature&gt;</td>
  </tr>
  <tr>
    <td>51</td>
    <td>&lt;Led&gt;</td>
  </tr>
  <tr>
    <td>52</td>
    <td>&lt;Fill&gt;</td>
  </tr>
  <tr>
    <td>53</td>
    <td>&lt;Front&gt;0&lt;/Front&gt;</td>
  </tr>
  <tr>
    <td>54</td>
    <td>&lt;Back&gt;0&lt;/Back&gt;</td>
  </tr>
  <tr>
    <td>55</td>
    <td>&lt;/Fill&gt;</td>
  </tr>
  <tr>
    <td>56</td>
    <td>&lt;/Led&gt;</td>
  </tr>
  <tr>
    <td>57</td>
    <td>&lt;GPS&gt;</td>
  </tr>
  <tr>
    <td>58</td>
    <td>&lt;Latitude&gt;0.0&lt;/Latitude&gt;</td>
  </tr>
  <tr>
    <td>59</td>
    <td>&lt;Longitude&gt;0.0&lt;/Longitude&gt;</td>
  </tr>
  <tr>
    <td>60</td>
    <td>&lt;Speed&gt;0.0&lt;/Speed&gt;</td>
  </tr>
  <tr>
    <td>61</td>
    <td>&lt;Course&gt;0.0&lt;/Course&gt;</td>
  </tr>
  <tr>
    <td>62</td>
    <td>&lt;FixQuality&gt;0.0&lt;/FixQuality&gt;</td>
  </tr>
  <tr>
    <td>63</td>
    <td>&lt;NumSatellites&gt;0&lt;/NumSatellites&gt;</td>
  </tr>
  <tr>
    <td>64</td>
    <td>&lt;Altitude&gt;0.0&lt;/Altitude&gt;</td>
  </tr>
  <tr>
    <td>65</td>
    <td>&lt;HDOP&gt;0.0&lt;/HDOP&gt;</td>
  </tr>
  <tr>
    <td>66</td>
    <td>&lt;VDOP&gt;0.0&lt;/VDOP&gt;</td>
  </tr>
  <tr>
    <td>67</td>
    <td>&lt;PDOP&gt;0.0&lt;/PDOP&gt;</td>
  </tr>
  <tr>
    <td>68</td>
    <td>&lt;VisibleSatellites&gt;0&lt;/VisibleSatellites&gt;</td>
  </tr>
  <tr>
    <td>69</td>
    <td>&lt;/GPS&gt;</td>
  </tr>
  <tr>
    <td>70</td>
    <td>&lt;DevEnable&gt;</td>
  </tr>
  <tr>
    <td>71</td>
    <td>&lt;FanSpeed&gt;100&lt;/FanSpeed&gt;</td>
  </tr>
  <tr>
    <td>72</td>
    <td>&lt;LoadPower&gt;0&lt;/LoadPower&gt;</td>
  </tr>
  <tr>
    <td>73</td>
    <td>&lt;LedHost&gt;0&lt;/LedHost&gt;</td>
  </tr>
  <tr>
    <td>74</td>
    <td>&lt;LedExt&gt;0&lt;/LedExt&gt;</td>
  </tr>
  <tr>
    <td>75</td>
    <td>&lt;FP&gt;0&lt;/FP&gt;</td>
  </tr>
  <tr>
    <td>76</td>
    <td>&lt;Lidar&gt;</td>
  </tr>
  <tr>
    <td>77</td>
    <td>&lt;Front&gt;0&lt;/Front&gt;</td>
  </tr>
  <tr>
    <td>78</td>
    <td>&lt;Back&gt;0&lt;/Back&gt;</td>
  </tr>
  <tr>
    <td>79</td>
    <td>&lt;/Lidar&gt;</td>
  </tr>
  <tr>
    <td>80</td>
    <td>&lt;GPS&gt;0&lt;/GPS&gt;</td>
  </tr>
  <tr>
    <td>81</td>
    <td>&lt;Video&gt;</td>
  </tr>
  <tr>
    <td>82</td>
    <td>&lt;Front&gt;0&lt;/Front&gt;</td>
  </tr>
  <tr>
    <td>83</td>
    <td>&lt;Back&gt;0&lt;/Back&gt;</td>
  </tr>
  <tr>
    <td>84</td>
    <td>&lt;/Video&gt;</td>
  </tr>
</table>

&lt;page_number&gt;25 / 35&lt;/page_number&gt;

---


## Page 26

DEEP Robotics

85 & </DevEnable>
86 & <CPU>
87 & & <CPU103>
88 & & & <Temperature>0.0</Temperature>
89 & & & <FrequencyInt>0.0</FrequencyInt>
90 & & & <FrequencyApp>0.0</FrequencyApp>
91 & & </CPU103>
92 & & <CPU105>
93 & & & <Temperature>0.0</Temperature>
94 & & & <FrequencyInt>0.0</FrequencyInt>
95 & & & <FrequencyApp>0.0</FrequencyApp>
96 & & </CPU105>
97 & & <CPU106>
98 & & & <Temperature>0.0</Temperature>
99 & & & <FrequencyInt>0.0</FrequencyInt>
100 & & & <FrequencyApp>0.0</FrequencyApp>
101 & & </CPU106>
102 & & </CPU>
103 & </Items>
104 & </PatrolDevice>

In this response message, the Items field contains six parameter groups: BatteryStatus、DeviceTemperature、Led、GPS、DevEnable and CPU.

The BatteryStatus<sup>[4]</sup> parameter group provides feedback on the status of batteries of the robot:

<table>
  <tr>
    <th>Parameters</th>
    <th>Meaning</th>
    <th>Type</th>
    <th>Value</th>
  </tr>
  <tr>
    <td>Voltage*</td>
    <td>Voltage of the battery (V)</td>
    <td>float</td>
    <td></td>
  </tr>
  <tr>
    <td>BatteryLevel*</td>
    <td>Percentage of remaining battery power (%)</td>
    <td>float</td>
    <td>[0,100]</td>
  </tr>
  <tr>
    <td>Battery_temperature*</td>
    <td>Temperature of battery (°C)</td>
    <td>float</td>
    <td></td>
  </tr>
  <tr>
    <td>Charge*</td>
    <td>Charging status of battery</td>
    <td>bool</td>
    <td>true = Charging<br>false = Not Charging</td>
  </tr>
</table>

The DeviceTemperature<sup>[5]</sup> parameter group provides feedback on the temperature information of each joint motor and driver:

<table>
  <tr>
    <th>Parameters</th>
    <th>Meaning</th>
    <th>Type</th>
    <th>Value</th>
  </tr>
  <tr>
    <td>*Motor</td>
    <td>Temperature of motor (°C)</td>
    <td>float</td>
    <td></td>
  </tr>
</table>

&lt;page_number&gt;26 / 35&lt;/page_number&gt;

---


## Page 27

DEEP Robotics

| Parameters | Meaning | Type | Value |
|---|---|---|---|
| *Driver | Temperature of driver (°C) | float |  |

The LED parameter group provides feedback on the front and back flashlight status of the robot:

| Parameters | Meaning | Type | Value |
|---|---|---|---|
| Fill:Front / Back | Status of front / back flashlight | int | Off = 0, On = 1 |

The GPS parameter group provides feedback satellite positioning module positioning data:

| Parameters | Meaning | Type | Value |
|---|---|---|---|
| Latitude | Latitude of the robot in the world coordinate system (deg) | float | Positive values indicate north of the equator; Negative values indicate south of the equator. |
| Longitude | Longitude of the robot in the world coordinate system (deg) | float | Positive values indicate east of the prime meridian; Negative values indicate west of the prime meridian. |
| Speed | Ground speed (km/h) | float |  |
| Course | Robot course in the world coordinate system (deg) | float | The angle between the direction of travel and true north |
| FixQuality | Robot positioning quality | float | The higher the value, the more accurate the positioning. |
| NumSatellites | Number of satellites involved in positioning | int |  |
| Altitude | Altitude of the robot (m) | float |  |
| HDOP | Position Dilution of Precision Comprehensive indicator reflecting | float | [0.5, 99.9] A smaller value indicates |

&lt;page_number&gt;27 / 35&lt;/page_number&gt;

---


## Page 28

DEEP Robotics

<table>
  <tr>
    <td><b>Parameters</b></td>
    <td><b>Meaning</b></td>
    <td><b>Type</b></td>
    <td><b>Value</b></td>
  </tr>
  <tr>
    <td></td>
    <td>positioning accuracy</td>
    <td></td>
    <td>higher precision.</td>
  </tr>
  <tr>
    <td>VDOP</td>
    <td>Horizontal Dilution of Precision<br>Reflects positioning accuracy in the horizontal direction</td>
    <td>float</td>
    <td></td>
  </tr>
  <tr>
    <td>PDOP</td>
    <td>Vertical Dilution of Precision<br>Reflects positioning accuracy in the vertical direction</td>
    <td>float</td>
    <td></td>
  </tr>
  <tr>
    <td>VisibleSatellites</td>
    <td>Total number of visible satellites</td>
    <td>int</td>
    <td></td>
  </tr>
</table>

The DevEnable parameter group provides feedback on the operating status of the robot's internal components. The meanings of some of the parameters are as follows:

<table>
  <tr>
    <td><b>Parameters</b></td>
    <td><b>Meaning</b></td>
    <td><b>Type</b></td>
    <td><b>Value</b></td>
  </tr>
  <tr>
    <td>Lidar:Front/Back</td>
    <td>Front and back power switch</td>
    <td>int</td>
    <td>Off = 0, On = 1</td>
  </tr>
  <tr>
    <td>GPS</td>
    <td>Satellite positioning module power switch</td>
    <td>int</td>
    <td>Off = 0, On = 1</td>
  </tr>
  <tr>
    <td>Video:Front/Back</td>
    <td>Front and back camera power switch</td>
    <td>int</td>
    <td>Off = 0, On = 1</td>
  </tr>
</table>

The CPU[6] parameter group provides feedback on the operating status of each CPU inside the robot:

<table>
  <tr>
    <td><b>Parameters</b></td>
    <td><b>Meaning</b></td>
    <td><b>Type</b></td>
    <td><b>Value</b></td>
  </tr>
  <tr>
    <td>Temperature</td>
    <td>Maximum CPU temperature</td>
    <td>°C</td>
    <td></td>
  </tr>
  <tr>
    <td>FrequencyInt</td>
    <td>Efficiency core (A55) usage (%)</td>
    <td>%</td>
    <td></td>
  </tr>
  <tr>
    <td>FrequencyApp</td>
    <td>Performance core (A76) usage (%)</td>
    <td>%</td>
    <td></td>
  </tr>
</table>

[4] The asterisk (*) to the right of the parameter item indicates the position of the omitted parameter name. The right side is the side closer to the hard emergency stop button. For example, ‘BatteryLevelRight’ refers to the remaining battery percentage on the right side (the side closer to the hard emergency stop button). The meanings of other parameters follow the same principle.

&lt;page_number&gt;28 / 35&lt;/page_number&gt;

---


## Page 29

DEEP Robotics

[5] The asterisk (*) on the left side of the parameter item indicates the omitted parameter name information. Please refer to [3] to determine the joint name. For example, RightFrontKneeDriver refers to the right front knee joint driver, and the parameter value indicates the temperature information of the right front knee joint driver. The names and meanings of other parameters follow the same principle.

[6] In the CPU parameter group, CPU103, CPU105, and CPU106 correspond to the CPUs of the three hosts inside the robot, respectively. CPU106 is only effective on the Lynx M20 Pro.

### 1.3.2 Obtain Abnormal Status

You can obtain the current abnormal status information of the robot by this request.

<table>
<thead>
<tr>
<th>Type</th>
<th>Command</th>
<th>Message type</th>
</tr>
</thead>
<tbody>
<tr>
<td>1002</td>
<td>3</td>
<td>Obtain Abnormal Status</td>
</tr>
</tbody>
</table>

[Notes] The response will actively report to the IP and port that sent the heartbeat command at a fixed frequency of 2 Hz, when the status changes (such as when an abnormal occurs or is resolved), the robot will perform an additional report.

JSON response:

```json
{
    "PatrolDevice": {
        "Type": 1002,
        "Command": 3,
        "Time": "2023-01-01 00:00:00",
        "Items": {
            "ErrorList":[
                {
                    "errorCode": errorCode,
                    "component": value,
                },
                {
                    "errorCode": errorCode,
```

&lt;page_number&gt;29 / 35&lt;/page_number&gt;

---


## Page 30

DEEP Robotics

```json
{
    "component": value,
}
...
//Other omitted errors
]
}
}
```

XML response:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<PatrolDevice>
    <Type>1002</Type>
    <Command>3</Command>
    <Time>2023-01-01 00:00:00</Time>
    <Items>
        <ErrorList>
            <errorCode>errorCode</errorCode>
            <component>value</component>
        </ErrorList>
        <ErrorList>
            <errorCode>errorCode</errorCode>
            <component>value</component>
        </ErrorList>
    </Items>
</PatrolDevice>
```

In this response message, the items field contains the following parameters:

<table>
<thead>
<tr>
<th>Parameters</th>
<th>Meaning</th>
<th>Type</th>
<th>Value</th>
</tr>
</thead>
<tbody>
<tr>
<td>errorCode</td>
<td>Error code</td>
<td>int</td>
<td>Check the table below</td>
</tr>
<tr>
<td>value</td>
<td>Location of the component where the error occurred[7]</td>
<td>int</td>
<td>Use bits as part numbers</td>
</tr>
</tbody>
</table>

[7] Please refer to Section "1.3.1.1 Motion Control Status Reporting" for joint numbering, which follows the order of parameters in the `<MotorStatus>` group. Joints are numbered from low to high bits in the order of FL/FR/BL/BR + HipX/HipY/Knee/Wheel. For example, 0x21 (0000 0000 0010 0001) indicates that the 1st joint (bit 0, LeftFrontHipX) and the 6th joint (bit 5, RightFrontHipY) have errors corresponding to the errorCode. Other joint bits follow the same convention. For battery numbering, bit 0 represents the right-side battery

&lt;page_number&gt;30 / 35&lt;/page_number&gt;

---


## Page 31

DEEP Robotics

(the side near the hardware emergency stop button), and bit 1 represents the left-side battery.

The meaning of the value of errorCode is shown in the following table:

<table>
<thead>
<tr>
<th>Value</th>
<th>Meaning</th>
<th>Value</th>
<th>Meaning</th>
</tr>
</thead>
<tbody>
<tr>
<td>0x8001</td>
<td>Motor Temperature Warning</td>
<td>0x8108</td>
<td>Battery Cell Undervoltage Protection</td>
</tr>
<tr>
<td>0x8002</td>
<td>Motor Over-temperature Protection</td>
<td>0x8112</td>
<td>Battery Discharge Low-temperature Protection</td>
</tr>
<tr>
<td>0x8003</td>
<td>Motor Temperature Critical Shutdown</td>
<td>0x8115</td>
<td>Battery Charging Over-temperature Protection</td>
</tr>
<tr>
<td>0x8007</td>
<td>Joint Driver Over-temperature</td>
<td>0x8116</td>
<td>Battery Charge Low-temperature Protection</td>
</tr>
<tr>
<td>0x8008</td>
<td>Driver Undervoltage Protection</td>
<td>0x8117</td>
<td>Battery Cell Overvoltage Protection</td>
</tr>
<tr>
<td>0x8009</td>
<td>Driver Overvoltage Protection</td>
<td>0x8118</td>
<td>Battery Pack Overvoltage Protection</td>
</tr>
<tr>
<td>0x8012</td>
<td>Joint Driver Communication Timeout</td>
<td>0x8119</td>
<td>Battery Pack Undervoltage Protection</td>
</tr>
<tr>
<td>0x8016</td>
<td>No Encoder Value</td>
<td>0x8120</td>
<td>Battery Charging Overcurrent Protection</td>
</tr>
<tr>
<td>0x8020</td>
<td>Driver Overcurrent Protection</td>
<td>0x8121</td>
<td>Battery Discharge Overcurrent Protection</td>
</tr>
<tr>
<td>0x8021</td>
<td>Temperature Sensor Disconnected</td>
<td>0x8122</td>
<td>Short Circuit Protection</td>
</tr>
<tr>
<td>0x8022</td>
<td>Joint Angle Limit Exceeded</td>
<td>0x8123</td>
<td>Battery Front-end Detection IC Error</td>
</tr>
<tr>
<td>0x8024</td>
<td>Joint Data is NaN</td>
<td>0x8124</td>
<td>Battery Software MOS Lock</td>
</tr>
<tr>
<td>0x8025</td>
<td>Joint Data Update Error</td>
<td>0x8125</td>
<td>Battery Discharge Over-temperature Warning</td>
</tr>
<tr>
<td>0x8027</td>
<td>Body Attitude Error</td>
<td>0x8126</td>
<td>Battery Discharge Low-temperature Warning</td>
</tr>
<tr>
<td>0x8028</td>
<td>Driver Status Error</td>
<td>0x8127</td>
<td>Battery Charging Over-temperature Warning</td>
</tr>
</tbody>
</table>

&lt;page_number&gt;31 / 35&lt;/page_number&gt;

---


## Page 32

DEEP Robotics

<table>
  <tr>
    <td><b>Value</b></td>
    <td><b>Meaning</b></td>
    <td><b>Value</b></td>
    <td><b>Meaning</b></td>
  </tr>
  <tr>
    <td>0x8029</td>
    <td>Motion Attitude Error</td>
    <td>0x8128</td>
    <td>Battery Charging Low-temperature Warning</td>
  </tr>
  <tr>
    <td>0x8030</td>
    <td>Joint Driver Over-temperature Warning</td>
    <td>0x8129</td>
    <td>Battery Output Minimum Voltage Warning</td>
  </tr>
  <tr>
    <td>0x8102</td>
    <td>Low Battery Warning</td>
    <td>0x8201</td>
    <td>CPU Usage Overload Warning</td>
  </tr>
  <tr>
    <td>0x8103</td>
    <td>Protected Battery Level</td>
    <td>0x8202</td>
    <td>CPU Temperature Overheat Warning</td>
  </tr>
  <tr>
    <td>0x8106</td>
    <td>Battery Output Minimum Voltage Protection</td>
    <td>0x8211</td>
    <td>CPU Usage Overload Protection</td>
  </tr>
  <tr>
    <td>0x8107</td>
    <td>Battery Discharge Over-temperature Protection</td>
    <td>0x8212</td>
    <td>CPU Temperature Overheat Protection</td>
  </tr>
</table>

&lt;page_number&gt;32 / 35&lt;/page_number&gt;

---


## Page 33

DEEP Robotics

# Appendix 1: UDP Sample Code

Example code for sending an autonomous charging command:

```c
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define SERVER_IP "10.21.31.103"
#define PORT 30000
#define BUFFER_SIZE 1024

struct udpMessage {
    unsigned char header[16];
    unsigned char data[BUFFER_SIZE];
};

int main() {
    // Create a UDP socket
    int client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_fd < 0) {
        perror("socket creation failed");
        return -1;
    }

    // Set server address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        close(client_fd);
        return -1;
    }

    udpMessage message;
    message.header[0] = 0xeb;
    message.header[1] = 0x90;
```

&lt;page_number&gt;33 / 35&lt;/page_number&gt;

---


## Page 34

DEEP Robotics

```cpp
39  message.header[2] = 0xeb;
40  message.header[3] = 0x90;

41

42  // JSON string for sending
43  const char *data = R"( 
44  { 
45      "PatrolDevice":{ 
46          "Type":2, 
47          "Command":24, 
48          "Time":"2023-01-01 00:00:00", 
49          "Items":{ 
50              } 
51          } 
52      } 
53  )";

54

55  unsigned short dataLength = strlen(data);

56

57  message.header[4] = dataLength & 0xFF;
58  message.header[5] = (dataLength >> 8) & 0xFF;
59  message.header[6] = 0x01;
60  message.header[7] = 0x00;
61  message.header[8] = 0x01;

62

63  memcpy(message.data, data, strlen(data));

64

65  // Send data
66  ssize_t send_len = sendto(client_fd, &message, dataLength + 16, 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
67  if (send_len < 0) {
68      perror("sendto failed");
69      close(client_fd);
70      return -1;
71  }
72

73  std::cout << "Message sent successfully." << std::endl;

74

75  // Close the socket
76  close(client_fd);
77  return 0;
78 }

79 }
```

&lt;page_number&gt;34 / 35&lt;/page_number&gt;

---


## Page 35

DEEP Robotics

# Appendix 2: Obtaining camera video stream

The front and rear wide-angle cameras of the Lynx M20 use the RTSP protocol for streaming. The RTSP addresses are as follows:

<table>
<thead>
<tr>
<th>Camera Position</th>
<th>RTSP Address</th>
</tr>
</thead>
<tbody>
<tr>
<td>Front Wide-angle Camera</td>
<td>rtsp://10.21.31.103:8554/video1</td>
</tr>
<tr>
<td>Rear Wide-angle Camera</td>
<td>rtsp://10.21.31.103:8554/video2</td>
</tr>
</tbody>
</table>

Developers can pull the RTSP video streams of the front and rear wide-angle cameras using the above addresses.

&lt;page_number&gt;35 / 35&lt;/page_number&gt;
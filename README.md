MotiveDirect Unity C#
============

This is a NatNet direct client running in Unity written in C# that allows you to receive tracking data sent from the OptiTrack Motive software directly without intermidiate NatNet SDK server or plug-in. Mostly rewritten from PacketClient.cpp in NatNet SDK and [Optirx](https://pypi.python.org/pypi/optirx).

If you just want to see your tracked object appears in Unity, use this. If you want to reduce the tracking network load, use this.  


## Configuration

![Motive Configuration](http://i.imgur.com/0b54qH9.png)

![Unity Configuration](http://i.imgur.com/SOiHm1L.png)

---

## New to Optitrack / Motive ?
Here are good references for you. 

[Optitrack Quick start Guide](http://wiki.optitrack.com/index.php?title=Quick_Start_Guide:_Getting_Started#Label_Data)

[Optitrack Data streaming](http://wiki.optitrack.com/index.php?title=Data_Streaming)

![Check NatNet Version](http://i.imgur.com/YVtQeVF.png)

[Check your NatNet Version] (https://www.optitrack.com/downloads/developer-tools.html#natnet-sdk)

---

## Features
- supported Motive 1.0 to 1.9 (NatNet 2.0 to 2.9)
- separated command thread for sending commands and receiving responses from Motive 
- separated data thread for receiving tracking data
- a tracking data buffer for keeping historical tracks
- store mappings of unreadable ID to readable names
- switch on/off for displaying debug cubes
- the gameobject in Unity is tagged "tracked" if the object can be recognized by Motive. otherwise it is tagged "untracked".

---

## Setup from Empty project
- Make an empty object in Unity and attach MotiveDirect.cs to it (as shwon in Configuration).
- Make sure you add "tracked" and "untracked" to your Tags in Unity.

---

## Usage

There are two easier ways to move your 3D model/gameobject. 

1. Use Gameobject.Find([name]) and let your 3D model/gameobject follows it. 
2. Name your 3D model/gameobject as the same name as the rigidbody in Motive. The system then will move your 3D model/gameobject instead of creating a new one. 

---

## License

This project is licensed under the terms of the **MIT** license.
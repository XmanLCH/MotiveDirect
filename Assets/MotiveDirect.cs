
//The MIT License (MIT)
//Copyright (c) 2016 Lung-Pan Cheng
//
//Permission is hereby granted, free of charge, to any person obtaining a copy of this 
//software and associated documentation files (the "Software"), to deal in the Software 
//without restriction, including without limitation the rights to use, copy, modify, merge, 
//publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
//to whom the Software is furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
//IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
//WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Runtime.InteropServices;
using System.Linq;


public class MotiveDirect : MonoBehaviour {
	public string multicastIP = "239.255.42.99";
	public string hostIP = "10.0.1.6";
	public int dataPort = 1511;
	public int commandPort = 1510;
	public string NatNetVersion = "2.7.0.0";
	public static double currentTimestamp = 0.0f;

	public bool showDebugObject = true;

	private IPEndPoint mRemoteIpEndPoint;
	private Socket     mDataListner;
	private byte[]     mDataReceiveBuffer;

	private Socket     mCommandListner;
	private byte[]     mCommandReceiveBuffer;

	private Thread dataThread;
	private Thread commandThread;
	private bool isRunning = false;

	private int[] versionNumbers;

	//Circular buffer for frame of data
	private FrameOfData[] dataBuffer;
	private int dataBufferSize = 360;
	private int dataBufferHead = -1;

	public float updateModelDefTime = 60.0f;
	private float modelDefTimer = 0;
	private bool modelDefUpdated = false;

	private static readonly object syncLock = new object();

	private Dictionary<int, string> rigidBodyIDtoName = new Dictionary<int, string>();
	private Dictionary<int, string> skeletonIDtoName = new Dictionary<int, string>();
	private Dictionary<int, Dictionary<int, string>> boneIDtoName = new Dictionary<int, Dictionary<int, string>>();
	private Dictionary<string, Dictionary<int, string>> markerSetIDtoName = new Dictionary<string, Dictionary<int, string>>();
	private Dictionary<string, Dictionary<int, Transform>> markerSetIDtoTransfrom = new Dictionary<string, Dictionary<int, Transform>>();
	private Dictionary<string, GameObject> gameObjectDictionary = new Dictionary<string, GameObject>();
	private List<GameObject> debugObjects = new List<GameObject>();
	//
	// Some constants as defined in NatNet SDK example 
	//

	// NATNET message ids (actually codes, they are not unique)
	private const int NAT_PING =                    0;
	private const int NAT_PINGRESPONSE =            1;
	private const int NAT_REQUEST =                 2;
	private const int NAT_RESPONSE =                3;
	private const int NAT_REQUEST_MODELDEF =        4;
	private const int NAT_MODELDEF =                5;
	private const int NAT_REQUEST_FRAMEOFDATA =     6;
	private const int NAT_FRAMEOFDATA =             7;
	private const int NAT_MESSAGESTRING =           8;
	private const int NAT_UNRECOGNIZED_REQUEST =    100;
	private const double UNDEFINED =                999999.9999;
	private readonly Dictionary<int, string> NAT_TYPES = new Dictionary<int, string> { 
		{NAT_PING, "ping"},
		{NAT_PINGRESPONSE, "pong"},
		{NAT_REQUEST, "request"},
		{NAT_RESPONSE, "response"},
		{NAT_REQUEST_MODELDEF, "request_modeldef"},
		{NAT_MODELDEF, "modeldef"},
		{NAT_REQUEST_FRAMEOFDATA, "request_frameofdata"},
		{NAT_FRAMEOFDATA, "frameofdata"},
		{NAT_MESSAGESTRING, "messagestring"},
		{NAT_UNRECOGNIZED_REQUEST, "unrecognized" }
	};


	private const int DATASET_MARKERSET =           0;  // PacketClient.cpp:778
	private const int DATASET_RIGIDBODY =           1;  // PacketClient.cpp:800
	private const int DATASET_SKELETON =            2;  // PacketClient.cpp:827


	private const int MAX_NAMELENGTH =              256;
	private const int MAX_PAYLOADSIZE =             100000;
	private const int SOCKET_BUFSIZE = 0x100000;




	//
	// NatNet packet format 
	//

	// sPacket struct (PacketClient.cpp:65)
	//  - iMessage (unsigned short),
	//  - nDataBytes (unsigned short),
	// - union of possible payloads (MAX_PAYLOADSIZE bytes)

	struct PACKET_HEADER_FORMAT{
		public ushort iMessage;
		public ushort nDataBytes;
	}
	struct PACKET_FORMAT{
		public PACKET_HEADER_FORMAT header;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_PAYLOADSIZE)]
		public byte[] payload;
	}
	private int MAX_PACKETSIZE = Marshal.SizeOf(default(PACKET_FORMAT));


	// sender payload struct (PacketClient.cpp:57)
	//  - szName (string MAX_NAMELENGTH),
	//  - Version (4 unsigned chars),
	//  - NatNetVersion (4 unsigned chars)
	struct SENDER_FORMAT{
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = MAX_PAYLOADSIZE)]
		public byte[] szName;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public byte[] Version;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public byte[] NatNetVersion;
	}
	struct SenderData{
		public string appname;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public int[] version;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public int natnet_version;
	}

	// rigid body payload (PacketClient.cpp:586)
	//  - id (int, 32 bits)
	//  - x,y,z (3 floats, 3x32 bits)
	//  - qx,qy,qz,qw (4 floats, 4x32 bits)
	struct RIGIDBODY_FORMAT{
		public int id;
		public Vector3 position;
		public Quaternion rotation;
	}


	// RigidBody:
	//   id is an integer
	//   position is a triple of coordinates
	//   orientation is a quaternion (qx, qy, qz, qw)
	//   markers is a list of triples
	//   mrk_ids is a list of integers or None (NatNet version < 2.0)
	//   mrk_sizes is a list of floats or None (NatNet version < 2.0)
	//   mrk_mean_error is a float or None (NatNet version < 2.0)
	//   tracking_valid is a boolean or None (NatNet version <= 2.6)
	struct RigidBody{
		public int id;
		public Vector3 position;
		public Quaternion rotation;
		public ArrayList markers;
		public ArrayList mrk_ids;
		public ArrayList mrk_sizes;
		public float mrk_mean_error;
		public bool tracking_valid;
	}

	// Skeleton (NetNet >= 2.1) is a collection of rigid bodies:
	struct Skeleton{
		public int id;
		public ArrayList rigid_bodies;
	}


	// LabeledMarker (NatNet >= 2.3)
	// New in NatNet 2.6 is support for:
	//   occluded, if the marker was not visible in this frame
	//   point_cloud_solved, if the position was provided by point cloud solver
	//   model_solved, if the position was provided by model solve
	// if version is older than 2.6 all values named above will be None
	struct LabeledMarker{
		public int id;
		public Vector3 position;
		public float size;
		public bool occluded;
		public bool point_cloud_solved;
		public bool model_solved;
	}


	// frame payload format (PacketClient.cpp:537) cannot be unpacked by
	// struct.unpack, because contains variable-length elements
	//  - frameNumber (int),
	//  - number of data sets nMarkerSets (int),
	//  - MARKERSETS, each of them:
	//     * null-terminated set name (max MAX_NAMELENGTH bytes),
	//     * marker count nMarkers (int),
	//     * MARKERS, each of them:
	//        + x (float),
	//        + y (float),
	//        + z (float),
	//  - UNIDENTIFIED_MARKERS:
	//     * nOtherMarkers (int),
	//     * MARKERS, each of them:
	//        + x (float),
	//        + y (float),
	//        + z (float),
	//  - RIGID_BODIES (...)
	//  - SKELETONS (...), ver >= 2.1
	//  - LABELED_MARKERS (...), ver >= 2.3
	//  - latency (float),
	//  - timecode (int, int),
	//  - timestamp (double), version >= 2.6(?),
	//  - is_recording (boolean), version >= 2.6(?),
	//  - tracked_models_changed (boolean), version >= 2.6(?),
	//  - end of data tag (int).
	struct FrameOfData{
		public int frameno;
		public Dictionary<string, ArrayList> sets;
		public ArrayList other_markers;
		public ArrayList rigid_bodies;
		public ArrayList skeletons;
		public ArrayList labeled_markers;
		public float latency;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
		public uint[] timecode;
		public double timestamp;
		public bool is_recording;
		public bool tracked_models_changed;
	}

	// type can be one of DATASET_MARKERSET, DATASET_RIGIDBODY, DATASET_SKELETON
	// name is a string (possibly empty)
	// data can be
	//   - a list of strings (names of the markers for a markerset)
	//   - a list of rigid bodies' dictionaries
	struct ModelDataset{
		public int type;
		public string name;
		public ArrayList data;
	}

	// defs is a list of ModelDataset elements
	struct ModelDefs{
		public ArrayList datasets;
	}

	// Use this for initialization
	void Start () {
		mDataReceiveBuffer = new byte[MAX_PACKETSIZE];
		mCommandReceiveBuffer = new byte[MAX_PACKETSIZE];

		//initialize data socket
		mRemoteIpEndPoint = new IPEndPoint(IPAddress.Parse(hostIP), commandPort);
		mDataListner = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
		mDataListner.Bind(new IPEndPoint(IPAddress.Any, dataPort));

		//join multicast group
		IPAddress mulIP=IPAddress.Parse(multicastIP);
		mDataListner.SetSocketOption(SocketOptionLevel.IP,
			SocketOptionName.AddMembership,
			new MulticastOption(mulIP,IPAddress.Any));
	
		mDataListner.Blocking          = false;
		mDataListner.ReceiveBufferSize = SOCKET_BUFSIZE;

		//initialize command socket
		mCommandListner = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
		mCommandListner.Bind(new IPEndPoint(IPAddress.Any, 0));

		//set socket to boradcast mode
		mCommandListner.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, 1);

		mCommandListner.Blocking = false;
		mCommandListner.ReceiveBufferSize = SOCKET_BUFSIZE;

		//create circular buffer for frame data
		dataBuffer = new FrameOfData[dataBufferSize];


		//get natnet version from Unity
		string[] versionChars =NatNetVersion.Split('.');
		versionNumbers = new int[4]{0,0,0,0};
		for(int i=0; i<4 ;i++){
			versionNumbers[i] = int.Parse (versionChars[i]);
		}

		startDataThread();
		startCommandThread();
	}

	private void startDataThread(){
		try
		{
			// define the thread and assign function for thread loop
			dataThread = new Thread(new ThreadStart(dataThreadLoop));
			// Boolean used to determine the thread is running
			isRunning = true;

			dataThread.Priority = System.Threading.ThreadPriority.Highest;
			// Start the thread
			dataThread.Start();
			Debug.Log("Data thread started");
		}
		catch (Exception ex)
		{
			// Failed to start thread
			Debug.Log("Error 3: " + ex.Message.ToString());
		}
	}
	private void startCommandThread(){
		try
		{
			// define the thread and assign function for thread loop
			commandThread = new Thread(new ThreadStart(commandThreadLoop));
			// Boolean used to determine the thread is running
			isRunning = true;

			commandThread.Priority = System.Threading.ThreadPriority.Highest;
			// Start the thread
			commandThread.Start();
			Debug.Log("Command thread started");
		}
		catch (Exception ex)
		{
			// Failed to start thread
			Debug.Log("Error 3: " + ex.Message.ToString());
		}
	}
	private void dataThreadLoop()
	{
		while (isRunning)
		{ dataRead(); }

		Debug.Log("Ending Thread!");
	}	
	private void commandThreadLoop()
	{
		while (isRunning)
		{ 
			if(!modelDefUpdated){
				_send_modeldef_request();
				Thread.Sleep(100); //flow control
			}
			commandRead(); 
		}

		Debug.Log("Ending Thread!");
	}
	public void StopUDPThread()
	{
		// Set isRunning to false to let the while loop
		// complete and drop out on next pass
		isRunning = false;

		// Pause a little to let this happen
		Thread.Sleep(100);

		// If the thread still exists kill it
		// A bit of a hack using Abort :p
		if (dataThread != null)
		{
			dataThread.Abort();
			// serialThread.Join();
			Thread.Sleep(100);
			dataThread = null;
		}

		if (commandThread != null)
		{
			commandThread.Abort();
			// serialThread.Join();
			Thread.Sleep(100);
			commandThread = null;
		}

		Debug.Log("Ended UDP Loop thread");
	}

	public void dataRead()
	{
		try
		{
			int bytesReceived = mDataListner.Receive(mDataReceiveBuffer);
			if(bytesReceived == 0) return;
			object msg;
			int msgtype;
			int offset = 0;
			unpack(mDataReceiveBuffer, ref offset, out msg, out msgtype, versionNumbers);

			if(msgtype == NAT_FRAMEOFDATA){
				lock(syncLock){
					dataBufferHead ++;
					if(dataBufferHead>=dataBufferSize) dataBufferHead = 0;
					dataBuffer[dataBufferHead] = (FrameOfData)msg;
					currentTimestamp = ((FrameOfData)msg).timestamp;
				}
			}
		}
		catch(System.Exception ex)
		{
		}
	}
	public void commandRead()
	{
		try
		{
			int bytesReceived = mCommandListner.Receive(mCommandReceiveBuffer);
			if(bytesReceived == 0) return;
			object msg;
			int msgtype;
			int offset = 0;
			unpack(mCommandReceiveBuffer, ref offset, out msg, out msgtype, versionNumbers);
			if(msgtype == NAT_FRAMEOFDATA){
				lock(syncLock){
					dataBufferHead ++;
					if(dataBufferHead>=dataBufferSize) dataBufferHead = 0;
					dataBuffer[dataBufferHead] = (FrameOfData)msg;
				}
			}
			else if(msgtype == NAT_MODELDEF){
				lock(syncLock){
					ModelDefs modelDef = (ModelDefs)msg;
					markerSetIDtoName.Clear();
					rigidBodyIDtoName.Clear();
					skeletonIDtoName.Clear();
					boneIDtoName.Clear();
					foreach (ModelDataset dataset in modelDef.datasets){
						if(dataset.type == DATASET_MARKERSET){
							Dictionary<int, string> setDict = null;
							if(!markerSetIDtoName.TryGetValue(dataset.name, out setDict)){
								setDict = new Dictionary<int, string>();
								markerSetIDtoName[dataset.name] = setDict;
							}
							int i=0;
							foreach(string name in dataset.data){
								setDict[i] = name;
								i++;
							}
						}
						else if(dataset.type == DATASET_RIGIDBODY){
							foreach(Dictionary<string, object> rb in dataset.data){
								rigidBodyIDtoName[(int)rb["id"]] = dataset.name;
							}
						}
						else if(dataset.type == DATASET_SKELETON){
							foreach(Dictionary<string, object> sk in dataset.data){
								Dictionary<int, string> skDict = null;
								int skid = (int)sk["skid"];
								if(!boneIDtoName.TryGetValue(skid, out skDict)){
									skDict = new Dictionary<int, string>();
									boneIDtoName[skid] = skDict;
								}
								int boneID = (int)sk["id"];
								skDict[boneID] = (string)sk["bname"];
								skeletonIDtoName[skid] = dataset.name;
							}
						}
						else{
							
						}
					}
				}
				modelDefUpdated = true;
			}
			else if(msgtype == NAT_PINGRESPONSE){
				Debug.Log("Received NatNet server ping response.");
			}
			else{
				//TODO: parse the other types
			}
		}
		catch(System.Exception ex)
		{
		}
	}

	void Update()
	{
		lock(syncLock){
			if(dataBufferHead !=-1){ // skip if no data comes in
				FrameOfData msg = dataBuffer[dataBufferHead];
				foreach(KeyValuePair<string, ArrayList> element in msg.sets){
					GameObject mSet = null;
					string setName = string.Concat(element.Key, "_set");
					if(!gameObjectDictionary.TryGetValue(setName, out mSet)){
						mSet = GameObject.Find(setName);
						if(mSet == null){
							mSet = new GameObject(setName);
							mSet.transform.parent = transform;
						}
						gameObjectDictionary[setName] = mSet;
					}
					Dictionary<int, Transform> transformDict = null;
					if(!markerSetIDtoTransfrom.TryGetValue(element.Key, out transformDict)){
						transformDict = new Dictionary<int, Transform>();
						markerSetIDtoTransfrom[element.Key] = transformDict;
					}
					Dictionary<int, string> nameDict = null;
					if(markerSetIDtoName.TryGetValue(element.Key, out nameDict)){
						int i=0;
						foreach(Vector3 marker in element.Value){
							string mkName = "";
							if(nameDict.TryGetValue(i, out mkName)){
								Transform mkTransform =  null;
								if(!transformDict.TryGetValue(i, out mkTransform)){
									mkTransform =  mSet.transform.Find(mkName);
									if (mkTransform == null){
										GameObject mk = new GameObject(mkName);
										mk.transform.parent = mSet.transform;
										GameObject debugObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
										debugObject.transform.localScale *= 0.02f;
										debugObject.transform.parent = mk.transform;
										debugObject.name = "debug";
										debugObjects.Add(debugObject);
										mkTransform = mk.transform;
									}
									transformDict[i] = mkTransform;
								}
								mkTransform.localPosition = convertToLeftHandPosition(marker);	
							}
							i++;
						}
					}
				}
				for(int i=0; i<msg.rigid_bodies.Count; i++){
					RigidBody rbody = (RigidBody)msg.rigid_bodies[i];
					GameObject rb = null;
					string rbName = "";
					if(rigidBodyIDtoName.TryGetValue(rbody.id, out rbName)){
						if(!gameObjectDictionary.TryGetValue(rbName, out rb)){
							rb = GameObject.Find(rbName);
							if(rb == null){
								rb = new GameObject(rbName);
								rb.transform.parent = transform;
								GameObject debugObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
								debugObject.transform.localScale *= 0.1f;
								debugObject.transform.parent = rb.transform;
								debugObject.name = "debug";
								debugObjects.Add(debugObject);
							}
							gameObjectDictionary[rbName] = rb;
						}
						rb.transform.localPosition = convertToLeftHandPosition(rbody.position);
						rb.transform.localRotation = convertToLeftHandRotation(rbody.rotation);
						rb.tag = rbody.tracking_valid? "tracked" : "untracked";
					}
				}
				for(int i=0; i<msg.skeletons.Count; i++){
					Skeleton skeleton = (Skeleton)msg.skeletons[i];
					GameObject sk = null;
					string skName = "";
					if(skeletonIDtoName.TryGetValue(skeleton.id, out skName)){
						if(!gameObjectDictionary.TryGetValue(skName, out sk)){
							sk = GameObject.Find(skName);
							if(sk == null){
								sk = new GameObject(skName);
								sk.transform.parent = transform;
								sk.transform.localPosition = Vector3.zero;
								sk.transform.localRotation = Quaternion.identity;
							}
							gameObjectDictionary[skName] = sk;
						}
					}
					for(int j=0; j< skeleton.rigid_bodies.Count ;j++){
						RigidBody rbody = (RigidBody)skeleton.rigid_bodies[j];
						int skeletonID = HighWord(rbody.id);
						int boneID = LowWord(rbody.id);
						Dictionary<int, string> skeletonDict = null;
						string boneName = "";
						GameObject bone = null;
						if(boneIDtoName.TryGetValue(skeletonID, out skeletonDict) && skeletonDict.TryGetValue(boneID, out boneName)){
							if(!gameObjectDictionary.TryGetValue(boneName, out bone)){
								bone = GameObject.Find(boneName);
								if(bone == null){
									bone = new GameObject(boneName);
									bone.transform.parent = sk.transform;
									GameObject debugObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
									debugObject.transform.localScale *= 0.1f;
									debugObject.transform.parent = bone.transform;
									debugObject.name = "debug";
									debugObjects.Add(debugObject);
								}
								gameObjectDictionary[boneName] = bone;
							}
							bone.transform.localPosition = convertToLeftHandPosition(rbody.position);
							bone.transform.localRotation = convertToLeftHandRotation(rbody.rotation);
							bone.tag = rbody.tracking_valid? "tracked" : "untracked";
						}
					}
				}
			}
		}

		//update model definition once a period
		modelDefTimer -= Time.deltaTime;
		if(modelDefTimer<=0){
			modelDefUpdated = false;
			modelDefTimer = updateModelDefTime;
		}

		//show debug objects
		foreach(GameObject debugObject in debugObjects){
			debugObject.SetActive(showDebugObject);
		}
	}

	void OnDestroy(){
		Thread.Sleep(500);
		StopUDPThread();
		Thread.Sleep(500);
	}

	void OnApplicationQuit(){
		Thread.Sleep(500);
		StopUDPThread();
		Thread.Sleep(500);
	}

	private void _send_modeldef_request(){
		if(mCommandListner != null){
			PACKET_FORMAT packet = new PACKET_FORMAT();
			packet.header.iMessage = NAT_REQUEST_MODELDEF;
			packet.header.nDataBytes = 0;
			byte[] msg = new byte[4];
			IntPtr ptr = Marshal.AllocHGlobal(4);
			Marshal.StructureToPtr(packet, ptr, true);
			Marshal.Copy(ptr, msg, 0,4);
			mCommandListner.SendTo(msg, mRemoteIpEndPoint);
			Marshal.FreeHGlobal(ptr);
		}
	}
		
	private bool _version_is_at_least(int[] version, int major, int minor=-1){
		int vmajor = version[0];
		int vminor = version[1];
		return (vmajor > major) || ((vmajor == major) && (vminor >=minor));
	}

	private void _unpack_head<T>(byte[] payload, ref int offset, out T val){
		int sz = Marshal.SizeOf(default(T));
		GCHandle handle = GCHandle.Alloc(payload, GCHandleType.Pinned);
		IntPtr head = new IntPtr(handle.AddrOfPinnedObject().ToInt64() + offset);
		val = (T)Marshal.PtrToStructure(head, typeof(T));
		handle.Free();
		offset += sz;
	}

	private void _unpack_cstring(byte[] payload, ref int offset, int maxstrlen, out string s){
		GCHandle handle = GCHandle.Alloc(payload, GCHandleType.Pinned);
		IntPtr head = new IntPtr(handle.AddrOfPinnedObject().ToInt64() + offset);
		s = (string)Marshal.PtrToStringAnsi(head);
		handle.Free();
		offset += s.Length+1; //include null character
	}

	private void _unpack_sender(byte[] payload, ref int offset, out SenderData sender){
		_unpack_head<SenderData>(payload, ref offset, out sender);
	}

	private void _unpack_markers(byte[] payload, ref int offset, out ArrayList markers){
		int nmarkers;
		_unpack_head<int>(payload, ref offset, out nmarkers);
		markers = new ArrayList();
		for(int i=0; i<nmarkers ; i++){
			Vector3 pos;
			_unpack_head<Vector3>(payload, ref offset, out pos);
			markers.Add(pos);
		}
	}

	private void _unpack_rigid_bodies(byte[] payload, ref int offset, int[] version, out ArrayList rbodies){
		int nbodies;
		_unpack_head<int>(payload, ref offset, out nbodies);
		rbodies = new ArrayList();
		for(int i=0; i<nbodies ;i++){
			RIGIDBODY_FORMAT rbody;
			_unpack_head<RIGIDBODY_FORMAT>(payload, ref offset, out rbody);
			ArrayList markers;
			_unpack_markers(payload,ref offset, out markers);


			ArrayList mrk_ids = new ArrayList();
			ArrayList mrk_sizes = new ArrayList();
			float mrk_mean_error = -1.0f;
			bool tracking_valid = false;

			if(_version_is_at_least(version, 2, 0)){
				int nmarkers = markers.Count;
				for(int j=0; j<nmarkers ;j++){
					int id;
					_unpack_head<int>(payload, ref offset, out id);
					mrk_ids.Add(id);
				}
				for(int j=0; j<nmarkers ;j++){
					float size;
					_unpack_head<float>(payload, ref offset, out size);
					mrk_sizes.Add(size);
				}
				_unpack_head<float>(payload, ref offset, out mrk_mean_error);
				if(_version_is_at_least(version, 2, 6)){
					short param;
					_unpack_head<short>(payload, ref offset, out param);
					tracking_valid = (param & 0x01) ==1;
				}
			}
			RigidBody rb = new RigidBody();
			rb.id = rbody.id;
			rb.position = rbody.position;
			rb.rotation = rbody.rotation;
			rb.markers = markers;
			rb.mrk_ids = mrk_ids;
			rb.mrk_sizes = mrk_sizes;
			rb.mrk_mean_error = mrk_mean_error;
			rb.tracking_valid = tracking_valid;
			rbodies.Add(rb);
		}
	}

	private void _unpack_skeletons(byte[] payload, ref int offset, int[] version, out ArrayList skeletons){
		skeletons = new ArrayList();
		if (!_version_is_at_least(version, 2,1)){
			return;
		}
		int nskels;
		_unpack_head<int>(payload, ref offset, out nskels);
		for(int i=0; i<nskels; i++){
			int skelid;
			_unpack_head<int>(payload, ref offset, out skelid);
			ArrayList rbodies;
			_unpack_rigid_bodies(payload, ref offset, version, out rbodies);
			Skeleton skel = new Skeleton();
			skel.id = skelid;
			skel.rigid_bodies = rbodies;
			skeletons.Add(skel);
		}
	}

	private void _unpack_labeled_markers(byte[] payload, ref int offset, int[] version, out ArrayList lmarkers){
		lmarkers = new ArrayList();
		if(!_version_is_at_least(version, 2, 3)){
			return;
		}
		int nmarkers;
		_unpack_head<int>(payload, ref offset, out nmarkers);
		if(_version_is_at_least(version, 2, 6)){
			for(int i=0; i<nmarkers ;i++){
				int id;
				_unpack_head<int>(payload,ref offset, out id);
				Vector3 position; 
				_unpack_head<Vector3>(payload, ref offset, out position);
				float size;
				_unpack_head<float>(payload, ref offset, out size);
				short param;
				_unpack_head<short>(payload, ref offset, out param);

				bool occluded = (param & 0x01) == 1;
				bool pc_solved = (param & 0x02) == 2;
				bool model_solved = (param & 0x04) == 4;
				LabeledMarker lmarker = new LabeledMarker();
				lmarker.id = id;
				lmarker.position = position;
				lmarker.size = size;
				lmarker.occluded = occluded;
				lmarker.point_cloud_solved = pc_solved;
				lmarker.model_solved = model_solved;
				lmarkers.Add(lmarker);
			}
		}
		else{
			for(int i=0; i<nmarkers ;i++){
				int id;
				_unpack_head<int>(payload, ref offset, out id);
				Vector3 position; 
				_unpack_head<Vector3>(payload, ref offset, out position);
				float size;
				_unpack_head<float>(payload, ref offset, out size);
				short param;
				_unpack_head<short>(payload, ref offset, out param);
				LabeledMarker lmarker = new LabeledMarker();
				lmarker.id = id;
				lmarker.position = position;
				lmarker.size = size;
				lmarkers.Add(lmarker);
			}
		}
	}

	private void _unpack_force_plates (byte[] payload, ref int offset, int[] version, out ArrayList force_plates){
		force_plates = new ArrayList();
		if(!_version_is_at_least(version, 2, 9)){
			return;
		}
		int nplates;
		_unpack_head<int>(payload, ref offset, out nplates);
		if(nplates >0)
			Debug.LogWarning("Force plate data not supported");
	}

	private void _unpack_frameofdata(byte[] payload, ref int offset, int[] version, out FrameOfData fod){
		int frameno;
		int nsets;

		_unpack_head<int>(payload, ref offset, out frameno);
		_unpack_head<int>(payload, ref offset, out nsets);
		Dictionary<string, ArrayList> sets = new Dictionary<string, ArrayList>();
		string setname;
		ArrayList markers;
		ArrayList rbodies;
		ArrayList skels;
		ArrayList lmarkers;
		ArrayList forceplates;
		for(int i=0; i<nsets; i++){
			_unpack_cstring(payload, ref offset, MAX_NAMELENGTH, out setname);
			_unpack_markers(payload, ref offset, out markers);
			if(sets.ContainsKey(setname)) sets[setname] = markers;
			else sets.Add(setname, markers);
		}
		_unpack_markers(payload, ref offset, out markers);
		_unpack_rigid_bodies(payload, ref offset, version, out rbodies);
		_unpack_skeletons(payload, ref offset, version, out skels);
		_unpack_labeled_markers(payload, ref offset, version, out lmarkers);
		_unpack_force_plates(payload, ref offset, version, out forceplates);
		float latency;
		uint timecode;
		uint timecode_sub;
		double timestamp = 0;
		float timestampf = 0;
		short param = 0;
		bool is_recording = false;
		bool tracked_models_changed = false;
		if(_version_is_at_least(version, 2, 7)){
			_unpack_head<float>(payload, ref offset, out latency);
			_unpack_head<uint>(payload, ref offset, out timecode);
			_unpack_head<uint>(payload, ref offset, out timecode_sub);
			_unpack_head<double>(payload, ref offset, out timestamp);
			_unpack_head<short>(payload, ref offset, out param);
			is_recording = (param & 0x01) ==1;
			tracked_models_changed = (param & 0x02) ==2;
		}
		else if(_version_is_at_least(version, 2, 6)){
			_unpack_head<float>(payload, ref offset, out latency);
			_unpack_head<uint>(payload, ref offset, out timecode);
			_unpack_head<uint>(payload, ref offset, out timecode_sub);
			_unpack_head<float>(payload, ref offset, out timestampf);
			_unpack_head<short>(payload, ref offset, out param);
			timestamp = timestampf;
			is_recording = (param & 0x01) ==1;
			tracked_models_changed = (param & 0x02) ==2;
		}
		else{
			_unpack_head<float>(payload, ref offset, out latency);
			_unpack_head<uint>(payload, ref offset, out timecode);
			_unpack_head<uint>(payload, ref offset, out timecode_sub);
		}
		int eod;
		_unpack_head<int>(payload, ref offset, out eod);
		if(eod != 0) Debug.LogError("End-of-data marker is not 0.");
		fod = new FrameOfData();
		fod.frameno = frameno;
		fod.sets = sets;
		fod.other_markers = markers;
		fod.rigid_bodies = rbodies;
		fod.skeletons = skels;
		fod.labeled_markers = lmarkers;
		fod.latency = latency;
		fod.timecode = new uint[2];
		fod.timecode[0] = timecode;
		fod.timecode[1] = timecode_sub;
		fod.timestamp = timestamp;
		fod.is_recording = is_recording;
		fod.tracked_models_changed = tracked_models_changed;
	}
	private void _unpack_modeldef (byte[] payload, ref int offset, int[] version, out ModelDefs modeldef){
		int ndatasets;
		_unpack_head<int>(payload, ref offset, out ndatasets);
		ArrayList datasets = new ArrayList();
		for (int i=0; i< ndatasets ;i++){
			int dtype;
			_unpack_head<int>(payload, ref offset,  out dtype);
			string name;
			int nmarkers; 
			ModelDataset dset;
			ArrayList mrk_names;
			if(dtype == DATASET_MARKERSET){
				_unpack_cstring(payload, ref offset,  MAX_NAMELENGTH, out name);
				_unpack_head<int>(payload, ref offset, out nmarkers);
				mrk_names = new ArrayList();
				for (int j=0 ; j<nmarkers ;j++){
					string mrk_name;
					_unpack_cstring(payload, ref offset, MAX_NAMELENGTH, out mrk_name);
					mrk_names.Add(mrk_name);
				}
				dset = new ModelDataset();
				dset.type = DATASET_MARKERSET;
				dset.name = name;
				dset.data = mrk_names;
				datasets.Add(dset);
			}
			else if(dtype == DATASET_RIGIDBODY){
				if(_version_is_at_least(version, 2, 0)){
					_unpack_cstring(payload, ref offset, MAX_NAMELENGTH, out name);
				}
				else{
					name = "";
				}
				int rbid;
				int parent;
				Vector3 pos_offset;
				_unpack_head<int>(payload, ref offset, out rbid);
				_unpack_head<int>(payload, ref offset, out parent);
				_unpack_head<Vector3>(payload, ref offset, out pos_offset);
				dset = new ModelDataset();
				dset.type = DATASET_RIGIDBODY;
				dset.name = name;
				dset.data = new ArrayList();
				dset.data.Add(
					new Dictionary<string, object>(){
					{"id", rbid},
					{"parent", parent},
					{"offset", pos_offset}
					}
				);
				datasets.Add(dset);
			}
			else if(dtype == DATASET_SKELETON){
				_unpack_cstring(payload, ref offset, MAX_NAMELENGTH, out name);
				int skid, nbodies;
				string bname;
				_unpack_head<int>(payload, ref offset, out skid);
				_unpack_head<int>(payload, ref offset, out nbodies);
				ArrayList bodies = new ArrayList();
				for(int j=0 ; j<nbodies; j++){
					if(_version_is_at_least(version, 2, 0)){
						_unpack_cstring(payload, ref offset, MAX_NAMELENGTH, out bname);
					}
					else{
						bname = "" ;
					}
					int rbid;
					int parent;
					Vector3 pos_offset;
					_unpack_head<int>(payload, ref offset, out rbid);
					_unpack_head<int>(payload, ref offset, out parent);
					_unpack_head<Vector3>(payload, ref offset, out pos_offset);
					Dictionary<string, object> body = new Dictionary<string, object>(){
						{"id", rbid},
						{"bname", bname},
						{"parent", parent},
						{"offset", pos_offset},
						{"skid", skid}
					};
					bodies.Add(body);
				}
				dset = new ModelDataset();
				dset.type = DATASET_SKELETON;
				dset.name = name;
				dset.data = bodies;
				datasets.Add(dset);

			}
			else{
				Debug.LogError("Noo implemented Error-- dataset type" + dtype);
			}
		}
		modeldef = new ModelDefs();
		modeldef.datasets = datasets;
	}

	private void unpack(byte[] payload, ref int offset, out object msg, out int msgtype, int[] version = null){
		version = version ?? new int[4]{2,5,0,0};
		if(payload == null || payload.Length <4){
			msg = null;
			msgtype = -1;
			Debug.Log("no payload");
			return;
		}
		PACKET_HEADER_FORMAT packetHeader;
		_unpack_head<PACKET_HEADER_FORMAT>(payload, ref offset, out packetHeader);
		if(packetHeader.iMessage == NAT_PINGRESPONSE){
			SenderData sender;
			_unpack_sender(payload, ref offset, out sender);
			msg = sender;
			msgtype = NAT_PINGRESPONSE;
			return;
		}
		else if(packetHeader.iMessage == NAT_FRAMEOFDATA){
			FrameOfData frame;
			_unpack_frameofdata(payload, ref offset, version, out frame);
			msg = frame;
			msgtype = NAT_FRAMEOFDATA;
			return;
		}
		else if(packetHeader.iMessage == NAT_MODELDEF){
			ModelDefs modelDef; 
			_unpack_modeldef(payload, ref offset, version, out modelDef);
			msg = modelDef;
			msgtype = NAT_MODELDEF;
			return;
		}
		else if(packetHeader.iMessage == NAT_RESPONSE){
			string sz; 
			_unpack_cstring(payload, ref offset, MAX_PAYLOADSIZE, out sz);
			msg = sz;
			msgtype = NAT_RESPONSE;
			Debug.Log("Response : " + sz);
		}
		else if(packetHeader.iMessage == NAT_UNRECOGNIZED_REQUEST){
			msg = null;
			msgtype = NAT_UNRECOGNIZED_REQUEST;
			Debug.Log("[Client] received 'unrecognized request'");
		}
		else if(packetHeader.iMessage == NAT_MESSAGESTRING){
			string sz; 
			_unpack_cstring(payload, ref offset, MAX_PAYLOADSIZE, out sz);
			msg = sz;
			msgtype =NAT_MESSAGESTRING;
			Debug.Log("[Client] Received message:" + sz);
		}
		else{
			msg = null;
			msgtype = -1;
			Debug.LogError("Not implemented error -- packet type" + packetHeader.iMessage);
		}
	}
	public int LowWord(int number)
	{
		return number & 0xFFFF; 
	}

	public int HighWord(int number)
	{
		return ((number >> 16) & 0xFFFF); 
	}

	public Vector3 convertToLeftHandPosition(Vector3 p){
		return new Vector3(-p.x, p.y, p.z);
	}
	public Quaternion convertToLeftHandRotation(Quaternion q){
		return new Quaternion(-q.x, q.y, q.z, -q.w);
	}
}

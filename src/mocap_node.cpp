/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include "mocap_optitrack/socket.h"
#include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/skeletons.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>

// System includes
#include <string>
#include <unistd.h>
#include <fstream>
////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP_KEY = "optitrack_config/multicast_address";
const std::string MULTICAST_IP_DEFAULT = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char ** DEFAULT_MOCAP_MODEL = OBJECT;
//const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

#define COMMAND_PORT			1510
#define LOCAL_PORT			1511

// NATNET message ids
#define NAT_PING			0
#define NAT_PINGRESPONSE		1
#define NAT_REQUEST			2
#define NAT_RESPONSE			3
#define NAT_REQUEST_MODELDEF		4
#define NAT_MODELDEF			5
#define NAT_REQUEST_FRAMEOFDATA		6
#define NAT_FRAMEOFDATA			7
#define NAT_MESSAGESTRING		8
#define NAT_UNRECOGNIZED_REQUEST	100

#define UNDEFINED			999999.9999
#define MAX_PACKETSIZE			100000  // max size of packet (actual packet size is dynamic)
#define MAX_NAMELENGTH			256

#pragma pack(push, 1)
// sender

typedef struct {
  char szName[MAX_NAMELENGTH];            // sending app's name
  uint8_t Version[4];    // sending app's version [major.minor.build.revision]
  uint8_t NatNetVersion[4]; // sending app's NatNet version [major.minor.build.revision]
} sSender;

typedef struct {
  sSender Common;

  uint64_t HighResClockFrequency; // host's high resolution clock frequency (ticks per second)
  uint16_t DataPort;
  bool IsMulticast;
  uint8_t MulticastGroupAddress[4];
} sSender_Server;

typedef struct {
  uint16_t iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
  uint16_t nDataBytes;              // Num bytes in payload
  union {
    uint8_t cData[MAX_PACKETSIZE];
    char szData[MAX_PACKETSIZE];
    uint32_t lData[MAX_PACKETSIZE / sizeof(uint32_t)];
    float fData[MAX_PACKETSIZE / sizeof(float)];
    sSender Sender;
    sSender_Server SenderServer;
  } Data;                                 // Payload
} sPacket;

#pragma pack(pop)
////////////////////////////////////////////////////////////////////////

void processMocapData(const char** mocap_model,
  RigidBodyMap& published_rigid_bodies, const std::string& multicast_ip) {
  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, multicast_ip);

  uint16_t payload_len;
  uint32_t numberOfPackets = 0;
  int32_t nver[4] = { 0, 0, 0, 0 }; // natnet version
  int32_t sver[4] = { 0, 0, 0, 0 }; // server version

  sPacket PacketOut;
  PacketOut.iMessage = NAT_PING;
  PacketOut.nDataBytes = 0;

  ROS_INFO("Start processMocapData");

  bool version = false;
  uint32_t numBytes = 0;
  sPacket PacketIn;

  while (ros::ok()) {
    bool packetread = false;

    if (!version) {
      int iRet = multicast_client_socket.send((char*) &PacketOut,
	  4 + PacketOut.nDataBytes, COMMAND_PORT);
    }

    // Receive data from mocap device
    numBytes = multicast_client_socket.recv();

    // Parse mocap data
    if (numBytes > 0) {
      const char* buffer = multicast_client_socket.getBuffer();
      memcpy((char*) &PacketIn, buffer, numBytes);
      unsigned short header = *((unsigned short*) (&buffer[0])); // 2-bytes, ushort.

      // Look for the beginning of a NatNet package
      if (header == NAT_FRAMEOFDATA && version && 0 == 1) {
	payload_len = *((unsigned short*) &buffer[2]);  // 2-bytes.
	MoCapDataFormat format(buffer, payload_len);
	format.nnVer.major = nver[0];
	format.nnVer.minor = nver[1];
	format.parse();
	packetread = true;
	numberOfPackets++;

	if (format.model.numRigidBodies > 0) {
	  for (int i = 0; i < format.model.numRigidBodies; i++) {
	    uint32_t ID = format.model.rigidBodies[i].ID;
	    RigidBodyMap::iterator item = published_rigid_bodies.find(ID);

	    if (item != published_rigid_bodies.end()) {
	      item->second.publish(format.model.rigidBodies[i]);
	    }
	  }
	}
      }
      else if (header == NAT_PINGRESPONSE) {
	ROS_DEBUG("Header : %d, %d", header, PacketIn.iMessage);
	ROS_DEBUG("nData : %d", PacketIn.nDataBytes);

	for (int i = 0; i < 4; ++i) {
	  nver[i] = (int) PacketIn.Data.Sender.NatNetVersion[i];
	  sver[i] = (int) PacketIn.Data.Sender.Version[i];
	}

	ROS_INFO_ONCE("NATNet Version : %d.%d.%d.%d", nver[0], nver[1], nver[2], nver[3]);
	ROS_INFO_ONCE("Server Version : %d.%d.%d.%d", sver[0], sver[1], sver[2], sver[3]);
	version = true;
      }
      else if (header == NAT_MESSAGESTRING) {
	ROS_INFO("Received message: %s", PacketIn.Data.szData);
      }
      // else skip packet
    }

    // Don't try again immediately
    if (!packetread) {
      usleep(10);
    }
  }
}

////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n("~");

  // Get configuration from ROS parameter server
  const char** mocap_model(DEFAULT_MOCAP_MODEL);
  if (n.hasParam(MOCAP_MODEL_KEY)) {
    std::string tmp;
    if (n.getParam(MOCAP_MODEL_KEY, tmp)) {
      if (tmp == "SKELETON_WITH_TOES")
	mocap_model = SKELETON_WITH_TOES;
      else if (tmp == "SKELETON_WITHOUT_TOES")
	mocap_model = SKELETON_WITHOUT_TOES;
      else if (tmp == "OBJECT")
	mocap_model = OBJECT;
    }
  }

  // Get configuration from ROS parameter server
  std::string multicast_ip(MULTICAST_IP_DEFAULT);
  if (n.hasParam(MULTICAST_IP_KEY)) {
    n.getParam(MULTICAST_IP_KEY, multicast_ip);
  }
  else {
    ROS_WARN_STREAM(
	"Could not get multicast address, using default: " << multicast_ip);
  }

  RigidBodyMap published_rigid_bodies;

  if (n.hasParam(RIGID_BODIES_KEY)) {
    XmlRpc::XmlRpcValue body_list;
    n.getParam("rigid_bodies", body_list);
    if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct
	&& body_list.size() > 0) {
      XmlRpc::XmlRpcValue::iterator i;
      for (i = body_list.begin(); i != body_list.end(); ++i) {
	if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
	  PublishedRigidBody body(i->second);
	  string id = (string&) (i->first);
	  RigidBodyItem item(atoi(id.c_str()), body);

	  std::pair<RigidBodyMap::iterator, bool> result =
	      published_rigid_bodies.insert(item);
	  if (!result.second) {
	    ROS_ERROR("Could not insert configuration for rigid body ID %s",
		id.c_str());
	  }
	}
      }
    }
  }

  ROS_INFO("Multicast Group: %s", multicast_ip.c_str());
  // Process mocap data until SIGINT
  processMocapData(mocap_model, published_rigid_bodies, multicast_ip);

  return 0;
}

//        ofstream myFile;
//        myFile.open ("~/testData.txt", ios::out | ios::binary);
//        myFile.write (buffer, numBytes);
//        myFile.close();

//        myFile2.open ("~/testData2.txt", ios::out | ios::binary | ios::app);
//        myFile2.write ((char*)&PacketIn, sizeof(PacketIn));
//        myFile2.write ((char*)&PacketIn.iMessage, sizeof(PacketIn.iMessage));
//        myFile2.write ((char*)&tempInt, sizeof(int));
//        myFile2.write ((char*)&PacketIn.nDataBytes, sizeof(PacketIn.nDataBytes));
//        myFile2.write ((char*)&tempInt, sizeof(int));
//        myFile2.write ((char*)&PacketIn.Data.Sender.NatNetVersion, sizeof(PacketIn.Data.Sender.NatNetVersion));
//        myFile2.write ((char*)&tempInt, sizeof(int));
//        myFile2.write ((char*)&PacketIn.Data.Sender.Version, sizeof(PacketIn.Data.Sender.Version));
//        myFile2.write ((char*)&tempInt, sizeof(int));
//        myFile2.write ((char*)&PacketIn.Data.Sender, sizeof(PacketIn.Data.Sender));
//        myFile2.write ((char*)&tempInt, sizeof(int));
//        myFile2.close();
//        ROS_DEBUG("Header : %d, %d", header, PacketIn.iMessage);
//        ROS_DEBUG("nData : %d", PacketIn.nDataBytes);
//        ROS_DEBUG("numBytes : %d", numBytes);
//        ROS_DEBUG("packet : %s", buffer);
//        ROS_DEBUG("cData : %s", PacketIn.Data.cData);
//        ROS_DEBUG("szData : %s", PacketIn.Data.szData);
//        ROS_DEBUG("nData : %d", PacketIn.nDataBytes);
//        ROS_DEBUG("lData : %s", PacketIn.Data.lData);

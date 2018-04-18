#include "mocap_optitrack/mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ros/console.h>
using namespace std;

RigidBody::RigidBody()
  : NumberOfMarkers(0), ID(0), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

const geometry_msgs::PoseStamped RigidBody::get_ros_pose(bool newCoordinates)
{
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.header.stamp = ros::Time::now();
  if (newCoordinates)
  {
    // Motive 1.7+ coordinate system
    ros_pose.pose.position.x = -pose.position.x;
    ros_pose.pose.position.y = pose.position.z;
    ros_pose.pose.position.z = pose.position.y;

    ros_pose.pose.orientation.x = -pose.orientation.x;
    ros_pose.pose.orientation.y = pose.orientation.z;
    ros_pose.pose.orientation.z = pose.orientation.y;
    ros_pose.pose.orientation.w = pose.orientation.w;
  }
  else
  {
    // y & z axes are swapped in the Optitrack coordinate system
    ros_pose.pose.position.x = pose.position.x;
    ros_pose.pose.position.y = -pose.position.z;
    ros_pose.pose.position.z = pose.position.y;

    ros_pose.pose.orientation.x = pose.orientation.x;
    ros_pose.pose.orientation.y = -pose.orientation.z;
    ros_pose.pose.orientation.z = pose.orientation.y;
    ros_pose.pose.orientation.w = pose.orientation.w;
  }
  return ros_pose;
}

bool RigidBody::has_data()
{
    static const char zero[sizeof(pose)] = { 0 };
    return memcmp(zero, (char*) &pose, sizeof(pose));
}

// SkeletonData::SkeletonData()
//   : numRigidBodies(0), rigidBodies(0)
// {
// }

// SkeletonData::~SkeletonData()
// {
//   delete[] rigidBodies;
// }

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  // : markerSets(0), otherMarkers(0), rigidBodies(0), skeletons(0), labelMarkers(0),
  //   numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0), numSkeletons(0), numLabelMarkers(0),
  //   latency(0.0)
  : markerSets(0), otherMarkers(0), rigidBodies(0), numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markerSets;
  delete[] otherMarkers;
  delete[] rigidBodies;
  // delete[] skeletons;
  // delete[] labelMarkers;
}

Version::Version()
  : v_major(0), v_minor(0), v_revision(0), v_build(0)
{
}

Version::Version(int major, int minor, int revision, int build)
  : v_major(major), v_minor(minor), v_revision(revision), v_build(build)
{
  std::ostringstream ostr;
  ostr << v_major << "." << v_minor << "." << v_revision << "." << v_build;
  v_string  = ostr.str();
}

Version::Version(const std::string& version)
  : v_string(version)
{
  std::sscanf(version.c_str(), "%d.%d.%d.%d", &v_major, &v_minor, &v_revision, &v_build);
}

Version::~Version()
{
}
void Version::setVersion(int major, int minor, int revision, int build)
{
  v_major = major;
  v_minor = minor;
  v_revision = revision;
  v_build = build;

}

const std::string& Version::getVersionString()
{
  return this->v_string;
}

bool Version::operator > (const Version& comparison)
{
  if (v_major > comparison.v_major)
    return true;
  if (v_minor > comparison.v_minor)
    return true;
  if (v_revision > comparison.v_revision)
    return true;
  if (v_build > comparison.v_build)
    return true;
  return false;
}
bool Version::operator < (const Version& comparison)
{
  if (v_major < comparison.v_major)
    return true;
  if (v_minor < comparison.v_minor)
    return true;
  if (v_revision < comparison.v_revision)
    return true;
  if (v_build < comparison.v_build)
    return true;
  return false;
}
bool Version::operator == (const Version& comparison)
{
  return v_major == comparison.v_major
      && v_minor == comparison.v_minor
      && v_revision == comparison.v_revision
      && v_build == comparison.v_build;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length)
  : packet(packet), length(length), frameNumber(0)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse()
{
  seek(4); // skip 4-bytes. message ID and packet size.

  // parse frame number
  read_and_seek(frameNumber);
  ROS_DEBUG("Frame number: %d", frameNumber);

  // count number of packetsets
  read_and_seek(model.numMarkerSets);
  model.markerSets = new MarkerSet[model.numMarkerSets];
  ROS_DEBUG("Number of marker sets: %d", model.numMarkerSets);

  for (int i = 0; i < model.numMarkerSets; i++)
  {
    strcpy(model.markerSets[i].name, packet);
    seek(strlen(model.markerSets[i].name) + 1);

    ROS_DEBUG("Parsing model named: %s", model.markerSets[i].name);

    // read number of markers that belong to the model
    read_and_seek(model.markerSets[i].numMarkers);
    ROS_DEBUG("Number of markers in set: %d", model.markerSets[i].numMarkers);
    model.markerSets[i].markers = new Marker[model.markerSets[i].numMarkers];

    for (int k = 0; k < model.markerSets[i].numMarkers; k++)
    {
      // read marker positions
      read_and_seek(model.markerSets[i].markers[k]);
      float x = model.markerSets[i].markers[k].positionX;
      float y = model.markerSets[i].markers[k].positionY;
      float z = model.markerSets[i].markers[k].positionZ;
      ROS_DEBUG("\t marker %d: [x=%3.2f,y=%3.2f,z=%3.2f]", k, x, y, z);
    }
  }

  // read number of 'other' markers. Unidentified markers. (cf. NatNet specs)
  read_and_seek(model.numOtherMarkers);
  model.otherMarkers = new Marker[model.numOtherMarkers];
  ROS_DEBUG("Number of markers not in sets: %d", model.numOtherMarkers);

  for (int l = 0; l < model.numOtherMarkers; l++)
  {
    // read positions of 'other' markers
    read_and_seek(model.otherMarkers[l]);
  }

  // read number of rigid bodies of the model
  read_and_seek(model.numRigidBodies);
  ROS_DEBUG("Number of rigid bodies: %d", model.numRigidBodies);

  model.rigidBodies = new RigidBody[model.numRigidBodies];
  for (int m = 0; m < model.numRigidBodies; m++)
  {
    // read id, position and orientation of each rigid body
    read_and_seek(model.rigidBodies[m].ID);
    read_and_seek(model.rigidBodies[m].pose);

    ROS_DEBUG("Rigid body ID: %d", model.rigidBodies[m].ID);
    ROS_DEBUG("pos: [%3.2f,%3.2f,%3.2f], ori: [%3.2f,%3.2f,%3.2f,%3.2f]",
        model.rigidBodies[m].pose.position.x,
        model.rigidBodies[m].pose.position.y,
        model.rigidBodies[m].pose.position.z,
        model.rigidBodies[m].pose.orientation.x,
        model.rigidBodies[m].pose.orientation.y,
        model.rigidBodies[m].pose.orientation.z,
        model.rigidBodies[m].pose.orientation.w);

//    // After Version 3.0 Marker data is in desription
//    if (NatNetVersion < Version("3.0"))
//    {
//      // get number of markers per rigid body
//      read_and_seek(model.rigidBodies[m].NumberOfMarkers);
//
//      ROS_DEBUG("Number of rigid body markers: %d", model.rigidBodies[m].NumberOfMarkers);
//
//      if (model.rigidBodies[m].NumberOfMarkers > 0)
//      {
//        model.rigidBodies[m].marker = new Marker [model.rigidBodies[m].NumberOfMarkers];
//
//        size_t byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(Marker);
//        memcpy(model.rigidBodies[m].marker, packet, byte_count);
//        seek(byte_count);
//
//        // skip marker IDs
//        byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(int);
//        seek(byte_count);
//
//        // skip marker sizes
//        byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(float);
//        seek(byte_count);
//      }
//    }

    // Skip padding inserted by the server
    seek(sizeof(int));

    // skip mean marker error, version > 2.0
    seek(sizeof(float));

    // 2.6 or later.
    if (NatNetVersion > Version("2.6"))
    {
      seek(sizeof(short));
    }

  }

  // // skeletons, version 2.1 and later
  // read_and_seek(model.numSkeletons);
  // model.skeletons = new SkeletonData[model.numSkeletons];
  // ROS_DEBUG("Number of skeletons: %d", model.numSkeletons);
  //
  // for (int i = 0; i < model.numSkeletons; i++)
  // {
  //   read_and_seek(model.skeletons[i].ID);
  //   read_and_seek(model.skeletons[i].numRigidBodies);
  //
  //   ROS_DEBUG("Number of rigid bodies in skeleton #%d is %d", model.numSkeletons, model.skeletons[i].numRigidBodies);
  //
  //   model.skeletons[i].rigidBodies = new RigidBody[model.skeletons[i].numRigidBodies];
  //   for (int m = 0; m < model.skeletons[i].numRigidBodies; m++)
  //   {
  //     // read id, position and orientation of each rigid body
  //     read_and_seek(model.skeletons[i].rigidBodies[m].ID);
  //     read_and_seek(model.skeletons[i].rigidBodies[m].pose);
  //
  //     ROS_DEBUG("Rigid body ID: %d", model.skeletons[i].rigidBodies[m].ID);
  //     ROS_DEBUG("pos: [%3.2f,%3.2f,%3.2f], ori: [%3.2f,%3.2f,%3.2f,%3.2f]",
  //         model.skeletons[i].rigidBodies[m].pose.position.x,
  //         model.skeletons[i].rigidBodies[m].pose.position.y,
  //         model.skeletons[i].rigidBodies[m].pose.position.z,
  //         model.skeletons[i].rigidBodies[m].pose.orientation.x,
  //         model.skeletons[i].rigidBodies[m].pose.orientation.y,
  //         model.skeletons[i].rigidBodies[m].pose.orientation.z,
  //         model.skeletons[i].rigidBodies[m].pose.orientation.w);
  //
  //     // After Version 3.0 Marker data is in desription
  //     if (NatNetVersion < Version("3.0"))
  //     {
  //       // get number of markers per rigid body
  //       read_and_seek(model.skeletons[i].rigidBodies[m].NumberOfMarkers);
  //
  //       ROS_DEBUG("Number of rigid body markers: %d", model.skeletons[i].rigidBodies[m].NumberOfMarkers);
  //
  //       if (model.skeletons[i].rigidBodies[m].NumberOfMarkers > 0)
  //       {
  //         model.skeletons[i].rigidBodies[m].marker = new Marker [model.skeletons[i].rigidBodies[m].NumberOfMarkers];
  //
  //         size_t byte_count = model.skeletons[i].rigidBodies[m].NumberOfMarkers * sizeof(Marker);
  //         memcpy(model.skeletons[i].rigidBodies[m].marker, packet, byte_count);
  //         seek(byte_count);
  //
  //         // skip marker IDs
  //         byte_count = model.skeletons[i].rigidBodies[m].NumberOfMarkers * sizeof(int);
  //         seek(byte_count);
  //
  //         // skip marker sizes
  //         byte_count = model.skeletons[i].rigidBodies[m].NumberOfMarkers * sizeof(float);
  //         seek(byte_count);
  //       }
  //     }
  //
  //     // Skip padding inserted by the server
  //     seek(sizeof(int));
  //
  //     // skip mean marker error, version > 2.0
  //     seek(sizeof(float));
  //
  //     // 2.6 or later.
  //     if (NatNetVersion > Version("2.6"))
  //     {
  //       seek(sizeof(short));
  //     }
  //   }
  // }
  //
  // // Labeled markers, for version 2.3 and later
  // read_and_seek(model.numLabelMarkers);
  // model.labelMarkers = new LabelMarker[model.numLabelMarkers];
  // ROS_DEBUG("Number of labeled markers: %d", model.numLabelMarkers);
  //
  // for (int i = 0; i < model.numLabelMarkers; i++)
  // {
  //   read_and_seek(model.labelMarkers[i].ID);
  //   read_and_seek(model.labelMarkers[i].position);
  //   read_and_seek(model.labelMarkers[i].markSize);
  //
  //   seek(sizeof(short)); //skip param, version 2.6 and later
  //   seek(sizeof(int));   //skip residual, version 3.0 and later
  // }
  //
  // // Force Plate data (version 2.9 and later)
  // read_and_seek(model.numForcePlates);
  // model.forcePlates = new ForcePlateData[model.numForcePlates];
  // ROS_DEBUG("Number of force plates: %d", model.numForcePlates);
  //
  // for (int i = 0; i < model.numForcePlates; i++)
  // {
  //   read_and_seek(model.forcePlates[i].ID);
  //   read_and_seek(model.forcePlates[i].nChannels);
  //
  //   //skipping all frame data
  //   int tempFrameCount = 0;
  //   size_t byte_count = 0;
  //   for (int m = 0; m < model.forcePlates[i].nChannels; m++)
  //   {
  //     read_and_seek(tempFrameCount);
  //     byte_count = tempFrameCount * sizeof(int);
  //     seek(byte_count);
  //   }
  // }
  //
  // // Device data (version 2.11 and later)
  // read_and_seek(model.numDevices);
  // model.devices = new DeviceData[model.numDevices];
  // ROS_DEBUG("Number of Devices: %d", model.numDevices);
  //
  // for (int i = 0; i < model.numDevices; i++)
  // {
  //   read_and_seek(model.devices[i].ID);
  //   read_and_seek(model.devices[i].nChannels);
  //
  //   //skipping all frame data
  //   int tempFrameCount = 0;
  //   size_t byte_count = 0;
  //   for (int m = 0; m < model.devices[i].nChannels; m++)
  //   {
  //     read_and_seek(tempFrameCount);
  //     byte_count = tempFrameCount * sizeof(int);
  //     seek(byte_count);
  //   }
  // }
  //
  // // get all time parameters
  // read_and_seek(model.timeCode);
  // read_and_seek(model.timeCodeSubframe);
  // read_and_seek(model.fTimestamp);
  //
  // read_and_seek(model.cameraMidExposureTimestamp);
  // read_and_seek(model.cameraDataReceivedTimestamp);
  // read_and_seek(model.transmitTimestamp);
  //
  // // get frame parameters
  // read_and_seek(model.params);

}

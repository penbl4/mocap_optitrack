/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, University of Bonn, Computer Science Institute VI
 *  Author: Kathrin Gräve, 01/2011
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
#ifndef __MOCAP_DATAPACKETS_H__
#define __MOCAP_DATAPACKETS_H__

#include <sys/types.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

// #define MAX_ANALOG_CHANNELS         32      // maximum number of data channels (signals) per analog/force plate device
// #define MAX_ANALOG_SUBFRAMES        30      // maximum number of analog/force plate frames per mocap frame

#pragma pack(push, 1)
/// \brief Data object holding the position of a single mocap marker in 3d space
class Marker {
public:
  float positionX;
  float positionY;
  float positionZ;
};

class Pose {
public:
  struct {
    float x;
    float y;
    float z;
  } position;
  struct {
    float x;
    float y;
    float z;
    float w;
  } orientation;
};
#pragma pack(pop)

/// \brief Data object holding information about a single rigid body within a mocap skeleton
class RigidBody {
public:
  RigidBody() :
      ID(0), params(0) {
  }
  ~RigidBody() {
  }

  uint32_t ID;
  Pose pose;
  int16_t params;                         // Host defined tracking flags

  const geometry_msgs::PoseStamped get_ros_pose(bool newCoordinates);
  bool has_data();
};

/// \brief Data object holding information about a mocap skeleton
class SkeletonData {
public:
//     SkeletonData();
//     ~SkeletonData();
  SkeletonData() :
    ID(0), numRigidBodies(0), rigidBodies(0) {
  }
  ~SkeletonData() {
    delete[] rigidBodies;
  }

  uint32_t ID;

  int32_t numRigidBodies;
  RigidBody *rigidBodies;
};

/// \brief Data object describing a single tracked model
class ModelDescription {
public:
  ModelDescription() :
    numMarkers(0), markerNames(0) {
  }
  ~ModelDescription() {
    delete[] markerNames;
  }

  string name;
  int32_t numMarkers;
  string *markerNames;
};

class MarkerSet {
public:
  MarkerSet() :
      numMarkers(0), markers(0) {
  }
  ~MarkerSet() {
    delete[] markers;
  }

  char szName[256];
  int32_t numMarkers;
  Marker *markers;
};

/// \brief Data object holding poses of a tracked model's components
class ModelFrame {
public:
  ModelFrame() :
      markerSets(0), otherMarkers(0), rigidBodies(0), skeletons(0),
      numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0), numSkeletons(0) {
  }
  ~ModelFrame() {
    delete[] markerSets;
    delete[] otherMarkers;
    delete[] rigidBodies;
    delete[] skeletons;
  }

  MarkerSet *markerSets;
  Marker *otherMarkers;
  RigidBody *rigidBodies;
  SkeletonData *skeletons;
  // LabelMarker *labelMarkers;
  // ForcePlateData *forcePlates;
  // DeviceData *devices;

  int32_t numMarkerSets;
  int32_t numOtherMarkers;
  int32_t numRigidBodies;
  int32_t numSkeletons;
  // int numLabelMarkers;
  // int numForcePlates;
  // int numDevices;

  // unsigned int timeCode;                              // SMPTE timecode (if available)
  // unsigned int timeCodeSubframe;                      // timecode sub-frame data
  // double fTimestamp;                              // timestamp since software start ( software timestamp )
  // uint64_t cameraMidExposureTimestamp;            // Given in host's high resolution ticks
  // uint64_t cameraDataReceivedTimestamp;           // Given in host's high resolution ticks
  // uint64_t transmitTimestamp;                     // Given in host's high resolution ticks
  // int16_t params;                                     // host defined parameters
  //
  // int latency;
};

/// \brief Parser for a NatNet data frame packet
class MoCapDataFormat {
public:
  MoCapDataFormat(const char *packet, uint16_t length);
  ~MoCapDataFormat();

  /// \brief Parses a NatNet data frame packet as it is streamed by the Arena software according to the descriptions in the NatNet SDK v1.4. Should support up to 3.0 now
  void parse();

  const char *packet;
  uint16_t length;

  struct {
    int32_t major;
    int32_t minor;
  } nnVer;

  uint32_t frameNumber;
  ModelFrame model;

private:
  void seek(size_t count);
  template<typename T> void read_and_seek(T& target) {
    target = *((T*) packet);
    seek(sizeof(T));
  }
};


#endif  /*__MOCAP_DATAPACKETS_H__*/

// // Labeled markers
// class LabelMarker
// {
//   public:
//     int ID;                           // Unique identifier:
//
//     struct __attribute__ ((__packed__)) {
//       float x;                    // x position
//       float y;                    // y position
//       float z;                    // z position
//     } position;
//
//     float markSize;                     // marker size
//     int16_t params;                     // host defined parameters
//     float residual;                     // marker error residual, in mm/ray
// };

// // MarkerSet Definition
// class MarkerSetDescription
// {
//   public:
//     char name[256];            // MarkerSet name
//     int nMarkers;                       // # of markers in MarkerSet
//     char** szMarkerNames;                   // array of marker names
// };

// class AnalogChannelData
// {
//   public:
//     int nFrames;                            // # of analog frames of data in this channel data packet
//     float fValues[MAX_ANALOG_SUBFRAMES];     // values
// };
//
// // force plates
// class ForcePlateData
// {
//   public:
//     int ID;                                         // ForcePlate ID (from data description)
//     int nChannels;                                  // # of channels (signals) for this force plate
//     AnalogChannelData channelsData[MAX_ANALOG_CHANNELS];// Channel (signal) data (e.g. Fx[], Fy[], Fz[])
//     int16_t params;                                     // Host defined flags
// };
//
// class DeviceData
// {
//   public:
//     int ID;                                         // Device ID (from data description)
//     int nChannels;                                  // # of active channels (signals) for this device
//     AnalogChannelData channelsData[MAX_ANALOG_CHANNELS];// Channel (signal) data (e.g. ai0, ai1, ai2)
//     int16_t params;                                     // Host defined flags
// };

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

/// \brief Data object holding the position of a single mocap marker in 3d space
class Marker
{
  public:
    float positionX;
    float positionY;
    float positionZ;
};

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

class Pose
{
  public:
    struct __attribute__ ((__packed__)) {
      float x;
      float y;
      float z;
    } position;
    struct __attribute__ ((__packed__)) {
      float x;
      float y;
      float z;
      float w;
    } orientation;
};

/// \brief Data object holding information about a single rigid body within a mocap skeleton
class RigidBody
{
  public:
    RigidBody();
    ~RigidBody();

    int ID;

    Pose pose;

    int NumberOfMarkers;
    Marker *marker;

    const geometry_msgs::PoseStamped get_ros_pose(bool newCoordinates);
    bool has_data();
};

// /// \brief Data object holding information about a mocap skeleton
// class SkeletonData
// {
//   public:
//     SkeletonData();
//     ~SkeletonData();
//
//     int ID;
//
//     int numRigidBodies;
//     RigidBody *rigidBodies;
// };

/// \brief Data object describing a single tracked model
class ModelDescription
{
  public:
    ModelDescription();
    ~ModelDescription();

    string name;
    int numMarkers;
    string *markerNames;
};

class MarkerSet
{
  public:
    MarkerSet() : numMarkers(0), markers(0) {}
    ~MarkerSet() { delete[] markers; }
    char name[256];
    int numMarkers;
    Marker *markers;
};

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

/// \brief Data object holding poses of a tracked model's components
class ModelFrame
{
  public:
    ModelFrame();
    ~ModelFrame();

    MarkerSet *markerSets;
    Marker *otherMarkers;
    RigidBody *rigidBodies;
    // SkeletonData *skeletons;
    // LabelMarker *labelMarkers;
    // ForcePlateData *forcePlates;
    // DeviceData *devices;

    int numMarkerSets;
    int numOtherMarkers;
    int numRigidBodies;
    // int numSkeletons;
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

/// \breif Version class containing the version information and helpers for comparison.
class Version
{
  public:
    Version();
    Version(int major, int minor, int revision, int build);
    Version(const std::string& version);
    ~Version();

    void setVersion(int major, int minor, int revision, int build);
    const std::string& getVersionString();
    bool operator > (const Version& comparison);
    bool operator < (const Version& comparison);
    bool operator == (const Version& comparison);

    int v_major;
    int v_minor;
    int v_revision;
    int v_build;
    std::string v_string;
};


/// \brief Parser for a NatNet data frame packet
class MoCapDataFormat
{
  public:
    MoCapDataFormat(const char *packet, unsigned short length);
    ~MoCapDataFormat();

    /// \brief Parses a NatNet data frame packet as it is streamed by the Arena software according to the descriptions in the NatNet SDK v1.4. Should support up to 3.0 now
    void parse ();

    void setVersion(int nver[4], int sver[4])
    {
      NatNetVersion.setVersion(nver[0], nver[1], nver[2], nver[3]);
      ServerVersion.setVersion(sver[0], sver[1], sver[2], sver[3]);
    }

    const char *packet;
    unsigned short length;

    int frameNumber;
    ModelFrame model;

    Version NatNetVersion;
    Version ServerVersion;

  private:
    void seek(size_t count);
    template <typename T> void read_and_seek(T& target)
    {
        target = *((T*) packet);
        seek(sizeof(T));
    }
};

#endif  /*__MOCAP_DATAPACKETS_H__*/

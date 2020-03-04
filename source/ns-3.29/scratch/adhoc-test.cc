/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright(c) 2009 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// This script configures two nodes on an 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000
//(application) bytes to the other node.  The physical layer is configured
// to receive at a fixed RSS(regardless of the distance and transmit
// power); therefore, changing position of the nodes has no effect.
//
// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-adhoc --help"
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when rss drops below -97 dBm.
// To see this effect, try running:
//
// ./waf --run "wifi-simple-adhoc --rss=-97 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-98 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-99 --numPackets=20"
//
// Note that all ns-3 attributes(not just the ones exposed in the below
// script) can be changed at command line; see the documentation.
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
//
// ./waf --run "wifi-simple-adhoc --verbose=1"
//
// When you are done, you will notice two pcap trace files in your directory.
// If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-adhoc-0-0.pcap -nn -tt
//

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/disconnected.h"
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiSimpleAdhoc");

void ReceivePacket(Ptr<Socket> socket){
  Ptr<Node> node = socket->GetNode();
  std::cout << node->GetId() << std::endl;
  Ptr<Packet> packet = socket->Recv();
  uint8_t *buf = new uint8_t[packet->GetSize()];
  packet->CopyData(buf, packet->GetSize() - 2);
  std::string msg = std::string(reinterpret_cast<const char *>(buf), packet->GetSize() - 2);
  //std::cout << msg << " " << msg.length() << std::endl;
  free(buf);
}

Vector GetPosition (Ptr<Node> node){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void SetPosition (Ptr<Node> node, Vector position){
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

void movePositions(Ptr<Node> node){
  Vector pos = GetPosition(node);
  pos.x += 1;
  pos.y += 1;
  SetPosition (node, pos);
}

void moveAllPositions(NodeContainer robots){
  for(int i = 0; i < 10; i++){
    movePositions(robots.Get(i));
  }
}

static void BroadcastMessage(const char* data, int sourceNode,
            NodeContainer robots, TypeId tid, Ipv4InterfaceContainer interface){
  moveAllPositions(robots);
  std::cout << sourceNode << std::endl;
  Ptr<Socket> source = Socket::CreateSocket(robots.Get(sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress(interface.GetAddress(5), 80);
  source->Bind(remote);
  source->SetRecvCallback(MakeCallback(&ReceivePacket));
  //source->SetAllowBroadcast(true);
  source->Connect(remote);

  Ptr<Packet> packet = Create<Packet>(reinterpret_cast<const uint8_t*>(data), sizeof(data));
  source->Send(packet);
  source->Close();

  sourceNode++;
  if(sourceNode == 4){
    sourceNode = 0;
  }

  Simulator::Schedule(Seconds(1.0), &BroadcastMessage, data, sourceNode, robots, tid, interface);
}

double fRand(double fMin, double fMax){
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char *argv[]){
  srand(0);
  int numRobots = 10;
  int numTasks = 10;
  int minMap = 150;
  int maxMap = 250;
  std::string phyMode("DsssRate1Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 1;
  double interval = 1.0; // seconds
  bool verbose = false;

  CommandLine cmd;
  cmd.AddValue("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue("rss", "received signal strength", rss);
  cmd.AddValue("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue("numPackets", "number of packets generated", numPackets);
  cmd.AddValue("interval", "interval(seconds) between packets", interval);
  cmd.AddValue("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse(argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds(interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue(phyMode));

  NodeContainer robots;
  robots.Create(numRobots);

  NodeContainer tasks;
  tasks.Create(numTasks);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if(verbose){
    wifi.EnableLogComponents();  // Turn on all Wifi logging
  }
  wifi.SetStandard(WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set("RxGain", DoubleValue(0) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel","Rss",DoubleValue(rss));
  wifiPhy.SetChannel(wifiChannel.Create());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue(phyMode),
                                "ControlMode",StringValue(phyMode));
  // Set it to adhoc mode
  wifiMac.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, robots);

  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  // Robot Positions
  MobilityHelper mobilityRobots;
  Ptr<ListPositionAllocator> positionAllocRobots = CreateObject<ListPositionAllocator>();
  for(int i = 0; i < numRobots; i++){
    positionAllocRobots->Add(Vector(150 + 5*i, fRand(minMap, maxMap), 0.0));
  }
  mobilityRobots.SetPositionAllocator(positionAllocRobots);
  mobilityRobots.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityRobots.Install(robots);

  // Task Positions
  MobilityHelper mobilityTasks;
  Ptr<ListPositionAllocator> positionAllocTasks = CreateObject<ListPositionAllocator>();
  for(int i = 0; i < numTasks; i++){
    positionAllocTasks->Add(Vector(150 + 5*i, fRand(minMap, maxMap), 0.0));
    // Ptr<Node> node = tasks.Get(i);
    // doesn't currently change color correctly
    // AnimationInterface* ya = new AnimationInterface("dynamic_linknode.xml");
    // ya->UpdateNodeColor (node, 0, 255, 0);
  }
  mobilityTasks.SetPositionAllocator(positionAllocTasks);
  mobilityTasks.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityTasks.Install(tasks);

  InternetStackHelper internet;
  internet.Install(robots);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interface = ipv4.Assign(devices);
  
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

  // for(int i = 0; i < numRobots; i++){
  //   Ptr<Socket> recvSink = Socket::CreateSocket(robots.Get(i), tid);
  //   InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
  //   recvSink->Bind(local);
  //   recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));
  // }
  
  // Tracing
  wifiPhy.EnablePcap("wifi-simple-adhoc", devices);

  // Output what we are doing
  NS_LOG_UNCOND("Testing " << numPackets  << " packets sent with receiver rss " << rss );

  Simulator::Schedule(Seconds(1.0), &BroadcastMessage, "hello", 0, robots, tid, interface);

  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
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
#include "ns3/structs.h"
#include "ns3/agent.h"
#include "ns3/globalInfo.h"
#include "ns3/debuggingFunctions.h"
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiSimpleAdhoc");

int main(int argc, char *argv[]) {
  // Variables to edit system
  globalInfo::probabilityDropped = 100;
  globalInfo::numAgents = 10;
  globalInfo::numTasks = globalInfo::numAgents;
  double speed = 3.0;
  int numInstrumentClasses = 1;
  globalInfo::agentsPerClass  = globalInfo::numAgents / numInstrumentClasses;  // Make sure not fractional
  globalInfo::minPosition = 0.0;
  globalInfo::maxPosition = 200.0;
  globalInfo::maxPositionDistance = sqrt(2*((globalInfo::maxPosition - globalInfo::minPosition) * (globalInfo::maxPosition - globalInfo::minPosition)));

  // Variables to initialize
  globalInfo::totalNumRequestMessages = 0;
  globalInfo::totalNumPositionMessages = 0;
  globalInfo::totalPercentageReceived = 0;
  globalInfo::totalNumberConflicts = 0;
  globalInfo::totalPercentOptimal = 0;
  globalInfo::totalMessagesReceived = 0;
  globalInfo::numMoves = 0;
  globalInfo::testNumber = 0;
  std::string phyMode("DsssRate1Mbps");
  double rss = -80;  // -dBm
  bool verbose = false;
  
  std::vector<int> temp(globalInfo::numAgents, 0);
  globalInfo::instrumentAssignment = temp;

  CommandLine cmd;
  cmd.AddValue("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue("rss", "received signal strength", rss);
  cmd.AddValue("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse(argc, argv);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue(phyMode));

  NodeContainer agents;
  agents.Create(globalInfo::numAgents);

  NodeContainer tasks;
  tasks.Create(globalInfo::numTasks);
  
  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose) {
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
  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, agents);

  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  // Robot Positions
  MobilityHelper mobilityRobots;
  Ptr<ListPositionAllocator> positionAllocRobots = CreateObject<ListPositionAllocator>();
  for (int i = 0; i < globalInfo::numAgents; i++) {
    positionAllocRobots->Add(Vector(randomDouble(globalInfo::minPosition, globalInfo::maxPosition), randomDouble(globalInfo::minPosition, globalInfo::maxPosition), 0.0));
  }
  mobilityRobots.SetPositionAllocator(positionAllocRobots);
  mobilityRobots.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityRobots.Install(agents);
  // Task Positions
  MobilityHelper mobilityTasks;
  Ptr<ListPositionAllocator> positionAllocTasks = CreateObject<ListPositionAllocator>();
  for (int i = 0; i < globalInfo::numTasks; i++) {
    positionAllocTasks->Add(Vector(randomDouble(globalInfo::minPosition, globalInfo::maxPosition), randomDouble(globalInfo::minPosition, globalInfo::maxPosition), 0.0));
  }
  mobilityTasks.SetPositionAllocator(positionAllocTasks);
  mobilityTasks.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityTasks.Install(tasks);
  vector<Agent*> halfAgents = createAgents();
  
  for (int i = 0; i < globalInfo::numAgents; i++) {
    AgentNode newAgentNode;
    newAgentNode.node = agents.Get(i);
    newAgentNode.agent = halfAgents.at(i);
    // Set position
    Vector pos = getPosition(newAgentNode.node);
    newAgentNode.agent->agentPosition = pos;
    newAgentNode.agent->originalPosition = pos;
    newAgentNode.agent->initializeKnownPositions();
    // Set id
    newAgentNode.agent->agentId = newAgentNode.node->GetId();
    newAgentNode.agent->setSpeed(speed);
    newAgentNode.agent->setNumAgents(globalInfo::numAgents);
    newAgentNode.agent->setNumTasks(globalInfo::numTasks);

    globalInfo::allAgents.push_back(newAgentNode);
  }

  for (int i = 0; i < globalInfo::numTasks; i++) {
    TaskNode newTaskNode;
    newTaskNode.node = tasks.Get(i);
    newTaskNode.task = new Task();
    // Set position
    Vector pos = getPosition(newTaskNode.node);
    newTaskNode.task->taskLocation = pos;
    // Set id
    newTaskNode.task->taskId = newTaskNode.node->GetId();
    globalInfo::allTasks.push_back(newTaskNode);
  }

  InternetStackHelper internet;
  internet.Install(agents);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interface = ipv4.Assign(devices);
  
  // Tracing
  wifiPhy.EnablePcap("wifi-simple-adhoc", devices);

  // Create sockets
  for (int i = 0; i < globalInfo::numAgents; i++) {
    Ptr<Socket> recvSink = Socket::CreateSocket(globalInfo::allAgents.at(i).node, UdpSocketFactory::GetTypeId ());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny (), 80);
    if (recvSink->Bind(local) == -1) {
      recvSink->Close();
      std::cout << "Error binding" << std::endl;
    } 
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));
  }

  // Run Tests
  // for (int i = 0; i < 10; i++) {
  //   std::cout << "Round " << i << std::endl;
  //   srand(i);
  //   globalInfo::totalMessagesReceived = 0;
    
  //   Simulator::Schedule(Seconds(1), &calculateCostsAndPrepareRequests, globalInfo::allAgents, globalInfo::allTasks, interface); 
    

  //   giveAllAgentsRandomPositions(minPosition, maxPosition, globalInfo::allAgents);
  //   updateAllPositions(globalInfo::allAgents);
  //   resetAgents(globalInfo::allAgents);
  //   //Simulator::Destroy();
  // }

  Simulator::Schedule(Seconds(1), &runTests, globalInfo::allTasks, globalInfo::allAgents, interface);
  Simulator::Run();
  
  return 0;
}

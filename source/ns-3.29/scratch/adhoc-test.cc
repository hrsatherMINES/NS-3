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

bool start = false;

void haydensMethod(std::vector<TaskNode> allTasks, std::vector<AgentNode> allAgents, Ipv4InterfaceContainer interface){
  globalInfo::numMoves++;

  //Process costs
  calculate_all_costs(allTasks, allAgents);
  fillAllLocalCosts(allAgents);
  
  //Different heuristics for requests
  initialize_all_needed_info(allAgents);
  initialize_all_requests(allAgents);
  
  //send all requests and info
  initial_request_sharing(allAgents, interface);
  all_send_position_info(allAgents, interface);

  //Determine and excecute movements
  compute_all_parital_assignments_hungarian(allAgents, allTasks);
  determine_assigned_location(allAgents, allTasks);
  move_all_agents_towards_goal_step(allAgents);

  //Check if all tasks assigned
  checkIfDone(allAgents);

  //print all agents positions
  // for(unsigned long int i = 0; i < allAgents.size(); i++){
  //     allAgents[i].agent->print_position();
  //     std::cout << " ";
  //     allAgents[i].agent->print_assigned_position();
  //     // allAgents[i].agent->print_agent_costs();
  //     // print_known_positions(allAgents[i]);
  //     // allAgents[i].agent->print_known_info();
  //     print_known_positions(allAgents[i]);
  // }
  // std::cout << std::endl;

  Simulator::Schedule(Seconds(0.5), &haydensMethod, allTasks, allAgents, interface);
}



int main(int argc, char *argv[]){
  srand(1);
  globalInfo::numAgents = 10;
  globalInfo::numMoves = 0;
  int numTasks = globalInfo::numAgents;
  double speed = 5.0;
  std::string phyMode("DsssRate1Mbps");
  double rss = -80;  // -dBm
  double interval = 1.0; // seconds
  bool verbose = false;
  //int num_instrument_classes = 1;
  globalInfo::agents_per_class  = globalInfo::numAgents; //make sure not fractional
  int minPosition = 0.0;
  int maxPosition = 400.0;

  //set global info
  // globalInfo::allAgents;
  // globalInfo::allTasks;
  // globalInfo::instrument_assignment;
  // globalInfo::agents_per_clas;
  std::vector<int> temp(globalInfo::numAgents, 0);
  globalInfo::instrument_assignment = temp;
 
  CommandLine cmd;
  cmd.AddValue("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue("rss", "received signal strength", rss);
  cmd.AddValue("interval", "interval(seconds) between packets", interval);
  cmd.AddValue("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.Parse(argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds(interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue(phyMode));

  NodeContainer agents;
  agents.Create(globalInfo::numAgents);

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
  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, agents);

  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  // Robot Positions
  MobilityHelper mobilityRobots;
  Ptr<ListPositionAllocator> positionAllocRobots = CreateObject<ListPositionAllocator>();
  for(int i = 0; i < globalInfo::numAgents; i++){
    positionAllocRobots->Add(Vector(fRand(minPosition, maxPosition), fRand(minPosition, maxPosition), 0.0));
  }
  mobilityRobots.SetPositionAllocator(positionAllocRobots);
  mobilityRobots.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityRobots.Install(agents);
  // Task Positions
  MobilityHelper mobilityTasks;
  Ptr<ListPositionAllocator> positionAllocTasks = CreateObject<ListPositionAllocator>();
  for(int i = 0; i < numTasks; i++){
    positionAllocTasks->Add(Vector(fRand(minPosition, maxPosition), fRand(minPosition, maxPosition), 0.0));
    // Ptr<Node> node = tasks.Get(i);
    // doesn't currently change color correctly
    // AnimationInterface* ya = new AnimationInterface("dynamic_linknode.xml");
    // ya->UpdateNodeColor (node, 0, 255, 0);
  }
  mobilityTasks.SetPositionAllocator(positionAllocTasks);
  mobilityTasks.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityTasks.Install(tasks);
  vector<Agent*> halfAgents = create_agents();
  
  for(int i = 0; i < globalInfo::numAgents; i++){
    AgentNode newAgentNode;
    newAgentNode.node = agents.Get(i);
    newAgentNode.agent = halfAgents.at(i);
    //set position
    Vector pos = GetPosition(newAgentNode.node);
    newAgentNode.agent->agent_position = pos;
    newAgentNode.agent->initialize_known_positions();
    //set id
    newAgentNode.agent->agent_id = newAgentNode.node->GetId();
    newAgentNode.agent->set_speed(speed);
    newAgentNode.agent->set_num_agents(globalInfo::numAgents);
    newAgentNode.agent->set_num_tasks(numTasks);

    globalInfo::allAgents.push_back(newAgentNode);
  }

  for(int i = 0; i < numTasks; i++){
    TaskNode newTaskNode;
    newTaskNode.node = tasks.Get(i);
    newTaskNode.task = new Task();
    //set position
    Vector pos = GetPosition(newTaskNode.node);
    newTaskNode.task->task_location = pos;
    //set id
    newTaskNode.task->task_id = newTaskNode.node->GetId();
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
  for(int i = 0; i < globalInfo::numAgents; i++){
    Ptr<Socket> recvSink = Socket::CreateSocket(globalInfo::allAgents.at(i).node, UdpSocketFactory::GetTypeId ());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny (), 80);
    if(recvSink->Bind(local) == -1){
      recvSink->Close();
      std::cout << "Error binding" << std::endl;
    } 
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));
  }

  Simulator::Schedule(Seconds(0.5), &haydensMethod, globalInfo::allTasks, globalInfo::allAgents, interface);
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
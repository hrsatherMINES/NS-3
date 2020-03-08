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

bool start = false;

void haydensMethod(std::vector<TaskNode> allTasks, std::vector<AgentNode> allAgents, TypeId tid, Ipv4InterfaceContainer interface){
  
  calculate_all_costs(allTasks, allAgents);

  costMatrix = create_cmatrix(allAgents);
  fillAllLocalCosts(allAgents);

  comm_g = create_init_comm_graph(allAgents);
  
  determine_connected_components(allAgents, comm_g);
  
  fill_in_component_arrs(allAgents);
  initialize_all_needed_info(allAgents);
  
  initialize_all_requests(allAgents);
  
  initial_request_sharing(allAgents, tid, interface);
  
  bool conflicts = conflicts_exist(allAgents);
  
  
  if(!conflicts && all_agents_assigned(allAgents)){
      std::cout << "SUCCESSFUL: No conflicts, all agents are assigned" << std::endl;
      return;
  }
  
  if(conflicts){
      std::cout << "Conflicts Exist" << std::endl;
  }

  // if(agent_leaving_connection(all_a)){
  //     send_all_info = true;
  // }
  
  all_send_position_info(allAgents, tid, interface);
  
  compute_all_parital_assignments_hungarian(allAgents, allTasks);
  
  determine_assigned_location(allAgents, allTasks);


  move_all_agents_towards_goal_step(allAgents);
  

  //print all agents positions
  // for(unsigned long int i = 0; i < allAgents.size(); i++){
      // allAgents[i].agent->print_position();
      // std::cout << " ";
      // allAgents[i].agent->print_assigned_position();
      //allAgents[i].agent->print_agent_costs();
      // print_known_positions(allAgents[i]);
      //allAgents[i].agent->print_known_info();
      //print_known_positions(allAgents[i]);
  // }
  //print_comm_g(comm_g);
  //std::cout << std::endl;
  
  Simulator::Schedule(Seconds(0.5), &haydensMethod, allTasks, allAgents, tid, interface);
}


int main(int argc, char *argv[]){
  srand(1);
  numAgents = 3;
  numTasks = 3;
  speed = 5.0;
  std::string phyMode("DsssRate1Mbps");
  double rss = -80;  // -dBm
  double interval = 1.0; // seconds
  bool verbose = false;
  time_period = 2; // idk
  agent_velocity = speed;
  num_instrument_classes = 1;
  agents_per_class  = numAgents; //make sure not fractional
  minPosition = 0.0;
  maxPosition = 400.0;
  comm_thresh = 120;
  maxCharsSent = 10;
  socketNum = 0;

  std::vector<int> temp2(numAgents, 0);
  instrument_assignment = temp2;
 

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
  agents.Create(numAgents);

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
  for(int i = 0; i < numAgents; i++){
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
  
  for(int i = 0; i < numAgents; i++){
    AgentNode newAgentNode;
    newAgentNode.node = agents.Get(i);
    //std::cout << halfAgents.at(i).agent_id << std::endl;
    newAgentNode.agent = halfAgents.at(i);
    //set position
    Vector pos = GetPosition(newAgentNode.node);
    newAgentNode.agent->agent_position = pos;
    newAgentNode.agent->initialize_known_positions();
    //set id
    newAgentNode.agent->agent_id = newAgentNode.node->GetId();

    allAgents.push_back(newAgentNode);
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
    allTasks.push_back(newTaskNode);
  }

  InternetStackHelper internet;
  internet.Install(agents);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interface = ipv4.Assign(devices);
  
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

  // Tracing
  wifiPhy.EnablePcap("wifi-simple-adhoc", devices);

  //Simulator::Schedule(Seconds(1.0), &BroadcastMessage, "hello", 0, agents, tid, interface);
  Simulator::Schedule(Seconds(0.5), &haydensMethod, allTasks, allAgents, tid, interface);
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
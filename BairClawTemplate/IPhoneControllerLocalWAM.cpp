/*
 * two_wams.cpp
 *
 *  Created on: Feb 15, 2011
 *      Author: dc
 */

#include <iostream>

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <pthread.h>



#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/os.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>
//JSON library to manipulate UDP packets
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include "rapidjson/filestream.h"   // wrapper of C stream for prettywriter as output
#include <cstdio>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <dirent.h>
#include <ctime>
#include <time.h>



#define DEBUG 0
#define BUFLEN 1500
#define PORT 9330 //Port for iphone app
#define MOVE_TO_START 5

#pragma mark - declarations
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
using namespace barrett;
using namespace rapidjson;
using detail::waitForEnter;
char  c ='c';
bool  shouldKeepRunning = true;
bool  shouldInitHand = false;
bool  shouldGravityComp = false;
bool  shouldHoldPosition = false;
bool  shouldSendWAMHome = false;
bool  shouldIterate = false;
bool  shouldReloadFileNames = false;
typedef Hand::jp_type hjp_t;


struct wam_controller_state{
  int  moveToStart;  //0-false 1-true 2-ready 3-itemSpecific
  int  gravityComp;
  int  startTeach; //4-stopTeach
  int  replayTeach;
  int  trajIndex;
  std::string trajSaveFileName;
  int  moveToStartOfTraj;
  int  shouldMoveHome;
  int  wamState;
  int  bus;
} wam0_state, wam1_state;

boost::thread startWam(ProductManager& pm,
                       wam_controller_state& wamControllerState,
                       boost::function<void (ProductManager&, wam_controller_state&, systems::Wam<4>&)> wt4,
                       boost::function<void (ProductManager&, wam_controller_state&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0( ProductManager& pm0, wam_controller_state& wamControllerState, systems::Wam<DOF>& wam0);
//template <size_t DOF> void wamThread1( ProductManager& pm1, wam_controller_state& wamControllerState, systems::Wam<DOF>& wam1);
void handIPadController(ProductManager& pm0);

void err(char *str)
{
  perror(str);
  exit(1);
}

//Build trajNamesVec for JSON objects
std::vector<std::string> searchResults;				// holds search results
std::vector<std::string> trajNamesVec;
std::string trajNamesString;

// recursive search algorithm
void searchFileSystem(std::string curr_directory, std::string extension){
  searchResults.clear();
  searchResults.push_back("teach");
  DIR *dir;
  struct dirent *ent;
  std::string currentFile;
  if ((dir = opendir (curr_directory.c_str())) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      //printf ("%s\n", ent->d_name);
      currentFile = ent->d_name;
      if( currentFile.find(extension) !=  std::string::npos)
      {
        searchResults.push_back(currentFile);
        if(DEBUG)
          printf("------------------Match----------------");
      }
      else
      {
        if(DEBUG)
          printf("---------------------------------------");
      }
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    return;
  }
  return;
}

void makeTrajNamesVec(std::vector<std::string> trajNamesVec, std::string &trajNamesArray)
{
  trajNamesArray.clear();
  if(trajNamesVec.size())
  {
    trajNamesArray = "[";
    for(int i=0; i< trajNamesVec.size(); i++)
    {
      trajNamesArray += '"';
      trajNamesArray += trajNamesVec[i];
      trajNamesArray += '"';
      
      if(i != (trajNamesVec.size()-1))
        trajNamesArray += ",";
      else
        trajNamesArray += "]";
    }
  }
  else
  {
    trajNamesArray = "[]";
  }
  
  if(1)
  {
    printf("\n trajNamesVec: %s \n",trajNamesArray.c_str());
  }
  
}


int main(int argc, char** argv) {
  // Give us pretty stack-traces when things die
  installExceptionHandler();
  wam0_state.bus = 0; //Does nothing just keeps track of which canbus is assign for each WAM
  wam1_state.bus = 1; //Actual bus assigned from defalut.conf defined in ProductManager initilizer
  wam0_state.wamState = 0;
  wam1_state.wamState = 0;
  
  //Load traj files to be sent to the app for UIPickerView
  searchFileSystem(".",".traj" );
  makeTrajNamesVec(searchResults, trajNamesString);
  
  ProductManager pm0;
  //ProductManager pm1("/etc/barrett/bus1/default.conf");
  
  printf("Starting the WAM on Bus 0...\n");
  boost::thread wt0 = startWam(pm0, wam0_state, wamThread0<4>, wamThread0<7>);
  //printf("Starting the WAM on Bus 1...\n");
  //boost::thread wt1 = startWam(pm1, wam1_state, wamThread1<4>, wamThread1<7>);
  boost::thread handIPad(handIPadController, boost::ref(pm0));
  
  wt0.join();
  //wt1.join();
  
  handIPad.interrupt();
  
  
  return 0;
}

boost::thread startWam(ProductManager& pm,
                       wam_controller_state& wamControllerState,
                       boost::function<void (ProductManager&, wam_controller_state&, systems::Wam<4>&)> wt4,
                       boost::function<void (ProductManager&, wam_controller_state&, systems::Wam<7>&)> wt7)
{
  pm.waitForWam();
  pm.wakeAllPucks();
  
  if (pm.foundWam4()) {
    return boost::thread(wt4, boost::ref(pm), boost::ref(wamControllerState), boost::ref(*pm.getWam4()));
  } else {
    return boost::thread(wt7, boost::ref(pm), boost::ref(wamControllerState), boost::ref(*pm.getWam7()));
  }
}


void handIPadController(ProductManager& pm0)
{
  
  
  
  //HandVariables
  int hand0_shouldInit(0), hand1_shouldInit(0);
  
  typedef Hand::jp_type hjp_t;
  hjp_t open(0.0);
  hjp_t close(2.4);
  double SC = M_PI;
  close[3] = SC;
  hjp_t partial(1.3);
  //WAM hand setup
  if ( !pm0.foundHand() ) {
    printf("ERROR: No Hand found on bus!\n");
  }
  std::cout << "FOUND Hand on bus - " << pm0.getWamDefaultConfigPath() << "  -" << std::endl;
  /*
  if ( !pm1.foundHand() ) {
    printf("ERROR: No Hand found on bus!\n");
  }
  std::cout << "FOUND Hand on bus - " << pm1.getWamDefaultConfigPath() << "  -" << std::endl;
  */
  Hand& hand0 = *pm0.getHand();
  //Hand& hand1 = *pm1.getHand();
  
  
  
  //JSON object handling
  char buf[BUFLEN];
  char sendbuf[BUFLEN];
  struct sockaddr_in my_addr, cli_addr;
  int sockfd, sendLen;
  socklen_t slen=sizeof(cli_addr);
  //char buf[BUFLEN];
  
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
    err("socket");
  else
    printf("Server : Socket() successful\n");
  
  bzero(&my_addr, sizeof(my_addr));
  my_addr.sin_family = AF_INET;
  my_addr.sin_port = htons(PORT);
  my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  
  if (bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr))==-1)
    err("bind");
  else
    printf("Server : bind() successful\n");
  
  
  
  
  while (pm0.getSafetyModule()->getMode() == SafetyModule::ACTIVE) // && pm1.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
  {
    Document document;
    memset(buf, 0, BUFLEN);
    if (recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1)
      err("recvfrom()");
    
    
    if(document.Parse(buf).HasParseError())
    {
      printf("\nParseError\n");
      printf("Original JSON:\n %s\n", buf);
    }
    
    if(document.IsObject())
    { //JSON success now handle what to do
      //Parse into Variables
      wam0_state.moveToStart        = document["wam0_moveToStart"].GetInt();
      wam1_state.moveToStart        = document["wam1_moveToStart"].GetInt();
      wam0_state.gravityComp        = document["wam0_gravityComp"].GetInt();
      wam1_state.gravityComp        = document["wam1_gravityComp"].GetInt();
      wam0_state.startTeach         = document["wam0_startTeach"].GetInt();
      wam1_state.startTeach         = document["wam1_startTeach"].GetInt();
      wam0_state.replayTeach        = document["wam0_replayTeach"].GetInt();
      wam1_state.replayTeach        = document["wam1_replayTeach"].GetInt();
      wam0_state.trajIndex          = document["wam0_trajIndex"].GetInt();
      wam1_state.trajIndex          = document["wam1_trajIndex"].GetInt();
      wam0_state.trajSaveFileName   = document["wam0_trajSaveName"].GetString();
      wam1_state.trajSaveFileName   = document["wam1_trajSaveName"].GetString();
      wam0_state.moveToStartOfTraj  = document["wam0_moveToStartOfTraj"].GetInt();
      wam1_state.moveToStartOfTraj  = document["wam1_moveToStartOfTraj"].GetInt();
      wam0_state.shouldMoveHome     = document["wam0_shouldMoveHome"].GetInt();
      wam1_state.shouldMoveHome     = document["wam1_shouldMoveHome"].GetInt();
      
      //printf("\nwam1_state.replayTeach - %d\n", wam1_state.Teach);
      
      hand0_shouldInit = document["hand0_shouldInit"].GetInt();
      hand1_shouldInit = document["hand1_shouldInit"].GetInt();
      
      //            printf("\n0 - TrajIndex - %d, saveName - %s",wam0_state.trajIndex, wam0_state.trajSaveFileName.c_str()   );
      //            printf("\n1 - TrajIndex - %d, saveName - %s",wam1_state.trajIndex, wam1_state.trajSaveFileName.c_str()   );
      
      const Value& hand0_finger = document["hand0_finger"]; // Using a reference for consecutive access is handy and faster.
      
      const Value& hand1_finger = document["hand1_finger"]; // Using a reference for consecutive access is handy and faster.
      
      
      if(hand0_shouldInit)
      {
        hand0.initialize();
      }
      else
      {
        double fingerPos1   = hand0_finger[SizeType(0)].GetDouble();
        double fingerPos2   = hand0_finger[SizeType(1)].GetDouble();
        double fingerPos3   = hand0_finger[SizeType(2)].GetDouble();
        double fingerSpread = hand0_finger[SizeType(3)].GetDouble();
        hand0.velocityMove(hjp_t(fingerPos1,fingerPos2,fingerPos3,fingerSpread), Hand::WHOLE_HAND);
      }
      
      /*
      if(hand1_shouldInit)
      {
        hand1.initialize();
      }
      else
      {
        double fingerPos1   = hand1_finger[SizeType(0)].GetDouble();
        double fingerSpread1 = hand1_finger[SizeType(3)].GetDouble();
        hand1.velocityMove(hjp_t(fingerPos1,fingerPos1,fingerPos1,fingerSpread1), Hand::WHOLE_HAND);
      }
      */
    }
    else
    {
      printf("!IsObject()");
    }
    
    //printf("\nUDP recieve\n");
    //UDP send
    if(shouldReloadFileNames)
    {
      //Load traj files to be sent to the app for UIPickerView
      searchFileSystem(".",".traj" );
      makeTrajNamesVec(searchResults, trajNamesString);
      shouldReloadFileNames = false;
    }
    
    memset(sendbuf, 0, BUFLEN);
    
    sendLen = sprintf(sendbuf,"{\"wam0_moveToStart\":%d,\"wam1_moveToStart\":%d,\"wam0_gravityComp\":%d,\"wam1_gravityComp\":%d,\"wam0_startTeach\":%d,\"wam1_startTeach\":%d,\"wam0_replayTeach\":%d,\"wam1_replayTeach\":%d,\"wam0_moveToStartOfTraj\":%d,\"wam1_moveToStartOfTraj\":%d,\"wam0_shouldMoveHome\":%d,\"wam1_shouldMoveHome\":%d,\"wam0_state\":%d,\"wam1_state\":%d, \"trajFileNames\":%s }", wam0_state.moveToStart, wam1_state.moveToStart, wam0_state.gravityComp, wam1_state.gravityComp, wam0_state.startTeach, wam1_state.startTeach, wam0_state.replayTeach, wam1_state.replayTeach, wam0_state.moveToStartOfTraj, wam1_state.moveToStartOfTraj, wam0_state.shouldMoveHome, wam1_state.shouldMoveHome, wam0_state.wamState, wam1_state.wamState, trajNamesString.c_str());
    
    //printf("\n%s\n", sendbuf);
    //printf("\n%s\n", buf);
    //printf("UDPlocal - wam0_shouldInit = %d\n", document["wam0_moveToStart"].GetInt());
    //printf("UDPlocal - wam1_shouldInit = %d\n", document["wam1_moveToStart"].GetInt());
    //printf("UDP - wam0_shouldInit = %d  wam1_shouldInit = %d\n", document["wam0_moveToStart"].GetInt(), document["wam1_moveToStart"].GetInt());
    
    
    sendto(sockfd, sendbuf, sendLen, 0, (struct sockaddr*)&cli_addr, slen);
  }
  
  close(sockfd);
  
  
}




template <size_t DOF> void wamThread0(ProductManager& pm, wam_controller_state& wamControllerState, systems::Wam<DOF>& wam) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  printf("\nbus - %d\n", wamControllerState.bus);
  wamControllerState.wamState = 0;
  
  while( wamControllerState.moveToStart != MOVE_TO_START)
  {
    btsleep(0.2);
  }
  
  
  wam.gravityCompensate();
  jp_type jp(0.0), jphold(0.0);
  
  jphold = math::saturate(wam.getJointPositions(), 9.999);
  wam.moveTo(jphold);
  
  btsleep(2);
  int fromIdle = 0;
  wamControllerState.wamState = 1;
  
  typedef boost::tuple<double, jp_type> jp_sample_type;
  systems::Ramp time(pm.getExecutionManager());
  
  while(wamControllerState.shouldMoveHome != 1)
  {
    
    while(wamControllerState.gravityComp == 0)
    {
      btsleep(0.1);
      fromIdle = 1;
    }
    
    if(wamControllerState.gravityComp == 1)
    {
      wam.idle();
      fromIdle = 1;
    }
    else if(wamControllerState.gravityComp == 3)
    {
      
      if(fromIdle == 1){
        jphold = math::saturate(wam.getJointPositions(), 9.999);
        wam.moveTo(jphold);
        fromIdle = 0;
      }
      wam.moveTo(jphold);
      
    }
    btsleep(0.1);
    
    
    if(wamControllerState.startTeach)
    {
      wamControllerState.wamState = 10;
      printf("\nEntering Teach, bus = %d startTeach = %d\n",wamControllerState.bus, wamControllerState.startTeach);
      
      
      char tmpFile[] = "/tmp/btXXXXXX";
      if (mkstemp(tmpFile) == -1) {
        printf("ERROR: Couldn't create temporary file!\n");
      }
      
      const double T_s = pm.getExecutionManager()->getPeriod();
      
      
      
      
      systems::TupleGrouper<double, jp_type> jpLogTg;
      
      // Record at 1/10th of the loop rate
      systems::PeriodicDataLogger<jp_sample_type> jpLogger(pm.getExecutionManager(),
                                                           new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 10*T_s), 10);
      
      
      //printf("Press [Enter] to start teaching.\n");
      //waitForEnter();
      wam.idle();
      {
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
        
        connect(time.output, jpLogTg.template getInput<0>());
        connect(wam.jpOutput, jpLogTg.template getInput<1>());
        connect(jpLogTg.output, jpLogger.input);
        time.start();
      }
      
      
      while( wamControllerState.startTeach != 2 )
      {
        btsleep(0.01);
      }
      
      jpLogger.closeLog();
      disconnect(jpLogger.input);
      wamControllerState.wamState = 3;
      printf("Done Teaching, bus - %d\n",wamControllerState.bus);
      //Asks if you want to save
      std::string fileName = "recT-";
      time_t nowR;
      char timeNow[50];
      timeNow[0] ='\0';
      nowR = std::time(NULL);
      if(nowR != -1)
      {
        strftime(timeNow, 50, "%d-%H-%M", localtime(&nowR));
        fileName.append(std::string(timeNow));
        if(wamControllerState.bus == 0)
          fileName.append("_w0");
        else
          fileName.append("_w1");
        fileName.append(".traj");
      }
      
      std::ifstream  src(tmpFile, std::ios::binary);
      std::ofstream  dst(fileName.c_str(),   std::ios::binary);
      dst << src.rdbuf();
      shouldReloadFileNames = true;
      wamControllerState.startTeach = 0;
      wamControllerState.wamState = 1;
      btsleep(1);
    }
    
    
    
    
    if(wamControllerState.moveToStartOfTraj || (wamControllerState.replayTeach == 10))
    {
      wamControllerState.wamState = 20;
      printf("\nPreinit bus - %d\n", wamControllerState.bus);
      printf("\nbus - %d traj index - %d name - %s \n", wamControllerState.bus, wamControllerState.trajIndex, searchResults[wamControllerState.trajIndex].c_str());
      
      btsleep(1.0);
      
      // Build spline between recorded points
      log::Reader<jp_sample_type> lr(searchResults[wamControllerState.trajIndex].c_str());
      std::vector<jp_sample_type> vec;
      printf("\nPreinit bus - %d\n", wamControllerState.bus);
      for (size_t i = 0; i < lr.numRecords(); ++i) {
        vec.push_back(lr.getRecord());
      }
      math::Spline<jp_type> spline(vec);
      
      
      //printf("Press [Enter] to play back the recorded trajectory.\n");
      //waitForEnter();
      
      // First, move to the starting position
      
      
      
      
      wam.moveTo(spline.eval(spline.initialS()));
      wamControllerState.wamState = 21;
      
      while(wamControllerState.replayTeach != 10)
      {
        btsleep(0.01);
        printf("PlayTeach, bus - %d\n",wamControllerState.bus);
      }
      wamControllerState.wamState = 22;
      printf("\n Start bus - %d\n", wamControllerState.bus);
      
      // Then play back the recorded motion
      time.stop();
      time.setOutput(spline.initialS());
      
      systems::Callback<double, jp_type> trajectory(boost::ref(spline));
      connect(time.output, trajectory.input);
      wam.trackReferenceSignal(trajectory.output);
      
      time.start();
      
      while (trajectory.input.getValue() < spline.finalS()) {
        usleep(100000);
      }
      printf("Done Playing Trajectory, bus - %d\n",wamControllerState.bus);
      
      //            while(wamControllerState.moveToStartOfTraj != 1)
      //            {
      //                btsleep(0.01);
      //                printf("%d:Waiting for moveToStartOfTraj - %d or shouldMoveHome - %d, bus - %d\n", count, wamControllerState.moveToStartOfTraj, wamControllerState.shouldMoveHome, wamControllerState.bus);
      //                if(wamControllerState.shouldMoveHome == 1 )
      //                {
      //                    break;
      //                }
      //            }
      
      wam.moveTo(spline.eval(spline.initialS()));
      btsleep(1.5);
      
      wamControllerState.wamState = 1;
      
    }
    
  }
  
  
  wam.moveHome();
  
  std::cout << std::endl << "Shift-idle to end program" << std::endl;
  while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
    sleep(1);
  }
  
}
/*
template <size_t DOF> void wamThread1(ProductManager& pm1, wam_controller_state& wamControllerState, systems::Wam<DOF>& wam1) {
  wamThread0(pm1, wamControllerState, wam1);
}*/





















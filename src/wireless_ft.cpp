/* wireless_ft.cpp - ROS driver for the ATi Wireless F/T system
 *
 *
 * 2017.03.16 - created (from scratch / datasheet)
 * 2017.03.15 - ATi Java demo program won't run on Linux...
 *
 * (C) 2017, fnh, hendrich@informatik.uni-hamburg.de
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>

#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>


static pthread_mutex_t mutex;


class WirelessFT {
  public:
    WirelessFT();
    ~WirelessFT();

    int  telnetConnect( std::string hostname, int port );
    void run();


  private:
    ros::NodeHandle   nh;
    ros::Publisher    wrench_1_publisher;
    ros::Publisher    wrench_2_publisher;
    ros::Publisher    wrench_3_publisher;
    ros::Publisher    wrench_4_publisher;
    ros::Publisher    wrench_5_publisher;
    ros::Publisher    wrench_6_publisher;

    int n_channels;

    // networking stuff
    std::string hostname;
    int telnet_port;
    int clientSocket, clilen;

}; // end class WirelessFT



WirelessFT::WirelessFT() {
  ROS_INFO( "WirelessFT::<init>..." );
  n_channels = 6;

}


WirelessFT::~WirelessFT() {
  ; // empty
}


/**
 * tries to connect to the Wireless F/T controller using the 
 * given IP-address (e.g. 192.168.104.107) and port (typically 23).
 * Returns 0 on success, -1 on error.
 */
int WirelessFT::telnetConnect( std::string hostname, int port ) {
  clientSocket = socket( AF_INET, SOCK_STREAM, 0 );
  if (clientSocket < 0) {
    ROS_ERROR( "WirelessFT: ferror opening client socket" );
    return -1;
  }

  struct hostent *server = gethostbyname( hostname.c_str() );
  if (server == NULL) {
    ROS_ERROR( "WirelessFT: no such host: '%s'", hostname.c_str() );
    exit( -1 );
  }

  struct sockaddr_in serv_addr;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons( telnet_port );
  bcopy( (char*) server->h_addr,
         (char*) &serv_addr.sin_addr.s_addr,
         server->h_length );

  if (connect( clientSocket, (sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    ROS_ERROR( "WirelessFT: error connecting to socket at host '%s' port '%d'", hostname.c_str(), telnet_port );
    exit( -1 );
  }

  int nodelay = 1; // 1=on, 0=off
  int result = setsockopt( clientSocket,
                           IPPROTO_TCP,
                           TCP_NODELAY,
                           (char*) &nodelay, sizeof(int));
  if (result < 0) {
     ROS_ERROR( "WirelessFT: failed to set TCP_NODELAY on the socket..." );
  }

  return 0;
}


void WirelessFT::run() {
  ROS_INFO( "WirelessFT::run()..." );
}


int main( int argc, char ** argv ) {
  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  pthread_mutex_init(&mutex, &attr);

  ros::init( argc, argv, "WirelessFT", 1 ); // no default cntl-c handler
  ros::AsyncSpinner spinner( 2 ); 
  spinner.start();

  WirelessFT wirelessFT;
  wirelessFT.run();

}


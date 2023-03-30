

#ifndef ole_DRIVER_INPUT_H
#define ole_DRIVER_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>

#include <ros/ros.h>
#include <ole_msgs/olePacket.h>

namespace ole_driver
{

static uint16_t DATA_PORT_NUMBER = 2368;      // default data port
static uint16_t POSITION_PORT_NUMBER = 8308;  // default position port

/** @brief ole input base class */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);
  virtual ~Input() {}

  /** @brief Read one ole packet.
   *
   * @param pkt points to olePacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(ole_msgs::olePacket *pkt,
                        const double time_offset) = 0;

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
  bool gps_time_;
};

/** @brief Live ole input from socket. */
class InputSocket: public Input
{
public:
  InputSocket(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER);
  virtual ~InputSocket();

  virtual int getPacket(ole_msgs::olePacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  int sockfd_;
  in_addr devip_;
};


/** @brief ole input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, ole's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP: public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh,
            uint16_t port = DATA_PORT_NUMBER,
            double packet_rate = 0.0,
            std::string filename = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~InputPCAP();

  virtual int getPacket(ole_msgs::olePacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

private:
  ros::Rate packet_rate_;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

}  // namespace ole_driver

#endif  // ole_DRIVER_INPUT_H

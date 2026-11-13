// Disconnect safety: on_deactivate must send the CANCEL ('X') on the command link (port 1105) BEFORE the
// disconnect ('D') and before the sockets are closed, when motion may be queued on the controller.
//
// The hardware interface runs against three fake controller servers on 127.0.0.1 (ephemeral ports, no real
// controller, no ROS graph). The fake robot server records every byte received: each command message is
// <id: uint32><command: char>, 5 bytes for X (cancel) and D (disconnect).
//
// Note: ComauHardwareInterface is intentionally leaked at the end of each test. RobotClient's destructor sends a
// TERMINATE, sleeps 4 s and detaches a thread that on_deactivate already detached (std::terminate); that is
// existing behaviour, out of the scope of this test. main() leaves with _exit for the same reason.

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "comau_hardware_interface/comau_hw_interface.hpp"

namespace comau_hardware_interface
{
struct ComauHardwareInterfaceTestPeer
{
  static void mark_streaming(ComauHardwareInterface & hw) { hw.streaming_ = true; hw.have_last_sent_ = true; }
  static bool robot_link_up(ComauHardwareInterface & hw) { return hw.robot_client_->isConnected(); }
  static bool all_links_up(ComauHardwareInterface & hw)
  {
    return hw.state_client_->isConnected() && hw.robot_client_->isConnected() && hw.arm1_client_->isConnected();
  }
};
}  // namespace comau_hardware_interface

using comau_hardware_interface::ComauHardwareInterface;
using comau_hardware_interface::ComauHardwareInterfaceTestPeer;

namespace
{

// Minimal TCP server: accepts connections one after the other and records what it receives
class FakeServer
{
public:
  FakeServer()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    EXPECT_EQ(0, ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)));
    EXPECT_EQ(0, ::listen(listen_fd_, 4));
    socklen_t len = sizeof(addr);
    ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &len);
    port_ = ntohs(addr.sin_port);
    thread_ = std::thread([this] { run(); });
  }
  ~FakeServer()
  {
    kill();
    if (thread_.joinable()) {thread_.join();}
  }
  // The controller is gone: connection dropped and nobody listening any more (the client cannot reconnect)
  void kill()
  {
    if (stop_.exchange(true)) {return;}
    ::shutdown(listen_fd_, SHUT_RDWR);
    ::close(listen_fd_);
    drop_client();
  }
  std::string port() const { return std::to_string(port_); }
  std::string received()
  {
    std::lock_guard<std::mutex> l(m_);
    return data_;
  }
  // Commands received (5-byte messages <id><char>) as a string of command characters, e.g. "XD"
  std::string commands()
  {
    std::string d = received(), out;
    for (size_t i = 4; i < d.size(); i += 5) {out.push_back(d[i]);}
    return out;
  }
  // Abrupt close of the accepted connection (the controller side of a dead link)
  void drop_client()
  {
    std::lock_guard<std::mutex> l(m_);
    if (client_fd_ >= 0) {
      ::shutdown(client_fd_, SHUT_RDWR);
    }
  }

private:
  void run()
  {
    while (!stop_) {
      int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd < 0) {return;}
      {
        std::lock_guard<std::mutex> l(m_);
        client_fd_ = fd;
      }
      char buf[256];
      ssize_t n;
      while ((n = ::recv(fd, buf, sizeof(buf), 0)) > 0) {
        std::lock_guard<std::mutex> l(m_);
        data_.append(buf, static_cast<size_t>(n));
      }
      std::lock_guard<std::mutex> l(m_);
      ::close(fd);
      client_fd_ = -1;
    }
  }
  int listen_fd_{-1};
  int client_fd_{-1};
  uint16_t port_{0};
  std::atomic<bool> stop_{false};
  std::mutex m_;
  std::string data_;
  std::thread thread_;
};

hardware_interface::HardwareInfo make_info(const FakeServer & state, const FakeServer & robot, const FakeServer & arm1)
{
  hardware_interface::HardwareInfo info;
  info.name = "comau_test";
  info.hardware_parameters = {
    {"robot_ip", "127.0.0.1"}, {"state_port", state.port()}, {"robot_port", robot.port()}, {"arm1_port", arm1.port()}};
  for (int i = 1; i <= 6; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    hardware_interface::InterfaceInfo pos;
    pos.name = "position";
    joint.state_interfaces.push_back(pos);
    joint.command_interfaces.push_back(pos);
    info.joints.push_back(joint);
  }
  return info;
}

template <typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (pred()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return pred();
}

struct Rig
{
  FakeServer state, robot, arm1;
  ComauHardwareInterface * hw{new ComauHardwareInterface()};  // leaked on purpose (see the header comment)

  void activate()
  {
    rclcpp_lifecycle::State s;
    ASSERT_EQ(CallbackReturn::SUCCESS, hw->on_init(make_info(state, robot, arm1)));
    ASSERT_EQ(CallbackReturn::SUCCESS, hw->on_configure(s));
    ASSERT_EQ(CallbackReturn::SUCCESS, hw->on_activate(s));
    ASSERT_TRUE(wait_for([&] {return ComauHardwareInterfaceTestPeer::all_links_up(*hw);},
                         std::chrono::seconds(10)));
  }
  using CallbackReturn = comau_hardware_interface::CallbackReturn;
};

}  // namespace

// Streaming was active: CANCEL first, then DISCONNECT, on the command link
TEST(CancelOnDeactivate, CancelIsSentBeforeDisconnectWhenStreaming)
{
  Rig rig;
  rig.activate();
  ComauHardwareInterfaceTestPeer::mark_streaming(*rig.hw);

  rclcpp_lifecycle::State s;
  EXPECT_EQ(Rig::CallbackReturn::SUCCESS, rig.hw->on_deactivate(s));

  EXPECT_EQ("XD", rig.robot.commands()) << "bytes received on the command link: " << rig.robot.received().size();
  EXPECT_TRUE(rig.arm1.received().empty());  // the cancel travels on the command link only
}

// Nothing streamed since activation: no spurious CANCEL (it would also stop motions of other programs)
TEST(CancelOnDeactivate, NoCancelWhenNothingWasStreamed)
{
  Rig rig;
  rig.activate();
  rclcpp_lifecycle::State s;
  EXPECT_EQ(Rig::CallbackReturn::SUCCESS, rig.hw->on_deactivate(s));
  EXPECT_EQ("D", rig.robot.commands());
}

// The command link is already dead: the cancel cannot be delivered, the deactivation must still complete
TEST(CancelOnDeactivate, DeactivationCompletesWhenTheCommandLinkIsDown)
{
  Rig rig;
  rig.activate();
  ComauHardwareInterfaceTestPeer::mark_streaming(*rig.hw);
  rig.robot.kill();
  ASSERT_TRUE(wait_for([&] {return !ComauHardwareInterfaceTestPeer::robot_link_up(*rig.hw);},
                       std::chrono::seconds(5)));

  rclcpp_lifecycle::State s;
  EXPECT_EQ(Rig::CallbackReturn::SUCCESS, rig.hw->on_deactivate(s));
  EXPECT_EQ("", rig.robot.commands());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  std::fflush(nullptr);
  _exit(result);
}

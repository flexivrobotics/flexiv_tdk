/**
 * @example transparent_cartesian_teleop_wifi.cpp
 * @brief Example usage of single- or dual-arm transparent Cartesian teleoperation between two
 * computers on the same WiFi network. Each computer is wired to its own robots (the leader computer
 * to the leader robots, the follower computer to the follower robots) and runs one
 * TransparentCartesianTeleopWAN instance with one or two robot pairs. Only the teleop messages
 * between the two computers travel over WiFi, through one TCP connection (Standard Edition), so no
 * public IP address or port forwarding is needed. A pedal on DI1 of the first leader robot engages
 * all pairs. Optionally, DI0 of each leader robot opens and closes the gripper on its follower
 * robot (hold to close); these commands travel over a second TCP connection on the next port. Uses
 * the TDK default Cartesian stiffness and sets no max contact wrench limit. Optionally homes all
 * local robots after connecting. Also supports per-pair restart, message latency monitoring,
 * nullspace posture tuning and fault recovery.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/tdk/data.hpp>
#include <flexiv/tdk/transparent_cartesian_teleop_wan.hpp>
#include <flexiv/rdk/gripper.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <getopt.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace {

/** Maximum number of leader-follower robot pairs: 1 for single-arm, 2 for dual-arm teleop */
constexpr unsigned int kMaxPairs = 2;

/** Nullspace to a preferred posture, one per robot pair. Adjust to how each arm is mounted. */
const std::array<std::vector<double>, kMaxPairs> kPreferredJntPos
    = {std::vector<double> {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
           115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0},
        std::vector<double> {60 * M_PI / 180.0, -60 * M_PI / 180.0, -85 * M_PI / 180.0,
            115 * M_PI / 180.0, 70 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0}};

/** Nullspace to Home posture */
const std::vector<double> kHomeJntPos = {0 * M_PI / 180.0, -40 * M_PI / 180.0, 0 * M_PI / 180.0,
    90 * M_PI / 180.0, 0 * M_PI / 180.0, 40 * M_PI / 180.0, 0 * M_PI / 180.0};

/** Upper bound of the latency limit accepted by SetTeleopLatencyLimit() */
constexpr double kMaxLatencyLimitMs = 350.0;

/** Digital input port of the 1st leader robot wired to the engage pedal */
constexpr size_t kPedalDI = 1;

/** Digital input port of each leader robot that closes its follower's gripper while high */
constexpr size_t kGripperDI = 0;

/** Consecutive equal samples (10 ms apart) required to accept a gripper input change */
constexpr int kGripperDebounceSamples = 3;

/** Period of re-sending the gripper commands, so the follower catches up after a reconnect */
constexpr auto kGripperResendPeriod = std::chrono::milliseconds(200);

/** Gripper closing force in N, clamped to the valid range of the enabled gripper */
constexpr double kGripperCloseForce = 40.0;

/** Side link heartbeat period */
constexpr auto kSideLinkHeartbeat = std::chrono::milliseconds(500);

/** Silence on the side link after which the connection is dropped and re-established */
constexpr auto kSideLinkTimeout = std::chrono::milliseconds(5000);

/** Atomic signal to stop console, fault monitor, link monitor and DI reading tasks */
std::atomic<bool> g_running {true};

/** Whether the link monitor task prints message latency every second */
std::atomic<bool> g_print_latency {false};
}

std::string BoolStr(bool v)
{
    return v ? "true" : "false";
}

std::string FmtFixed(double v, int prec)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << v;
    return oss.str();
}

void LogInfo(const std::string& msg)
{
    std::cout << "[info] " << msg << std::endl;
}

void LogWarn(const std::string& msg)
{
    std::cerr << "[warn] " << msg << std::endl;
}

void LogError(const std::string& msg)
{
    std::cerr << "[error] " << msg << std::endl;
}

bool HasActiveIssue(const flexiv::tdk::TeleopStatus& status, flexiv::tdk::TeleopIssueCode code)
{
    return std::any_of(status.issues.begin(), status.issues.end(),
        [code](const flexiv::tdk::TeleopIssue& issue) { return issue.code == code; });
}

/**
 * @brief Whether [ip] is an IPv4 address in a private range (10/8, 172.16/12, 192.168/16), i.e. an
 * address assigned by a local WiFi router.
 */
bool IsPrivateIPv4(const std::string& ip)
{
    unsigned int a = 0, b = 0, c = 0, d = 0;
    char tail = 0;
    if (std::sscanf(ip.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail) != 4) {
        return false;
    }
    if (a > 255 || b > 255 || c > 255 || d > 255) {
        return false;
    }
    return a == 10 || (a == 172 && b >= 16 && b <= 31) || (a == 192 && b == 168);
}

/**
 * @brief Robot pairs a console command applies to: all pairs by default, or the single pair given
 * by an index suffix (e.g. "u1"). Returns empty for an invalid suffix.
 */
std::vector<unsigned int> TargetPairs(const std::string& input, unsigned int num_pairs)
{
    if (input.size() == 1) {
        std::vector<unsigned int> all(num_pairs);
        std::iota(all.begin(), all.end(), 0u);
        return all;
    }
    if (input.size() == 2 && input[1] >= '0' && input[1] < static_cast<char>('0' + num_pairs)) {
        return {static_cast<unsigned int>(input[1] - '0')};
    }
    return {};
}

//============================================ SIDE LINK ===========================================
/** Sleep for [ms] in short steps, returning early once [stop] is set */
void SleepUnless(const std::atomic<bool>& stop, int ms)
{
    for (int t = 0; t < ms && !stop.load(); t += 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/** Open a TCP socket listening on [port] on all interfaces. Returns -1 on failure. */
int ListenOn(unsigned int port)
{
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }
    const int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || ::listen(fd, 1) < 0) {
        ::close(fd);
        return -1;
    }
    return fd;
}

/** Connect a TCP socket to [ip]:[port] within [timeout_ms]. Returns -1 on failure. */
int ConnectTo(const std::string& ip, unsigned int port, int timeout_ms)
{
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
        return -1;
    }
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        int err = 0;
        socklen_t len = sizeof(err);
        pollfd pfd {fd, POLLOUT, 0};
        if (errno != EINPROGRESS || ::poll(&pfd, 1, timeout_ms) != 1
            || ::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) < 0 || err != 0) {
            ::close(fd);
            return -1;
        }
    }
    ::fcntl(fd, F_SETFL, flags);
    return fd;
}

/**
 * @brief Minimal line-based TCP link between the two computers, used to send gripper commands from
 * the leader to the follower. The computer that is the teleop TCP server listens on [port]; the
 * other connects to [server_ip]:[port]. Both sides send a heartbeat and re-establish the connection
 * when it drops or goes silent. Received lines are passed to [on_line] on the link thread.
 */
class SideLink
{
public:
    SideLink(bool is_server, const std::string& server_ip, unsigned int port,
        std::function<void(const std::string&)> on_line)
    : is_server_(is_server)
    , server_ip_(server_ip)
    , port_(port)
    , on_line_(std::move(on_line))
    {
        thread_ = std::thread(&SideLink::Run, this);
    }

    ~SideLink()
    {
        stop_ = true;
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    SideLink(const SideLink&) = delete;
    SideLink& operator=(const SideLink&) = delete;

    /**
     * @brief Send one line without blocking. Returns false if the link is not connected or the
     * line could not be queued, e.g. while the network is stalled.
     */
    bool Send(const std::string& line)
    {
        const std::string msg = line + "\n";
        std::lock_guard<std::mutex> lock(fd_mutex_);
        if (fd_ < 0) {
            return false;
        }
        return ::send(fd_, msg.data(), msg.size(), MSG_DONTWAIT | MSG_NOSIGNAL)
               == static_cast<ssize_t>(msg.size());
    }

private:
    using Clock = std::chrono::steady_clock;

    /** Use [fd] as the connection, replacing any previous one */
    void Attach(int fd)
    {
        const int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        std::lock_guard<std::mutex> lock(fd_mutex_);
        if (fd_ >= 0) {
            ::close(fd_);
        }
        fd_ = fd;
        buffer_.clear();
        last_rx_ = last_tx_ = Clock::now();
    }

    /** Close the current connection, if any */
    void Detach(const std::string& reason)
    {
        std::lock_guard<std::mutex> lock(fd_mutex_);
        if (fd_ < 0) {
            return;
        }
        ::close(fd_);
        fd_ = -1;
        if (!reason.empty()) {
            LogWarn("Side link " + reason);
        }
    }

    void Run()
    {
        int listen_fd = -1;
        bool listen_warned = false;
        while (!stop_.load()) {
            // Server side: open the listening socket
            if (is_server_ && listen_fd < 0) {
                listen_fd = ListenOn(port_);
                if (listen_fd < 0) {
                    if (!listen_warned) {
                        LogWarn("Side link cannot listen on TCP port " + std::to_string(port_)
                                + ", retrying");
                        listen_warned = true;
                    }
                    SleepUnless(stop_, 1000);
                    continue;
                }
            }

            // Client side: connect to the server
            if (!is_server_ && fd_ < 0) {
                const int fd = ConnectTo(server_ip_, port_, 1000);
                if (fd < 0) {
                    SleepUnless(stop_, 1000);
                    continue;
                }
                Attach(fd);
                LogInfo("Side link connected to " + server_ip_ + ":" + std::to_string(port_));
            }

            // Heartbeat, and drop a connection that has gone silent
            if (fd_ >= 0) {
                const auto now = Clock::now();
                if (now - last_rx_ > kSideLinkTimeout) {
                    Detach("timed out, reconnecting");
                    continue;
                }
                if (now - last_tx_ >= kSideLinkHeartbeat) {
                    Send("PING");
                    last_tx_ = now;
                }
            }

            const int conn = fd_;
            pollfd fds[2];
            nfds_t n = 0;
            if (conn >= 0) {
                fds[n++] = pollfd {conn, POLLIN, 0};
            }
            if (listen_fd >= 0) {
                fds[n++] = pollfd {listen_fd, POLLIN, 0};
            }
            if (::poll(fds, n, 100) <= 0) {
                continue;
            }
            for (nfds_t i = 0; i < n; ++i) {
                if (fds[i].revents == 0) {
                    continue;
                }
                if (fds[i].fd == listen_fd) {
                    // A new connection replaces the old one, which may be dead without notice
                    const int fd = ::accept(listen_fd, nullptr, nullptr);
                    if (fd >= 0) {
                        Attach(fd);
                        LogInfo("Side link connected");
                    }
                } else if (fds[i].fd == conn && conn == fd_) {
                    char chunk[256];
                    const ssize_t len = ::recv(conn, chunk, sizeof(chunk), 0);
                    if (len < 0 && errno == EINTR) {
                        continue;
                    }
                    if (len <= 0) {
                        Detach("disconnected");
                        continue;
                    }
                    buffer_.append(chunk, static_cast<size_t>(len));
                    size_t pos = 0;
                    while ((pos = buffer_.find('\n')) != std::string::npos) {
                        const std::string line = buffer_.substr(0, pos);
                        buffer_.erase(0, pos + 1);
                        if (line != "PING" && on_line_) {
                            on_line_(line);
                        }
                    }
                    if (buffer_.size() > 4096) {
                        buffer_.clear();
                    }
                    last_rx_ = Clock::now();
                }
            }
        }
        Detach("");
        if (listen_fd >= 0) {
            ::close(listen_fd);
        }
    }

    const bool is_server_;
    const std::string server_ip_;
    const unsigned int port_;
    const std::function<void(const std::string&)> on_line_;
    std::atomic<bool> stop_ {false};
    std::mutex fd_mutex_;
    int fd_ = -1;
    std::string buffer_;
    Clock::time_point last_rx_;
    Clock::time_point last_tx_;
    std::thread thread_;
};

/** Gripper command sent from leader to follower: "GRIP <pair index> <1: close, 0: open>" */
std::string GripperCommand(unsigned int idx, bool close)
{
    return "GRIP " + std::to_string(idx) + (close ? " 1" : " 0");
}

/** Parse a gripper command. Returns false if [line] is not one. */
bool ParseGripperCommand(const std::string& line, unsigned int& idx, bool& close)
{
    std::istringstream iss(line);
    std::string tag;
    unsigned int pair = 0;
    int state = -1;
    if (!(iss >> tag >> pair >> state) || tag != "GRIP" || (state != 0 && state != 1)) {
        return false;
    }
    idx = pair;
    close = (state == 1);
    return true;
}

//======================================== FOLLOWER GRIPPERS =======================================
/**
 * @brief Grippers on the local follower robots, opened and closed by the commands received from the
 * leader. Uses the rdk::Robot connections owned by the TransparentCartesianTeleopWAN instance,
 * which must outlive this object.
 */
class FollowerGrippers
{
public:
    /**
     * @brief Enable the named gripper on the follower robot of every pair and trigger its
     * initialization. The robots must be operational, i.e. construct this after
     * TransparentCartesianTeleopWAN::Init().
     */
    FollowerGrippers(const flexiv::tdk::TransparentCartesianTeleopWAN& teleop,
        unsigned int num_pairs, const std::string& name)
    {
        for (unsigned int idx = 0; idx < num_pairs; ++idx) {
            auto gripper = std::make_unique<flexiv::rdk::Gripper>(*teleop.instance(idx));
            gripper->Enable(name);
            gripper->Init();
            params_.push_back(gripper->params());
            grippers_.push_back(std::move(gripper));
            closed_.push_back(-1);
            LogInfo("Pair " + std::to_string(idx) + " gripper [" + name
                    + "] enabled, initialization triggered");
        }
    }

    /** Open or close the gripper of the specified pair, unless it was already commanded so */
    void Apply(unsigned int idx, bool close)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (idx >= grippers_.size() || closed_[idx] == static_cast<int>(close)) {
            return;
        }
        const auto& p = params_[idx];
        if (close) {
            const double force = std::clamp(kGripperCloseForce, p.min_force, p.max_force);
            grippers_[idx]->Grasp(force);
            LogInfo("Pair " + std::to_string(idx) + " gripper closing, force " + FmtFixed(force, 1)
                    + " N");
        } else {
            grippers_[idx]->Move(p.max_width, std::clamp(0.5 * p.max_vel, p.min_vel, p.max_vel),
                std::clamp(0.5 * p.max_force, p.min_force, p.max_force));
            LogInfo("Pair " + std::to_string(idx) + " gripper opening to "
                    + FmtFixed(p.max_width, 3) + " m");
        }
        closed_[idx] = close ? 1 : 0;
    }

    /** Trigger initialization of the gripper of the specified pair */
    void Init(unsigned int idx)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        grippers_.at(idx)->Init();
        closed_[idx] = -1;
    }

    void PrintStates(unsigned int idx) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto s = grippers_.at(idx)->states();
        LogInfo("Pair " + std::to_string(idx) + " gripper: width " + FmtFixed(s.width, 4)
                + " m, force " + FmtFixed(s.force, 2) + " N, moving " + BoolStr(s.is_moving));
    }

private:
    mutable std::mutex mutex_;
    std::vector<std::unique_ptr<flexiv::rdk::Gripper>> grippers_;
    std::vector<flexiv::rdk::GripperParams> params_;
    /** Last commanded state per pair: -1 none, 0 open, 1 closed */
    std::vector<int> closed_;
};

//============================================== PRINT =============================================
void PrintTeleopStatus(unsigned int idx, const flexiv::tdk::TeleopStatus& status)
{
    LogInfo(
        "Teleop pair " + std::to_string(idx) + " status: initialized=" + BoolStr(status.initialized)
        + " started=" + BoolStr(status.started) + " engaged=" + BoolStr(status.engaged)
        + " stopped=" + BoolStr(status.stopped) + " fault=" + BoolStr(status.fault)
        + " motion_restricted=" + BoolStr(status.motion_restricted) + " latency="
        + FmtFixed(status.latency_ms, 1) + "/" + FmtFixed(status.latency_threshold_ms, 1) + " ms");
    if (status.primary.code == flexiv::tdk::TeleopIssueCode::NONE) {
        LogInfo("No teleop restriction. Safe to continue.");
        return;
    }
    const auto& issue = status.primary;
    LogWarn(std::string("[") + flexiv::tdk::TeleopIssueLevelStr[static_cast<size_t>(issue.level)]
            + "/" + flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(issue.side)] + "] "
            + issue.title + " | " + issue.description + " | " + issue.suggestion);
    for (const auto& extra : status.issues) {
        if (extra.code == issue.code && extra.side == issue.side
            && extra.joint_index == issue.joint_index) {
            continue;
        }
        LogWarn(std::string("  also: [")
                + flexiv::tdk::TeleopIssueCodeStr[static_cast<size_t>(extra.code)] + "/"
                + flexiv::tdk::TeleopIssueSideStr[static_cast<size_t>(extra.side)] + "] "
                + extra.title);
    }
}

/**
 * @brief Print the message latency of the specified robot pair and why it is out of range, if so.
 */
void PrintLatency(const flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int idx)
{
    const std::string tag = "pair " + std::to_string(idx);
    double latency_ms {};
    const bool ok = teleop.CheckTeleopConnectionLatency(idx, latency_ms);
    if (ok) {
        LogInfo(tag + " message latency: " + FmtFixed(latency_ms, 1) + " ms (within limit)");
    } else if (HasActiveIssue(teleop.GetTeleopStatus(idx),
                   flexiv::tdk::TeleopIssueCode::NETWORK_DISCONNECTED)) {
        LogWarn(tag + " disconnected from the other computer");
    } else if (latency_ms < 0.0) {
        LogWarn(tag + " clock mismatch: latency " + FmtFixed(latency_ms, 1)
                + " ms. Check 'chronyc tracking' on both computers");
    } else {
        LogWarn(tag + " latency over limit: " + FmtFixed(latency_ms, 1) + " ms");
    }
}

void PrintHelp()
{
    // clang-format off
    std::cout<<"Invalid program arguments!"<<std::endl;
    std::cout<<"     -l     [necessary] Serial number of the 1st leader robot."<<std::endl;
    std::cout<<"     -f     [necessary] Serial number of the 1st follower robot."<<std::endl;
    std::cout<<"     -L     [optional] Serial number of the 2nd leader robot, for dual-arm teleop."<<std::endl;
    std::cout<<"     -F     [optional] Serial number of the 2nd follower robot, for dual-arm teleop."<<std::endl;
    std::cout<<"     -r     [necessary] Role of participants in teleop, can be [leader] or [follower]."<<std::endl;
    std::cout<<"     -t     [necessary] Role in the TCP connection, can be [server] or [client]."<<std::endl;
    std::cout<<"     -i     [necessary] WiFi IPv4 address of the machine that functions as TCP server."<<std::endl;
    std::cout<<"     -p     [necessary] Listening port of the TCP server machine."<<std::endl;
    std::cout<<"     -A     [optional] LAN IPv4 address of the NIC wired to the robots. Repeatable."<<std::endl;
    std::cout<<"     -W     [optional] WiFi interface name, e.g. wlo1. Repeatable."<<std::endl;
    std::cout<<"     -T     [optional] Maximum acceptable message latency in ms, range [0, 350]. Default 200."<<std::endl;
    std::cout<<"     -D     [optional] Leader only. The pedal on DI1 of the 1st leader robot engages all pairs."<<std::endl;
    std::cout<<"     -n     [optional] Follower gripper device name in Elements -> Device, e.g. Flexiv-GN01."<<std::endl;
    std::cout<<"                       DI0 of each leader robot closes (high) or opens (low) its follower's gripper."<<std::endl;
    std::cout<<"                       Pass on both computers. Gripper commands use TCP port -p + 1."<<std::endl;
    std::cout<<"     -H     [optional] Move all local robots to home posture after connecting, before teleop starts."<<std::endl;
    std::cout<<"Pass -L and -F together for dual-arm teleop, or neither for single-arm teleop."<<std::endl;
    std::cout<<"Both computers must pass the same serial numbers in the same order, and the same -i and -p."<<std::endl;
    std::cout<<"Pass -H on both computers to home every arm; each computer homes only its own robots."<<std::endl;
    std::cout<<"Usage: sudo ./transparent_cartesian_teleop_wifi -l <sn> -f <sn> [-L <sn> -F <sn>] -r leader|follower -t server|client -i <server_wifi_ip> -p <port> [-A <lan_ipv4>] [-W <wifi_iface>] [-T <ms>] [-D] [-n <gripper_name>] [-H]"<<std::endl;
    // clang-format on
}

/**
 * @brief Print the network setup that teleop over a local WiFi network depends on.
 */
void PrintWiFiChecklist(bool is_tcp_server, unsigned int port, bool side_link)
{
    std::cout << R"(
  --- WiFi teleop checklist ---
    1. Both computers are on the same WiFi network with no AP/client isolation, and the WiFi
       subnet does not overlap the robot subnet.
    2. WiFi power saving is off on both computers: sudo iw dev <wifi_iface> set power_save off
    3. Clocks are synchronized: 'chronyc tracking' shows System time < 1 ms. Over local WiFi the
       real latency is only a few ms, so a small clock offset already reads as negative latency
       and forces teleop to disengage.
)";
    if (is_tcp_server) {
        const std::string ports = side_link ? std::to_string(port) + ":" + std::to_string(port + 1)
                                            : std::to_string(port);
        std::cout << "    4. TCP port" << (side_link ? "s " : " ") << ports
                  << " open in this computer's firewall: sudo ufw allow " << ports << "/tcp"
                  << std::endl;
    }
    std::cout << std::endl;
}

const struct option kLongOptions[] = {
    // clang-format off
    {"leader SN",                   required_argument,  0, 'l'},
    {"follower SN",                 required_argument,  0, 'f'},
    {"2nd leader SN",               required_argument,  0, 'L'},
    {"2nd follower SN",             required_argument,  0, 'F'},
    {"role",                        required_argument,  0, 'r'},
    {"tcp role",                    required_argument,  0, 't'},
    {"server wifi ipv4 address",    required_argument,  0, 'i'},
    {"port",                        required_argument,  0, 'p'},
    {"lan whitelist ip",            required_argument,  0, 'A'},
    {"wifi interface name",         required_argument,  0, 'W'},
    {"latency limit ms",            required_argument,  0, 'T'},
    {"enable digital input",        no_argument,        0, 'D'},
    {"gripper name",                required_argument,  0, 'n'},
    {"home on start",               no_argument,        0, 'H'},
    {0,                             0,                  0,  0 }
    // clang-format on
};

//============================================== TASKS =============================================
/**
 * @brief Task for reading the leader robots' digital inputs. If [use_pedal], the pedal on DI1 of
 * the 1st leader robot engages all pairs. If [gripper_link] is given, DI0 of each leader robot
 * closes (high) or opens (low) the gripper of its follower robot.
 */
void ReadDigitalInputTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop,
    unsigned int num_pairs, bool use_pedal, SideLink* gripper_link)
{
    std::array<bool, kMaxPairs> logged_idle {};
    std::array<bool, kMaxPairs> grip_closed {};
    std::array<int, kMaxPairs> grip_count {};
    bool grip_initialized = false;
    auto last_grip_send = std::chrono::steady_clock::time_point {};
    std::string last_error;
    while (g_running.load()) {
        try {
            std::array<std::array<bool, flexiv::tdk::kIOPorts>, kMaxPairs> di {};
            for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                di[idx] = teleop.digital_inputs(idx);
            }

            if (use_pedal) {
                const bool pedal = di[0][kPedalDI];
                for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                    const auto status = teleop.GetTeleopStatus(idx);
                    // Robot fault / Stop() drops the pair back to not-started. Engage() is invalid
                    // in that state and must not be polled every cycle.
                    if (!status.started || status.stopped) {
                        if (!logged_idle[idx]) {
                            LogWarn("ReadDigitalInputTask: pair " + std::to_string(idx)
                                    + " is not started, pause Engage (press 'u"
                                    + std::to_string(idx) + "' to resume)");
                            logged_idle[idx] = true;
                        }
                        continue;
                    }
                    logged_idle[idx] = false;
                    teleop.Engage(idx, pedal);
                }
            }

            if (gripper_link) {
                // Start from the actual DI0 levels, so a gripper held closed at startup is never
                // sent an open command first
                if (!grip_initialized) {
                    for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                        grip_closed[idx] = di[idx][kGripperDI];
                        LogInfo("Pair " + std::to_string(idx) + " gripper initial state: "
                                + (grip_closed[idx] ? "close (DI0 high)" : "open (DI0 low)"));
                    }
                    grip_initialized = true;
                }
                // Debounce DI0 of each leader robot: high closes, low opens its follower's gripper
                bool changed = false;
                for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                    const bool raw = di[idx][kGripperDI];
                    if (raw == grip_closed[idx]) {
                        grip_count[idx] = 0;
                        continue;
                    }
                    if (++grip_count[idx] < kGripperDebounceSamples) {
                        continue;
                    }
                    grip_closed[idx] = raw;
                    grip_count[idx] = 0;
                    changed = true;
                    LogInfo("Pair " + std::to_string(idx)
                            + " gripper: " + (raw ? "close (DI0 high)" : "open (DI0 low)"));
                }
                // Send on change, and periodically so the follower catches up after a reconnect
                const auto now = std::chrono::steady_clock::now();
                if (changed || now - last_grip_send >= kGripperResendPeriod) {
                    for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                        gripper_link->Send(GripperCommand(idx, grip_closed[idx]));
                    }
                    last_grip_send = now;
                }
            }
            last_error.clear();
        } catch (const std::exception& e) {
            if (last_error != e.what()) {
                LogError(std::string("Exception in ReadDigitalInputTask: ") + e.what());
                last_error = e.what();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    LogInfo("ReadDigitalInputTask exiting.");
}

/**
 * @brief Task for monitoring robot fault of all pairs and prompting the operator to recover.
 */
void FaultMonitorTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs)
{
    std::array<bool, kMaxPairs> last_fault {};
    std::string last_error;
    while (g_running.load()) {
        try {
            for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                const bool fault = teleop.fault(idx);
                if (fault == last_fault[idx]) {
                    continue;
                }
                last_fault[idx] = fault;
                const std::string i = std::to_string(idx);
                if (fault) {
                    LogError("Fault detected on pair " + i + ". Press 'h" + i
                             + "' for details, 'F' to clear fault, then 'u" + i
                             + "' to restart the pair.");
                    PrintTeleopStatus(idx, teleop.GetTeleopStatus(idx));
                } else {
                    LogInfo(
                        "Fault cleared on pair " + i + ". Press 'u" + i + "' to restart the pair.");
                }
            }
            last_error.clear();
        } catch (const std::exception& e) {
            if (last_error != e.what()) {
                LogError(std::string("Exception in FaultMonitorTask: ") + e.what());
                last_error = e.what();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LogInfo("FaultMonitorTask exiting.");
}

/**
 * @brief Task for printing the message latency of all pairs every second while enabled with 'm'.
 */
void LinkMonitorTask(
    const flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs)
{
    std::string last_error;
    while (g_running.load()) {
        if (g_print_latency.load()) {
            try {
                for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                    PrintLatency(teleop, idx);
                }
                last_error.clear();
            } catch (const std::exception& e) {
                if (last_error != e.what()) {
                    LogError(std::string("Exception in LinkMonitorTask: ") + e.what());
                    last_error = e.what();
                }
            }
        }
        // Sleep 1 s in short steps so the task exits promptly
        for (int i = 0; i < 10 && g_running.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    LogInfo("LinkMonitorTask exiting.");
}

/**
 * @brief Task for calling TransparentCartesianTeleopWAN functions from console. [grippers] is null
 * unless this is the follower computer started with -n.
 */
void ConsoleTask(flexiv::tdk::TransparentCartesianTeleopWAN& teleop, unsigned int num_pairs,
    FollowerGrippers* grippers)
{
    auto PrintCommandMenu = [num_pairs, has_grippers = grippers != nullptr]() {
        if (num_pairs > 1) {
            std::cout << R"(
  Commands act on all pairs. Append a pair index to act on one pair only, e.g. "u1".)";
        }
        std::cout << R"(
  --- Teleop Engagement (leader only) ---
    r        : Engage teleop
    R        : Disengage teleop

  --- Null Space Posture ---
    i/I      : Set local nullspace to Preferred/Home posture

  --- Start/Stop ---
    u        : Init and start teleop
    U        : Stop teleop
    H        : Move all local robots to home posture (stop teleop first)

  --- Message latency ---
    l        : Print current message latency (disconnected / clock mismatch / over limit)
    m        : Toggle printing message latency every second

  --- Teleop status ---
    h        : Print why teleop is restricted / paused and what to do next

  --- Fault recovery ---
    F        : Clear fault on all local robots (blocking, up to 30 s), then press 'u' to restart

  --- Identity ---
    n        : Print role() and robot_pair_sn()
)";
        if (has_grippers) {
            std::cout << R"(
  --- Gripper (opened/closed by DI0 of the leader robots) ---
    g        : Print gripper states
    N        : Initialize gripper
)";
        }
        std::cout << R"(
  --- Exit ---
    quit     : Stop teleop and exit the program

  --- Help ---
    Any other key to show this help menu
        )" << std::endl;
    };

    while (g_running.load()) {

        std::string user_input {};

        // Get user input, exit when stdin is closed
        if (!std::getline(std::cin, user_input)) {
            LogInfo("Console input closed.");
            break;
        }

        if (user_input.empty()) {
            LogWarn("Empty command!");
            PrintCommandMenu();
            continue;
        }

        if (user_input == "quit") {
            LogInfo("Quit requested.");
            break;
        }

        const auto targets = TargetPairs(user_input, num_pairs);
        if (targets.empty()) {
            LogWarn(num_pairs == 1
                        ? "Invalid command! Single-arm teleop takes no pair index."
                        : "Invalid command! Valid pair index: 0-" + std::to_string(num_pairs - 1));
            PrintCommandMenu();
            continue;
        }
        const bool all_pairs = (targets.size() == num_pairs);

        try {
            switch (user_input[0]) {
                case 'r':
                    for (auto idx : targets) {
                        teleop.Engage(idx, true);
                    }
                    break;
                case 'R':
                    for (auto idx : targets) {
                        teleop.Engage(idx, false);
                    }
                    break;
                case 'i':
                    for (auto idx : targets) {
                        teleop.SetNullSpacePosture(idx, kPreferredJntPos[idx]);
                    }
                    break;
                case 'I':
                    for (auto idx : targets) {
                        teleop.SetNullSpacePosture(idx, kHomeJntPos);
                    }
                    break;
                case 'u':
                    if (all_pairs) {
                        teleop.Init();
                        teleop.Start();
                    } else {
                        teleop.InitWithIdx(targets[0]);
                        teleop.StartWithIdx(targets[0]);
                    }
                    // Engagement survives Stop/Init/Start, so restart disengaged
                    if (teleop.role() == flexiv::tdk::Role::WAN_TELEOP_LEADER) {
                        for (auto idx : targets) {
                            teleop.Engage(idx, false);
                        }
                    }
                    break;
                case 'U':
                    if (all_pairs) {
                        teleop.Stop();
                    } else {
                        teleop.StopWithIdx(targets[0]);
                    }
                    break;
                case 'H':
                    if (!all_pairs) {
                        LogWarn("'H' moves all local robots home; it takes no pair index");
                        break;
                    }
                    LogInfo("Moving all local robots to home posture...");
                    teleop.HomeAll();
                    LogInfo("All local robots are at home posture. Press 'u' to restart teleop.");
                    break;
                case 'l':
                    for (auto idx : targets) {
                        PrintLatency(teleop, idx);
                    }
                    break;
                case 'm':
                    g_print_latency = !g_print_latency.load();
                    LogInfo(std::string("Periodic latency print ")
                            + (g_print_latency.load() ? "enabled" : "disabled"));
                    break;
                case 'h':
                    for (auto idx : targets) {
                        PrintTeleopStatus(idx, teleop.GetTeleopStatus(idx));
                    }
                    break;
                case 'F': {
                    LogInfo("Clearing fault, this may take up to 30 seconds...");
                    const auto results = teleop.ClearFault();
                    for (size_t i = 0; i < results.size(); ++i) {
                        LogInfo("Teleop pair " + std::to_string(i)
                                + " clear fault: " + BoolStr(results[i]));
                    }
                    break;
                }
                case 'n':
                    for (auto idx : targets) {
                        const auto pair = teleop.robot_pair_sn(idx);
                        LogInfo("pair " + std::to_string(idx) + " role="
                                + flexiv::tdk::RoleTypeStr[static_cast<size_t>(teleop.role())]
                                + " leader_sn=" + pair.first + " follower_sn=" + pair.second);
                    }
                    break;
                case 'g':
                case 'N':
                    if (!grippers) {
                        LogWarn(
                            "Gripper commands are available on the follower computer started "
                            "with -n");
                        break;
                    }
                    for (auto idx : targets) {
                        if (user_input[0] == 'g') {
                            grippers->PrintStates(idx);
                        } else {
                            grippers->Init(idx);
                            LogInfo("Pair " + std::to_string(idx) + " gripper initialized");
                        }
                    }
                    break;

                default:
                    LogWarn("Invalid command!");
                    PrintCommandMenu();
                    break;
            }
        } catch (const std::exception& e) {
            LogError(std::string("Command '") + user_input + "' failed: " + e.what());
            try {
                for (auto idx : targets) {
                    PrintTeleopStatus(idx, teleop.GetTeleopStatus(idx));
                }
            } catch (const std::exception& status_e) {
                LogError(std::string("Failed to query teleop status: ") + status_e.what());
            }
        }
    }
    LogInfo("Console thread exiting.");
}

int main(int argc, char* argv[])
{
    // A dropped side link must not kill the program on the next send
    std::signal(SIGPIPE, SIG_IGN);

    std::array<std::string, kMaxPairs> leader_sn {};
    std::array<std::string, kMaxPairs> follower_sn {};
    std::string teleop_role, tcp_role, server_ip, gripper_name;
    unsigned int server_port = 0;
    std::optional<double> latency_limit_ms;
    std::vector<std::string> lan_interface_whitelist {};
    std::vector<std::string> wan_interface_whitelist {};
    bool enable_digital_input = false;
    bool home_on_start = false;

    int opt = 0;
    while ((opt = getopt_long_only(argc, argv, "l:f:L:F:r:t:i:p:A:W:T:n:DH", kLongOptions, nullptr))
           != -1) {
        switch (opt) {
            case 'l':
                leader_sn[0] = std::string(optarg);
                break;
            case 'f':
                follower_sn[0] = std::string(optarg);
                break;
            case 'L':
                leader_sn[1] = std::string(optarg);
                break;
            case 'F':
                follower_sn[1] = std::string(optarg);
                break;
            case 'r':
                teleop_role = std::string(optarg);
                break;
            case 't':
                tcp_role = std::string(optarg);
                break;
            case 'i':
                server_ip = std::string(optarg);
                break;
            case 'p': {
                const std::string port_str(optarg);
                unsigned long port = 0;
                size_t parsed = 0;
                try {
                    port = std::stoul(port_str, &parsed);
                } catch (...) {
                    parsed = 0;
                }
                if (port_str.empty() || port_str[0] == '-' || parsed != port_str.size() || port < 1
                    || port > 65535) {
                    LogError("Invalid port number: " + port_str + ", valid range is 1-65535");
                    return 1;
                }
                server_port = static_cast<unsigned int>(port);
                break;
            }
            case 'A':
                lan_interface_whitelist.push_back(std::string(optarg));
                break;
            case 'W':
                wan_interface_whitelist.push_back(std::string(optarg));
                break;
            case 'T': {
                const std::string limit_str(optarg);
                double limit = -1.0;
                size_t parsed = 0;
                try {
                    limit = std::stod(limit_str, &parsed);
                } catch (...) {
                    parsed = 0;
                }
                if (limit_str.empty() || parsed != limit_str.size()
                    || !(limit >= 0.0 && limit <= kMaxLatencyLimitMs)) {
                    LogError("Invalid latency limit: " + limit_str + " ms, valid range is 0-"
                             + FmtFixed(kMaxLatencyLimitMs, 0));
                    return 1;
                }
                latency_limit_ms = limit;
                break;
            }
            case 'D':
                enable_digital_input = true;
                break;
            case 'n':
                gripper_name = std::string(optarg);
                break;
            case 'H':
                home_on_start = true;
                break;
            default:
                PrintHelp();
                return 1;
        }
    }
    if (leader_sn[0].empty() || follower_sn[0].empty() || teleop_role.empty() || tcp_role.empty()
        || server_ip.empty() || server_port == 0) {
        PrintHelp();
        return 1;
    }

    // Single-arm teleop with one pair, or dual-arm teleop when the 2nd pair is given
    if (leader_sn[1].empty() != follower_sn[1].empty()) {
        LogError("[-L] and [-F] must be given together for dual-arm teleop");
        return 1;
    }
    const unsigned int num_pairs = leader_sn[1].empty() ? 1 : 2;

    // Gripper commands use the port after the teleop port
    const bool use_grippers = !gripper_name.empty();
    if (use_grippers && server_port == 65535) {
        LogError("[-p] must be below 65535 with [-n]: gripper commands use TCP port -p + 1");
        return 1;
    }

    // Whether this is a TCP server or client
    bool is_tcp_server;
    if (tcp_role == "server") {
        is_tcp_server = true;
    } else if (tcp_role == "client") {
        is_tcp_server = false;
    } else {
        LogError("Valid inputs for [-t] are: server, client");
        return 1;
    }

    // Whether this is leader or follower
    flexiv::tdk::Role role;
    if (teleop_role == "follower") {
        role = flexiv::tdk::Role::WAN_TELEOP_FOLLOWER;
    } else if (teleop_role == "leader") {
        role = flexiv::tdk::Role::WAN_TELEOP_LEADER;
    } else {
        LogError("Valid inputs for [-r] are: follower, leader");
        return 1;
    }
    const bool is_leader = (role == flexiv::tdk::Role::WAN_TELEOP_LEADER);

    // Network sanity checks, warnings only
    if (!IsPrivateIPv4(server_ip)) {
        LogWarn("[-i] " + server_ip
                + " is not a private IPv4 address. For teleop over a local WiFi network, use the "
                  "WiFi IPv4 address of the TCP server computer (see 'ip -4 addr show "
                  "<wifi_iface>').");
    }
    if (lan_interface_whitelist.empty()) {
        LogWarn(
            "LAN whitelist [-A] is not provided, RDK will search all network interfaces "
            "including WiFi. Pass the IPv4 address of the NIC wired to the robots.");
    }
    if (wan_interface_whitelist.empty()) {
        LogWarn(
            "WiFi interface [-W] is not provided, the teleop link may use any network "
            "interface. Pass the WiFi interface name, e.g. wlo1.");
    }
    if (enable_digital_input && !is_leader) {
        LogWarn("[-D] only applies to the leader and is ignored.");
    }
    PrintWiFiChecklist(is_tcp_server, server_port, use_grippers);

    // Network configuration: the TCP server's WiFi address replaces the public IPv4 address
    flexiv::tdk::NetworkCfgStd network_cfg;
    network_cfg.is_tcp_server = is_tcp_server;
    network_cfg.public_ipv4_address = server_ip;
    network_cfg.listening_port = server_port;
    network_cfg.lan_interface_whitelist = lan_interface_whitelist;
    network_cfg.wan_interface_whitelist = wan_interface_whitelist;

    std::vector<std::pair<std::string, std::string>> robot_sn_pairs {};
    for (unsigned int idx = 0; idx < num_pairs; ++idx) {
        robot_sn_pairs.push_back({leader_sn[idx], follower_sn[idx]});
    }
    LogInfo(num_pairs == 1 ? "Single-arm teleop" : "Dual-arm teleop");

    try {
        // Allocate tdk object with all robot pairs (Standard Edition TCP peer-to-peer)
        flexiv::tdk::TransparentCartesianTeleopWAN tctw(robot_sn_pairs, role, network_cfg);
        for (unsigned int idx = 0; idx < num_pairs; ++idx) {
            const auto pair_sn = tctw.robot_pair_sn(idx);
            LogInfo("Pair " + std::to_string(idx)
                    + " role=" + flexiv::tdk::RoleTypeStr[static_cast<size_t>(tctw.role())]
                    + " leader_sn=" + pair_sn.first + " follower_sn=" + pair_sn.second);
        }

        // Set message latency limit if specified
        if (latency_limit_ms) {
            for (unsigned int idx = 0; idx < num_pairs; ++idx) {
                tctw.SetTeleopLatencyLimit(idx, *latency_limit_ms);
            }
            LogInfo("Message latency limit set to " + FmtFixed(*latency_limit_ms, 1) + " ms");
        }

        // Optionally move all local robots to home posture first. HomeAll() needs enabled robots,
        // so enable them with an Init() that skips sensor zeroing; the Init() below then zeroes the
        // force/torque sensors at the home posture and switches back to teleop control.
        if (home_on_start) {
            LogWarn("Moving all local robots to home posture, make sure the workspace is clear");
            tctw.Init(true, flexiv::tdk::ZeroFTSensor::Disable);
            tctw.HomeAll();
            LogInfo("All local robots are at home posture");
        }

        // Init high transparency teleop for all pairs
        tctw.Init();

        // Start high transparency teleop for all pairs
        tctw.Start();

        // Gripper control: the follower enables its grippers and applies the commands that the
        // leader sends over the side link. Declared after tctw so they are destroyed before it.
        std::unique_ptr<FollowerGrippers> grippers;
        std::unique_ptr<SideLink> gripper_link;
        std::string gripper_error;
        if (use_grippers) {
            const unsigned int link_port = server_port + 1;
            if (is_leader) {
                gripper_link
                    = std::make_unique<SideLink>(is_tcp_server, server_ip, link_port, nullptr);
            } else {
                try {
                    grippers = std::make_unique<FollowerGrippers>(tctw, num_pairs, gripper_name);
                } catch (const std::exception& e) {
                    LogError("Gripper control disabled, failed to enable gripper [" + gripper_name
                             + "]: " + e.what());
                }
                if (grippers) {
                    gripper_link = std::make_unique<SideLink>(is_tcp_server, server_ip, link_port,
                        [&grippers, &gripper_error](const std::string& line) {
                            unsigned int idx = 0;
                            bool close = false;
                            if (!ParseGripperCommand(line, idx, close)) {
                                return;
                            }
                            try {
                                grippers->Apply(idx, close);
                                gripper_error.clear();
                            } catch (const std::exception& e) {
                                if (gripper_error != e.what()) {
                                    LogError("Pair " + std::to_string(idx)
                                             + " gripper command failed: " + e.what());
                                    gripper_error = e.what();
                                }
                            }
                        });
                }
            }
            if (gripper_link) {
                LogInfo("Gripper commands use TCP port " + std::to_string(link_port));
            }
        }

        // Start console, fault monitor and link monitor threads
        std::thread console_thread(ConsoleTask, std::ref(tctw), num_pairs, grippers.get());
        std::thread fault_monitor_thread(FaultMonitorTask, std::ref(tctw), num_pairs);
        std::thread link_monitor_thread(LinkMonitorTask, std::cref(tctw), num_pairs);

        // On the leader, read the pedal (-D) and the gripper inputs (-n)
        std::optional<std::thread> di_thread;
        const bool use_pedal = is_leader && enable_digital_input;
        SideLink* leader_gripper_link = is_leader ? gripper_link.get() : nullptr;
        if (use_pedal || leader_gripper_link) {
            LogInfo(std::string("Starting ReadDigitalInputTask:")
                    + (use_pedal ? " pedal on DI1 of the 1st leader robot engages all pairs;" : "")
                    + (leader_gripper_link ? " DI0 of each leader robot drives its follower's "
                                             "gripper (high: close);"
                                           : ""));
            di_thread.emplace(
                ReadDigitalInputTask, std::ref(tctw), num_pairs, use_pedal, leader_gripper_link);
        } else {
            LogInfo("ReadDigitalInputTask NOT started (needs -D or -n on the leader).");
        }

        // Wait for console_thread to finish
        console_thread.join();

        // Stop all threads, notify other threads exit
        g_running = false;

        // Wait for monitor and digital reading tasks to exit
        fault_monitor_thread.join();
        link_monitor_thread.join();
        if (di_thread && di_thread->joinable()) {
            di_thread->join();
        }

        // Close the gripper link before the grippers it drives
        gripper_link.reset();
        grippers.reset();

        // Exit high transparency teleop
        tctw.Stop();

    } catch (const std::exception& e) {
        LogError(e.what());
        return 1;
    }

    return 0;
}

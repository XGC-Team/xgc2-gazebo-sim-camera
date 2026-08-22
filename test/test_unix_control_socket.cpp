#include "unix_control_socket.h"

#include <cstdio>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <string>

namespace {

using gazebo_sim_camera::BindAndListenUnixControlSocket;
using gazebo_sim_camera::CheckMediaRuntimeAncestors;
using gazebo_sim_camera::CloseListeningUnixControlSocket;
using gazebo_sim_camera::IsSafeAbsoluteUnixSocketPath;
using gazebo_sim_camera::IsXgc2PrivateMediaSocketPath;
using gazebo_sim_camera::RemoveStaleUnixSocket;

TEST(UnixControlSocketPath, AcceptsCurrentMediaRuntimeNamespaces) {
  EXPECT_TRUE(IsXgc2PrivateMediaSocketPath(
      "/run/xgc2-local-fleet/media/gazebo_world_camera.sock"));
  EXPECT_TRUE(
      IsXgc2PrivateMediaSocketPath("/run/xgc2-local-fleet/media/usb_cam.sock"));
  EXPECT_TRUE(IsXgc2PrivateMediaSocketPath("/tmp/xgc2/media/usb_cam.sock"));
}

TEST(UnixControlSocketPath, RejectsSocketsOutsidePrivateMediaRoots) {
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/var/run/docker.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/run/docker.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/var/run/xgc2/front.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/run/xgc2/camera.sock"));
  EXPECT_FALSE(
      IsXgc2PrivateMediaSocketPath("/run/xgc2-agent/media/odin1.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/tmp/xgc2/media/nested/cam.sock"));
  EXPECT_TRUE(IsSafeAbsoluteUnixSocketPath("/var/run/docker.sock"));
  EXPECT_EQ(
      RemoveStaleUnixSocket("/var/run/docker.sock"),
      "control socket path is not under an XGC2 private media runtime root");
}

TEST(UnixControlSocketPath, RejectsEmptyRelativeAndTraversal) {
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath(""));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("gazebo_world_camera.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/run/xgc2-local-fleet/media/"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/run/xgc2-local-fleet/media/foo"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath(
      "/run/xgc2-local-fleet/media/../gazebo_world_camera.sock"));
  EXPECT_FALSE(
      IsXgc2PrivateMediaSocketPath("/tmp/xgc2/media/../../../etc/passwd.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/run/./xgc2/media/cam.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/run//xgc2/media/cam.sock"));
}

TEST(UnixControlSocketPath, RejectsUnsafeNamesAndControls) {
  EXPECT_FALSE(
      IsXgc2PrivateMediaSocketPath("/run/xgc2-local-fleet/media/.sock"));
  EXPECT_FALSE(
      IsXgc2PrivateMediaSocketPath("/run/xgc2-local-fleet/media/.hidden.sock"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/tmp/xgc2/media/cam.sock/"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/tmp/xgc2/media/cam.sock extra"));
  EXPECT_FALSE(IsXgc2PrivateMediaSocketPath("/tmp/xgc2/media/cam\nsock.sock"));
}

class UnixControlSocketRemove : public ::testing::Test {
protected:
  void SetUp() override {
    if (mkdir("/tmp/xgc2", 0750) != 0) {
      ASSERT_EQ(errno, EEXIST);
    }
    if (mkdir("/tmp/xgc2/media", 0750) != 0) {
      ASSERT_EQ(errno, EEXIST);
    }
    socket_path_ =
        "/tmp/xgc2/media/xgc-gtest-" + std::to_string(getpid()) + ".sock";
    unlink(socket_path_.c_str());
  }

  void TearDown() override {
    if (!socket_path_.empty()) {
      unlink(socket_path_.c_str());
    }
  }

  int BindUnixSocket(const std::string &path) {
    const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) {
      return -1;
    }
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::snprintf(address.sun_path, sizeof(address.sun_path), "%s",
                  path.c_str());
    if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) !=
        0) {
      close(fd);
      return -1;
    }
    return fd;
  }

  std::string socket_path_;
};

TEST_F(UnixControlSocketRemove, UnlinksAnExistingSocketInPrivateRoot) {
  ASSERT_TRUE(IsXgc2PrivateMediaSocketPath(socket_path_));
  const int fd = BindUnixSocket(socket_path_);
  ASSERT_GE(fd, 0);
  close(fd);
  EXPECT_EQ(RemoveStaleUnixSocket(socket_path_), "");
  struct stat info {};
  EXPECT_EQ(lstat(socket_path_.c_str(), &info), -1);
  EXPECT_EQ(errno, ENOENT);
  socket_path_.clear();
}

TEST_F(UnixControlSocketRemove, RefusesToUnlinkARegularFile) {
  const int fd = open(socket_path_.c_str(), O_CREAT | O_WRONLY, 0600);
  ASSERT_GE(fd, 0);
  close(fd);
  EXPECT_EQ(RemoveStaleUnixSocket(socket_path_),
            "control socket path exists and is not a unix socket");
  struct stat info {};
  ASSERT_EQ(lstat(socket_path_.c_str(), &info), 0);
  EXPECT_TRUE(S_ISREG(info.st_mode));
}

TEST_F(UnixControlSocketRemove, RefusesToUnlinkASymlink) {
  ASSERT_EQ(symlink("/etc/passwd", socket_path_.c_str()), 0);
  EXPECT_EQ(RemoveStaleUnixSocket(socket_path_),
            "media runtime path component is a symlink");
  struct stat info {};
  ASSERT_EQ(lstat(socket_path_.c_str(), &info), 0);
  EXPECT_TRUE(S_ISLNK(info.st_mode));
}

TEST_F(UnixControlSocketRemove, RefusesToUnlinkADirectory) {
  ASSERT_EQ(mkdir(socket_path_.c_str(), 0700), 0);
  EXPECT_EQ(RemoveStaleUnixSocket(socket_path_),
            "control socket path exists and is not a unix socket");
  struct stat info {};
  ASSERT_EQ(lstat(socket_path_.c_str(), &info), 0);
  EXPECT_TRUE(S_ISDIR(info.st_mode));
  ASSERT_EQ(rmdir(socket_path_.c_str()), 0);
  socket_path_.clear();
}

TEST(UnixControlSocketAncestors, AcceptsExistingPrivateMediaRoot) {
  if (mkdir("/tmp/xgc2", 0750) != 0) {
    ASSERT_EQ(errno, EEXIST);
  }
  if (mkdir("/tmp/xgc2/media", 0750) != 0) {
    ASSERT_EQ(errno, EEXIST);
  }
  EXPECT_EQ(CheckMediaRuntimeAncestors("/tmp/xgc2/media/usb_cam.sock"), "");
}

TEST(UnixControlSocketRemoveLexical, RejectsUnsafePathsWithoutTouchingDisk) {
  EXPECT_EQ(
      RemoveStaleUnixSocket(""),
      "control socket path is not under an XGC2 private media runtime root");
  EXPECT_EQ(
      RemoveStaleUnixSocket("/run/docker.sock"),
      "control socket path is not under an XGC2 private media runtime root");
  EXPECT_EQ(
      RemoveStaleUnixSocket("/run/xgc2-local-fleet/media/../cam.sock"),
      "control socket path is not under an XGC2 private media runtime root");
}

TEST_F(UnixControlSocketRemove, ListensOnPrivateMediaSocketAndAcceptsAClient) {
  std::string error;
  int fd = BindAndListenUnixControlSocket(socket_path_, &error);
  ASSERT_GE(fd, 0) << error;
  int accepting = 0;
  socklen_t accepting_size = sizeof(accepting);
  ASSERT_EQ(
      getsockopt(fd, SOL_SOCKET, SO_ACCEPTCONN, &accepting, &accepting_size),
      0);
  EXPECT_NE(accepting, 0);
  const int client = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  ASSERT_GE(client, 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::snprintf(address.sun_path, sizeof(address.sun_path), "%s",
                socket_path_.c_str());
  ASSERT_EQ(
      connect(client, reinterpret_cast<sockaddr *>(&address), sizeof(address)),
      0);
  close(client);
  EXPECT_EQ(CloseListeningUnixControlSocket(&fd, socket_path_), "");
  EXPECT_EQ(fd, -1);
  struct stat info {};
  EXPECT_EQ(lstat(socket_path_.c_str(), &info), -1);
  EXPECT_EQ(errno, ENOENT);
  socket_path_.clear();
}

TEST(UnixControlSocketListen, RejectsForeignSocketsWithoutCreatingThem) {
  std::string error;
  EXPECT_EQ(BindAndListenUnixControlSocket("/var/run/docker.sock", &error), -1);
  EXPECT_EQ(
      error,
      "control socket path is not under an XGC2 private media runtime root");
  EXPECT_EQ(BindAndListenUnixControlSocket("/run/xgc2-agent/media/odin1.sock",
                                           &error),
            -1);
  EXPECT_EQ(
      error,
      "control socket path is not under an XGC2 private media runtime root");
  EXPECT_EQ(
      BindAndListenUnixControlSocket("/tmp/xgc2/media/nested/cam.sock", &error),
      -1);
  EXPECT_EQ(
      error,
      "control socket path is not under an XGC2 private media runtime root");
}

} // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

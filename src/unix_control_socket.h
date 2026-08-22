#pragma once

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>

namespace gazebo_sim_camera {

inline constexpr std::size_t MaxUnixSocketPathBytes() {
  return sizeof(sockaddr_un::sun_path) - 1;
}

// Current namespaces for this capture-source plugin. They are two live product
// roots, each a current media runtime:
//   /tmp/xgc2/media/              Gazebo camera package and catalog default
//   /run/xgc2-local-fleet/media/  local-fleet station runtime
// Other XGC2 products keep their own roots (for example
// /run/xgc2-agent/media/).
inline constexpr std::string_view kPrivateMediaRuntimeRoots[] = {
    "/tmp/xgc2/media/",
    "/run/xgc2-local-fleet/media/",
};

inline bool IsSafeUnixSocketFileName(std::string_view name) {
  constexpr std::string_view suffix = ".sock";
  if (name.size() <= suffix.size() || name.size() > 128 ||
      name.substr(name.size() - suffix.size()) != suffix) {
    return false;
  }
  const std::string_view stem = name.substr(0, name.size() - suffix.size());
  if (stem.empty() || stem.front() == '.' || stem.back() == '.') {
    return false;
  }
  return std::all_of(stem.begin(), stem.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '-' || character == '_' ||
           character == '.';
  });
}

inline bool IsSafeAbsoluteUnixSocketPath(std::string_view path) {
  if (path.empty() || path.size() > MaxUnixSocketPathBytes() ||
      path.front() != '/' || path.back() == '/') {
    return false;
  }
  for (unsigned char character : path) {
    if (character < 0x21 || character > 0x7e) {
      return false;
    }
  }
  std::size_t begin = 1;
  bool saw_file = false;
  while (begin < path.size()) {
    const std::size_t end = path.find('/', begin);
    const std::string_view component = path.substr(
        begin, (end == std::string_view::npos ? path.size() : end) - begin);
    if (component.empty() || component == "." || component == "..") {
      return false;
    }
    if (end == std::string_view::npos) {
      if (!IsSafeUnixSocketFileName(component)) {
        return false;
      }
      saw_file = true;
      break;
    }
    if (!std::all_of(component.begin(), component.end(),
                     [](unsigned char character) {
                       return std::isalnum(character) || character == '-' ||
                              character == '_' || character == '.';
                     })) {
      return false;
    }
    begin = end + 1;
  }
  return saw_file;
}

inline bool IsXgc2PrivateMediaSocketPath(std::string_view path) {
  if (!IsSafeAbsoluteUnixSocketPath(path)) {
    return false;
  }
  for (const std::string_view root : kPrivateMediaRuntimeRoots) {
    if (path.size() > root.size() && path.compare(0, root.size(), root) == 0) {
      const std::string_view rest = path.substr(root.size());
      if (rest.find('/') == std::string_view::npos &&
          IsSafeUnixSocketFileName(rest)) {
        return true;
      }
    }
  }
  return false;
}

inline std::vector<std::string>
SplitAbsolutePathComponents(std::string_view path) {
  std::vector<std::string> components;
  std::size_t begin = 1;
  while (begin < path.size()) {
    const std::size_t end = path.find('/', begin);
    const std::size_t count =
        (end == std::string_view::npos ? path.size() : end) - begin;
    components.emplace_back(path.substr(begin, count));
    if (end == std::string_view::npos) {
      break;
    }
    begin = end + 1;
  }
  return components;
}

inline bool IsOsPrefixComponent(std::string_view component) {
  return component == "tmp" || component == "run";
}

// Walks the lexical path with AT_SYMLINK_NOFOLLOW. The first component may be
// an OS prefix (`tmp` or `run`). Every XGC namespace component and the socket
// itself must not be a symlink.
inline std::string CheckMediaRuntimeAncestors(const std::string &path) {
  if (!IsXgc2PrivateMediaSocketPath(path)) {
    return "control socket path is not under an XGC2 private media runtime "
           "root";
  }
  const std::vector<std::string> components = SplitAbsolutePathComponents(path);
  int dir = open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir < 0) {
    return std::string("cannot open filesystem root: ") + std::strerror(errno);
  }
  for (std::size_t index = 0; index < components.size(); ++index) {
    const std::string &component = components[index];
    const bool last = index + 1 == components.size();
    const bool os_prefix = index == 0 && IsOsPrefixComponent(component);
    struct stat info {};
    if (fstatat(dir, component.c_str(), &info, AT_SYMLINK_NOFOLLOW) != 0) {
      if (errno == ENOENT) {
        close(dir);
        return {};
      }
      const std::string error = std::strerror(errno);
      close(dir);
      return std::string("cannot inspect media runtime path: ") + error;
    }
    if (S_ISLNK(info.st_mode) && !os_prefix) {
      close(dir);
      return "media runtime path component is a symlink";
    }
    if (last) {
      close(dir);
      return {};
    }
    const int flags =
        O_RDONLY | O_DIRECTORY | O_CLOEXEC | (os_prefix ? 0 : O_NOFOLLOW);
    const int next = openat(dir, component.c_str(), flags);
    close(dir);
    if (next < 0) {
      if (errno == ENOENT) {
        return {};
      }
      return std::string("cannot open media runtime directory: ") +
             std::strerror(errno);
    }
    dir = next;
  }
  close(dir);
  return {};
}

inline std::string EnsurePrivateMediaSocketParent(const std::string &path) {
  const std::string ancestors = CheckMediaRuntimeAncestors(path);
  if (!ancestors.empty()) {
    return ancestors;
  }
  const std::vector<std::string> components = SplitAbsolutePathComponents(path);
  int dir = open("/", O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dir < 0) {
    return std::string("cannot open filesystem root: ") + std::strerror(errno);
  }
  for (std::size_t index = 0; index + 1 < components.size(); ++index) {
    const std::string &component = components[index];
    const bool os_prefix = index == 0 && IsOsPrefixComponent(component);
    struct stat info {};
    if (fstatat(dir, component.c_str(), &info, AT_SYMLINK_NOFOLLOW) != 0) {
      if (errno != ENOENT) {
        const std::string error = std::strerror(errno);
        close(dir);
        return std::string("cannot inspect media runtime path: ") + error;
      }
      if (os_prefix) {
        close(dir);
        return "operating-system media prefix is missing";
      }
      if (mkdirat(dir, component.c_str(), 0750) != 0 && errno != EEXIST) {
        const std::string error = std::strerror(errno);
        close(dir);
        return std::string("cannot create media runtime directory: ") + error;
      }
    } else if (S_ISLNK(info.st_mode) && !os_prefix) {
      close(dir);
      return "media runtime path component is a symlink";
    } else if (!S_ISDIR(info.st_mode) && !S_ISLNK(info.st_mode)) {
      close(dir);
      return "media runtime path component is not a directory";
    }
    const int flags =
        O_RDONLY | O_DIRECTORY | O_CLOEXEC | (os_prefix ? 0 : O_NOFOLLOW);
    const int next = openat(dir, component.c_str(), flags);
    close(dir);
    if (next < 0) {
      return std::string("cannot open media runtime directory: ") +
             std::strerror(errno);
    }
    dir = next;
  }
  close(dir);
  return {};
}

inline std::string RemoveStaleUnixSocket(const std::string &path) {
  const std::string ancestors = CheckMediaRuntimeAncestors(path);
  if (!ancestors.empty()) {
    return ancestors;
  }
  struct stat info {};
  if (lstat(path.c_str(), &info) != 0) {
    if (errno == ENOENT) {
      return {};
    }
    return std::string("cannot inspect control socket: ") +
           std::strerror(errno);
  }
  if (S_ISLNK(info.st_mode) || !S_ISSOCK(info.st_mode)) {
    return "control socket path exists and is not a unix socket";
  }
  if (unlink(path.c_str()) != 0 && errno != ENOENT) {
    return std::string("cannot remove stale control socket: ") +
           std::strerror(errno);
  }
  return {};
}

// Bind+listen a stream unix socket under a current private media root.
// Fail-closed: never creates a socket outside those roots.
inline int BindAndListenUnixControlSocket(const std::string &path,
                                          std::string *error) {
  const auto fail = [&](const std::string &message) {
    if (error != nullptr) {
      *error = message;
    }
    return -1;
  };
  if (!IsXgc2PrivateMediaSocketPath(path)) {
    return fail(
        "control socket path is not under an XGC2 private media runtime root");
  }
  if (path.size() >= sizeof(sockaddr_un::sun_path)) {
    return fail("control socket path is too long");
  }
  const std::string parent = EnsurePrivateMediaSocketParent(path);
  if (!parent.empty()) {
    return fail(parent);
  }
  const std::string stale = RemoveStaleUnixSocket(path);
  if (!stale.empty()) {
    return fail(stale);
  }
  const int fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return fail(std::string("cannot create control socket: ") +
                std::strerror(errno));
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
  if (bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) !=
          0 ||
      listen(fd, 8) != 0 || chmod(path.c_str(), 0600) != 0) {
    const std::string message =
        std::string("cannot bind control socket: ") + std::strerror(errno);
    close(fd);
    const std::string leftover = RemoveStaleUnixSocket(path);
    if (error != nullptr) {
      *error = leftover.empty() ? message : leftover;
    }
    return -1;
  }
  if (error != nullptr) {
    error->clear();
  }
  return fd;
}

inline std::string CloseListeningUnixControlSocket(int *fd,
                                                   const std::string &path) {
  if (fd != nullptr && *fd >= 0) {
    shutdown(*fd, SHUT_RDWR);
    close(*fd);
    *fd = -1;
  }
  if (path.empty()) {
    return {};
  }
  return RemoveStaleUnixSocket(path);
}

} // namespace gazebo_sim_camera

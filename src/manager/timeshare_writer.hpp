#pragma once

// Timeshare: share the latest LiDAR frame timestamp with other processes.
//
// The legacy driver (WilsonGuo/HesaiLidar_General_ROS_timeshare) exposes the
// latest point-cloud timestamp through an mmap'ed file at $HOME/timeshare,
// holding the LiDAR timestamp in nanoseconds since the Unix epoch.
//
// Byte layout note: the legacy Hesai driver wrote a single uint64_t (8 bytes,
// offset 0). The camera/trigger nodes shipped in this NUC workspace use
//     struct time_stamp { int64_t high; int64_t low; };
// and read `low` at offset 8 (the same layout the Livox drivers write), so a
// pure 8-byte file would read as 0 there. To stay compatible with both kinds of
// consumers, this writer creates a 16-byte file and writes the same nanosecond
// value into BOTH 64-bit slots:
//     offset 0 : legacy `low`
//     offset 8 : `low` for the high/low layout (high remains unused by readers)
// Readers that only consume offset 0 keep working; readers using the
// high/low struct also get a valid nanosecond timestamp.

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <pwd.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

class TimeshareWriter {
 public:
  TimeshareWriter() = default;
  ~TimeshareWriter() { Close(); }

  TimeshareWriter(const TimeshareWriter&) = delete;
  TimeshareWriter& operator=(const TimeshareWriter&) = delete;

  bool Init() {
    if (mapped_ != nullptr) {
      return true;
    }

    std::string path;
    const char* home = std::getenv("HOME");
    if (home != nullptr && home[0] != '\0') {
      path = std::string(home) + "/timeshare";
    } else {
      struct passwd* pw = getpwuid(getuid());
      if (pw == nullptr || pw->pw_dir == nullptr) {
        std::fprintf(stderr, "[timeshare] cannot get HOME directory\n");
        return false;
      }
      path = std::string(pw->pw_dir) + "/timeshare";
    }

    fd_ = open(path.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
      std::fprintf(stderr, "[timeshare] open %s failed: %s\n", path.c_str(),
                   std::strerror(errno));
      return false;
    }

    if (ftruncate(fd_, static_cast<off_t>(kBytes)) != 0) {
      std::fprintf(stderr, "[timeshare] ftruncate %s failed: %s\n", path.c_str(),
                   std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    void* addr = mmap(nullptr, kBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
                      0);
    if (addr == MAP_FAILED) {
      std::fprintf(stderr, "[timeshare] mmap %s failed: %s\n", path.c_str(),
                   std::strerror(errno));
      close(fd_);
      fd_ = -1;
      return false;
    }

    mapped_ = static_cast<uint64_t*>(addr);
    mapped_[0] = 0;
    mapped_[1] = 0;
    std::fprintf(stdout,
                 "[timeshare] mapped %s (uint64 nanoseconds, offsets 0 and 8)\n",
                 path.c_str());
    std::fflush(stdout);
    return true;
  }

  // timestamp_sec: LiDAR frame start time in seconds since the Unix epoch.
  void Write(double timestamp_sec) {
    if (mapped_ == nullptr || timestamp_sec <= 0.0) {
      return;
    }
    const uint64_t ns = static_cast<uint64_t>(timestamp_sec * 1000000000.0);
    mapped_[0] = ns;  // legacy single-uint64 layout (offset 0)
    mapped_[1] = ns;  // high/low layout consumers read `low` at offset 8
  }

  void Close() {
    if (mapped_ != nullptr) {
      munmap(mapped_, kBytes);
      mapped_ = nullptr;
    }
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

 private:
  static constexpr size_t kBytes = sizeof(uint64_t) * 2;
  int fd_ = -1;
  uint64_t* mapped_ = nullptr;
};

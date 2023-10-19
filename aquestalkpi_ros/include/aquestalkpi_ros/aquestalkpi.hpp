#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace aquestalkpi_ros {

class AquesTalkPi {
public:
  AquesTalkPi(const std::string &path) : path_(path) {
    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("AquesTalkPi not found.");
    }
  }

  template <class T>
  void synthesize(const std::string &text, const std::string &voice, int speed,
                  std::vector<T> &out, std::string &err) {
    std::string speed_str = std::to_string(speed);
    int out_pipe[2], err_pipe[2];
    pipe(out_pipe);
    pipe(err_pipe);
    pid_t pid = fork();

    if (pid == 0) {
      dup2(out_pipe[1], STDOUT_FILENO);
      dup2(err_pipe[1], STDERR_FILENO);
      close(out_pipe[0]);
      close(out_pipe[1]);
      close(err_pipe[0]);
      close(err_pipe[1]);
      execl(path_.c_str(), path_.c_str(), "-v", voice.c_str(), "-s",
            speed_str.c_str(), text.c_str(), nullptr);
    } else {
      close(out_pipe[1]);
      close(err_pipe[1]);
      read_from_fd(out_pipe[0], out);
      read_from_fd(err_pipe[0], err);
      close(out_pipe[0]);
      close(err_pipe[0]);
      waitpid(pid, nullptr, 0);
      if (!err.empty()) {
        for (auto itr = err.rbegin(); isspace(*itr); ++itr) {
          err.pop_back();
        }
      }
    }
  }

private:
  std::string path_;

  template <class T> void read_from_fd(int fd, T &out) {
    size_t current = 0;
    size_t chunk = 4096;

    while (true) {
      out.resize(current + chunk);
      size_t n = read(fd, out.data() + current, chunk);
      current += n;
      if (!n) {
        out.resize(current);
        break;
      }
    }
  }
};

}

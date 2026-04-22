#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>

namespace quad_rope_lift {

/// Writes a per-run YAML manifest capturing everything needed to
/// reproduce a single simulation: the full CLI invocation, UTC
/// timestamps, hostname, git SHA, binary SHA256, platform, and
/// seed. Called once per run from the simulator main; the manifest
/// lands next to the CSV output and is consumed by the campaign
/// post-processing scripts.
struct RunManifest {
  std::string output_dir;
  std::string experiment_id;
  std::string scenario;
  std::string controller;
  int    num_quads = 0;
  double duration  = 0.0;
  int    seed      = 0;
  std::string invocation;   // full argv joined with spaces
  std::string git_sha;
  std::string binary_sha256;
  std::string hostname;
  std::string utc_start;
  std::string utc_end;

  static std::string RunShell(const char* cmd) {
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return {};
    std::string out;
    char buf[256];
    while (std::fgets(buf, sizeof buf, pipe.get()) != nullptr) {
      out += buf;
    }
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) {
      out.pop_back();
    }
    return out;
  }

  static std::string UtcNow() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
    return buf;
  }

  static std::string Hostname() {
    char buf[256];
    if (::gethostname(buf, sizeof buf) == 0) return buf;
    return "unknown";
  }

  void Populate(int argc, char** argv, const std::string& exe_path) {
    std::ostringstream ss;
    for (int i = 0; i < argc; ++i) {
      if (i) ss << ' ';
      ss << argv[i];
    }
    invocation = ss.str();
    git_sha    = RunShell("git -C /workspaces/Tether_Grace rev-parse HEAD 2>/dev/null");
    const std::string sha_cmd = "sha256sum " + exe_path + " 2>/dev/null | awk '{print $1}'";
    binary_sha256 = RunShell(sha_cmd.c_str());
    hostname   = Hostname();
    utc_start  = UtcNow();
  }

  void Finalise() { utc_end = UtcNow(); }

  void Write(const std::string& path) const {
    std::ofstream f(path);
    f << "experiment_id: " << experiment_id << '\n'
      << "scenario: "      << scenario      << '\n'
      << "controller: "    << controller    << '\n'
      << "num_quads: "     << num_quads     << '\n'
      << "duration_s: "    << duration      << '\n'
      << "seed: "          << seed          << '\n'
      << "invocation: '"   << invocation    << "'\n"
      << "git_sha: "       << git_sha       << '\n'
      << "binary_sha256: " << binary_sha256 << '\n'
      << "hostname: "      << hostname      << '\n'
      << "utc_start: "     << utc_start     << '\n'
      << "utc_end: "       << utc_end       << '\n';
  }
};

}  // namespace quad_rope_lift

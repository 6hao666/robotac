#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace {

struct DeviceRecord {
  std::string ip;
  std::string serial;
  uint8_t type;
};

std::map<std::string, DeviceRecord> devices;
std::mutex devices_mutex;

std::string DeviceTypeName(uint8_t type) {
  switch (type) {
    case kLivoxLidarTypeMid360:
      return "MID360";
    case kLivoxLidarTypeMid360s:
      return "MID360s";
    case kLivoxLidarTypeHAP:
    case kLivoxLidarTypeIndustrialHAP:
      return "HAP";
    case kLivoxLidarTypeAvia:
      return "Avia";
    default:
      return "Livox";
  }
}

std::string JsonEscape(const std::string& value) {
  std::string escaped;
  for (char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

void LidarInfoChangeCallback(const uint32_t,
                             const LivoxLidarInfo* info,
                             void*) {
  if (info == nullptr || info->lidar_ip[0] == '\0') {
    return;
  }
  DeviceRecord record;
  record.ip = info->lidar_ip;
  record.serial = info->sn;
  record.type = info->dev_type;
  std::lock_guard<std::mutex> lock(devices_mutex);
  devices[record.ip] = record;
}

void Usage(const char* program) {
  std::cerr << "用法：" << program
            << " --host-ip <地址> [--timeout <秒>] [--json]" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  std::string host_ip;
  int timeout_seconds = 5;
  bool json_output = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--host-ip" && index + 1 < argc) {
      host_ip = argv[++index];
    } else if (argument == "--timeout" && index + 1 < argc) {
      timeout_seconds = std::atoi(argv[++index]);
    } else if (argument == "--json") {
      json_output = true;
    } else {
      Usage(argv[0]);
      return 64;
    }
  }

  if (host_ip.empty() || timeout_seconds < 1 || timeout_seconds > 60) {
    Usage(argv[0]);
    return 64;
  }

  DisableLivoxSdkConsoleLogger();
  EnableLivoxLidarDiscoveryOnly();
  if (!LivoxLidarSdkInit(nullptr, host_ip.c_str())) {
    std::cerr << "Livox SDK 无法在主机地址 " << host_ip << " 上初始化。"
              << std::endl;
    return 69;
  }

  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);
  LivoxLidarSdkStart();
  std::this_thread::sleep_for(std::chrono::seconds(timeout_seconds));
  LivoxLidarSdkUninit();

  std::lock_guard<std::mutex> lock(devices_mutex);
  if (json_output) {
    std::cout << "[";
    bool first = true;
    for (const auto& item : devices) {
      const DeviceRecord& device = item.second;
      if (!first) {
        std::cout << ",";
      }
      first = false;
      std::cout << "{\"lidar_ip\":\"" << JsonEscape(device.ip)
                << "\",\"serial\":\"" << JsonEscape(device.serial)
                << "\",\"device_type\":" << static_cast<int>(device.type)
                << ",\"device_name\":\""
                << JsonEscape(DeviceTypeName(device.type)) << "\"}";
    }
    std::cout << "]" << std::endl;
  } else {
    for (const auto& item : devices) {
      const DeviceRecord& device = item.second;
      std::cout << DeviceTypeName(device.type) << "\t" << device.ip << "\t"
                << device.serial << std::endl;
    }
  }

  return devices.empty() ? 4 : 0;
}

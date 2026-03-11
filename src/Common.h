#pragma once

#include <string>
#include <sstream>
#include <vector>
#include <array>
#include <map>
#include <set>
#include <optional>
#include <memory>
#include <mutex>
#include <atomic>
#include <queue>
#include <functional>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <filesystem>
#include <cstdint>
#include <chrono>
#include <iomanip>

//#define DEBUG 1

using namespace std::chrono_literals;
using namespace std::string_literals;

template <typename TType>
using Atomic = std::atomic<TType>;

using AtomicBool = Atomic<bool>;
using Mutex = std::mutex;
using UniqueLock = std::unique_lock<Mutex>;
using LockGuard = std::lock_guard<Mutex>;
using ConditionVariable = std::condition_variable;
using Thread = std::thread;

template <typename TVal>
using Queue = std::queue<TVal>;

using String = std::string;
using StringStream = std::stringstream;
using WString = std::wstring;
using WStringStream = std::wstringstream;

template <typename TType, size_t TSize>
using StaticArray = std::array<TType, TSize>;
template <size_t TSize>
using StaticByteArray = StaticArray<uint8_t, TSize>;
template <typename TType>
using DynamicContainer = std::vector<TType>;
using DynamicByteContainer = DynamicContainer<uint8_t>;

template <typename TKey, typename TVal>
using Map = std::map<TKey, TVal>;
template <typename TKey>
using Set = std::set<TKey>;
template <typename TVal>
using UniquePtr = std::unique_ptr<TVal>;
template <typename TVal>
using Option = std::optional<TVal>;

using IFStream = std::ifstream;
using OFStream = std::ofstream;
using FS = std::filesystem::filesystem_error;
using Path = std::filesystem::path;

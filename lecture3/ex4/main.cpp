#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

int counter = 0;

constexpr int N = 50000;  // 常量表达式，强制要求在编译期确定值

std::mutex mutex;

void work()
{
  std::lock_guard<std::mutex> lock(mutex);  // 使用互斥锁保护对 counter 的访问
  for (int i = 0; i < N; ++i) {
    counter++;
  }
}

int main()
{
  std::vector<std::thread> threads;

  for (int i = 0; i < 8; ++i) {
    threads.emplace_back(work);
  }

  for (auto & t : threads) {
    t.join();
  }

  std::cout << "expected: " << 8 * N << '\n';
  std::cout << "actual:   " << counter << '\n';
}
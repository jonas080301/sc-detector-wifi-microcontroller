// Copyright 2024 Fabian Steger
#ifndef CORE_CPP_SOMETHINGINCPP_HPP_
#define CORE_CPP_SOMETHINGINCPP_HPP_

#include <cstdint>

/**
 * @brief Beispiel Klasse um Klassen zu lehren
 */
class SomethingInCPP {
private:
  uint32_t myvalue{4711};

public:
  SomethingInCPP();
  void Increase();
  [[nodiscard]] uint32_t GetValue() const;
  void Set(uint32_t arg);
};

#endif // CORE_CPP_SOMETHINGINCPP_HPP_

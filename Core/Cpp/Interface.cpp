// Copyright 2024 Fabian Steger

#include "Interface.hpp"
#include "../Inc/main.h"
#include "SomethingInCPP.hpp"

// einfaches "Interface" zum Plain-C-Teil
extern "C" void do_cpp_things(void) {
// Code nur Demo - die Studis im Basismodul machen kein c++ - und sollen auch
// keine Warnung aus dem Teil des Demoprojektes bekommen - zumindest beim
// Kompilieren nicht
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

  // erzeugt eine instanz auf dem stack, ruft funktionen auf
  SomethingInCPP SampleClassInstance;
  SomethingInCPP SampleClassInstanceB;

  SampleClassInstanceB.Set(999);
  SampleClassInstance.Increase();
  volatile uint32_t value1 = SampleClassInstance.GetValue();
  SampleClassInstance.Increase();
  volatile uint32_t value2 = SampleClassInstance.GetValue();
  SampleClassInstanceB.Increase();
  volatile uint32_t value3 = SampleClassInstanceB.GetValue();
  // Hier breakpoint setzen, values anschauen
  __NOP();
  // nachdem die ja am stack liegen werden die instanzen jetzt geloescht

#pragma GCC diagnostic pop
}
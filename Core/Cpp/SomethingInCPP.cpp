// Copyright 2024 Fabian Steger

#include "SomethingInCPP.hpp"
#include "../Inc/main.h"

/**
 * @brief Liest den Private-Wert aus
 *
 * @return uint32_t Wert
 */
uint32_t SomethingInCPP::GetValue() const { return myvalue; }

/**
 * Konstruktor der Klasse, tut nichts (Standardwerte alle in HPP gesetzt)
 */
SomethingInCPP::SomethingInCPP() = default;

/**
 * @brief Erhoeht den Private-Wert um 1
 *
 */
void SomethingInCPP::Increase() { myvalue++; }

/**
 * @brief Setzt den Private-Wert auf den im Parameter angegebenen Wert
 *
 * @param arg Der neue Private-Wert
 */
void SomethingInCPP::Set(uint32_t arg) { myvalue = arg; }

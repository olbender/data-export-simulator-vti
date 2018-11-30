#include <iostream>

#include "decoder.hpp"

Decoder::Decoder(
    std::string const &a_filename
    ) noexcept
    : m_file{nullptr} 
{
  m_file = fopen(a_filename.c_str(), "r");
  if(m_file == nullptr) {
    std::clog << "Failed to open file." << std::endl;
  }
}

Decoder::~Decoder() {
    fclose(m_file);
    m_file = nullptr;
}


std::vector<float> Decoder::decodeDat() noexcept {
  uint8_t const SIZE = 54;
  float v[SIZE];
  int c = fread(v, sizeof(float), SIZE, m_file);
  (void) c;
  std::vector<float> vec(std::begin(v), std::end(v));
  return vec;
}

#include <fstream>
#include <cstdio>
#include <string>
#include <vector>

class Decoder{
  private:
    Decoder(const Decoder &) = delete;
    Decoder(Decoder &&)      = delete;
    Decoder &operator=(const Decoder &) = delete;
    Decoder &operator=(Decoder &&) = delete;

  public:
    Decoder(std::string const &) noexcept;
    ~Decoder();

  public:
    std::vector<float> decodeDat() noexcept;

  private:

  private:
    FILE *m_file{nullptr};
};
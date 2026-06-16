#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace radxa
{

  class I2cDriver
  {
  public:
    explicit I2cDriver(const std::string& device);
    ~I2cDriver();

    bool open();
    void close();

    bool write(uint8_t address_7bit,
	       const uint8_t* data,
	       std::size_t length);

    bool read(uint8_t address_7bit,
	      uint8_t* data,
	      std::size_t length);

    bool writeRead(uint8_t address_7bit,
		   const uint8_t* write_data,
		   std::size_t write_length,
		   uint8_t* read_data,
		   std::size_t read_length);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

}

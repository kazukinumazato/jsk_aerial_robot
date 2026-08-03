#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace radxa
{

  class I2cInterface
  {
  public:
    virtual ~I2cInterface() = default;
    virtual bool write(uint8_t address_7bit, const uint8_t* data,
                       std::size_t length) = 0;
    virtual bool read(uint8_t address_7bit, uint8_t* data,
                      std::size_t length) = 0;
    virtual bool writeRead(uint8_t address_7bit,
                           const uint8_t* write_data,
                           std::size_t write_length,
                           uint8_t* read_data,
                           std::size_t read_length) = 0;
  };

  class I2cDriver : public I2cInterface
  {
  public:
    explicit I2cDriver(const std::string& device);
    ~I2cDriver();

    bool open();
    void close();

    bool write(uint8_t address_7bit,
	       const uint8_t* data,
	       std::size_t length) override;

    bool read(uint8_t address_7bit,
	      uint8_t* data,
	      std::size_t length) override;

    bool writeRead(uint8_t address_7bit,
		   const uint8_t* write_data,
		   std::size_t write_length,
		   uint8_t* read_data,
		   std::size_t read_length) override;

    bool isOpen() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

}

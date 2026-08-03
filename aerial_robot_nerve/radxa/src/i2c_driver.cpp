#include "radxa/i2c_driver.h"
#include <cerrno>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <limits>
#include <cstring>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace radxa
{
  struct I2cDriver::Impl
  {
    std::string device;
    int fd;

    Impl(const std::string& dev)
      : device(dev),  fd(-1)
    {
    }
  };

  I2cDriver::I2cDriver(const std::string& device)
    : impl_(std::make_unique<Impl>(device))
  {
  }

  I2cDriver::~I2cDriver()
  {
    close();
  }

  bool I2cDriver::open()
  {
    if (impl_->fd >= 0) {
      return true;
    }

    impl_->fd = ::open(impl_->device.c_str(), O_RDWR);
    
    if (impl_->fd < 0) {
      std::cerr << "device open failed: " << impl_->device << ": "
                << std::strerror(errno) << std::endl;
      return false;
    }
    std::cout << "device opened: " << impl_->device << std::endl;

    return true;
  }

  bool I2cDriver::isOpen() const
  {
    return impl_->fd >= 0;
  }

  void I2cDriver::close()
  {
    if (impl_->fd < 0){
      return;
    }
    if (::close(impl_->fd) < 0) {
      std::cerr << "device close failed: " << impl_->device << std::endl;
    }
    impl_->fd = -1;
  }

  static bool setAddress(int fd, uint8_t address_7bit)
  {
    if (::ioctl(fd, I2C_SLAVE, address_7bit) < 0)
      {
	std::cerr << "fail to set i2c slave address 0x" << std::hex
		  << static_cast<int>(address_7bit) << std::dec << std::endl;
	return false;
      }

    return true;
  }  

  bool I2cDriver::write(uint8_t address_7bit, const uint8_t* data, std::size_t length)
  {
    if ((length > 0 && data == nullptr) ||
        length > static_cast<std::size_t>(std::numeric_limits<ssize_t>::max())) {
      return false;
    }
    if(impl_->fd < 0){
      std::cerr << "i2c device is not open" << std::endl;
      return false;
    }
    if(!setAddress(impl_->fd, address_7bit)) return false;
    ssize_t written_size = ::write(impl_->fd, data, length);
    if(written_size < 0){
      std::cerr << "fail to write to 0x" << std::hex << static_cast<int>(address_7bit) << std::dec << std::endl;
      return false;
    }
    if(static_cast<std::size_t>(written_size) != length){
      std::cout << "partial write to 0x" << std::hex << static_cast<int>(address_7bit) << std::dec << std::endl;
      return false;
    }

    return true;
  }

  bool I2cDriver::read(uint8_t address_7bit, uint8_t* data, std::size_t length)
  {
    if ((length > 0 && data == nullptr) ||
        length > static_cast<std::size_t>(std::numeric_limits<ssize_t>::max())) {
      return false;
    }
    if(impl_->fd < 0){
      std::cerr << "i2c device is not open" << std::endl;
      return false;
    }    
    if(!setAddress(impl_->fd, address_7bit)) return false;
    ssize_t read_size = ::read(impl_->fd, data, length);
    if(read_size < 0){
      std::cerr << "fail to read from 0x" << std::hex << static_cast<int>(address_7bit) << std::dec << std::endl;
      return false;
    }
    if(static_cast<std::size_t>(read_size) != length){
      std::cout << "partial read from 0x" << std::hex << static_cast<int>(address_7bit) << std::dec << std::endl;
      return false;
    }

    return true;
  }

  bool I2cDriver::writeRead(uint8_t address_7bit, const uint8_t* write_data, std::size_t write_length, uint8_t* read_data, std::size_t read_length){
    if ((write_length > 0 && write_data == nullptr) ||
        (read_length > 0 && read_data == nullptr) ||
        write_length > std::numeric_limits<__u16>::max() ||
        read_length > std::numeric_limits<__u16>::max()) {
      return false;
    }
    if(impl_->fd < 0){
      std::cerr << "i2c device is not open" << std::endl;
      return false;
    }
    struct i2c_msg messages[] = {
				 {address_7bit, 0, static_cast<__u16>(write_length), const_cast<uint8_t*>(write_data) },
				 {address_7bit, I2C_M_RD, static_cast<__u16>(read_length), read_data},
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 }; 
    if(::ioctl(impl_->fd, I2C_RDWR, &ioctl_data) < 0){
      std::cerr << "fail to rdwr to 0x" << std::hex << static_cast<int>(address_7bit)
	       << std::dec << std::endl;
      return false;
    }

    return true;
  }

 
    
}

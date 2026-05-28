#include "tmc5130.hpp"

#include <array>
#include <cassert>
#include <cstdint>

namespace {

class MockSpi : public tmc5130::SpiDevice {
public:
    std::array<std::uint8_t, 5> last_write{};
    std::array<std::uint8_t, 5> transfer_response{};
    std::size_t write_count{0};
    std::size_t transfer_count{0};

    void write(const std::array<std::uint8_t, 5>& data) override {
        last_write = data;
        ++write_count;
    }

    void transfer(const std::array<std::uint8_t, 5>&, std::array<std::uint8_t, 5>& read_data) override {
        read_data = transfer_response;
        ++transfer_count;
    }
};

void testSetCurrentPacksExpectedRegister() {
    MockSpi spi;
    tmc5130::Driver driver(spi);

    driver.setCurrent(200, 50);

    assert(spi.write_count == 1);
    assert((spi.last_write == std::array<std::uint8_t, 5>{0x90, 0x00, 0x03, 0x03, 0x00}));
}

void testReadVersionUsesTransferResponse() {
    MockSpi spi;
    spi.transfer_response = {0xAA, 0x12, 0x34, 0x56, 0x78};
    tmc5130::Driver driver(spi);

    assert(driver.readVersion() == 0x12);
    assert(spi.transfer_count == 2);
}

void testSetCurrentPositionUpdatesActualAndTarget() {
    MockSpi spi;
    tmc5130::Driver driver(spi);

    driver.setCurrentPosition(1.0);

    assert(spi.write_count == 2);
    assert((spi.last_write == std::array<std::uint8_t, 5>{0xAD, 0x00, 0x00, 0xC8, 0x00}));
}

}  // namespace

int main() {
    testSetCurrentPacksExpectedRegister();
    testReadVersionUsesTransferResponse();
    testSetCurrentPositionUpdatesActualAndTarget();
    return 0;
}

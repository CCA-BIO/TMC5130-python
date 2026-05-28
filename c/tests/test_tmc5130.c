#include "tmc5130.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint8_t last_write[5];
    uint8_t transfer_response[5];
    size_t write_count;
    size_t transfer_count;
} mock_spi_t;

static int mock_write(void *context, const uint8_t *data, size_t length) {
    mock_spi_t *mock = (mock_spi_t *)context;
    assert(length == 5u);
    memcpy(mock->last_write, data, length);
    mock->write_count += 1u;
    return 0;
}

static int mock_transfer(void *context, const uint8_t *write_data, uint8_t *read_data, size_t length) {
    mock_spi_t *mock = (mock_spi_t *)context;
    (void)write_data;
    assert(length == 5u);
    memcpy(read_data, mock->transfer_response, length);
    mock->transfer_count += 1u;
    return 0;
}

static tmc5130_t make_driver(mock_spi_t *mock) {
    tmc5130_config_t config;
    tmc5130_t driver;
    tmc5130_spi_t spi = {
        .context = mock,
        .write = mock_write,
        .transfer = mock_transfer,
    };

    tmc5130_default_config(&config);
    assert(tmc5130_init_driver(&driver, spi, &config) == TMC5130_OK);
    return driver;
}

static void test_set_current_packs_expected_register(void) {
    mock_spi_t mock = {0};
    tmc5130_t driver = make_driver(&mock);
    const uint8_t expected[] = {0x90u, 0x00u, 0x03u, 0x03u, 0x00u};

    assert(tmc5130_set_current(&driver, 200u, 50u) == TMC5130_OK);
    assert(mock.write_count == 1u);
    assert(memcmp(mock.last_write, expected, sizeof(expected)) == 0);
}

static void test_read_version_uses_transfer_response(void) {
    mock_spi_t mock = {
        .transfer_response = {0xAAu, 0x12u, 0x34u, 0x56u, 0x78u},
    };
    tmc5130_t driver = make_driver(&mock);
    uint8_t version = 0u;

    assert(tmc5130_read_version(&driver, &version) == TMC5130_OK);
    assert(version == 0x12u);
    assert(mock.transfer_count == 2u);
}

static void test_set_current_position_updates_actual_and_target(void) {
    mock_spi_t mock = {0};
    tmc5130_t driver = make_driver(&mock);
    const uint8_t expected[] = {0xADu, 0x00u, 0x00u, 0xC8u, 0x00u};

    assert(tmc5130_set_current_position(&driver, 1.0) == TMC5130_OK);
    assert(mock.write_count == 2u);
    assert(memcmp(mock.last_write, expected, sizeof(expected)) == 0);
}

int main(void) {
    test_set_current_packs_expected_register();
    test_read_version_uses_transfer_response();
    test_set_current_position_updates_actual_and_target();
    return 0;
}

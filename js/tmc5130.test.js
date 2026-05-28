const test = require('node:test');
const assert = require('node:assert/strict');

const { TMC5130 } = require('./tmc5130');

class MockSpi {
  constructor() {
    this.writes = [];
    this.transferResponse = Buffer.alloc(5);
    this.transferCount = 0;
  }

  async write(data) {
    this.writes.push(Buffer.from(data));
  }

  async transfer() {
    this.transferCount += 1;
    return Buffer.from(this.transferResponse);
  }
}

test('setCurrent packs the expected current register', async () => {
  const spi = new MockSpi();
  const driver = new TMC5130(spi);

  await driver.setCurrent(200, 50);

  assert.equal(spi.writes.length, 1);
  assert.deepEqual([...spi.writes[0]], [0x90, 0x00, 0x03, 0x03, 0x00]);
});

test('readVersion uses the transfer response version byte', async () => {
  const spi = new MockSpi();
  spi.transferResponse = Buffer.from([0xAA, 0x12, 0x34, 0x56, 0x78]);
  const driver = new TMC5130(spi);

  await assert.doesNotReject(async () => {
    assert.equal(await driver.readVersion(), 0x12);
  });
  assert.equal(spi.transferCount, 2);
});

test('setCurrentPosition updates actual and target positions', async () => {
  const spi = new MockSpi();
  const driver = new TMC5130(spi);

  await driver.setCurrentPosition(1);

  assert.equal(spi.writes.length, 2);
  assert.deepEqual([...spi.writes[1]], [0xAD, 0x00, 0x00, 0xC8, 0x00]);
});

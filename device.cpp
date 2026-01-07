// MIT License
//
// Copyright (c) 2022 Joe Roback <joe.roback@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cstdio>
#include "device.hpp"

namespace racechrono
{

device& device::get() noexcept
{
    static device instance;
    return instance;
}

bool device::start() noexcept
{
    String device_name = "RC_DIY_" + String((uint16_t)((uint64_t)ESP.getEfuseMac() >> 32));

    BLEDevice::init(device_name);
    BLEDevice::setPower(ESP_PWR_LVL_P12);

    _server = BLEDevice::createServer();
    _server->setCallbacks(this);

    _service = _server->createService(racechrono_service_uuid);
    _pid_requests = _service->createCharacteristic(pid_characteristic_uuid, BLECharacteristic::PROPERTY_WRITE);

    _canbus_frames = _service->createCharacteristic(can_bus_characteristic_uuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    _canbus_frames->addDescriptor(&_2902_desc);
    _service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(_service->getUUID());
    advertising->setScanResponse(false);
    BLEDevice::startAdvertising();

    return true;
}

void device::onConnect(BLEServer*)
{
    _client_connected = true;
    Serial.println("Client connected");
}

void device::onDisconnect(BLEServer*)
{
    // once connection is made, BLE stops advertising, so on disconnect, start advertising again..

    _client_connected = false;
    BLEDevice::startAdvertising();
    Serial.println("Client disconnected");
}

} // namespace racechrono

racechrono::device& RCDEV = racechrono::device::get();

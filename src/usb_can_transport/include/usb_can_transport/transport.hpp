#pragma once

#include <libusb-1.0/libusb.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>

#define USB_CAN_OK 0
#define USB_CAN_LIBUSB_FAIL -1
#define USB_CAN_OPEN_FAIL -2
#define USB_CAN_INVALID_LENGTH -3
#define USB_CAN_INVALID_ID -4
#define USB_CAN_NULL_PTR -5

namespace usb_can_transport {

// Each instance owns its parser state. A zero-length or malformed frame cannot
// enter the payload-writing state. Input may be fragmented between USB reads.
class Decoder {
public:
    using Callback = std::function<void(uint16_t, uint8_t *, int)>;
    void feed(const uint8_t *bytes, int size, const Callback &callback);
private:
    unsigned int header_count_ = 0;
    int state_ = 0;
    int length_ = 0;
    int position_ = 0;
    uint16_t id_ = 0;
    std::array<uint8_t, 8> data_{};
};

class UsbCan {
public:
    using Callback = Decoder::Callback;
    UsbCan();
    ~UsbCan();
    UsbCan(const UsbCan &) = delete;
    UsbCan &operator=(const UsbCan &) = delete;
    int transmit(uint16_t id, const uint8_t *data, int length);
    int listen(uint16_t id, Callback callback);
    bool ready() const { return ready_.load(); }
    // Idempotent: join receiver before destroying callback owners or resources.
    void close();
private:
    void receiveLoop();
    libusb_context *context_ = nullptr;
    libusb_device_handle *handle_ = nullptr;
    std::array<bool, 2> claimed_{{false, false}};
    std::array<bool, 2> detached_{{false, false}};
    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    std::thread receiver_;
    std::mutex tx_mutex_;
    std::mutex callback_mutex_;
    std::unordered_map<uint16_t, Callback> callbacks_;
    Decoder decoder_;
};
}  // namespace usb_can_transport

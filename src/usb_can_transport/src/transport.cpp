#include "usb_can_transport/transport.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>
#include <utility>

namespace usb_can_transport {
void Decoder::feed(const uint8_t *bytes, int size, const Callback &callback)
{
    for (int i = 0; i < size; ++i) {
        const uint8_t byte = bytes[i];
        if (state_ == 0) {
            if (byte == 0xFF) {
                header_count_ = std::min(header_count_ + 1, 9U);
            } else {
                const int length = byte >> 4;
                if (header_count_ >= 9 && length >= 0 && length <= 8 && byte >= 0x0F) {
                    length_ = length;
                    position_ = 0;
                    state_ = 1;
                }
                header_count_ = 0;
            }
        } else if (state_ == 1) {
            if (byte <= 0x07) {
                id_ = static_cast<uint16_t>(byte) << 8;
                state_ = 2;
            } else {
                state_ = 0;
                header_count_ = byte == 0xFF ? 1 : 0;
            }
        } else if (state_ == 2) {
            id_ |= byte;
            if (length_ == 0) {
                state_ = 0;
                callback(id_, data_.data(), 0);
            } else {
                state_ = 3;
            }
        } else {
            data_[position_++] = byte;
            if (position_ == length_) {
                state_ = 0;
                callback(id_, data_.data(), length_);
            }
        }
    }
}

UsbCan::UsbCan()
{
    int ret = libusb_init(&context_);
    if (ret != LIBUSB_SUCCESS) {
        context_ = nullptr;
        return;
    }
    handle_ = libusb_open_device_with_vid_pid(context_, 0x1a86, 0x55d3);
    if (!handle_) return;
    for (int i = 0; i < 2; ++i) {
        const int active = libusb_kernel_driver_active(handle_, i);
        if (active == 1) {
            ret = libusb_detach_kernel_driver(handle_, i);
            if (ret != LIBUSB_SUCCESS) return;
            detached_[i] = true;
        } else if (active < 0 && active != LIBUSB_ERROR_NOT_SUPPORTED) {
            return;
        }
        ret = libusb_claim_interface(handle_, i);
        if (ret != LIBUSB_SUCCESS) return;
        claimed_[i] = true;
    }
    // CDC line coding is exactly seven bytes, little endian (4,000,000 baud).
    // Do not send an ABI-dependent padded C++ struct to the USB device.
    unsigned char line_coding[7] = {0x00, 0x09, 0x3D, 0x00, 0, 0, 8};
    ret = libusb_control_transfer(handle_, 0x21, 0x20, 0, 0,
                                  line_coding, sizeof(line_coding), 1000);
    if (ret != static_cast<int>(sizeof(line_coding))) return;
    ready_ = true;
    running_ = true;
    try {
        receiver_ = std::thread(&UsbCan::receiveLoop, this);
    } catch (...) {
        close();
        throw;
    }
}

UsbCan::~UsbCan() { close(); }

void UsbCan::close()
{
    running_ = false;
    ready_ = false;
    // RX synchronous reads have a finite timeout; no callback can resubmit.
    if (receiver_.joinable()) receiver_.join();
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (handle_) {
        for (int i = 1; i >= 0; --i) {
            if (claimed_[i]) libusb_release_interface(handle_, i);
            if (detached_[i]) libusb_attach_kernel_driver(handle_, i);
            claimed_[i] = detached_[i] = false;
        }
        libusb_close(handle_);
        handle_ = nullptr;
    }
    if (context_) {
        libusb_exit(context_);
        context_ = nullptr;
    }
}

int UsbCan::transmit(uint16_t id, const uint8_t *data, int length)
{
    if (length < 0 || length > 8) return USB_CAN_INVALID_LENGTH;
    if (id > 0x7FF) return USB_CAN_INVALID_ID;
    if (length > 0 && !data) return USB_CAN_NULL_PTR;
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (!ready_ || !handle_) return LIBUSB_ERROR_NO_DEVICE;
    std::array<unsigned char, 10> buffer{};
    buffer[0] = id >> 8;
    buffer[1] = id & 0xFF;
    if (length) std::memcpy(buffer.data() + 2, data, length);
    int actual_length = 0;
    // This call completes before the local buffer is destroyed. The same
    // thread locks and unlocks; USB stalls cannot block indefinitely.
    const int ret = libusb_bulk_transfer(handle_, 0x02, buffer.data(), length + 2,
                                         &actual_length, 100);
    if (ret == LIBUSB_ERROR_NO_DEVICE) ready_ = false;
    if (ret != LIBUSB_SUCCESS) return ret;
    return actual_length == length + 2 ? USB_CAN_OK : LIBUSB_ERROR_IO;
}

int UsbCan::listen(uint16_t id, Callback callback)
{
    if (id > 0x7FF) return USB_CAN_INVALID_ID;
    if (!callback) return USB_CAN_NULL_PTR;
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_[id] = std::move(callback);
    return USB_CAN_OK;
}

void UsbCan::receiveLoop()
{
    std::array<unsigned char, 64> buffer{};
    while (running_) {
        int received = 0;
        const int ret = libusb_bulk_transfer(handle_, 0x82, buffer.data(),
                                             buffer.size(), &received, 100);
        if (!running_) break;
        if (received > 0 && (ret == LIBUSB_SUCCESS || ret == LIBUSB_ERROR_TIMEOUT)) {
            decoder_.feed(buffer.data(), received, [this](uint16_t id, uint8_t *data, int length) {
                Callback callback;
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    const auto it = callbacks_.find(id);
                    if (it != callbacks_.end()) callback = it->second;
                }
                if (callback) {
                    try {
                        callback(id, data, length);
                    } catch (const std::exception &error) {
                        std::cerr << "USB-CAN callback failed: " << error.what() << std::endl;
                        ready_ = false;
                        running_ = false;
                    } catch (...) {
                        ready_ = false;
                        running_ = false;
                    }
                }
            });
        }
        if (ret == LIBUSB_ERROR_NO_DEVICE) {
            ready_ = false;
            running_ = false;
        } else if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_TIMEOUT &&
                   ret != LIBUSB_ERROR_INTERRUPTED) {
            // A persistent endpoint error must not busy-loop or report ready.
            ready_ = false;
            running_ = false;
        }
    }
}
}  // namespace usb_can_transport

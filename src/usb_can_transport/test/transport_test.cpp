#include "usb_can_transport/transport.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace usb_can_transport;
namespace fake {
std::atomic<int> fail_stage{0}, tx_error{0}, tx_count{0}, opens{0}, exits{0}, releases{0};
std::atomic<bool> unplugged{false};
std::mutex mutex;
std::deque<std::vector<uint8_t>> rx;
std::vector<std::vector<uint8_t>> tx;
void reset(int stage=0) {
    fail_stage=stage; tx_error=0; tx_count=0; opens=0; exits=0; releases=0; unplugged=false;
    std::lock_guard<std::mutex> lock(mutex); rx.clear(); tx.clear();
}
}
extern "C" {
int libusb_init(libusb_context **c) { if(fake::fail_stage==1)return LIBUSB_ERROR_OTHER; *c=reinterpret_cast<libusb_context*>(1);return 0; }
void libusb_exit(libusb_context*) { ++fake::exits; }
libusb_device_handle *libusb_open_device_with_vid_pid(libusb_context*,uint16_t,uint16_t) { return fake::fail_stage==2 ? nullptr : reinterpret_cast<libusb_device_handle*>(1); }
void libusb_close(libusb_device_handle*) { ++fake::opens; }
int libusb_kernel_driver_active(libusb_device_handle*,int) {return 0;}
int libusb_detach_kernel_driver(libusb_device_handle*,int) {return 0;}
int libusb_attach_kernel_driver(libusb_device_handle*,int) {return 0;}
int libusb_claim_interface(libusb_device_handle*,int i) {return fake::fail_stage==3+i ? LIBUSB_ERROR_BUSY : 0;}
int libusb_release_interface(libusb_device_handle*,int) {++fake::releases;return 0;}
int libusb_control_transfer(libusb_device_handle*,uint8_t,uint8_t,uint16_t,uint16_t,unsigned char *b,uint16_t len,unsigned int) {
    if(len!=7 || b[0]!=0 || b[1]!=9 || b[2]!=0x3D || b[6]!=8)return LIBUSB_ERROR_INVALID_PARAM;
    return fake::fail_stage==5 ? 6 : len;
}
int libusb_bulk_transfer(libusb_device_handle*,unsigned char endpoint,unsigned char *buffer,int length,int *actual,unsigned int timeout) {
    if(timeout==0)throw std::runtime_error("unbounded USB timeout");
    *actual=0;
    if(fake::unplugged)return LIBUSB_ERROR_NO_DEVICE;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if(endpoint==2) {
        int err=fake::tx_error.exchange(0);if(err)return err;
        std::lock_guard<std::mutex> lock(fake::mutex);
        fake::tx.emplace_back(buffer,buffer+length);++fake::tx_count;*actual=length;return 0;
    }
    std::lock_guard<std::mutex> lock(fake::mutex);
    if(fake::rx.empty())return LIBUSB_ERROR_TIMEOUT;
    auto b=fake::rx.front();fake::rx.pop_front();
    *actual=std::min(length,static_cast<int>(b.size()));std::copy_n(b.begin(),*actual,buffer);return 0;
}
}
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
std::vector<uint8_t> frame(uint16_t id,std::vector<uint8_t> payload) {
    std::vector<uint8_t> b(9,0xFF);b.push_back((payload.size()<<4)|0x0F);b.push_back(id>>8);b.push_back(id&255);b.insert(b.end(),payload.begin(),payload.end());return b;
}
#ifndef USB_CAN_FAKE_ONLY
int main() {
    // All partial construction paths can be destroyed and closed twice safely.
    for(int stage=1;stage<=5;++stage) {
        fake::reset(stage);
        { UsbCan can;require(!can.ready(),"failed init reported ready");can.close();can.close(); }
        require(fake::exits==(stage==1 ? 0 : 1),"context leaked/double freed");
        require(fake::releases==(stage<=3 ? 0 : stage==4 ? 1 : 2),"partial interface cleanup incorrect");
    }
    Decoder decoder;std::vector<std::vector<uint8_t>> decoded;
    auto callback=[&](uint16_t id,uint8_t *p,int n){require(id==0x153,"wrong frame id");decoded.emplace_back(p,p+n);};
    auto valid=frame(0x153,{1,2,3,4,5,6,7,8});
    for(auto b:valid)decoder.feed(&b,1,callback);
    auto zero=frame(0x153,{});decoder.feed(zero.data(),zero.size(),callback);
    auto malformed=std::vector<uint8_t>(9,0xFF);malformed.push_back(0x9F);
    decoder.feed(malformed.data(),malformed.size(),callback);
    decoder.feed(valid.data(),valid.size(),callback);
    require(decoded.size()==3 && decoded[1].empty() && decoded[0]==decoded[2],"fragment/zero/malformed decoder failed");
    fake::reset();
    {
        UsbCan can;require(can.ready(),"healthy init failed");
        require(can.transmit(0,nullptr,-1)==USB_CAN_INVALID_LENGTH,"negative length accepted");
        require(can.transmit(0,nullptr,8)==USB_CAN_NULL_PTR,"null payload accepted");
        std::array<uint8_t,8> data{{1,2,3,4,5,6,7,8}};
        fake::tx_error=LIBUSB_ERROR_TIMEOUT;
        require(can.transmit(0x120,data.data(),8)==LIBUSB_ERROR_TIMEOUT,"TX failure hidden");
        require(can.transmit(0x120,data.data(),8)==0,"TX failed to recover");
        std::vector<std::thread> workers;
        for(int k=0;k<4;++k) workers.emplace_back([&can,k](){
            for(int i=0;i<100;++i){std::array<uint8_t,8> bytes{};bytes.fill(k+10);require(can.transmit(0x120,bytes.data(),8)==0,"concurrent send failed");}
        });
        for(auto &thread:workers)thread.join();
        require(fake::tx_count==401,"TX frame count mismatch");
        {std::lock_guard<std::mutex> lock(fake::mutex);
         for(std::size_t i=1;i<fake::tx.size();++i) {
             auto &b=fake::tx[i];require(b.size()==10 && b[0]==1 && b[1]==0x20,"wire header corruption");
             for(int j=2;j<10;++j)require(b[j]==b[2],"payload lifetime/interleaving corruption");
         }}
        std::atomic<int> received{0};
        can.listen(0x153,[&](uint16_t,uint8_t*,int n){if(n==8)++received;});
        {std::lock_guard<std::mutex> lock(fake::mutex);fake::rx.push_back(valid);}
        for(int i=0;i<100 && !received;++i)std::this_thread::sleep_for(std::chrono::milliseconds(2));
        require(received==1,"receiver callback not delivered");
        fake::unplugged=true;
        for(int i=0;i<100 && can.ready();++i)std::this_thread::sleep_for(std::chrono::milliseconds(2));
        require(!can.ready(),"unplugged device still ready");
        require(can.transmit(0x120,data.data(),8)==LIBUSB_ERROR_NO_DEVICE,"unplugged TX accepted");
        const auto start=std::chrono::steady_clock::now();can.close();
        require(std::chrono::steady_clock::now()-start<std::chrono::milliseconds(300),"shutdown blocked");
    }
    std::cout << "PASS partial init, decoding, timeout recovery, 400 concurrent frames, RX, unplug, shutdown\n";
}

#endif

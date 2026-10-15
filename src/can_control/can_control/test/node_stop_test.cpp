#define USB_CAN_FAKE_ONLY
#include "../../../usb_can_transport/test/transport_test.cpp"
#define main can_control_application_main
#include "../src/can_control.cc"
#undef main

int main(int argc,char **argv) {
    rclcpp::init(argc,argv);
    fake::reset();
    auto controller=std::make_shared<OmniWheelController>();
    auto driver=std::make_shared<rclcpp::Node>("wheel_stop_test");
    auto publisher=driver->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    auto stop=driver->create_publisher<std_msgs::msg::Bool>("emergency_stop",10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller);executor.add_node(driver);
    const auto spin=[&](double seconds) {
        auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);
        while(std::chrono::steady_clock::now()<end) {
            executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    };
    const auto lastIsZero=[]() {
        std::lock_guard<std::mutex> lock(fake::mutex);
        if(fake::tx.empty())throw std::runtime_error("No wheel CAN frames");
        const auto &b=fake::tx.back();require(b.size()==10 && b[0]==2 && b[1]==0,"wrong motor wire format");
        return std::all_of(b.begin()+2,b.end(),[](uint8_t value){return value==0;});
    };
    const auto moving=[&]() {
        geometry_msgs::msg::Twist msg;msg.linear.x=.2;
        for(int i=0;i<4;++i){publisher->publish(msg);spin(.03);}
    };
    spin(.3);require(lastIsZero(),"startup did not send zero");
    moving();require(!lastIsZero(),"valid wheel command rejected");
    spin(.35);require(lastIsZero(),"timeout retained wheel/PID command");
    moving();publisher->publish(geometry_msgs::msg::Twist());spin(.08);
    require(lastIsZero(),"zero command retained PID kick");
    moving();std_msgs::msg::Bool halted;halted.data=true;stop->publish(halted);spin(.05);
    moving();require(lastIsZero(),"command bypassed emergency stop");
    halted.data=false;stop->publish(halted);spin(.08);
    require(lastIsZero(),"stop release replayed cached command");
    moving();require(!lastIsZero(),"fresh command did not resume");
    geometry_msgs::msg::Twist invalid;invalid.linear.x=std::numeric_limits<double>::quiet_NaN();
    publisher->publish(invalid);spin(.08);require(lastIsZero(),"invalid command retained wheel motion");
    executor.remove_node(controller);controller.reset();
    require(lastIsZero(),"shutdown did not send final stop");
    executor.remove_node(driver);driver.reset();rclcpp::shutdown();
    std::cout << "PASS wheel controller startup, movement, timeout, zero PID reset, stop, release, fresh command, NaN, shutdown\n";
}

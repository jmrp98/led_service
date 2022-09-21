#include "rclcpp/rclcpp.hpp"
#include "led_service/srv/turn_led.hpp"
#include "serialib.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

serialib my_serial;
void controlLED(const std::shared_ptr<led_service::srv::TurnLED::Request> request, const std:: shared_ptr<led_service::srv::TurnLED:: Response> response);


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std:: shared_ptr<rclcpp:: Node> node = rclcpp:: Node :: make_shared("led_service");

    rclcpp:: Service<led_service::srv::TurnLED>::SharedPtr service = node->create_service<led_service::srv::TurnLED>("turn_led",&controlLED);

    char err = my_serial.openDevice("/dev/ttyUSB0", 115200);
    if(err !=1)
    {
        std::cout << "Error:" << err << "\n";
        return err;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to turn on/off LEDs :)");

    rclcpp::spin(node);
    my_serial.closeDevice();
    rclcpp::shutdown();
}

void controlLED(const std::shared_ptr<led_service::srv::TurnLED::Request> request, const std:: shared_ptr<led_service::srv::TurnLED:: Response> response)
{
    my_serial.flushReceiver();
    //char* out_cmd = std:: to_string(request->led_color) + std:: to_string((int)request->led_on)+std:: to_string("\n");
    uint8_t led_color;
    switch(request->led_color)
    {
        case 1:
        led_color = 0b0001;
        break;

        case 2:
        led_color = 0b0010;
        break;

        case 3:
        led_color = 0b0100;
        break;

        default:
        led_color = 0;
        break;
    }
    
    if(led_color != 0)
    {
        uint8_t out_cmd =  led_color | ((request->led_on) << 3);
        
        /*sprintf(out_cmd,"%c%d\n",led_color,(int)request->led_on);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED service out: %s",out_cmd); */
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LED service out: %d",out_cmd);
        my_serial.writeBytes(&out_cmd,1);
    }
    response->r_led_color = request->led_color;
    response->r_led_on = request->led_on;

}
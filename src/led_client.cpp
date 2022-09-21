#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "led_service/srv/turn_led.hpp"
#include <chrono>
#include <memory>
#include <unistd.h>
#include <array>
#include <thread>
#include <vector>
#include <mutex>


#define LED_NUMBER 3
#define BLUE_LED 1
#define YELLOW_LED 2
#define RED_LED 3

#define LED_SHORTEST_PERIOD_MS 100



struct  led_t
{
    std::string param_name;
    uint16_t frequency_ms;
    uint8_t led_color, led_status;

    led_t(){};
    led_t(std::string param_name, uint16_t frequency_ms, uint8_t led_color)
    {
        this->param_name = param_name;
        this->frequency_ms = frequency_ms;
        this->led_color = led_color;
        led_status = 0;
    }
    
};




class LEDClient : public rclcpp:: Node 
{
    public: 

    std::vector<led_t>  led_array_;
    //Create a request for the led_service service call
    LEDClient(const std::vector<led_t> & led_array) : Node("led_client_node")
    {
       
        led_client_ = this->create_client<led_service::srv::TurnLED>("turn_led");
        this->led_array_ = led_array;
        for(auto &led : led_array_)
        {
            this-> declare_parameter<uint16_t>(led.param_name, led.frequency_ms);
        }  

         //Wait for service to be available

        if(!led_client_ ->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(),"Unable to find led_service service");
            return;
        }

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&LEDClient::parametersCallback, this, std::placeholders::_1));
    }


    rcl_interfaces::msg::SetParametersResult parametersCallback( 
            const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";

        for(const auto &param : parameters)
        {
            if(param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER )
            {    
                result.reason = "Parameter is not an integer";
                RCLCPP_ERROR(this->get_logger(), "Parameter update failed");
                continue;
            }
            
            uint16_t param_value = param.as_int();
            if(param_value < LED_SHORTEST_PERIOD_MS && param_value != 0)
            {
                result.reason = "Blinking period shorter than ms: " + LED_SHORTEST_PERIOD_MS;
                RCLCPP_ERROR(this->get_logger(), "Parameter update failed");
                continue;
            }

            std::string param_name = param.get_name();

           for(auto &led : led_array_ )
           {
                if(led.param_name == param_name)
                {
                    led.frequency_ms = param.as_int();
                    result.successful = true;
                    result.reason = "Success setting parameter";
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameter updated successfully");
                    break;
                }
           }
        }

        return result;
    }

    void start(led_t* led)
    {
        while(true)
        {
            if(led->frequency_ms > 0 && led->frequency_ms > LED_SHORTEST_PERIOD_MS)
            {
            auto request = std::make_shared<led_service::srv::TurnLED::Request>();
            request->led_color = led->led_color;
            led->led_status= !(led->led_status);
            request->led_on = led->led_status;
            send_request(request);
            std::this_thread::sleep_for(std::chrono::milliseconds(led->frequency_ms));
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

    }

    void send_request(std::shared_ptr<led_service::srv::TurnLED::Request> request)
    {
       // RCLCPP_INFO(get_logger(),"Attempting to change LED frequency");

        auto future = led_client_ ->async_send_request(request);
        
        this->start_func_mutex_.lock();
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),future) != rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to receive led_service service response");
            return;
        }
        this->start_func_mutex_.unlock();

        auto response = future.get();
        if(response -> r_led_color != request->led_color)
        {
            RCLCPP_ERROR(this->get_logger(), "led_service service failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(),"Led_service request successful");
    }

    private:
    rclcpp::Client<led_service::srv::TurnLED>::SharedPtr led_client_;
    std::mutex start_func_mutex_ ;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

};





int main(int argc, char **argv)
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);
    
    
    led_t blue_led("blue_freq_ms",4000,BLUE_LED);
    led_t red_led("red_freq_ms",4000,RED_LED);
    led_t yellow_led("yellow_freq_ms",4000,YELLOW_LED);
    std::vector<led_t> led_array = {blue_led,red_led,yellow_led};

    
    auto led_client_node = std::make_shared<LEDClient>(led_array);
    std::vector<std::thread> thread_array;
    for(size_t i=0; i<led_array.size(); i++)
    {
        std::thread thread_ ((&LEDClient::start),led_client_node,&(led_client_node->led_array_[i]));
        thread_array.push_back(std::move(thread_));

    }

    for(auto &thread_t : thread_array)
    {
        if(thread_t.joinable())
        thread_t.join();
    }


    
    led_client_node->start(&blue_led);
    
    //rclcpp::spin(led_client_node);
    rclcpp::shutdown();
    
    return 0;
}
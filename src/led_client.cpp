#include "rclcpp/rclcpp.hpp"
#include "led_service/srv/turn_led.hpp"
#include <chrono>
#include <memory>
#include <unistd.h>
#include <array>

//using namespace std::chrono_literals;


class LEDClient : public rclcpp:: Node 
{
    public: 
    //Create a request for the led_service service call
    LEDClient() : Node("led_client_node")
    {
        this-> declare_parameter<uint16_t>("b_freq_ms",4000);
        this-> declare_parameter<uint16_t>("y_freq_ms",0);
        this->declare_parameter<uint16_t>("r_freq_ms",0);
        led_client_ = this->create_client<led_service::srv::TurnLED>("turn_led");

        /*blue_timer_ = this ->create_wall_timer(std::chrono::milliseconds(50),
            [this]()
            {
                timer_callback(&b_freq_ms_,"b_freq_ms",1,blue_timer_);
            }
            );*/



    }

    void start(void)
    {
        using namespace std::chrono;

        std::array<std::chrono::system_clock::time_point,3> led_timers;
        std::array<std::string,3>  params_names = {"b_freq_ms","y_freq_ms","r_freq_ms"};
        std::array<bool,3> led_status = {false,false,false};

        auto request = std::make_shared<led_service::srv::TurnLED::Request>();

        for(auto i : led_timers)
        {
            i = std::chrono::system_clock::now();
        }


        //uint32_t min_freq;
        while(true)
        {
            for(std::size_t i = 0 ; i<led_freq_param_.size();i++)
            {
                this->get_parameter(params_names[i],led_freq_param_[i]);
            }

            
            for(std::size_t i =0 ; i<led_timers.size();i++)
            {
                auto led_timer_ms = time_point_cast<milliseconds>(led_timers[i]).time_since_epoch().count();
                auto  now = system_clock::now() ;
                auto now_ms= time_point_cast<milliseconds>(now).time_since_epoch().count();
                auto delta_time = now_ms - led_timer_ms;

                if(led_freq_param_[i]>0 && delta_time >= led_freq_param_[i])
                {
                    //LED number
                    request->led_color = i+1; 
                    
                    led_status[i] = !(led_status[i]);
                    request->led_on =  !(led_status[i]);

                    send_request(request);
                    led_timers[i] = system_clock::now();    

                }
            }
        }

    }

    void send_request(std::shared_ptr<led_service::srv::TurnLED::Request> request)
    {
        RCLCPP_INFO(get_logger(),"Attempting to change LED frequency");

        //Wait for service to be available

        if(!led_client_ ->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(get_logger(),"Unable to find led_service service");
            return;
        }


        auto future = led_client_ ->async_send_request(request);
        
        
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),future) != rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(),"Failed to receive led_service service response");
            return;
        }

        auto response = future.get();
        if(response -> r_led_color != request->led_color)
        {
            RCLCPP_ERROR(this->get_logger(), "led_service service failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(),"Led_service service working");
    }

    private:
    rclcpp::Client<led_service::srv::TurnLED>::SharedPtr led_client_;
    rclcpp::TimerBase::SharedPtr blue_timer_;
    rclcpp::TimerBase::SharedPtr yellow_timer_;
    rclcpp::TimerBase::SharedPtr red_timer_;

    /*uint16_t b_freq_ms_ =0;
    uint16_t y_freq_ms_ =0;
    uint16_t r_freq_ms_=0;*/
    std::array<uint16_t,3> led_freq_param_ = {0,0,0};
    
    
    
    //Tried  implementation using timers
    void timer_callback(uint16_t* led_freq_param, std::string param_name, uint8_t led_color,rclcpp::TimerBase::SharedPtr led_timer)
    {
        led_timer ->cancel();
        //Check if the frequency for a given LED was
        uint16_t prev_freq  = *led_freq_param;
        this->get_parameter(param_name,*led_freq_param);
        

        auto request = std::make_shared<led_service::srv::TurnLED::Request>();
        request->led_color = led_color;
        request->led_on= !(request->led_on);
        
        send_request(request);
        
        if(prev_freq != *led_freq_param)
        {
            
            
            led_timer = this ->create_wall_timer(std::chrono::milliseconds(*led_freq_param),
            [&]()
            {
                timer_callback(led_freq_param,param_name,led_color,led_timer);
            }
            );

            return;
        }

        led_timer ->reset();

    }


    

};





int main(int argc, char **argv)
{
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);
    
    auto led_client_node = std::make_shared<LEDClient>();
    led_client_node->start();
    
    //rclcpp::spin(led_client_node);
    rclcpp::shutdown();
    
    return 0;
}
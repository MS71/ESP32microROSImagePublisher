#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

//#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/compressed_image.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#include "esp_camera.h"

#define BOARD_ESP32CAM_AITHINKER 1

// WROVER-KIT PIN Map
#ifdef BOARD_WROVER_KIT

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER

#define FLASH_GPIO 4

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

static const char *TAG = "example:take_picture";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    //.pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    //.frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 4, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

rcl_publisher_t publisher;
sensor_msgs__msg__CompressedImage msg_static;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
    		//gpio_set_level(FLASH_GPIO, 1);
        	camera_fb_t *pic = esp_camera_fb_get();
    		//gpio_set_level(FLASH_GPIO, 0);
		if( pic != NULL )
		{
		  if( pic->len <= msg_static.data.capacity )
		  {
			// use pic->buf to access the image
			msg_static.data.size = pic->len;
			memcpy(msg_static.data.data,pic->buf,pic->len);
			ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes %d %d %d %d %p %02x%02x%02x%02x%02x%02x%02x%02x", 
				pic->len,pic->width,pic->height,
				msg_static.data.size,msg_static.data.capacity,msg_static.data.data,
				msg_static.data.data[0],
				msg_static.data.data[1],
				msg_static.data.data[2],
				msg_static.data.data[3],
				msg_static.data.data[4],
				msg_static.data.data[5],
				msg_static.data.data[6],
				msg_static.data.data[7]);
			
			msg_static.header.frame_id = micro_ros_string_utilities_set(msg_static.header.frame_id, "myframe");
			msg_static.format = micro_ros_string_utilities_set(msg_static.format, "jpeg");
			
			RCSOFTCHECK(rcl_publish(&publisher, &msg_static, NULL));
		  }
		  else
		  {
			ESP_LOGE(TAG, "Picture taken! Its size was: %zu bytes %d %d", 
				pic->len,pic->width,pic->height);
		  }
		
		  esp_camera_fb_return(pic);
		}
		else
		{
			ESP_LOGE(TAG, "Picture not taken!");
		} 
	}
}

void micro_ros_task(void * arg)
{ 
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32cam", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
    	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
		"image/compressed"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 500;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	// INIT MESSAGE MEMORY

	// --- Configuration ---

	// micro-ROS utilities allows to configure the dynamic memory initialization using a micro_ros_utilities_memory_conf_t structure
	// If some member of this struct is set to zero, the library will use the default value.
	// Check `micro_ros_utilities_memory_conf_default` in `<micro_ros_utilities/type_utilities.h>` for those defaults.

	static micro_ros_utilities_memory_conf_t conf = {};

	// OPTIONALLY this struct can configure the default size of strings, basic sequences and composed sequences

	conf.max_string_capacity = 50;
	conf.max_ros2_type_sequence_capacity = 5;
	conf.max_basic_type_sequence_capacity = 5;

	// OPTIONALLY this struct can store rules for specific members
	// !! Using the API with rules will use dynamic memory allocations for handling strings !!

	micro_ros_utilities_memory_rule_t rules[] = {
		{"header.frame_id", 30},
		{"format",4},
		{"data", 15000}
	};
	conf.rules = rules;
	conf.n_rules = sizeof(rules) / sizeof(rules[0]);
	
	micro_ros_utilities_create_message_memory(
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
		&msg_static,
		conf
	);

	// spin executor
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

#if 1
    if(ESP_OK != init_camera()) {
        return;
    }
#endif

    //gpio_reset_pin(FLASH_GPIO);
    //gpio_set_direction(FLASH_GPIO, GPIO_MODE_OUTPUT);
    //gpio_set_level(FLASH_GPIO, 0);

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}


#include <stdio.h>
#include <algorithm>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include <FastLED.h>
#include "LEDStrip.h"
#include "mpack.h"

#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (2048)
#define RD_BUF_SIZE (BUF_SIZE)

#define LEFT_STRIP   19
#define MIDDLE_STRIP 21
#define RIGHT_STRIP  18

#define LEFT_STRIP_LEDS 85
#define MIDDLE_STRIP_LEDS 85
#define RIGHT_STRIP_LEDS 85

static const char* TAG = "main";

LEDStrip* strips[10];
int stripCount;
int ledSum;

void clearStrip(LEDStrip* strip, CRGB color = CRGB(0, 0, 0)) {
    const auto& leds = strip->getLeds();
    for (int i = 0; i < strip->getSize(); i++) {
        leds[i] = color;
    }
}

template<uint8_t DATA_PIN> void addStrip(int size) {
    const auto strip = new LEDStrip(size);
    strips[stripCount] = strip;
    stripCount++;
    ledSum += size;
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(strip->getLeds(), size);
    clearStrip(strip);
}

extern "C" {
  void app_main();
}

QueueHandle_t uart0_queue;

void init_serial()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
	//uart0_queue = xQueueCreate(5, sizeof(uint8_t)); //initialize the queue
   
	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
}

void update_led_strip(mpack_reader_t* reader) {
	ESP_LOGD(TAG, "Reading strip tag.");
	mpack_tag_t tag = mpack_read_tag(reader);
	if (mpack_tag_type(&tag) != mpack_type_uint) {
		ESP_LOGE(TAG, "Reading strip tag. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_uint);
		return;
	}

	uint64_t strip_index = mpack_tag_uint_value(&tag);
	if (strip_index >= stripCount) {
		ESP_LOGE(TAG, "Invalid strip index '%lld'.", strip_index);
		return;
	}

	ESP_LOGD(TAG, "Reading led updates array tag.");
	tag = mpack_read_tag(reader);
	if (mpack_tag_type(&tag) != mpack_type_array) {
		ESP_LOGE(TAG, "Reading led updates array tag. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_array);
		return;
	}

	const auto& strip = strips[strip_index];
	const auto& leds = strip->getLeds();

	uint32_t led_update_count = mpack_tag_array_count(&tag);
	ESP_LOGD(TAG, "LED updates count: %u", led_update_count);
	for (uint32_t i = 0; i < led_update_count; i++) {
		ESP_LOGD(TAG, "Reading next LED update (%u).", i);
		ESP_LOGD(TAG, "Validating LED Update array type.");
		tag = mpack_read_tag(reader);
		if (mpack_tag_type(&tag) != mpack_type_array) {
			ESP_LOGE(TAG, "Validating LED Update array type. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_array);
			return;
		}
		ESP_LOGD(TAG, "Reading LED index.");
		tag = mpack_read_tag(reader);
		if (mpack_tag_type(&tag) != mpack_type_uint) {
			ESP_LOGE(TAG, "Reading LED index. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_uint);
			return;
		}
		uint64_t led_index = mpack_tag_uint_value(&tag);

		ESP_LOGD(TAG, "Reading LED R component.");
		tag = mpack_read_tag(reader);
		if (mpack_tag_type(&tag) != mpack_type_uint) {
			ESP_LOGE(TAG, "LED R. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_uint);
			return;
		}
		uint64_t r = mpack_tag_uint_value(&tag);

		ESP_LOGD(TAG, "Reading LED G component.");
		tag = mpack_read_tag(reader);
		if (mpack_tag_type(&tag) != mpack_type_uint) {
			ESP_LOGE(TAG, "LED G. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_uint);
			return;
		}
		uint64_t g = mpack_tag_uint_value(&tag);

		ESP_LOGD(TAG, "Reading LED B component.");
		tag = mpack_read_tag(reader);
		if (mpack_tag_type(&tag) != mpack_type_uint) {
			ESP_LOGE(TAG, "LED B. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_uint);
			return;
		}
		uint64_t b = mpack_tag_uint_value(&tag);

		ESP_LOGD(TAG, "Updating led %llu on strip %llu with RGB values %llu %llu %llu.", led_index, strip_index, r, g, b);
		leds[led_index] = CRGB((uint8_t)r, (uint8_t)g, (uint8_t)b);
	}
	mpack_done_array(reader);
	FastLED.show();
}

void update_led_strips(char* message, size_t message_size) {
	mpack_reader_t reader;
	mpack_reader_init_data(&reader, message, message_size);
	ESP_LOGD(TAG, "Reading update data array tag.");
	mpack_tag_t tag = mpack_read_tag(&reader);
	if (mpack_tag_type(&tag) != mpack_type_array) {
		ESP_LOGE(TAG, "Reading update data array tag. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_array);
		return;
	}

	ESP_LOGD(TAG, "Reading strip updates array tag.");
	tag = mpack_read_tag(&reader);
	if (mpack_tag_type(&tag) != mpack_type_array) {
		ESP_LOGE(TAG, "Reading strip updates array tag. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_array);
		return;
	}

	uint32_t update_count = mpack_tag_array_count(&tag);
	for (uint32_t i = 0; i < update_count; i++) {
		ESP_LOGD(TAG, "Reading strip update object/array tag.");
		tag = mpack_read_tag(&reader);
		if (mpack_tag_type(&tag) != mpack_type_array) {
			ESP_LOGE(TAG, "Reading strip update object/array tag. Invalid tag type %i, expected %i.", mpack_tag_type(&tag), mpack_type_array);
			return;
		}
		ESP_LOGD(TAG, "Handling LED strip update.");
		update_led_strip(&reader);
		ESP_LOGD(TAG, "Handling LED strip update. Finished handling LED strip update.");
	}
	mpack_done_array(&reader);
	mpack_reader_destroy(&reader);
}

size_t pending_data_amount = 0;
char* current_message;
size_t current_message_pos;

void read_uart_message(size_t event_size) {
	while (event_size > 0) {
		if (pending_data_amount == 0) {
			ESP_LOGD(TAG, "Reading size of next message.");
			uint8_t* size_bytes_buffer = (uint8_t*) malloc(sizeof(uint16_t));
			uart_read_bytes(EX_UART_NUM, size_bytes_buffer, sizeof(uint16_t), portMAX_DELAY);
			pending_data_amount = (uint16_t)((size_bytes_buffer[0] << 8) + (size_bytes_buffer[1]));
			free(size_bytes_buffer);
			ESP_LOGD(TAG, "Length of next message: %u", pending_data_amount);
			ESP_LOGD(TAG, "Allocating space for next message.");
			current_message = (char*) malloc(sizeof(char) * pending_data_amount);
			current_message_pos = 0;
			event_size -= sizeof(uint16_t);
		}

		size_t bytes_to_read = std::min(event_size, pending_data_amount);
		ESP_LOGD(TAG, "Reading next %u bytes for current message, remaining: %u", bytes_to_read, pending_data_amount);
		uart_read_bytes(EX_UART_NUM, current_message + current_message_pos, bytes_to_read, portMAX_DELAY);
		current_message_pos += bytes_to_read;
		pending_data_amount -= bytes_to_read;
		event_size -= bytes_to_read;
		if (pending_data_amount == 0) {
			ESP_LOGD(TAG, "Message complete. Handling.");
			update_led_strips(current_message, current_message_pos);
			free(current_message);
		}
	}
}

void serial_read_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGD(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGD(TAG, "[UART DATA]: %d", event.size);
					size_t buffered_data_len;
					uart_get_buffered_data_len(EX_UART_NUM, &buffered_data_len);
					if (event.size != buffered_data_len) {
						ESP_LOGE(TAG, "Event Size (%u) and buffered data (%u) is not matching.", event.size, buffered_data_len);
					}
                    read_uart_message(buffered_data_len);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(TAG, "UART PATTERN DETECTED\n");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


void app_main() {
  	printf(" entering app main, call add leds\n");
  	// the WS2811 family uses the RMT driver
	addStrip<LEFT_STRIP>(LEFT_STRIP_LEDS);
	addStrip<MIDDLE_STRIP>(MIDDLE_STRIP_LEDS);
	addStrip<RIGHT_STRIP>(RIGHT_STRIP_LEDS);

	FastLED.setMaxPowerInVoltsAndMilliamps(5, 20000);

	FastLED.setCorrection(TypicalPixelString);
    FastLED.show();

	// change the task below to one of the functions above to try different patterns
	printf("Creating task for handling data.\n");
	//xTaskCreatePinnedToCore(&blinkLeds_simple, "blinkLeds", 4000, NULL, 5, NULL, 0);

	init_serial();
	//Reset the pattern queue length to record at most 20 pattern positions.
    //uart_pattern_queue_reset(EX_UART_NUM, 20);
    //Create a task to handler UART event from ISR
    xTaskCreate(serial_read_task, "serial_read_task", 2048, NULL, 12, NULL);
}
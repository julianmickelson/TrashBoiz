//_________________HEADERS______________
#include <samd21g18a.h>
#include <asf.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>

#include "main.h"
#include "stdio_serial.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"
#include "iot/mqtt/mqtt.h"
#include "iot/sw_timer.h"

//_________________DEFINES______________
#define STRING_EOL                      "\r\n"
#define STRING_HEADER                   "-- TrashBoiz --"STRING_EOL \
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
#define MAX_RX_BUFFER_LENGTH	100
#define BAUDRATE				115200
#define CR						'\r' // Carriage return
#define LF						'\n' // New Line
#define BS						'\b' // Backspace
#define NULLCHAR				'\0'
#define SPACE					' '
#define DATA_LENGTH				8
#define SLAVE_ADDRESS			0x40
#define MAX_ARGS				8
//#define CONF_PWM_MODULE      EXT1_PWM_MODULE //references TCC0
//#define CONF_PWM_CHANNEL     EXT1_PWM_0_CHANNEL
//#define CONF_PWM_OUTPUT      0						//check later
//#define CONF_PWM_OUT_PIN     PIN_PA10F_TCC0_WO2
//#define CONF_PWM_OUT_MUX     MUX_PA10F_TCC0_WO2
#define UART_BAUDRATE			115200
#define LED_0_PIN				PIN_PA22	//PIN_PA23 for explained
#define BUTTON_DEBUG			PIN_PA24	//PIN_PA24 for our board PIN_PB23 for explained
#define PIN_1					PIN_PA17
#define PIN_2					PIN_PA21
#define HTTP_METADATA			"https://www.seas.upenn.edu/~jmick/TrashBoiz_metadata.txt"

//_________________GLOBAL VARIABLES______________
bool meta_flag = true; //if true download metadata first
struct usart_module usart_instance;
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
char * original_rx_buffer;
volatile char singleInput;
char *arguments[MAX_ARGS];
volatile uint8_t charCounter = 0;
volatile bool processFlag;
volatile uint16_t nReadings;
bool usart_read_flag;
bool first_CRC;
uint16_t bufferIndex;
uint8_t bufferOverflow1;
uint8_t bufferOverflow2;
bool button_change = false;

//----------------METADATA-------------
uint32_t appcodeSize; // = 21716; //total size of appcode downloaded 
uint32_t crc_master;
uint8_t	hardware_major = 1;
uint8_t	hardware_minor = 0;
uint8_t hardware_patch = 0;
uint8_t software_major = 1;
uint8_t	software_minor = 0;
uint8_t	software_patch = 0;

bool downloadedImage = false; //true is image 1, false is image
bool write_image_1 = true;
static download_state down_state = NOT_READY; /** File download processing state. */
static uint32_t http_file_size = 0;	/** Http cotnent length. */
static uint32_t received_file_size = 0;	/** Receiving content length. */
static char save_file_name[MAIN_MAX_FILE_NAME_LENGTH + 1] = "0:";	/** File name to download. */

//----------------NVM-------------
#define NVM_PAGE_SIZE			64
#define	NVM_ROW_PAGES			4
#define	NVM_NUM_PAGES			4096
#define NVM_BOOTSTATUS			0x03F00
#define NVM_APPCODE				0x04000
#define NVM_END					0x40000

struct nvm_parameters nvm_parameters_current;
enum status_code error_code;

uint8_t NVM_page_buffer[NVM_PAGE_SIZE];
uint8_t NVM_read_buffer[NVM_PAGE_SIZE];
uint32_t flashWriteAddress = 0x1000; //change depending of which image

//----------------ADC-------------
#define PROX_BUFFER_SIZE  10

struct adc_module adc_prox_instance;
struct adc_config config_adc_prox;

volatile int distance;

uint16_t prox_buffer[PROX_BUFFER_SIZE];

//----------------TIMER-------------
#define DELAY				5		// in 100ms
#define PERIOD				75000	// 48Mhz/64/100ms

struct tcc_module tcc_instance;
struct tcc_config config_tcc;

volatile int timer_counter;
volatile bool prox_read_flag;

volatile int prox_read_count;


//----------------PWM-------------
struct tcc_module tcc_pwm_instance;
struct tcc_config config_pwm_tcc;

//----------------CRC-------------
crc32_t crc;

bool first_CRC = true;
uint8_t block1[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	
//----------------AT25DFX-------------
#define AT25DFX_IMAGE1_ADDR		0x1000
#define AT25DFX_IMAGE2_ADDR		0x40000
#define AT25DFX_BUFFER_SIZE		256
#define AT25DFX_STATUS			0x0000
static uint8_t read_buffer1[AT25DFX_BUFFER_SIZE];
static uint8_t read_buffer2[AT25DFX_BUFFER_SIZE];
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint32_t test_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_master_vec_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
struct usart_module usart_instance;

//----------------HTTP-------------
struct sw_timer_module swt_module_inst;	/** Instance of Timer module. */
struct http_client_module http_client_module_inst;	/** Instance of HTTP client module. */

//----------------WiFi-------------
tstrWifiInitParam param;
int8_t ret;

//----------------MQTT-------------
#define MAIN_MQTT_BUFFER_SIZE 128
#define	MAIN_MQTT_TOPIC "ese516/g2/#"
#define	MAIN_MQTT_ROOT "ese516/g2/"
#define MQTT_PROX "prox"
#define MQTT_LED "LED"
#define MQTT_ID "txjyotmk"
#define MQTT_PASSWORD "ztwAOvhgO9Vq"
#define MQTT_CLIENT_ID	"txjyotmk"

static const char main_mqtt_broker[] = "m14.cloudmqtt.com";

struct sw_timer_module swt_module_inst_mqtt;
static struct mqtt_module mqtt_inst;
static char mqtt_buffer[MAIN_MQTT_BUFFER_SIZE];

#define TRASH_PIC_ROOT			"https://www.seas.upenn.edu/~jmick/trash_"
#define FILE_EXTENSION			".jpg"
#define ALERT_NOTIFICATION		"Trash can ID: 001 is full. Located outside of Towne 123. For more information, visit https://goo.gl/DYqgUG"

void mqtt_update();
uint8_t trash_bracket = 2;
uint8_t trash_level = 69;

//_________________AVAILABLE FUNCTIONS______________
char *commands[] = {"help", "ver_bl", "ver_app", "gpio_set", "gpio_clear", "gpio_get", "mac", "ip", "prox"};
char *desc[] =
{
	"Prints the bootloader firmware version",
	"Prints the application code firmware version",
	"Set a GPIO pin to high / 1",
	"Set a GPIO pin to low / 0",
	"Get state of specified GPIO pin",
	"returns the mac address of the device",
	"returns the ip address of the device in the standard format: ex. 255.255.255.255",
	"Trigger a proximity sensor distance reading",
};

//_________________DECLARE FUNCTIONS______________
void prox_init();
void prox_read();
void set_pin(uint8_t PIN);
void check_pin(uint8_t PIN2);
void calc_distance(uint16_t number);
void interval_count();
void servo(int8_t degrees);
void usart_write_callback(struct usart_module *const usart_module);
static void tcc_interval_wait(struct tcc_module *const module_inst);
void usart_read_callback(struct usart_module *const usart_module);
void at25dfx_commands();
void crc_check(uint8_t *block, uint32_t length);
void erase_at25dfx(void);
void save_packet(char *data, uint32_t length);
void process_metadata(char *data, uint32_t length);
void gpio_set();
void gpio_clear();
void gpio_get();
void wifi_init();
void bubbleSort(uint16_t arr[], int n);
void average();
static void configure_tcc(void);
void prox_read_handler();

bool tcc_flag = false;
void 

_update();
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

//_________________PROCESS USER INPUT______________
bool processUserInput(void)
{
	switch (singleInput)
	{
		case CR:
		case LF:
		processFlag = true; //flag to start processing the command
		charCounter = 0; //reset counter
		for(int i = 0; i<strlen(rx_buffer); i++){
			rx_buffer[i] = tolower(rx_buffer[i]);
		}
		original_rx_buffer = malloc(sizeof(char) * strlen(rx_buffer));
		strcpy(original_rx_buffer,rx_buffer);
		printf("\r\n");
		break;
		
		case BS:
		charCounter--;
		rx_buffer[charCounter] = NULLCHAR;
		break;
		
		default:
		if (charCounter < MAX_RX_BUFFER_LENGTH)
		{
			rx_buffer[charCounter] = singleInput; //save single input to rx buffer
			rx_buffer[charCounter+1] = NULLCHAR; //Input nullchar in the next position
			charCounter++;
			usart_write_buffer_wait(&usart_instance, &singleInput, 1); //write to terminal
		}
		break;
	}
}

void processCommand()
{
	#define DELIMITER " "
	int16_t nargs = 0;
	char *midStringPtr;
	arguments[nargs++] = rx_buffer;
	midStringPtr = strpbrk(rx_buffer, DELIMITER);
	while (midStringPtr != NULL) {
		arguments[nargs] = midStringPtr+1;
		*midStringPtr = '\0';
		midStringPtr = strpbrk(arguments[nargs], DELIMITER);
		nargs++;
	}
	
	uint8_t indexCommand = 10;
	
	for (int i=0; i<9; i++)
	{
		if(0==strcmp(rx_buffer, commands[i]))
		{
			indexCommand=i;
			break;
		}
	}
	
	switch(indexCommand)
	{
		case 0:
		for (int i=1; i<8; i++)
		{
			printf(commands[i]);
			printf("\r\n");
			printf(desc[i-1]);
			printf("\r\n");
		}
		break;
		
		case 1:
		printf("Bootloader version 1.0.0\r\n");
		break;
		
		case 2:
		printf("Application firmware version 1.0.0\r\n");
		break;
		
		case 3:
		gpio_set();
		break;
		
		case 4:
		gpio_clear();
		break;
		case 5:
		gpio_get();
		break;
		
		case 6:
		printf("MAC: ff.ff.ff.ff.ff.ff\r\n");
		//
		//m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
		//
		//if (!u8IsMacAddrValid) {
		///* Cannot found MAC Address from OTP. Set user define MAC ADDRESS. */
		//m2m_wifi_set_mac_address((uint8_t *)main_user_define_mac_address);
		//}
		//
		///* Get MAC Address. */
		//m2m_wifi_get_mac_address(mac_addr);
		break;
		
		case 7:
		printf("IP: 255.255.255.255\r\n");
		break;
		
		case 8:
		prox_read_flag = true;
		printf("Taking proximity sensor readings");
		break;
		
		default:
		printf("ERROR, no such command available: %s\r\n", original_rx_buffer);
		//printf(" %s", original_rx_buffer);
		break;
	}
	processFlag = false;
}

void gpio_set()
{
	if ((strcmp(arguments[1], "b") && strcmp(arguments[2], "10")) == 0)
	{
		set_pin(0);
	}
	else if ((strcmp(arguments[1], "a") && strcmp(arguments[2], "19")) == 0)
	{
		set_pin(1);
	}
	else
	{
		printf("There is no such pin available. Try B 10, or A 19\r\n");
	}
}

void gpio_clear()
{
	if ((strcmp(arguments[1], "b") && strcmp(arguments[2], "10")) == 0)
	{
		set_pin(2);
	}
	else if ((strcmp(arguments[1], "a") && strcmp(arguments[2], "19")) == 0)
	{
		set_pin(3);
	}
	else
	{
		printf("There is no such pin available. Try B 10, or A 19\r\n");
	}
}

void gpio_get()
{
	if ((strcmp(arguments[1], "b") && strcmp(arguments[2], "10")) == 0)
	{
		check_pin(0);
	}
	else if ((strcmp(arguments[1], "a") && strcmp(arguments[2], "19")) == 0)
	{
		check_pin(1);
	}
	else
	{
		printf("There is no such pin available. Try B 10, or A 19\r\n");
	}
}

//_________________PIN OPERATIONS ______________

void set_pin(uint8_t PIN) //configure the port pin
{
	uint8_t pin_state;
	struct port_config configure_port_pin;
	port_get_config_defaults(&configure_port_pin);
	configure_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	
	if (PIN==0)
	{
		port_pin_set_config(PIN_2, &configure_port_pin);
		port_pin_set_output_level(PIN_2, true);
		printf("Pin PB10 is set high\r\n");
	}
	else if (PIN == 1)
	{
		port_pin_set_config(PIN_1, &configure_port_pin);
		port_pin_set_output_level(PIN_1, true);
		printf("Pin PA19 is set high\r\n");
	}
	else if (PIN == 2)
	{
		port_pin_set_config(PIN_2, &configure_port_pin);
		port_pin_set_output_level(PIN_2, false);
		printf("Pin PB10 is set low\r\n");
	}
	else if (PIN == 3)
	{
		port_pin_set_config(PIN_1, &configure_port_pin);
		port_pin_set_output_level(PIN_1, false);
		printf("Pin PA19 is set low\r\n");
	}
	else
	{
		printf("PIN passed, undefined");
	}
}

void check_pin(uint8_t PIN2)
{
	bool pin_state;
	if (PIN2 == 0)
	{
		pin_state = port_pin_get_output_level(PIN_2);
		if (pin_state)
		{
			printf("Pin PB10 is set high\r\n");
		}
		else
		{
			printf("Pin PB10 is set low\r\n");
		}
	}
	else if (PIN2 == 1)
	{
		pin_state = port_pin_get_output_level(PIN_1);
		if (pin_state)
		{
			printf("Pin PA19 is set high\r\n");
		}
		else
		{
			printf("Pin PA19 is set low\r\n");
		}
	}
	else
	{
		printf("PIN passed, undefined");
	}
}

//_________________USART______________

void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = UART_BAUDRATE;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PB11D_SERCOM4_PAD3;
	
	stdio_serial_init(&usart_instance, SERCOM4, &config_usart);
	usart_enable(&usart_instance);
}

void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance,
	usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

void usart_read_callback(struct usart_module *const usart_module)
{
	processUserInput();
}

void usart_write_callback(struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
}

//_________________TIMER______________

static void configure_tcc(void)
{
	tcc_get_config_defaults(&config_tcc, TCC1);
	config_tcc.counter.clock_source = GCLK_GENERATOR_3; // 8 Mhz
	config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV64; // 8 Mhz/64 = 125000
	config_tcc.counter.period =  PERIOD; //100ms 12500
	tcc_init(&tcc_instance, TCC1, &config_tcc);
	
	tcc_register_callback(&tcc_instance, tcc_interval_wait, TCC_CALLBACK_OVERFLOW);
	tcc_enable_callback(&tcc_instance, TCC_CALLBACK_OVERFLOW);
	tcc_enable(&tcc_instance);
}

static void tcc_interval_wait(struct tcc_module *const module_inst)
{
	timer_counter++; //overflow trigger
	//tcc_flag = true;
}
//static void configure_tcc(void)
//{
	//tcc_get_config_defaults(&config_tcc, TCC0);
	//config_tcc.counter.clock_source = GCLK_GENERATOR_3; //8Mhz
	//config_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV64;
	////config_tcc.counter.count = 20000;
	//config_tcc.counter.period =  125; //1ms
	////config_tcc.compare.match[0] =  900;
	//tcc_init(&tcc_instance, TCC0, &config_tcc);
	//
	////tcc_get_config_defaults(&config_pwm_tcc, CONF_PWM_MODULE);
	////config_pwm_tcc.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV8;
	////config_pwm_tcc.counter.period = 10; // 50Hz or 20ms (pre scale = 64)
	////config_pwm_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	////config_pwm_tcc.pins.wave_out_pin[CONF_PWM_OUTPUT]        = CONF_PWM_OUT_PIN;
	////config_pwm_tcc.pins.wave_out_pin_mux[CONF_PWM_OUTPUT]    = CONF_PWM_OUT_MUX;
//}
//
//static void configure_tcc_callbacks(void)
//{
	//tcc_register_callback(&tcc_instance, tcc_interval_wait, TCC_CALLBACK_OVERFLOW);
	//tcc_enable_callback(&tcc_instance, TCC_CALLBACK_OVERFLOW);
//}
//
//static void tcc_interval_wait(struct tcc_module *const module_inst)
//{
	//
	//interval_count(); //overflow trigger
//}
//
//void interval_count()
//{
	//if (nReadings < atoi(arguments[1])) // user input for number of readings
	//{
		//static uint16_t counter;
		//if (counter == 	atoi(arguments[2])) //user input for wait interval (ms)
		//{
			//proxSensorRead();
			////port_pin_toggle_output_level(LED0_PIN); //debug LED
			//nReadings++;
			//counter = 0;
		//}
		//counter++;
	//}
	//else
	//{
		//adc_disable(&adc_prox_instance);
		//tcc_disable(&tcc_instance);
	//}
//}

//_________________SERVO______________

//void servo(int8_t degrees)
//{
	//int pwm = 125 + ((double)(90 + degrees)/180.0)*125.0; // 1ms = -90 | 1.5ms = 0 | 2ms = 90
	//
	//config_pwm_tcc.compare.match[CONF_PWM_CHANNEL] = pwm;
	//
	//tcc_init(&tcc_pwm_instance, CONF_PWM_MODULE, &config_pwm_tcc);
	//tcc_enable(&tcc_pwm_instance);
	//
	//config_pwm_tcc.pins.enable_wave_out_pin[CONF_PWM_OUTPUT] = true;
//}

//_________________AT25DFX______________

void at25dfx_commands()
{
	at25dfx_chip_wake(&at25dfx_chip);
		
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("Error: AT25DFX");	// Handle missing or non-responsive device
	}
		
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x0000, read_buffer, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_set_sector_protect(&at25dfx_chip, 0x10000, false);
	at25dfx_chip_erase_block(&at25dfx_chip, 0x10000, AT25DFX_BLOCK_SIZE_4KB);
	at25dfx_chip_write_buffer(&at25dfx_chip, 0x10000, write_buffer, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_read_buffer(&at25dfx_chip, 0x10000, read_buffer, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, true);
	at25dfx_chip_sleep(&at25dfx_chip);
}

void configure_nvm(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	nvm_set_config(&config_nvm);
}

void configure_pins(void)
{
	// LED
	struct port_config configure_port_pin;
	port_get_config_defaults(&configure_port_pin);
	configure_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &configure_port_pin);
	
	// Button
	configure_port_pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(BUTTON_DEBUG, &configure_port_pin);
}

static void init_state(void)
{
	down_state = NOT_READY;
}


static void clear_state(download_state mask)
{
	down_state &= ~mask;
}

static void add_state(download_state mask)
{
	down_state |= mask;
}


static inline bool is_state_set(download_state mask)
{
	return ((down_state & mask) != 0);
}


static void start_download(void)
{
	if (!is_state_set(WIFI_CONNECTED)) {
		printf("start_download: Wi-Fi is not connected.\r\n");
		return;
	}

	if (is_state_set(GET_REQUESTED)) {
		printf("start_download: request is sent already.\r\n");
		return;
	}

	if (is_state_set(DOWNLOADING)) {
		printf("start_download: running download already.\r\n");
		return;
	}

	/* Send the HTTP request. */
	printf("start_download: sending HTTP request...\r\n");
	
	//if (meta_flag)
	//{
		http_client_send_request(&http_client_module_inst, MAIN_HTTP_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
		//meta_flag = false;
	//}
	//else
	//{
		//http_client_send_request(&http_client_module_inst, MAIN_HTTP_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
		//}
}

static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	switch (type) {
		case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
		printf("http_client_callback: HTTP client socket connected.\r\n");
		break;

		case HTTP_CLIENT_CALLBACK_REQUESTED:
		printf("http_client_callback: request completed.\r\n");
		erase_at25dfx();
		add_state(GET_REQUESTED);
		break;

		case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
		printf("http_client_callback: received response %u data size %u\r\n",
		(unsigned int)data->recv_response.response_code,
		(unsigned int)data->recv_response.content_length);
		if ((unsigned int)data->recv_response.response_code == 200) {
			http_file_size = data->recv_response.content_length;
			received_file_size = 0;
		}
		else {
			add_state(CANCELED);
			return;
		}
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			save_packet(data->recv_response.content, data->recv_response.content_length);
			//process_metadata(data->recv_response.content, data->recv_response.content_length);
			add_state(COMPLETED);
		}
		break;

		case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		save_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
		//process_metadata(data->recv_chunked_data.data, data->recv_chunked_data.length);
		if (data->recv_chunked_data.is_complete) {
			add_state(COMPLETED);
		}
		break;

		case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		if (data->disconnected.reason == -EAGAIN) {
			/* Server has not responded. Retry immediately. */
			if (is_state_set(DOWNLOADING)) {
				//f_close(&file_object);
				clear_state(DOWNLOADING);
			}
			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}
			start_download();
		}
		break;
	}
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	http_client_socket_event_handler(sock, u8Msg, pvMsg);
	mqtt_socket_event_handler(sock, u8Msg, pvMsg);
}

static void resolve_cb(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
	(int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
	(int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
	http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
	mqtt_socket_resolve_handler(pu8DomainName, u32ServerIP);
}

static void mqtt_callback(struct mqtt_module *module_inst, int type, union mqtt_data *data)
{
	switch (type) {
	case MQTT_CALLBACK_SOCK_CONNECTED:
	{
		if (data->sock_connected.result >= 0) {
			mqtt_connect_broker(module_inst, 1, MQTT_ID, MQTT_PASSWORD, MQTT_CLIENT_ID, NULL, NULL, 0, 0, 0);
		} else {
			printf("Connect fail to server(%s)! retry it automatically.\r\n", main_mqtt_broker);
			mqtt_connect(module_inst, main_mqtt_broker); /* Retry that. */
		}
	}
	break;

	case MQTT_CALLBACK_CONNECTED:
		if (data->connected.result == MQTT_CONN_RESULT_ACCEPT) {
			mqtt_subscribe(module_inst, MAIN_MQTT_TOPIC, 1);					/* Subscribe chat topic. */
			
		} else {																/* Cannot connect for some reason. */
			printf("MQTT broker decline your access! error code %d\r\n", data->connected.result);
		}

		break;

	case MQTT_CALLBACK_RECV_PUBLISH:
		
		if (data->recv_publish.topic != NULL && data->recv_publish.msg != NULL) {
			//if (!strncmp(data->recv_publish.topic, MAIN_MQTT_TOPIC, strlen(MAIN_MQTT_TOPIC))) {
				
				for (int i = strlen(MAIN_MQTT_TOPIC)-1; i < data->recv_publish.topic_size; i++) {
					printf("%c", data->recv_publish.topic[i]);
				
				}
				printf(" >> ");
				for (int i = 0; i < data->recv_publish.msg_size; i++) {
					printf("%c", data->recv_publish.msg[i]);
				}
				printf("\r\n");
				//printf("%c", data->recv_publish.topic[10]);
				//char temp = "b";
				if (data->recv_publish.topic[11] == 'u')
				{
					printf("inside if\r\n");
					prox_read_flag=true;
				}
				
			//}
		}

		break;

	case MQTT_CALLBACK_DISCONNECTED:
		/* Stop timer and USART callback. */
		printf("MQTT disconnected\r\n");
		break;
	}
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
		case M2M_WIFI_RESP_CON_STATE_CHANGED:
		{
			tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
			if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
				printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
				m2m_wifi_request_dhcp_client();
				} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
				printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
				clear_state(WIFI_CONNECTED);
				if (is_state_set(DOWNLOADING)) {
					//f_close(&file_object);
					clear_state(DOWNLOADING);
				}

				if (is_state_set(GET_REQUESTED)) {
					clear_state(GET_REQUESTED);
				}

				m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
				MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
				mqtt_disconnect(&mqtt_inst, 1);
			}

			break;
		}

		case M2M_WIFI_REQ_DHCP_CONF:
		{
			uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
			printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
			add_state(WIFI_CONNECTED);
			start_download();
			mqtt_connect(&mqtt_inst, main_mqtt_broker);
			break;
		}

		default:
		break;
	}
}

static void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = MAIN_BUFFER_MAX_SIZE;
	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0)
	{
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1)
		{
			/* Loop forever. */
		}
	}
	//http_client_register_callback(&http_client_module_inst, http_client_callback); //this is to start downloading files
}

static void configure_timer(void)
{
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);

	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
	
	sw_timer_init(&swt_module_inst_mqtt, &swt_conf);
	sw_timer_enable(&swt_module_inst_mqtt);
}

static void configure_mqtt(void)
{
	struct mqtt_config mqtt_conf;
	int result;

	mqtt_get_config_defaults(&mqtt_conf);
	/* To use the MQTT service, it is necessary to always set the buffer and the timer. */
	mqtt_conf.port = 16615; //8000;
	mqtt_conf.timer_inst = &swt_module_inst_mqtt;
	mqtt_conf.recv_buffer = mqtt_buffer;
	mqtt_conf.recv_buffer_size = MAIN_MQTT_BUFFER_SIZE;

	result = mqtt_init(&mqtt_inst, &mqtt_conf);
	if (result < 0) {
		printf("MQTT initialization failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}

	result = mqtt_register_callback(&mqtt_inst, mqtt_callback);
	if (result < 0) {
		printf("MQTT register callback failed. Error code is (%d)\r\n", result);
		while (1) {
		}
	}
}

static void at25dfx_init(void)
{
	struct at25dfx_chip_config at25dfx_config;
	struct spi_master_vec_config at25dfx_spi_config;
	at25dfx_spi_master_vec_get_config_defaults(&at25dfx_spi_config);
	at25dfx_spi_config.baudrate    = 125000;
	at25dfx_spi_config.mux_setting = SPI_SIGNAL_MUX_SETTING_E;
	at25dfx_spi_config.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	at25dfx_spi_config.pinmux_pad1 = PINMUX_UNUSED;
	at25dfx_spi_config.pinmux_pad2 = PINMUX_PA18C_SERCOM1_PAD2;
	at25dfx_spi_config.pinmux_pad3 = PINMUX_PA19C_SERCOM1_PAD3;
	spi_master_vec_init(&at25dfx_spi, SERCOM1, &at25dfx_spi_config);
	spi_master_vec_enable(&at25dfx_spi);
		
	at25dfx_config.type = AT25DFX_081A;
	at25dfx_config.cs_pin = PIN_PA07;
	at25dfx_chip_init(&at25dfx_chip, &at25dfx_spi, &at25dfx_config);
}

void erase_at25dfx()
{
	at25dfx_chip_wake(&at25dfx_chip);

	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("Error: AT25DFX");	// Handle missing or non-responsive device
	}
		
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
	if (downloadedImage == true)	//erase image1
	{
		int i;
			
		for (i = AT25DFX_IMAGE1_ADDR; i<AT25DFX_IMAGE2_ADDR; i=i+4096)
		{
			at25dfx_chip_erase_block(&at25dfx_chip, i, AT25DFX_BLOCK_SIZE_4KB);
		}
	}
	else							//erase image2
	{
		int i;
			
		for (i = AT25DFX_IMAGE2_ADDR; i<0x7D0000; i=i+4096)
		{
			at25dfx_chip_erase_block(&at25dfx_chip, i, AT25DFX_BLOCK_SIZE_4KB);
		}
	}
}

void save_packet(char *data, uint32_t length)
{
	crc_check(*data, length);
		
	for (int i=0; i <length;i++)
	{
		if (bufferIndex<AT25DFX_BUFFER_SIZE)
		{
			read_buffer1[bufferIndex] = data[i];
		}
		else
		{
			read_buffer2[bufferIndex-AT25DFX_BUFFER_SIZE] = data[i];
			bufferOverflow1=1;
		}
		bufferIndex++;
			
		if (bufferIndex == AT25DFX_BUFFER_SIZE)
		{
			at25dfx_chip_write_buffer(&at25dfx_chip, flashWriteAddress, read_buffer1, AT25DFX_BUFFER_SIZE);
			flashWriteAddress+= AT25DFX_BUFFER_SIZE;
		}
		if (bufferIndex == AT25DFX_BUFFER_SIZE*2)
		{
			bufferIndex=0;
			at25dfx_chip_write_buffer(&at25dfx_chip, flashWriteAddress, read_buffer2, AT25DFX_BUFFER_SIZE);
			flashWriteAddress+= AT25DFX_BUFFER_SIZE;
		}
	}
		
	appcodeSize-=length;
	if (appcodeSize==0)
	{
		if(bufferIndex<AT25DFX_BUFFER_SIZE)
		{
			for (int i=bufferIndex; i < AT25DFX_BUFFER_SIZE; i++)

			{
				read_buffer1[i] = AT25DFX_BUFFER_SIZE-1;
			}
			at25dfx_chip_write_buffer(&at25dfx_chip, flashWriteAddress, read_buffer1, AT25DFX_BUFFER_SIZE);
		}
		else
		{
			for (int i=bufferIndex;i < AT25DFX_BUFFER_SIZE*2; i++)
			{
				read_buffer2[i] = AT25DFX_BUFFER_SIZE-1;
			}
			at25dfx_chip_write_buffer(&at25dfx_chip, flashWriteAddress, read_buffer2, AT25DFX_BUFFER_SIZE);
		}
		add_state(COMPLETED);
	}
}

void crc_check(uint8_t *block, uint32_t length)
{
	if (!first_CRC) {						// first CRC check
		crc32_recalculate(block, length, &crc);
	}
	
	else {									// subsequent CRC checks
		crc32_calculate(block, length, &crc);
		first_CRC = false;
	}
}

void process_metadata(char *data, uint32_t length)
{
	#define DELIMITER " "
	int16_t nargs = 0;
	char *midStringPtr;
	arguments[nargs++] = data;
	midStringPtr = strpbrk(data, DELIMITER);
	while (midStringPtr != NULL) {
		arguments[nargs] = midStringPtr+1;
		*midStringPtr = '\0';
		midStringPtr = strpbrk(arguments[nargs], DELIMITER);
		nargs++;
	}
	
	uint32_t temp_array[MAX_ARGS];
	
	for (int i = 0; i < MAX_ARGS; i++)
	{
		temp_array[i] = atoll(arguments[i]);
	}
	appcodeSize = temp_array[0];
	crc_master  = temp_array[1]; //FIX THIS, DOES NOT SAVE THE CORRECT VALUE, ONLY 0X0000000
	hardware_major  = temp_array[2];
	hardware_minor  = temp_array[3];
	hardware_patch  = temp_array[4];
	software_major  = temp_array[5];
	software_minor  = temp_array[6];
	software_patch  = temp_array[7];
	
	uint32 metadata_address;				//write to metadata flash
	
	if (write_image_1)
	{
		downloadedImage = true;
		metadata_address = 0x1000;
	}
	else
	{
		downloadedImage = false;
		metadata_address = 0x40000;
	}
	
	at25dfx_chip_wake(&at25dfx_chip);
		
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("Error: AT25DFX");	// Handle missing or non-responsive device
	}
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
	
	at25dfx_chip_erase_block(&at25dfx_chip, AT25DFX_STATUS, AT25DFX_BLOCK_SIZE_4KB);
	at25dfx_chip_write_buffer(&at25dfx_chip, AT25DFX_STATUS, &downloadedImage, 1);
	
	at25dfx_chip_erase_block(&at25dfx_chip, metadata_address, AT25DFX_BLOCK_SIZE_4KB);
	at25dfx_chip_write_buffer(&at25dfx_chip, metadata_address, temp_array, AT25DFX_BUFFER_SIZE);
	at25dfx_chip_read_buffer(&at25dfx_chip, metadata_address, test_buffer, AT25DFX_BUFFER_SIZE);

	at25dfx_chip_sleep(&at25dfx_chip);
}

void wifi_init()
{
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));	/* Initialize Wi-Fi parameters structure. */

	param.pfAppWifiCb = wifi_cb;	/* Initialize Wi-Fi driver with data and status callbacks. */
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error! (res %d)\r\n", ret);
		while (1) {
		}
	}

	socketInit();		/* Initialize socket module. */
	registerSocketCallback(socket_cb, resolve_cb);	/* Register socket callback function. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
}

void button_debug()
{
	if (port_pin_get_input_level(
	_DEBUG) == false){
		
		if (button_change == false){
			prox_read_flag = true,
			button_change = true;
			printf("button");
			char topic[19] = "ese516/g2/prox";
			char msg[2];
			//sprintf(topic, "%s%s", MAIN_MQTT_ROOT, MQTT_PROX);
			sprintf(msg, "%d", distance);
			//printf(dist);
			//printf("%d", distance);
			//printf(msg);
			//mqtt_publish(&mqtt_inst, topic, msg, sizeof(msg) , 1, 1); //to publish the result to cloud
		}
	}
	else
	{
		button_change = false;
	}
}

//----------------PROXIMITY SENSOR-------------

void prox_init()
{
	// configure proximity sensor ADC
	adc_get_config_defaults(&config_adc_prox);
	config_adc_prox.reference = ADC_REFERENCE_AREFA;
	config_adc_prox.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
	config_adc_prox.positive_input = ADC_POSITIVE_INPUT_PIN10;	// corresponds to PIN_PB02
	adc_init(&adc_prox_instance, ADC, &config_adc_prox);
	adc_enable(&adc_prox_instance);
	tcc_enable(&tcc_instance);
}

void average()
{
	uint16_t temp;
	for (int i = 2; i<PROX_BUFFER_SIZE-2; i++){
		temp += prox_buffer[i];
		//printf("proxBuffer: %d\r\n", temp);
	}
	calc_distance(temp/6);
}

void calc_distance(uint16_t number)
{
	double dist;
	dist = 27.86 * pow(((number*3.3)/4096),- 1.15);
	//uint16_t temp = 20;
		if (dist > 10 && dist < 80)
		{
			printf("The distance is %d cm and the read is %d \r\n", (int)dist, number);
		}
		else
		{
			printf("Point at something between 10-80 cm\r\n");
		}
	trash_bracket = map((uint16_t)dist, 0, 80, 4, 0);
	trash_level = map((uint16_t)dist, 0, 80, 100, 0);
	printf("%d \r\n" , (int)trash_bracket);
	 //

}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void swap(uint16_t *xp, uint16_t *yp)
{
	uint16_t temp = *xp;
	*xp = *yp;
	*yp = temp;
}

// A function to implement bubble sort
void bubbleSort(uint16_t arr[], int n)
{
	int i, j;
	for (i = 0; i < n-1; i++)
	
	// Last i elements are already in place
	for (j = 0; j < n-i-1; j++)
	if (arr[j] > arr[j+1])
	swap(&arr[j], &arr[j+1]);
}

void prox_read_handler()
{
	//printf("prox read handler");
	if (prox_read_flag && timer_counter >= DELAY)		// 1 second has gone by 
	{
		//if (!prox_read_count) {
			//config_tcc.counter.count = 0;
		//}
		prox_read();
		prox_read_count++;
		//printf("Prox Counter = %d\r\n", prox_read_count);
		timer_counter = 0;
		
		printf(".");
		
		
		if (prox_read_count == PROX_BUFFER_SIZE) {
			printf("Prox Readings Complete\r\n");
			prox_read_count = 0;
			prox_read_flag = false;
			bubbleSort(prox_buffer, PROX_BUFFER_SIZE);
			average();
			mqtt_update();
		}
	}
}

void prox_read(void)
{
	//start conversion
	uint16_t result;
	adc_start_conversion(&adc_prox_instance);

	while (adc_read(&adc_prox_instance, &result) == STATUS_BUSY);
	prox_buffer[prox_read_count] = result;
	//printf("%d\r\n", result); //
}

void mqtt_update(void)
{
	// UPDATING THE BLUEPRINT PHOTO
	char *topic1 = "ese516/g2/blueprint";
	char *msg1 = malloc(44);
	sprintf(msg1, "%s%d%s", TRASH_PIC_ROOT, trash_bracket, FILE_EXTENSION);
	printf("MQTT UPDATE\r\n");
	//printf("%s", msg);
	mqtt_publish(&mqtt_inst, topic1, msg1, 45, 1, 0);
	
	// UPDATING TRASH LEVEL
	char *topic2 = "ese516/g2/prox";
	char *msg2 = malloc(3);
	sprintf(msg2, "%d", trash_level);
	mqtt_publish(&mqtt_inst, topic2, msg2, 3, 1, 1);
	
	// TWILIO
	if (trash_bracket == 4)
	{
		char *topic3 = "ese516/g2/alert";
		char *msg3 = malloc(122);
		sprintf(msg3, "%s", ALERT_NOTIFICATION);
		//printf("%s", msg3);
		mqtt_publish(&mqtt_inst, topic3, msg3, 106, 1, 0);
	}
}

int main (void)
{	
	//Initializing
	system_init();
	board_init();
	init_state();
	configure_usart();
	configure_usart_callbacks();
	configure_tcc();
	configure_timer();					/* Initialize the Timer. */
	configure_http_client();			/* Initialize the HTTP client service. */
	configure_mqtt();					//Configure MQTT
	nm_bsp_init();						/* Initialize the BSP. */
	system_interrupt_enable_global();
	at25dfx_init();
	prox_init();
	//configure_tcc();
	//configure_tcc_callbacks();
	configure_nvm();
	configure_pins();
	
	printf("Configuration Complete");
		
	wifi_init();
	
	uint8_t string[] = "Write 'help' to see all the functions\r\n";
	usart_write_buffer_wait(&usart_instance, string, sizeof(string));
	while (true)
	{
		//port_pin_toggle_output_level(LED_0_PIN);;
		if (!(is_state_set(COMPLETED) || is_state_set(CANCELED)))
		{
			m2m_wifi_handle_events(NULL);				/* Handle pending events from network controller. */
			sw_timer_task(&swt_module_inst);			/* Checks the timer timeout. */
		}
		
		button_debug();
		prox_read_handler();
		//if(tcc_flag == true)
		//{
			//printf("%d\r\n", timer_counter);
			//tcc_flag = false;
		//}
		usart_read_buffer_job(&usart_instance, &singleInput, 1); //write to single buffer
		if (processFlag)
		{
			processCommand();
		}
	}
}
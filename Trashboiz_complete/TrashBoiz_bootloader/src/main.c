#include <asf.h>
// -Wl,-Ttext=8000
#define BUTTON_0_PIN		PIN_PA24
#define BUTTON_0_ACTIVE		false
#define LED_0_PIN			PIN_PA23
#define LED_0_ACTIVE		false
#define UART_BAUDRATE		115200
#define NVM_PAGE_SIZE		64
#define	NVM_ROW_PAGES		4
#define	NVM_NUM_PAGES		4096
#define NVM_BOOTSTATUS		0x003F00
#define NVM_APPCODE			0x004000
#define NVM_END				0x040000
#define flash_metadata		0x001000
#define flash_data			0x002000
#define flash_end			0x040000
#define AT25DFX_BUFFER_SIZE		256
#define AT25DFX_IMAGE1_ADDR		0x1000
#define AT25DFX_IMAGE2_ADDR		0x40000


struct usart_module usart_instance;
struct nvm_parameters nvm_parameters_current;
enum status_code error_code;

uint8_t nvm_page_buffer[NVM_PAGE_SIZE];
uint8_t nvm_read_buffer[NVM_PAGE_SIZE];
uint8_t temp_buffer[NVM_PAGE_SIZE*NVM_ROW_PAGES];

//----------------AT25DFX-------------
#define AT25DFX_IMAGE1_ADDR 0x1000
#define AT25DFX_IMAGE2_ADDR 0x40000
static uint8_t read_buffer1[AT25DFX_BUFFER_SIZE];
static uint8_t read_buffer2[AT25DFX_BUFFER_SIZE];
static uint8_t read_buffer[AT25DFX_BUFFER_SIZE];
static uint8_t write_buffer[AT25DFX_BUFFER_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
struct spi_master_vec_module at25dfx_spi;
struct at25dfx_chip_module at25dfx_chip;
struct usart_module usart_instance;

static void check_boot_mode(void)
{
	uint32_t app_check_address;
	uint32_t *app_check_address_ptr;
	/* Check if WDT is locked */
	if (!(WDT->CTRL.reg & WDT_CTRL_ALWAYSON)) {
		/* Disable the Watchdog module */
		WDT->CTRL.reg &= ~WDT_CTRL_ENABLE;
	}
	app_check_address = NVM_APPCODE;
	app_check_address_ptr = (uint32_t *)app_check_address;
	board_init();
	
	if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
		/* Button is pressed, run bootloader */
		return;
	}
	if (*app_check_address_ptr == 0xFFFFFFFF) {
		/* No application; run bootloader */
		return;
	}
	
	usart_disable(&usart_instance);

	/* Pointer to the Application Section */
	void (*application_code_entry)(void);
	/* Rebase the Stack Pointer */
	__set_MSP(*(uint32_t *)NVM_APPCODE);
	/* Rebase the vector table base address TODO: use RAM */
	SCB->VTOR = ((uint32_t)NVM_APPCODE & SCB_VTOR_TBLOFF_Msk);
	/* Load the Reset Handler address of the application */
	application_code_entry = (void (*)(void))(unsigned *)(*(unsigned *)
	(NVM_APPCODE + 4));
	/* Jump to user Reset Handler in the application */
	application_code_entry();
}

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

void configure_nvm(void)
{
	struct nvm_config config_nvm;
	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
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
	port_pin_set_config(BUTTON_0_PIN, &configure_port_pin);
}

void erase_nvm(void)
{
	for (int i = NVM_APPCODE; i < NVM_END; i += NVM_ROW_PAGES * NVM_PAGE_SIZE)
	{
		do
		{
			error_code = nvm_erase_row(i);
		} while (error_code == STATUS_BUSY);
	}
}

void rewrite_appcode(void)
{
	at25dfx_chip_wake(&at25dfx_chip);
		
	if (at25dfx_chip_check_presence(&at25dfx_chip) != STATUS_OK) {
		printf("Error: AT25DFX");	// Handle missing or non-responsive device
	}
	at25dfx_chip_set_global_sector_protect(&at25dfx_chip, false);
	
	for (uint16_t i = NVM_APPCODE; i<NVM_APPCODE + length; i+=256) //need to save length from the wifi download into metadata
	{
		{
			at25dfx_chip_read_buffer(&at25dfx_chip, 0x002000+i, temp_buffer, AT25DFX_BUFFER_SIZE); //change depending on what the metadata says
			for (int j = 0; j < NVM_ROW_PAGES; j++)
			{
				do
				{
					error_code = nvm_write_buffer(i + j*NVM_APPCODE, temp_buffer[j*NVM_PAGE_SIZE], NVM_PAGE_SIZE);
				} while(error_code == STATUS_BUSY);
			}
		}
	}
	at25dfx_chip_sleep(&at25dfx_chip);
}

int main (void)
{
	system_init();
	configure_usart();
	configure_nvm();
	configure_pins();
	
	printf("Bootloader");
	
	do
	{
		error_code = nvm_read_buffer(NVM_APPCODE, nvm_read_buffer, NVM_PAGE_SIZE);
	} while (error_code == STATUS_BUSY);
	
	erase_nvm();
	rewrite_appcode();
	
	do
	{
		error_code = nvm_read_buffer(NVM_APPCODE, nvm_read_buffer, NVM_PAGE_SIZE);
	} while (error_code == STATUS_BUSY);
	
	while (1) {
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, so turn LED on. */
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
			//check_boot_mode(); //check boot status
		} else {
			/* No, so turn LED off. */
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
		}
	}
	
	//nvm_get_parameters(&nvm_parameters_current);
	//do
	//{
		//error_code = nvm_erase_row(100 * NVM_ROW_PAGES * NVM_PAGE_SIZE);
	//} while (error_code == STATUS_BUSY);
	//
	//
	//do
	//{
		//error_code = nvm_read_buffer(100 * NVM_ROW_PAGES * NVM_PAGE_SIZE, nvm_read_buffer, NVM_PAGE_SIZE);
	//} while (error_code == STATUS_BUSY);
}

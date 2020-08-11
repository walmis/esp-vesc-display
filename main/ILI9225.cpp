#include "FreeRTOS.h"
#include "task.h"
#include "ILI9225.h"
#include <driver/gpio.h>
#include "sdkconfig.h"
#include "spi.h"
#include <algorithm>
#include <unistd.h>

/* ILI9225 screen size */
#define ILI9225_LCD_WIDTH  176
#define ILI9225_LCD_HEIGHT 220

/* ILI9225 LCD Registers */
#define ILI9225_DRIVER_OUTPUT_CTRL      (0x01u)  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     (0x02u)  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE              (0x03u)  // Entry Mode
#define ILI9225_DISP_CTRL1              (0x07u)  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      (0x08u)  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        (0x0Bu)  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          (0x0Cu)  // Interface Control
#define ILI9225_OSC_CTRL                (0x0Fu)  // Osc Control
#define ILI9225_POWER_CTRL1             (0x10u)  // Power Control 1
#define ILI9225_POWER_CTRL2             (0x11u)  // Power Control 2
#define ILI9225_POWER_CTRL3             (0x12u)  // Power Control 3
#define ILI9225_POWER_CTRL4             (0x13u)  // Power Control 4
#define ILI9225_POWER_CTRL5             (0x14u)  // Power Control 5
#define ILI9225_VCI_RECYCLING           (0x15u)  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           (0x20u)  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           (0x21u)  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           (0x22u)  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          (0x30u)  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   (0x31u)  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   (0x32u)  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   (0x33u)  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    (0x34u)  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    (0x35u)  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 (0x36u)  // Horizontal Address Start Position
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 (0x37u)  // Horizontal Address End Position
#define ILI9225_VERTICAL_WINDOW_ADDR1   (0x38u)  // Vertical Address Start Position
#define ILI9225_VERTICAL_WINDOW_ADDR2   (0x39u)  // Vertical Address End Position
#define ILI9225_GAMMA_CTRL1             (0x50u)  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             (0x51u)  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3             (0x52u)  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4             (0x53u)  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5             (0x54u)  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6             (0x55u)  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7             (0x56u)  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8             (0x57u)  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             (0x58u)  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            (0x59u)  // Gamma Control 10

#define ILI9225C_INVOFF  0x20
#define ILI9225C_INVON   0x21

enum autoIncMode_t {
	R2L_BottomUp,
	BottomUp_R2L,
	L2R_BottomUp,
	BottomUp_L2R,
	R2L_TopDown,
	TopDown_R2L,
	L2R_TopDown,
	TopDown_L2R
};

#define ILI9225_LCD_WIDTH  176
#define ILI9225_LCD_HEIGHT 220

static uint8_t _orientation;
static uint8_t _maxX, _maxY;

#define delay(n) usleep(n*1000)

#define CS_LOW gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_CS, 0);
#define CS_HIGH gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_CS, 1);
#define DC_LOW gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 0);
#define DC_HIGH gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 1);

static void _writeRegister(uint16_t reg, uint16_t data) {
    spi_writeCommand16(reg);
    spi_writeData16(data);
    spi_flushdata();
}

const uint8_t modeTab [3][8] = {
    //          { R2L_BottomUp, BottomUp_R2L, L2R_BottomUp, BottomUp_L2R, R2L_TopDown,  TopDown_R2L,  L2R_TopDown,  TopDown_L2R }//
    /* 90° */{BottomUp_L2R, L2R_BottomUp, TopDown_L2R, L2R_TopDown, BottomUp_R2L, R2L_BottomUp, TopDown_R2L, R2L_TopDown},
    /*180° */{L2R_TopDown , TopDown_L2R, R2L_TopDown, TopDown_R2L, L2R_BottomUp, BottomUp_L2R, R2L_BottomUp, BottomUp_R2L},
    /*270° */{TopDown_R2L , R2L_TopDown, BottomUp_R2L, R2L_BottomUp, TopDown_L2R, L2R_TopDown, BottomUp_L2R, L2R_BottomUp}
};

static inline  void _swap(uint16_t &a, uint16_t &b) {
	uint16_t w = a;
	a = b;
	b = w;
}

void setOrientation(uint8_t orientation) {

	_orientation = orientation % 4;

	switch (_orientation) {
	case 0:
		_maxX = ILI9225_LCD_WIDTH;
		_maxY = ILI9225_LCD_HEIGHT;

		break;
	case 1:
		_maxX = ILI9225_LCD_HEIGHT;
		_maxY = ILI9225_LCD_WIDTH;
		break;
	case 2:
		_maxX = ILI9225_LCD_WIDTH;
		_maxY = ILI9225_LCD_HEIGHT;
		break;
	case 3:
		_maxX = ILI9225_LCD_HEIGHT;
		_maxY = ILI9225_LCD_WIDTH;
		break;
	}
}

static void _orientCoordinates(uint16_t &x1, uint16_t &y1) {

	switch (_orientation) {
	case 0:  // ok
		break;
	case 1:  // ok
		y1 = _maxY - y1 - 1;
		_swap(x1, y1);
		break;
	case 2:  // ok
		x1 = _maxX - x1 - 1;
		y1 = _maxY - y1 - 1;
		break;
	case 3:  // ok
		x1 = _maxX - x1 - 1;
		_swap(x1, y1);
		break;
	}
}

void ILI9225_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t mode = L2R_TopDown;

	// clip to TFT-Dimensions
	x0 = std::min(x0, (uint16_t) (_maxX - 1));
	x1 = std::min(x1, (uint16_t) (_maxX - 1));
	y0 = std::min(y0, (uint16_t) (_maxY - 1));
	y1 = std::min(y1, (uint16_t) (_maxY - 1));
	_orientCoordinates(x0, y0);
	_orientCoordinates(x1, y1);

	if (x1 < x0)
		_swap(x0, x1);
	if (y1 < y0)
		_swap(y0, y1);

	// autoincrement mode
	if (_orientation > 0)
		mode = modeTab[_orientation - 1][mode];

    spi_beginTransaction(CONFIG_GPIO_TFT_CS);

	_writeRegister(ILI9225_ENTRY_MODE, 0x1000 | (mode << 3));
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, x1);
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, x0);

	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, y1);
	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, y0);

	// starting position within window and increment/decrement direction
	switch (mode >> 1) {
	case 0:
		_writeRegister(ILI9225_RAM_ADDR_SET1, x1);
		_writeRegister(ILI9225_RAM_ADDR_SET2, y1);
		break;
	case 1:
		_writeRegister(ILI9225_RAM_ADDR_SET1, x0);
		_writeRegister(ILI9225_RAM_ADDR_SET2, y1);
		break;
	case 2:
		_writeRegister(ILI9225_RAM_ADDR_SET1, x1);
		_writeRegister(ILI9225_RAM_ADDR_SET2, y0);
		break;
	case 3:
		_writeRegister(ILI9225_RAM_ADDR_SET1, x0);
		_writeRegister(ILI9225_RAM_ADDR_SET2, y0);
		break;
	}
    
	spi_writeCommand16( ILI9225_GRAM_DATA_REG);

    spi_endTransaction();


	//_writeRegister(ILI9225_RAM_ADDR_SET1,x0);
	//_writeRegister(ILI9225_RAM_ADDR_SET2,y0);

	//_writeCommand(0x00, 0x22);

}


void ILI9225_init() {
    gpio_config_t cfg = {};
    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.mode = GPIO_MODE_OUTPUT;

    cfg.pin_bit_mask = BIT(CONFIG_GPIO_TFT_CS) | BIT(CONFIG_GPIO_TFT_DC) | BIT(CONFIG_GPIO_TFT_RST) ;
    gpio_config(&cfg);



	// Control pins
    gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_CS, 1);
    gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 1);

	// Initialization Code
	gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_RST, 1);

	delay(10);
	gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_RST, 0);
	delay(10);
	gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_RST, 1);; // Pull the reset pin high to release the ILI9225C from the reset status
	delay(10);

	/* Start Initial Sequence */
    spi_beginTransaction(CONFIG_GPIO_TFT_CS);

	/* Set SS bit and direction output from S528 to S1 */
	_writeRegister(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
	_writeRegister(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
	_writeRegister(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
	_writeRegister(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
	_writeRegister(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
    
    spi_endTransaction();


	delay(50);
    
    spi_beginTransaction(CONFIG_GPIO_TFT_CS);
	// Power-on sequence
	_writeRegister(ILI9225_POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
	_writeRegister(ILI9225_POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
	_writeRegister(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
	_writeRegister(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
	_writeRegister(ILI9225_POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
    spi_endTransaction();

	delay(10);

    spi_beginTransaction(CONFIG_GPIO_TFT_CS);
	_writeRegister(ILI9225_POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
    spi_endTransaction();

	delay(50);
    
    spi_beginTransaction(CONFIG_GPIO_TFT_CS);
	_writeRegister(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
	_writeRegister(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
	_writeRegister(ILI9225_ENTRY_MODE, 0x1038); // set GRAM write direction and BGR=1.
	_writeRegister(ILI9225_DISP_CTRL1, 0x0000); // Display off
	_writeRegister(ILI9225_BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
	_writeRegister(ILI9225_FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
	_writeRegister(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
	_writeRegister(ILI9225_OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
	_writeRegister(ILI9225_VCI_RECYCLING, 0x0020); // Set VCI recycling
	_writeRegister(ILI9225_RAM_ADDR_SET1, 0x0000); // RAM Address
	_writeRegister(ILI9225_RAM_ADDR_SET2, 0x0000); // RAM Address

	/* Set GRAM area */
	_writeRegister(ILI9225_GATE_SCAN_CTRL, 0x0000);
	_writeRegister(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB);
	_writeRegister(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000);
	_writeRegister(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000);
	_writeRegister(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB);
	_writeRegister(ILI9225_PARTIAL_DRIVING_POS2, 0x0000);
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF);
	_writeRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000);
	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB);
	_writeRegister(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000);

	/* Set GAMMA curve */
	_writeRegister(ILI9225_GAMMA_CTRL1, 0x0000);
	_writeRegister(ILI9225_GAMMA_CTRL2, 0x0808);
	_writeRegister(ILI9225_GAMMA_CTRL3, 0x080A);
	_writeRegister(ILI9225_GAMMA_CTRL4, 0x000A);
	_writeRegister(ILI9225_GAMMA_CTRL5, 0x0A08);
	_writeRegister(ILI9225_GAMMA_CTRL6, 0x0808);
	_writeRegister(ILI9225_GAMMA_CTRL7, 0x0000);
	_writeRegister(ILI9225_GAMMA_CTRL8, 0x0A00);
	_writeRegister(ILI9225_GAMMA_CTRL9,  0x1000);
	_writeRegister(ILI9225_GAMMA_CTRL10, 0x001f);

	_writeRegister(ILI9225_DISP_CTRL1, 0x0012);
    spi_endTransaction();

	delay(50);

    spi_beginTransaction(CONFIG_GPIO_TFT_CS);
	_writeRegister(ILI9225_DISP_CTRL1, 0x1017);
    spi_endTransaction(); 


	setOrientation(CONFIG_DISPLAY_ROTATION);


}
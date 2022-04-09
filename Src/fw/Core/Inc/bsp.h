#ifndef __BSP_H__
#define __BSP_H__

#include "main.h"
#include "config.h"
//------------------------------ Delay -------------------------------------
void delay_us(int val);

//------------------------------ SPI -------------------------------------
uint8_t spi_read_single(uint8_t adr);
uint8_t spi_write_single(uint8_t adr, uint8_t data);
uint8_t spi_write_cmd(uint8_t cmd);
uint8_t spi_read_burst(uint8_t adr, uint8_t *buf, int16_t len);
uint8_t spi_write_burst(uint8_t adr, uint8_t *buf, int16_t len);
//------------------------------ CC1101 -------------------------------------
#define SPI_CS_LOW() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define SPI_CS_HIGH() HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

#define GDO0 (HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin))
#define GDO2 (HAL_GPIO_ReadPin(GDO2_GPIO_Port, GDO2_Pin))

//------------------------------ I2C -------------------------------------
#define EEROM_WRITE     0xA0
#define EEROM_READ      0xA1
#define EEROM_BSIZE     256
#define EEROM_PAGE      8

int eerom_write(uint8_t adr, uint8_t *buf, uint16_t size);
int eerom_read(uint8_t adr,  uint8_t *buf, uint16_t size);

#pragma pack(push,1)
struct eerom_content_s
{
    union{
        uint32_t d32;
        uint8_t  d8[4];
    }magic;
    union
    {
        uint16_t d16;
        uint8_t d8[2];
    } crc;
    uint8_t co2_acm_en;
    union{
        int16_t d16;
        uint8_t  d8[2];
    }alt_offset;
    uint8_t  temp_offset;
    uint8_t  hm_offset;
    uint8_t  auto_bl_en;
    uint8_t  fix_bl_val;
    uint8_t  dev_addr;
};
#pragma pack(pop)
extern struct eerom_content_s eerom_cfg;

int eerom_read_cfg(struct eerom_content_s *cfg);
int eerom_write_cfg(struct eerom_content_s *cfg);
int eerom_boot_init(struct eerom_content_s *cfg);
void eerom_write_increase_time(unsigned int time);
unsigned int eerom_read_time();

//------------------------------ ADC -------------------------------------
uint16_t Read_ADC1();

//------------------------------ PM25 & CO2 -------------------------------------
#define PM25SET(x)   (HAL_GPIO_WritePin(PM25_SET_GPIO_Port, PM25_SET_Pin, x))
#define PM25RESET(x) (HAL_GPIO_WritePin(PM25_RESET_GPIO_Port, PM25_RESET_Pin, x))

#define CO2_READY (HAL_GPIO_ReadPin(CO2_RDY_GPIO_Port, CO2_RDY_Pin))

void co2_init();
void co2_enable(int enable);
int co2_read_alt();
int co2_setup_alt(int alt);
void co2_auto_calib(int enable);
void co2_calib_reset();
int co2_getdata(float *co2, float *temp, float *hm, int force);

//------------------------------ LCD -------------------------------------
#define LCD_PAGE_INIT   0
#define LCD_PAGE_ALL    1
#define LCD_PAGE_HELP   2
#define LCD_PAGE_INFO   3
#define LCD_PAGE_NUM    4
#define LCD_PAGE_CTRL   5
void sensor_stable(int stable);
void lcd_reset();
void lcd_switch_page(int idx);
void lcd_update_init_page(int val);
void lcd_update_main(int pm25, int co2, int temp, int hm);
void lcd_show_diag_page(float co2, float temp, float hm);
int lcd_show_info_page();
void lcd_set_fix_bl(int bl);
void config_mode();
#endif

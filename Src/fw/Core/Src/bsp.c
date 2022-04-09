#include "bsp.h"

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

int reset_co2_acm = 0;
int factory_reset = 10;
int menu_setup_error = 0;
//------------------------------ System ------------------------------------
int disable_irq()
{
    int irq = __get_PRIMASK();
    __disable_irq();
    return irq;
}

void enable_irq(int irq)
{
    if(irq)
        __enable_irq();
}

int take_lock(int *lock)
{
    int suc = 0;
    int sr = disable_irq();
    if(0 == *lock)
    {
        *lock = 1;
        suc = 1;
    }
    enable_irq(sr);

    return suc;
}

void release_lock(int *lock)
{
    *lock = 0;
}

static uint16_t crc16_mod_bus(const unsigned char *buf, unsigned int len)
{
    static const uint16_t wCRCTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (len--)
    {
        nTemp = *buf++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord ^= wCRCTable[nTemp];
    }
    return wCRCWord;
}


//------------------------------ Delay -------------------------------------
void delay_us(int val)
{
    (&htim1)->Instance->CNT = (0x0000efff - val * 8);
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
    while (((&htim1)->Instance->CNT) < 0x0000effe);
    CLEAR_BIT(TIM1->CR1, TIM_CR1_CEN);
}

//------------------------------ SPI -------------------------------------
uint8_t spi_read_single(uint8_t adr)
{
    uint8_t readval = 0;
    SPI_CS_LOW();
    adr |= 0x80;
    HAL_SPI_Transmit(&hspi1, &adr, 1, -1);
    HAL_SPI_Receive(&hspi1, &readval, 1, -1);
    SPI_CS_HIGH();

    return readval;
}

uint8_t spi_write_single(uint8_t adr, uint8_t data)
{
    uint8_t tx_buf[2];
    SPI_CS_LOW();
    tx_buf[0] = adr & 0x7f;
    tx_buf[1] = data;
    HAL_SPI_Transmit(&hspi1, tx_buf, 2, -1);
    SPI_CS_HIGH();
    return 0;
}

uint8_t spi_write_cmd(uint8_t cmd)
{
    uint8_t status = cmd;
    SPI_CS_LOW();
    HAL_SPI_Receive(&hspi1, &status, 1, -1);
    SPI_CS_HIGH();
    return status;
}

uint8_t spi_read_burst(uint8_t adr, uint8_t *buf, int16_t len)
{
    int i = 0;
    adr |= 0xc0;
    SPI_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &adr, 1, -1);

    for (i = 0; i < len; i++)
        buf[i] = 0;

    HAL_SPI_Receive(&hspi1, buf, len, -1);

    SPI_CS_HIGH();
    return 0;
}

uint8_t spi_write_burst(uint8_t adr, uint8_t *buf, int16_t len)
{
    uint8_t head = ((adr & 0x7f) | 0x40);

    SPI_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &head, 1, -1);
    HAL_SPI_Transmit(&hspi1, buf, len, -1);
    SPI_CS_HIGH();
    return 0;
}

//------------------------------ I2C -------------------------------------
int eerom_read(uint8_t adr, uint8_t *buf, uint16_t size)
{
    int err = 0;
    err = HAL_I2C_Mem_Read(&hi2c1, EEROM_READ, adr, I2C_MEMADD_SIZE_8BIT, buf, size, -1);
    if(HAL_OK == err)
        return 0;
    else
        return -1;
}

int eerom_write(uint8_t adr, uint8_t *buf, uint16_t size)
{
    uint8_t j = 0;
    uint8_t loop = 0;
    uint8_t cnt = 0;
    if((size % EEROM_PAGE) != 0)
    {
        loop = size / EEROM_PAGE;
        cnt = size % EEROM_PAGE;
    }
    else
    {
        loop = size / EEROM_PAGE;
        cnt = 0;
    }

    for(j = 0; j < loop; j++)
    {
        if (HAL_OK != HAL_I2C_Mem_Write(&hi2c1,
                                        EEROM_WRITE,
                                        adr + EEROM_PAGE * j,
                                        I2C_MEMADD_SIZE_8BIT,
                                        buf + EEROM_PAGE * j,
                                        EEROM_PAGE,
                                        -1))
        {
            PrintERR("Write I2C error %d\r\n", j);
            return -1;
        }
        
        HAL_Delay(5);
    }    

    if(cnt != 0)
    {
        if (HAL_OK != HAL_I2C_Mem_Write(&hi2c1,
                                        EEROM_WRITE,
                                        adr + EEROM_PAGE * loop,
                                        I2C_MEMADD_SIZE_8BIT,
                                        buf + EEROM_PAGE * loop,
                                        cnt,
                                        -1))
        {
            PrintERR("Write I2C error %d\r\n", j);
            return -2;
        }

        HAL_Delay(5);
    }
    
    return 0;
}

#define EEROM_MAGIC_WORD    0xC0FEC0FE

int eerom_read_cfg(struct eerom_content_s *cfg)
{
    uint8_t *ptr;
    uint16_t crc_val = 0;
    eerom_read(0, (uint8_t *)cfg, sizeof(struct eerom_content_s));
    if (cfg->magic.d32 != EEROM_MAGIC_WORD)
    {
        return -1;
    }
    ptr = &cfg->co2_acm_en;
    crc_val = crc16_mod_bus(ptr, sizeof(struct eerom_content_s) - 6);
    if(crc_val != cfg->crc.d16)
    {
        return -2;
    }

    return 0;
}


int eerom_write_cfg(struct eerom_content_s *cfg)
{
    uint8_t *ptr1, *ptr2;
    struct eerom_content_s cfg_back;
    int i = 0;
    ptr1 = &cfg->co2_acm_en;

    uint16_t crc_calc = crc16_mod_bus(ptr1, sizeof(struct eerom_content_s) - 6);
    cfg->crc.d16 = crc_calc;
    cfg->magic.d32 = EEROM_MAGIC_WORD;

    eerom_write(0, (uint8_t *)cfg, sizeof(struct eerom_content_s));
    HAL_Delay(20);
    eerom_read(0, (uint8_t *)&cfg_back, sizeof(struct eerom_content_s));
    ptr1 = (uint8_t *)cfg;
    ptr2 = (uint8_t *)&cfg_back;

    for(i = 0; i < sizeof(struct eerom_content_s); i++)
    {
        if(ptr1[i] != ptr2[i])
            return -1;
    }

    return 0;
}

int eerom_boot_init(struct eerom_content_s *cfg)
{
    int err = 0;
    uint8_t *ptr;
    uint16_t crc_val;
    err = eerom_read_cfg(cfg);
    if (err < 0)
    {
        //eerom broken - go back to facotry
        cfg->magic.d32      = EEROM_MAGIC_WORD;
        cfg->alt_offset.d16 = CONFIG_BASE_CO2_ALT_OFFSET;
        cfg->co2_acm_en     = 0;
        cfg->temp_offset    = 0;
        cfg->hm_offset      = 0;
        cfg->auto_bl_en     = 1;
        cfg->fix_bl_val     = 0;
        cfg->dev_addr       = 0;
        ptr = &cfg->co2_acm_en;
        crc_val = crc16_mod_bus(ptr, sizeof(struct eerom_content_s) - 6);
        cfg->crc.d16 = crc_val;
        eerom_write_cfg(cfg);

        int saved_time = 0;
        HAL_Delay(20);
        eerom_write(252, (uint8_t *)&saved_time, 4);
        HAL_Delay(20);

#ifdef CONFIG_ENALBE_CO2
        co2_setup_alt(CONFIG_BASE_CO2_ALT_OFFSET);
#endif
    }

    return err;
}

void eerom_write_increase_time(unsigned int time)
{
    unsigned int saved_time = 0;
    eerom_read(252, (uint8_t *)&saved_time, 4);
    saved_time += time;
    eerom_write(252, (uint8_t *)&saved_time, 4);
    HAL_Delay(20);
}

unsigned int eerom_read_time()
{
    unsigned int saved_time = 0;
    eerom_read(252, (uint8_t *)&saved_time, 4);
    return saved_time;
}

uint16_t Read_ADC1()
{
    uint16_t AD_Value = 0;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, -1);
    AD_Value = HAL_ADC_GetValue(&hadc1);
    return AD_Value;
}

//------------------------------ CO2 -------------------------------------
const uint8_t co2_trigger[]         = {0x61, 0x06, 0x00, 0x36, 0x00, 0x00, 0x60, 0x64};
const uint8_t co2_stop[]            = {0x61, 0x06, 0x00, 0x37, 0x00, 0x01, 0xf0, 0x64};
const uint8_t co2_read[]            = {0x61, 0x03, 0x00, 0x28, 0x00, 0x06, 0x4c, 0x60};
const uint8_t co2_calib_disable[]   = {0x61, 0x06, 0x00, 0x3a, 0x00, 0x00, 0xa0, 0x67};
const uint8_t co2_calib_enable[]    = {0x61, 0x06, 0x00, 0x3a, 0x00, 0x01, 0x61, 0xa7};
const uint8_t co2_calib_frc_reset[] = {0x61, 0x06, 0x00, 0x39, 0x01, 0x90, 0x51, 0x9b};
const uint8_t co2_get_alt[]         = {0x61, 0x03, 0x00, 0x38, 0x00, 0x01, 0x0c, 0x67};
uint8_t co2_set_alt[]               = {0x61, 0x06, 0x00, 0x38, 0x55, 0xAA, 0x00, 0x00};

void co2_init()
{
    uint8_t co2_ack[20];
    HAL_Delay(1000);
    HAL_UART_Receive(&huart3, co2_ack, 20, 1);
}

void co2_enable(int enable)
{
    uint8_t co2_ack[10];
    if(enable)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)&co2_trigger, 8, 2000);
        HAL_UART_Receive(&huart3, co2_ack, 8, 2000);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)&co2_stop, 8, 2000);
        HAL_UART_Receive(&huart3, co2_ack, 8, 2000);
    }
}

int co2_read_alt()
{
    int alt = 0;
    uint8_t co2_ack[8];
    HAL_UART_Transmit(&huart3, (uint8_t *)&co2_get_alt, 8, 2000);
    HAL_UART_Receive(&huart3, co2_ack, 7, 2000);
    alt = (co2_ack[3] << 8) | co2_ack[4];
    return alt;
}

int co2_setup_alt(int alt)
{
    uint16_t crc = 0;
    uint8_t co2_ack[10];
    co2_set_alt[4]  = (alt >> 8) & 0x00ff;
    co2_set_alt[5]  = alt & 0x00ff;
    crc = crc16_mod_bus(co2_set_alt, 6);
    co2_set_alt[6]  = crc & 0x00ff;
    co2_set_alt[7]  = (crc >> 8) & 0x00ff;
    HAL_UART_Transmit(&huart3, (uint8_t *)&co2_set_alt, 8, 2000);
    HAL_UART_Receive(&huart3, co2_ack, 8, 2000);

    return 0;
}

void co2_auto_calib(int enable)
{
    uint8_t co2_ack[10];
    if(enable)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)&co2_calib_enable, 8, 2000);
        HAL_UART_Receive(&huart3, co2_ack, 8, 2000);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)&co2_calib_disable, 8, 2000);
        HAL_UART_Receive(&huart3, co2_ack, 8, 2000);
    }
}

void co2_calib_reset()   //reset calib to 400ppm
{
    uint8_t co2_ack[10];
    HAL_UART_Transmit(&huart3, (uint8_t *)&co2_calib_frc_reset, 8, 2000);
    HAL_UART_Receive(&huart3, co2_ack, 8, 2000);
}


int co2_getdata(float *co2, float *temp, float *hm, int force)
{
    uint8_t co2_ack[17];
    int crc_chk;
    int get_crc = 0;
    if(CO2_READY || force)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)&co2_read, 8, 2000);
        HAL_UART_Receive(&huart3, co2_ack, 17, 2000);
        if ((co2_ack[0] == 0x61) && (co2_ack[1] == 0x03) && (co2_ack[2] == 0x0c))
        {
            crc_chk = crc16_mod_bus(co2_ack, 15);
            get_crc = co2_ack[15] | (co2_ack[16] << 8);
            if (crc_chk == get_crc)
            {
                uint32_t co2_val = 0, temp_val = 0, hum_val = 0;
                co2_val = (co2_ack[3] << 24) | (co2_ack[4] << 16) | (co2_ack[5] << 8) | co2_ack[6];
                temp_val = (co2_ack[7] << 24) | (co2_ack[8] << 16) | (co2_ack[9] << 8) | co2_ack[10];
                hum_val = (co2_ack[11] << 24) | (co2_ack[12] << 16) | (co2_ack[13] << 8) | co2_ack[14];
                *co2 = *(float *)&co2_val;
                *temp = *(float *)&temp_val;
                *hm = *(float *)&hum_val;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            return -2;
        }
    }
    //else
    //{
    //    return -3;
    //}

    return 0;
}

//------------------------------ LCD -------------------------------------
void int2string(int val, int hexmode, char *buf, int *len)
{
    int i = 0;
    int max_len = *len;
    int negative = 0;
    if(0 == hexmode)
    {
        if(val == 0)
        {
            buf[0] = '0';
            *len = 1;
            return;
        }

        if(val < 0)
        {
            buf[0] = '-';
            val = 0 - val;
            negative = 1;
        }

        for (i = negative; i < max_len; i++)
        {
            if(val == 0)
            {
                break;
            }

            buf[i] = val % 10 + 0x30;
            val = val / 10;
        }
        *len = i;

        uint8_t temp = 0;
        for (i = 0; i < ((*len - negative) / 2); i++)
        {
            temp = buf[i + negative];
            buf[i + negative] = buf[*len - 1 - i];
            buf[*len - 1 - i] = temp;
        }

    }
    else
    {
        buf[0]='0';
        buf[1]='x';
        uint8_t temp = 28;
        int start_flag = 0;
        int idx = 2;
        for(i = 0; i < 8; i++)
        {
            if(idx >= max_len)
                return;
            
            uint8_t get_val = (val >> temp) & 0x0000000f;
            temp -= 4;
            if(get_val != 0)
                start_flag = 1;
                
            if(0 == start_flag)
                continue;

            buf[idx++] = (get_val >= 10) ? (get_val - 10 + 'a') : (get_val + '0');
        }

        if(0 == start_flag)
        {
            buf[2] = '0';
            *len = 3;
        }
        else
        {
            *len = idx;
        }

    }

}

void lcd_reset()
{
    const char lcd_reset[] = "rest";
    HAL_UART_Transmit(&huart2, (uint8_t *)lcd_reset, sizeof(lcd_reset) - 1, 2000);

    const uint8_t end_marker[] = {0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}

void lcd_switch_page(int idx)
{
    switch(idx)
    {
        case LCD_PAGE_INIT:
        {
            const char page_init[] = "page page_glb";
            HAL_UART_Transmit(&huart2, (uint8_t *)page_init, sizeof(page_init) - 1, 2000);
            break;
        }
        case LCD_PAGE_ALL :
        {
            const char page_all[] = "page page_all";
            HAL_UART_Transmit(&huart2, (uint8_t *)page_all, sizeof(page_all) - 1, 2000);
            break;
        }
        case LCD_PAGE_HELP:
        {
            const char page_help[] = "page page_help";
            HAL_UART_Transmit(&huart2, (uint8_t *)page_help, sizeof(page_help) - 1, 2000);
            break;
        }
        case LCD_PAGE_INFO:
        {
            const char page_info[] = "page page_info";
            HAL_UART_Transmit(&huart2, (uint8_t *)page_info, sizeof(page_info) - 1, 2000);
            break;
        }
        case LCD_PAGE_NUM :
        {
            const char page_num[] = "page page_num";
            HAL_UART_Transmit(&huart2, (uint8_t *)page_num, sizeof(page_num) - 1, 2000);
            break;
        }
        case LCD_PAGE_CTRL:
        {
            const char page_ctrl[] = "page page_ctrl";
            HAL_UART_Transmit(&huart2, (uint8_t *)page_ctrl, sizeof(page_ctrl) - 1, 2000);
            break;
        }
    }

    const uint8_t end_marker[] = {0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}

static void lcd_set_cmd(const char *cmd)
{
    const uint8_t end_marker[] = {0xff, 0xff, 0xff};
    int lable_len = 0;
    int i = 0;
    for (i = 0; i < 1024; i++)
    {
        if (cmd[i] == 0)
            break;
    }
    lable_len = i;

    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, lable_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}

static void lcd_set_value(const char *lable, int value, int hexmode)
{
    int lable_len = 0;
    int i = 0;
    for(i = 0; i < 1024; i++)
    {
        if(lable[i] == 0)
            break;
    }
    lable_len = i;

    char value_string[12];
    int  value_str_len = 11;
    const uint8_t end_marker[] = {0xff, 0xff, 0xff};

    int2string(value, hexmode, value_string, &value_str_len);

    HAL_UART_Transmit(&huart2, (uint8_t *)lable, lable_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)value_string, value_str_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}

static void lcd_set_txt_value(const char *lable, int value, int hexmode)
{
    const uint8_t quote = '\"';
    int lable_len = 0;
    int i = 0;
    for (i = 0; i < 1024; i++)
    {
        if (lable[i] == 0)
            break;
    }
    lable_len = i;

    char value_string[12];
    int value_str_len = 11;
    const uint8_t end_marker[] = {0xff, 0xff, 0xff};

    int2string(value, hexmode, value_string, &value_str_len);

    HAL_UART_Transmit(&huart2, (uint8_t *)lable, lable_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)&quote, 1, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)value_string, value_str_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)&quote, 1, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}

static void lcd_set_txt_float(const char *lable, int symbol, unsigned int value1, unsigned int value2)
{
    const uint8_t quote = '\"';
    int lable_len = 0;
    int i = 0;
    for (i = 0; i < 1024; i++)
    {
        if (lable[i] == 0)
            break;
    }
    lable_len = i;

    char value1_string[12];
    int value1_str_len = 11;
    char value2_string[12];
    int value2_str_len = 11;
    const uint8_t end_marker[] = {0xff, 0xff, 0xff};
    const uint8_t pointer = '.';
    int2string(value1, 0, value1_string, &value1_str_len);
    int2string(value2, 0, value2_string, &value2_str_len);
    HAL_UART_Transmit(&huart2, (uint8_t *)lable, lable_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)&quote, 1, 2000);
    if(symbol < 0)
    {
        const uint8_t negative_symbol = '-';
        HAL_UART_Transmit(&huart2, (uint8_t *)&negative_symbol, 1, 2000);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)value1_string, value1_str_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)&pointer, 1, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)value2_string, value2_str_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)&quote, 1, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}

static void lcd_set_chinese(const char *lable, uint16_t *str, int len)
{
    const uint8_t quote = '\"';
    int lable_len = 0;
    int i = 0;

    for (i = 0; i < 1024; i++)
    {
        if (lable[i] == 0)
            break;
    }
    lable_len = i;

    HAL_UART_Transmit(&huart2, (uint8_t *)lable, lable_len, 2000);
    HAL_UART_Transmit(&huart2, (uint8_t *)&quote, 1, 2000);

    for(i = 0; i < len; i++)
    {
        uint8_t char_buf[2];
        char_buf[0] = (str[i] >> 8) & 0x00ff;
        char_buf[1] = str[i] & 0x00ff;
        HAL_UART_Transmit(&huart2, (uint8_t *)char_buf, 2, 2000);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)&quote, 1, 2000);

    const uint8_t end_marker[] = {0xff, 0xff, 0xff};
    HAL_UART_Transmit(&huart2, (uint8_t *)end_marker, 3, 2000);
}


void lcd_update_init_page(int val)
{
    lcd_set_value("j0.val=", val, 0);
}

static void lcd_update_backlight(int enable)
{
    volatile uint16_t getval = 0;
    float light_vol = 0.0;
    uint16_t dim_vol = 50;
    float calc = 0.0;

    if(enable)
    {
        getval = Read_ADC1();
        light_vol = ((float)(getval * 3.3) / 4095);
        if (light_vol < 0.03)
        {
            dim_vol = 30;
        }
        else if ((light_vol >= 0.03) && (light_vol < 1.2))
        {
            calc = (light_vol - 0.03) * 81.9 + 30;
            dim_vol = (uint16_t)calc;
            if (dim_vol > 100)
                dim_vol = 100;
        }
        else
        {
            dim_vol = 100;
        }
        PrintDBG("ADC=%.1f, dim=%d\r\n", light_vol, dim_vol);
        lcd_set_value("dim=", dim_vol, 0);
    }

}

void lcd_set_fix_bl(int bl)
{
    if(bl < 0)
        bl = 0;
    if(bl > 100)
        bl = 100;
    lcd_set_value("dim=", bl, 0);
}

#define AQ_LEVEL_EXCELLENT  0   //you
#define AQ_LEVEL_GOOD       1   //liang
#define AQ_LEVEL_FAIR       2   //qing du
#define AQ_LEVEL_NO_GOOD    3   //zhong du
#define AQ_LEVEL_BAD        4   //zhong du
#define AQ_LEVEL_VERY_BAD   5   //ji du
#define AQ_LEVEL_RUN        6   //tao ming
#define AQ_LEVEL_WAIT       7   //dai wen ding

static int lcd_pm25_leveler_color(int pm25)
{
    int lvl = 0;
    if(pm25 <= 12)
    {
        lvl = AQ_LEVEL_EXCELLENT; //Excellent
        lcd_set_cmd("n0.bco=3333"); 
        lcd_set_cmd("t3.bco=3333");
    }
    else if((pm25 > 12) && (pm25 <= 35))
    {
        lvl = AQ_LEVEL_GOOD;
        lcd_set_cmd("n0.bco=65088");
        lcd_set_cmd("t3.bco=65088");
    }
    else if ((pm25 > 35) && (pm25 <= 55))
    {
        lvl = AQ_LEVEL_FAIR;
        lcd_set_cmd("n0.bco=58649");
        lcd_set_cmd("t3.bco=58649");
    }
    else if ((pm25 > 55) && (pm25 <= 150))
    {
        lvl = AQ_LEVEL_NO_GOOD;
        lcd_set_cmd("n0.bco=55555");
        lcd_set_cmd("t3.bco=55555");
    }
    else if ((pm25 > 150) && (pm25 <= 250))
    {
        lvl = AQ_LEVEL_BAD;
        lcd_set_cmd("n0.bco=38931");
        lcd_set_cmd("t3.bco=38931");
    }
    else if ((pm25 > 250) && (pm25 <= 350))
    {
        lvl = AQ_LEVEL_VERY_BAD;
        lcd_set_cmd("n0.bco=30724");
        lcd_set_cmd("t3.bco=30724");
    }
    else
    {
        lvl = AQ_LEVEL_RUN;
        lcd_set_cmd("n0.bco=0");
        lcd_set_cmd("t3.bco=0");
    }

    return lvl;
}

static int lcd_co2_leveler_color(int co2)
{
    int lvl = 0;
    if(co2 <= 600)
    {
        lvl = AQ_LEVEL_EXCELLENT;
        lcd_set_cmd("n1.bco=9545");
        lcd_set_cmd("t0.bco=9545");
    }
    else if((co2 > 600) && (co2 <= 800))
    {
        lvl = AQ_LEVEL_GOOD;
        lcd_set_cmd("n1.bco=13436");
        lcd_set_cmd("t0.bco=13436");
    }
    else if ((co2 > 800) && (co2 <= 1000))
    {
        lvl = AQ_LEVEL_FAIR;
        lcd_set_cmd("n1.bco=64864");
        lcd_set_cmd("t0.bco=64864");
    }
    else if ((co2 > 1000) && (co2 <= 2000))
    {
        lvl = AQ_LEVEL_BAD;
        lcd_set_cmd("n1.bco=64008");
        lcd_set_cmd("t0.bco=64008");
    }
    else if ((co2 > 2000) && (co2 <= 5000))
    {
        lvl = AQ_LEVEL_VERY_BAD;
        lcd_set_cmd("n1.bco=30735");
        lcd_set_cmd("t0.bco=30735");
    }
    else
    {
        lvl = AQ_LEVEL_RUN;
        lcd_set_cmd("n1.bco=0");
        lcd_set_cmd("t0.bco=0");
    }

    return lvl;
}

/* Indicator Matrix
        CO2         EXCELLENT   GOOD    FAIR    BAD     VERY_BAD  RUN
PM25    EXCELLENT   极佳        优      良       通风     强通风    逃命
        GOOD        良          良      良       通风     强通风    逃命
        FAIR        轻度        轻度    轻度      通风     强通风    逃命
        NO_GOOD     中度        中度    中度      通风     强通风    逃命
        BAD         重度        重度    重度      短通风   短通风    逃命
        VERY_BAD    极度        极度    极度      极度     撤离      逃命
        RUN         逃命        逃命    逃命      逃命     逃命      逃命
*/
static void lcd_update_indicator(int pm25_level, int co2_level)
{
    uint16_t str_buf[5];
    int str_len = 0;

    if((co2_level == AQ_LEVEL_RUN) || (pm25_level == AQ_LEVEL_RUN))
    {
        str_buf[0] = 0xCCD3;    //逃命
        str_buf[1] = 0xC3FC;
        str_len = 2;
    }
    else if((co2_level == AQ_LEVEL_WAIT) || (pm25_level == AQ_LEVEL_WAIT))
    {
        str_buf[0] = 0xB4FD;    //待稳定
        str_buf[1] = 0xCEC8;
        str_buf[2] = 0xB6A8;
        str_len = 3;
    }
    else
    {
        if (co2_level == AQ_LEVEL_BAD)
        {
            if (pm25_level < AQ_LEVEL_BAD)
            {
                //str_buf[0] = 0xC7BF; //强
                str_buf[0] = 0xCDA8; //通风
                str_buf[1] = 0xB7E7;
                str_len = 2;
            }
            else if (pm25_level == AQ_LEVEL_BAD)
            {
                str_buf[0] = 0xB6CC; //短
                str_buf[1] = 0xCDA8; //通风
                str_buf[2] = 0xB7E7;
                str_len = 3;
            }
            else if (pm25_level == AQ_LEVEL_VERY_BAD)
            {
                str_buf[0] = 0xBCAB; //极差
                str_buf[1] = 0xB2EE;
                str_len = 2;
            }
        }
        else if (co2_level == AQ_LEVEL_VERY_BAD)
        {
            if (pm25_level < AQ_LEVEL_BAD)
            {
                str_buf[0] = 0xC7BF; //强
                str_buf[1] = 0xCDA8; //通风
                str_buf[2] = 0xB7E7;
                str_len = 3;
            }
            else if (pm25_level == AQ_LEVEL_BAD)
            {
                str_buf[0] = 0xB6CC; //短
                str_buf[1] = 0xCDA8; //通风
                str_buf[2] = 0xB7E7;
                str_len = 3;
            }
            else if (pm25_level == AQ_LEVEL_VERY_BAD)
            {
                str_buf[0] = 0xC3B7; //撤离
                str_buf[1] = 0xC0EB;
                str_len = 2;
            }
        }
        else
        {
            switch(pm25_level)
            {
                case AQ_LEVEL_EXCELLENT:
                    if (co2_level >= AQ_LEVEL_FAIR)
                    {
                        str_buf[0] = 0xC1BC; //liang
                        str_len = 1;
                    }
                    else if(co2_level == AQ_LEVEL_EXCELLENT)
                    {
                        str_buf[0] = 0xBCAB; //jijia
                        str_buf[1] = 0xBCD1;
                        str_len = 2;
                    }
                    else
                    {
                        str_buf[0] = 0xD3C5; //you
                        str_len = 1;
                    }
                    break;
                case AQ_LEVEL_GOOD:
                    str_buf[0] = 0xC1BC;    //liang
                    str_len = 1;
                    break;
                case AQ_LEVEL_FAIR:
                    str_buf[0] = 0xC7E1;    //qing du
                    str_buf[1] = 0xB6C8;
                    str_len = 2;
                    break;
                case AQ_LEVEL_NO_GOOD:
                    str_buf[0] = 0xD6D0;    //zhogn du
                    str_buf[1] = 0xB6C8;
                    str_len = 2;
                    break;
                case AQ_LEVEL_BAD:
                    str_buf[0] = 0xD6D8;    //zhong du
                    str_buf[1] = 0xB6C8;
                    str_len = 2;
                    break;
                case AQ_LEVEL_VERY_BAD :
                    str_buf[0] = 0xBCAB; //极差
                    str_buf[1] = 0xB2EE;
                    str_len = 2;
                    break;
            }
        }
    }

    if(str_len == 0)
    {
        str_buf[0] = 0xB4ED;
        str_buf[1] = 0xCEF3;
        str_len = 2;
    }

    lcd_set_chinese("t5.txt=", str_buf, str_len);
}

int sensor_stabled = 0;
void sensor_stable(int stable)
{
    if(stable)
        sensor_stabled = 1;
    
}

void lcd_update_main(int pm25, int co2, int temp, int hm)
{
    int pm25_lvl = 0;
    int co2_lvl = 0;

    if(eerom_cfg.auto_bl_en)
        lcd_update_backlight(1);   
    lcd_set_cmd("ref_stop");
    lcd_set_value("n0.val=", pm25, 0);
    lcd_set_value("n1.val=", co2, 0);
    lcd_set_value("n2.val=", temp, 0);
    lcd_set_value("n3.val=", hm, 0);

    lcd_set_cmd("vis t0,1");
    lcd_set_cmd("vis t1,1");
    lcd_set_cmd("vis t2,1");
    lcd_set_cmd("vis t3,1");        

    pm25_lvl = lcd_pm25_leveler_color(pm25);
    co2_lvl = lcd_co2_leveler_color(co2);
    if (0 == sensor_stabled)
    {
        pm25_lvl = AQ_LEVEL_WAIT;
        co2_lvl = AQ_LEVEL_WAIT;
    }

    extern int system_error;
    if(system_error)
    {
        uint16_t str_buf[3];
        int str_len = 0;
        str_buf[0] = 0xB4ED; //错误
        str_buf[1] = 0xCEF3;
        str_len = 2;
        lcd_set_chinese("t5.txt=", str_buf, str_len);
    }
    else
    {
        lcd_update_indicator(pm25_lvl, co2_lvl);
    }

    lcd_set_cmd("ref_star");
}

int lcd_show_info_page()
{
    uint16_t str_buf[2] = {0xD0C5, 0xCFA2}; //xin xi
    lcd_switch_page(LCD_PAGE_INFO);
    lcd_set_chinese("t1.txt=", str_buf, 2);
    lcd_set_cmd("t0.txt=\"Version:\"");
    lcd_set_cmd("t2.txt=\"HW:RevA/FW:v1.0\"");
    lcd_set_cmd("t3.txt=\"Hours:\"");
    //lcd_set_cmd("t4.txt=\"v1.0\"");
    unsigned int read_time = 0;
    read_time = eerom_read_time();
    read_time *= 10;
    read_time /= 60;
    lcd_set_txt_value("t4.txt=", read_time, 0);

    lcd_set_cmd("t5.txt=\"Build:\"");
    lcd_set_cmd("t6.txt=\"Tiger/02\"");
    lcd_set_cmd("t7.txt=\"PicoNet:\"");
#ifdef CONFIG_ENABLE_PCN
    lcd_set_cmd("t8.txt=\"Enable\"");
#else
    lcd_set_cmd("t8.txt=\"Disable\"");
#endif

    lcd_set_cmd("t9.txt=\"ErrorStatus:\"");
    extern int system_error;
    lcd_set_txt_value("t10.txt=", system_error, 1);
    return 0;
}

int lcd_show_help()
{
    lcd_switch_page(LCD_PAGE_HELP);

    return 0;
}

int lcd_show_diag_mode()
{
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_CTRL);
    lcd_set_cmd("t1.txt=\"DIAG MODE\"");
    lcd_set_cmd("t0.txt=\"DIAG OFF:\"");
    lcd_set_cmd("r0.val=0");
    lcd_set_cmd("ref_star");
    return 0;
}

int lcd_update_diag_mode(int orig, int btn_up, int btn_down)
{
    extern int diag_mode;

    if (btn_up == btn_down)
        return 0;

    if(btn_up)
    {
        lcd_set_cmd("t0.txt=\"DIAG ON:\"");
        lcd_set_cmd("r0.val=1");
        diag_mode = 1;
        return 1;
    }
    else
    {
        lcd_set_cmd("t0.txt=\"DIAG OFF:\"");
        lcd_set_cmd("r0.val=0");
        diag_mode = 0;
        return 0;
    }

    return 0;
}

void lcd_show_diag_page(float co2, float temp, float hm)
{
    int val = 0;
    uint16_t str_buf[2] = {0xBCEC, 0xB2E2}; //jian ce
    lcd_switch_page(LCD_PAGE_INFO);
    lcd_set_chinese("t1.txt=", str_buf, 2);

    extern int system_error;
    if(system_error)
    {
        lcd_set_cmd("t0.txt=\"Err Flag:\"");
        lcd_set_txt_value("t2.txt=", system_error, 1);
    }
    else
    {
        lcd_set_cmd("t0.txt=\"PM25 RAW:\"");
        extern volatile int pm25_value;
        lcd_set_txt_value("t2.txt=", pm25_value, 0);
    }

    lcd_set_cmd("t3.txt=\"PM25 Count:\"");
    extern int uart2_irq_count;
    lcd_set_txt_value("t4.txt=", uart2_irq_count, 0);

    lcd_set_cmd("t5.txt=\"CO2 RAW:\"");
    val = (int)(co2 * 10.0+0.5);
    lcd_set_txt_float("t6.txt=", 0, (int)co2, val % 10);

    lcd_set_cmd("t7.txt=\"Temp RAW:\"");
    val = (int)(temp * 10.0 + 0.5);
    lcd_set_txt_float("t8.txt=", 0, (int)temp, val % 10);

    lcd_set_cmd("t9.txt=\"HM RAW:\"");
    val = (int)(hm * 10.0 + 0.5);
    lcd_set_txt_float("t10.txt=", 0, (int)hm, val % 10);
}

int lcd_show_co2_acm()
{
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_CTRL);
    lcd_set_cmd("t1.txt=\"CO2 AMS\"");
    if (eerom_cfg.co2_acm_en)
    {
        lcd_set_cmd("t0.txt=\"AMS ON:\"");
        lcd_set_cmd("r0.val=1");
    }
    else
    {
        lcd_set_cmd("t0.txt=\"AMS OFF:\"");
        lcd_set_cmd("r0.val=0");
    }
    lcd_set_cmd("ref_star");

    return eerom_cfg.co2_acm_en;
}


int lcd_update_co2_acm(int orig, int btn_up, int btn_down)
{
    if(btn_up == btn_down)
        return 0;
    if(btn_up)
    {
        lcd_set_cmd("t0.txt=\"AMS ON:\"");
        lcd_set_cmd("r0.val=1");
        if(orig != 1)
        {
            eerom_cfg.co2_acm_en = 1;
            return 1;
        }
    }
    else
    {
        lcd_set_cmd("t0.txt=\"AMS OFF:\"");
        lcd_set_cmd("r0.val=0");
        if(orig != 0)
        {
            eerom_cfg.co2_acm_en = 0;
            return 1;
        }
    }

    return 0;
}

void lcd_set_co2_acm()
{
    int err = 0;
    PrintDBG("Setup CO2 ACM!!\r\n");
    if(eerom_cfg.co2_acm_en)
        co2_auto_calib(1);
    else
        co2_auto_calib(0);

    err = eerom_write_cfg(&eerom_cfg);
    if(err < 0)
    {
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }
}

int lcd_show_co2_acm_reset()
{
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_CTRL);
    lcd_set_cmd("t1.txt=\"RESET AMS\"");

    lcd_set_cmd("t0.txt=\"No Reset:\"");
    lcd_set_cmd("r0.val=0");

    lcd_set_cmd("ref_star");
    return 0;
}

int lcd_update_co2_acm_reset(int orig, int btn_up, int btn_down)
{
    if (btn_up == btn_down)
        return 0;
    if (btn_up)
    {
        lcd_set_cmd("t0.txt=\"Reset:\"");
        lcd_set_cmd("r0.val=1");
        reset_co2_acm = 1;
        return 1;
    }
    else
    {
        lcd_set_cmd("t0.txt=\"No Reset:\"");
        lcd_set_cmd("r0.val=0");
        reset_co2_acm = 0;
        return 0;
    }

}

void lcd_set_co2_acm_reset()
{
    PrintDBG("Setup CO2 ACM RESET!!\r\n");
    if(reset_co2_acm)
        co2_calib_reset();
}

int lcd_show_co2_alt()
{
    uint16_t str_buf[2] = {0xBAA3, 0xB0CE}; //hai ba
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_NUM);
    lcd_set_chinese("t1.txt=", str_buf, 2);

    lcd_set_cmd("t0.txt=\"Altitude:\"");
    
    //int value = (int16_t)eerom_cfg.alt_offset.d16;
    int value = co2_read_alt();
    eerom_cfg.alt_offset.d16 = value;
    lcd_set_txt_value("t2.txt=", value, 0);

    lcd_set_cmd("ref_star");

    return value;
}

int lcd_modify_co2_alt(int orig, int btn_up, int btn_down)
{
    int16_t new_val = (int16_t)eerom_cfg.alt_offset.d16;
    if (btn_up == btn_down)
        return 0;
    
    if(btn_up)
    {
        new_val++;
    }
    else
    {
        new_val--;
    }
    int show_val = (int16_t)new_val;
    lcd_set_txt_value("t2.txt=", show_val, 0);
    eerom_cfg.alt_offset.d16 = new_val;
    if(new_val == orig)
        return 0;
    else
        return 1;
}

void lcd_setup_co2_alt()
{
    PrintDBG("Setup CO2 Alt offset\r\n");
    co2_setup_alt((int)eerom_cfg.alt_offset.d16);
}

int lcd_show_temp_adj()
{
    uint16_t str_buf[4] = {0xCEC2, 0xB6C8, 0xD0A3, 0xD7BC}; 
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_NUM);
    lcd_set_chinese("t1.txt=", str_buf, 4);

    lcd_set_cmd("t0.txt=\"Offset:\"");

    int negative = ((eerom_cfg.temp_offset & 0x80) == 0) ? 1 : -1;
    int temp_val = eerom_cfg.temp_offset & 0x7f;
    unsigned int msb_val = (int)(temp_val / 10);
    unsigned int lsb_val = temp_val % 10;

    lcd_set_txt_float("t2.txt=", negative, msb_val, lsb_val);
    lcd_set_cmd("ref_star");

    return (uint8_t)eerom_cfg.temp_offset;
}

int lcd_modify_temp_adj(int orig, int btn_up, int btn_down)
{

    if (btn_up == btn_down)
        return 0;

    int symbol = ((eerom_cfg.temp_offset & 0x80) == 0) ? 1 : -1;
    int temp_val = eerom_cfg.temp_offset & 0x7f;
    if(btn_up)
    {
        if(temp_val == 0)
        {
            temp_val = 1;
            symbol = 1;
        }
        else
        {
            if (symbol > 0)
            {
                if (temp_val < 0x7f)
                {
                    temp_val++;
                }
            }
            else
            {
                if (temp_val > 0)
                    temp_val--;
            }
        }

    }
    else
    {
        if(temp_val == 0)
        {
            temp_val = 1;
            symbol = -1;
        }
        else
        {
            if (symbol > 0)
            {
                if (temp_val > 0)
                    temp_val--;
            }
            else
            {
                if (temp_val < 0x7f)
                {
                    temp_val++;
                }
            }
        }

    }

    unsigned int msb_val = (int)(temp_val / 10);
    unsigned int lsb_val = temp_val % 10;

    eerom_cfg.temp_offset = (symbol > 0) ? (uint8_t)temp_val : (uint8_t)temp_val | 0x80;

    lcd_set_txt_float("t2.txt=", symbol, msb_val, lsb_val);

    uint8_t orig_val = (uint8_t)orig;
    if (orig_val == (uint8_t)eerom_cfg.temp_offset)
        return 0;
    else
        return 1;
}

void lcd_update_temp_adj()
{
    int err = 0;
    PrintDBG("Update Temp Adj Value\r\n");

    err = eerom_write_cfg(&eerom_cfg);
    if (err < 0)
    {
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }

}

int lcd_show_hm_adj()
{
    uint16_t str_buf[4] = {0xCAAA, 0xB6C8, 0xD0A3, 0xD7BC};
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_NUM);
    lcd_set_chinese("t1.txt=", str_buf, 4);

    lcd_set_cmd("t0.txt=\"Offset:\"");

    int negative = ((eerom_cfg.hm_offset & 0x80) == 0) ? 1 : -1;
    int temp_val = eerom_cfg.hm_offset & 0x7f;
    temp_val *= negative;

    lcd_set_txt_value("t2.txt=", temp_val, 0);
    lcd_set_cmd("ref_star");

    return (uint8_t)eerom_cfg.hm_offset;
}

int lcd_modify_hm_adj(int orig, int btn_up, int btn_down)
{

    if (btn_up == btn_down)
        return 0;

    int symbol = ((eerom_cfg.hm_offset & 0x80) == 0) ? 1 : -1;
    int temp_val = eerom_cfg.hm_offset & 0x7f;
    if (btn_up)
    {
        if (temp_val == 0)
        {
            temp_val = 5;
            symbol = 1;
        }
        else
        {
            if (symbol > 0)
            {
                if (temp_val < 100)
                {
                    temp_val+=5;
                }
            }
            else
            {
                if (temp_val > 0)
                    temp_val-=5;
            }
        }
    }
    else
    {
        if (temp_val == 0)
        {
            temp_val = 5;
            symbol = -1;
        }
        else
        {
            if (symbol > 0)
            {
                if (temp_val > 0)
                    temp_val-=5;
            }
            else
            {
                if (temp_val < 100)
                {
                    temp_val+=5;
                }
            }
        }
    }

    int show_value = temp_val * symbol;
    eerom_cfg.hm_offset = (symbol > 0) ? temp_val : (temp_val | 0x80);
    lcd_set_txt_value("t2.txt=", show_value, 0);

    uint8_t orig_val = (uint8_t)orig;
    if (orig_val == (uint8_t)eerom_cfg.hm_offset)
        return 0;
    else
        return 1;
}

void lcd_update_hm_adj()
{
    int err = 0;
    PrintDBG("Update HM Adj Value\r\n");
    err = eerom_write_cfg(&eerom_cfg);
    if (err < 0)
    {
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }
}


int lcd_show_auto_bl()
{
    uint16_t str_buf[4] = {0xD7D4, 0xB6AF, 0xB1B3, 0xB9E2};
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_CTRL);
    lcd_set_chinese("t1.txt=", str_buf, 4);

    if (eerom_cfg.auto_bl_en)
    {
        lcd_set_cmd("t0.txt=\"ON\"");
        lcd_set_cmd("r0.val=1");
    }
    else
    {
        lcd_set_cmd("t0.txt=\"OFF\"");
        lcd_set_cmd("r0.val=0");
    }

    lcd_set_cmd("ref_star");

    return eerom_cfg.auto_bl_en;
}

int lcd_modify_auto_bl(int orig, int btn_up, int btn_down)
{
    if (btn_up == btn_down)
        return 0;
    
    if(btn_up)
    {
        lcd_set_cmd("t0.txt=\"ON\"");
        lcd_set_cmd("r0.val=1");
        eerom_cfg.auto_bl_en = 1;
    }
    else
    {
        lcd_set_cmd("t0.txt=\"OFF\"");
        lcd_set_cmd("r0.val=0");
        eerom_cfg.auto_bl_en = 0;
    }

    if(orig == eerom_cfg.auto_bl_en)
        return 0;
    else
        return 1;
}

void lcd_update_auto_bl()
{
    int err = 0;
    PrintDBG("Update ABL\r\n");
    err = eerom_write_cfg(&eerom_cfg);
    if (err < 0)
    {
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }
}

int lcd_show_fix_bl()
{
    uint16_t str_buf[4] = {0xB9CC, 0xB6A8, 0xB1B3, 0xB9E2};
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_NUM);
    lcd_set_chinese("t1.txt=", str_buf, 4);

    lcd_set_cmd("t0.txt=\"Value:\"");
    int value = eerom_cfg.fix_bl_val;
    lcd_set_txt_value("t2.txt=", value, 0);
    lcd_set_cmd("ref_star");

    return (uint8_t)eerom_cfg.fix_bl_val;
}

int lcd_modify_fix_bl(int orig, int btn_up, int btn_down)
{
    unsigned int value = eerom_cfg.fix_bl_val;
    if (btn_up == btn_down)
        return 0;
    
    if(btn_up)
    {
        if(value < 100)
            value+=10;
        if(value > 100)
            value = 100;
    }
    else
    {
        if(value > 0)
            value -= 10;
        if(value < 0)
            value = 0;
    }

    eerom_cfg.fix_bl_val = value;
    lcd_set_txt_value("t2.txt=", value, 0);

    if(value == orig)
        return 0;
    else
        return 1;
}

void lcd_update_fix_bl()
{
    int err = 0;
    PrintDBG("Update FBL\r\n");
    err = eerom_write_cfg(&eerom_cfg);
    if (err < 0)
    {
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }

}

int lcd_show_device_addr()
{
    uint16_t str_buf[4] = {0xC9E8, 0xB1B8, 0xB5D8, 0xD6B7};
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_NUM);
    lcd_set_chinese("t1.txt=", str_buf, 4);

    lcd_set_cmd("t0.txt=\"Value:\"");
    int value = eerom_cfg.dev_addr;
    lcd_set_txt_value("t2.txt=", value, 1);
    lcd_set_cmd("ref_star");

    return (uint8_t)eerom_cfg.dev_addr;
}

int lcd_modify_device_addr(int orig, int btn_up, int btn_down)
{
    if(btn_up == btn_down)
        return 0;
    
    uint8_t value = eerom_cfg.dev_addr;
    if(btn_up)
    {
        value++;
    }
    else
    {
        value--;
    }
    eerom_cfg.dev_addr = value;
    lcd_set_txt_value("t2.txt=", value, 1);
    if(orig == value)
        return 0;
    else
        return 1;
}

void lcd_update_device_addr()
{
    int err = 0;
    PrintDBG("Update DevAddr\r\n");
    err = eerom_write_cfg(&eerom_cfg);
    if (err < 0)
    {
        PrintERR("ERR:Update Cfg error:%d\r\n", err);
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }
}

int lcd_show_factory_reset()
{
    uint16_t str_buf[4] = {0xB3F6, 0xB3A7, 0xC9E8, 0xD6C3};
    lcd_set_cmd("ref_stop");
    lcd_switch_page(LCD_PAGE_NUM);
    lcd_set_chinese("t1.txt=", str_buf, 4);

    lcd_set_cmd("t0.txt=\"Confirm:\"");
    int value = 10;
    lcd_set_txt_value("t2.txt=", value, 0);
    lcd_set_cmd("ref_star");
    factory_reset = 10;
    return 10;
}

int lcd_modify_factory_reset(int orig, int btn_up, int btn_down)
{
    if(btn_up == btn_down)
        return 0;
    if(orig != 10)
        return 0;
    int value = factory_reset;
    if(btn_up)
    {
        if(value > 0)
            value--;
        if(value < 0)
            value = 0;
    }
    else
    {
        value = 10;
    }
    factory_reset = value;
    lcd_set_txt_value("t2.txt=", value, 0);
    
    if(value == 0)
        return 1;
    else
        return 0;

}

void lcd_update_factory_reset()
{
    int err = 0;
    uint8_t *ptr;
    uint16_t crc_val = 0;

    PrintDBG("WARN: Factory Reset\r\n");
    
    //Disable ACM
    co2_auto_calib(0);
    HAL_Delay(200);
    //Reset ACM
    co2_calib_reset();

    co2_setup_alt(CONFIG_BASE_CO2_ALT_OFFSET);

    eerom_cfg.magic.d32      = EEROM_MAGIC_WORD;
    eerom_cfg.alt_offset.d16 = CONFIG_BASE_CO2_ALT_OFFSET;
    eerom_cfg.co2_acm_en     = 0;
    eerom_cfg.temp_offset    = 0;
    eerom_cfg.hm_offset      = 0;
    eerom_cfg.auto_bl_en     = 1;
    eerom_cfg.fix_bl_val     = 0;
    eerom_cfg.dev_addr       = 0;
    ptr = &eerom_cfg.co2_acm_en;
    crc_val = crc16_mod_bus(ptr, sizeof(struct eerom_content_s) - 6);
    eerom_cfg.crc.d16 = crc_val;

    err = eerom_write_cfg(&eerom_cfg);
    if (err < 0)
    {
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }
}

//------------------------------------- CONFIG MODE ---------------------------------------

typedef int (*panel_show_func)();
typedef int (*panel_modify_func)(int orig, int btn_up, int btn_down);
typedef void (*panel_setup_func)();

struct menu_ops_s
{
    panel_show_func panel_show;
    panel_modify_func panel_modify;
    panel_setup_func panel_setup;
};

struct menu_ops_s menu_ops[] = {
    {lcd_show_help,             NULL,                       NULL},                  //Help page
    {lcd_show_info_page,        NULL,                       NULL},                  //Infomation page
    {lcd_show_diag_mode,        lcd_update_diag_mode,       NULL},                  //Diagnocity mode
    {lcd_show_co2_acm,          lcd_update_co2_acm,         lcd_set_co2_acm},       //setup CO2 Auto calibration
    {lcd_show_co2_acm_reset,    lcd_update_co2_acm_reset,   lcd_set_co2_acm_reset}, //Reset CO2 Calibration
    {lcd_show_co2_alt,          lcd_modify_co2_alt,         lcd_setup_co2_alt},     //Setup CO2 altitude calibration
    {lcd_show_temp_adj,         lcd_modify_temp_adj,        lcd_update_temp_adj},   //temprature calibration
    {lcd_show_hm_adj,           lcd_modify_hm_adj,          lcd_update_hm_adj},     //Humidity calibration
    {lcd_show_auto_bl,          lcd_modify_auto_bl,         lcd_update_auto_bl},    //Auto backlight calibration
    {lcd_show_fix_bl,           lcd_modify_fix_bl,          lcd_update_fix_bl},     //Fix backlight value
    {lcd_show_device_addr,      lcd_modify_device_addr,     lcd_update_device_addr},//Device address(For PCN)
    {lcd_show_factory_reset,    lcd_modify_factory_reset,   lcd_update_factory_reset}   //Factory reset
};

#define MENU_ITEMS_CNT (sizeof(menu_ops) / sizeof(struct menu_ops_s))


extern volatile int key_down_press_type;
extern volatile int key_up_press_type;
void config_mode()
{
    int current_page = 0;
    int return_cnt = 15;
    lcd_switch_page(LCD_PAGE_HELP);
    unsigned int tick_val = 0;
    int prev_value = 0;
    int update_next = 0;
    int err = 0;

    if(eerom_read_cfg(&eerom_cfg) < 0)
    {
        PrintERR("ERROR:EEROM Invalid\r\n");
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        lcd_show_info_page();

        HAL_Delay(10000);
        lcd_switch_page(LCD_PAGE_ALL);
        return;
    }

    while (1)
    {
        tick_val = HAL_GetTick();
        if ((tick_val % 1000) == 0)
        {
            return_cnt--;
            if (return_cnt == 0)
                goto menu_exit;
            HAL_Delay(2);
        }

        if (key_up_press_type == KEY_PRESS_LONG)
        {
            key_up_press_type = KEY_PRESS_NONE;
            key_down_press_type = KEY_PRESS_NONE;
            return_cnt = 15;
            goto menu_exit;
        }

        if (key_down_press_type == KEY_PRESS_LONG)
        {
            key_down_press_type = KEY_PRESS_NONE;
            key_up_press_type = KEY_PRESS_NONE;
            return_cnt = 15;

            if(update_next)
            {
                if(menu_ops[current_page].panel_setup != NULL)
                    menu_ops[current_page].panel_setup();
                update_next = 0;
            }

            current_page++;
            if (current_page >= MENU_ITEMS_CNT)
                current_page = 0;

            if (menu_ops[current_page].panel_show != NULL)
            {
                prev_value = menu_ops[current_page].panel_show();
            }
        }

        if ((key_up_press_type == KEY_PRESS_SHORT) ||
            (key_down_press_type == KEY_PRESS_SHORT))
        {
            return_cnt = 15;
            if (menu_ops[current_page].panel_modify != NULL)
            {
                update_next = menu_ops[current_page].panel_modify(prev_value, key_up_press_type, key_down_press_type);
            }
            key_down_press_type = KEY_PRESS_NONE;
            key_up_press_type = KEY_PRESS_NONE;
            
        }
    }

menu_exit:
    //when exit, everything should be save to eerom already
    //if not, then the saved value will be overwritten by the
    //content of eerom
    err = eerom_read_cfg(&eerom_cfg);
    if (err < 0)
    {
        PrintERR("ERROR:EEROM Invalid : %d\r\n", err);
        extern int system_error;
        system_error |= SYS_ERROR_FLAG_ROM;
        menu_setup_error = 1;
    }

    if(menu_setup_error)
    {
        lcd_show_info_page();
        HAL_Delay(10000);
        menu_setup_error = 0;
    }

    lcd_switch_page(LCD_PAGE_ALL);
}

#include "esp32_s3_szp.h"

static const char *TAG = "esp32_s3_szp";

/******************************************************************************/
/***************************  I2C ↓ *******************************************/
esp_err_t bsp_i2c_init(void)
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_FREQ_HZ
    };
    i2c_param_config(BSP_I2C_NUM, &i2c_conf);

    return i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0);
}
/***************************  I2C ↑  *******************************************/
/*******************************************************************************/


/*******************************************************************************/
/***************************  姿态传感器 QMI8658 ↓   ****************************/

// 读取QMI8658寄存器的值
esp_err_t qmi8658_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BSP_I2C_NUM, QMI8658_SENSOR_ADDR,  &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// 给QMI8658的寄存器写值
esp_err_t qmi8658_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(BSP_I2C_NUM, QMI8658_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

// 初始化qmi8658
void qmi8658_init(void)
{
    uint8_t id = 0; // 芯片的ID号

    qmi8658_register_read(QMI8658_WHO_AM_I, &id ,1); // 读芯片的ID号
    while (id != 0x05)  // 判断读到的ID号是否是0x05
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延时1秒
        qmi8658_register_read(QMI8658_WHO_AM_I, &id ,1); // 读取ID号
    }
    ESP_LOGI(TAG, "QMI8658 OK!");  // 打印信息

    qmi8658_register_write_byte(QMI8658_RESET, 0xb0);  // 复位  
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 延时10ms
    qmi8658_register_write_byte(QMI8658_CTRL1, 0x40); // CTRL1 设置地址自动增加
    qmi8658_register_write_byte(QMI8658_CTRL7, 0x03); // CTRL7 允许加速度和陀螺仪
    qmi8658_register_write_byte(QMI8658_CTRL2, 0x95); // CTRL2 设置ACC 4g 250Hz
    qmi8658_register_write_byte(QMI8658_CTRL3, 0xd5); // CTRL3 设置GRY 512dps 250Hz 
}

// 读取加速度和陀螺仪寄存器值
void qmi8658_Read_AccAndGry(t_sQMI8658 *p)
{
    uint8_t status, data_ready=0;
    int16_t buf[6];

    qmi8658_register_read(QMI8658_STATUS0, &status, 1); // 读状态寄存器 
    if (status & 0x03) // 判断加速度和陀螺仪数据是否可读
        data_ready = 1;
    if (data_ready == 1){  // 如果数据可读
        data_ready = 0;
        qmi8658_register_read(QMI8658_AX_L, (uint8_t *)buf, 12); // 读加速度和陀螺仪值
        p->acc_x = buf[0];
        p->acc_y = buf[1];
        p->acc_z = buf[2];
        p->gyr_x = buf[3];
        p->gyr_y = buf[4];
        p->gyr_z = buf[5];
    }
}

// 获取XYZ轴的倾角值
void qmi8658_fetch_angleFromAcc(t_sQMI8658 *p)
{
    float temp;

    qmi8658_Read_AccAndGry(p); // 读取加速度和陀螺仪的寄存器值
    // 根据寄存器值 计算倾角值 并把弧度转换成角度
    temp = (float)p->acc_x / sqrt( ((float)p->acc_y * (float)p->acc_y + (float)p->acc_z * (float)p->acc_z) );
    p->AngleX = atan(temp)*57.29578f; // 180/π=57.29578
    temp = (float)p->acc_y / sqrt( ((float)p->acc_x * (float)p->acc_x + (float)p->acc_z * (float)p->acc_z) );
    p->AngleY = atan(temp)*57.29578f; // 180/π=57.29578
    temp = sqrt( ((float)p->acc_x * (float)p->acc_x + (float)p->acc_y * (float)p->acc_y) ) / (float)p->acc_z;
    p->AngleZ = atan(temp)*57.29578f; // 180/π=57.29578
}
/***************************  姿态传感器 QMI8658 ↑  ****************************/
/*******************************************************************************/


/***********************************************************/
/***************    IO扩展芯片 ↓   *************************/

// 读取PCA9557寄存器的值
esp_err_t pca9557_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BSP_I2C_NUM, PCA9557_SENSOR_ADDR,  &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// 给PCA9557的寄存器写值
esp_err_t pca9557_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(BSP_I2C_NUM, PCA9557_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

// 初始化PCA9557 IO扩展芯片
void pca9557_init(void)
{
    // 写入控制引脚默认值 DVP_PWDN=1  PA_EN = 0  LCD_CS = 1
    pca9557_register_write_byte(PCA9557_OUTPUT_PORT, 0x05);  
    // 把PCA9557芯片的IO1 IO1 IO2设置为输出 其它引脚保持默认的输入
    pca9557_register_write_byte(PCA9557_CONFIGURATION_PORT, 0xf8); 
}

// 设置PCA9557芯片的某个IO引脚输出高低电平
esp_err_t pca9557_set_output_state(uint8_t gpio_bit, uint8_t level)
{
    uint8_t data;
    esp_err_t res = ESP_FAIL;

    pca9557_register_read(PCA9557_OUTPUT_PORT, &data, 1);
    res = pca9557_register_write_byte(PCA9557_OUTPUT_PORT, SET_BITS(data, gpio_bit, level));

    return res;
}

// 控制 PCA9557_LCD_CS 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
void lcd_cs(uint8_t level)
{
    pca9557_set_output_state(LCD_CS_GPIO, level);
}

// 控制 PCA9557_PA_EN 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
void pa_en(uint8_t level)
{
    pca9557_set_output_state(PA_EN_GPIO, level);
}

// 控制 PCA9557_DVP_PWDN 引脚输出高低电平 参数0输出低电平 参数1输出高电平 
void dvp_pwdn(uint8_t level)
{
    pca9557_set_output_state(DVP_PWDN_GPIO, level);
}

/***************    IO扩展芯片 ↑   *************************/
/***********************************************************/




/***********************************************************/
/***************    AXP2101电源管理芯片 ↓   *************************/

// 读取PCA9557寄存器的值
esp_err_t axp2101_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BSP_I2C_NUM, AXP2101_PMU_ADDR,  &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// 给PCA9557的寄存器写值
esp_err_t axp2101_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(BSP_I2C_NUM, AXP2101_PMU_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

// 初始化AXP2101电源管理芯片
void axp2101_init(void)
{
    // axp2101_register_write_byte(PCA9557_OUTPUT_PORT, 0x05);  
    axp2101_register_write_byte(AXP2101_PWROFF_EN, 0b110);  // PWRON > OFFLEVEL as POWEROFF Source enable
    axp2101_register_write_byte(AXP2101_PONLEVEL, 0x10);    // hold 4s to power off

    // axp2101_register_write_byte(AXP2101_DCDC_CFG0, 0x0F);   // 0x0F = 0b1111 启用DCDC1, DCDC2, DCDC3, DCDC4
    axp2101_register_write_byte(AXP2101_DCDC_CFG0, 0x05); 

    // Disable All LDOs
    axp2101_register_write_byte(AXP2101_LDO_EN_CFG0, 0x00);   
    axp2101_register_write_byte(AXP2101_LDO_EN_CFG1, 0x00); 

    // Set DC1
    axp2101_register_write_byte(AXP2101_DCDC1_CFG, 0x12);   // 3.3V
    // axp2101_register_write_byte(AXP2101_DCDC2_CFG, 0x46);   // 1.2V
    axp2101_register_write_byte(AXP2101_DCDC3_CFG, 0x67);   // 3.3V
    // axp2101_register_write_byte(AXP2101_DCDC4_CFG, 0x46);   // 1.2V
    axp2101_register_write_byte(AXP2101_DCDC_CFG0, 0x05); 
    
    // 设置ALDO输出电压
    axp2101_register_write_byte(AXP2101_ALDO1_CFG, 0x1C);   // 3.3V
    axp2101_register_write_byte(AXP2101_ALDO2_CFG, 0x17);   // 2.8V
    axp2101_register_write_byte(AXP2101_ALDO3_CFG, 0x07);   // 1.2V
    axp2101_register_write_byte(AXP2101_ALDO4_CFG, 0x1C);   // 3.3V
    
    // 启用ALDO1-4
    axp2101_register_write_byte(AXP2101_LDO_EN_CFG0, 0x0F);     // 0x0F = 0b00001111 启用ALDO1, ALDO2, ALDO3, ALDO4

    // 其他LDO/Buck转换器如果不使用，则确保它们被关闭
    axp2101_register_write_byte(AXP2101_LDO_EN_CFG1, 0x00);     // 确保BLDO1, BLDO2, CPUSLDO, DLDO1都关闭
    
    axp2101_register_write_byte(AXP2101_CHG_V_CFG, 0x02);       // CV charger voltage setting to 4.1V

    axp2101_register_write_byte(AXP2101_IPRECHG_CFG, 0x02);     // set Main battery precharge current to 50mA
    axp2101_register_write_byte(AXP2101_ICC_CFG, 0x08);         // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
    axp2101_register_write_byte(AXP2101_ITERM_CFG, 0x01);       // set Main battery term charge current to 25mA
}

/***************    AXP2101电源管理芯片 ↑   *************************/
/***********************************************************/

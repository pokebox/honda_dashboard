#ifndef CONFIG_H
#define CONFIG_H

// 网络配置
#define USE_BROADCAST
#define USE_BLE
//#define USE_DISPLAY

#define BTN_PIN 0
#ifdef USE_DISPLAY
    // 硬件引脚定义
    #define TFT_DC 46
    #define TFT_RST 21
    #define TFT_CS 10
    #define TFT_MOSI 11
    #define TFT_MISO 9
    #define TFT_CLK 12
    #define TFT_LED 45

    #define SPI_FREQUENCY 35000000

    // 屏幕尺寸
    #define SCREEN_WIDTH 480
    #define SCREEN_HEIGHT 320
    
    // 颜色定义
    #define COLOR_BACKGROUND lv_color_hex(0x1a1a2e)    // 深蓝背景
    #define COLOR_TEXT_MAIN lv_color_hex(0xffffff)     // 白色主文字
    #define COLOR_TEXT_SECONDARY lv_color_hex(0xcccccc) // 灰色次要文字
    #define COLOR_ACCENT lv_color_hex(0x00ff88)        // 绿色强调色
    #define COLOR_DOOR_OPEN lv_color_hex(0xff5555)     // 红色（门开）
    #define COLOR_DOOR_CLOSED lv_color_hex(0x55ff55)   // 绿色（门关）
#endif

#ifdef USE_BROADCAST
#define ESPNOW_WIFI_CHANNEL 6
#define ESPNOW_PMK "your_primary_key"
#define ESPNOW_LMK "your_local_key00"
#endif

#endif // CONFIG_H
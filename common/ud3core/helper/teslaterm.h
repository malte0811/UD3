
#include <stdint.h>


#define TT_GAUGE 1
#define TT_GAUGE_CONF 2
#define TT_CHART 3
#define TT_CHART_DRAW 4
#define TT_CHART_CONFIG 5
#define TT_CHART_CLEAR 6
#define TT_CHART_LINE 7
#define TT_CHART_TEXT 8


#define TT_UNIT_NONE 0
#define TT_UNIT_V 1
#define TT_UNIT_A 2
#define TT_UNIT_W 3
#define TT_UNIT_Hz 4
#define TT_UNIT_C 5



void send_gauge(uint8_t gauge, int16_t val, uint8_t port);

void send_chart(uint8_t chart, int16_t val, uint8_t port);
void send_chart_draw(uint8_t port);
void send_chart_config(uint8_t chart, int16_t min, int16_t max, int16_t offset, uint8_t unit,char * text, uint8_t port);
void send_gauge_config(uint8_t gauge, int16_t min, int16_t max, char * text, uint8_t port);
void send_chart_text(int16_t x, int16_t y, uint8_t color, uint8_t size, char * text, uint8_t port);
void send_chart_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t color, uint8_t port);
void send_chart_clear(uint8_t port);
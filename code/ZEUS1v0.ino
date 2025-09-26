// ZEUS v1.0 - DIY Digital Lab Power Supply
// Author: MikołajDIY
// Date: 2025-09
// License: MIT
//
// NOTE: Main code blocks are documented in English.
// Some detailed comments are in Polish (native language).

// Program structure:
// 1. Initialization (setup):
//    - SPI for TFT and MCP41010
//    - GPIO setup (buttons, relay)
//    - ADC configuration
//
// 2. Main loop:
//    - Read and average voltage, current, temperature
//    - Safety checks (overcurrent, overtemperature)
//    - User input handling (buttons)
//    - Display update

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include "hardware/gpio.h"
#include "hardware/adc.h"

// --- TFT na SPI0 ---
#define TFT_CS   5
#define TFT_DC   6
#define TFT_RST  7
Adafruit_ST7735 tft = Adafruit_ST7735(&SPI, TFT_CS, TFT_DC, TFT_RST);

// --- MCP41010 na SPI1 ---
#define MCP_CS   13
void mcp41010_write(byte value) {
  digitalWrite(MCP_CS, LOW);
  delayMicroseconds(1);
  SPI1.transfer(0x11);
  SPI1.transfer(value);
  digitalWrite(MCP_CS, HIGH);
  delayMicroseconds(1);
}

// --- przyciski ---
#define BTN_UP   19
#define BTN_DOWN 21
#define BTN_RESET 20

// --- wyjście GP8 ---
#define OVERCURRENT_OUT 8

// --- ADC ---
#define ADC_VOLTAGE 0
#define ADC_CURRENT 1
#define ADC_TEMP    2

// --- stałe ---
const float VREF = 3.3;
const float R_TOP = 7610.0;
const float R_BOTTOM = 1002.0;
const float R_SHUNT = 0.02;
const float G_DIFF = 100000.0 / 3000.0;

// --- uśrednianie ---
#define AVG_COUNT_V 65
#define AVG_COUNT_I 20
#define AVG_COUNT_TEMP 50
float voltage_buf[AVG_COUNT_V] = {0};
float current_buf[AVG_COUNT_I] = {0};
int temp_buf[AVG_COUNT_TEMP] = {0};
int buf_index_v=0, buf_index_i=0, temp_index=0;

// --- blokady ---
bool overcurrent_blocked=false;
bool overtemp_blocked=false;
bool btn_reset_last=HIGH;
int last_gp8_state=-1;
unsigned long overcurrent_start=0;
const unsigned long OVERCURRENT_DELAY=80; // ms

// --- funkcje odczytu ---
float read_voltage() {
  adc_select_input(ADC_VOLTAGE);
  uint16_t raw = adc_read();
  float v_adc = (raw / 4095.0) * VREF;
  float v_meas = v_adc * ((R_TOP + R_BOTTOM) / R_BOTTOM);
  float v_calibrated = 0.000116451993 * v_meas*v_meas*v_meas // Calibration polynomial (3rd order) generated from CSV measurement data
                      -0.00363755165*v_meas*v_meas
                      +1.00563567*v_meas
                      -0.178398237;
  return v_calibrated;
}

float read_current() {
  adc_select_input(ADC_CURRENT);
  uint16_t raw = adc_read();
  float v_adc = (raw / 4095.0) * VREF - 0.02;
  return v_adc / (R_SHUNT * G_DIFF);
}

int read_temp() {
  adc_select_input(ADC_TEMP);
  uint16_t raw = adc_read();
  float v_adc = (raw / 4095.0) * VREF;
  //float temp = (v_adc - 0.5 * (3.3 / 5.0)) / (0.01 * (3.3 / 5.0));  // TMP36 przy 3.3V
  float temp = (v_adc / 0.01) - 50; // TMP36
  return round(temp); // zaokrąglenie do 1°C
}

// --- średnie ---
float average_voltage(float new_val) {
  voltage_buf[buf_index_v] = new_val;
  buf_index_v = (buf_index_v + 1) % AVG_COUNT_V;
  float sum=0; for(int i=0;i<AVG_COUNT_V;i++) sum+=voltage_buf[i];
  return sum / AVG_COUNT_V;
}

float average_current(float new_val) {
  current_buf[buf_index_i] = new_val;
  buf_index_i = (buf_index_i + 1) % AVG_COUNT_I;
  float sum=0; for(int i=0;i<AVG_COUNT_I;i++) sum+=current_buf[i];
  return sum / AVG_COUNT_I;
}

int average_temp(int new_val) {
  temp_buf[temp_index] = new_val;
  temp_index = (temp_index + 1) % AVG_COUNT_TEMP;
  long sum=0; for(int i=0;i<AVG_COUNT_TEMP;i++) sum+=temp_buf[i];
  return sum / AVG_COUNT_TEMP;
}

// --- wyświetlanie ---
void display_overcurrent_warning() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED); tft.setTextSize(3);
  tft.setCursor(10,30); tft.println("I>3,5A");
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(2);
  tft.setCursor(10,100); tft.println("zresetuj  zasilacz");
}

void display_overtemp_warning() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_RED); tft.setTextSize(3);
  tft.setCursor(10,30); tft.println("T>70 C");
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(2);
  tft.setCursor(10,100); tft.println("zresetuj  zasilacz");
}

void display_gp8_state() {
  int state = digitalRead(OVERCURRENT_OUT);
  if(state!=last_gp8_state){
    tft.fillRect(10,120,150,16,ST77XX_BLACK);
    tft.setCursor(10,120); tft.setTextColor(ST77XX_CYAN);
    if(state) tft.print("Zas. wl"); else tft.print("Zas. wyl");
    last_gp8_state = state;
  }
}

// --- setup ---
void setup() {
  SPI.setRX(4); SPI.setTX(3); SPI.setSCK(2); SPI.begin();
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2); tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(10,10); tft.println("ZEUS v1.0");

  pinMode(MCP_CS, OUTPUT); digitalWrite(MCP_CS,HIGH);
  gpio_set_function(10,GPIO_FUNC_SPI);
  gpio_set_function(11,GPIO_FUNC_SPI);
  gpio_set_function(12,GPIO_FUNC_SPI);
  SPI1.begin(); SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);

  pinMode(OVERCURRENT_OUT, OUTPUT); digitalWrite(OVERCURRENT_OUT, LOW);

  adc_init();
  adc_gpio_init(26+ADC_VOLTAGE);
  adc_gpio_init(26+ADC_CURRENT);
  adc_gpio_init(26+ADC_TEMP);
}

// --- loop ---
void loop() {
  static byte mcp_val=128;
  static float last_voltage=-1;
  static float last_current=-1;
  static int last_mcp=-1;
  static int last_temp=-100;

  // --- odczyty ---
  float voltage = average_voltage(read_voltage());
  float current = read_current();
  float current_avg = average_current(current);
  int temp_raw = read_temp();
  int temp = average_temp(temp_raw);

  float voltage_r = round(voltage*10.0)/10.0;
  float current_r = round(current_avg*100.0)/100.0;

  // --- nadprąd ---
  if(!overcurrent_blocked){
    if(current>3.5){
      if(overcurrent_start==0) overcurrent_start=millis();
      if(millis()-overcurrent_start>=OVERCURRENT_DELAY){
        overcurrent_blocked=true;
        digitalWrite(OVERCURRENT_OUT,LOW);
        display_overcurrent_warning();
      }
    } else overcurrent_start=0;
  }

  // --- nadtemperatura ---
  if(!overtemp_blocked && temp>70){
    overtemp_blocked=true;
    digitalWrite(OVERCURRENT_OUT,LOW);
    display_overtemp_warning();
  }

  // --- przycisk GP20 ---
  bool btn_reset_now = digitalRead(BTN_RESET);
  if(btn_reset_last==HIGH && btn_reset_now==LOW){
    digitalWrite(OVERCURRENT_OUT, !digitalRead(OVERCURRENT_OUT));
    if(overcurrent_blocked || overtemp_blocked){
      overcurrent_blocked=false;
      overtemp_blocked=false;
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextSize(2); tft.setTextColor(ST77XX_GREEN);
      tft.setCursor(10,10); tft.println("ZEUS v1.0");
      last_voltage=-1; last_current=-1; last_mcp=-1; last_gp8_state=-1; last_temp=-100;
    }
  }
  btn_reset_last = btn_reset_now;

  if(!overcurrent_blocked && !overtemp_blocked){
    if(digitalRead(BTN_UP)==LOW){if(mcp_val<255)mcp_val++; mcp41010_write(mcp_val); delay(120);}
    if(digitalRead(BTN_DOWN)==LOW){if(mcp_val>0)mcp_val--; mcp41010_write(mcp_val); delay(120);}

    if(fabs(voltage_r-last_voltage)>0.1){
      tft.fillRect(10,40,150,16,ST77XX_BLACK);
      tft.setCursor(10,40); tft.setTextColor(ST77XX_GREEN);
      tft.print("V "); tft.print(voltage_r,1); tft.print(" V");
      last_voltage = voltage_r;
    }

    if(fabs(current_r-last_current)>0.02){
      tft.fillRect(10,60,150,16,ST77XX_BLACK);
      tft.setCursor(10,60); tft.setTextColor(ST77XX_GREEN);
      tft.print("I "); tft.print(current_r,2); tft.print(" A");
      last_current = current_r;
    }

    if(mcp_val!=last_mcp){
      tft.fillRect(10,80,120,16,ST77XX_BLACK);
      tft.setCursor(10,80); tft.setTextColor(ST77XX_GREEN);
      tft.print("MCP "); tft.print(mcp_val);
      last_mcp = mcp_val;
    }

    if(temp!=last_temp){
      tft.fillRect(10,140,80,16,ST77XX_BLACK);
      tft.setCursor(10,140); tft.setTextColor(ST77XX_YELLOW);
      tft.print("T "); tft.print(temp); tft.print(" C");
      last_temp = temp;
    }

    display_gp8_state();
  }

  delay(10);
}

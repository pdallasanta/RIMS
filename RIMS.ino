
#include <FastIO.h>
#include <I2CIO.h>
#include <LCD.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_SR.h>
#include <LiquidCrystal_SR2W.h>
#include <LiquidCrystal_SR3W.h>

#include <Arduino.h>
#include <avr/eeprom.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include "RIMS.h"

volatile unsigned char state = 0; // estado atual do encoder
volatile boolean buttonPressed = false; // botao foi apertado? (por interrupcao)

settings_t settings;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
OneWire ds(OW_PIN);
DallasTemperature sensors(&ds);
// TODO: autodetectar sensor
DeviceAddress tempSensor = { 0x28, 0x88, 0x85, 0xA8, 0x04, 0x00, 0x00, 0x67 };
double pid_in, pid_out;
unsigned long ssr_win_starttime;
boolean ssr_state; // estado atual do SSR
int ssr_win_size = 2000; // 2000ms de janela (mudar de acordo com o modo?)
PID pid(&pid_in, &pid_out, &settings.sv, PID_CONS_KP, PID_CONS_KI, PID_CONS_KD, DIRECT);

void setup() {
  // tenta ler a configuracao salva nd eeprom
  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));

  // pelos meus testes, a eeprom nao inicializada fica sempre como NAN
  // nesse caso, faz uma config inicial com uns valores default
  if (isnan(settings.sv)) {
    settings.sv = 65.0;
    settings.Kp = PID_CONS_KP;
    settings.Ki = PID_CONS_KI;
    settings.Kd = PID_CONS_KD;
    settings.pid_mode = PID_MODE_CONS;
  }

  // inicializa pinos
  pinMode(SSR_PIN, OUTPUT);
  pinMode(ENC_PINA, INPUT); 
  pinMode(ENC_PINB, INPUT);
  pinMode(ENC_SW, INPUT);
  digitalWrite(ENC_PINA, HIGH);
  digitalWrite(ENC_PINB, HIGH);
  digitalWrite(ENC_SW, HIGH);

  // inicializa o resto
  sensors.begin();
  sensors.setResolution(tempSensor, 10); // 10 bits de precisao
  ssr_win_starttime = millis();
  pid.SetOutputLimits(0, ssr_win_size);
  pid.SetMode(AUTOMATIC);
  attachInterrupt(ENC_IRQ, buttonPress, FALLING);
  lcd.begin(20,4);
}

void loop() {
  unsigned long now;
  static boolean ssr_last_state = LOW; // SSR sempre inicia desligado
  static unsigned long last_lcd_update = 0;
  static unsigned long last_temp_update = 0;

  if (buttonPressed) {
    ssr_state = LOW; // desliga o SSR antes de entrar no menu
    digitalWrite(SSR_PIN, ssr_state);
    changeTemp(); // menu
  }

  // se diferenca entre PV e SV for maior que GAP, usa PID mais agressivo
  if (abs(settings.sv - pid_in) < PID_GAP) {
    settings.Kp = PID_CONS_KP;
    settings.Ki = PID_CONS_KI;
    settings.Kd = PID_CONS_KD;
    settings.pid_mode = PID_MODE_CONS;
  } else {
    settings.Kp = PID_AGG_KP;
    settings.Ki = PID_AGG_KI;
    settings.Kd = PID_AGG_KD;
    settings.pid_mode = PID_MODE_AGG;
  }

  // le temperatura, ajusta PID e calcula
  // le a temperatura a cada intervalo (default = 500ms)
  now = millis();
  if (now - last_temp_update > TEMP_INTERVAL) {
    sensors.requestTemperatures();
    pid_in = (double)sensors.getTempC(tempSensor);
    last_temp_update = now;
  }
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  pid.Compute();

  // o output vai determinar se ativa o SSR ou nao, com base na janela
  now = millis();
  if (now - ssr_win_starttime > ssr_win_size) ssr_win_starttime += ssr_win_size;
  if (pid_out > now - ssr_win_starttime) ssr_state = HIGH;
  else ssr_state = LOW;

  // se o SSR vai mudar de estado
  if (ssr_state != ssr_last_state) {
    digitalWrite(SSR_PIN, ssr_state); // muda o pino
    lcdUpdateSSR(ssr_state); // atualiza de imediato no LCD
    ssr_last_state = ssr_state;
  }

  // atualiza o LCD a cada intervalo (default = 1000ms)
  if (now - last_lcd_update > LCD_INTERVAL) {
    last_lcd_update = now;
    lcdUpdate();
  }
}

// menu para alterar temperatura
void changeTemp() {
  unsigned char result;
  unsigned long now, last_blink;
  boolean blink = false;

  lcdUpdateSSR(ssr_state); // mostra o novo valor do SSR porque ele eh desligado quando entra aqui
  lcdUpdateSV(); // mostrar o SV em modo de mudanca de temperatura
  lcdBlinkSV(false); // mostra os colchetes ao redor da temperatura
  buttonPressed = false;

  last_blink = millis();
  while (!buttonPressed) {
    // fica nesse loop de ajuste do SV ateh o cara apertar o botao de novo
    result = rotary_process();
    if (result == DIR_CCW) settings.sv -= ENC_STEP;
    else if (result == DIR_CW) settings.sv += ENC_STEP;
    if (result) lcdUpdateSV(); // soh atualiza LCD se mexeu o encoder
    
    // pisca os colchetes ao redor do SV a cada 500ms
    now = millis();
    if (now - last_blink > 500) {
      blink = !blink;
      last_blink = now;
      lcdBlinkSV(blink);
    }
  }
  
  // chegou aqui porque botao foi apertado novamente, entao salva e sai
  eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
  buttonPressed = false;
}

// tah tosco isso aqui
// cuidar com overflow dessas strings
void lcdUpdate() {
  char line[20], tmp1[20], tmp2[20];
  int i;

  lcd.setCursor(0,0); // linha 1
  dtostrf(pid_in, 5, 2, tmp1);
  dtostrf(settings.sv, 5, 2, tmp2);
  sprintf(line, "PV:%s  SV:%s", tmp1, tmp2);
  lcd.print(line);
  for (i=strlen(line);i<20;i++) lcd.print(" "); // completa ateh o final da linha com espacos

  lcd.setCursor(0,1); // linha 2
  dtostrf(pid.GetKp(), 5, 2, tmp1);
  sprintf(line, "Kp:%s   M:%s", tmp1, (settings.pid_mode == PID_MODE_CONS ? "Cons" : "Agg"));
  lcd.print(line);
  for (i=strlen(line);i<20;i++) lcd.print(" ");

  lcd.setCursor(0,2); // linha 3
  dtostrf(pid.GetKi(), 5, 2, tmp1);
  dtostrf(pid_out, 5, 2, tmp2);
  sprintf(line, "Ki:%s Out:%s", tmp1, tmp2);
  lcd.print(line);
  for (i=strlen(line);i<20;i++) lcd.print(" ");

  lcd.setCursor(0,3); // linha 4
  dtostrf(pid.GetKd(), 5, 2, tmp1);
  sprintf(line, "Kd:%s SSR:%s", tmp1, ssr_state == HIGH ? "On" : "Off");
  lcd.print(line);
  for (i=strlen(line);i<20;i++) lcd.print(" ");
}

// atualiza o SV no LCD
void lcdUpdateSV() {
  char tmp[20];
  dtostrf(settings.sv, 5, 2, tmp);
  lcd.setCursor(14,0);
  lcd.print(tmp);
}

// liga ou desliga os colchetes ao redor do SV
void lcdBlinkSV(boolean blink) {
  if (blink) {
    lcd.setCursor(13,0);
    lcd.print(" ");
    lcd.setCursor(19,0);
    lcd.print(" ");    
  } else {
    lcd.setCursor(13,0);
    lcd.print("[");
    lcd.setCursor(19,0);
    lcd.print("]");    
  }
}

// mostra On ou Off de acordo com o estado do SSR
void lcdUpdateSSR(boolean state) {
  lcd.setCursor(13,3);
  if (state == HIGH) lcd.print("On "); else
  lcd.print("Off");
}

// handler da interrupcao do pino do botao do encoder
void buttonPress() {
  // debounce de pobre
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) {
    buttonPressed = true;
  }
  last_interrupt_time = interrupt_time;
}

// move a maquina de estados do encoder de acordo com o estado dos pinos
// ttable estah no RIMS.h
unsigned char rotary_process() {
  unsigned char pinstate = (digitalRead(ENC_PINB) << 1) | digitalRead(ENC_PINA);
  state = ttable[state & 0xf][pinstate];
  return (state & 0x30);
}


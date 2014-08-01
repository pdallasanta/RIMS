#include <Arduino.h>
#include <avr/eeprom.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "RIMS.h"

volatile unsigned char state = 0; // estado atual do encoder
volatile boolean buttonPressed = false; // botao foi apertado? (por interrupcao)
volatile int op_state = OS_OFF; // inicia desligado
settings_t settings;

// LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Sensores de temperatura
OneWire ds(OW_PIN);
DallasTemperature sensors(&ds);
DeviceAddress tempSensor;

// PID
volatile long on_time = 0;
int pid_mode = PID_MODE_NORMAL;
double pid_in, pid_out;
unsigned long ssr_win_starttime;
int ssr_win_size = 10000; // 10000ms de janela (mudar de acordo com o modo?)
PID pid(&pid_in, &pid_out, &settings.sv, PID_KP, PID_KI, PID_KD, DIRECT);
double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;
boolean tuning = false;
PID_ATune aTune(&pid_in, &pid_out);

void setup() {
  // tenta ler a configuracao salva nd eeprom
  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));

  // pelos meus testes, a eeprom nao inicializada fica sempre como NAN
  // nesse caso, faz uma config inicial com uns valores default
  if (isnan(settings.sv)) {
    settings.sv = 65.0;
    settings.Kp = PID_KP;
    settings.Ki = PID_KI;
    settings.Kd = PID_KD;
  }

  // inicializa pinos
  pinMode(SSR_PIN, OUTPUT);
  pinMode(ENC_PINA, INPUT); 
  pinMode(ENC_PINB, INPUT);
  pinMode(ENC_SW, INPUT);
  digitalWrite(SSR_PIN, LOW);
  digitalWrite(ENC_PINA, HIGH);
  digitalWrite(ENC_PINB, HIGH);
  digitalWrite(ENC_SW, HIGH);

  // inicializa o sensor de temperatura (pega o primeiro que achar)
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0)) {
    // faz alguma coisa se nao achar nenhum sensor
  }
  sensors.setResolution(tempSensor, 10); // 10 bits de precisao
  sensors.setWaitForConversion(false);

  // inicializa o PID
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  pid.SetSampleTime(1000);
  pid.SetOutputLimits(0, ssr_win_size);
  pid.SetMode(AUTOMATIC);

  // timer2 a cada 15ms
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
  TIMSK2 |= 1<<TOIE2; // overflow interrupt enable

  // inicializa o resto
  ssr_win_starttime = millis();
  attachInterrupt(ENC_IRQ, buttonPress, FALLING);
  lcd.begin(20,4);
  op_state = OS_RUN;
}

void loop() {
  unsigned long now;
  static unsigned long last_lcd_update = 0;
  static unsigned long last_temp_update = 0;

  if (buttonPressed) {
    op_state = OS_OFF; // desliga o SSR antes de entrar no menu
    changeTemp(); // menu
  }

  // le temperatura, ajusta PID e calcula
  // le a temperatura a cada intervalo (default = 500ms)
  now = millis();
  if (now - last_temp_update > TEMP_INTERVAL) {
    sensors.requestTemperatures();
    pid_in = (double)sensors.getTempC(tempSensor);
    last_temp_update = now;
  }

  // fica em tune ate aTune.Runtime() ele retornar true
  if (tuning) {
    if (aTune.Runtime()) {
      finish_autotune();
    }
  } else {
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    pid.Compute();
  }

  // atualiza o LCD a cada intervalo (default = 1000ms)
  if (now - last_lcd_update > LCD_INTERVAL) {
    last_lcd_update = now;
    lcdUpdate();
  }

  // copia pid_out para on_time que eh volatile
  on_time = pid_out;
}

// handler da interrupcao do timer
SIGNAL(TIMER2_OVF_vect) {
  if ((op_state == OS_OFF) || (op_state == OS_ERR)) {
    digitalWrite(SSR_PIN, LOW);
  } else {
    SSR_Output();
  }
}

// decide quando o SSR liga ou desliga
void SSR_Output() {
  unsigned long now = millis();
  
  if (now - ssr_win_starttime > ssr_win_size) {
    ssr_win_starttime += ssr_win_size;
  }
  
  if ((on_time > 100) && (on_time > (now - ssr_win_starttime))) {
    digitalWrite(SSR_PIN, HIGH);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }
}

// menu para alterar temperatura
void changeTemp() {
  unsigned char result;
  unsigned long now, last_blink;
  boolean blink = false;
  
  lcdUpdateSV(); // mostrar o SV em modo de mudanca de temperatura
  lcdBlinkSV(false); // mostra os colchetes ao redor da temperatura
  buttonPressed = false;
  last_blink = millis();
  
  // fica nesse loop de ajuste do SV ateh o cara apertar o botao de novo
  while (!buttonPressed) {
    result = rotary_process();
    
    if (result == DIR_CCW) {
      settings.sv -= ENC_STEP;
    } else if (result == DIR_CW) {
      settings.sv += ENC_STEP;
    }
    
    if (result) {
      lcdUpdateSV(); // soh atualiza LCD se mexeu o encoder
    }
    
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
  op_state = OS_RUN;
  
  // gambiarra temporaria: se depois do ajuste a temperatura atual estiver a
  // 1 grau de distancia da SV, ele entra em autotune
  if (abs(pid_in - settings.sv) < 1.0) {
    start_autotune();
  }
}

void start_autotune() {
  pid_mode = PID_MODE_TUNE;
  aTune.SetNoiseBand(aTuneNoise);
  aTune.SetOutputStep(aTuneStep);
  aTune.SetLookbackSec((int)aTuneLookBack);
  tuning = true;
}

void finish_autotune() {
  tuning = false;
  settings.Kp = aTune.GetKp();
  settings.Ki = aTune.GetKi();
  settings.Kd = aTune.GetKd();
  pid_mode = PID_MODE_NORMAL;
  
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));   
}

// tah tosco isso aqui
// cuidar com overflow dessas strings
void lcdUpdate() {
  char tmp1[20], tmp2[20];

  lcd.clear();
  dtostrf(pid_in, 5, 2, tmp1);
  dtostrf(settings.sv, 5, 2, tmp2);
  lcd.setCursor(0,0);
  lcd.print(F("PV: "));
  lcd.print(tmp1);
  lcd.setCursor(10,0);
  lcd.print(F("SV: "));
  lcd.print(tmp2);

  lcd.setCursor(0,1);
  lcd.print(F("M: "));
  lcd.print(pid_mode == PID_MODE_NORMAL ? "Norm" : "Tune");
  lcd.setCursor(9,1);
  lcd.print(F("SSR: "));
  lcd.print(digitalRead(SSR_PIN) == HIGH ? "On" : "Off");

  lcd.setCursor(0,2);
  lcd.print(F("Kp/Ki/Kd:"));

  lcd.setCursor(0,3);
  dtostrf(pid.GetKp(), 0, 2, tmp1);
  lcd.print(tmp1);
  lcd.print(F("/"));
  dtostrf(pid.GetKi(), 0, 2, tmp1);
  lcd.print(tmp1);
  lcd.print(F("/"));
  dtostrf(pid.GetKd(), 0, 2, tmp1);
  lcd.print(tmp1);
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


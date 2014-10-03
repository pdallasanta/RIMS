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
double pid_in, pid_out;
unsigned long ssr_win_starttime;
int ssr_win_size = 2000; // 2000ms de janela
PID pid(&pid_in, &pid_out, &settings.sv, PID_KP, PID_KI, PID_KD, DIRECT);

//Serial Communication
char command_buffer[MAX_COMMAND_SIZE] =  "";
char command_ptr = 0;

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
  pinMode(ENC_SW_ALT, INPUT);
  pinMode(FLOW_SW, INPUT);
  pinMode(PUMP_REL, OUTPUT);
  pinMode(PROT_REL, OUTPUT);
  pinMode(AUX_REL, OUTPUT);
  pinMode(SPARE1, OUTPUT);
  pinMode(SPARE2, OUTPUT);
  pinMode(SPARE3, OUTPUT);
  pinMode(SPARE4, OUTPUT);
  pinMode(SPARE5, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  digitalWrite(ENC_PINA, HIGH);
  digitalWrite(ENC_PINB, HIGH);
  digitalWrite(ENC_SW, HIGH);
  digitalWrite(ENC_SW_ALT, HIGH);
  digitalWrite(PUMP_REL, HIGH);
  digitalWrite(PROT_REL, HIGH);
  digitalWrite(AUX_REL, LOW);
  digitalWrite(SPARE1, LOW);
  digitalWrite(SPARE2, LOW);
  digitalWrite(SPARE3, LOW);
  digitalWrite(SPARE4, LOW);
  digitalWrite(SPARE5, LOW);

  // inicializa serial
  Serial.begin(SERIAL_PORT_SPEED);

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
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  pid.Compute();

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
  lcdBlinkSV(true); // garante que os colchetes ficaram desligados
  eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
  buttonPressed = false;
  op_state = OS_RUN;
}

// tah tosco isso aqui
// cuidar com overflow dessas strings
void lcdUpdate() {
  char tmp1[20], tmp2[20];
  
  dtostrf(pid_in, 5, 2, tmp1);
  dtostrf(settings.sv, 5, 2, tmp2);
  lcd.setCursor(0,0);
  lcd.print(F("PV: "));
  lcd.print(tmp1);
  lcd.setCursor(10,0);
  lcd.print(F("SV: "));
  lcd.print(tmp2);
  
  lcd.setCursor(0,1);
  lcd.print(F("Kp: "));
  dtostrf(pid.GetKp(), 0, 1, tmp1);
  lcd.print(tmp1);
  lcd.setCursor(0,2);
  lcd.print(F("Ki: "));
  dtostrf(pid.GetKi(), 0, 1, tmp1);
  lcd.print(tmp1);
  lcd.setCursor(0,3);
  lcd.print(F("Kd: "));
  dtostrf(pid.GetKd(), 0, 1, tmp1);
  lcd.print(tmp1);
  
  lcd.setCursor(12,3);
  lcd.print(F("SSR: "));
  lcd.print(digitalRead(SSR_PIN) == HIGH ? "On " : "Off");
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

/*
 * Funcoes para comunicacao serial
*/

// handler da serial
void serialEvent() {
  unsigned char read_byte;

  while (Serial.available()) {
    read_byte = Serial.read();

    // command must start with a '?'
    if ((command_ptr == 0) && (read_byte != '?'))
      continue;

    // if command start received, reset command buffer
    if (read_byte == '?') {
      command_ptr = 0;
      command_buffer[command_ptr++] = read_byte;
      continue;
    }

    // write byte to command buffer
    command_buffer[command_ptr++] = read_byte;

    // command end (0x0A)
    if (read_byte == 0x0A) {
      parseCommand(); // call command parser
      command_ptr = 0;
      break;
    }

    // command size violation: reset buffer
    if (command_ptr >= MAX_COMMAND_SIZE)
      command_ptr = 0;
  }
}

// command parser
void parseCommand() {
  unsigned char command_id = command_buffer[1];
  unsigned char checksum = 0;
  unsigned char address;
  unsigned long data;
  unsigned char data_size;
  unsigned char i;

  // Checksum computation
  for (i = 0; i < command_ptr - 4; i++) { // CR/LF end markers and checksum itself are excluded from checksum computation
    checksum += command_buffer[i];
  }

  // Verify checksum
  if ((long)checksum != ascii2long(command_buffer, command_ptr - 4, command_ptr - 3)) {
    sendError(command_id, 1);
    return;
  }

  // get address
  address = (char) ascii2long(command_buffer, 2, 3);

  // data field size
  data_size = command_ptr - 8; // 8 = size of START + CMD_ID + ADDR + CHECKSUM + CR/LF

  switch(command_id){
    // Write command
    case WRITE_REGISTER_CMD:
      // Data field size verification
      if ((data_size < 1) || (data_size > 8)) {
          sendError(command_id, 0);
          return;
      }
      // Extract data
      data = ascii2long(command_buffer, 4, 4 + data_size - 1);

      if (setRegister(address, data)) {
        sendAck(command_id);
      } else {
        sendError(command_id, 0);
      }
      break;

    // Read command
    case READ_REGISTER_CMD:
      sendCommand('R', getRegister(address), 4);
      break;
    // Invalid
    default:
      break;
  }
}

// ASCII hex string to long
unsigned long ascii2long (const char *ptr, unsigned char left, unsigned char right) {
  unsigned long value = 0;
  char ch;
  unsigned char nibble;

  for (nibble = left; nibble <= right; nibble++) {
    ch = *(ptr + nibble);
    if (ch >= '0' && ch <= '9')
        value = (value << 4) + (ch - '0');
    else if (ch >= 'A' && ch <= 'F')
      value = (value << 4) + (ch - 'A' + 10);
    else if (ch >= 'a' && ch <= 'f')
      value = (value << 4) + (ch - 'a' + 10);
  }
  return value;
}

// Send command
void sendCommand(char id, unsigned long data, unsigned char data_bytes){
  unsigned char checksum = 0;
  char ptr = 0;
  char i;
  char nibble;
  char command[MAX_COMMAND_SIZE];

  command[ptr++] = '=';
  command[ptr++] = id;

  // Convert data nibbles to ASCII
  for (i = 2*data_bytes - 1; i >= 0; i--){
    nibble = (data >> i*4) & 0xf;
    command[ptr++] = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
  }

  // Send command bytes and calculate checksum
  for (i = 0; i < ptr; i++){
    Serial.write(command[i]); // write byte to serial
    checksum += command[i]; // update checksum
  }

  // Send checksum
  if (checksum < 16) Serial.print('0');
  Serial.print(checksum, HEX);

  // Ending
  Serial.write(0x0D); // CR
  Serial.write(0x0A); // LF

  Serial.flush();
}

// Send command error messasge
void sendError(char src_cmd_id, char error_code){
  unsigned char checksum;
  unsigned char err_code_ascii = error_code < 10 ? '0' + error_code : 'A' + error_code - 10;

  Serial.write('=');
  Serial.write('E');
  Serial.write(src_cmd_id);
  Serial.write(err_code_ascii);

  checksum = '=' + 'E' + src_cmd_id + err_code_ascii;

  // Send checksum
  if (checksum < 16) Serial.print('0');
  Serial.print(checksum, HEX);

  // Ending
  Serial.write(0x0D); // CR
  Serial.write(0x0A); // LF

  Serial.flush();
}

// Send acknowledge
void sendAck(char src_cmd_id){
  unsigned char checksum;

  Serial.write('=');
  Serial.write('K');
  Serial.write(src_cmd_id);

  checksum = '=' + 'K' + src_cmd_id;

  // Send checksum
  if (checksum < 16) Serial.print('0');
  Serial.print(checksum, HEX);

  // Ending
  Serial.write(0x0D); // CR
  Serial.write(0x0A); // LF

  Serial.flush();
}

// Get register value
unsigned long getRegister(unsigned char address) {
  switch(address){
    // System ID
    case REG_SYS_ID:
      return SYSTEM_ID;
    // Firmware version:
    case REG_FW_VER:
        return (FW_MAJOR_VERSION << 8) + FW_MINOR_VERSION;
    // PID parameters
    case REG_PID_SV:
        return long(settings.sv*pow(2.0, 16));
    case REG_PID_KP:
        return long(settings.Kp*pow(2.0, 16));
    case REG_PID_KI:
        return long(settings.Ki*pow(2.0, 16));
    case REG_PID_KD:
        return long(settings.Kd*pow(2.0, 16));
    case REG_PID_OP_MODE:
        return long(op_state);
    case REG_RIMS_OUT_T:
        return long(pid_in*pow(2.0, 16));
    case REG_HEATER_PWM:
        return long(100.0*pow(2.0, 16)*pid_out/double(ssr_win_size));
    case REG_ALARMS:
    default:
      return 0;
  }
}

// Set register value
char setRegister(unsigned char address, unsigned long data) {
  switch(address){
    case REG_PID_SV:
      settings.sv = double(data)/pow(2.0, 16);
      return 1;
    case REG_PID_KP:
      settings.Kp = double(data)/pow(2.0, 16);
      return 1;
    case REG_PID_KI:
      settings.Ki = double(data)/pow(2.0, 16);
      return 1;
    case REG_PID_KD:
      settings.Kd    = double(data)/pow(2.0, 16);
      return 1;
    // Non-volatile settings read/write control
    case REG_PID_NV_SETTINGS:
      switch(data) {
        // Save settings
        case 1:
          eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));
          return 1;
        // load settings
        case 2:
          eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));
          return 1;
        // Invalid
        default:
          return 0;
      }
    default:
      return 0;
  }
}

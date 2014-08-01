/*
 * RIMS.h
 */

/*
 * Definicoes de estado do sistema
 */
#define OS_RUN 0
#define OS_OFF 1
#define OS_ERR 2

/*
 * Definicoes do PID
 */ 
#define PID_MODE_NORMAL 0
#define PID_MODE_TUNE   1
#define PID_KP          850
#define PID_KI          0.5
#define PID_KD          0.1

/*
 * Definicoes de timers
 */
#define LCD_INTERVAL  1000 // refresh do LCD
#define TEMP_INTERVAL 1000 // refresh da temperatura

/*
 * Definicoes de seguranca
 * MIN_TEMP - Define a temperatura minima lida pelo sensor para o sistema operar.
 * Serve como uma seguranca para indicar se algum liquido ja esta circulando.
 * MAX_TEMP_DELTA - Delta maximo entre uma leitura e outra de tempertura. Pode
 * indicar um superaquecimento no sistema (resistencia seca).
 */
#define MIN_TEMP       45.0
#define MAX_TEMP_DELTA 1.0

/*
 * Definicoes de pinos
 * SSR_PIN - Pino do SSR
 * ENC_PINA - Pino A do encoder
 * ENC_PINB - Pino B do encoder
 * ENC_SW - Botao do encoder. O Arduino Uno soh tem interrupcao nos pinos
 * 2 e 3, entao eh bom deixar em um desses dois.
 * ENC_IRQ - Interrupcao para o botao
 * OW_PIN - Pino do OneWire
 */
#define SSR_PIN  13 // pino do SSR
#define ENC_PINA 6 // pino A do Rotary Encoder
#define ENC_PINB 7 // pino B do Rotary Encoder
#define ENC_SW   2 // botao do Rotary Encoder
#define ENC_IRQ  0 // IRQ0 fica associada ao pino 2
#define OW_PIN   A0 // pino do 1-wire

/*
 * Definicoes do Rotary Encoder
 * ENC_STEP define quantos graus vai alterar a cada click do encoder.
 */
#define ENC_STEP 0.1

// estados do encoder
#define R_START     0x0
#define R_CW_FINAL  0x1
#define R_CW_BEGIN  0x2
#define R_CW_NEXT   0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT  0x6
#define DIR_CCW     0x10
#define DIR_CW      0x20

// a magica do encoder estah nessa tabela de estados
const unsigned char ttable[7][4] = {
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

// configuracoes do sistema que vao pra eeprom
struct settings_t {
  double sv; // SetValue
  double Kp, Ki, Kd; // PID
};


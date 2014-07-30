/*
 * RIMS.h
 */

/*
 * Definicoes do PID
 * PID_GAP - quando a diferenca entre a temperatura atual e a medida
 * for maior que esse valor, ele muda os parametros para valores
 * mais agressivo (PID_AGG_*). Senao ele fica no modo mais conservador
 * (PID_CONS_*).
 */ 
#define PID_GAP       10 // gap para decidir entre os modos
#define PID_MODE_CONS 0 // conservador
#define PID_MODE_AGG  1 // agressivo
#define PID_CONS_KP   50
#define PID_CONS_KI   0.2
#define PID_CONS_KD   10
#define PID_AGG_KP    50
#define PID_AGG_KI    0.5
#define PID_AGG_KD    1

/*
 * Definicoes de timers
 */
#define LCD_INTERVAL  1000 // refresh do LCD
#define TEMP_INTERVAL 500 // refresh da temperatura

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
  int pid_mode; // pid_mode (PID_MODE_AGG | PID_MODE_CONS)
};

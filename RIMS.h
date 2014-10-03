/*
 * RIMS.h
 */

 /*
 * Versao de firmware
 */
#define FW_MAJOR_VERSION 0
#define FW_MINOR_VERSION 2

/*
 * Definicoes de estado do sistema
 */
#define OS_RUN 0
#define OS_OFF 1
#define OS_ERR 2

/*
 * Definicoes do PID
 */
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
 * Definicoes da comunicacao serial
 *
*/
#define SERIAL_PORT_SPEED   57600 // Serial port bit rate
#define MAX_COMMAND_SIZE    16    // Maximum command size

//System Id
#define SYSTEM_ID   0x52494d53    // 'RIMS'

// IDs dos comandos
#define WRITE_REGISTER_CMD  'W' // Write register command Id
#define READ_REGISTER_CMD   'R' // Read register command Id
#define ACK_CMD             'K' // ACK command Id
#define ERROR_CMD           'E' // Error commmand Id

// Offsets dos registradores
#define REG_SYS_ID          0x00 // System Id
#define REG_FW_VER          0x01 // FW version
#define REG_PID_SV          0x08 // PID set value
#define REG_PID_KP          0x09 // KP
#define REG_PID_KI          0x0A // KI
#define REG_PID_KD          0x0B // KD
//#define REG_PID_SET_MODE    0x0C // PID set mode -- obsoleto
#define REG_PID_OP_MODE     0x0D // PID operating mode
#define REG_PID_NV_SETTINGS 0x0F // PID non-volatile settings control
#define REG_RIMS_OUT_T      0x80 // RIMS output temperature
#define REG_HEATER_PWM      0x81 // Heater PWM
#define REG_ALARMS          0x82 // Alarms

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
#define SSR_PIN    A3 // pino do SSR
#define ENC_PINA   5  // pino A do Rotary Encoder (CLK)
#define ENC_PINB   6  // pino B do Rotary Encoder (DT)
#define ENC_SW     2  // botao do Rotary Encoder
#define ENC_SW_ALT 7  // pino alternativo para o botao do Rotary Encoder (para evitar conflito com shield Ethernet)
#define ENC_IRQ    0  // IRQ0 fica associada ao pino 2
#define OW_PIN     8  // pino do 1-wire
#define FLOW_SW    9  // sensor de fluxo
#define PUMP_REL   A0 // rele de acionamento da bomba
#define PROT_REL   A1	// rele de protecao da resistencia
#define AUX_REL    13 // Auxiliary/spare relay
#define SPARE1     11 // Spare 1
#define SPARE2     12 // Spare 2
#define SPARE3     A2 // Spare 3
#define SPARE4     A5 // Spare 4
#define SPARE5     A4 // Spare 5

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


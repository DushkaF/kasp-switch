#include <Arduino.h>

#define TIME_MEASUREMENT_PIN 13

#define BAUD_RATE 115200
#define PKT_KP0 0x00
#define PKT_KP1 0x01
#define PKT_KP2 0x02
#define PKT_OP0 0x10

#define KP0_LEN 4
#define KP1_LEN 6
#define KP2_LEN 20
#define OP0_LEN 8

uint8_t tiers[4];
uint8_t series[13];


/*
  Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 байт(127 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
unsigned char Crc8(unsigned char *pcBlock, unsigned int len)
{
    unsigned char crc = 0xFF;
    unsigned int i;

    while (len--)
    {
        crc ^= *pcBlock++;

        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }

    return crc;
}

void  op0_send(int8_t num_1,int8_t num_2){     // Формируем ОП0
      uint8_t op0[OP0_LEN];
      op0[0] = PKT_OP0;        // тип пакета
      op0[1] = num_1;         // номер цикла (старший байт)
      op0[2] = num_2;         // номер цикла (младший байт)
    
      // состояние секторов 
      for (int i;i<4;i++){
        op0[i+3] = tiers[i];
        }
      op0[7] = Crc8(op0, OP0_LEN - 1);
            
      // Отправка ответа
      Serial.write(op0, OP0_LEN);
}
bool decodeAndRespond(uint8_t *kp) {
  // Проверка CRC КП1
  if (kp[0]==PKT_KP1){
    uint8_t kp1[KP1_LEN];
    for (int i=0;i<KP1_LEN;i++){
      kp1[i]=kp[i];
      } 
    if (Crc8(kp1, KP1_LEN - 1) == kp1[KP1_LEN - 1]) {
      uint16_t cycle = (kp[1] << 8) | kp[2];
      uint8_t pair = kp[3];
      uint8_t sector = kp[4];
      tiers[pair]=sector;
      op0_send(kp[1],kp[2]);
      return 1;
    }
    else{
      return 0;
      }
  }
  if (kp[0]==PKT_KP2){
    uint8_t kp2[KP2_LEN];
    for (int i=0;i<KP2_LEN;i++){
      kp2[i]=kp[i];
      } 
    if (Crc8(kp2, KP2_LEN - 1) == kp2[KP2_LEN - 1]) {
      uint16_t cycle = (kp[1] << 8) | kp[2];
      uint8_t pair = kp[3];
      uint8_t period = (kp[4] << 8) | kp[5];
      for (int i=0;i<13;i++){
        series[i]=kp2[6+i];
        }
      tiers[pair]=series[12];
      op0_send(kp[1],kp[2]);
      return 1;
    }
    else{
      return 0;
      }
  }
  if (kp[0]==PKT_KP0){
    uint8_t kp0[KP0_LEN];
    for (int i=0;i<KP0_LEN;i++){
      kp0[i]=kp[i];
      } 
    if (Crc8(kp0, KP0_LEN - 1) == kp0[KP0_LEN - 1]) {
      uint16_t cycle = (kp[1] << 8) | kp[2];    
      op0_send(kp[1],kp[2]);
      return 1;
    }
    else{
      return 0;
      }
  }
}

void setup() {
//  gpio_reset_pin((gpio_num_t) TIME_MEASUREMENT_PIN);
//  gpio_set_direction((gpio_num_t) TIME_MEASUREMENT_PIN, GPIO_MODE_OUTPUT); // вместо pinMode
  Serial.begin(BAUD_RATE);
  while (!Serial) {}
}

void loop() {
  static uint8_t buf[KP2_LEN];
  static size_t idx = 0;

  while (Serial.available()) {
    buf[idx++] = Serial.read();
   // а нафига этот тут?! 
//    gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 1);
//    gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 0);

    // 1 пин переключается 
    if (buf[0] == PKT_KP1 and idx>KP1_LEN-1) {
      if(decodeAndRespond(buf)){
        //      gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 1);
        //      gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 0);
        }
      idx = 0;
    }
        // 13 пинов переключаются 
    if (buf[0] == PKT_KP2 and idx>KP2_LEN-1) {
      if(decodeAndRespond(buf)){
        //      gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 1);
        //      gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 0);
        }
      idx = 0;
    }
            // ничего не переключается, возвращается состояние 
    if (buf[0] == PKT_KP0 and idx>KP0_LEN-1) {
      if(decodeAndRespond(buf)){
        //      gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 1);
        //      gpio_set_level((gpio_num_t)TIME_MEASUREMENT_PIN, 0);
        }
      idx = 0;
    }
  }
}

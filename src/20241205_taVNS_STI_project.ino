#include <bluefruit.h>
#include <SPI.h>
#define BUF_LENGTH          20
#define BUF_LENGTH2         240
#define DEVICE_NAME       "taVNS_Sti"
#define TICK_INTERVAL_us    10

union MyUnion{
  uint16_t myVar;
  uint8_t myByte[2];
};

uint16_t swt_ctr = 0;
uint16_t swt_ref_v = 0x32;
uint16_t const swt_ref_p = 0xC350; //50000 in decimal

uint16_t swt_full = 0x1388; //5000
uint8_t swt_pos = 0x1E;
uint8_t swt_mid = 0x00;
uint8_t swt_neg = 0x00;

bool enb_switching = false;
bool enb_taVNSmode = false;
bool enb_tDCSmode = false;
bool enb_sendData_p = false;
bool enb_sendData_n = false;
// PFM
bool enb_PFM = false;
int pulse_width = 10;
int base_frequency = 24;  // Base frequency in Hz
int mod_depth = 12;  // Modulation depth
float mod_frequency = 6.0;  // Modulation frequency in Hz
unsigned long last_time = 0;

MyUnion dac_data_p;
MyUnion dac_data_n;
MyUnion swt_full_16;
MyUnion swt_data;
/* ECG Service: 00001523-1212-EFDE-1523-785FEABCD123
 * ECG RAW : 00001524-1212-EFDE-1523-785FEABCD123
 */
const uint8_t HS_UUID_SERVICE[] =
{
    0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
    0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00
};
const uint8_t HS_UUID_CHR_RAW[] =
{
    0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 
    0x75, 0x4C, 0x3E, 0x50,    1,    0, 0x3D, 0x71
};

static int counter2 = 0;
static bool connected2 = false;
BLEDis  bledis;    // DIS (Device Information Service) helper class instance
BLEService        hs(HS_UUID_SERVICE);
BLECharacteristic hsraw(HS_UUID_CHR_RAW);


extern "C" {

  void SysTick_Handler(void)
  {
     if (connected2){
      if(enb_switching){
        switching();
      }
    }
  }

}// extern C

void PFM(){
  if(enb_PFM){
    unsigned long current_time = millis();

    // Calculate modulated frequency based on the sine of the modulation signal
    float modulation = sin(2 * PI * mod_frequency * (current_time / 1000.0)) * mod_depth;
    int modulated_frequency = base_frequency + modulation;

    // Calculate pulse period based on modulated frequency
    int pulse_period = 1000 / modulated_frequency;  // In milliseconds

    // Generate positive pulse
    if (current_time - last_time >= pulse_period) {
      digitalWrite(11, LOW);
      digitalWrite(12, LOW);
      delay(pulse_width);  // Keep pulse width constant

      // Switch to ground after pulse width duration
      digitalWrite(12, HIGH);
      delay(pulse_period - pulse_width);

      // Reset timing for the next cycle
      last_time = current_time;
    }
  }
}

void switching(){
  if(enb_switching){
    if(swt_ctr <= swt_pos){ //positive
      // if(swt_ctr == 0x01){
      //   enb_sendData_p = true;
      // }
      digitalWrite(11, LOW);
      digitalWrite(12, LOW);
      swt_ctr ++;
    }else if(swt_ctr > swt_pos && swt_ctr <= swt_pos + swt_mid){
      if(swt_ctr == swt_pos + 1){
        enb_sendData_n = true;
      }
      digitalWrite(11, LOW);
      digitalWrite(12, HIGH);
      swt_ctr ++;
    }else if(swt_ctr > swt_pos + swt_mid && swt_ctr <= swt_pos + swt_mid + swt_neg){ //negative
      // if(swt_ctr == swt_pos + swt_mid + 1){
      //   enb_sendData_n = true;
      // }
      digitalWrite(11, HIGH);
      digitalWrite(12, LOW);
      swt_ctr ++;
    }else if(swt_ctr < swt_full){
      if(swt_ctr == swt_pos + swt_mid + swt_neg + 1){
        enb_sendData_p = true;
      }
      digitalWrite(11, LOW);
      digitalWrite(12, HIGH);
      swt_ctr ++;
    }else{
      digitalWrite(11, LOW);
      digitalWrite(12, HIGH);
      swt_ctr = 0x0001;
    }
  // }else{
  //   digitalWrite(11, LOW);
  //   digitalWrite(12, HIGH);
  }
}

void setup()
{
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(A7, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  Serial.begin(115200);
  Serial.println("Bluefruit52 nRF Blinky Example");
  Serial.println("------------------------------\n");
  analogReadResolution(8);

  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  
  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);
  Bluefruit.setTxPower(7);

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 6);//(4, 8);//(5, 10);//(6, 12);

  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Yutech, Taiwan");
  bledis.setModel("TriAnswer");
  bledis.begin();

  hs.begin();
  hsraw.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  hsraw.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  //hsraw.setMaxLen(BUF_LENGTH);
  hsraw.setFixedLen(BUF_LENGTH);
  hsraw.begin();
  hsraw.setWriteCallback(dac_write_callback);
  
  Serial.println("enb_sendControl up the advertising");
  startAdv();
  SysTick_Config( (F_CPU/1000000)*TICK_INTERVAL_us ); //every 10us
  // SysTick_Config( (F_CPU/1000)*TICK_INTERVAL_us ); //every 10ms
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(hs);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

// data[0][4~7],data[0][0~3]    ,data[1]  ,data[2]  ,(data[4],data[3]) ,data[5]      ,data[6]       ,data[7]
// output mode ,select Sti mode ,pos amp. ,neg amp. ,total duration(T) ,pos dur.(T1) ,mid dur.(T2) ,neg dur.(T3)
void dac_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  (void) conn_hdl;
  (void) chr;
  (void) len;
  digitalWrite(LED_BUILTIN, data[0] ? LED_STATE_ON : (1-LED_STATE_ON));

  uint8_t low_nibble = data[0] & 0x0F; // 取出低4位 (0b00001111)
  uint8_t high_nibble = (data[0] >> 4) & 0x0F; // 取出高4位 (0b11110000 >> 4)
  Serial.print("Low nibble: ");
  Serial.println(low_nibble);
  Serial.print("High nibble: ");
  Serial.println(high_nibble);
  Serial.println(data[1]);
  Serial.println(data[2]);
  
  if(low_nibble == 0x00){ //taVNS mode
    enb_taVNSmode = true;
  }
  else if(low_nibble == 0x01){ //tDCS mode
    enb_tDCSmode = true;
  }

  if(high_nibble == 0x01){ //taVNS AC mode
    Serial.println("AC mode");
    enb_switching = true;
  }
  else if(high_nibble == 0x02){ //taVNS DC mode
    Serial.println("DC mode");
    enb_switching = false;
    if(data[1] == 0x00){
      dac_data_p.myByte[0] = 0x00;
      dac_data_p.myByte[1] = data[1];
      enb_sendData_p = true;
    }
  }
  else if(high_nibble == 0x03){
    Serial.println("PFM mode");
    enb_PFM = true;
  }

  if(data[1] != 0x00){           //pos amplitude
    dac_data_p.myByte[0] = 0x00;
    dac_data_p.myByte[1] = data[1];
    enb_sendData_p = true;
  }
  if(data[2] != 0x00){           //neg amplitude
    dac_data_n.myByte[0] = 0x00;
    dac_data_n.myByte[1] = data[2];
    enb_sendData_n = true;
  }

  Serial.println(data[3]);
  Serial.println(data[4]);
  Serial.println(data[5]);
  Serial.println(data[6]);
  Serial.println(data[7]);
  swt_full_16.myByte[0] = data[3];
  swt_full_16.myByte[1] = data[4];
  swt_full = swt_full_16.myVar;
  if(data[7] == 0){
    Serial.println("monophasic");
    swt_pos = data[5];
    swt_mid = 0x00;
    swt_neg = 0x00;
  }else{                    //T1, T2, T3
    Serial.println("biphasic");
    swt_pos = data[5];
    swt_mid = data[6];
    swt_neg = data[7];
  }
}

void loop() {
  if(enb_taVNSmode){
    enb_taVNSmode = false;
    DAC_taVNS_setup();
  }
  else if(enb_tDCSmode){
    enb_tDCSmode = false;
    DAC_tDCS_setup();
  }

  if(enb_sendData_p){
    enb_sendData_p = false;
    writeData(dac_data_p.myByte[1], dac_data_p.myByte[0]);
    //Serial.println(dac_data_p.myVar, HEX);
  }
  if(enb_sendData_n){
    enb_sendData_n = false;
    writeData(dac_data_n.myByte[1], dac_data_n.myByte[0]);
    //Serial.println(dac_data_n.myVar, HEX);
  }
}


void connect_callback(uint16_t conn_handle)
{
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  Serial.println("Connected");
  delay(1000);
  connected2 = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  connected2 = false;
  enb_switching = false;
  enb_taVNSmode = false;
  enb_tDCSmode = false;
  enb_sendData_p = false;
  enb_sendData_n = false;
  delay(1000);
  Serial.println("Disconnected");
}

void writeData(byte data8, byte data4){
  digitalWrite(8, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(210);
  SPI.transfer(0x01);
  SPI.transfer(data8); //也許[data8, data4]是用[0x????, 0x????]的格式傳遞
  SPI.transfer(data4);
  digitalWrite(8, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(8, LOW);
  SPI.end();
}
void DAC_taVNS_setup(){
  digitalWrite(8, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(210);
  SPI.transfer(0x55);
  SPI.transfer(0x30); //0x00110000
  SPI.transfer(0x06); //AD5410 Iout transfer function [0x0000_0,R2,R1,R0]
  digitalWrite(8, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(8, LOW);
  SPI.end();
}
void DAC_tDCS_setup(){
  digitalWrite(8, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(210);
  SPI.transfer(0x55);
  SPI.transfer(0x3F); //0x00111111
  SPI.transfer(0x16); //0x00010110 AD5410 Iout transfer function [0x0001_0,R2,R1,R0]
  digitalWrite(8, LOW);
  digitalWrite(8, HIGH);
  digitalWrite(8, LOW);
  SPI.end();
}

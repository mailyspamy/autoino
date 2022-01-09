/*
   VAG_CDC.c

   Created: 23.06.2013 20:00:51
    Author: Dennis Schuett, dev.shyd.de
    port for arduino by kovo
*/
//#include <SPI.h>
//#include <AltSoftSerial.h>
//AltSoftSerial altSerial;

#define DataIn 11
#define ClockPin 13
#define SSPin 10
//#define PhonePin 9
//D12 DataOut

#define PACKET_DALEY 50

#define UART_BAUD_RATE 115200
#define RADIO_OUT PB4
#define RADIO_OUT_IS_HIGH (PINB & (1<<RADIO_OUT))

#define CDC_PREFIX1 0x53 //ok zacatek povelu
#define CDC_PREFIX2 0x2C //ok zacatek povelu

#define CDC_END_CMD 0x14
#define CDC_PLAY 0xE4 //nasleduje po vlevo nebo vpravo
#define CDC_STOP 0x10 //ok radio/kazeta hraje cyklicky
#define CDC_NEXT 0xF8 //ok nahoru
#define CDC_PREV 0x78 //ok dolu
#define CDC_SEEK_FWD 0xD8 //ok vpravo
#define CDC_SEEK_RWD 0x58 //ok vlevo
#define CDC_CD1 0x0C //ok
#define CDC_CD2 0x8C //ok
#define CDC_CD3 0x4C //ok
#define CDC_CD4 0xCC //ok
#define CDC_CD5 0x2C //ok
#define CDC_CD6 0xAC //ok
#define CDC_SCAN 0xA0 //ok
#define CDC_SFL 0x60 //pokud play dlouhy stisk nahoru/dolu
#define CDC_PLAY_NORMAL 0x08 //pokud SCAN nebo SHFFL dlouhy stisk nahoru/dolu, nebo dvakrat cislo
#define CDC_PREV_CD 0x18
#define CDC_END_CMD2 0x38 //nasleduje po volbe cisla

#define MODE_PLAY 0xFF
#define MODE_SHFFL 0x55
#define MODE_SCAN 0x00

volatile uint16_t captimehi = 0;
volatile uint16_t captimelo = 0;
volatile uint8_t capturingstart = 0;
volatile uint8_t capturingbytes = 0;
volatile uint32_t cmd = 0;
volatile uint8_t cmdbit = 0;
volatile uint8_t newcmd = 0;

volatile uint8_t cd = 0x0;
volatile uint8_t tr = 0x00;
volatile uint8_t mode = MODE_SCAN;

volatile uint8_t c = 0;
volatile uint8_t prev_c = 0;
volatile uint32_t cas = 0;
volatile long previousMillis = 0;
volatile long previousMillis_prev_c = 0;
volatile long previousMillis2 = 0;

boolean btOff = true;
boolean soundMax = false;

uint8_t spi_xmit(uint8_t val);
void send_package(uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7);
uint8_t getCommand(uint32_t cmd);



ISR(PCINT0_vect) //remote signals
{
  //Serial.println(RADIO_OUT_IS_HIGH,DEC);
  if (RADIO_OUT_IS_HIGH)
  {
    if (capturingstart || capturingbytes)
    {
      captimelo = TCNT1;
    }
    else
      capturingstart = 1;
    TCNT1 = 0;

    //eval times
    if (captimehi > 16600 && captimelo > 7000)
    {
      capturingstart = 0;
      capturingbytes = 1;
      //uart_puts("startseq found\r\n");
    }
    else if (capturingbytes && captimelo > 3000)
    {
      //uart_puts("bit 1\r\n");
      cmd = (cmd << 1) | 0x00000001;
      cmdbit++;
    }
    else if (capturingbytes && captimelo > 1000)
    {
      //uart_puts("bit 0\r\n");
      cmd = (cmd << 1);
      cmdbit++;
    }
    else
    {
      //uart_puts("nothing found\r\n");
    }
    if (cmdbit == 32)
    {
      //Serial.println("COMAND");
      newcmd = 1;
      cmdbit = 0;
      capturingbytes = 0;
    }
  }
  else
  {
    captimehi = TCNT1;
    TCNT1 = 0;
  }
}

uint8_t spi_xmit(uint8_t val)
{
  SPDR = val;
  while (!(SPSR & (1 << SPIF)));
  return SPDR;
}

void myTransfer(uint8_t val) {
  //arduino SPI library
  //SPI.transfer(val);
  //software "spi"
  //  for (uint8_t i = 0; i < 8; i++)  {
  //    digitalWrite(ClockPin, HIGH);
  //    digitalWrite(DataIn, !!(val & (1 << (7 - i))));
  //    //delayMicroseconds(5);
  //    digitalWrite(ClockPin, LOW);
  //    //delayMicroseconds(5);
  //  }
  //hw spi by shyd
  spi_xmit(val);
}

void send_package(uint8_t c0, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7)
{
  myTransfer(c0);
  delayMicroseconds(874);
  myTransfer(c1);
  delayMicroseconds(874);
  myTransfer(c2);
  delayMicroseconds(874);
  myTransfer(c3);
  delayMicroseconds(874);
  myTransfer(c4);
  delayMicroseconds(874);
  myTransfer(c5);
  delayMicroseconds(874);
  myTransfer(c6);
  delayMicroseconds(874);
  myTransfer(c7);
}

void setup()
{
  // pin change interrupt
  PCMSK0 |= bit(PCINT4);  // want pin D12
  PCIFR  |= bit(PCIF0);   // clear any outstanding interrupts
  PCICR  |= bit(PCIE0);   // enable pin change interrupts for ???A0 to A5

  Serial.begin(UART_BAUD_RATE);
  //init SPI - no need for arduino SPI.h
  pinMode(DataIn, OUTPUT);
  pinMode(ClockPin, OUTPUT);
  pinMode(SSPin, OUTPUT); //SS output to enable spi
  //pinMode(PhonePin, OUTPUT); //Telefon
  //digitalWrite(PhonePin, HIGH);
  // SPI Type: Master
  // SPI Clock Rate: 62,500 kHz
  // SPI Clock Phase: Cycle Start
  // SPI Clock Polarity: Low
  // SPI Data Order: MSB First
  SPCR = 0x57;
  SPSR = 0x00;

  //arduino sPI library ... do not forget to uncomeny define SPI.h at top
  //        SPI.begin();
  //        SPI.setBitOrder(MSBFIRST);
  //        SPI.setDataMode(SPI_MODE1);
  //        SPI.setClockDivider(SPI_CLOCK_DIV128); //62.5kHz@8Mhz 125kHz@16MHz

  //beta commands -> cdc
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0;
  TCCR1B |= (1 << CS11);  // no prescaler 8 -> 1 timer clock tick is 0.5us long on 16Mhz
  //EIMSK |= (1 << INT0);
  //EICRA |= (1 << ISC00); //any change on INT0 and INT1
  sei();

  Serial.println("cdc vysila");


}

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {

    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;

    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.println(receivedChars);
    if (strcmp(receivedChars, "TS+01\r") == 0)
    {
      mode = MODE_SCAN;
      //digitalWrite(PhonePin, HIGH);
    }
    if (strcmp(receivedChars, "TS+02\r") == 0)
    {
      mode = MODE_PLAY;
      //digitalWrite(PhonePin, HIGH);
      if (soundMax == false) {
        delay(70);
        Serial.write("AT+CA20\r\n");
        soundMax = true;
      }
    }
    if (strcmp(receivedChars, "TS+03\r") == 0)
    {
      cd = 0xF;
      mode = MODE_SCAN;
      //digitalWrite(PhonePin, LOW);
    }
    newData = false;
  }
}

void loop()
{
  if ((millis() - previousMillis) > PACKET_DALEY) {
    send_package(0x34, 0xBF ^ cd, 0xFF ^ tr, 0xFF, 0xFF, mode, 0xCF, 0x3C);
    previousMillis = millis();
  }

  if ((millis() - previousMillis_prev_c) > 800) {
    prev_c = 0;
  }

  if ((millis() - previousMillis_prev_c) > 2000) {
    if (mode == MODE_PLAY) {
      cd = 0xB;
    }
    if (btOff == true) {
      btOff = false;
      Serial.write("AT+CM00\r\n");
      cd = 0xB;
    }
  }

  recvWithEndMarker();
  showNewData();

  if (newcmd)
  {
    newcmd = 0;
    c = getCommand(cmd);
    if (c != prev_c)
    {
      switch (c)
      {
        case CDC_CD1:
          cd = 1;
          //Serial.write("1\r\n");
          Serial.write("AT+BA04\r\n");  //vezme hovor
          previousMillis_prev_c = millis();
          break;
        case CDC_CD2:
          cd = 2;
          //Serial.write("2\r\n");
          Serial.write("AT+BA02\r\n");  //odmitne hovor
          previousMillis_prev_c = millis();
          break;
        case CDC_CD3:
          cd = 3;
          //Serial.write("3\r\n");
          Serial.write("AT+BA03\r\n");  //polozi zvednuty hovor
          previousMillis_prev_c = millis();
          break;
        case CDC_CD4:
          cd = 4;
          //Serial.write("4\r\n");
          Serial.write("AT+CE\r\n");  //zvedne hlasitost, vypne vyzvaneni? test
          previousMillis_prev_c = millis();
          break;
        case CDC_CD5:
          cd = 5;
          Serial.write("5\r\n");
          previousMillis_prev_c = millis();
          break;
        case CDC_CD6:
          cd = 6;
          Serial.write("6\r\n");
          previousMillis_prev_c = millis();
          break;
        case CDC_SEEK_RWD: //ok vlevo
          cd = 0xB;
          tr = 0x0B;
          Serial.write("AT+CD\r\n");
          previousMillis_prev_c = millis();
          break;
        case CDC_SEEK_FWD: //ok vpravo
          cd = 0xB;
          tr = 0x0F;
          Serial.write("AT+CC\r\n");
          previousMillis_prev_c = millis();
          break;
        case CDC_NEXT: //ok nahoru
          cd = 0xB;
          tr = 0xF0;
          Serial.write("AT+CC\r\n");
          delay(70);
          Serial.write("AT+CC\r\n");
          previousMillis_prev_c = millis();
          break;
        case CDC_PREV: //ok dolu
          cd = 0xB;
          tr = 0xB0;
          Serial.write("AT+CD\r\n");
          delay(70);
          Serial.write("AT+CD\r\n");
          previousMillis_prev_c = millis();
          break;
        case CDC_SCAN:
          cd = 0xB;
          Serial.write("AT+CB\r\n"); //play/pause
          previousMillis_prev_c = millis();
          break;
        case CDC_STOP:
          if (btOff == false) {
            btOff = true;
            Serial.write("AT+CZ\r\n");
            cd = 0x0;
            mode = MODE_SCAN;
            soundMax = false;
          }
          previousMillis_prev_c = millis();
          break;
        default:
          // statements
          //Serial.println(c);
          break;
      }
      prev_c = c;
      c = 0;
    }
  }



}

ISR(TIMER1_OVF_vect)
{
  cmdbit = 0;
}

// check captured bytes
uint8_t getCommand(uint32_t cmd)
{
  if (((cmd >> 24) & 0xFF) == CDC_PREFIX1 && ((cmd >> 16) & 0xFF) == CDC_PREFIX2)
    if (((cmd >> 8) & 0xFF) == (0xFF ^ ((cmd) & 0xFF)))
      return (cmd >> 8) & 0xFF;
  return 0;
}

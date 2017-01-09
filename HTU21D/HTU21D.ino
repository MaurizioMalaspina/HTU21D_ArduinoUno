#include <Wire.h>

#define HTU21D_I2C_ADDRESS              (uint8_t)0x40

#define SOFT_RESET_CMD                  byte(0xFE)
#define TRIGGER_T_MEAS_NO_HOLD_CMD      byte(0xF3)

uint8_t DATA_MSB, DATA_LSB, CHECKSUM;
float Temp;
int16_t Temp_binary;
uint16_t TimeOut;
uint8_t CRC_CHECK;

void setup() {

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  delay(5);

  Wire.begin(); // Init I²C bus (no address for master mode)
 
   // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
                            // Wait T>15ms in order to let time to reset internal circuitry
  delay(1000);

     // ISSUE SOFT RESET COMMAND
  Wire.beginTransmission(HTU21D_I2C_ADDRESS); // Transmit to slave device HTU21D(F)
  Wire.write(SOFT_RESET_CMD);                 // sends instruction byte
  Wire.endTransmission();                     // stop transmitting
  delay(50); 

  Serial.println("SOFT RESET DONE");
}

void loop() {

  Wire.beginTransmission(HTU21D_I2C_ADDRESS); // Transmit to slave device HTU21D(F)
  Wire.write(TRIGGER_T_MEAS_NO_HOLD_CMD);     // sends instruction byte
  Wire.endTransmission();                     // stop transmitting
  
  delay(2000); 

  Wire.requestFrom(HTU21D_I2C_ADDRESS, (uint8_t)3);    // request 3 bytes from slave device
  TimeOut = 65536;
  while ( (Wire.available()<3) && TimeOut--); 
  
/*
  TimeOut = 65536;
  do
  {
    Wire.requestFrom(HTU21D_I2C_ADDRESS, (uint8_t)3);    // request 3 bytes from slave device
  }
  while ( (Wire.available()<3) && TimeOut--); 
*/
  if(Wire.available()>=3)
  {
    DATA_MSB = Wire.read();  
    DATA_LSB = Wire.read();  
    CHECKSUM = Wire.read();
    Temp_binary = ((((uint16_t)DATA_MSB)<<8)+((uint16_t)DATA_LSB));
    CRC_CHECK = check_crc(Temp_binary,CHECKSUM); 
    Temp_binary &= ~0x0003;
    Temp = -46.85 + (175.72/65536)*(float)Temp_binary;
     
    Serial.println("****************");
    Serial.print("Frame rx: ");
    Serial.print(DATA_MSB, HEX);
    Serial.print(DATA_LSB, HEX);
    Serial.println(CHECKSUM, HEX);
    Serial.print("T binary: ");
    Serial.println(Temp_binary, HEX);
    if(!CRC_CHECK) 
      Serial.println("CRC OK");
     else
      Serial.println("CRC FAILED");
    Serial.print("T [Cdeg]: ");
    Serial.println(Temp);

  }
  else
  {
    Serial.println("NO REPLY RX");

    Wire.beginTransmission(HTU21D_I2C_ADDRESS); // Transmit to slave device HTU21D(F)
    Wire.write(SOFT_RESET_CMD);                 // sends instruction byte
    Wire.endTransmission();                     // stop transmitting
    delay(50); 

    Serial.println("SOFT RESET DONE");  
  }
}

//Give this function the 2 byte message (measurement) and the check_value byte from the HTU21D
//If it returns 0, then the transmission was good
//If it returns something other than 0, then the communication was corrupted
//From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
//POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

uint8_t check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  //Test cases from datasheet:
  //message = 0xDC, checkvalue is 0x79
  //message = 0x683A, checkvalue is 0x7C
  //message = 0x4E85, checkvalue is 0x6B

  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
  remainder |= check_value_from_sensor; //Add on the check value

  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {
    //Serial.print("remainder: ");
    //Serial.println(remainder, BIN);
    //Serial.print("divsor:    ");
    //Serial.println(divsor, BIN);
    //Serial.println();

    if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return (uint8_t)remainder;
}

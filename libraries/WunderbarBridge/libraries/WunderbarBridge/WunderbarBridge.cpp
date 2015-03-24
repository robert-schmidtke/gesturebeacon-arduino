/* Library for connect Arduino to the Wunderbar Bridge Module 
** Author: Daniel Mancuso
** Relayr.io
*/

#include "Arduino.h"
#include "WunderbarBridge.h"


#include <SoftwareSerial.h>
//#define BRIDGE_DEBUG true


SoftwareSerial mySerial(10, 11);

Bridge::Bridge(){
      useDebugOutput = false;
}

Bridge::Bridge(uint8_t rx_pin, uint8_t tx_pin, int32_t baudrate){
      _baudrate = baudrate;
      _rx_pin = rx_pin;
      _tx_pin = tx_pin;
      useDebugOutput = true;
      SoftwareSerial mySerial(_rx_pin, _tx_pin); // RX (default 10), TX (default 11)*/
}

//------------------------------------------------------------------------------
/**
 *    Initialize Serial port and print welcome message
 *
 * \param
 *
 * \return True if a Bridge Connection was detected
 *         False if no answer received from the Bridge
 */
bool Bridge::begin()
{
  /* 115200 is the default baudrate for the Bridge 
    (could be changed in the BLE Config char)*/
  Serial.begin(115200); 

  if (useDebugOutput){
    //SoftwareSerial mySerial(_rx_pin, _tx_pin); // RX (default 10), TX (default 11)
    //initialize Soft Serial UART: 
    mySerial.begin(_baudrate);

  	mySerial.println("\n\r\n\r------------------------------- \
                          \n\r relayr - bring things to life  \
                          \n\r Arduino / WunderBar-Bridge lib \
                          \n\r-------------------------------\n\r");
  } 	
  return checkConnection();
}

//------------------------------------------------------------------------------
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void Bridge::processSerial(void)
{
  //mySerial.println("processSerial");//******************debug
  while (Serial.available())
  {
    // get the new byte:
    uint8_t inChar = (uint8_t) Serial.read(); 

    //mySerial.println(inChar);//******************debug

    down.rec_bytes++;
    
    if (down.rec_bytes == 1) //1: command
    {
      down.channel.command = inChar;

      switch (inChar)
      {
          case BRIDGE_COMM_ACK:
            down.rec_bytes = 0;
            commandReceived = true;
            break;

          case BRIDGE_COMM_NACK:
            down.rec_bytes = 0;
            commandReceived = true;
            break;

          case BRIDGE_COMM_NCONN:
            down.rec_bytes = 0;
            commandReceived = true;

            if (useDebugOutput){
              mySerial.println("\n\n\rError: Bridge Module not connected to the Wunderbar");
            }
            break;

          case BRIDGE_COMM_PING:
            Serial.write(BRIDGE_COMM_ACK);  
            down.rec_bytes = 0;
            commandReceived = true;
            break;  

          case BRIDGE_COMM_RCV_FROM_BLE:
            break;  

          default:
            down.rec_bytes = 0;
            down.packet_ok = false;
      }
    }
    
    if (down.rec_bytes == 2) 
        down.channel.length = inChar; //2: Lenght
    
    if ((down.rec_bytes > 2) && (down.rec_bytes <= (down.channel.length + 2)))  //3:Start Payload
    {
      down.channel.payload[down.payload_c] = inChar;
      down.payload_c++;
    }
    
    if (down.rec_bytes == down.channel.length + 3) 
        down.channel.crc = inChar; //CRC16 LSB
    
    if (down.rec_bytes == down.channel.length + 4)  //CRC16 MSB
    {
      down.channel.crc += (inChar << 8);
      
      uint16_t c_crc = crc16Compute((uint8_t *) &down.channel, down.channel.length+2, NULL);
      
      if (c_crc == down.channel.crc)
      {
        down.packet_ok = true;
      } else 
            down.packet_ok = false;
    
      commandReceived = true;
      newData = true;

      //reset counters
      down.rec_bytes = 0;
      down.payload_c = 0;

      if (useDebugOutput){
        mySerial.println("\n\n\r<<<== Packet received:");

        if (down.packet_ok) 
            mySerial.print("CRC OK");
          
        dumpPacket(down.channel); 
      }
    }
  }
}

//------------------------------------------------------------------------------
/**
 * Sends a packet over the Hardware Serial port (TX)
 *
 * \param: byte array with the payload and payload size
 *
 * \return 
 */
bool Bridge::sendData(uint8_t payload[], int size)
{
  //prepares the packet
  bridge_comm_t outPacket = createUpPacket(payload, sizeof(payload), outputBuffer);

  if (!_bridgeConnected){
    if (!checkConnection()) return false;
  } 

   // memcpy((char *) &outPacket, outputBuffer, sizeof(outPacket));//outPacket.length + 4);
  if (useDebugOutput){    
    mySerial.println("\n\r==>>> OutPacket:");
    dumpPacket(outPacket);

    mySerial.print("output Buffer: "); 
    for (char i=0; i < outPacket.length+4; i++)
    {
      mySerial.print(outputBuffer[i], HEX);
      mySerial.print(",");
    }
  }
  //sends the packet to the UART  
  Serial.write((uint8_t *)outputBuffer, outPacket.length + 4); 

  return true;
}

//------------------------------------------------------------------------------
/**
 *    Calculate a CRC (16 bits) of a packet
 *
 * \params: pointer to the data, data size, initialization word (NULL by default)
 *
 * \return: the calculated CRC 
 */
uint16_t Bridge::crc16Compute(uint8_t * p_data, int size, uint8_t * p_crc)
{
  uint32_t i;
  uint16_t crc = (p_crc == NULL) ? 0xffff : *p_crc;

  for (i = 0; i < size; i++)
  {
    crc = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= p_data[i];
    crc ^= (uint8_t)(crc & 0xff) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0xff) << 4) << 1;
  }    
  return crc;
}

//------------------------------------------------------------------------------
/**
 *   Creates an Up Data Packet 
 *
 * \params: pointer to the payload, payload length, pointer to the output buffer.
 *
 * \return: the bridge_comm object 
 */
bridge_comm_t Bridge::createUpPacket(uint8_t * payload, int length, uint8_t * outBuffer)
{
  bridge_comm_t packet;

  packet.command = BRIDGE_COMM_WRITE_UP_CHANNEL;
  packet.length = length;

  memcpy(packet.payload, payload, length);

  outBuffer[0] = packet.command;
  outBuffer[1] = packet.length;
  
  for (char i = 2; i < length + 2; i++)
  {
    outBuffer[i] = packet.payload[i-2];
  }

  packet.crc = crc16Compute(outBuffer, length + 2, NULL);

  outBuffer[length + 2] = (packet.crc & 0xff);
  outBuffer[length + 3] = (packet.crc >> 8);

  return packet;
}

//------------------------------------------------------------------------------
/**
 *   Returns a struct with received payload
 * \param: A pointer to the array where to put the data
 *
 * \return none
 */
bridge_payload_t Bridge::getData(void)
{
  bridge_payload_t rx_payload;

  memcpy(rx_payload.payload, down.channel.payload, down.channel.length);
  rx_payload.length = down.channel.length; 
  newData = false;

  return rx_payload;
}

//------------------------------------------------------------------------------
/**
 *   Print the details of a packet in the Software Serial port
 * \param: A bridge_comm Packet
 *
 * \return none
 */
void Bridge::dumpPacket(bridge_comm_t packet)
{
  if (useDebugOutput){
    mySerial.print("\n\r-----------\n\r");
    mySerial.print("Packet size: ");
    mySerial.println(packet.length+4);
    mySerial.print("CMD: ");
    mySerial.println(packet.command, HEX);
    mySerial.print("Payload length: ");
    mySerial.println(packet.length, DEC);
    mySerial.print("Payload: ");

    for (char i = 2; i < packet.length+2; i++)
    {
      mySerial.print(packet.payload[i-2], HEX);
      mySerial.print(", ");
    }  

    mySerial.print("\n\rCRC: ");
    mySerial.print(packet.crc, HEX);
    mySerial.print("\n\r-----------\n\r");
  } 
}

//------------------------------------------------------------------------------
/**
 * Sends a PING command to the Bridge and wait for the response
 *
 * \param
 *
 * \return True if gets the ACK from the Bridge
 *         False if no command received 
 */
bool Bridge::checkConnection()
{
  Serial.write(BRIDGE_COMM_PING);  
  
  for (uint8_t i = 0; i < 10; ++i)
  {
    if (Serial.available()){
      processSerial();
      break;
    }
    delay(10);
  }

  if (!commandReceived || (down.channel.command != BRIDGE_COMM_ACK))
  {
    if (useDebugOutput){
      mySerial.println("\n\n\r *** Could not find a valid Bridge module, check wiring and that the Brige has the UART firmware.\n\r\
       Arduino TX ==>> Bridge RX (white wire)\n\r\
       Arduino RX ==>> Bridge TX (Yellow wire)");
    }
   _bridgeConnected = false;        
   return false;
  }
  _bridgeConnected = true;
  return true;
}  


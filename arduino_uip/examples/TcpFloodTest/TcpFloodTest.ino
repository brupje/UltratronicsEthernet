/*
 * Server will accept data and send it back to the client.
 */

#include <UIPEthernet.h>

EthernetServer server = EthernetServer(8876);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("Start!");
  
  uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};
  IPAddress myIP(192,168,2,45);

  Ethernet.begin(mac,myIP);

  server.begin();
}

long nextmillis = 0;
void loop()
{
  size_t size;

  if (EthernetClient client = server.available())
    {
      while((size = client.available()) > 0)
        {
          uint8_t* msg = (uint8_t*)malloc(size);
          size = client.read(msg,size);
          Serial.write(msg,size);
          client.write(msg,size);
          client.println("");
          free(msg);
        }
      
    }

    
}

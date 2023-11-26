#include <UniversalTelegramBot.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

  const char* ssid     = "Redmi Note 8T";
  const char* password = "tdraaron";
  #define BOT_TOKEN "6811226934:AAFDBROB0UXVLFAxXUewVEaHD1f8k1VPrsI"
  #define CHAT_ID "5523505509"

  WiFiClientSecure client;
  UniversalTelegramBot bot(BOT_TOKEN, client);

  bool sent = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.begin(ssid,password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org

   while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
  /*if (acera==true){
    infr[0] ++;
  }
  if (velocidad==true){
    infr[1] ++;
  }
  if (sentido==true){
    infr[2] ++;
  }*/
  int infr[3] = {0,2,9};
  String text[3] = {" infraccion(es) de ir por la acera, ", " infraccion(es) de velocidad y ", " infraccion(es) de ir en sentido contrario."};
  int indexs = 12;
  String infracciones = "Has cometido ";
  if (WiFi.status()==WL_CONNECTED && sent == false){
    sent = true;
    for (int i = 0; i < 3; i++){

      infracciones.concat(infr[i]);
      infracciones.concat(text[i]);

    }
    bot.sendMessage(CHAT_ID, infracciones, "");
    Serial.print(infracciones);
  }
  sleep(1);
}

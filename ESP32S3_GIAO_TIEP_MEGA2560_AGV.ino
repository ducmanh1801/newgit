#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPUpdateServer.h>

const char* ssid = "TOT"; //192.168.79.120
const char* password = "TNG12345";
//const char* ssid = "crown"; //your wifi ssid
//const char* password = "crown2015";//your wifi password

const char * udpAddress = "10.0.21.152"; // enter your ip destination IP here
const char* host = "VNSOFTWARE";
const char* updatePath = "/update";
const char* updateUsername = "admin";
const char* updatePassword = "admin";

//const char * udpAddress_master = "192.168.0.199"; // Master2
const char * udpAddress_master = "192.168.0.198"; // Master1
const int Y0 = 6;
const int Y1 = 7;
const int Y2 = 15;
const int Y3 = 16;

const int X0 = 39;
const int X1 = 38;
const int X2 = 37;
const int X3 = 36;
const int X4 = 35;
const int X5 = 45;
const int X6 = 48;
const int X7 = 47;

void gui_trangthaiagv_khi_khoi_dong();
void gui_trangthaiagv_khi_di_chuyen(int ten_line);
void gui_vitri_agv(String t);
void AGV01_vetram();
void AGV01_dichuyen();
void  gui_mega2560();
void doc_mega2560_VT();
void nhan_data();
int kiemtrabien();
int line=0,line1=0,line2=0,line3=0,line4=0,line5=0,line6=0,line7=0,line8=0,line9=0,line10=0,linevetram=0;
int        line11=0,line12=0,line13=0,line14=0,line15=0,line16=0,line17=0,line18=0,line19=0,line20=0;
int AGV01=4; // nghĩa là xe đang bận à
int a=0; 
int k=0;
int i=0,j=0;
int b=0;
int b1=0;
bool Chuyen[50]={0};
uint8_t buffer[50];
String chuoi, chuoi1;
byte moc1,moc;
int giatri1=-1;
WebServer webServer(80);
HTTPUpdateServer httpUpdater;
//-----------------------------------------//
const char MainPage[] PROGMEM = R"=====(
  <!DOCTYPE html> 
  <html>
   <head> 
       <title>VNSOFTWARE-SMART HOME</title> 
       <style> 
          body
          {
            text-align: center;
          }
       </style>
       <meta name="viewport" content="width=device-width,user-scalable=0" charset="UTF-8">
   </head>
   <body> 
      <div>
        <img src='https://i.pinimg.com/originals/2a/40/ec/2a40ec0333897aa3309ffbfd5bff47d5.jpg' height='200px' width='330px'>
      </div>
      <div>
        <button onclick="window.location.href='/update'">UPLOAD FIRMWARE</button><br><br>
        <a href='http://muthoidai.org'>Xem Web Của Tôi</a>
      </div>
      <script>
      </script>
   </body> 
  </html>
)=====";

WiFiUDP udp;
unsigned int udpPort=8080; // Server port
IPAddress staticIP(192,168,0,191); // địa chỉ tbị AGV01
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
IPAddress dns1(192,168,0,1);
 /* Gắn cho ESP32 IP tĩnh bat wifi dien thoai*/
 /*IPAddress staticIP(192,168,186,100);
 IPAddress gateway(192,168,186,58);
 IPAddress subnet(255,255,255,0);
 IPAddress dns1(192,168,186,58);*/

char pktbuf[10]; //buffer to store udp data
char rx_val;
void setup() 
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 18, 17);//RX1,TX1
  WiFi.begin(ssid, password);
  WiFi.config(staticIP, gateway, subnet,dns1);
  Serial.println("");
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
  Serial.println("Connection Failed! Rebooting...");
  delay(5000);
  ESP.restart();
  }
  if(!MDNS.begin("esp32"))
  {
      Serial.println("Error starting mDNS");
      return;
    }
    MDNS.begin(host);
    MDNS.addService("http", "tcp", 80);
    Serial.println(WiFi.localIP());
    Serial.println("Status: Connected");
    httpUpdater.setup(&webServer, updatePath, updateUsername, updatePassword);
    webServer.on("/",[]{
      String s = MainPage;
      webServer.send(200,"text/html",s);
    });
    
    webServer.begin();
    Serial.println("Web Server is started!");
  /* Enable udp */
  udp.begin(udpPort);
  Serial.println(udpPort);

    pinMode(Y0, OUTPUT);
    pinMode(Y1, OUTPUT);
    pinMode(Y2, OUTPUT);
    pinMode(Y3, OUTPUT);
    pinMode(X0, INPUT_PULLUP);
    pinMode(X1, INPUT_PULLUP);
    pinMode(X2, INPUT_PULLUP);
    pinMode(X3, INPUT_PULLUP);
    pinMode(X4, INPUT_PULLUP);
    pinMode(X5, INPUT_PULLUP);
    pinMode(X6, INPUT_PULLUP);
    pinMode(X7, INPUT_PULLUP);
    // Set outputs to LOW
    digitalWrite(Y0, LOW);
    digitalWrite(Y1, LOW);
    digitalWrite(Y2, LOW);
    digitalWrite(Y3, HIGH); // den bao ket noi wifi thanh cong
    //khi khoi dong thi gui du lieu bao master AGV da san sang
    // Binh xóa hàm gửi "AGV01_davetram" luc mới bật điện xe lên------------------------7.3.2024---------------------------------------
    //gui_trangthaiagv_khi_khoi_dong(); // sẽ gửi trạng thái AGV sẵn sàng
  }

void loop()
{
 unsigned long conditionMettime;
  webServer.handleClient();
  nhan_data();
  gui_mega2560();
  AGV01_vetram();
  //Serial.print("gia tri X2: ");
 // Serial.println(digitalRead(X2));
  if(digitalRead(X2)==0)
  {
    AGV01_dichuyen();
    //if(millis()-conditionMettime>2500)
  }

  doc_mega2560_VT();
  ketnoi_lai_wifi();

  //String str_ = String((char *)buffer);
  //Serial.println(str_);  
} 
void ketnoi_lai_wifi()
{
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    digitalWrite(Y3, LOW);
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  digitalWrite(Y3, HIGH);
}
/*
void blinkTask(void *pvParameters) 
{
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(ledPin, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay 500ms
      digitalWrite(ledPin, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);  // Delay 500ms
    } else {
      digitalWrite(ledPin, LOW);  // Tắt đèn khi không có kết nối WiFi
      vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1000ms
    }
  }
}
*/
void nhan_data()
{
  webServer.handleClient();
  uint8_t buffer[50];
  memset(buffer, 0, 50);
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read(buffer, 50) > 0)
  {
    digitalWrite(Y0, HIGH);
    String str = String((char *)buffer);
    // nhan du lieu tu line goi
    if(str=="line1")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L1" ;
        udp.write(buffer3,15);
        udp.endPacket();
        line=1;
      }
    else if(str=="line2")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L2" ;
        udp.write(buffer3,15);
        udp.endPacket();        
        line=2;
      }
    else if(str=="line3")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L3" ;
        udp.write(buffer3,15);
        udp.endPacket();     
        line=3;
      }
    else if(str=="line4")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L4" ;
        udp.write(buffer3,15);
        udp.endPacket();      
        line=4;
      }
    else if(str=="line5")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L5" ;
        udp.write(buffer3,15);
        udp.endPacket();     
        line=5;
      }
    else if(str=="line6")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L6" ;
        udp.write(buffer3,15);
        udp.endPacket();     
        line=6;
      }
    else if(str=="line7")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L7" ;
        udp.write(buffer3,15);
        udp.endPacket(); 
        line=7;
      }
    else if(str=="line8")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L8" ;
        udp.write(buffer3,15);
        udp.endPacket();   
        line=8;
      }
    else if(str=="line9")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L9" ;
        udp.write(buffer3,15);
        udp.endPacket();  
        line=9;
      }  
    else if(str=="line10")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L10" ;
        udp.write(buffer3,16);
        udp.endPacket();  
        line=10;
      }                                                   
    else if(str=="line11")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L11" ;
        udp.write(buffer3,16);
        udp.endPacket();  
        line=11;
      }
    else if(str=="line12")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L12" ;
        udp.write(buffer3,16);
        udp.endPacket();   
        line=12;
      }
    else if(str=="line13")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L13" ;
        udp.write(buffer3,16);
        udp.endPacket(); 
        line=13;
      }
      else if(str=="line14")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L14" ;
        udp.write(buffer3,16);
        udp.endPacket();  
        line=14;
      }
      else if(str=="line15")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L15" ;
        udp.write(buffer3,16);
        udp.endPacket();  
        line=15;
      }
      else if(str=="line16")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L16" ;
        udp.write(buffer3,16);
        udp.endPacket();  
        line=16;
      }
      else if(str=="line17")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L17" ;
        udp.write(buffer3,16);
        udp.endPacket(); 
        line=17;
      }
      else if(str=="line18")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L18" ;
        udp.write(buffer3,16);
        udp.endPacket();
        line=18;
      }
      else if(str=="line19")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L19" ;
        udp.write(buffer3,16);
        udp.endPacket(); 
        line=19;
      }
      else if(str=="line20")
      {
        Serial.println(str);  
        udp.beginPacket(udpAddress_master, udpPort); //gui master
        uint8_t buffer3[50] = "AGV01_danhan_L20" ;
        udp.write(buffer3,16);
        udp.endPacket(); 
        line=20;
      }
     // ve tram
      if(str=="master_danhan_agv_vetram") //master_danhan_agv_vetra
       {
          Serial.println(str); 
          b=0;
          digitalWrite(Y0, LOW);
        }
        // nhan du lieu AGV bat dau di chuyen -> phai viet du cho cac chuyen.....................................................
      if(str=="master_danhan_agv_bd1")//master_danhan_agv_bd1
        {if(Chuyen[1]==1){Chuyen[1]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd2")//master_danhan_agv_bd1
        {if(Chuyen[2]==1){Chuyen[2]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd3")//master_danhan_agv_bd1
        {if(Chuyen[3]==1){Chuyen[3]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd4")//master_danhan_agv_bd1
        {if(Chuyen[4]==1){Chuyen[4]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd5")//master_danhan_agv_bd1
        {if(Chuyen[5]==1){Chuyen[5]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd6")//master_danhan_agv_bd1
        {if(Chuyen[6]==1){Chuyen[6]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd7")//master_danhan_agv_bd1
        {if(Chuyen[7]==1){Chuyen[7]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd8")//master_danhan_agv_bd1
        {if(Chuyen[8]==1){Chuyen[8]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd9")//master_danhan_agv_bd1
        {if(Chuyen[9]==1){Chuyen[9]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd10")//master_danhan_agv_bd1
        {if(Chuyen[10]==1){Chuyen[10]=0;b1=0;line=0;}}  
              //        
      if(str=="master_danhan_agv_bd11")//master_danhan_agv_bd1
        {if(Chuyen[11]==1){Chuyen[11]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd12")//master_danhan_agv_bd1
         {if(Chuyen[12]==1){Chuyen[12]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd13")//master_danhan_agv_bd1
        {if(Chuyen[13]==1){Chuyen[13]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd14")//master_danhan_agv_bd1
         {if(Chuyen[14]==1){Chuyen[14]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd15")//master_danhan_agv_bd1
        {if(Chuyen[15]==1){Chuyen[15]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd16")//master_danhan_agv_bd1
        {if(Chuyen[16]==1){Chuyen[16]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd17")//master_danhan_agv_bd1
        {if(Chuyen[17]==1){Chuyen[17]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd18")//master_danhan_agv_bd1
        {if(Chuyen[18]==1){Chuyen[18]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd19")//master_danhan_agv_bd1
        {if(Chuyen[19]==1){Chuyen[19]=0;b1=0;line=0;}}
      if(str=="master_danhan_agv_bd20")//master_danhan_agv_bd1
        {if(Chuyen[20]==1){Chuyen[20]=0;b1=0;line=0;}}                                                                         
      /*                                            
      
      */
      else 
    {
      Serial.print("Server to client: ");
      Serial.println((char *)buffer);
    }
  }
    delay(100);
    /*
    b1=0;
      for (int i = 0; i < 50; i++) {
        if (Chuyen[i] != 0) {
          // Nếu có ít nhất một phần tử khác 0, đặt b1 thành 1 và thoát khỏi vòng lặp
          b1=1;
          break;
        }
      }
      */
}

//code cu chay tu line 1-5 ngon (6/3/2024) dang chay ngon-------------------------------------------------------------------------------------
/*
void gui_mega2560()
{
  while (line==1)
  {
    digitalWrite(Y0, HIGH);
    Serial1.print("#1A"); // gui line 1
    Serial.println("dang gui line1"); // gui line 1
    i=0;
    for(i=0;i<200;i++)
    {
      nhan_data();
      //delay(1);
      Serial.println(i);
      Serial.print("Gia tri line: ");
      Serial.println(line);
      if(digitalRead(X0)==0)
      {
        line=0;
        digitalWrite(Y0, LOW);
        Serial.println("danhan_mega"); // gui line 1
        break;
      }
    }
  }
  while (line==2)
  {
    digitalWrite(Y0, HIGH);
    Serial1.print("#2A"); // gui line 2
    Serial.println("dang gui line2"); // gui line 2
    i=0;
    for(i=0;i<200;i++)
    {
      nhan_data();
      //delay(1);
      Serial.println(i);
      if(digitalRead(X0)==0)
      {
        while(digitalRead(X0)==0)
        //Serial.println("test hhii");
        line=0;
        digitalWrite(Y0, LOW);
        Serial.println("danhan_mega");
        break;
      }
    }
  }
  while (line==3)
  {
    digitalWrite(Y0, HIGH);
    Serial1.print("#3A"); // gui line 3
    Serial.println("dang gui line3"); // gui line 3
    i=0;
    for(i=0;i<200;i++)
    {
      nhan_data();
      //delay(1);
      Serial.println(i);
      if(digitalRead(X0)==0)
      {
        line=0;
        digitalWrite(Y0, LOW);
        Serial.println("danhan_mega");
        break;
      }
    }
  }
  while (line==4)
  {
    digitalWrite(Y0, HIGH);
    Serial1.print("#4A"); // gui line 4
    Serial.println("dang gui line4"); // gui line 4
    i=0;
    for(i=0;i<200;i++)
    {
      nhan_data();
      //delay(1);
      Serial.println(i);
      if(digitalRead(X0)==0)
      {
        line=0;
        digitalWrite(Y0, LOW);
        Serial.println("danhan_mega");
        break;
      }
    }
  }
  while (line==5)
  {
    digitalWrite(Y0, HIGH);
    Serial1.print("#5A"); // gui line 5
    Serial.println("dang gui line5"); // gui line 5
    i=0;
    for(i=0;i<200;i++)
    {
      nhan_data();
      //delay(1);
      Serial.println(i);
      if(digitalRead(X0)==0)
      {
        line=0;
        digitalWrite(Y0, LOW);
        Serial.println("danhan_mega");
        break;
      }
    }
  }
}
*/
void xulyline (int giatri)
{
  digitalWrite(Y0, HIGH);
  String newStr1;
  newStr1= String('#') + String(giatri) + String('A');
  Serial1.print(newStr1);
  Serial.println(newStr1); // check point
  Serial.print("dang gui line "); // gui 
  Serial.println(giatri); // gui line 1
   i=0;
    for(i=0;i<200;i++)
    {
      nhan_data();
      //delay(1);
      Serial.println(i);
      Serial.print("Gia tri line: ");
      Serial.println(line);
      Serial.println(newStr1);
      if(digitalRead(X0)==0)
      {
        line=0;
        AGV01=4;
        digitalWrite(Y0, LOW);
        Serial.println("danhan_mega"); // 
        break;
      }
    }
}
void gui_mega2560()
{
  // nếu xe đang rảnh, ở trạm mới gọi hàm xử lý line
    //Serial.println("checkpoint1");
    //Serial.print("line: ");
    //Serial.println(line);
    //Serial.print("AGV01: ");   
    //Serial.println(AGV01);
   while ((line!=0)&&(AGV01==1))
  {
    xulyline (line);
  }
}
// ham doc Mega-Binh 4.11
/*
int doc_mega2560_test()
{
   int combinedValue=-1;
  if (Serial1.available() > 0) 
  {
    String receivedString = Serial1.readStringUntil('\n'); // Đọc chuỗi đến khi gặp ký tự xuống dòng '\n'

    // Kiểm tra xem chuỗi có ít nhất 3 ký tự không
    if (receivedString.length() >= 2) 
    {
      // Lấy ký tự thứ 2 và thứ 3 từ chuỗi
      char firstCharacter = receivedString.charAt(0);
      if(firstCharacter=='$')
      {
        char secondCharacter = receivedString.charAt(1);
        char thirdCharacter = receivedString.charAt(2);
        
        // Ghép ký tự thứ 2 và thứ 3 thành một số nguyên
        combinedValue = (secondCharacter - '0') * 10 + (thirdCharacter - '0');

        // In ra giá trị số nguyên được ghép từ ký tự thứ 2 và thứ 3
        //Serial.print("Giá trị số nguyên từ ký tự thứ 2 và 3: ");
        //Serial.println(combinedValue);
        return (combinedValue);
        Serial.println("test doc mega ok");
      }
    }
  }
}
*/
/*
int doc_mega2560_test() {
  int result=-1;
  if ((Serial1.available() >2)&& (Serial1.available() <4))
  {
    char ky_tu_thu_nhat = Serial1.read(); // Đọc giá trị từ Serial1 vào ky_tu_thu_nhat
    if (ky_tu_thu_nhat == '$')
    {
    char ky_tu_thu_hai = Serial1.read();
    char ky_tu_thu_ba = Serial1.read();
    if (isdigit(ky_tu_thu_hai) && isdigit(ky_tu_thu_ba)) 
      {
        int hangchuc = ky_tu_thu_hai - '0';
        int hangdv = ky_tu_thu_ba - '0';
        result = hangchuc * 10 + hangdv;
      }
    }
  return result;
  }
}
/*
//

/*
int doc_mega2560_VT()
  {

    if (Serial1.available() >1) 
    {
      char ky_tu_thu_nhat;
      ky_tu_thu_nhat=input.charAt(0);
      if(ky_tu_thu_nhat=='!')
      {
        char ky_tu_thu_hai2 = input.charAt(1);
        char ky_tu_thu_ba2 = input.charAt(2);
        char ky_tu_thu_bon2 = input.charAt(3);
        char ky_tu_thu_nam2 = input.charAt(4);
        if (isdigit(ky_tu_thu_hai2) && isdigit(ky_tu_thu_ba2)&& isdigit(ky_tu_thu_bon2)&& isdigit(ky_tu_thu_nam2)) 
        {
          String CombinedSTR= String(ky_tu_thu_hai2) + String(ky_tu_thu_ba2) + String(ky_tu_thu_bon2) + String(ky_tu_thu_nam2);
          Serial.print("vi tri:");
          Serial.println(CombinedSTR);
          gui_vitri_agv(CombinedSTR);
        }
      }
    }
  }*/
//10.1.24
int doc_mega2560_test()
  {
    int sum=-1;
    if (Serial1.available() >1) 
    {
      String input = Serial1.readStringUntil('\n');
      //String input = Serial1.readString();
      char ky_tu_thu_nhat2;
      ky_tu_thu_nhat2=input.charAt(0);
      if(ky_tu_thu_nhat2=='$')
      {
        char ky_tu_thu_hai2 = input.charAt(1);
        char ky_tu_thu_ba2 = input.charAt(2);
        if (isdigit(ky_tu_thu_hai2) && isdigit(ky_tu_thu_ba2)) 
        {
          sum=(ky_tu_thu_hai2-48)*10+(ky_tu_thu_ba2-48);
        }
      }
      return sum;
    }
  }
void doc_mega2560_VT() {
  if (Serial1.available() > 3) {
    char ky_tu_thu_nhat2 = Serial1.read(); // Đọc giá trị từ Serial1 vào ky_tu_thu_nhat
    if (ky_tu_thu_nhat2 == '!') {
      String CombinedSTR = "";
      for (int i = 0; i < 4; ++i) {
        while (!Serial1.available()) {} // Chờ cho đến khi có đủ số ký tự cần đọc
        CombinedSTR += (char)Serial1.read(); // Đọc giá trị từ Serial1 vào chuỗi CombinedSTR
      }

      Serial.print("vi tri:");
      Serial.println(CombinedSTR);
      gui_vitri_agv(CombinedSTR);
    }
  }
}
//
void AGV01_vetram()
{
  if(digitalRead(X1)==0)
  {
    delay(200);
    while(digitalRead(X1)==0)
    {
      Serial.println("X1 = 0 ");
      Serial1.println("100F"); //gửi cho mega esp32 đã nhận dữ liệu về trạm ORG
      //Serial1.print("100F"); //gửi cho mega esp32 đã nhận dữ liệu về trạm (Binh update)
      delay(2000);
      b=1;
      // Binh them trang thai xe đang rảnh vào code
      AGV01=1;
      while((b==1)&&(digitalRead(X1)==1))
      {
        Serial.println("X1 = 1 ");
        digitalWrite(Y0, HIGH);
        
        udp.beginPacket(udpAddress_master, udpPort); // gửi toi master
        uint8_t buffer3[50] = "AGV01_davetram" ;
        udp.write(buffer3,14);
        udp.endPacket();

        j=0;
        for(j=0;j<50;j++)
        {
          nhan_data();
          Serial.println(j);
          if(b==0) break;
        }
      }
    }
  }
}
// phai viet 1 ham gui chuoi send xuong AGV, AGV doc duoc chuoi send thì dung gui giatri1. 
void AGV01_dichuyen()
{  
  int giatri_previous=0;
  if(digitalRead(X2)==0)
  {
    delay(200);
    while((giatri_previous<=0) && (digitalRead(X2)==0))
    {
        giatri1 = doc_mega2560_test();
        giatri_previous=giatri1;
        Serial.println("wating..");
        Serial.println(giatri1);
        if (giatri1>0)
        {
          digitalWrite(Y1, HIGH); // chân 48 ARDUINO => KO GỬI DATA LÊN NỮA.
          break;
        }
    }
  }
  delay(50);
  digitalWrite(Y1, LOW);
  Serial.print("gia tri:");
  Serial.println(giatri1);
    //Serial1.println("sd");

  if(giatri1>0)
  {
    Chuyen[giatri1]=1;
    gui_trangthaiagv_khi_di_chuyen(giatri1);
  }
  while(Chuyen[giatri1]==1)
  {
    nhan_data();
    Serial.println("ket");
  }
// trong chương trình của AGV sẽ gửi chuyen[1] sau đó gửi chuyen[2] thì không lỗi. Tuy nhiên nếu gửi đồng thời chuyen[1] và chuyen[2] thì sẽ xảy ra lỗi.
// Vì điều kiện của xe AGV được đi là gửi chuyen[1] sau đó sẽ kẹt trong hàm gui_ESP32S3 đến khi nào xác nhận xong (set chân Y2 của esp lên mức cao) chuyền 1 thì gửi chuyen[2] rồi lại kẹp trong hàm
// Trong khi đó code mình đang viết ở đây là nhận đồng thời 2 chuyền và xử lý đồng thời. Nên đoạn check tổng này có vẻ hơi thừa => khi xe gửi 2 chuyền đồng thời đến sẽ xảy ra lỗi kẹp ở hàm tổng này. Tổng không thể = 0 được.
  int tong=0;
  for (int i=0;i<50;i++)
    {
      tong = tong + Chuyen[i];
    }
  Serial.println("gia tri bien: ");
  Serial.println(tong);
  // code mới fix
  if(Chuyen[giatri1]==0)
    {
      digitalWrite(Y2, HIGH); // CHÂN 47 KÉO VỀ MỨC 0, BÁO CHO MEGA ĐÃ HOÀN THÀNH GIAO TIẾP
      unsigned long startTime = millis();
      while(millis() - startTime < 500)
      {
        if(digitalRead(X2) == 1)
          {
            break;
          }
      }
      digitalWrite(Y2, LOW);
    }
  // code cũ
  /*
  if(tong==0)
  {
    digitalWrite(Y2, HIGH);
    delay(500);
    digitalWrite(Y2, LOW);
  } // gui ve master
  */
}

//binh update 2/1/24
void gui_vitri_agv(String t)
{
  int dem;
  //for (dem = 2; dem > 0; dem--) {
    digitalWrite(Y0, HIGH);
    udp.beginPacket(udpAddress_master, udpPort); // Gửi tới master
    char buffer3[50];
    char temp_buffer[50];
    snprintf(temp_buffer, sizeof(temp_buffer), "VT_AGV01_%s", t.c_str());
    strncpy(buffer3, temp_buffer, sizeof(buffer3));
    Serial.println(buffer3); // In chuỗi để kiểm tra
    udp.write(reinterpret_cast<const uint8_t*>(buffer3), strlen(buffer3));
    udp.endPacket();
    delay(50); // Tạm dừng 1 giây
    digitalWrite(Y0, LOW);
    delay(50); // Tạm dừng 1 giây trước khi gửi lại
}
void gui_trangthaiagv_khi_khoi_dong()
{
  b=1;
  while(b==1)
  {
    digitalWrite(Y0, HIGH);
    udp.beginPacket(udpAddress_master, udpPort); // gửi toi master
    uint8_t buffer3[50] = "AGV01_davetram" ;
    udp.write(buffer3,14);
    udp.endPacket();
    j=0;
    for(j=0;j<50;j++)
    {
      ketnoi_lai_wifi();
      nhan_data();
      Serial.println(j);
      Serial.println("test ket");
      if(b==0) break;
    }
  
  }
}

/*void gui_trangthaiagv_khi_di_chuyen(int ten_line)
{
  b1=1;
  while(b1==1)
  {
    digitalWrite(Y0, HIGH);
    udp.beginPacket(udpAddress_master, udpPort); // gửi toi master
    uint8_t buffer3[50];
    snprintf(buffer3, sizeof(buffer3), "AGV01_BD%d", ten_line);
    udp.write(buffer3,14);
    udp.endPacket();
    j=0;
    for(j=0;j<50;j++)
    {
      ketnoi_lai_wifi();
      nhan_data();
      Serial.println(j);
      if(b1==0) break;
    }
  }
}*/
void gui_trangthaiagv_khi_di_chuyen(int ten_line) {
  b1 = 1;
  while (b1==1) {
    if(b1==0) break;
    digitalWrite(Y0, HIGH);
    udp.beginPacket(udpAddress_master, udpPort); // Gửi tới master
    
    char buffer3[50];
    char temp_buffer[50];
    snprintf(temp_buffer, sizeof(temp_buffer), "AGV01_BD%d", ten_line);
    if (ten_line<10)
    {
      strncpy(buffer3, temp_buffer, sizeof(buffer3));
      Serial.println(buffer3[9]);
      udp.write(reinterpret_cast<const uint8_t*>(buffer3), 9);
    } // Ép kiểu con trỏ char* thành uint8_t*
    if (ten_line>=10)
    {
      strncpy(buffer3, temp_buffer, sizeof(buffer3));
      Serial.println(buffer3[10]);
      udp.write(reinterpret_cast<const uint8_t*>(buffer3), 10);
    } // Ép kiểu con trỏ char* thành uint8_t*
    udp.endPacket();
    
    j = 0;
    for (j = 0; j < 50; j++) {
      ketnoi_lai_wifi();
      nhan_data();
       Serial.println(j);
       Serial.println(buffer3);
      if (b1 == 0) break;
    }
  }
}



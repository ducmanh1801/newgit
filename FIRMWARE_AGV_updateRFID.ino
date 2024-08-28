/*
  SU DUNG JY-L8800
 */
#include <Nextion.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13); //rX, tX //12-13//
void ResetBoard()
{
  asm volatile ( "jmp 0");  
}
// Khai bao chan dieu khien toc do-chieu quay, break DC BLDC ZD 
#define PWM_T 10 // DC trai
#define PWM_P 11 // DC phai
#define CW_T 44 //44 // DAO CHIEU DC TRAI
#define CW_P 45 //45 // DAO CHIEU DC PHAI
#define BREAK12 26
// Dinh nghia cac chan cam bien
#define ss1 A15 //B -1 LED MÉP TRAI
#define ss2 A14 //BR-1
#define ss3 A13 //L-1
#define ss4 A12 //W-1
#define ss5 A11 // GR-1
#define ss6 A10 //B-2
#define ss7 A9 //BR-2
#define ss8 A8  //L-2
#define ss9 A7 //W-2
#define ss10 A6 //GR-2
#define ss11 A5 //B-3
#define ss12 A4 //BR-3
#define ss13 A3 //L-3
#define ss14 4 //B-4
#define ss15 5 //BR-4
#define ss16 2  //L-4 LED MEP PHAI
#define sstoi_0V 24  //chon chieu bat sensor chay toi
#define sstoi_24V 36  //chon chieu bat sensor chay toi
#define sslui_0V 25  //chon chieu bat sensor chay lui
#define sslui_24V 35  //chon chieu bat sensor chay lui
//Dinh nghia cac chan cam bien vat can, cam bien co hang
#define CBVC1 3 // CBVC lidar truoc out 1
#define CBVC2 6 // CBVC lidar truoc out 2
#define CBVC3 7 // CBVC lidar sau out 1
#define CBVC4 8 // CBVC lidar sau out 2
#define CB_VACHAM 9 // CAM BIEN VA CHAM truoc/sau 
#define do_dungluongpin 49 // giao tiep mach do dung luong pin
// Dinh nghia cac chan dieu khien nhac,bao pin
#define I_stop 28 //dk loa nhac STOP
#define I_start 27 // dk loa nhac START
#define I_1 33 // dk loa nhac xin cap lieu
//#define dienapsac 48 // dk sac acquy
#define giaotiep_ESP32S3_X0 29 //XAC NHAN ESP DA NHAN DU LIEU LINE
#define giaotiep_ESP32S3_X1 39 // bao agv da ve tram
#define giaotiep_ESP32S3_X2 30 // gui DL xe bat dau di chuyen
#define denbao_RFID 23 
#define dieukhien_denxanh 32 //den 3 thap
#define dieukhien_denvang 31 //den 3 thap
#define cambienphat 34 // Điều khiển cảm biến phát
// Setup variables:
int serNum0=0;
int serNum1=0;
int serNum2=0;
int serNum3=0;
//Khai bao RFID_JY
char name_arr[21];
char name_arr1[19];
int Status_phanh=0;
char *test = "Serial.begin is Ready";
String str;
int check[50]={0};
int checksum=0;
///////////////////////////////////////////////////////////// 
int line=0,line1=0,line2=0,line3=0,line4=0,line5=0,line6=0,line7=0,line8=0,line9=0,line10=0,linevetram=0;
int        line11=0,line12=0,line13=0,line14=0,line15=0,line16=0,line17=0,line18=0,line19=0,line20=0;
int Chuyen[50] = {0}; // chứa giá trị các line sẽ chạy. line nào chạy phần tử thứ i mang giá trị bằng 1. Dùng để kiểm tra và đẩy dữ liệu đi. Khi cấp hàng xong line nào thì biến sẽ được gán lại =0;
int new_line=0;
int temp=0;// biến tạm lưu giá trị line.
float Kp=23,Ki=0.000005,Kd=18; 
//float Kp=22,Ki=0.5,Kd=15; 
float Kpc=20,Kic=0.07,Kdc=20;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
int initial_motor_speed=200;// 255 la toc do toi da
int speed_max=200;
int speed_max_=200;
int speed_min=5;
int speed_min_=5;
int speed_=200; // cho này nữa
int ChoPhepQuay1=1;
int ChoPhepQuay2=1;
int ChoPhepQuay3=1;
int ChoPhepQuay4=1;
float errorqueotrai=2;
float errorqueophai=-2;
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void queotrai_90();
void queophai_90();
int left_motor_speed = 0;
int right_motor_speed = 0;
float duytriline=0;
int duytrigiamtoc=0;
int duytri_queophai=0;
int duytri_queotrai=0;
int thoigianmatline=0;
int trangthai_CBVC1=0;
int trangthai_CBVC2=0;
int trangthai_CBVC3=0;
int trangthai_CBVC4=0;
int errorDoc=0;
int tam=0,trangthaiagv=1,trangthai_sac=0;
int k=0;
int i=0;
int a=0; // sdung cho lora
int trangthaitramtrahang=0;
int trangthai_vetram=0;
int trangthai_lenchuyen=0;
int demthoat=0;
int tatvatcan=0;
int deba1=0;
int demchieu=0;
int chaytiep=0;
String input,input1,thuchien,inputnhanlenhKT,inputnhanlenhLH,inputnhanlenhLP,inputnhanlenhLHP,chuoi;
String chuoi1,chuoi2;
String state="toi";
byte moc,moc1,moc2;
int wait=1;
int thoat=0;
bool conditionMet = false;
unsigned long conditionMetTime = 0;// Bien kiem tra thoi gian dut line tu
unsigned long checkTime = 0;// Bien kiem tra thoi gian dut line tu
void nhacstar();
void tatnhac();
void guitrangthai_hmi(String chuoi);
void mothang();
void chieutoi();
void chieului();
int value_va0 =0; // Bình thêm, biến chạy nhanh
int value_va1 =0; // Bình thêm, biến chạy chậm
int value_va2 =0; // Bình thêm, biến cài loa
//khai bao bien cho HMI
int variable1 = 0;  // Create a variable to have a counter going up by one on each cycle
int counter = 0;  // Create a variable to have a counter for the + and - buttons
int CurrentPage = 0;  // Create a variable to store which page is currently loaded
int dem_laichuyen=0;
bool BTstt[50]={0};

NexButton b1 = NexButton(0, 13, "b1");   // Button LINE 1
NexButton b2 = NexButton(0, 14, "b2");   // Button LINE 2
NexButton b5 = NexButton(0, 15, "b5");   // Button LINE 3
NexButton b6 = NexButton(0, 12, "b6");   // Button LINE 4
NexButton b7 = NexButton(0, 11, "b7");   // Button LINE 5
NexButton b9 = NexButton(0, 10, "b9");   // Button LINE 6
NexButton b10 = NexButton(0, 7, "b10");  // Button LINE 7
NexButton b0 = NexButton(0, 23, "b0");   // Button LINE 8
NexButton b14 = NexButton(0, 25, "b14"); // Button LINE 9
NexButton b13 = NexButton(0, 24, "b13"); // Button LINE 10
NexButton b3 = NexButton(0, 34, "b3");     // Button LINE 11
NexButton b12 = NexButton(0, 35, "b12");   // Button LINE 12
NexButton b15 = NexButton(0, 36, "b15");   // Button LINE 13
NexButton b16 = NexButton(0, 37, "b16");   // Button LINE 14
NexButton b17 = NexButton(0, 38, "b17");   // Button LINE 15
NexButton b18 = NexButton(0, 39, "b18");   // Button LINE 16
NexButton b11 = NexButton(0, 8, "b11");  // Button Tiep tuc

NexButton b37 = NexButton(0, 26, "b37");  // Button Cap hang
NexButton b8 = NexButton(0, 9, "b8");  // Button VE TRAM


//NexButton b3 = NexButton(0, 8, "b3");  // Button chieu tien// /////////////////////////// cũ rồi


NexButton b4 = NexButton(0, 6, "b4");  // Button RESET
NexText t3 = NexText(0, 18, "t3");  // Text box HIEN THI TRANG THAI AGV
NexText t5 = NexText(0, 19, "t5");  // Text box HIEN THI THONG TIN LOI
NexPage page0 = NexPage(0, 0, "page0");  // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event
//NexText va0 = NexText(1, 14, "va0"); // lấy thẳng từ T13 của Page1
NexText va0 = NexText(0, 22, "va0");  // bo đi 
NexText va1 = NexText(0, 51, "va1");  // bo đi 
NexText va2 = NexText(0, 53, "va2");  // bo đi 
// bỏ ko lấy từ value nữa mà lấy từ Number
NexNumber n0 = NexNumber(0, 30, "n0"); 
NexNumber n1 = NexNumber(0, 31, "n1");
NexNumber n2 = NexNumber(0, 32, "n2");
//
char buffer[100] = {0};
char buffer2[10] = {0}; 
//
uint32_t number0=0;// toc do nhanh
uint32_t number1=0;// toc do cham
uint32_t number2=0;// bat or tat Loa
NexTouch *nex_listen_list[] = 
{
  &b0,  // line 15
  &b1,  // line 1
  &b2,  // line 8
  &b3,  // line 11
  &b4,  // Xoa loi
  &b5,  // line 17
  &b6,  // line 4
  &b7,  // line 12
  &b8,  // Ve tram
  &b9,  // line 6
  &b10,  // line 14
  &b11,  // Tiep tuc
  &b37,  // Cap lieu
  &b13,  // line 3
  &b14,  // line 5

  &b12,  // line 12
  &b15,  // line 13
  &b16,  // line 14
  &b17,  // line 15
  &b18,  // line 16

  &n0,
  &n1,
  &n2,
  &page0,  // Page added as a touch event
  &page1,  // Page added as a touch event
  NULL  // String terminated
};  // End of touch event list
void b4PushCallback(void *ptr)  // Press event for button b4 reset
{
  //Serial2.print("@reset#");
  delay(500);
  tatnhac();
  ResetBoard();
}
void b1PushCallback(void *ptr)  // Press event for button b1 line1
{
  BTstt[1]=!BTstt[1];
  delay(100);
  if(BTstt[1])
  {
    temp=line;
    deba1=1;
    line1=1;
    line=1;
    new_line=1;
    Chuyen[1]=1;
    sendCommand("b1.bco=1024");
  }
  else
  {
    deba1=0;
    line1=0;
    line=temp;
    new_line=temp;
    Chuyen[1]=0;
    sendCommand("b1.bco=40179");
  }
}
void b2PushCallback(void *ptr)  // Press event for button b2 line8
{
  BTstt[2]=!BTstt[2];
  delay(100);
  if(BTstt[2])
  {
    temp=line;
    deba1=1;
    line2=1;
    line=2;
    new_line=2;
    Chuyen[2]=1;
    sendCommand("b2.bco=1024");
  }
  else
  {
    deba1=0;
    line2=0;
    line=temp;
    new_line=temp;
    Chuyen[2]=0;
    sendCommand("b2.bco=40179");
  }
}
void b5PushCallback(void *ptr)  // Press event for button b5 line17
{
  BTstt[3]=!BTstt[3];
  delay(100);
  if(BTstt[3])
  {
    temp=line;
    deba1=1;
    line3=1;
    line=3;
    new_line=3;
    Chuyen[3]=1;
    sendCommand("b5.bco=1024");
  }
  else
  {
    deba1=0;
    line3=0;
    line=temp;
    new_line=temp;
    Chuyen[3]=0;
    sendCommand("b5.bco=40179");
  }
}
void b6PushCallback(void *ptr)  // Press event for button b6 line4
{
  BTstt[4]=!BTstt[4];
  delay(100);
  if(BTstt[4])
  {
    temp=line;
    deba1=1;
    line4=1;
    line=4;
    new_line=4;
    Chuyen[4]=1;
    sendCommand("b6.bco=1024");
  }
  else
  {
    deba1=0;
    line4=0;
    line=temp;
    new_line=temp;
    Chuyen[4]=0;
    sendCommand("b6.bco=40179");
  }
}
void b7PushCallback(void *ptr)  // Press event for button b7 line 12
{
  BTstt[5]=!BTstt[5];
  delay(100);
  if(BTstt[5])
  {
    temp=line;
    deba1=1;
    line5=1;
    line=5;
    new_line=5;
    Chuyen[5]=1;
    sendCommand("b7.bco=1024");
  }
  else
  {
    deba1=0;
    line5=0;
    line=temp;
    new_line=temp;
    Chuyen[5]=0;
    sendCommand("b7.bco=40179");
  }
}
void b9PushCallback(void *ptr)  // Press event for button b9 line6
{
  BTstt[6]=!BTstt[6];
  delay(100);
  if(BTstt[6])
  {
    temp=line;
    deba1=1;
    line6=1;
    line=6;
    new_line=6;
    Chuyen[6]=1;
    sendCommand("b9.bco=1024");
  }
  else
  {
    deba1=0;
    line6=0;
    line=temp;
    new_line=temp;
    Chuyen[6]=0;
    sendCommand("b9.bco=40179");
  }
}
void b10PushCallback(void *ptr)  // Press event for button b10 line14
{
  BTstt[7]=!BTstt[7];
  delay(100);
  if(BTstt[7])
  {
    temp=line;
    deba1=1;
    line7=1;
    line=14;
    new_line=7;
    Chuyen[7]=1;
    sendCommand("b10.bco=1024");
  }
  else
  {
    deba1=0;
    line7=0;
    line=temp;
    new_line=temp;
    Chuyen[7]=0;
    sendCommand("b10.bco=40179");
  }
}
void b0PushCallback(void *ptr)  
{
  BTstt[8]=!BTstt[8];
  delay(100);
  if(BTstt[8])
  {
    temp=line;
    deba1=1;
    line8=1;
    line=8;
    new_line=8;
    Chuyen[8]=1;
    sendCommand("b0.bco=1024");
  }
  else
  {
    deba1=0;
    line8=0;
    line=temp;
    new_line=temp;
    Chuyen[8]=0;
    sendCommand("b0.bco=40179");
  }
}
void b14PushCallback(void *ptr)  // Press event for button b14 line5
{
  BTstt[9]=!BTstt[9];
  delay(100);
  if(BTstt[9])
  {
    temp=line;
    deba1=1;
    line9=1;
    line=9;
    new_line=9;
    Chuyen[9]=1;
    sendCommand("b14.bco=1024");
  }
  else
  {
    deba1=0;
    line9=0;
    line=temp;
    new_line=temp;
    Chuyen[9]=0;
    sendCommand("b14.bco=40179");
  }
}
void b13PushCallback(void *ptr)  // Press event for button b13 line3
{
  BTstt[10]=!BTstt[10];
  delay(100);
  if(BTstt[10])
  {
    temp=line;
    deba1=1;
    line10=1;
    line=10;
    new_line=10;
    Chuyen[10]=1;
    sendCommand("b13.bco=1024");
  }
  else
  {
    deba1=0;
    line10=0;
    line=temp;
    new_line=temp;
    Chuyen[3]=0;
    sendCommand("b13.bco=40179");
  }
}
void b3PushCallback(void *ptr)  // Press event for button b13 line3
{
  BTstt[11]=!BTstt[11];
  delay(100); 
  if(BTstt[11])
  {
    temp=line;
    deba1=1;
    line11=1;
    line=11;
    new_line=11;
    Chuyen[11]=1;
    sendCommand("b3.bco=1024");   
  }
  else
  {
    deba1=0;
    line11=0;
    line=temp;
    new_line=temp;
    Chuyen[11]=0;
    sendCommand("b3.bco=40179");    
  }
}
//12 new
void b12PushCallback(void *ptr)  // Press event for button b13 line3
{
  BTstt[12]=!BTstt[12];
  delay(100);
  if(BTstt[12])
  {
    temp=line;
    deba1=1;
    line12=1;
    line=12;
    new_line=12;
    Chuyen[12]=1;
    sendCommand("b12.bco=1024");
  }
  else
  {
    deba1=0;
    line12=0;
    line=temp;
    new_line=temp;
    Chuyen[12]=0;
    sendCommand("b12.bco=40179");   
  }
}
//old
/*
void b12PushCallback(void *ptr)  // Press event for button b21 line12
{
  BTstt[12]=!BTstt[12];
  delay(100);
  if(BTstt[12])
  {
    temp=line;
    deba1=1;
    line12=1;
    line=12;
    new_line=12;
    Chuyen[12]=1;
    sendCommand("b12.bco=1024");
  }
  else
  {
    deba1=0;
    line12=0;
    line=temp;
    new_line=temp;
    Chuyen[12]=0;
    sendCommand("b12.bco=40179");
  }
}*/
void b15PushCallback(void *ptr)  // Press event for button b15 line13
{
  BTstt[13]=!BTstt[13];
  delay(100);
  if(BTstt[13])
  {
    temp=line;
    deba1=1;
    line13=1;
    line=13;
    new_line=13;
    Chuyen[13]=1;
    sendCommand("b15.bco=1024");
  }
  else
  {
    deba1=0;
    line13=0;
    line=temp;
    new_line=temp;
    Chuyen[13]=0;
    sendCommand("b15.bco=40179");
  }
}
void b16PushCallback(void *ptr)  // Press event for button b16 line14
{
  BTstt[14]=!BTstt[14];
  delay(100);
  if(BTstt[14])
  {
    temp=line;
    deba1=1;
    line14=1;
    line=14;
    new_line=14;
    Chuyen[14]=1;
    sendCommand("b16.bco=1024");
  }
  else
  {
    deba1=0;
    line14=0;
    line=temp;
    new_line=temp;
    Chuyen[14]=0;
    sendCommand("b16.bco=40179");
  }
}
void b17PushCallback(void *ptr)  // Press event for button b17 line15
{
  BTstt[15]=!BTstt[15];
  delay(100);
  if(BTstt[15])
  {
    temp=line;
    deba1=1;
    line15=1;
    line=15;
    new_line=15;
    Chuyen[15]=1;
    sendCommand("b17.bco=1024");
  }
  else
  {
    deba1=0;
    line15=0;
    line=temp;
    new_line=temp;
    Chuyen[15]=0;
    sendCommand("b17.bco=40179");
  }
}
void b18PushCallback(void *ptr)  // Press event for button b18 line16
{
  BTstt[16]=!BTstt[16];
  delay(100);
  if(BTstt[16])
  {
    temp=line;
    deba1=1;
    line16=1;
    line=16;
    new_line=16;
    Chuyen[16]=1;
    sendCommand("b18.bco=1024");
  }
  else
  {
    deba1=0;
    line16=0;
    line=temp;
    new_line=temp;
    Chuyen[16]=0;
    sendCommand("b18.bco=40179");
  }
}
void b8PushCallback(void *ptr)  // Press event for button b8 ve tram
{
  //Serial2.print("@vetram#");
  guitrangthai_hmi("AGV về trạm");
 // stopdongco();
 // moquat_hut();
  mothang();
  delay(200);
  tatnhac();
  chieului();// lui ve tram luon
  deba1=1;
  linevetram=1;
 // state_vetram=1;
  //sendCommand("sleep=1"); // tat man hinh
  //delay(100);
}
void b11PushCallback(void *ptr)  // Press event for button b11 tiep tuc
{
  delay(200);
  tatnhac();
  chaytiep=1;
  trangthai_vetram=1;
  for (int i = 0; i < 50; i++) {
    if (Chuyen[i] != 0) {
      // Nếu có ít nhất một phần tử khác 0, đặt trangthai_vetram thành 0 và thoát khỏi vòng lặp
      trangthai_vetram=0;
      break;
    }
  }
  if(trangthai_vetram==1)
  guitrangthai_hmi("AGV về trạm");
 // trangthai_lenchuyen=0;
  //sendCommand("page 1"); // chuyen trang
  //delay(100);
}
void b37PushCallback(void *ptr)  // Press event for button b37 Cap lieu
{ 
  delay(200);
  tatnhac();
  //chaytiep=1;
  trangthai_lenchuyen=1;
  trangthai_vetram=0;
  Serial.println("da bam chay tiep");
  //sendCommand("page 1"); // chuyen trang
  //delay(100);
}
// Page change event:
void page0PushCallback(void *ptr)  // 

{
  CurrentPage = 0;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  //Serial.write("ok");
}  // End of press event
 
void page1PushCallback(void *ptr)  // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1;  // Set variable as 1 so from now on arduino knows page 1 is loaded on the display
}  // End of press event
// hàm xử lý sự kiện
/////////////////////////////////////////////////////////////////
void laygiatri_hmi(){
  n0.getValue(&number0);
  Serial.print("Value of value 0: ");
  Serial.println(number0);
  speed_=number0;
  // Đọc giá trị từ biến va1
  n1.getValue(&number1);
  Serial.print("Value of value 1: ");
  Serial.println(number1);
// Đọc giá trị từ biến va1
  n2.getValue(&number2);
  Serial.print("Value of value 2: ");
  Serial.println(number2);
}
void setup() 
{
  delay(2000);
  Serial.begin(9600);
  Serial1.begin(9600); // GIAO TIEP JY-L8800 truoc
  Serial2.begin(9600); // GIAO TIEP HMI
 // Serial3.begin(9600); // GIAO TIEP JY-L8800 sau
  mySerial.begin(9600); //giao tiep ESP
    nexInit(); //inicializa a comunicação com o nextion
    delay(500);  // This dalay is just in case the nextion display didn't start yet, to be sure it will receive the following command.
    b0.attachPush(b0PushCallback,&b0);  // Button press
    b1.attachPush(b1PushCallback,&b1);  // Button press
    b2.attachPush(b2PushCallback,&b2);  // Button press
    b3.attachPush(b3PushCallback,&b3);  // Button press
    b4.attachPush(b4PushCallback,&b4);  // Button press
    b5.attachPush(b5PushCallback,&b5);  // Button press
    b6.attachPush(b6PushCallback,&b6);  // Button press
    b7.attachPush(b7PushCallback,&b7);  // Button press
    b8.attachPush(b8PushCallback,&b8);  // Button press
    b9.attachPush(b9PushCallback,&b9);  // Button press
    b10.attachPush(b10PushCallback,&b10);  // Button press
    b11.attachPush(b11PushCallback,&b11);  // Button press
    //new
    b12.attachPush(b12PushCallback,&b12);  // Button press
    b15.attachPush(b15PushCallback,&b15);  // Button press
    b16.attachPush(b16PushCallback,&b16);  // Button press
    b17.attachPush(b17PushCallback,&b17);  // Button press
    b18.attachPush(b18PushCallback,&b18);  // Button press 
    //old   
    b13.attachPush(b13PushCallback,&b13);  // Button press
    b14.attachPush(b14PushCallback,&b14);  // Button press
    b37.attachPush(b37PushCallback,&b11);
    page0.attachPush(page0PushCallback);  // Page press event
    page1.attachPush(page1PushCallback);
    dbSerialPrintln("setup done");
  pinMode(PWM_T, OUTPUT);
  pinMode(PWM_P, OUTPUT);
  pinMode(CW_T, OUTPUT); // dao chieu dc trai
  pinMode(CW_P, OUTPUT); //dap chieu dc phai
  pinMode(BREAK12, OUTPUT);
  pinMode(I_stop, OUTPUT);
  pinMode(I_start, OUTPUT);
  pinMode(I_1, OUTPUT);
  pinMode(giaotiep_ESP32S3_X0, OUTPUT);
  pinMode(giaotiep_ESP32S3_X1, OUTPUT);
  pinMode(giaotiep_ESP32S3_X2, OUTPUT);
  pinMode(denbao_RFID, OUTPUT);
  pinMode(dieukhien_denxanh, OUTPUT);
  pinMode(dieukhien_denvang, OUTPUT);
  pinMode(sstoi_0V, OUTPUT);
  pinMode(sstoi_24V, OUTPUT);
  pinMode(sslui_0V, OUTPUT);
  pinMode(sslui_24V, OUTPUT);
  pinMode(ss1, INPUT_PULLUP);
  pinMode(ss2, INPUT_PULLUP);
  pinMode(ss3, INPUT_PULLUP);
  pinMode(ss4, INPUT_PULLUP);
  pinMode(ss5, INPUT_PULLUP);
  pinMode(ss6, INPUT_PULLUP);
  pinMode(ss7, INPUT_PULLUP);
  pinMode(ss8, INPUT_PULLUP);
  pinMode(ss9, INPUT_PULLUP);
  pinMode(ss10, INPUT_PULLUP);
  pinMode(ss11, INPUT_PULLUP);
  pinMode(ss12, INPUT_PULLUP);
  pinMode(ss13, INPUT_PULLUP);
  pinMode(ss14, INPUT_PULLUP);
  pinMode(ss15, INPUT_PULLUP);
  pinMode(ss16, INPUT_PULLUP);
  pinMode(CBVC1, INPUT_PULLUP);
  pinMode(CBVC2, INPUT_PULLUP);
  pinMode(CBVC3, INPUT_PULLUP);
  pinMode(CBVC4, INPUT_PULLUP);
  pinMode(CB_VACHAM, INPUT_PULLUP);
  pinMode(do_dungluongpin, INPUT_PULLUP);
  pinMode(cambienphat, OUTPUT);
  digitalWrite(I_stop,LOW);
  digitalWrite(I_start,LOW);
  analogWrite(PWM_T, 0);
  analogWrite(PWM_P, 0);
  digitalWrite(BREAK12,LOW);//DONG CHAN BREKAK XUONG GND la HIGH (MO THANG)
  digitalWrite(giaotiep_ESP32S3_X0,LOW);
  digitalWrite(giaotiep_ESP32S3_X1,LOW);
  digitalWrite(giaotiep_ESP32S3_X2,LOW);
  digitalWrite(denbao_RFID,LOW);
  tatden_3thap();
  //delay(2000);
  guitrangthai_hmi("ĐANG KIỂM TRA RFID-KHÔNG ẤN GÌ");
  Serial1.print("$S#"); // gui kiem tra dau doc the
  delay(100);
  while(1)
  {
    Serial1.print("$S#"); // gui kiem tra dau doc the
    delay(100);
    giaotiephmi();
    i=0;
    for(i=0;i<200;i++)
    {
      kiemtra_docrfid_JY();
      Serial.print("dang kiem tra dau doc RFID ");
      Serial.println(i);
      //giaotiephmi();
      if(demthoat==1)
      break;
    }
    if(demthoat==1)
    break;
  }
  demthoat=0;
  chieutoi();
  guitrangthai_hmi("ẤN NÚT <VỀ TRẠM>");
  guithongtinloi_hmi(" ");
  Serial.println("SAN SANG");
  laygiatri_hmi();// Binh them để đọc giá trị 3 biến
  checksum=0;
}
void Bat_cambienphat()
{
    digitalWrite(cambienphat,HIGH);
}
void Tat_cambienphat()
{
    digitalWrite(cambienphat,LOW);
}
void read_sensor_values() // DO 4-5 LED
 {
  conditionMet = false;
  //chuong trinh chay thang////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=0;     
  // chuong trinh lech trai////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=0;
        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
          error=0.15;
          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
            error=0.15;
            else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
              error=0.6;
              else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                error=0.6;
                else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                  error=0.9;
                  else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                    error=0.9;
                    else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                      error=1.2;
                      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                       error=1.2;
                       else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                        error=1.5;
                        else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                          error=2;
                          else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                            error=2.5;
                            else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                              error=3;
                              else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                                error=3.5;
                                else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                                  error=4;
                 
   // chuong trinh lech phai/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
      error=0;
      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=-0.15;
        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
         error=-0.15;
         else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
          error=-0.6;
          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
            error=-0.6;
            else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
              error=-0.9;
              else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                error=-0.9;
                else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                  error=-1.2;
                  else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==1)))  
                    error=-1.2;
                    else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==1)))  
                      error=-1.5;
                      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                       error=-2;
                       else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                        error=-2.5;
                        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                          error=-3;
                          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                            error=-3.5;
                            else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==0)))  
                             error=-4;

/* else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
    error=0;
    else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
      error=0;
      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
       error=0;
       else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=0;
        else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
          error=0;

  else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
    error=0;
    else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
      error=0;
      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=0;
        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==1)))  
          error=0;
          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
            error=0;     */    
/////////////////////////2.12 update. Hiện tại đang lỗi ko chạy được                   
/* else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
  {
    if (!conditionMet) 
    {
      conditionMet = true;
      conditionMetTime = millis();  // Ghi lại thời điểm điều kiện được thỏa mãn
    }

    if ((conditionMet) &&(millis() - conditionMetTime >= 1000))
    {
    dunglaimatline();
    conditionMet = false;
    conditionMetTime = millis(); // Thiết lập lại thời gian
    }
  }*/
//////////////////////// 4.12// repair
  else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
  {
      conditionMetTime = millis();  // Ghi lại thời điểm điều kiện được thỏa mãn
      while(((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
      {
        if ((millis() - conditionMetTime >= 1000))
        {
          dunglaimatline();
          conditionMet = false;
          conditionMetTime = millis(); // Thiết lập lại thời gian
        }
      }
  }
  /////////////////////// hết update 4.12
     /*if(error>1)
      { 
           error=6; 
      }
    else if(error<-1)
      { 
           error=-6; 
      }
    delay (5000);
    if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
      */   
      //   Binh update thoat khoi line tu 3s. Them bien toan cuc: unsigned long lastConditionTime = 0;
      

    //  hết phần update
     //dunglaimatline();
  else error=error;// --------------------------------------------------(Giữ lại sai lệch cũ à) bỏ đi xem sao---------------------------------------
}
void read_sensor_values_trai_phai() // DO 4-5 LED
 {
  //chuong trinh chay thang////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=0;     
  // chuong trinh lech trai////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=0;
        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
          error=0.3;
          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
            error=0.3;
            else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
              error=1.5;
              else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                error=1.5;
                else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                  error=1.5;
                  else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==0) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                    error=3;
                    else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                      error=3.5;
                      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==0) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                       error=4;
                       else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                        error=4.5;
                        else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==0) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                          error=5;
                          else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==0) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                            error=5.5;
                            else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==0) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                              error=6;
                              else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==0) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                                error=6.5;
                                else if (((digitalRead(ss1)==0) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                                  error=6.9;
                 
   // chuong trinh lech phai/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==0) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
      error=0;
      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
        error=-0.3;
        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==0) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
         error=-0.8;
         else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
          error=-1.5;
          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==0) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
            error=-1.5;
            else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
              error=-1.5;
              else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==0) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                error=-3;
                else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
                  error=-3.5;
                  else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==0) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==1)))  
                    error=-4;
                    else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==1)))  
                      error=-4.5;
                      else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==0) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                       error=-5;
                       else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==0) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                        error=-5.5;
                        else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==0) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                          error=-6;
                          else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==0) && (digitalRead(ss16)==0)))  
                            error=-6.5;
                            else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==0)))  
                             error=-6.9;
 else if (((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
  {   
    error=error;
  }
 else
 error=error;
}
// Binh khai bao them:
unsigned long previousTime = 0;
double MaxI = 0.0005;
////////////////////////////////////////////////code có thêm deltaTime 13.6

void calculate_pid() {
    unsigned long now = millis();
    double deltaTime = (double)(now - previousTime) / 1000.0;  // Thời gian delta tính theo giây
    previousTime = now;

    // Giới hạn deltaTime để tránh các giá trị bất thường
    if (deltaTime > 2.5) 
    {
        deltaTime = 2.5;
    } else if (deltaTime < 0.5) //////////// 0.2
    {
        deltaTime = 0.5;
    }

    // Tính toán P, I, D
    P = error;

    // Giới hạn tốc độ thay đổi của thành phần I
    double potentialI = I + (error * deltaTime);
    if (potentialI > MaxI) {
        I = MaxI;
    } else if (potentialI < -MaxI) {
        I = -MaxI;
    } else {
        I = potentialI;
    }

    double deltaError = error - previous_error;
    D = deltaError / deltaTime;

    // Giới hạn giá trị của thành phần D
    double maxD = 5.0; // có thể điều chỉnh giá trị này tùy thuộc vào hệ thống
    if (D > maxD) {
        D = maxD;
    } else if (D < -maxD) {
        D = -maxD;
    }
    if(error!=0)
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    if(error==0)
    PID_value = 0;    

    // Anti-windup: Nếu điều khiển PID_value vượt quá giới hạn, giảm giá trị tích lũy của I
    //double outputMax = initial_motor_speed/6; // Giới hạn đầu ra của hệ thống
    //double outputMin = -initial_motor_speed/6; // Giới hạn đầu ra của hệ thống
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////update 19.6
    double outputMax = abs(error)*30 + 15; // Giới hạn đầu ra của hệ thống (+2 xem nhé)
    double outputMin = abs(error)*-30 -15; // Giới hạn đầu ra của hệ thống (-2 xem nhé)
    if (PID_value > outputMax) {
        PID_value = outputMax;
        I -= error * deltaTime; // Giảm giá trị tích lũy của I
    } else if (PID_value < outputMin) {
        PID_value = outputMin;
        I -= error * deltaTime; // Giảm giá trị tích lũy của I
    }
    previous_error = error;

    // In ra các giá trị để kiểm tra (có thể bỏ sau khi kiểm tra)deltaTime
    /*Serial.print("deltaTime: ");
    Serial.print(deltaTime);      
    Serial.print("error: ");
    Serial.print(error);    
    Serial.print("P: ");
    Serial.print(P);
    Serial.print(", I: ");
    Serial.print(I);
    Serial.print(", D: ");
    Serial.print(D);
    Serial.print(", PID_value: ");
    Serial.println(PID_value);
    */
}

// Test delta time & thêm điều kiện
/*
void calculate_pid() {
    unsigned long now = millis();
    double deltaTime = (double)(now - previousTime) / 1000.0;  // Thời gian delta tính theo giây
    previousTime = now;
    // Giới hạn deltaTime để tránh các giá trị bất thường
    if (deltaTime > 2.0) {
        deltaTime = 2.0;
    } else if (deltaTime < 0.5) {
        deltaTime = 0.5;
    }   
    // Tính toán P, I, D
    P = error;
    I += error * deltaTime;
    D = (error - previous_error) / deltaTime;

    // Giới hạn tổng của I để tránh tích lũy quá nhanh
    if (I > MaxI) {
        I = MaxI;
    } else if (I < -MaxI) {
        I = -MaxI;
    }

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    // Anti-windup: Nếu điều khiển PID_value vượt quá giới hạn, giảm giá trị tích lũy của I
    double outputMax = 30.0; // Giới hạn đầu ra của hệ thống
    double outputMin = -30.0; // Giới hạn đầu ra của hệ thống
    if (PID_value > outputMax) {
        PID_value = outputMax;
        I -= error * deltaTime; // Giảm giá trị tích lũy của I
    } else if (PID_value < outputMin) {
        PID_value = outputMin;
        I -= error * deltaTime; // Giảm giá trị tích lũy của I
    }
    previous_error = error;
}
*/
//////////////////////////////////////////////// Bỏ code có thêm deltaTime => code cũ
/*
void calculate_pid()
{
    P = error;
    I = I + error;
    D = error-previous_error;    
    // Giới hạn tổng của I để tránh tích lũy quá nhanh
    if (I > MaxI)
    {
      I = MaxI;
    } else if (I < -MaxI)
    {
      I = -MaxI;
    }    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    previous_error=error;
}
*/
void motor_control()
{
   if(state=="toi")
        {
          left_motor_speed = initial_motor_speed + PID_value;
          right_motor_speed = initial_motor_speed - PID_value;
        }
      else if(state=="lui")
        {
          left_motor_speed = initial_motor_speed - PID_value;
          right_motor_speed = initial_motor_speed + PID_value;
        }
   if (left_motor_speed > speed_max)
    { 
      left_motor_speed=speed_max;
    }
    else if (left_motor_speed < speed_min)
    {
      left_motor_speed=speed_min;
    }
    else left_motor_speed=left_motor_speed;
    ////////////////////////////////////////////////////
    if (right_motor_speed > speed_max)
    { 
      right_motor_speed=speed_max;
    }
    else if (right_motor_speed < speed_min)
    {
      right_motor_speed=speed_min;
    }
    else right_motor_speed=right_motor_speed;
    /////////////////////////////////////////////////////
    {       
      analogWrite(PWM_T, left_motor_speed);
      analogWrite(PWM_P, right_motor_speed);
    }
    //bo debug
    Serial.println("");
    Serial.print("error: ");    
    Serial.print(error);
    Serial.print("  ");
    Serial.print("PID_value: ");
    Serial.print(PID_value);
    Serial.print("  ");
    Serial.print(left_motor_speed);
    Serial.print("  ");
    Serial.println(right_motor_speed);
}
void deba()
{
  laygiatri_hmi();
  docrfid();
  clear_rifd();
  batden_xanh_3thap();
  initial_motor_speed=30;
  for(int i=0;i<20;i++)
  {
      Serial.println("deba");
      mothang();
      read_sensor_values();
      docrfid();
      calculate_pid();
      motor_control();
      if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==54)) clear_rifd();   //the so 50 stop
      if((serNum0 !=0) || (serNum1 !=0) || (serNum2 !=0) || (serNum3 !=0)) break; 
     // if((serNum0 ==52) && (serNum1 ==50) && (serNum2 ==52) && (serNum3 ==55)) {dunglaigap;i=0; break;}   //the so VT2-2 dung lai tra hang
      if(i>=10)
      {
        initial_motor_speed=initial_motor_speed+18;
        if(initial_motor_speed>=number1)
        {
          if(duytrigiamtoc==1)// đọc được thẻ giảm tốc
          {
            initial_motor_speed=number1;//
           // i=0;
            break; 
          }
          else
          {
            initial_motor_speed=number0;
           // i=0;
            break;
          }
        } 
      }
      if((duytrigiamtoc==1)&&(initial_motor_speed>=number1))
      {
        initial_motor_speed=number1;
        //i=0;
        break; 
      }
    }
  Serial1.begin(9600); //MO GIAO TIEP DAU DOC THE RFID
  deba1=0;
}
void mothang()
{
  digitalWrite(BREAK12,HIGH);
  bat_RFID();
}
void dongthang()
{
  digitalWrite(BREAK12,LOW);
  tat_RFID();
}
void chieutoi()
{
  // DAO CHIEU DC
  digitalWrite(CW_T,HIGH); // CHAY TOI
  digitalWrite(CW_P,LOW); // CHAY TOI
  digitalWrite(sstoi_0V,HIGH);
  digitalWrite(sstoi_24V,HIGH);  
  state="toi";
  //Serial1.begin(9600); // RFID toi
  //Serial3.end(); //tat RFID lui
  digitalWrite(sslui_0V,LOW);
  digitalWrite(sslui_24V,LOW);
  Serial.println("chieu toi");
  delay(1000);
}
void chieului()
{
  // DAO CHIEU DC
  digitalWrite(CW_T,LOW); // CHAY LUI
  digitalWrite(CW_P,HIGH); // CHAY LUI
  digitalWrite(sslui_0V,HIGH); 
  digitalWrite(sslui_24V,HIGH); 
  state="lui";
  //Serial1.end(); // tat rfid
  digitalWrite(sstoi_0V,LOW);
  digitalWrite(sstoi_24V,LOW);
  Serial.println("chieu lui");
  delay(1000);
}
void tat_RFID()
{
  Serial1.end(); // rfid
}
void bat_RFID()
{
  Serial1.begin(9600); 
  
}
void chieuqueotrai_90()
{
  // DAO CHIEU DC
  if(state=="toi")
  {
    digitalWrite(CW_T,HIGH); 
    digitalWrite(CW_P,HIGH); 
    delay(200);
    Serial.println("chieu toi re trai");
  }
  else if(state=="lui")
  {
    digitalWrite(CW_T,HIGH); 
    digitalWrite(CW_P,HIGH); 
    delay(200);
    Serial.println("chieu lui re trai");
  }
}
void chieuqueophai_90()
{
  // DAO CHIEU DC
  if(state=="toi")
  {
    digitalWrite(CW_T,LOW); 
    digitalWrite(CW_P,LOW); 
    delay(200);
    Serial.println("chieu toi re phai");
  }
  else if(state=="lui")
  {
    digitalWrite(CW_T,LOW); 
    digitalWrite(CW_P,LOW); 
    delay(200);
    Serial.println("chieu lui re phai");
  }
}
void dunglai()
{
   //initial_motor_speed=70;
   for(k=0;k<10;k++)
   {
      read_sensor_values();
      docrfid();
      calculate_pid();
      motor_control();
      vatcan();
      delay(5);
      if(k==9)
      {
        initial_motor_speed=initial_motor_speed-20;
        Serial.println(initial_motor_speed);
        if(initial_motor_speed <speed_min )
        {
            initial_motor_speed=0;
            k=0;
            Serial.println("bbbbb");
            break;       
        }
        k=0;      
      }
   }
   dongthang(); 
   analogWrite(PWM_T, 0);
   analogWrite(PWM_P, 0);
   //initial_motor_speed=speed_;
}
void dunglaigap()
{  
   analogWrite(PWM_T, 50);
   analogWrite(PWM_P, 50);
   delay(10);
   analogWrite(PWM_T, 30);
   analogWrite(PWM_P, 30);
   dongthang();
   analogWrite(PWM_T, 0);
   analogWrite(PWM_P, 0);
  // delay(2000);
}
void dunglaimatline()
{
   initial_motor_speed=50;
   calculate_pid();
   motor_control();
   delay(200);
   dongthang(); 
   analogWrite(PWM_T, 0);
   analogWrite(PWM_P, 0);
   guithongtinloi_hmi("XE RA KHOI DUONG TU");
   delay(200);
  while(((digitalRead(ss1)==1) && (digitalRead(ss2)==1) && (digitalRead(ss3)==1) && (digitalRead(ss4)==1) && (digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))  
    {
      analogWrite(PWM_T, 0);
      analogWrite(PWM_P, 0);
      guithongtinloi_hmi("XE RA KHOI DUONG TU");
      delay(200);
      giaotiephmi();
      nhacstop();
     // Serial.println("XE RA KHOI DUONG TU");
    }
    guithongtinloi_hmi(" ");
    delay(200);
    mothang();
    deba();
}
void batden_vang_3thap()
{
  digitalWrite(dieukhien_denvang, HIGH);
  digitalWrite(dieukhien_denxanh, LOW);
}
void batden_xanh_3thap()
{
  digitalWrite(dieukhien_denvang, LOW);
  digitalWrite(dieukhien_denxanh, HIGH);
}
void tatden_3thap()
{
  digitalWrite(dieukhien_denvang, LOW);
  digitalWrite(dieukhien_denxanh, LOW);
}
void tocdomacdinh()
{
  //ChoPhepQuay=1;  // mới chuyển cái này ra đây, lúc trước nó ở sau cùng của hàm deba()
  laygiatri_hmi();
  docrfid();
  clear_rifd();
  // lấy thêm biên dao động cho tốc độ di chuyển // 25.4=> rat quan trọng
  speed_max=number0 + 30;
  speed_min=5;
  for(i=0;i<20;i++)
  {
      Serial.println("tangtoctutu");
      docrfid();
      if((serNum0 !=0) && (serNum1 !=0) && (serNum2 !=0) && (serNum3 !=0)) break;
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      if(i==10)
      {
        initial_motor_speed=initial_motor_speed+10;
        if(initial_motor_speed>=number0)
        {
          initial_motor_speed=number0;
          duytrigiamtoc=0;
          i=0;
          break;
         }
        i=0;      
       }
    }
}
void datlaitocdo()
{
      tatvatcan=0;
      tocdomacdinh();
      clear_rifd();

}
void giamtoc_tutu()
{
  laygiatri_hmi();

  for(int k=8;k<20;k++)
  {
      Serial.println("giamtoctutu");
      Serial.println(k);
      Serial.println(initial_motor_speed);
      read_sensor_values();
      calculate_pid();
      motor_control();
      //docrfid_JY();
     // vatcan();
      while(k==10)
      {
       initial_motor_speed=initial_motor_speed-40;
        Serial.println(initial_motor_speed);
        if(initial_motor_speed <10 )
        {
            initial_motor_speed=0;
            Serial.println("DA GIAM VE 0");
            dongthang();
            break;       
        }
      }
  }
}
void vatcan()
{
  if(digitalRead(CB_VACHAM) == 0)
    {
      dunglaigap();
      guithongtinloi_hmi("CO VAT CAN");
      delay(200);
      while(digitalRead(CB_VACHAM) == 0)
      {            
        Serial.println("dunggap");
        guithongtinloi_hmi("CO VAT CAN");
        delay(200);
        nhacstop();
        giaotiephmi();
      }        
      delay(2000);
      guithongtinloi_hmi(" ");
      delay(200);
      tatnhac();
      //mothang();
      deba();
     }
  if(state=="toi")
    {
      if((digitalRead(CBVC2) == 1)&&(trangthai_CBVC2==0)&&(tatvatcan==0)) // vùng 2 giam tốc
      { 
        nhacstar();
        trangthai_CBVC2=1;
        initial_motor_speed=number1;
        guithongtinloi_hmi("CO VAT CAN MUC 2");
      }
      if((digitalRead(CBVC2) == 0) &&(trangthai_CBVC2==1)&&(tatvatcan==0))
      { 
        trangthai_CBVC2=0;
        guithongtinloi_hmi("");
        if(!duytrigiamtoc){tocdomacdinh();}
        tatnhac();
      }
      if((digitalRead(CBVC1) == 1)&&(tatvatcan==0))
      { 
        giamtoc_tutu();
        Serial.print("tat vat can =");
        Serial.print(tatvatcan);
        while(digitalRead(CBVC1) == 1)
        {
          guithongtinloi_hmi("CO VAT CAN MUC 1");
          delay(200);
          nhacstop();
          giaotiephmi();
          Serial.print("da tat vat can = ");
          Serial.println(tatvatcan);
        }
        delay(2000);
        guithongtinloi_hmi(" ");
        delay(200);
        tatnhac();
        deba();
      }
    }
  if(state=="lui")
    {
      if((digitalRead(CBVC4) == 1) &&(trangthai_CBVC4==0)&&(tatvatcan==0))
      { 
        trangthai_CBVC4=1;
        initial_motor_speed=number1;
        guithongtinloi_hmi("CO VAT CAN MUC 2");
        nhacstar();        
      }
      if((digitalRead(CBVC4) == 0) &&(trangthai_CBVC4==1)&&(tatvatcan==0))
      { 
        trangthai_CBVC4=0;
        guithongtinloi_hmi("");
        if(!duytrigiamtoc){tocdomacdinh();}
        tatnhac();        
      }
      if((digitalRead(CBVC3) == 1)&&(tatvatcan==0))
      { 
        giamtoc_tutu();
        while(digitalRead(CBVC3) == 1)
        {
          guithongtinloi_hmi("CO VAT CAN MUC 1");
          delay(200);
          nhacstop();
          giaotiephmi();
        }
        delay(2000);
        guithongtinloi_hmi(" ");
        delay(200);
        tatnhac();
        deba();
      }
    }
}
void nhacstar()
{
  if(number2==1)
  {
    digitalWrite(I_1,LOW);
    digitalWrite(I_stop,LOW);
    digitalWrite(I_start,HIGH);
  }
}
void nhacstop()
{
  if(number2==1)
  {  
    digitalWrite(I_1,LOW);
    digitalWrite(I_stop,HIGH);
    digitalWrite(I_start,LOW);
  }
}
void nhacdenline()
{
    if(number2==1)
  {
    digitalWrite(I_1,LOW);
    digitalWrite(I_stop,HIGH);
    digitalWrite(I_start,HIGH);
  }
}
void nhacxincaplieu()
{
  if(number2==1)
  { 
    digitalWrite(I_1,HIGH);
    digitalWrite(I_stop,LOW);
    digitalWrite(I_start,LOW);
  }
}
void tatnhac()
{
  digitalWrite(I_1,LOW);
  digitalWrite(I_stop,LOW);
  digitalWrite(I_start,LOW);
  delay(100);
}
void giaotiephmi() // qua cong serial 2
{
    nexLoop(nex_listen_list);  // Check for any touch event
}
void guitrangthai_hmi(String guichuoitrangthai_hmi)
{
  giaotiephmi();
  //sendCommand("page 0"); // chuyen ve trang 0
  Serial2.print("t3.txt="); // trang thai AGV
  Serial2.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial2.print(guichuoitrangthai_hmi);  // This is the text you want to send to that object and atribute mentioned before.
  Serial2.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  delay(100);
  giaotiephmi();
}
void guithongtinloi_hmi(String guichuoiloi_hmi)
{
  //sendCommand("page 0"); // chuyen ve trang 0
  giaotiephmi();
  Serial2.print("t5.txt="); // trang thai AGV
  Serial2.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial2.print(guichuoiloi_hmi);  // This is the text you want to send to that object and atribute mentioned before.
  Serial2.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial2.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  delay(100);
}
void docrfid()// update 23.4// thiếu thông tin nếu vị trí của F lớn hơn 3.......................
{
   // Serial1.println("$S#");// gui yeu cau dau doc gui lại du lieu
  int checkSum_docrfid=0;
  int check_coData=0;
  while(Serial1.available()>1) 
  {
   check_coData=1;
   errorDoc=0;
   // size_t num_read = Serial1.readBytesUntil('\r', name_arr, sizeof(name_arr)-1 );
    size_t num_read = Serial1.readBytes( name_arr, sizeof(name_arr)-1 );
    name_arr[num_read] = '\0';
    Serial.println(name_arr);
    digitalWrite(denbao_RFID,HIGH);
    for (int i = 0; i < num_read; i++) 
      {
        if (name_arr[i] == 'F') {
        if(i>=4) errorDoc=1; //update 23.4// thiếu thông tin nếu vị trí của F lớn hơn 3......................
        // Lưu giá trị từ vị trí của ký tự 'F' vào các biến serNum0, serNum1, serNum2, serNum3
        serNum0 = name_arr[i + 12];
        serNum1 = name_arr[i + 13];
        serNum2 = name_arr[i + 14];
        serNum3 = name_arr[i + 15];
        checkSum_docrfid=checkSum_docrfid+1;
        }
      }
    name_arr[0]='0';        
    if((serNum0!=0)||(serNum1!=0)||(serNum2!=0)||(serNum3!=0))
    {
    int The1= serNum0 - '0';
    int The2= serNum1 - '0';
    int The3= serNum2 - '0';
    int The4= serNum3 - '0';
    String combinedStr =String('!') + String(The1) + String(The2) + String(The3) + String(The4);
    mySerial.println(combinedStr);
    //delay(20);// sau khi gửi xong cũng ko gửi liên tục đâu.
    }
    digitalWrite(denbao_RFID,LOW);
    //update 23.4// thiếu thông tin nếu vị trí của F lớn hơn 3......................
  if (((checkSum_docrfid==0)||(errorDoc==1))&&(check_coData==1)) // 
  //if ((checkSum_docrfid==0)||(check_coData==1)||(errorDoc==1)) // nếu có data gửi từ đầu đọc rfid về và ko tìm thấy ký tự 'F' nào;
  {
    dunglai();
    while(1)
    {
      guithongtinloi_hmi("Loi doc du lieu RFID");
      delay(200);
      nhacstop();
      giaotiephmi();
    }
  }
 }
}
void kiemtra_docrfid_JY()
{
   // Serial1.println("$S#");// gui yeu cau dau doc gui lại du lieu
  while(Serial1.available()>1) 
  {
   // size_t num_read = Serial1.readBytesUntil('\r', name_arr, sizeof(name_arr)-1 );
    size_t num_read = Serial1.readBytes( name_arr, sizeof(name_arr)-1 );
    name_arr[num_read] = '\0';
    Serial.println(name_arr);
    //digitalWrite(denbao_mauxanh,HIGH);
  if(name_arr[0]=='$')
  {
    name_arr1[0]=name_arr[1];
    name_arr1[1]=name_arr[2];
    name_arr1[2]=name_arr[3];
    name_arr1[3]=name_arr[4];
    name_arr1[4]=name_arr[5];
    name_arr1[5]=name_arr[6];
    name_arr1[6]=name_arr[7];
    name_arr1[7]=name_arr[8];
    name_arr1[8]=name_arr[9];
    name_arr1[9]=name_arr[10];
    name_arr1[10]=name_arr[11];
    name_arr1[11]=name_arr[12];
    name_arr1[12]=name_arr[13];
    name_arr1[13]=name_arr[14];
    name_arr1[14]=name_arr[15];
    name_arr1[15]=name_arr[16];
    name_arr1[16]=name_arr[17];
    name_arr1[17]=name_arr[18];
    String str((char*)name_arr1);
    Serial.print(name_arr1[12],DEC);
    Serial.print(", ");
    Serial.print(name_arr1[13],DEC);
    Serial.print(", ");
    Serial.print(name_arr1[14],DEC);
    Serial.print(", ");
    Serial.print(name_arr1[15],DEC);
    serNum0=name_arr1[12];
    serNum1=name_arr1[13];
    serNum2=name_arr1[14];
    serNum3=name_arr1[15];
    name_arr[0]='0';
    while(1)
    {
      guithongtinloi_hmi("DAU DOC THE SAN SANG");
      delay(200);
      giaotiephmi();
      delay(1000);
      guithongtinloi_hmi(" ");
      delay(200);
      tatnhac();
      demthoat=1;
      clear_rifd();
      break;
    }
   // Serial1.println("$S#");
  }
  else
  {
    while(1)
    {
      guithongtinloi_hmi("DAU DOC THE SAN SANG");
      delay(200);
      giaotiephmi();
      delay(1000);
      guithongtinloi_hmi(" ");
      delay(200);
      tatnhac();
      demthoat=1;
      clear_rifd();
      break;
    }
  }
 }
}
void clear_rifd()
{
  serNum0=0;
  serNum1=0;
  serNum2=0;
  serNum3=0;
  duytri_queophai=0;
  duytri_queotrai=0;
  digitalWrite(denbao_RFID,LOW);
}
/*
String nhan_ESP32S3_binh() // NHAN TIN HIEU TU ESP
{
   String input ="";
  if(mySerial.available() >= 1)
  {
    input=mySerial.readString();
    Serial.println(input); 
  }
  return input;
}
String nhan_ESP32S3_binh() // NHAN TIN HIEU TU ESP
{
   String input = "";
   char terminator = '\n';  // Ký tự phân tách (line break)
   
   while (mySerial.available() > 0)
   {
      char c = mySerial.read();  // Đọc ký tự từ mySerial
      
      if (c != terminator)
      {
         input += c;  // Thêm ký tự vào chuỗi input
      }
   }
   if (input != "") {
      Serial.println(input);
   }
   return input;
}
*/
// Code ORG dùng từ lâu rồi
/*
void gui_ESP32S3(int number)
{
  unsigned long ConditionTime=0;
  digitalWrite(giaotiep_ESP32S3_X2,HIGH);
  delay(200);
  label1:
  {
    ConditionTime=millis();
    char character = '$';
    if(number>9)
    {
      String combinedString = String(character) + String(number);
      mySerial.println(combinedString);
      delay(300);
    }
    if(number<=9)
    {
      String combinedString = String(character) + String(0) + String(number);
      mySerial.println(combinedString);
      delay(300);
    }
  }
  //Serial.println("dang ket Set X2");
  //Serial.println(nhan_ESP32S3_binh());
  delay(500);
  if(digitalRead(47)!=0)
  {
    Serial.print("wating..........");
    Serial.println(number);
    goto label1;
  }
  if(digitalRead(47)==0) 
  {
    check[number]=0;
    Serial.print("da gan chuyen:");
    Serial.print(number);
    Serial.print(" =");
    Serial.println(check[number]);   
  }
  digitalWrite(giaotiep_ESP32S3_X2,LOW);
}
*/
// code nâng cấp 18.6 loại bỏ delay dẫn đến không đọc được lúc chân 47 ==0. (có thể kéo dài thời gian output của chân Y2 của ESP32 ra cũng được => cách đấy hơi đần)

void gui_ESP32S3(int number)
{
    bool success = false;
    Serial.println("Vao vong lap moi");
    digitalWrite(giaotiep_ESP32S3_X2, HIGH);// Bật chân giao tiếp khi bắt đầu gửi
    delay(250); // Đợi cho chân giao tiếp ESP32 ổn định
    while (!success) 
      {
        char character = '$';
        String combinedString = (number > 9) ? String(character) + String(number) : String(character) + String(0) + String(number);
        Serial.println(combinedString);
        mySerial.println(combinedString);
        
        unsigned long sendTime = millis();
        while (millis() - sendTime < 1000) // Timeout sau 500ms cho mỗi lần gửi // thinking về thời gian gửi 500ms có nhanh quá ko.
        {
            if ((digitalRead(48) == 0)||(digitalRead(47) == 0))
            {
                check[number] = 0;
                success = true;
                Serial.print("check thu:");
                Serial.print(number);
                Serial.println("= 0");
                break; // Thoát vòng lặp nếu gửi thành công
            }
        }
        //nhận được chân 48 có tín hiệu (Y1_ESP xác nhận được chuyền cần đi)=> thoát
        if (success)
        {
            break; // Thoát vòng lặp chính nếu gửi thành công
        }
        // Không nhận được => In waiting
        if (!success)
        {
            Serial.print("Waiting 1...: ");
            Serial.println(number);
            // Xử lý khi không gửi được dữ liệu
        }       
      }
    while(digitalRead(47)==1) // (Y2_ESP đã nhận được bản tin từ master)=> thoát
    {
            Serial.print("Waiting 2...: ");
            Serial.println(number);
    }
    digitalWrite(giaotiep_ESP32S3_X2, LOW); // Tắt chân giao tiếp sau khi hoàn thành gửi
    delay(100);
}

void nhan_ESP32S3() // NHAN TIN HIEU TU ESP
{
  if(mySerial.available() > 1)
  {
    String input = mySerial.readString();
    Serial.println(input); 
    delay(20);
    for (i = 0; i < input.length(); i++) // viet chuong trinh cho 1 xe AGV sau do chinh sua cho AGV tiep theo
      {
        if (input.charAt(i) =='#')  
          {
            moc1 = i; //Tìm vị trí của "#" de tach chuoi lan 1
          }
        else if (input.charAt(i) =='A') ////Tìm vị trí của "A" de tach chuoi lan 2
          {
            moc = i; //Tìm vị trí của "A"
            //Serial.println(moc1);
            chuoi=input;
            chuoi.remove(0,moc1+1);
            chuoi.remove(moc);
            line= chuoi.toInt();
            moc=0;
            Serial.print("da nhan line "); 
            Serial.println(line);
            digitalWrite(giaotiep_ESP32S3_X0, HIGH);  
            Serial.println("Bat x0");           
            delay(3000);
            digitalWrite(giaotiep_ESP32S3_X0, LOW);  
            Serial.println("Tat x0");
            //mySerial.end();        
            break;
          }
        else if (input.charAt(i) =='F') //ESP GƯI "100F" DE XÁC NHẬN ESP ĐÃ NHẬN DỮ LIỆU
          {
            moc = i; //Tìm vị trí của "F"
            //Serial.println(moc1);
            chuoi=input;
            chuoi.remove(moc);
            inputnhanlenhKT= chuoi;
            moc=0;
            chuoi="";
            Serial.print("da nhan phan hoi esp "); 
            Serial.println(inputnhanlenhKT);    
            break;
          }
      }            
    if((line==1)&&(line1==0))
    {
      line1=1;// bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[1]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 1");
      delay(100);
    }
    if((line==2)&&(line2==0))
    {
      line2=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[2]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 14");
      delay(100);
    }
    if((line==3)&&(line3==0))
    {
      line3=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[3]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 4");
      delay(100);
    }
    if((line==4)&&(line4==0))
    {
      line4=1;// bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[4]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 7");
      delay(100);
    }
    if((line==5)&&(line5==0))
    {
      line5=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[5]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 6");
      delay(100); 
    }
    if((line==6)&&(line6==0))
    {
      line6=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[6]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 5");
      delay(100); 
    }
    if((line==7)&&(line7==0))
    {
      line7=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[7]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 8");
      delay(100); 
    }
    if((line==8)&&(line8==0))
    {
      line8=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[8]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 2");
      delay(100); 
    }
    if((line==9)&&(line9==0))
    {
      line9=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[9]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 9");
      delay(100); 
    }
    if((line==10)&&(line10==0))
    {
      line10=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[10]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 13");
      delay(100); 
    }
    if((line==11)&&(line11==0))
    {
      line11=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[11]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 10");
      delay(100); 
    }
    if((line==12)&&(line12==0))
    {
      line12=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[12]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 11");
      delay(100); 
    }
    if((line==13)&&(line13==0))
    {
      line13=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[13]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 12");
      delay(100); 
    }
    if((line==14)&&(line14==0))
    {
      line14=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[14]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 15");
      delay(100); 
    }
    if((line==15)&&(line15==0))
    {
      line15=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[15]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 3");
      delay(100); 
    }
    if((line==16)&&(line16==0))
    {
      line16=1; //bỏ chỗ này đi để chạy ra các line ngẫu nhiên
      //Binh them 19.6
      Chuyen[16]=1;
      deba1=1;
      guitrangthai_hmi("AGV cấp hàng chuyền 16");
      delay(100); 
    }                              
  }
}
void giamtoc()
{
      initial_motor_speed=number1;
      clear_rifd();
      duytrigiamtoc=1;
      tatvatcan=1;
      //Serial.print("da tat vat can = ");
     //Serial.println(tatvatcan);
}
void giamtoc2()
{
      initial_motor_speed=30;
      clear_rifd();
      duytrigiamtoc=1;
      tatvatcan=1;
      Serial.println("check point: Tat vat can 2");
     // Serial.print("da tat vat can = ");
     // Serial.println(tatvatcan);
}
void queotrai() // error duong bit ben trai tuong ung ss1
{
  error=errorqueotrai;
  calculate_pid();
  motor_control();
  hamdelayqueo_trai();
  clear_rifd();
  calculate_pid();
  motor_control();
}
void queophai() //error am bit ben phai tinh tu ss16
{
  error=errorqueophai;
  calculate_pid();
  motor_control();
  hamdelayqueo_phai();
  clear_rifd();
  calculate_pid();
  motor_control();
}
void queotrai_90()
{
 // tatvatcan=1;
  mothang();
  chieuqueotrai_90();
  analogWrite(PWM_T, 40);
  analogWrite(PWM_P, 40);
  while((digitalRead(ss15)==1)||(digitalRead(ss16)==1))
  {
    Serial.println("dang queo trai 90");
  }
  while((digitalRead(ss8)==1)&&(digitalRead(ss9)==1))
  {
    Serial.println("dang duy tri queo trai 90");
    if((digitalRead(ss7)==0)||(digitalRead(ss8)==0)||(digitalRead(ss9)==0)||(digitalRead(ss10)==0))
    {
  //    tatvatcan=0;
      dunglaigap();
      if(state=="toi") chieutoi();
      else if(state=="lui") chieului();   
      break;
    }
  }
  clear_rifd();
 // deba();
}
void queophai_90()
{
 // tatvatcan=1;
  mothang();
  chieuqueophai_90();
  analogWrite(PWM_T, 40);
  analogWrite(PWM_P, 40);
  while((digitalRead(ss1)==1)||(digitalRead(ss2)==1))
  {
    Serial.println("dang queo phai 90");
  }
  while((digitalRead(ss8)==1)&&(digitalRead(ss9)==1))
  {
 
    Serial.println("dang duy tri queo phai 90");
    if((digitalRead(ss7)==0)||(digitalRead(ss8)==0)||(digitalRead(ss9)==0)||(digitalRead(ss10)==0))
    {
  //    tatvatcan=0;
      dunglaigap();
      if(state=="toi") chieutoi();
      else if(state=="lui") chieului();   
      break;
    }
  }
  clear_rifd();
 // deba();
}
void hamdelayqueo_trai()
{
  while((digitalRead(ss16)==1)&&(duytri_queotrai==0))
  {
    Serial.println("dangqueo");
    speed_min=0;
    calculate_pid();
    motor_control();
    while((digitalRead(ss16)==0)||(duytri_queotrai==2)||((digitalRead(ss1)==1) &&(digitalRead(ss2)==1) &&(digitalRead(ss3)==1) &&(digitalRead(ss4)==1) &&(digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))
      {
        Serial.println("dangqueo_tiep");
         duytri_queotrai=2;
         calculate_pid();
         motor_control();
         error=1.7;
          while(duytri_queotrai==2)
          {
            Serial.println("dangqueo_tieptuc");
            //read_sensor_values();
          if(((digitalRead(ss4)==0)||(digitalRead(ss5)==0)||(digitalRead(ss6)==0)||(digitalRead(ss7)==0)||(digitalRead(ss8)==0)||(digitalRead(ss9)==0)||(digitalRead(ss10)==0)||(digitalRead(ss11)==0)||(digitalRead(ss12)==0)||(digitalRead(ss13)==0))&&(digitalRead(ss16)==1))
          {
             speed_min=5;
             Serial.println("da vao tam line");
             read_sensor_values();
             calculate_pid();
             motor_control();
             duytri_queotrai=1;
             break;
          }
          }
          break;
      }
  }
}
void hamdelayqueo_phai()
{
 while((digitalRead(ss1)==1)&&(duytri_queophai==0))
  {
    Serial.println("dangqueo");
    speed_min=0;
    calculate_pid();
    motor_control();
    Serial.println(error);
        while((digitalRead(ss1)==0)||(duytri_queophai==2)||((digitalRead(ss1)==1) &&(digitalRead(ss2)==1) &&(digitalRead(ss3)==1) &&(digitalRead(ss4)==1) &&(digitalRead(ss5)==1) && (digitalRead(ss6)==1) && (digitalRead(ss7)==1) && (digitalRead(ss8)==1) && (digitalRead(ss9)==1) && (digitalRead(ss10)==1) && (digitalRead(ss11)==1) && (digitalRead(ss12)==1) && (digitalRead(ss13)==1) && (digitalRead(ss14)==1) && (digitalRead(ss15)==1) && (digitalRead(ss16)==1)))
        {
          Serial.println("dangqueo_tiep");
          calculate_pid();
          motor_control();
          duytri_queophai=2;
          error=-1.7;
         while(duytri_queophai==2)
         {
          Serial.println("dangqueo_tieptuc");
         // read_sensor_values();
         if(((digitalRead(ss16)==0)||(digitalRead(ss15)==0)||(digitalRead(ss14)==0)||(digitalRead(ss13)==0)||(digitalRead(ss12)==0)||(digitalRead(ss11)==0)||(digitalRead(ss10)==0)||(digitalRead(ss9)==0)||(digitalRead(ss8)==0)||(digitalRead(ss7)==0)||(digitalRead(ss6)==0)||(digitalRead(ss5)==0)||(digitalRead(ss4)==0))&&(digitalRead(ss1)==1))
          {
             Serial.println("da vao tam line");
             read_sensor_values();
             calculate_pid();
             motor_control();
             duytri_queophai=1;
             speed_min=5;
             break;
          }
         }
          break;
      }
  }
}
void check_wait1()
{ 
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==49))   // the 51 dung lai cho cap hang
   {
     initial_motor_speed=number1;
     duytrigiamtoc=1;
     dunglai();
     initial_motor_speed=number0;
     duytrigiamtoc=0;
     wait=0;
     checkTime=millis();
     clear_rifd();
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56))   //the so 58 dung lai dao chieu ve tram
  {  
      trangthai_vetram=1;
      trangthai_lenchuyen=0;
      dunglaigap();
      chieului();
      delay(100);
      deba();
      clear_rifd();
  }
}
void check_wait2()
{ 
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==50))   // the 52 dung lai cho cap hang
   {
     initial_motor_speed=number1;
     duytrigiamtoc=1;
     dunglai();
     initial_motor_speed=number0;
     duytrigiamtoc=0;
     wait=0;
     checkTime=millis();
     clear_rifd();
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56))   //the so 58 dung lai dao chieu ve tram
  {  
      trangthai_vetram=1;
      trangthai_lenchuyen=0;
      dunglaigap();
      chieului();
      delay(100);
      deba();
      clear_rifd();
  }
}
void check_wait3()
{ 
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==51))   // the 53 dung lai cho cap hang
   {
     initial_motor_speed=number1;
     duytrigiamtoc=1;
     dunglai();
     initial_motor_speed=number0;
     duytrigiamtoc=0;
     wait=0;
     checkTime=millis();
     clear_rifd();
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56))   //the so 58 dung lai dao chieu ve tram
  {  
      trangthai_vetram=1;
      trangthai_lenchuyen=0;
      dunglaigap();
      chieului();
      delay(100);
      deba();
      clear_rifd();
  }
}
void check_wait4()
{ 
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==52))   // the 54 dung lai cho cap hang
   {
     initial_motor_speed=number1;
     duytrigiamtoc=1;
     dunglai();
     initial_motor_speed=number0;
     duytrigiamtoc=0;
     wait=0;
     checkTime=millis();
     clear_rifd();
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56))   //the so 58 dung lai dao chieu ve tram
  {  
      trangthai_vetram=1;
      trangthai_lenchuyen=0;
      dunglaigap();
      chieului();
      delay(100);
      deba();
      clear_rifd();
  }
}
void check_wait5()
{ 
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==53))   // the 55 dung lai cho cap hang
   {
     initial_motor_speed=number1;
     duytrigiamtoc=1;
     dunglai();
     initial_motor_speed=number0;
     duytrigiamtoc=0;
     wait=0;
     checkTime=millis();
     clear_rifd();
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56))   //the so 58 dung lai dao chieu ve tram
  {  
      trangthai_vetram=1;
      trangthai_lenchuyen=0;
      dunglaigap();
      chieului();
      delay(100);
      deba();
      clear_rifd();
  }
}
void caphangchung()
{
  if ((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56))   //the so 58 giam toc, tat vat can
   {  
      giamtoc();
      clear_rifd();
   }
  if(((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==52) && (serNum3 ==48)))   //the so 40 re trai 90/180 

   {
    if(ChoPhepQuay3==1)
    {
      ChoPhepQuay3=0;
      queotrai_90();
      deba();
      clear_rifd();
    }
   }
  if(((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==52) && (serNum3 ==49)))   //the so 41 re trai 90/180 
   {
    if(ChoPhepQuay4==1)
    {
      ChoPhepQuay4=0;
      queotrai_90();
      deba();
      clear_rifd();
    }
   }   
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==57))   //the so 59 tang toc, bat lidar
  {
      datlaitocdo();
      clear_rifd();
  }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==56) && (serNum3 ==51))   //the so 83 dung lai dao chieu ve tram
  {  
      trangthai_vetram=1;
      trangthai_lenchuyen=0;
      dunglaigap();
      chieului();
      delay(100);
      deba();
      clear_rifd();
  }
}   
void caphangline1()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==49))   //the so 61 dung cap hang line 1
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[1]=0;
     sendCommand("b1.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      initial_motor_speed=number0;
      duytrigiamtoc=0;      
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }
}
void caphangline2()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==50))   //the so 62 dung cap hang line 2
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[2]=0;
     sendCommand("b2.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      initial_motor_speed=number0;
      duytrigiamtoc=0;      
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }   
}
void caphangline3()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==51))   //the so 63 dung cap hang line 3
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[3]=0;
     sendCommand("b5.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;          
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline4()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==52))   //the so 64 dung cap hang line 4
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[4]=0;
     sendCommand("b6.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;          
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }   
}
void caphangline5()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==53))   //the so 65 dung cap hang line 5
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[5]=0;
     sendCommand("b7.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;  
      initial_motor_speed=number0;    
      delay(300);
      guithongtinloi_hmi("");
      deba();
      //chieutoi();
     //chieului();// cho hàm vetram ở đây được k nhỉ// bam tiep tu tu lui về, line của e phải tiến lên mới về được, chứ k phải lùi, tự nhiên xe lùi ý
   }   
}
void caphangline6()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==54))   //the so 66 dung cap hang line 6
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[6]=0;
     sendCommand("b9.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;  
      initial_motor_speed=number0;     
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }   
}
void caphangline7()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==55))   //the so 67 dung cap hang line 7
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[7]=0;
     sendCommand("b10.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;    
      initial_motor_speed=number0; 
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }  
}
void caphangline8()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==56))   //the so 68 dung cap hang line 8
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[8]=0;
     sendCommand("b0.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;     
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }     
}
void caphangline9()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==54) && (serNum3 ==57))   //the so 69 dung cap hang line 9
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[9]=0;
     sendCommand("b14.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0;  
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline10()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==48))   //the so 70 dung cap hang line 3
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[10]=0;
     sendCommand("b13.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline11()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==49))   //the so 71 dung cap hang line 11
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[11]=0;
     sendCommand("b3.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline12()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==50))   //the so 72 dung cap hang line 12
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[12]=0;
     sendCommand("b12.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline13()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==51))   //the so 73 dung cap hang line 13
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[13]=0;
     sendCommand("b15.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline14()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==52))   //the so 74 dung cap hang line 14
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[14]=0;
     sendCommand("b15.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline15()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==53))   //the so 75 dung cap hang line 15
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[15]=0;
     sendCommand("b17.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void caphangline16()
{
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==55) && (serNum3 ==54))   //the so 76 dung cap hang line 16
   {
     initial_motor_speed=55;
     duytrigiamtoc=1;
     dunglai();
     chaytiep=0;
     Chuyen[16]=0;
     sendCommand("b18.bco=40179");
      while(1)
      {
        nhacdenline();
        batden_vang_3thap();       
        giaotiephmi();
        guithongtinloi_hmi("lay hang va bam tiep tuc");
        if(chaytiep==1)
          {
            Serial.println("da lay hang");
            duytrigiamtoc=0;
            break;
          }
      }
      chaytiep=0;
      clear_rifd();
      chieului();
      for (int i = 0; i < 50; i++) 
      {
        if (Chuyen[i] != 0) 
        {
          // Nếu có ít nhất một phần tử khác 0, dao chieu tien len cap hang
          chieutoi();
          break;
        }
      }
      duytrigiamtoc=0; 
      initial_motor_speed=number0;
      delay(300);
      guithongtinloi_hmi("");
      deba();
   }    
}
void vetram()
{
  /*
  // phân nhánh rẽ từng chuyền// giả sử đang viết cho line 1
  if((line==1)&&(trangthai_vetram==1))
  {
    if(((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==50) && (serNum3 ==56))) // thẻ 28, line 1 rẽ về luôn.
    {
      queotrai_90();
      deba();
      clear_rifd();
    }
  }
 // phân nhánh rẽ từng chuyền// giả sử đang viết cho line 2
  if((line==2)&&(trangthai_vetram==1))
  {
    if(((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==50) && (serNum3 ==56))) // thẻ 28, line 1 rẽ về luôn.
    {
      queotrai_90();
      deba();
      clear_rifd();
    }
  }
  */
  // code ve tram chung
    if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==57))  //the so 59 giam toc, tat vat can
   {  
      giamtoc();
      clear_rifd();
   }
  if(((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==56)))   //the so 58 tang toc, bat lidar
   {  
      datlaitocdo();
      clear_rifd();
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==52) && (serNum3 ==49))   // the so 41 re phâi
   {
    if(ChoPhepQuay1==1)
    {
      ChoPhepQuay1=0;
      queophai_90();
      deba();
      clear_rifd();
    }
   }
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==52) && (serNum3 ==48))   // the so 40 re phâi
   {
    if(ChoPhepQuay2==1)
    {
      ChoPhepQuay2=0;
      queophai_90();
      deba();
      clear_rifd();
    }
   }     
    if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==55))   // the 57 sieu giam toc, tat lidar
   {
      Serial.println("check point: Tat vat can");
      giamtoc2();
      clear_rifd();
      tatnhac();
   }
}
void guidulieu_AGVvetram() // sua lai cho AGV khac nhau
{
  Serial.println("check point 2");   
  while((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==54))  //the so 56 STOP// xong coppy lên đây để AGV gửi về master/ sau khi gửi về master xong mới bấm dc// màn hình hiển thị san sang
   {
      digitalWrite(giaotiep_ESP32S3_X1,HIGH);
      //delay(100);
      nhan_ESP32S3();
      Serial.println("doc gia tri");      
      Serial.println(inputnhanlenhKT);
      if((inputnhanlenhKT=="100") && (line1==0) && (line2==0) && (line3==0) && (line4==0) && (line5==0)&& (line6==0)&& (line7==0)&& (line8==0)&& (line9==0)&& (line10==0)&&(line11==0) && (line12==0) && (line13==0) && (line14==0) && (line15==0)&& (line16==0)&& (line17==0)&& (line18==0)&& (line19==0)&& (line20==0)) // esp32 gui 100F
      {
        Serial.println("kiem tra 2");        
        digitalWrite(giaotiep_ESP32S3_X1,LOW);
        giaotiephmi();
        clear_rifd();
        Serial.println("KET THUC");
        guitrangthai_hmi("SẴN SÀNG"); 
        inputnhanlenhKT="";   
      }
      else if(inputnhanlenhKT=="100") // truong hop nhan lệnh lien tuc
      {
        digitalWrite(giaotiep_ESP32S3_X1,LOW);
        clear_rifd();
        inputnhanlenhKT="";  
      }          
   }
}
void ketthuc()
{
  // AGV01_ Debug 
  //Serial.println("check point 1");
  //Serial.print("line: ");
  //Serial.println(line); 
  if((serNum0 ==48) && (serNum1 ==48) && (serNum2 ==53) && (serNum3 ==54))  //the so 56 STOP // viet ở đây// 
   {
      dunglaigap(); 
      tatnhac();
      //line=0,line1=0,line2=0,line3=0,line4=0,line5=0,line6=0,line7=0,line2=0,line9=0,line10=0,linevetram=0;// ORG
      if ((line1==1)){line1=0,   Chuyen[1]=0;   sendCommand("b1.bco=40179");}
      if ((line2==1)){line2=0,   Chuyen[2]=0;   sendCommand("b2.bco=40179");}
      if ((line3==1)){line3=0,   Chuyen[3]=0;   sendCommand("b5.bco=40179");}
      if ((line4==1)){line4=0,   Chuyen[4]=0;   sendCommand("b6.bco=40179");}
      if ((line5==1)){line5=0,   Chuyen[5]=0;   sendCommand("b7.bco=40179");}
      if ((line6==1)){line6=0,   Chuyen[6]=0;   sendCommand("b9.bco=40179");}
      if ((line7==1)){line7=0,   Chuyen[7]=0;   sendCommand("b10.bco=40179");}
      if ((line8==1)){line8=0,   Chuyen[8]=0;   sendCommand("b0.bco=40179");}
      if ((line9==1)){line9=0,   Chuyen[9]=0;   sendCommand("b14.bco=40179");}
      if ((line10==1)){line10=0, Chuyen[10]=0;  sendCommand("b13.bco=40179");}
      if ((line11==1)){line11=0,   Chuyen[11]=0;   sendCommand("b3.bco=40179");}
      if ((line12==1)){line12=0,   Chuyen[12]=0;   sendCommand("b12.bco=40179");}
      if ((line13==1)){line13=0,   Chuyen[13]=0;   sendCommand("b15.bco=40179");}      
      if ((line14==1)){line14=0,   Chuyen[14]=0;   sendCommand("b16.bco=40179");}
      if ((line15==1)){line15=0,   Chuyen[15]=0;   sendCommand("b17.bco=40179");}
      if ((line16==1)){line16=0,   Chuyen[16]=0;   sendCommand("b18.bco=40179");}
      if ((line17==1)){line17=0,   Chuyen[17]=0;   }// thêm sau
      if ((line18==1)){line18=0,   Chuyen[18]=0;   }// thêm sau
      if ((line19==1)){line19=0,   Chuyen[19]=0;   }// thêm sau
      if ((line20==1)){line20=0,   Chuyen[20]=0;   }     // thêm sau 
      line=0, linevetram=0;
      Serial.print("line: ");
      Serial.println(line);
      duytri_queophai=0;
      duytri_queotrai=0;
      duytrigiamtoc=0;
      trangthai_vetram=0;
      tatvatcan=0;
      trangthai_lenchuyen=0;
      wait=1;
      thoat=0;
      new_line=0;
      temp=0;
      checksum=0;
      Status_phanh=0;
      previousTime = 0;
      dem_laichuyen=0;
      mySerial.begin(9600);
      guithongtinloi_hmi(" ");
      guitrangthai_hmi("Gửi data về Master");
      delay(500);
      dongthang();
      chieutoi();  
      batden_vang_3thap();
      Bat_cambienphat();
      guidulieu_AGVvetram();
      checkTime=0;
      conditionMetTime=0;
      error=0, P=0, I=0, D=0, PID_value=0;
      ChoPhepQuay1=1;
      ChoPhepQuay2=1;
      ChoPhepQuay3=1;
      ChoPhepQuay4=1;
   }
   //clear_rifd();  
}
// Sau khi nhận chuyền từ ESP _ rút ngắn code 5 điểm chờ code lần 1
/*
void Rutgon1()
{
while ((trangthai_lenchuyen==0)||(new_line==0)||(thoat==0))
    {
      label2:
      {
        if(millis()- checkTime < 6400)
        {
          nhacxincaplieu(); 
        }
      if(millis() - checkTime > 6400)
        {
          tatnhac();
        }
      }
      //////////code cu no o day
      giaotiephmi();
      sendCommand("page0=1");
      guithongtinloi_hmi(" Chon chuyen va bam Cap Lieu de chay tiep");
      if(linevetram==1)
        break;
      if(trangthai_lenchuyen==1)
      {
        guithongtinloi_hmi(" ");
        for (int i = 1; i < 50; i++)
        {
          //Serial.print(Chuyen[i]);
          if (Chuyen[i] != 0)
          {
            check[i]=1;
            //Serial.print("gia tri check trước:");
            //Serial.print(i);
            //Serial.print(" gia tri la:");
            //Serial.println(check[i]);
            gui_ESP32S3(i);
            Serial.print("gia tri check sau:");
            Serial.println(check[i]);
            while(check[i]==1)
            {
              Serial.println("ko nhan du data");
            }
          }
        }
        // kiểm tra lại
        for (int m = 1; m < 50; m++)
        {
          checksum=checksum+check[m];
          Serial.print("kiem tra lại check[");
          Serial.print(m);
          Serial.print("] =");
          Serial.println(check[m]);
          if(check[m]==1)
          {
            gui_ESP32S3(m);
          }
        }
      }
      if(checksum==0) thoat=1;
      //code doi vi tri
      if((millis()- checkTime > 30000))
      {
        checkTime=millis();
        goto label2;
      }      
    }
}
*/
//////////////////////////////////////////////////////////////////// code rutgon mới 19.6 lần 2

void Rutgon1()
{
    Serial.println(" Call RG1");
    Serial.println(trangthai_lenchuyen);
    Serial.println(new_line);
    Serial.println(thoat);
    for (int i = 1; i < 50; i++)
    {
      if (Chuyen[i] == 1)
      {
        check[i] = 1;
        Serial.println("co gia tri check");
      }
    }
    while ((trangthai_lenchuyen == 0) || (new_line == 0) || (thoat == 0))
    {
        Serial.println("Da lam ket ham");
        // nếu thoat==1 thì thoát luôn.
        if (thoat == 1)
        {
            Serial.println("loi duoc thoat 1");
            break;
        }
        // bật nhạc xin cấp liệu
        if (millis() - checkTime < 6300)
        {
            nhacxincaplieu();
        }
        else
        {
            tatnhac();
        }

        giaotiephmi();
        sendCommand("page0=1");
        guithongtinloi_hmi(" Chon chuyen va bam Cap Lieu de chay tiep");

        if (linevetram == 1)
            {break;}
        // code dang de cho nay
        if (trangthai_lenchuyen == 1)
        {
          if(dem_laichuyen==0)
          {
            for (int i = 1; i < 50; i++)
            {
              check[i] = Chuyen[i];
              Serial.print("check[");
              Serial.print(i);
              Serial.print("] = ");
              Serial.println(check[i]);
            }
          }
          dem_laichuyen=1;
          guithongtinloi_hmi(" ");
          tatnhac();
          for (int i = 1; i < 50; i++)
            {
              if (check[i] != 0)
              {
                gui_ESP32S3(i);
                while (check[i] == 1)
                  {
                    Serial.println("ko nhan du data");
                  }
              }
            }
        }
        checksum = 0; // Khởi tạo lại checksum trước khi tính toán
        for (int m = 1; m < 50; m++)
          {
            checksum = checksum + check[m];
          }
        Serial.print("checksum: ");
        Serial.println(checksum);
        if (checksum == 0)
        {
            Serial.println("loi duoc thoat 2");
            thoat = 1;
            break;
        }

        if (millis() - checkTime > 30000)
        {
            checkTime = millis();
        }
    }
}

void loop() 
{
  nhan_ESP32S3();
  giaotiephmi();
  // ra điểm cấp hàng 5
  if(((line1==1)||(line8==1)||(line15==1))&&(trangthai_lenchuyen==0))
  {
    if(wait==1)
    {
      Tat_cambienphat();
      delay(1000);
      deba();
      deba1=0;
    }
    while(wait==1)
    {
      docrfid();      
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      check_wait1();
      if(wait==0) break;
    }
    Rutgon1();
    thoat=0;
    delay(100);
    tatnhac();
    deba();
    deba1=0;    
  }
  // ra điểm cấp hàng 4
  if(((line6==1)||(line3==1))&&(trangthai_lenchuyen==0))
  {
    if(wait==1)
    {
      Tat_cambienphat();
      delay(1000);
      deba();
      deba1=0;
    }
    while(wait==1)
    {
      docrfid();      
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      check_wait1();
      if(wait==0) break;
    }
    Rutgon1();
    thoat=0;
    delay(100);
    tatnhac();
    deba();
    deba1=0;
  } 
  // ra điểm cấp hàng 3
  if(((line7==1)||(line4==1)||(line5==1))&&(trangthai_lenchuyen==0))
  {
    if(wait==1)
    {
      Tat_cambienphat();
      delay(1000);
      deba();
      deba1=0;
    }
    while(wait==1)
    {
      docrfid();      
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      check_wait1();
      if(wait==0) break;
    }
    Rutgon1();
    thoat=0;
    delay(100);
    tatnhac();
    deba();
    deba1=0;
  }
  // ra điểm cấp hàng 2
  if(((line13==1)||(line12==1)||(line11==1)||(line9==1))&&(trangthai_lenchuyen==0))
  {
    if(wait==1)
    {
      Tat_cambienphat();
      delay(1000);
      deba();
      deba1=0;
    }
    while(wait==1)
    {
      docrfid();      
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      check_wait1();
      if(wait==0) break;
    }
    Rutgon1();
    thoat=0;
    delay(100);
    tatnhac();
    deba();
    deba1=0;
  }
  // ra điểm cấp hàng 1
  if(((line10==1)||(line2==1)||(line14==1)||(line16==1))&&(trangthai_lenchuyen==0))
  {
    if(wait==1)
    {
      Tat_cambienphat();
      delay(1000);
      deba();
      deba1=0;
    }
    while(wait==1)
    {
      docrfid();      
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      check_wait1();
      if(wait==0) break;
    }
    Rutgon1();
    thoat=0;
    delay(100);
    tatnhac();
    deba();
    deba1=0;
  }  
  // line 1
  if((Chuyen[1])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 1");
    while((Chuyen[1])&&(trangthai_lenchuyen==1))
    {
      nhan_ESP32S3();
      docrfid();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline1();
      ketthuc();
    }    
  }
  // line 2
   if((Chuyen[2])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 14");
    while((Chuyen[2])&&(trangthai_lenchuyen==1))
    {
      nhan_ESP32S3();
      docrfid();      
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline2();
      ketthuc();
    }    
  }
  // line 3
   if((Chuyen[3])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 4");
    while((Chuyen[3])&&(trangthai_lenchuyen==1))
    {
      nhan_ESP32S3();
      docrfid();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline3();
      ketthuc();
    }
  }
  // line 4
  if((Chuyen[4])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 7");
    while((Chuyen[4])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline4();
      ketthuc();
    }
  }
  // line 5
  if((Chuyen[5])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 6");
    while((Chuyen[5])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline5();
      ketthuc();
    }
  }
    // line 6
  if((Chuyen[6])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 5");
    while((Chuyen[6])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline6();
      ketthuc();
    }
  }
  //line 7
  if((Chuyen[7])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 8");
    while((Chuyen[7])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline7();
      ketthuc();
    }
  }
  //line 8
  if((Chuyen[8])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 2");
    while((Chuyen[8])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline8();
      ketthuc();
    }
  }
  //line 9
  if((Chuyen[9])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 9");
    while((Chuyen[9])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline9();
      ketthuc();
    }
  }
  //line 10
  if((Chuyen[10])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 13");
    while((Chuyen[10])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline10();
      ketthuc();
    }
  }
  //line 11
  if((Chuyen[11])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 10");
    while((Chuyen[11])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline11();
      ketthuc();
    }
  }
  //line 12
  if((Chuyen[12])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 11");
    while((Chuyen[12])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline12();
      ketthuc();
    }
  }
  //line 13
  if((Chuyen[13])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 12");
    while((Chuyen[13])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline13();
      ketthuc();
    }
  }
  //line 14
  if((Chuyen[14])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 15");
    while((Chuyen[14])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline14();
      ketthuc();
    }
  }
  //line 15
  if((Chuyen[15])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 3");
    while((Chuyen[15])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline15();
      ketthuc();
    }
  }
  //line 16
  if((Chuyen[16])&&(trangthai_lenchuyen==1)&&(linevetram!=1))
  {
    guitrangthai_hmi("AGV đi cấp hàng chuyền 16");
    while((Chuyen[16])&&(trangthai_lenchuyen==1))
    {
      docrfid();
      nhan_ESP32S3();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      caphangchung();
      caphangline16();
      ketthuc();
    }
  }              
  //ve tram tu dong
  if((trangthai_vetram==1)&&(linevetram!=1))
   {
      while(trangthai_vetram==1)
      {
        docrfid();
        read_sensor_values();
        calculate_pid();
        motor_control();
        vatcan();
        vetram();
        ketthuc();
      }
    }
  // ve tram thu cong
  if (linevetram==1)
  {
    if(deba1==1)
    deba();
    deba1=0;
    while(linevetram==1)
    {
      docrfid();
      read_sensor_values();
      calculate_pid();
      motor_control();
      vatcan();
      vetram();
      ketthuc();   
    }
  }
}

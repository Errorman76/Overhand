#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MPU6050.h>

SoftwareSerial HM10(8, 9); // TX, RX

// �뒳由�
volatile bool sleep_flag = false;
volatile unsigned long wake_time = 0;

// 踰꾪듉 �씤�꽣�읇�듃
volatile int mode = 0;  // �닽�옄/�븳湲� 蹂��솚 紐⑤뱶
volatile int mode_flag = 0; // �닽�옄/�븳湲� 蹂��솚紐⑤뱶
volatile unsigned long interrupt_time = 0;

// �븬�젰 �꽱�꽌
int p_flag[5] = {0};
int p_order[5] = { -1, -1, -1, -1, -1};
int order = 0; // p_order 利앷� 蹂��닔
unsigned int thresholds[7] = { 50, 50, 50, 50, 50, 300, 100 };  // 媛� �넀媛��씫 �븳怨꾧컪

// �옄�씠濡� �꽱�꽌
MPU6050 mpu;
float pitch = 0;
float pitch_temp = 0;
float yaw = 0;
float yaw_temp = 0;
int backspace = 0;
int enter = 0;
unsigned long gyro_time = 0;
bool gyro_flag = false;

// 硫붿떆吏�
int r_index = 0;
char r_msg[4] = { 0, 0, 0, 0 };
byte s_msg[4] = { 0, 0, 0, 0 };

const int MAX_CHARS = 4;
const int RECEIVE_LIMIT = 4;

// EEPROM
const int FIN_ADDR = 1;
const int GYRO_ADDR = 11;
const int SLEEP_ADDR = 13;

void setup ()
{
  HM10.begin(9600);
  Serial.begin(9600);

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), btn_interrupt, LOW);
  load_data();
  wake_time = millis();
}

void loop ()
{
  if (!sleep_flag)
  {
    if (receive() >= MAX_CHARS)
    {
      parse_msg(); 
      wake_time = millis();
    } 
    else
    {
      if(make_sign() != -1 && millis() - wake_time >= 200)
      {
        wake_time = millis();
      }
      else if(thresholds[6] == 100);
      else if(millis() - wake_time > thresholds[6] * 1000 * 60)
        sleep_flag = true;
    }
    clear_data();
  }
  else if(sleep_flag)
    Serial.println("Sleeping~zZ");
}

void load_data()
{
  unsigned int temp;
  for(int i = 0; i < 7; i++)
  {
    temp = 0x00FF & EEPROM.read(FIN_ADDR + (i*2));
    temp = temp << 8;
    temp = temp | (0x00FF & EEPROM.read(FIN_ADDR + (i*2) + 1));
    thresholds[i] = temp;
  }  
}

void save_data(int addr, unsigned int value)
{
  byte temp;
  for(int i = 1; i >= 0; i--)
  {
    temp = (byte) (0x00FF & value);
    value = value >> 8;
    EEPROM.write(addr + i, temp);
  }
}
int receive()
{
  for (int i = 0; i < RECEIVE_LIMIT; i++)
  {
    char c = -1;
    if (HM10.available())
      c = HM10.read();
    if (r_index < MAX_CHARS && c != -1)
      r_msg[r_index++] = c;
    if (r_index >= MAX_CHARS)
    {
      r_index = 0;
      return MAX_CHARS;
    }
  }
  return r_index;
}

void parse_msg()
{
  unsigned int r_type = 0, r_target = 0, r_value = 0;
  r_type = 0x00FF & (r_msg[0] & 0xF0);
  r_target = 0x00FF & (r_msg[0] & 0x0F);
  r_value = 0x00FF & r_msg[1];
  r_value = r_value << 8;
  r_value = r_value | (0x00FF & r_msg[2]);
  
  for (int i = 0; i < 4; i++)
    s_msg[i] = 0;
  
  if (r_type == 0x00)
  {
    if (r_target == 0) // PRESSURE �씤 寃쎌슦
    {
      for(int i = 0; i < 5; i++)
      {
        s_msg[0]   = 0x10 | (i+1);
        s_msg[1] = (byte) ((thresholds[i] & 0xFF00) >> 8);
        s_msg[2] = (byte) (thresholds[i] & 0x00FF);
        HM10.write(s_msg, 4);
        delay(100); 
      }
    }
    else if (r_target >= 1 && r_target <= 5)
    {
      int fin = r_target;
      int i = 0;
      s_msg[0] = 0x10 | fin;
      while (true)
      {
        unsigned int p = analogRead(fin);
        if(p != 0)
        {
          s_msg[1] = (byte) ((p & 0xFF00) >> 8);
          s_msg[2] = (byte) (p & 0x00FF);
          Serial.print(fin);
          Serial.print("POWER IS : ");
          Serial.print(s_msg[1], BIN);
          Serial.print(", ");
          Serial.println(s_msg[2], BIN);
          HM10.write(s_msg, 4);
        }
        

        if (receive() >= MAX_CHARS)
        {
          r_type = 0x00FF & (r_msg[0] & 0xF0);
          r_target = 0x00FF & (r_msg[0] & 0x0F);
          r_value = 0x00FF & r_msg[1];
          r_value = r_value << 8;
          r_value = r_value | (0x00FF & r_msg[2]);
          Serial.print("Type : ");
          Serial.println(r_type);
          if (r_type == 0x20 && r_target == fin)
          {
            thresholds[fin - 1] = r_value;
            save_data(FIN_ADDR + ((fin - 1) * 2), r_value);
            Serial.print("CHANGING THRESHOLD is : ");
            Serial.println(thresholds[fin - 1]);
            break;
          }
          else if (r_type == 0x20)
          {
            Serial.print("Type is ok");
            Serial.println(r_target);
             if(r_target == 15)
             {
              Serial.print("Target is 15");
              break;
             }
          }
        }
        delay(100);
      }
    }
    else if (r_target == 6)
    {
      s_msg[0] = 0x16;
      s_msg[1] = (byte) ((thresholds[6] & 0xFF00) >> 8);
      s_msg[2] = (byte) (thresholds[6] & 0x00FF);
      HM10.write(s_msg, 4);
    }
    else if (r_target == 7)
    {
      s_msg[0] = 0x17;
      s_msg[1] = (byte) ((thresholds[5] & 0xFF00) >> 8);
      s_msg[2] = (byte) (thresholds[5] & 0x00FF);
      HM10.write(s_msg, 4);
    }
  }
  else if (r_type == 0x20)
  {
    if(r_target >= 1 && r_target <= 5)
    {
      thresholds[r_target - 1] = r_value;
      Serial.print(r_target);
      Serial.print("번 센서 감도 : ");
      Serial.println(r_value);
      save_data(FIN_ADDR + ((r_target - 1) * 2), r_value);
    }
    else if(r_target == 6)
    {
      thresholds[6] = r_value;
      save_data(SLEEP_ADDR, r_value); 
    }
    else if(r_target == 7)
    {
      thresholds[5] = r_value;
      save_data(GYRO_ADDR, r_value); 
    }
  }
}

void clear_data()
{
  for (int i = 0; i < 5; i++)
  {
    p_flag[i] = 0;
    p_order[i] = -1;
  }
  for (int i = 0; i < 4; i++)
    s_msg[i] = 0;
    
  backspace = 0;
  enter = 0;
  order = 0;
}

int make_sign()
{
  int sign = -1;
  if (is_push(1) == true || is_push(2) == true || is_push(3) == true || is_push(4) == true || is_push(5) == true)
  {
    s_msg[0] = 0x20;
    press_check();
    sign = press_to_sign();
  }
  else
  {
    s_msg[0] = 0x27;
    gyro_check();
    sign = gyro_to_sign();
  }

  if(sign != 99 && sign != -1)
  {
    s_msg[2] = sign;
    HM10.write(s_msg, 4);
    Serial.print(" SIGN = ");
    Serial.println(sign);
    
    if(s_msg[0] == 0x20 && mode != 2)
    {
      mode = !mode;
      mode_flag = 0;
    }
    else if(s_msg[2] == 43 || s_msg[2] == 44)
    {
      mode = 0;
      mode_flag = 0;
    }
    else if(s_msg[2] == 45)
    {
      mode = !mode;
      mode_flag = 0;
    }
  }
  return sign;
}

int gyro_to_sign()
{
  if (enter == 1)
    return 43;
  else if (backspace == 1)
    return 45;
  else
    return -1;
}

int press_to_sign()
{
  int temp = 0;
  int digit = 0;

  if (p_order[4] != -1) digit = 5;
  else if (p_order[3] != -1) digit = 4;
  else if (p_order[2] != -1) digit = 3;
  else if (p_order[1] != -1) digit = 2;
  else if (p_order[0] != -1) digit = 1;

  for (int i = 0; i < digit; i++)
    temp += (p_order[i] * ppow(digit - i - 1));

  if (mode == 0)
  {
    switch (temp)
    {
      case 4: return 10;
      case 45: return 11;
      case 14: case 41: return 12;
      case 24: case 42: return 13;
      case 245: case 425: return 14;
      case 514: case 541: return 15;
      case 1: return 16;
      case 1234: case 1243: case 1324: case 1342: case 1423: case 1432:
      case 2134: case 2143: case 2314: case 2341: case 2413: case 2431:
      case 3124: case 3142: case 3214: case 3241: case 3412: case 3421:
      case 4123: case 4132: case 4213: case 4231: case 4312: case 4321: return 17;
      case 12345: case 12435: case 13245: case 13425: case 14235: case 14325:
      case 21345: case 21435: case 23145: case 23415: case 24135: case 24315:
      case 31245: case 31425: case 32145: case 32415: case 34125: case 34215:
      case 41235: case 41325: case 42135: case 42315: case 43125: case 43215: return 18;
      case 34: case 43: return 19;
      case 345: case 435: return 20;
      case 3: return 21;
      case 2: return 22;
      case 25: return 23;
      case 52: return 24;
      case 54: return 25;
      case 524: case 542: return 26;
      case 51: return 27;
      case 53: return 28;
      case 123: case 132: case 213: case 231: case 312: case 321: return 44;
      default: return 99;
    }
  }
  else if (mode == 1)
  {
    switch (temp)
    {
      case 2: return 29;
      case 24: return 30;
      case 25: return 31;
      case 254: return 32;
      case 1: return 33;
      case 14: return 34;
      case 51: return 35;
      case 514: return 36;
      case 53: return 37;
      case 533: return 38;
      case 35: return 39;
      case 355: return 40;
      case 3: return 41;
      case 4: return 42;
      case 123: case 132: case 213: case 231: case 312: case 321: return 44;
      default: return 99;
    }
  }
  else if (mode == 2)
  {
    switch (temp)
    {
      case 5: return 0;
      case 4: return 1;
      case 3: return 2;
      case 2: return 3;
      case 1: return 4;
      case 54: case 45: return 5;
      case 35: case 53: return 6;
      case 25: case 52: return 7;
      case 15: case 51: return 8;
      case 1234: case 1243: case 1324: case 1342: case 1423: case 1432:
      case 2134: case 2143: case 2314: case 2341: case 2413: case 2431:
      case 3124: case 3142: case 3214: case 3241: case 3412: case 3421:
      case 4123: case 4132: case 4213: case 4231: case 4312: case 4321: return 9;
      case 123: case 132: case 213: case 231: case 312: case 321: return 44;
      default: return 99;
    }
  }
  else
    return 99;
}

int ppow(int ex)
{
  int base = 1;
  if (ex == 0) return 1;
  else
  {
    for (int i = 0; i < ex; i++) base *= 10;
    return base;
  }
}

void gyro_check()
{
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * 0.01;
  yaw = yaw + norm.ZAxis * 0.01;
  
  if(millis() - gyro_time >= 300)
  {
    gyro_time = millis();
    pitch_temp = pitch;
    yaw_temp = yaw;   
    gyro_flag = false;
  }

  if(pitch_temp - pitch > thresholds[5] && gyro_flag == false)
  {
    backspace = 1; 
    gyro_flag = true;
  }
  else if(yaw_temp - yaw > thresholds[5] && gyro_flag == false)
  {
    enter = 1;
    gyro_flag = true;
  }
}

void press_check()
{
  while (is_push(1) == true || is_push(2) == true || is_push(3) == true || is_push(4) == true || is_push(5) == true)
  {
    for (int pin = 0; pin < 5; pin++)
    {
      if (thresholds[pin] < analogRead(pin + 1) && p_flag[pin] == 0)
      {
        p_flag[pin] = 1; 
        p_order[order++] = pin + 1;
      }
      else if (thresholds[pin] > analogRead(pin + 1) && p_flag[pin] == 1)
        p_flag[pin] = 0;
    }
  }
  Serial.print("p_order : ");
  Serial.print(p_order[0]);
  Serial.print(p_order[1]);
  Serial.print(p_order[2]);
  Serial.print(p_order[3]);
  Serial.println(p_order[4]);
}

boolean is_push(int pin)
{
  if (thresholds[pin - 1] < analogRead(pin))
    return true;
  return false;
}

void btn_interrupt()
{
  if(millis() - interrupt_time >= 50)
  {
    interrupt_time = millis();
    if (sleep_flag == true)
    {
      sleep_flag = false;
      wake_time = millis();
      return;
    }
    mode_flag++;
    switch (mode_flag)
    {
      case 1: mode = !mode; break;
      case 2: mode = 2; break;
      case 3: mode = 0; mode_flag = 0; break;
    }
    Serial.print(" MODE = ");
    Serial.println(mode);
  }
}






















#include "TimerOne.h"
#define ENCA 2  // YELLOW (Encoder A)
#define ENCB 3  // GREEN (Encoder B)
#define IN1 4   //  OUT2 black red
#define IN2 5   //  OUT1  RED
#define PWM 6   // ENA
// Orange = 5v (nguon encoder)
// black = GND (dat encoder)
#define Cambien1 60
#define limit2 56
#define limit3 57
//link3
#define en3 7
#define step3 8
#define dir3 9
//link 2
#define en2 12
#define step2 11
#define dir2 10
//link 4
#define namcham 58  //int4 black out 4
#define gnd 59      //int 3 red out 3
String inString;
// link 2
int currentPos2 = 0;  // Vị trí hiện tại của động cơ
int target2;

int moveAmount2;
//link3
int currentPos3 = 0;  // Vị trí hiện tại của động cơ
int target3;
int moveAmount3;
int values[4];
int gripp[41];
//
int targetPos3, targetPos2, targetPos4, targetPos5, targetPos6, targetPos7, targetPos8, targetPos9, targetPos10, targetPos11, targetPos12, targetPos13, targetPos14, targetPos15, targetPos16, targetPos17, targetPos18, targetPos19, targetPos20, targetPos21, targetPos22, targetPos23, targetPos24, targetPos25, targetPos26, targetPos27, targetPos28, targetPos29, targetPos30;
int H1, H2, H3, H4, H5, H6, H7, H8, H9, H10;
//
int targetPos1;
float u;
float pwr;
int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
float degree1 = 0;
float e;
int giatri;
long currT;
//
int dir;
int Start_mode, Stop_mode, DHN, DHT, detec;
//
int sobuoc3, sobuoc2;
//
int stepCount3, stepCount2;
int tacdong1, tacdong2, tacdong3;
int relayState = LOW;  // Trạng thái ban đầu của relay (tắt)

unsigned long previous2 = 0;
unsigned long previous3 = 0;
unsigned vitri = 0;
unsigned long timedelay1 = 100;
unsigned long timedelay2 = 200;
unsigned long thoigian1 = 0;
unsigned long thoigian2 = 0;
unsigned long thoigian3 = 0;
unsigned long time = 0;
float initialPos1 = 0, initialPos2 = 0, initialPos3 = 0;
unsigned long startTime1 = 0, startTime2 = 0, startTime3 = 0;
int run;
int D;  // de detect
#define placeY -90
#define placeR -80
#define placeB -75
void setup() {
  //link2
  Serial.begin(9600);      // Baudrate cao để giảm thời gian truyền
  pinMode(en2, OUTPUT);    // Enable
  pinMode(step2, OUTPUT);  // Step
  pinMode(dir2, OUTPUT);   // Direction
  digitalWrite(en2, LOW);  // Set Enable low
  //link3
  pinMode(en3, OUTPUT);    // Enable
  pinMode(step3, OUTPUT);  // Step
  pinMode(dir3, OUTPUT);   // Direction
  digitalWrite(en3, LOW);  // Set Enable low
  //link 1
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(Cambien1, INPUT_PULLUP);  //Cảm biến nhận tín hiệu
  pinMode(limit2, INPUT_PULLUP);    // Use internal pull-up resistor
  pinMode(limit3, INPUT_PULLUP);    // Use internal pull-up resistor
  pinMode(namcham, OUTPUT);         // Direction
  pinMode(gnd, OUTPUT);
  tacdong1, tacdong2, tacdong3 = 0;
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  Timer1.initialize(1000);  //  // 10ms ( 10 000)
  Timer1.attachInterrupt(cambiendetect);
}
void loop() {
  if (Serial.available() > 0) {  // Kiểm tra xem có dữ liệu nào được gửi đến

    receiValue();
  }
  if (DHT == 1) {
    Motor1PID(targetPos1);
    moveMotor3((targetPos3 + targetPos2));
    if (targetPos2 < 0 && targetPos3 < -80) {
      if (currentPos3 == (targetPos3 + targetPos2)) {
        moveMotor2(targetPos2);
      }
    } else {
      moveMotor2(targetPos2);
    }
  }
  // code xu ly anh
  if (detec == 1) {
    if (vitri == 0 && run == 1) {
      Motor1PID(targetPos1);
      moveMotor3((targetPos3 + targetPos2));
      if (targetPos2 < 0 && targetPos3 < -80) {
        if (currentPos3 == (targetPos3 + targetPos2)) {
          moveMotor2(targetPos2);
        }
      } else {
        moveMotor2(targetPos2);
      }
    } else if (vitri == 1 && run == 1 && D == 1) {  // điểm thả màu xanh
      Motor1PID(-30);
      moveMotor3(placeB);
      moveMotor2(0);
    } else if (vitri == 1 && run == 1 && D == 2) {  // điểm thả màu vàng
      Motor1PID(-35);
      moveMotor3(placeY);
      moveMotor2(5);
    } else if (vitri == 1 && run == 1 && D == 3) {  // điểm thả màu đỏ
      Motor1PID(-28);
      moveMotor3(placeR);
      moveMotor2(0);
    } else if (run == 0) {
      moveMotor2(10);
      moveMotor3(-45);
      if (currentPos2 == 10 && currentPos3 == -45) {
        thoigian3++;
        if (thoigian3 == 30) {
          run = 1;
          thoigian3 = 0;
        }
      }
    }

    if (fabs(e) < 1 & currentPos2 == targetPos2 & currentPos3 == (targetPos3 + targetPos2) && vitri == 0) {
      thoigian1++;
      if (thoigian1 == timedelay1) {
        digitalWrite(namcham, HIGH);
        digitalWrite(gnd, LOW);
        thoigian1 = 0;
        vitri = 1;
        run = 0;
      }
    }  // vi trí thả màu xanh
    else if (fabs(e) < 2 & currentPos2 == 0 & currentPos3 == placeB && vitri == 1 && D == 1) {
      if (thoigian2 == timedelay2) {
        vitri = 2;
        thoigian2 = 0;
        run = 0;
      }
      digitalWrite(namcham, LOW);
      digitalWrite(gnd, LOW);
    } else if (fabs(e) < 1 & currentPos2 == 5 & currentPos3 == placeY && vitri == 1 && D == 2) {  // vị trí thả màu vàng // -35 -90
      if (thoigian2 == timedelay2) {
        vitri = 2;
        thoigian2 = 0;
        run = 0;
      }
      digitalWrite(namcham, LOW);
      digitalWrite(gnd, LOW);
    } else if (fabs(e) < 1 & currentPos2 == 0 & currentPos3 == placeR && vitri == 1 && D == 3) {  // vị trí thả màu đỏ // -28 -75
      if (thoigian2 == timedelay2) {
        vitri = 2;
        thoigian2 = 0;
        run = 0;
      }
      digitalWrite(namcham, LOW);
      digitalWrite(gnd, LOW);
    }
  }
  // code quy hoach quy dao
  if (DHN == 1) {
    if (vitri == 0 && run == 1) {
      Motor1PID(targetPos1);
      moveMotor3((targetPos3 + targetPos2));
      if (targetPos2 < 0 && targetPos3 < -80) {
        if (currentPos3 == (targetPos3 + targetPos2)) {
          moveMotor2(targetPos2);
        }
      } else {
        moveMotor2(targetPos2);
      }
    } else if (vitri == 1 && run == 1) {
      Motor1PID(targetPos4);
      moveMotor3((targetPos6 + targetPos5));
      if (targetPos5 < 0 && targetPos6 < -80) {
        if (currentPos3 == (targetPos5 + targetPos6)) {
          moveMotor2(targetPos5);
        }
      } else {
        moveMotor2(targetPos5);
      }
    } else if (vitri == 2 && run == 1) {
      Motor1PID(targetPos7);
      moveMotor3((targetPos8 + targetPos9));
      if (targetPos8 < 0 && targetPos9 < -80) {
        if (currentPos3 == (targetPos8 + targetPos9)) {
          moveMotor2(targetPos8);
        }
      } else {
        moveMotor2(targetPos8);
      }
    } else if (vitri == 3 && run == 1) {
      Motor1PID(targetPos10);
      moveMotor3((targetPos11 + targetPos12));
      if (targetPos11 < 0 && targetPos12 < -80) {
        if (currentPos3 == (targetPos11 + targetPos12)) {
          moveMotor2(targetPos11);
        }
      } else {
        moveMotor2(targetPos11);
      }
    } else if (vitri == 4 && run == 1) {
      Motor1PID(targetPos13);
      moveMotor3((targetPos14 + targetPos15));
      if (targetPos14 < 0 && targetPos15 < -80) {
        if (currentPos3 == (targetPos14 + targetPos15)) {
          moveMotor2(targetPos14);
        }
      } else {
        moveMotor2(targetPos14);
      }
    } else if (vitri == 5 && run == 1) {
      Motor1PID(targetPos16);
      moveMotor3((targetPos17 + targetPos18));
      if (targetPos17 < 0 && targetPos18 < -80) {
        if (currentPos3 == (targetPos17 + targetPos18)) {
          moveMotor2(targetPos17);
        }
      } else {
        moveMotor2(targetPos17);
      }
    } else if (vitri == 6 && run == 1) {
      Motor1PID(targetPos19);
      moveMotor3((targetPos20 + targetPos21));
      if (targetPos20 < 0 && targetPos21 < -80) {
        if (currentPos3 == (targetPos20 + targetPos21)) {
          moveMotor2(targetPos20);
        }
      } else {
        moveMotor2(targetPos20);
      }
    } else if (vitri == 7 && run == 1) {
      Motor1PID(targetPos22);
      moveMotor3((targetPos23 + targetPos24));
      if (targetPos23 < 0 && targetPos24 < -80) {
        if (currentPos3 == (targetPos23 + targetPos24)) {
          moveMotor2(targetPos23);
        }
      } else {
        moveMotor2(targetPos23);
      }
    } else if (vitri == 8 && run == 1) {
      Motor1PID(targetPos25);
      moveMotor3((targetPos26 + targetPos27));
      if (targetPos26 < 0 && targetPos27 < -80) {
        if (currentPos3 == (targetPos26 + targetPos27)) {
          moveMotor2(targetPos26);
        }
      } else {
        moveMotor2(targetPos26);
      }
    } else if (vitri == 9 && run == 1) {
      Motor1PID(targetPos28);
      moveMotor3((targetPos30 + targetPos29));
      if (targetPos29 < 0 && targetPos30 < -80) {
        if (currentPos3 == (targetPos29 + targetPos30)) {
          moveMotor2(targetPos29);
        }
      } else {
        moveMotor2(targetPos29);
      }
    } else if (run == 0) {
      moveMotor2(10);
      moveMotor3(-45);
      if (currentPos2 == 10 && currentPos3 == -45) {
        thoigian3++;
        if (thoigian3 == 30) {
          run = 1;
          thoigian3 = 0;
        }
      }
    }
    if (fabs(e) < 1 & currentPos2 == targetPos2 & currentPos3 == (targetPos3 + targetPos2) && vitri == 0) {
      if (H1 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 1;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 1;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos5 & currentPos3 == targetPos6 + targetPos5 && vitri == 1) {
      if (H2 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 2;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 2;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos8 & currentPos3 == (targetPos8 + targetPos9) && vitri == 2) {
      if (H3 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 3;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 3;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos11 & currentPos3 == targetPos11 + targetPos12 && vitri == 3) {
      if (H4 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 4;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 4;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos14 & currentPos3 == (targetPos14 + targetPos15) && vitri == 4) {
      if (H5 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 5;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 5;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos17 & currentPos3 == (targetPos17 + targetPos18) && vitri == 5) {
      if (H6 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 6;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 6;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos20 & currentPos3 == (targetPos20 + targetPos21) && vitri == 6) {
      if (H7 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 7;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 7;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos23 & currentPos3 == (targetPos23 + targetPos24) && vitri == 7) {
      if (H8 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 8;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 8;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (fabs(e) < 1 & currentPos2 == targetPos26 & currentPos3 == (targetPos26 + targetPos27) && vitri == 8) {
      if (H9 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 9;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 9;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    } else if (abs(e) < 1 & currentPos2 == targetPos29 & currentPos3 == (targetPos29 + targetPos30) && vitri == 9) {
      if (H10 == 1) {
        thoigian1++;
        if (thoigian1 == timedelay1) {
          digitalWrite(namcham, HIGH);
          digitalWrite(gnd, LOW);
          thoigian1 = 0;
          vitri = 10;
          run = 0;
        }
      } else {
        thoigian2++;
        if (thoigian2 == timedelay2) {
          vitri = 10;
          thoigian2 = 0;
          run = 0;
        }
        digitalWrite(namcham, LOW);
        digitalWrite(gnd, LOW);
      }
    }
  }
  if (Start_mode == 1) {
    Motor1PID(targetPos1);
  }
  Timer1.stop();
  Serial.print("pos:  ");
  Serial.println(pos);
}
void receiValue() {
  String inChar = Serial.readString();
  if (inChar[0] == 'S' && (Start_mode == 0)) {
    DHT = 0;
    DHN = 0;
    detec = 0;
    Start_mode++;
    Stop_mode = 0;
    tacdong2 = 0;
    tacdong1 = 0;
    tacdong3 = 0;
    Serial.println("Robot mode: ON");
    home();
  } else if (inChar[0] == 'F') {
    DHT = 1;
    DHN = 0;
    detec = 0;
    if (Start_mode == 1) {
      pos = e * 5544.00 / 360;  // sau khi set home thì set lại vị trí mới cho động cơ PID nếu phát hiện cảm biến
      Start_mode = 0;
    }
    inString = "";  //clear chuoi nay di
    Serial.println("FORWARD Kinematic");
    extractAndStoreValues(inChar, values);
    targetPos1 = values[0];
    targetPos2 = values[1];
    targetPos3 = values[2];
    D = 0;
    // Serial.print("theta1: ");
    // Serial.println(values[0]);
    // Serial.print("theta2: ");
    // Serial.println(values[1]);
    // Serial.print("theta3: ");
    // Serial.println(values[2]);
  } else if (inChar[0] == 'I') {
    DHT = 0;
    DHN = 1;
    detec = 0;
    vitri = 0;
    thoigian1 = 0;
    thoigian2 = 0;
    if (Start_mode == 1) {
      pos = e * 5544.00 / 360;  // sau khi set home thì set lại vị trí mới cho động cơ PID nếu phát hiện cảm biến
      Start_mode = 0;
    }
    inString = "";  //clear chuoi nay di
    Serial.println("Iverse Kinematic");
    extractAndStoreValues40values(inChar, gripp);



    targetPos1 = gripp[1];  //vitri gap
    targetPos2 = gripp[2];
    targetPos3 = gripp[3];
    H1 = gripp[4];
    targetPos4 = gripp[5];  // vitri tha
    targetPos5 = gripp[6];
    targetPos6 = gripp[7];
    H2 = gripp[8];
    targetPos7 = gripp[9];  // vitrigap 2
    targetPos8 = gripp[10];
    targetPos9 = gripp[11];
    H3 = gripp[12];
    targetPos10 = gripp[13];  //vitri tha
    targetPos11 = gripp[14];
    targetPos12 = gripp[15];
    H4 = gripp[16];
    targetPos13 = gripp[17];  // vitri gap 3
    targetPos14 = gripp[18];
    targetPos15 = gripp[19];
    H5 = gripp[20];
    targetPos16 = gripp[21];  // vitri tha
    targetPos17 = gripp[22];
    targetPos18 = gripp[23];
    H6 = gripp[24];
    targetPos19 = gripp[25];
    targetPos20 = gripp[26];
    targetPos21 = gripp[27];
    H7 = gripp[28];
    targetPos22 = gripp[29];
    targetPos23 = gripp[30];
    targetPos24 = gripp[31];
    H8 = gripp[32];
    targetPos25 = gripp[33];
    targetPos26 = gripp[34];
    targetPos27 = gripp[35];
    H9 = gripp[36];
    targetPos28 = gripp[37];
    targetPos29 = gripp[38];
    targetPos30 = gripp[39];
    H10 = gripp[40];
    // Serial.print("theta1: ");
    // Serial.println(values[0]);
    // Serial.print("theta2: ");

    // Serial.println(values[1]);
    // Serial.print("theta3: ");
    // Serial.println(values[2]);receiValue();
  } else if ((inChar[0] == 'T') && (Stop_mode == 0)) {
    Serial.println("Robot mode: OFF");
    Start_mode = 0;
    Stop_mode++;
    sobuoc2 = 0;
    sobuoc3 = 0;
    pwr = 0;
    DHT = 0;
    DHN = 0;
    detec = 0;
    tacdong2 = 0;
    tacdong1 = 0;
    tacdong3 = 0;
    setMotor(0, 0, PWM, IN1, IN2);
    digitalWrite(dir3, LOW);
    digitalWrite(dir2, LOW);
  } else if ((inChar[0] == 'R')) {
    Serial.println("Robot mode: OFF");
    Start_mode = 0;
    Stop_mode++;
    sobuoc2 = 0;
    sobuoc3 = 0;
    pwr = 0;
    DHT = 0;
    DHN = 0;
    detec = 0;
    tacdong2 = 0;
    tacdong1 = 0;
    tacdong3 = 0;
    pos = 0;
    currentPos2 = 0;
    currentPos3 = 0;
    moveAmount3 = 0;
    moveAmount2 = 0;
    stepCount2 = 0;
    stepCount3 = 0;
    setMotor(0, 0, PWM, IN1, IN2);
    digitalWrite(dir3, LOW);
    digitalWrite(dir2, LOW);
    digitalWrite(en2, HIGH);  // Set Enable HIGH
    digitalWrite(en3, HIGH);  // Set Enable hIGH
  } else if (inChar[0] == 'H') {
    digitalWrite(namcham, HIGH);
    digitalWrite(gnd, LOW);
  } else if (inChar[0] == 'L') {
    digitalWrite(namcham, LOW);
    digitalWrite(gnd, LOW);
  } else if (inChar[0] == 'D') {
    detec = 1;
    DHT = 0;
    DHN = 0;
    vitri = 0;
    run = 0;
    if (Start_mode == 1) {
      pos = e * 5544.00 / 360;  // sau khi set home thì set lại vị trí mới cho động cơ PID nếu phát hiện cảm biến
      Start_mode = 0;
    }
    inString = "";  //clear chuoi nay di
    Serial.println("Detect");
    extractAndStoreValues(inChar, values);
    targetPos1 = values[0];
    targetPos2 = values[1];
    targetPos3 = values[2];
    D = values[3];  // câu lệnh phân biệt xanh đỏ vàng
  }
}
void moveMotor2(int target2) {  // 60  - 0
  moveAmount2 = target2 - currentPos2;
  sobuoc2 = abs(moveAmount2 * 3 / 1.8 * 4.0);  //60*3/1.8 * 4.0
  digitalWrite(dir2, moveAmount2 > 0 ? LOW : HIGH);

  if (stepCount2 < sobuoc2) {
    if (micros() - previous2 >= 3000.0) {  //10 0000
      previous2 = micros();
      digitalWrite(step2, HIGH);
      // Tạo xung Step
      digitalWrite(step2, LOW);
    }
    stepCount2++;
  }
  if (stepCount2 >= sobuoc2) {
    currentPos2 = target2;
    stepCount2 = 0;
  }
}
// link3
void moveMotor3(int target3) {
  moveAmount3 = target3 - currentPos3;
  sobuoc3 = abs(moveAmount3 * 3 / 1.8 * 4.0);
  digitalWrite(dir3, moveAmount3 > 0 ? LOW : HIGH);

  // Kiểm tra thời gian giữa các bước
  if (stepCount3 < sobuoc3) {
    if (micros() - previous3 >= 1500.0) {
      previous3 = micros();

      digitalWrite(step3, HIGH);
      digitalWrite(step3, LOW);
      stepCount3++;
    }
  }

  if (stepCount3 >= sobuoc3) {
    currentPos3 = target3;
    stepCount3 = 0;
  }
}
void Motor1PID(int targetPos1) {
  // set target position
  currT = micros();
  // PID constants
  float kp = 80.0;   // 20
  float kd = 1.955;  //2
  float ki = 0.0;
  // time difference
  float deltaT = ((float)(currT - prevT) / (1.0e6));
  prevT = currT;
  degree1 = (pos / 5544.00) * 360.00;  //5544.00
  // error
  e = degree1 - targetPos1;
  // derivative
  float dedt = (e - eprev) / (deltaT);
  // integral
  eintegral = eintegral + e * deltaT;
  // control signal
  u = kp * e + kd * dedt + ki * eintegral;
  // motor power
  pwr = fabs(u);
  if (pwr > 100) {
    pwr = 100;
  }
  // huong quay motor
  dir = -1;
  if (u < 0) {
    dir = 1;
  }
  setMotor(dir, pwr, PWM, IN1, IN2);
  // luu sai so
  eprev = e;
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // bam xung
  if (dir == 1) {
    digitalWrite(in1, HIGH);  // dong co quay thuan
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);  // dong co quay nghich
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  // tat dong co
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos--;
  } else {
    pos++;
  }
}
void home() {
  Timer1.start();
  digitalWrite(en2, LOW);  // Set Enable low
  digitalWrite(en3, LOW);  // Set Enable low
  if (digitalRead(limit2) == LOW) {
    if (digitalRead(limit2) == LOW) {
      tacdong2 = 1;
      moveAmount2 = 29;
    }
  }
  if (tacdong2 == 0) {
    targetPos2 = -80;
    moveAmount2 = targetPos2 - currentPos2;
    sobuoc2 = abs(moveAmount2 * 3 / 1.8 * 4.0);
    digitalWrite(dir2, moveAmount2 > 0 ? LOW : HIGH);
    for (int i = 0; i < sobuoc2; i++) {
      digitalWrite(step2, HIGH);
      delay(2.5);
      digitalWrite(step2, LOW);
      delay(2.5);
      if (digitalRead(limit2) == LOW) {
        if (digitalRead(limit2) == LOW) {
          tacdong2 = 1;
          moveAmount2 = 26;
          break;
        }
      }
    }
    currentPos2 = targetPos2;
  }
  sobuoc2 = abs(moveAmount2 * 3 / 1.8 * 4.0);
  digitalWrite(dir2, moveAmount2 > 0 ? LOW : HIGH);
  for (int i = 0; i < sobuoc2; i++) {
    digitalWrite(step2, HIGH);
    delay(2.5);
    digitalWrite(step2, LOW);
    delay(2.5);
  }
  currentPos2 = 0;
  if (tacdong3 == 0) {
    targetPos3 = 230;
    moveAmount3 = targetPos3 - currentPos3;
    sobuoc3 = abs(moveAmount3 * 3 / 1.8 * 4.0);
    digitalWrite(dir3, moveAmount3 > 0 ? LOW : HIGH);
    for (int i = 0; i < sobuoc3; i++) {
      digitalWrite(step3, HIGH);
      delayMicroseconds(1000);
      digitalWrite(step3, LOW);
      delayMicroseconds(1000);
      if (digitalRead(limit3) == LOW) {
        tacdong3 = 1;
        break;
      }
    }
    if (tacdong3 != 1) {
      currentPos3 = targetPos3;
    }
  }

  while (tacdong1 != 1) {
    setMotor(-1, 100, PWM, IN1, IN2);
    if (pos <= -924 && tacdong1 == 0) break;
  }

  while (tacdong1 != 1) {
    setMotor(1, 100, PWM, IN1, IN2);
    Serial.print(" pos ");
    Serial.println(pos);
    if (pos >= 924 && tacdong1 == 0) break;
  }
  setMotor(0, 0, PWM, IN1, IN2);
  // nếu không phát hiện cảm biến thì cho nó ngừng đồng thời cho nó quay ngược lại về 0 tránh làm tắc dây
  moveAmount3 = 0 - 83;
  sobuoc3 = abs(moveAmount3 * 3 / 1.8 * 4.0);

  digitalWrite(dir3, moveAmount3 > 0 ? LOW : HIGH);
  for (int i = 0; i < sobuoc3; i++) {
    digitalWrite(step3, HIGH);
    delayMicroseconds(1000);
    digitalWrite(step3, LOW);
    delayMicroseconds(1000);
  }
  currentPos3 = 0;
}
void extractAndStoreValues40values(String data, int gripp[]) {
  // Tách chuỗi theo dấu phẩy
  int valueIndex = 0;

  // Loại bỏ các ký tự không cần thiết, nếu có
  data.replace("theta1:", "");
  data.replace("theta2:", "");
  data.replace("theta3:", "");
  data.replace("H:", "");

  // Tách chuỗi thành các phần tử con dựa trên dấu phẩy
  int startIndex = 0;
  while (startIndex < data.length() && valueIndex < 41) {
    int endIndex = data.indexOf(',', startIndex);
    if (endIndex == -1) {
      endIndex = data.length();
    }
    gripp[valueIndex] = data.substring(startIndex, endIndex).toInt();
    valueIndex++;
    startIndex = endIndex + 1;
  }
  for (int i = 0; i < valueIndex; i++) {
    Serial.print("gripp[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(gripp[i]);
  }
}
void extractAndStoreValues(String data, int values[]) {
  // Các prefix cần tìm
  String prefixes[] = { "theta1:", "theta2:", "theta3:", "D:" };
  int numPrefixes = sizeof(prefixes) / sizeof(prefixes[0]);

  int valueIndex = 0;  // Chỉ số của mảng values
  for (int i = 0; i < numPrefixes; i++) {
    int startIndex = data.indexOf(prefixes[i]);
    if (startIndex != -1) {
      startIndex += prefixes[i].length();
      int endIndex = data.indexOf(',', startIndex);
      if (endIndex == -1) endIndex = data.length();
      values[valueIndex] = data.substring(startIndex, endIndex).toInt();
      valueIndex++;
    }
  }
}
void cambiendetect() {
  if (Start_mode == 1 && tacdong1 == 0) {
    if (digitalRead(Cambien1) == 1) {
      tacdong1 = 1;
      Serial.print("CAMBIEN TAC DONG  ");
      Serial.println(tacdong1);
      targetPos1 = (pos / 5544.00) * 360.00;  //5544
      Serial.print("targetPos1  ");
      Serial.println(targetPos1);
    }
  }
}

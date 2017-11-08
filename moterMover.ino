#include <MsTimer2.h>

int movementMode; // 1は直線軌道、2は円軌道

// Setting for MsTimer2
//タイマー用の値
unsigned long prevMilli = 0;  //前回呼び出し時間

unsigned long arrayCount = 0;  //制御配列参照用
unsigned long secCount = 0; //msカウント用

volatile int period;  //呼び出し時間間隔

//ポテンショメータのピンの設定
//左→ #1 GRD
//中→ #2 Output
//右→ #3 DC 5V
const int P[] = {1, 2};

//};

// Setting for Motor Control
//モータをコントロールするピンの設定
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;
const float ana2rad = 2*3.141592/360.0 * 330.0 / 1023.0;  //角度定義
volatile float curAngle[2];
volatile float Kp=1000, Kd=0.3;

volatile float moter1l[100];
volatile float moter2l[100];
volatile float moter1c[100];
volatile float moter2c[100];

volatile float tgtAngle1[100];
volatile float tgtAngle2[100];

volatile int tgtArray; //入力した制御値の数
volatile int tgtCycle;  //往復分ループ


//volatile int tgtLength;  // length of tgtAngle array
//volatile int tgtCycle;  // cycle number for target position modification
//volatile int repeatTime;  // repeattime for tgtCycle

//setup関数内で定義されている関数
//1フレームに1回呼び出されている
//やっていること↓
// 1, 毎フレームのカウントし、カウント数から目標角度を設定
// 2, モーターの角度と速度を読み込み 
// 3, 誤差角度と角速度からトルクを算出
// 4, トルク値を正規化して代入
// 5, カウントアップ

// 目標角度は、setupの関数の中で書き込んでいます
// 任意の値を書き込めば制御可能

// csvで読み込むとしたらのコードをメモ

// BufferedReader reader;
// reader = createReader(“test.csv”);　


void debug(){


    Serial.print("analogRead(P1) = ");
    Serial.print(analogRead(P[0]));
    Serial.print("     ");

    Serial.print("analogRead(P2) = ");
    Serial.print(analogRead(P[1]));
    Serial.println("     ");
    
    
  
}

void periodicFunction() {
    
    unsigned long curMilli; //現在の時間カウント
    
    //msカウントが次の制御を行うタイミングを超えたら
    if(secCount >= tgtArray){
        
        
        //モータ制御用の値
        float motorAngle[] = {0,0};   //モーターの現在位置と
        float targetAngle[] = {0,0};  //モーターの目標位置
        float angleError[] = {0,0}; //角度誤差
        float angleVelo[] = {0,0};  //角速度
        float oldAngle[] = {0,0};  //前角度
        float torque[] = {0,0};  //トルク
        int outMotor[] = {0,0};  //モーターの出力値
        
        // for checking of cycle period
        curMilli = micros(); //実行開始時から現在までの時間をカウント
        period = curMilli - prevMilli;  //前回の呼び出しからの経過時間をカウント
        prevMilli = curMilli;  //現在の時間を保存
        
        // Set target angle and read motor angle and velocity
        // 目標角度を設定し、モーターの角度と速度を読み込み
        
        //モードに応じて値をセット
        if(movementMode == 1){
            tgtCycle = 28;
            
            targetAngle[0] = tgtAngle1[arrayCount];
            targetAngle[1] = tgtAngle2[arrayCount];
            
            
        }else if(movementMode == 2){
            tgtCycle = 37;
            
            targetAngle[0] = tgtAngle1[arrayCount];
            targetAngle[1] = tgtAngle2[arrayCount];
            
        }
        
        //配列用カウンタの更新
        arrayCount++;
        if(arrayCount == tgtArray){
            arrayCount = 0;
        }
        
        
        //targetAngle[i] = tgtAngle[count2];  //目標角度
        
        
        
        //値をセットします
        
        for(int moterNo =0; moterNo < 2; moterNo++){
            
            motorAngle[moterNo] = ana2rad * analogRead(P[0]);  //1ピンから角度を読み込みradian変換
            curAngle[moterNo] = motorAngle[moterNo];  //モーターの角度を現在の角度に保存
            angleError[moterNo] = targetAngle[moterNo] - motorAngle[moterNo];  //角度誤差を算出
            angleVelo[moterNo] = motorAngle[moterNo] - oldAngle[moterNo] * 1000.0;  //角速度を算出
            oldAngle[moterNo] = motorAngle[moterNo];  //角度を保存
            
            // Controller calculation
            // 制御計算
            torque[moterNo] = Kp * angleError[moterNo] - Kd * angleVelo[moterNo];  //誤差角度と角速度からトルクを算出
            
            // Output to motor driver
            // モータードライバーに書き出し
            outMotor[moterNo] = (int)torque[moterNo];  //トルクを代入
        }
        
        //----------------------------
        //モーター1


        //ピンへの書き込み

        Serial.print("outMotor[0] = " );
        Serial.print(outMotor[0] );
        Serial.print("    ");        

        Serial.print("outMotor[1] = " );
        Serial.println(outMotor[1] );
        Serial.println("    ");

        writePin(outMotor[0], 1);
        writePin(outMotor[1], 2);

        Serial.println("    ");

        
        
        /*
        // 値を確認しつつ信号を送信(-255 < outMotor <= 255)
        if(outMotor[0] > 255) {
            digitalWrite(M1, LOW);  //方向制御
            analogWrite(E1, 255);  //指定したピンからアナログ値(PWM波)を出力
        } else if(outMotor[0] > 0) {
            digitalWrite(M1, LOW);
            analogWrite(E1, outMotor[0]);  //PWM
        } else if(outMotor[0] > -255) {
            digitalWrite(M1, HIGH);
            analogWrite(E1, -outMotor[0]);  //PWM
        } else {
            digitalWrite(M1, HIGH);
            analogWrite(E1, 255);  //PWM
        } 
        */
        //----------------------------
        //モーター2


        
        /*
        if(outMotor[1] > 255) {
            digitalWrite(M2, LOW);  //方向制御
            analogWrite(E2, 255);  //指定したピンからアナログ値(PWM波)を出力
        } else if(outMotor[1] > 0) {
            digitalWrite(M2, LOW);
            analogWrite(E2, outMotor[1]);  //PWM
        } else if(outMotor[1] > -255) {
            digitalWrite(M2, HIGH);
            analogWrite(E2, -outMotor[1]);  //PWM
        } else {
            digitalWrite(M2, HIGH);
            analogWrite(E2, 255);  //PWM
        } 
        */
        
        //-------------------------
        // loop cycle 
        // ループを回す
        
    }


    secCount++;
    
    if(secCount == tgtCycle){
        secCount = 0;
        
    }   

    debug();

}    



void writePin(int _outMotor, int _pin){

    int E, M;

    if(_pin == 1){
        E = E1;
        M = M1;
    }else if(_pin == 2){
        E = E2;
        M = M2;
    }else {
      return;      
    }
        
   if(_outMotor > 255) {
            digitalWrite(M, LOW);  //方向制御
            analogWrite(E, 255);  //指定したピンからアナログ値(PWM波)を出力
            Serial.println("Write:  d = LOW   a = 255" );
            
        } else if(_outMotor > 0) {
            digitalWrite(M, LOW);
            analogWrite(E, _outMotor);  //PWM
            Serial.print("Write:  d = LOW   a = " );
            Serial.println(_outMotor);
            
        } else if(_outMotor > -255) {
            digitalWrite(M, HIGH);
            analogWrite(E, -1 * _outMotor);  //PWM
            Serial.print("Write:  d = HIGH   a = " );
            Serial.println(-1 * _outMotor);
            
        } else {
            digitalWrite(M2, HIGH);
            analogWrite(E2, 255);  //PWM
            Serial.println("Write:  d = HIGH   a = 255" );
        }
    } 
  

void setValue(){
    
    //28 *2 要素
    volatile float moter1l[] = {
        
        0,0
        ,0.04,-6.340191747
        ,0.08,-23.19859052
        ,0.12,-44.99999999
        ,0.16,-66.80140947
        ,0.2,-83.65980824
        ,0.24,-89.99999998
        ,0.241,-89.99999998
        ,0.28,-83.65980824
        ,0.32,-66.80140947
        ,0.36,-44.99999999
        ,0.4,-23.19859052
        ,0.44,-6.340191747
        ,0.48,0
        ,0.481,0
        ,0.52,-6.340191747
        ,0.56,-23.19859052
        ,0.6,-44.99999999
        ,0.64,-66.80140947
        ,0.68,-83.65980824
        ,0.72,-89.99999998
        ,0.721,-89.99999998
        ,0.76,-83.65980824
        ,0.8,-66.80140947
        ,0.84,-44.99999999
        ,0.88,-23.19859052
        ,0.92,-6.340191747
        ,0.96,0
        ,
        
    };
    
    
    //28 *2 要素
    volatile float moter2l[] = 
    
    {
        0,0
        ,0.04,0.819376032
        ,0.08,-3.547437309
        ,0.12,-19.79031689
        ,0.16,-47.15025626
        ,0.2,-76.50024046
        ,0.24,-89.99999998
        ,0.241,-89.99999998
        ,0.28,-76.50024046
        ,0.32,-47.15025626
        ,0.36,-19.79031689
        ,0.4,-3.547437309
        ,0.44,0.819376032
        ,0.48,0
        ,0.481,0
        ,0.52,0.819376032
        ,0.56,-3.547437309
        ,0.6,-19.79031689
        ,0.64,-47.15025626
        ,0.68,-76.50024046
        ,0.72,-89.99999998
        ,0.721,-89.99999998
        ,0.76,-76.50024046
        ,0.8,-47.15025626
        ,0.84,-19.79031689
        ,0.88,-3.547437309
        ,0.92,0.819376032
        ,0.96,0
        
    };
    
    //37 *2 要素
    volatile float moder1c[] = 
    {
        0,0
        ,0.03,4.03524143
        ,0.06,9.130335137
        ,0.09,14.84646976
        ,0.12,20.88973532
        ,0.15,27.03524259
        ,0.18,33.07850816
        ,0.21,38.79464278
        ,0.24,43.88973649
        ,0.27,47.92497792
        ,0.3,50.18303828
        ,0.33,49.44632369
        ,0.36,43.82617745
        ,0.39,31.77753094
        ,0.42,16.14744706
        ,0.45,4.098800519
        ,0.48,-1.521345748
        ,0.51,-2.258060358
        ,0.54,-1.11E-08
        ,0.57,4.035241414
        ,0.6,9.130335119
        ,0.63,14.84646974
        ,0.66,20.8897353
        ,0.69,27.03524257
        ,0.72,33.07850814
        ,0.75,38.79464276
        ,0.78,43.88973647
        ,0.81,47.92497791
        ,0.84,50.18303827
        ,0.87,49.4463237
        ,0.9,43.82617748
        ,0.93,31.77753099
        ,0.96,16.14744711
        ,0.99,4.098800549
        ,1.02,-1.521345739
        ,1.05,-2.258060361
        ,1.08,-2.23E-08  
    };
    
    //37 *2 要素
    volatile float moter2c[] = {
        
        0,0
        ,0.03,-8.40741584
        ,0.06,-15.38719467
        ,0.09,-21.51566158
        ,0.12,-27.19154121
        ,0.15,-21.04603397
        ,0.18,-3.283623201
        ,0.21,14.27711295
        ,0.24,31.4470792
        ,0.27,47.9249779
        ,0.3,63.16415831
        ,0.33,76.05390029
        ,0.36,84.14714724
        ,0.39,82.49125339
        ,0.42,66.86116954
        ,0.45,44.41977037
        ,0.48,25.08623092
        ,0.51,10.72305974
        ,0.54,3.17E-08
        ,0.57,-8.407415815
        ,0.6,-15.38719464
        ,0.63,-21.51566156
        ,0.66,-27.1915412
        ,0.69,-21.04603403
        ,0.72,-3.283623261
        ,0.75,14.2771129
        ,0.78,31.44707914
        ,0.81,47.92497784
        ,0.84,63.16415826
        ,0.87,76.05390025
        ,0.9,84.14714722
        ,0.93,82.49125342
        ,0.96,66.86116961
        ,0.99,44.41977045
        ,1.02,25.08623098
        ,1.05,10.72305978
        ,1.08,6.34E-08
        ,
        
    };

    //直線軌道
    if(movementMode == 1){
    
        for(int i=0; i < tgtArray; i++){

            tgtAngle1[i] = 3.141592/2 + 3.141592 * (1-cos(3.141592*moter1l[i*2]/100))/2;
            tgtAngle2[i] = 3.141592/2 + 3.141592 * (1-cos(3.141592*moter2l[i*2]/100))/2;
//            
//           tgtAngle1[i] = moter1l[i];
//           tgtAngle2[i] = moter2l[i];
        
        }
    }
    
    //円軌道
    else if(movementMode == 2){    
        for(int i=0; i < tgtArray; i++){

            //tgtAngle1[i] = moter1c[i*2] * ;
            tgtAngle1[i] = 3.141592/2 + 3.141592 * (1-cos(3.141592*moter1c[i*2]/100))/2;
            
//            tgtAngle2[i] = moter2c[i*2];

            tgtAngle2[i] = 3.141592/2 + 3.141592 * (1-cos(3.141592*moter2c[i*2]/100))/2;
        }      
    }
}

void setSampleValue(){
  
    for(int i=0; i < 3*2; i++){
        
        tgtAngle1[i] = 100 * (i + 1);
        tgtAngle2[i] = 100 * (i + 1);
        
    }

  
}

void donon(){
    
    
    
}

void setup() {
    // Set up Serial Com.
    Serial.begin(9600);  //シリアル通信のデータ転送レートをbps(baud)で指定
    
    //制御モードをセット
    movementMode = 1; //1は直線、2は円
    
    // Set up motion path
    // モーションパスを指定
    
    //    int i;
    //    for(i=0; i<100; i++) {
    //    tgtAngle[i] = 3.141592/2 + 3.141592 * (1-cos(3.141592*i/100))/2;
    //    tgtAngle[199-i] = tgtAngle[i];
    //    }
    
    //setValue();
    setSampleValue();
    
    //直線モード
    if( movementMode == 1){
        //tgtCycle = 28;
        tgtCycle = 0.96 * 1000;
    } 
    
    //円モード
    else if( movementMode == 2){
        //tgtCycle = 37;
        tgtCycle = 1.08 * 1000;
    }
    
    //  tgtCycle = 5;  //目標角度
    //  tgtLength = 200;  //目標長さ
    //  repeatTime = 10;  //繰り返し回数
    
    // Motor Controller
    // モーター制御
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    
    //pinMode(M1, OUTPUT);
    
    // Set up MsTimer2
    // MsTimerのセットアップ
    MsTimer2::set(1, periodicFunction);
    //MsTimer2::set(1, periodicFunction);
    MsTimer2::start();
}

void loop() {
    // Serial Com.
    //  Serial.print(float2String(2,(curAngle*180/3.141592)));
    /*
    Serial.print("analogRead(P1) = ");
    Serial.print(analogRead(P[0]));
    Serial.println(",    ");

    Serial.print("analogRead(P2) = ");
    Serial.print(analogRead(P[1]));
    Serial.println("     ");
    */
    /*
    Serial.print("curAngle1 = ");
    Serial.print(curAngle[0]*180/3.141592);  //現在の角度を書き出し
    Serial.print("     ");
    Serial.print("tergetAngle1 = ");
    Serial.print(3.141592/2 + 3.141592 * (1-cos(3.141592*moter1l[1*2]/100))/2);
    Serial.print("     ");
    
    /*
    Serial.print("curAngle2 = ");
    Serial.print(curAngle[1]*180/3.141592);
    Serial.print("     ");
    */
    Serial.println(period);
}


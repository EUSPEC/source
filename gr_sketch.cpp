/*======================================*/
/* インクルード                         */
/*======================================*/
#include <rxduino.h>
#include <eeprom.h>
#include <sdmmc.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* LCD出力設定 */
/* LiquidCrystal(rs, rw, enable, d4, d5, d6, d7)
rs: LCDのRSピンに接続するArduino側のピン番号
rw: LCDのRWピンに接続するArduino側のピン番号
enable: LCDのenableピンに接続するArduino側のピン番号
d0～d7: LCDのdataピンに接続するArduino側のピン番号 */
LiquidCrystal lcd(27, 28, 29, 23, 24, 25, 26);

/* ESCパルス出力用変数 */
Servo motor1;
Servo motor2;
Servo brake;

SDMMC MMC;
EEPROM EEPROM;

char msd[128];

#define INTERVAL 100
#define PIN 35
#define NUMPIXELS 8

#define enc_cal 2.3

#define Angle_count 50
#define Angle_brake 0

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


/*======================================*/
/* パラメータ                           */
/*======================================*/
int UPDISTANCE = 15;                    // 上昇距離
int DWDISTANCE = 15;                    // 下降距離
int WAIT = 10;                        // 停止時間
int Start_WAIT = 5;


/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
const char *C_DATE = __DATE__;
const char *C_TIME = __TIME__;

long lEncoderTotal = 0;                 // 走行距離
long uEncoderBuff;                      // 距離保存バッファ
long lEncoderBuff = 0;                  // 距離保存バッファ
long DisatanceTotal = 0;
long cnt_int = 0;
long cnt1 = 0;                          // 時間計測カウンタ
long cnt_lcd = 0;                       // LCD用カウンタ
long cnt_buzz = 0;
long cnt100 = 0;

int iEncoder;                           // 走行速度
int val1;                               // モータ1速度制御用一時変数
int val2;                               // モータ2速度制御用一時変数

int iTimer10; 
int iTimer100; 

int pattern = 0;                        // メインプログラム パターン
int lcd_pattern = 1;                    // パラメータ設定 パターン
const int led1 = 41;                           // led1 ポート設定
const int led2 = 42;                           // led2 ポート設定
const int led3 = 43;                           // led1 ポート設定
const int ledm = 22;
const int sw1 = 55;                            // sw1 ポート設定
const int sw2 = 53;                           // sw2 ポート設定
const int sw3 = 54;                           // sw3 ポート設定
const int sw4 = 21;                           // sw4 ポート設定
const int sw5 = 52;                           // sw5 ポート設定
int sw1_val = 1;                        // sw1 取得値保存用
int sw2_val = 1;                        // sw2 取得値保存用
int sw3_val = 1;                        // sw3 取得値保存用
int sw4_val = 1;                        // sw4 取得値保存用
int sw5_val = 1;                        // sw5 取得値保存用
int PIN_SW_val = 1;
const int dsw = 20;                           // dsw ポート設定
int dsw_val = 0;

const int m1_ain = 14;
const int m2_ain = 15;
int m1_ain_val = 0;
int m2_ain_val = 0;

const int Trig = 32;
const int Echo = 33;
int Duration;
float Distance;

const int re1 = 38;                           // 
const int re2 = 37;                           // 
const int re3 = 36;                           // 


int m1_val;
int m2_val;
int brake_val;

const int buzz = 34;                           // buzzer ポート設定


static boolean output = HIGH;           // LED 出力初期値
static boolean led1_output = HIGH;      // LED 出力初期値
static boolean buzz_output = HIGH;

const int chipSelect = 10;              // SDcard チップセレクト端子

const int rcv1 = 16; //1ch目、AUX              // AUX入力ポート
int RV1 = 0; // 1ch目読み取り用変数      // AUX取得値格納変数
int RV1_f = 0;
int AUX1 = 0;
const int rcv2 = 17; //1ch目、AUX              // AUX入力ポート
int RV2 = 0; // 1ch目読み取り用変数      // AUX取得値格納変数
int RV2_f = 0;
int AUX2 = 0;
const int rcv3 = 18; //1ch目、AUX              // AUX入力ポート
int RV3 = 0; // 1ch目読み取り用変数      // AUX取得値格納変数
int RV3_f = 0;
int AUX3 = 0;




/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void motor(int accele1, int accele2);
void brake_s(int accele3);
void led_flash(int interval);
void setup_parameter(void);
void write_eeprom(void);
void read_eeprom(void);
void rotRotEnc(void);
void int_1msec(void);
void onboard_flash(void);
void serialmonitor(void);
void buzzer(int interval, int count);
void distance(void);
void AUX(void);
void microSDProcess(void);
void motor_ain(void);
void sw_input(void);


/************************************************************************/
/* SETUP                                                             　 */
/************************************************************************/
void setup()
{


	Serial.begin(9600);                          // ボーレートを9600に設定
	pixels.begin(); 
	strip.show();

	/* ESCパルス出力設定 */
	/* servo.attach( pin, min, max )
	servo: Servo型の変数
	pin: サーボを割り当てるピンの番号
	min: サーボの角度が0度のときのパルス幅(マイクロ秒)。デフォルトは544
	max: サーボの角度が180度のときのパルス幅(マイクロ秒)。デフォルトは2400 */
	
	motor1.attach(1, 900, 2100);                 // motor1を9番ピン、パルス幅を1050～1900に設定
	motor2.attach(2, 900, 2100);                // motor2を10番ピン、パルス幅を1050～1900に設定
	brake.attach(0, 990, 1990);                 // brakeを11番ピン、パルス幅を1050～1850に設定

	/* ポート入出力設定 */
	/* pinMode(pin, mode)
	pin: 設定したいピンの番号
	mode: INPUT、OUTPUT、INPUT_PULLUP */
	
	pinMode(30, INPUT);
	pinMode(13, OUTPUT);
	pinMode(sw1, INPUT);
	pinMode(sw2, INPUT);
	pinMode(sw3, INPUT);
	pinMode(sw4, INPUT);
	pinMode(sw5, INPUT);
	pinMode(dsw, INPUT);
	pinMode(rcv1, INPUT);
	pinMode(rcv2, INPUT);
	pinMode(rcv3, INPUT);
	pinMode(m1_ain, INPUT);
	pinMode(m2_ain, INPUT);
	pinMode(rcv3, INPUT);
	pinMode(led1, OUTPUT);
	pinMode(led2, OUTPUT);
	pinMode(ledm, OUTPUT);
	pinMode(buzz, OUTPUT);

	pinMode(Trig, OUTPUT);
	pinMode(Echo, INPUT);

	pinMode(re1, OUTPUT);
	pinMode(re2, OUTPUT);
	pinMode(re3, OUTPUT);

	pinMode(PIN_LED0, OUTPUT);
	pinMode(PIN_LED1, OUTPUT);
	pinMode(PIN_LED2, OUTPUT);
	pinMode(PIN_LED3, OUTPUT);
	pinMode(PIN_SW, INPUT);

	digitalWrite(buzz, LOW);

	digitalWrite(ledm, HIGH);

	/* プルアップ */
	/* digitalWrite(pin, value)
	pin: ピン番号
	value: HIGHかLOW */
	digitalWrite(30, HIGH);                        // 30番ピンをプルアップ


	/* 外部割込み設定 */
	/* attachInterrupt(interrupt, function, mode)
	interrupt: 割り込み番号 0または1
	function: 割り込み発生時に呼び出す関数
	mode: 割り込みを発生させるトリガ */
	
	attachInterrupt(2, rotRotEnc, CHANGE);        // 30番ピンの値が変化すると割り込み発生
	

	/* タイマー割り込み設定 */
	/* MsTimer2::set(unsigned long ms, void (*f)())
	ms: 割り込み周期
	(*f)(): 割り込みで呼ばれる関数 */
	
	timer_regist_userfunc(&int_1msec);


	/* LCD設定 */
	/* begin(cols, rows)
	cols: 桁数(横方向の字数)
	rows: 行数 */
	lcd.begin(16, 2);                             // 桁数16 行数2に設定

	lcd.print("SPEC_prot");

	/* LCD カーソル設定 */
	/* setCursor(col, row)
	col: 桁 (0が左端)
	row: 行 (0が1行目)  */
	lcd.setCursor(0, 1);

	lcd.print("       ver. 1.07");

	delay(3000);
	/* LCD クリア */
	/* clear() */
	lcd.clear();
												   /* EEPROM データ読み込み */
//	read_eeprom();                                // EEPROMのパラメータを読み込み

	MMC.begin();
	MMC.remove("log.csv");
	File file = MMC.open("log.csv", FILE_WRITE);
	file.print("pattern");
	file.print(",motor1");
	file.print(",motor2");
	file.print(",m1_analog");
	file.print(",m2_analog");
	file.print(",brake");
	file.print(",lEncoderTotal");
	file.print(",iEncoder");
	file.print(",cnt1");
	file.println("");
	file.close();
	
	onboard_flash();

  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(100); // Delay for a period of time (in milliseconds).

  }

}



/************************************************************************/
/* メインプログラム                                                   　*/
/************************************************************************/
void loop(void) {

	motor_ain();
	setup_parameter();                            // パラメータ設定関数

	AUX();
	

	
	if (pattern >= 100) {
		digitalWrite(re1, LOW);
		digitalWrite(re2, LOW);
		digitalWrite(re3, LOW);
	}

	if (pattern < 100) {
		digitalWrite(re1, HIGH);
		digitalWrite(re2, HIGH);
		digitalWrite(re3, HIGH);
	}


	digitalWrite(led1, HIGH);

	switch (pattern) {
	case 0:
		sw_input();
		brake_s(Angle_count);
		
		distance();
		
		if (PIN_SW_val == LOW || RV3 >= 1500) {
			lcd.clear();
			buzzer(50, 20);
			pattern = 1;
			break;
		}
		break;
		
	case 1:
		motor(0, 0);
		sw_input();
		brake_s(Angle_brake);		
		if (PIN_SW_val == LOW || RV3 >= 1500) {
			lcd.clear();
			buzzer(100, 5);
			cnt1 = 0;
			pattern = 2;
			break;
		}
		break;

	case 2:
		brake_s(Angle_brake);
		led_flash(50);
		if ( cnt1 >= Start_WAIT * 100) {
			lEncoderTotal = 0;
			lEncoderBuff = lEncoderTotal;
			buzzer(500, 2);
			pattern = 3;
			break;
		}
		break;

	case 3:
		motor(40, 40);
		brake_s(Angle_count);
		digitalWrite(led1, LOW);
		if (lEncoderTotal - lEncoderBuff >= 3000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			pattern = 4;
			break;
		}
		if (lEncoderTotal >= UPDISTANCE * 1000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			lcd.clear();
			cnt1 = 0;
			pattern = 21;
			break;
		}
		break;

	case 4:
		motor(50, 50);
		brake_s(Angle_count);
		if (lEncoderTotal - lEncoderBuff >= 3000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			pattern = 5;
			break;
		}
		if (lEncoderTotal >= UPDISTANCE * 1000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			lcd.clear();
			cnt1 = 0;
			pattern = 21;
			break;
		}
		break;

	case 5:
		motor(60, 60);
		brake_s(Angle_count);
		if (lEncoderTotal - lEncoderBuff >= 3000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			pattern = 6;
			break;
		}
		if (lEncoderTotal >= UPDISTANCE * 1000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			lcd.clear();
			cnt1 = 0;
			pattern = 21;
			break;
		}
		break;

	case 6:
		motor(100, 100);
		brake_s(Angle_count);
		if (lEncoderTotal >= UPDISTANCE * 1000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			lcd.clear();
			cnt1 = 0;
			pattern = 21;
			break;
		}
		break;

	case 21:
		motor(0, 0);
		if ( cnt1 >= 50 ) {
			cnt1 = 0;
			pattern = 22;
			break;
		}
		break;

	case 22:
		motor(-30, -30);
		brake_s(0);
		led_flash(100);
		if (cnt1 >= 200) {
			lcd.clear();
			cnt1 = 0;
			pattern = 23;
			break;
		}
		break;
		
	case 23:
		motor(-30, -30);
		brake_s(0);
		lEncoderTotal = 0;
		lEncoderBuff = 0;
		led_flash(100);
		if ( iEncoder >= 5) {
			lcd.clear();
			cnt1 = 0;
			lEncoderTotal = 0;
			lEncoderBuff = 0;
			pattern = 31;
			break;
		}
		if ( cnt1 >= WAIT * 100 ) {
			lcd.clear();
			cnt1 = 0;
			lEncoderTotal = 0;
			lEncoderBuff = 0;
			pattern = 32;
			break;
		}
		break;

	case 31:
		motor(-30, -30);
		brake_s(Angle_count);
		if (cnt1 >= 50) {
			pattern = 32;
			break;
		}
		break;
		
	case 32:
		motor(-30, -30);
		brake_s(Angle_count);
		if (lEncoderTotal - lEncoderBuff >= 1000 / enc_cal) {
			lEncoderBuff = lEncoderTotal;
			pattern = 33;
			break;
		}
		break;

	case 33:
		motor( 0, 0 );
		brake_s(Angle_count);
		if (iEncoder >= 20) {
			lEncoderBuff = lEncoderTotal;
			pattern = 34;
			break;
		}
		if (lEncoderTotal >= DWDISTANCE * 1000 / enc_cal) {
			pattern = 41;
			break;
		}
		break;

	case 34:
		motor(-30, -30);
		brake_s(Angle_count);
		if (iEncoder <= 15 ) {
			lEncoderBuff = lEncoderTotal;
			pattern = 33;
			break;
		}
		if (lEncoderTotal >= DWDISTANCE * 1000 / enc_cal) {
			cnt1 = 0;
			pattern = 41;
			break;
		}
		break;
		
		
	case 41:
		motor(15, 15);
		brake_s(Angle_count);
		if ( cnt1 >= 100) {
			lEncoderBuff = lEncoderTotal;
			pattern = 42;
			break;
		}
		break;
		
	case 42:
		distance();
		motor(10, 10);
		brake_s(Angle_count);
		if ( Distance <= 40 ) {
			lEncoderTotal = 0;
			pattern = 1;
			break;
		}
		
		break;

	case 101:
		motor(0, 0);
		brake_s(90);
		break;

	}

}


/************************************************************************/
/* シリアルモニタ関数                                                   */
/*                                                                      */
/************************************************************************/
void serialmonitor(void)
{

	Serial.print("pattern: ");
	Serial.print(pattern);
	Serial.print(", lEncoderTotal: ");
	Serial.print(lEncoderTotal);
	Serial.print(", iEncoder: ");
	Serial.print(iEncoder);
	Serial.print(", val1: ");
	Serial.print(val1);
	Serial.print(", val2: ");
	Serial.print(val2);
	Serial.print(", RV1: ");
	Serial.print(RV1);
	Serial.print(", RV2: ");
	Serial.print(RV2);
	Serial.print(", RV3: ");
	Serial.print(RV3);
	Serial.print(", m1_analog: ");
	Serial.print(m1_ain_val);
	Serial.print(", m2_analog: ");
	Serial.print(m2_ain_val);
	Serial.print(", Distance: ");
	Serial.print(Distance);
	Serial.print(", cnt1: ");
	Serial.println(cnt1);

}


/************************************************************************/
/* microSDログ保存                                                      */
/*                                                                      */
/************************************************************************/
void microSDProcess(void)
{

	sprintf(msd, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", pattern, m1_val, m2_val, m1_ain_val, m2_ain_val, brake_val, lEncoderTotal, iEncoder,cnt1);

	File file = MMC.open("log.csv", FILE_WRITE);
	file.write(msd);
	file.close();

}


/************************************************************************/
/* ブザー                                                         　 */
/*                                                                      */
/************************************************************************/
void buzzer(int interval, int count)
{
	for (int i=0; i <= count * 2; i++) {
		digitalWrite(buzz, buzz_output);
		buzz_output = !buzz_output;
		delay(interval);
	}
	digitalWrite(buzz, LOW);
}

/************************************************************************/
/* アナログ入力                                                      　 */
/*                                                                      */
/************************************************************************/
void motor_ain(void)
{

	m1_ain_val = analogRead(m1_ain);
	m2_ain_val = analogRead(m2_ain);

}

/************************************************************************/
/* スイッチ入力	                                                     　 */
/*                                                                      */
/************************************************************************/
void sw_input(void)
{

	sw1_val = digitalRead(sw1);
	sw2_val = digitalRead(sw2);
	sw3_val = digitalRead(sw3);
	sw4_val = digitalRead(sw4);
	sw5_val = digitalRead(sw5);
	PIN_SW_val = digitalRead(PIN_SW);
	dsw_val = digitalRead(dsw);
	delay(40);
	
	if (dsw_val == LOW) {                          // シリアルモニタ切り替え
		serialmonitor();
	}

}


/************************************************************************/
/* AUX入力                                                       　 */
/*                                                                      */
/************************************************************************/
void AUX(void)
{

	/* plusein設定 */
	/* pulseIn(pin, value, timeout)
	pin: パルスを入力するピンの番号
	value: 測定するパルスの種類。HIGHまたはLOW
	timeout(省略可): タイムアウトまでの時間(単位・マイクロ秒)。デフォルトは1秒 (unsigned long) */
	RV1 = pulseIn(rcv1, HIGH, 21000);
	RV2 = pulseIn(rcv2, HIGH, 21000);
	RV3 = pulseIn(rcv3, HIGH, 21000);

	if (RV1 >= 1500) {
		RV1_f++;
		if (RV1_f >= 10) {
			AUX1 = 1;
		}
	}
	else {
		RV1_f = 0;
		AUX1 = 0;
	}
	
	if (AUX1 == 1) {                          // 自律手動切り替え
		pattern = 101;
	}

}

/************************************************************************/
/* 超音波センサ距離測定                                              　 */
/*                                                                      */
/************************************************************************/
void distance(void)
{
	digitalWrite(Trig, LOW);
	delayMicroseconds(1);
	digitalWrite(Trig, HIGH);
	delayMicroseconds(11);
	digitalWrite(Trig, LOW);
	Duration = pulseIn(Echo, HIGH);
	if (Duration>0) {
		Distance = Duration / 2;
		Distance = Distance * 340 * 100 / 1000000; // ultrasonic speed is 340m/s = 34000cm/s = 0.034cm/us 
	}
}

/************************************************************************/
/* led_flash                                                         　 */
/*                                                                      */
/************************************************************************/
void led_flash(int interval)
{
	digitalWrite(led1, led1_output);
	led1_output = !led1_output;
	delay(interval);
}


/************************************************************************/
/* onboard_flash                                                     　 */
/*                                                                      */
/************************************************************************/
void onboard_flash(void)
{
	digitalWrite(PIN_LED0, 1);
	digitalWrite(PIN_LED1, 1);
	digitalWrite(PIN_LED2, 1);
	digitalWrite(PIN_LED3, 1);

}
/************************************************************************/
/* write_eeprom                                                         */
/*                                                                      */
/************************************************************************/
void write_eeprom(void)
{

	
	EEPROM.write(0x7fff, UPDISTANCE);
	delay(5);
	EEPROM.write(0x7ffe, DWDISTANCE);
	delay(5);
	EEPROM.write(0x7ffd, WAIT);
	delay(5);
	EEPROM.write(0x7ffc, Start_WAIT);
	delay(5);


}



/************************************************************************/
/* read_eeprom                                                         */
/*                                                                      */
/************************************************************************/
void read_eeprom(void)
{

	UPDISTANCE = EEPROM.read(0x7fff);
	delay(5);
	DWDISTANCE = EEPROM.read(0x7ffe);
	delay(5);
	WAIT = EEPROM.read(0x7ffd);
	delay(5);
	Start_WAIT = EEPROM.read(0x7ffc);
	delay(5);


}



/************************************************************************/
/* setup_parameter                                                      */
/*                                                                      */
/************************************************************************/
void setup_parameter(void)
{
	int i;
	
	if (pattern != 0) {
		if (cnt_lcd >= 25) {
			cnt_lcd = 0;
			lcd.setCursor(0, 0);
			/* 0123456789abcbef 1行16文字 */
			lcd.print("Pattern    = ");
			lcd.print(pattern);
			lcd.setCursor(0, 1);
			/* 01234567..89abcde.f 1行16文字 */
			lcd.print("Time       = ");
			lcd.print(cnt1 / 100);
		}
		return;
		lcd.clear();
	}
	

	if (sw1_val == 0) {
		lcd_pattern++;
//		write_eeprom();
		buzzer(50,2);
	}

	/* スイッチ3　メニュー＋１ */
	if (sw3_val == 0) {
		lcd_pattern++;
		lcd.clear();
		if (lcd_pattern == 6) lcd_pattern = 1;
	}

	/* スイッチ2　メニュー－１ */
	if (sw2_val == 0) {
		lcd_pattern--;
		lcd.clear();
		if (lcd_pattern == 0) lcd_pattern = 5;
	}
	/* LCD、スイッチ処理 */
	switch (lcd_pattern) {

	case 1:
		/* 上昇距離 */
		i = UPDISTANCE;
		if (sw5_val == 0) {
			i++;
			if (i > 1000) i = 1000;
		}
		if (sw4_val == 0) {
			i--;
			if (i < 0) i = 0;
		}

		UPDISTANCE = i;

		lcd.setCursor(0, 0);
		/* 0123456789abcd..f 1行16文字 */
		lcd.print("01 UPDIST  =  ");
		lcd.print(i);
		lcd.setCursor(0, 1);
		/* 01234567..89abcde.f 1行16文字 */
		lcd.print("Time       = ");
		lcd.print(cnt1 / 100);
		break;

	case 2:
		/* 下降距離 */
		i = DWDISTANCE;
		if (sw5_val == 0) {
			i++;
			if (i > 1000) i = 1000;
		}
		if (sw4_val == 0) {
			i--;
			if (i < 0) i = 0;
		}

		DWDISTANCE = i;

		lcd.setCursor(0, 0);
		/* 0123456789abcd..f 1行16文字 */
		lcd.print("02 DWDIST  =  ");
		lcd.print(i);
		lcd.setCursor(0, 1);
		/* 01234567..89abcde.f 1行16文字 */
		lcd.print("Time       = ");
		lcd.print(cnt1 / 100);
		break;

	case 3:
		/* 停止時間 */
		i = WAIT;
		if (sw5_val == 0) {
			i++;
			if (i > 100) i = 100;
		}
		if (sw4_val == 0) {
			i--;
			if (i < 0) i = 0;
		}

		WAIT = i;

		lcd.setCursor(0, 0);
		/* 0123456789abcd..f 1行16文字 */
		lcd.print("03 WAIT   =   ");
		lcd.print(i);
		lcd.setCursor(0, 1);
		/* 01234567..89abcde.f 1行16文字 */
		lcd.print("Time       = ");
		lcd.print(cnt1 / 100);
		break;

	case 4:
		/* スタート待ち時間 */
		i = Start_WAIT;
		if (sw5_val == 0) {
			i++;
			if (i > 100) i = 100;
		}
		if (sw4_val == 0) {
			i--;
			if (i < 0) i = 0;
		}

		Start_WAIT = i;

		lcd.setCursor(0, 0);
		/* 0123456789abcd..f 1行16文字 */
		lcd.print("04 Start_W  = ");
		lcd.print(i);
		lcd.setCursor(0, 1);
		/* 01234567..89abcde.f 1行16文字 */
		lcd.print("Time       = ");
		lcd.print(cnt1 / 100);
		break;

	case 5:
		/* スタート待ち時間 */
		lcd.setCursor(0, 0);
		/* 0123456789abcd..f 1行16文字 */
		lcd.print("05 lEnc  = ");
		lcd.print(lEncoderTotal);
		lcd.setCursor(0, 1);
		/* 01234567..89abcde.f 1行16文字 */
		lcd.print("Distance = ");
		lcd.print(Distance);
		break;
	}
}


/************************************************************************/
/* 駆動輪の速度制御                                                      */
/* 引数　 -100～100                                                     */
/*        0で停止、100で正転100%、-100で逆転100%                         */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor(int accele1, int accele2)
{
	m1_val = accele1;
	m2_val = accele2;
	val1 = map(accele1, -100, 100, 0, 180);
	val2 = map(accele2, -100, 100, 0, 180);
	motor1.write(val1);
	motor2.write(val2);
}


/************************************************************************/
/* ブレーキの制御                                                     　 */
/* 引数　 -100～100                                                     */
/*        0で停止、100で正転100%、-100で逆転100%                         */
/* 戻り値 なし                                                          */
/************************************************************************/
void brake_s(int accele3)
{
	brake_val = accele3;
	brake.write(accele3);
}


/************************************************************************/
/* 外部割込み                                                         　 */
/* 立ち上がり、立下り両エッジで検出                                      */
/************************************************************************/
void rotRotEnc(void)
{
	lEncoderTotal++;                           // データーをインクリメント
	digitalWrite(led2, output);      // 13番ピン(LED)に出力する(HIGH>ON LOW>OFF)
	output = !output;
	DisatanceTotal = lEncoderTotal * 1.075;
}


/************************************************************************/
/* タイマー割り込み                                                   　 */
/************************************************************************/
void int_1msec(void) {                   // MsTimer2割込み処理
	
    iTimer10++;
	iTimer100++;
    switch( iTimer10 ) {
    case 1:
		if (pattern >= 2) microSDProcess();
        break;

    case 2:
		iEncoder = lEncoderTotal - uEncoderBuff;
		uEncoderBuff = lEncoderTotal;
        break;

    case 3:
		cnt1++;
		cnt_buzz++;
		cnt_lcd++;
        break;

     case 4:

        break;

    case 5:
        break;

    case 6:
        break;

    case 7:
        break;

    case 8:
        break;

    case 9:
        break;

    case 10:
        /* iTimer10変数の処理 */
        iTimer10 = 0;
        break;
    }
	
	
}



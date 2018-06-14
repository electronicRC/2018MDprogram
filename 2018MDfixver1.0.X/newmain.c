#include <xc.h>
#include <pic16F1827.h>

#define _XTAL_FREQ  32000000     // delay(32MHz)

// 
#pragma config FOSC = INTOSC    // 内部クロックで動作(INTOSC)
#pragma config WDTE = OFF       // (OFF)
#pragma config PWRTE = ON       // ON64ms(ON)
#pragma config MCLRE = OFF      // (RA5)(OFF)
#pragma config CP = OFF         // (OFF)
#pragma config CPD = OFF        // (OFF)
#pragma config BOREN = ON       // ON(ON)
#pragma config CLKOUTEN = OFF   // CLKOUTRA6(OFF)
#pragma config IESO = OFF       // (OFF)
#pragma config FCMEN = OFF      // (FCMEN_OFF)

// 
#pragma config WRT = OFF        // Flash(OFF)
#pragma config PLLEN = ON      // 内部クロックを４倍、32MHzで動作(ON)
#pragma config STVREN = ON      // (ON)
#pragma config BORV = HI        // (2.5V)(HI)
#pragma config LVP = OFF        // (OFF)

#use i2c(SLAVE,SDA=PIN_B1,SCL=PIN_B4,SLOW,NOFORCE_SW)

//
/*0x81*/
#define ADDRESS		0x88

/*--LED--*/
#define PWR_LED		RA6	//
#define USER_LED1	RA7	//
#define USER_LED2	RA0	//

/*--SCL,SDA--*/
#define S_CLK		PB4
#define S_DAT		PB1

/**/
#define ROT_SW		RA2

/**/
#define SBREAK		0x200

/*--duty(duty)--*/
#define TRANSIT	16

//
void setup(void);
void receive(void);
void inc(void);
void dec(void);
void mode(void);

void i2c_start(void);
void i2c_write(int address,int data[]);
void i2c_stop(void);
void i2c_idle(void);
void i2c_inrt(void);
void i2c_2018inrt(void);

void interrupt SubRoutine(void);

//
int baff_data = 0x80;			//
long duty = SBREAK;				//duty
long old_duty =	SBREAK;			//duty
int indirect_data = 0x80;		//
int flag_sb = 0;
int flag_run = 0;
int flag_act = 0;
int flag_mode = 0;
int data = 0x80;

static int icount = 0;

//-------------------------------------setup--------------------------------------
void setup()
{
	OSCCON = 0b01110000 ;     // 32(PULLEN ON)
    ANSELA = 0b00000000 ;     // AN0-AN4I/O
    ANSELB = 0b00000000 ;     // AN5-AN11I/O
    TRISA  = 0b00000000 ;     // (RA)(RA5)
    TRISB  = 0b00000000 ;     // (RB)
    PORTA  = 0b00000000 ;     // RA(LOW)
    PORTB  = 0b00000000 ;     // RB(LOW)

	PWR_LED = 1;	//LED

	//icount = 0;

	//0
	//OPTION_REG = 0x07;//1/256
    //TMR0 = 0xD9;//
    
	PIE1bits.SSP1IE = 1;//i2c
    INTCONbits.PEIE = 1;//
    INTCONbits.GIE = 1;//
   
	//2(,2)
	setup_timer_2(T2_DIV_BY_1,0xFF,1);

}

//----------------------------------duty------------------------------------
void inc()
{
	old_duty = old_duty + TRANSIT;
}

//----------------------------------duty------------------------------------
void dec()
{
	old_duty = old_duty - TRANSIT;
}

//---------------------------------------------------------------------
void mode(){
	if(!input(ROT_SW)){
		flag_mode = 1;
	}
	else{
		flag_mode = 0;
	}
}

//-------------------------------I2C--------------------------------
#INT_SSP
void receive()
{
	if(i2c_poll()){
		baff_data = i2c_read();
		output_high(USER_LED1);
		icount = 0;
	}
}

//-------------------------------0----------------------------------

#INT_TIMER0
void intval(){
	set_timer0(0xD9);
	icount++;
	if(icount >= 100){
		setup_ccp2(CCP_OFF);
		output_toggle(USER_LED2);
		icount = 0;
	}
}

//----------------------------------------------------------------------------
int main(void)
{
	int indirect_data = SBREAK;
	setup();
	mode();

	while(true){
		//1(while1)
		indirect_data = baff_data;
		duty = baff_data;

		//duty
		if(flag_mode == 1){
			duty = 128-duty;
		}

		duty = duty*8;

		//0x81
		if(indirect_data <= 0x80){
			//duty5%

			//--------------------------------------------------------------------------------//
			//duty0x01(01)~0x3C(60), 0x44(68)~0x80(128)
			//0x3D~0x4360duty
			//dutyX x / 64(0 <= x <= 60),
			// x / 128(68<= x <= 128)
			//0x20103232 / 64 = 0.5duty50%()
			//0x16102222 / 64 = 0.343...duty34%()

			if((indirect_data >= 0x3D) && (indirect_data <= 0x43)){
				if(old_duty < SBREAK){
					inc();
				}
				else if(old_duty > SBREAK){
					dec();
				}
				if(old_duty == SBREAK){
					flag_sb = 1;
					flag_run = 0;
				}
				output_low(USER_LED2);
			}

			else if((indirect_data < 0x3D) || (indirect_data > 0x43)){
				if(old_duty < duty){
					inc();
				}
				else if(old_duty > duty){
					dec();
				}
				output_high(USER_LED2);
				flag_run = 1;
			}
		}
		// Nucleo
		//12V
		//PWM
		//								by    2018/05/11
		
		if(flag_run == 1){
			if(flag_sb == 1){
				flag_act = 1;
			}
		}
		else if(flag_sb == 1){
			setup_ccp2(CCP_OFF);
		}
		if(flag_act == 1){
			setup_ccp2(CCP_PWM_H_H | CCP_PWM_HALF_BRIDGE);
			flag_sb = 0;
			flag_act = 0;
		}

		set_pwm2_duty(old_duty);
		//delay_ms(10);				//picCCPir	by    2018/05/11

	}
}

//割り込み関数
void interrupt SubRoutine(int rcv_pattern){
    if(PIR1bits.SSP1IF == 1){//i2cフラグ判定
        if (rcv_pattern = 0) {
            i2c_inrt();//i2c通常受信関数読み込み
        }
        else{
            i2c_2018inrt();//2018通信フォーマット受信関数
        }
        PIR1bits.SSP1IF =0;//割り込みフラグリセット
    }
}
//i2cアイドル状態関数
void i2c_idle(void){
    while (SSP1CON2bits.SEN//スタートビット送信中(マスター)、ストレッチを許可(スレーブ)
        || SSP1CON2bits.PEN//ストップコンディション送信中(マスター)
        || SSP1CON2bits.RCEN//10bitのリピートスタートコンディション送信中(マスター)
        || SSP1CON2bits.ACKEN//ACKDTビットを送信中(マスター受信中)
        || SSP1STATbits.R_nW//read受信で正、write受信で負(スレーブ)、送信中で正(マスター)
        );
}

//7bitモード受信関数
void i2c_inrt(void){
    int data;
    SSP1CON2bits.ACKDT = 0;//ASK設定
    while (SSP1STATbits.BF = 0) {//受信バッファチェック
        SSP1CON2bits.ACKEN = 1;//ACK返信
    }
    data = SSP1BUF;//データ受信
    SSP1CON1bits.SSPOV = 0;//エラークリア
    i2c_idle();//アイドル待ち
    return data;
}



void i2c_start(void){
    i2c_idle();
    SSP1CON2bits.SEN = 1;
}                    
#include <htc.h>
#include <pic16f1824.h>

#define _XTAL_FREQ 16000000

__CONFIG(FOSC_INTOSC & WDTE_OFF & PWRTE_ON & MCLRE_ON & CP_OFF & BOREN_ON & CLKOUTEN_OFF & IESO_OFF & FCMEN_OFF);
__CONFIG(WRT_BOOT & STVREN_ON & BORV_LO & LVP_OFF & PLLEN_OFF);

#define led trig_output
#define signal_output echo_output
#define echo_output LATC4
#define trig_output LATC3
#define output_pwr LATC5
#define thresh LATA2
#define thresh_tris TRISA2
#define wav_mask 0b11111100
#define wav_1 0b00000010
#define wav_2 0b00000001

#define min_echo 200	//200=6.8cm. time in microseconds. if echo is sooner than this, reject it and wait for the second echo
#define max_distance 300	//cm
#define min_distance 10	//cm
#define max_step_change 40	//cm
#define min_throttle 50
#define throttle_output_max 220
#define throttle_output_min 80
#define rate 100	//ping rate, ms
#define limit_lo 20
#define limit_hi 60
#define dac_level 25
#define default_num_echoes 5
#define num_echoes_analyzed 3
#define abort_threshold 25
#define abort_threshold_sig 40
#define time_to_abort 3000	//milliseconds
#define enable_threshold 127
#define throttle_deadband 50
#define output_1_failsafe 50

#define i_limit 7000//50000
#define i_gain 9//15  //inverse of this value, then /10
#define d_gain 14//75	//   /10
#define p_gain 4	//   /10
#define time_limit_to_use_dt 400

unsigned int led_timer;
unsigned int timer;
bit update;
bit new_data;
bit accept;
unsigned char ping_state;
unsigned int start;
unsigned int echo_time_1;//, echo_time_2, echo_time_3;
unsigned int multiple_echo[8];
unsigned int echo_period;
unsigned char multiple_echo_index;
unsigned char number_echoes;
unsigned int period_1, period_2;
unsigned int ping_timer;
unsigned int samples[4];
unsigned char sample_index;
unsigned int distance;
unsigned char output_1;
unsigned int input_1, input_2;
unsigned int timer_1, timer_2; 
unsigned char overflows_1, overflows_2;
bit update_1; 
bit update_2;
bit output_fire;
bit signal_loss_1;
bit signal_loss_2;
bit enable_alt_hold;
bit disable_alt_hold;
bit output_state;
unsigned char out_1_state;
unsigned int alt_change_holdoff;
unsigned char throttle;
unsigned char throttle_down_count;
unsigned char aux;
unsigned char throttle_baseline;
unsigned int altitude;
unsigned int altitude_last;
unsigned int altitude_setpoint;
unsigned int last;
signed int error_history[8];
unsigned char error_index;
unsigned int samples[4];
unsigned char index;
signed int result;
signed int result_last;
signed int plant_derivative;
signed int error;
signed int derivative;
signed int integral;
unsigned char pid_output;
unsigned char bad_alt_count;
unsigned char bad_sig_count;
bit altitude_good;
unsigned int ms_without_pings;
bit ms_reset;
bit update_pid;


unsigned char servo_scale(unsigned int microseconds);
void init_alt_hold(void);
void configure(void);
unsigned int get_analog(void);
void delayms(int milliseconds);
void ping(void);
void tune(void);
void ping_machine(void);
void indicate_alt_hold(unsigned char state);
void handle_inputs(void);
void failsafe_logic(void);
void pid_update(void);

void interrupt isr(void)
{
	if(C1IF){
		C1IF=0; 
		if(ping_state==2){	//waiting for first echo. 
			echo_time_1 = TMR1-start;	//get echo time
			if(echo_time_1>min_echo){
				multiple_echo[0]=echo_time_1;
				ping_state++;
				ping_timer=2;	//set timeout in case it never comes in
			}
		}
		else if(ping_state==3){//waiting for multiple echoes
			multiple_echo[multiple_echo_index]=TMR1-start;
			multiple_echo_index++;
			if(multiple_echo_index==number_echoes) ping_state++;
		}	
	}	

	if(TMR0IF){
		TMR0IF=0;
		if(led_timer){
			led=1;
			led_timer--;
			if(led_timer==0) led=0;
		}	
		if(++timer>rate){
			timer=0;
			update=1;
		}	
		if(ping_timer){
			ping_timer--;
			if(ping_timer==0){
				if(ping_state==2 || ping_state==3){	//timeout waiting for  echo
					thresh_tris=0;	//stop listening
					thresh=0;
					ping_state=1;	//retry
					if(ping_state==2) bad_alt_count++;
					else bad_sig_count++;
				}	
			}
		}		
		if(alt_change_holdoff) alt_change_holdoff--;
		if(ms_reset){
			ms_without_pings=0;
			ms_reset=0;
		}
		else{
			ms_without_pings++;
			if(ms_without_pings>time_limit_to_use_dt) update_pid=1;
		}
	}

	if(TMR1IF){
		TMR1IF=0;
		overflows_1++;
		overflows_2++;
		if(overflows_1>31){	//2 seconds without signal
			signal_loss_1=1;
		}
		if(overflows_2>31){	//2 seconds without signal
			signal_loss_2=1;
		}
	}
	
	if(TMR2IF){
		TMR2IF=0;
		if(output_state){
			output_state=0;
			TMR2=255-output_1;
		}
		else if(output_fire){
			output_fire=0;
			output_state=1;
			signal_output=1;
		}
		else{
			signal_output=0;
		}
	}	

	if(IOCIF){
		if(IOCAF5){	//if channel 1 fired
			IOCAF5=0; 
			if(RA5){	//if it went high
				timer_1=TMR1;	//mark start time
				overflows_1=0; //clear overflow monitor
				signal_loss_1=0; //indicate valid signal
			}
			else{	//if it went low
				if(overflows_1<2){	//if tmr1 rolled more than once it's invalid
					input_1 = TMR1 - timer_1;
					update_1=1;	//signal new data
				}
			}
		}
		if(IOCAF4){	//if channel 2 fired
			IOCAF4=0; 
			if(RA4){	//if it went high
				timer_2=TMR1;	//mark start time
				overflows_2=0; //clear overflow monitor
				signal_loss_2=0; //indicate valid signal
			}
			else{	//if it went low
				if(overflows_2<2){	//if tmr1 rolled more than once it's invalid
					input_2 = TMR1 - timer_2;
					update_2=1;	//signal new data
				}
			}
		}
	}
	
}
		


void main(void)
{
	configure();
	delayms(500);
	tune();	
	number_echoes=default_num_echoes;	
	GIE=1;
	
	while(1){

		ping_machine();

		handle_inputs();
		
		failsafe_logic();
		
		if(new_data){
			new_data=0;
			signed int delta = distance-last;
			last = distance;
			if(delta<0) delta = -delta;
			if(delta<max_step_change){
				if(distance>min_distance && distance<max_distance){
					samples[index]= distance;
					altitude = distance;
					index++; index&=0b00000011;
					derivative = samples[index] - distance;
					if(disable_alt_hold==0) led_timer=6;
					altitude_good=1;
					ms_reset=1;
					bad_alt_count=0;
					update_pid=1;
				}
				else bad_alt_count++;
			}		
			else bad_alt_count++;
		}
		
		if(update_pid){update_pid=0; pid_update();}

				
	}	
}//main function

void pid_update(void)
{
	error = altitude_setpoint - altitude; //positive errors = positive output change
	//8x moving average on inputs to integral to reject noise
	error_index++; error_index&=0b00000111;
	error_history[error_index]=error;
	signed int avg=0;
	for(char i=0; i<8; i++){avg+=error_history[i];}
	avg/=8;
	integral+=(avg/4); //cutting down speed of accumulation
	if(integral>i_limit) integral=i_limit;
	if(integral<0) integral=0;//	if(integral<-i_limit) integral=-i_limit;

	altitude_last=altitude;
	result_last=result;
	if(ms_without_pings>time_limit_to_use_dt) derivative=0;
	result = error*p_gain + derivative*d_gain + integral/i_gain;
	result/=10;
	//plant_derivative= result_last - result; //
	//result += plant_derivative/plant_d_gain;
	result+=throttle_baseline;
	if(result>throttle_output_max) result=throttle_output_max; 
	if(result<throttle_output_min) result=throttle_output_min;
	pid_output=result;
}

void failsafe_logic(void)
{
//	if((bad_alt_count>abort_threshold) || (bad_sig_count>abort_threshold_sig)){
//		altitude_good=0;
//		bad_alt_count=0;
//		bad_sig_count=0;
//	}	
	if(ms_without_pings>time_to_abort){
		altitude_good=0;
	}	
	if((altitude_good==0) && enable_alt_hold){
		enable_alt_hold=0; 
		indicate_alt_hold(0);//LATC2=0; 
		disable_alt_hold=1;
	}
	if(disable_alt_hold) led=1;
}

void handle_inputs(void)
{
	if(update_1){
		update_1=0;
		throttle = servo_scale(input_1);
		if(enable_alt_hold){
			if(throttle<min_throttle) throttle_down_count++;
			else throttle_down_count=0;
			if(throttle_down_count>7){
				throttle_down_count=0;
				enable_alt_hold=0;
				disable_alt_hold=1;
				indicate_alt_hold(0);//LATC2=0;
				output_1=throttle;
			}
			else{
				output_1 = pid_output;
			}	
			if(alt_change_holdoff==0){
				if(throttle>(throttle_baseline+throttle_deadband)){
					if(altitude_setpoint<max_distance) altitude_setpoint++;
					alt_change_holdoff=500; //2cm/s change rate
				}
				else if(throttle<(throttle_baseline-throttle_deadband)){
					if(altitude_setpoint>min_distance) altitude_setpoint--;
					alt_change_holdoff=500;
				}
			}
		}
		else{
			output_1 = throttle;
			if(throttle<min_throttle){
				unsigned char reducer = min_throttle - throttle;
				//reducer/=2;
				if(throttle>reducer) output_1 -= reducer;
				else output_1=2;
			}	
		}
		output_fire=1;
	}	

	if(update_2){
		update_2=0;
		aux = servo_scale(input_2);	
		if(aux<enable_threshold){
			if((altitude_good) && (throttle>min_throttle) && (disable_alt_hold==0)){
				if(enable_alt_hold==0) init_alt_hold();
				enable_alt_hold=1;
				indicate_alt_hold(1);//LATC2=1;
			}
		} 
		else{
			disable_alt_hold=0;
			enable_alt_hold=0;
			indicate_alt_hold(0);//LATC2=0;
		}	
	}	
}

void indicate_alt_hold(unsigned char state)
{
	//no indicator on v2 hardware at this time
}

void ping_machine(void)
{
	if(update){
		update=0;
		if(ping_state==0) ping_state=1;	//if idle, initiate new ping
	}

	switch(ping_state){
		case 0: //idle
			break;
		case 1:		//initial. send ping
			//start=TMR1;
			ping();
			start=TMR1-250; //pulse train takes 200us to run, so this is now the start time. 
			ping_timer=40;	//start timer for maximum time to wait for echo
			thresh_tris=1;	//start listening
			multiple_echo_index=1;
			ping_state=2;
			break;
		case 2:		//waiting for first echo
			break;		//transitions handled in interrupts
		case 3:		//multiple echoes in process
			break;
		case 4:		//echoes received
			thresh_tris=0;	//stop listening
			thresh=0;
			accept=1;
			for(char i=(number_echoes-num_echoes_analyzed); i<number_echoes; i++){
				echo_period = multiple_echo[i] - multiple_echo[i-1];
				if(echo_period>limit_hi || echo_period<limit_lo) accept=0;
			}
			if(accept){
				new_data=1;
				bad_sig_count=0;
				distance=multiple_echo[0]/60;	//cm
			}	
			else{bad_sig_count++;}

			ping_state=0;	//return to idle
			break;	
	}	
}

unsigned char servo_scale(unsigned int microseconds)	//transform microsecond measurement to a 0-255 value
{
	if(microseconds<1000){return 0;}
	if(microseconds>2000){return 255;}
	
	microseconds-=1000;	//subtract 1000	
	
	unsigned char output = (microseconds>>=2)&0xff;	//divide by 4
	output += (microseconds>>8)&0x03;	//add in 1/256th
	output += 1;	//center it within our error (2 counts)
	
	return output;	
}

void init_alt_hold(void)
{
	throttle_baseline=throttle;
	integral=0;//throttle*(i_gain*10);
	if(altitude_good){
		altitude_setpoint=altitude;
		altitude_last=altitude;
	}	
	else altitude_setpoint=100;
	pid_output=throttle;
}

unsigned int get_analog(void)
{
	GO_nDONE=1;
	while(GO_nDONE);
	return ADRES;
}

void delayms(int milliseconds)
{
	while(milliseconds!=0){ __delay_ms(1); milliseconds--;}
}

	
void configure(void)
{
	INTCON=0b01101000;	//peripheral, t0, ioc
	T1CON=0b00100001;	//source instruction clock (fosc/4). prescale 1:4 to count microseconds
	T2CON=0b00000110;	//1:16 prescale for 1ms rollover
//	T4CON=0b01011111;	//1:64 prescale for rollover in 4ms, 1:12 postscale for 48ms interrupts
	PR4=255; PR2=255;
	LATA=0;
	LATC=0;
	TRISA=0b11111111;
	TRISC=0b11000100;
	WPUA=0b11111011;
	WPUC=0b11000000;
	OSCCON=0b01111000;	//16MHz
	PIE1=0b00000011;	//tmr1 and 2
	PIE2=0b00100000;	//comparator1
	PIE3=0b00000000; 	//
	PIR1=0; PIR2=0; PIR3=0;
	OPTION_REG=0b00000011;	//1:16 for 1ms interrupt
	ANSELA=0b00000000;
	ANSELC=0b00000100;//0b00001000;
	FVRCON=0b11001000;	//2.048v to dac and comparator,nothing to  adc
	IOCAP=0b00110000;
	IOCAN=0b00110000;
	IOCAF=0;
	CM1CON0=0b10010110;	//on, inverted, internal, high speed
	CM1CON1=0b10010010;	//interrupt on positive, ref from dac, input from in2-
	DACCON0=0b10001000;	//on, output on dacout, positive from fvr
	DACCON1=dac_level;		//number out of 31 to get voltage
}

void ping(void)
{
	#define delay 10
	#define extra_delay NOP();//NOP();NOP();
	GIE=0;
	output_pwr=0; 
	__delay_us(250);
	unsigned char mask=LATC&wav_mask;
	// -------------------------
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------7
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------8
	mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
	mask|=wav_1;
	LATC=mask;
	__delay_us(delay);		//total 12.5 desired
	extra_delay
	mask&=wav_mask;
	mask|=wav_2;
	LATC=mask;
	__delay_us(delay);
	extra_delay
	// -------------------------

	
	LATC&=wav_mask;
	output_pwr=1;
	//__delay_us(50);
	GIE=1;
}	

void tune(void)
{
	#define tune_time 900
	output_pwr=0;
	__delay_us(250);
	unsigned char mask=LATC&wav_mask;
	for(int i=0; i<tune_time; i++){
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_1;
		LATC=mask;
		__delay_us(70);		//total 12.5 desired
		mask&=wav_mask;
		mask|=wav_2;
		LATC=mask;
		__delay_us(65);
	}	
	for(int i=0; i<tune_time+50; i++){
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_1;
		LATC=mask;
		__delay_us(65);		//total 12.5 desired
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_2;
		LATC=mask;
		__delay_us(60);
	}	
	for(int i=0; i<tune_time+100; i++){
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_1;
		LATC=mask;
		__delay_us(60);		//total 12.5 desired
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_2;
		LATC=mask;
		__delay_us(55);
	}	
	for(int i=0; i<tune_time+150; i++){
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_1;
		LATC=mask;
		__delay_us(55);		//total 12.5 desired
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_2;
		LATC=mask;
		__delay_us(50);
	}	
	for(int i=0; i<tune_time+200; i++){
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_1;
		LATC=mask;
		__delay_us(50);		//total 12.5 desired
		mask&=wav_mask;	//takes 9 instructions to update latc (2.25us)
		mask|=wav_2;
		LATC=mask;
		__delay_us(45);
	}	
	output_pwr=1;
	LATC&=0b00100111;
}	
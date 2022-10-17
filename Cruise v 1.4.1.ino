#define ai_cruise_pin_1 7	// 10 ���  �������// 5k // cancel, resume, speed+, speed -
#define ai_cruise_pin_2 3	// 22 ���  �������// 25k // set
#define ai_cruise_pin_4 6	// 23 ���  �������// 5k // on off
#define ai_5v_refrense 0 

#define di_lamp 3 //�����
#define drl_lamp 9 
#define di_stop 4
#define di_speed 2

#define led_red 6
#define led_green 16
#define drive_enable 8
#define drive_step 11
#define drive_dir 12
#define drive_sleep 5

#define do_servo 10
#define dio_temp_meas 7 //������ �����������
#define ai_temp_R 1		// �������� �����������

//!!!!!!!!!!!!!!!!!!!!! DRL !!!!!!!!!!!!!!!!!!!!!!!!!

#define drl_power 408 //�������� ��� 1024*% 102 244 hz 10bit
#define drl_first_start_zone 5 //����� ������ ���������� ��� ��������

//!!!!!!!!!!!!  ��������� ������   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define button_refrense 496 //�������� ��� ������������ ���������� �� ����� 5�

#define button_off 0
#define button_drl 1
#define button_on 2
#define buttton_set 3
#define button_resume 4
#define button_cancel 5
#define button_speed_up 6
#define button_speed_down 7

#define button_on_up 20			//������� ����� on
#define button_off_down 160		//������ ����� off
#define button_set_down 100

//cancel 70 up+20 down-20
#define button_cancel_up 90		//������� ����� on
#define button_cancel_down 40		//������ ����� off

//resume 200 up+20 down-20
#define button_resume_up 160		//������� ����� on
#define button_resume_down 110		//������ ����� off

//speedUP 570 - 580 up+20 down-20
#define button_SpeedUP_up 600		//������� ����� on
#define button_SpeedUP_down 540		//������ ����� off

//SpeedDOWN 940 up+20 down-20
#define button_SpeedDOWN_up 840		//������� ����� on
#define button_SpeedDOWN_down 790		//������ ����� off

#define button_delay_const 150

// !!!!!!!!!!!!!!!!!!!!!!!!!!!  ��������� ������� !!!!!!!!!!!!!!!!

// ��������� ������������� 16-29 ��������
//#define T_R_min 800
#define T_R_16 775 
#define T_R_17 715
#define T_R_18 665
#define T_R_19 615
#define T_R_20 565
#define T_R_21 515
#define T_R_22 465
#define T_R_23 415
#define T_R_24 365
#define T_R_25 315
#define T_R_26 265
#define T_R_27 215
#define T_R_28 165
#define T_R_29 115
#define T_R_30 65 
#define T_R_max 20 

//����������� ��������
#define T_Ktemp 1 

// ������ ������������
#define T_sigma_max 150		//max ���� servo
#define T_sigma_min 100		//min ���� servo

#define T_dead_time_meas 1000		//����� ��������� ��
#define T_dead_time_regulate 15000	//����� ����� ������������ ��
#define T_dead_temp 5				//���� ���������������� ������������� ���� �10 
#define T_zone_1 25					//���� ����������� ������� 1
#define T_zone_2 50					//���� ����������� ������� 2

#define T_target_min 100 //������� ����������� ��� min
#define T_target_16 160
#define T_target_17 170
#define T_target_18 180
#define T_target_19 190
#define T_target_20 200
#define T_target_21 210
#define T_target_22 220
#define T_target_23 230
#define T_target_24 240
#define T_target_25 250
#define T_target_26 260
#define T_target_27 270
#define T_target_28 280
#define T_target_29 290
#define T_target_30 300
#define T_target_max 400 //������� ����������� ��� max

//��� ������� �������
#define T_sigma_16 147
#define T_sigma_17 143
#define T_sigma_18 140
#define T_sigma_19 137
#define T_sigma_20 133
#define T_sigma_21 130
#define T_sigma_22 127
#define T_sigma_23 123
#define T_sigma_24 120
#define T_sigma_25 117 
#define T_sigma_26 113
#define T_sigma_27 110
#define T_sigma_28 107
#define T_sigma_29 104 
#define T_sigma_30 102


//!!!!!!!!!!!!!!!!!!!!! CRUISE  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define wait_step 1750 //�������� 1 ����

#define C_dead_band_spd 50			// ������� ������ speed �10
#define C_dead_band_acc 25			// ������� ������ ��������� ��/(�*�) �100
#define C_dead_time_regulate 4000	//����� ����� ������������ ��
#define C_time_speed_meas 1000		//����� ��������� ��������

#define C_translate_speed 75000 // ���� ������������� �������� �100 ����� ����� �� 10 �� ��� ���������� �������� ��������

#define C_button_delta_spd 20 // ��������� �������� �������� �� ������
//need?
//#define drive_step_pack 7 // ������� ����� ���� ������� �� ������ 

//�������� ���� +60 �������� - ��� +2 ��/�

#define C_acc_max 200	//�������� ��������� � 2,4 �������� 
#define C_acc_min 100 

#define drive_step_start 35 //����������� ����� ����� ��� ����������� �������
#define drive_step_max 150 //����������� � 60% 
#define drive_step_min 10 //����������� � 60% 
//������������ ��
#define C_k1_1 6
#define C_k1_2 6

#define C_k2_1 40
#define C_k2_2 40
#define C_k2_3 80
#define C_k2_4 80

#define C_k3_1 60
#define C_k3_2 60



#include <Wire.h>
#include "avr/pgmspace.h"
#include <OneWire.h>

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F, 16, 2);  // � ���� ������� ����� 0x3F ��� ������ 0x27

OneWire ds(dio_temp_meas); // ������� ������ OneWire ��� ���� 1-Wire, � ������� �������� ����� �������������� ������ � ��������

byte bt_prev[3]; // ��������� ������

unsigned long bt_timer;		// ������ ������
unsigned long switch_timer; // ����������� on-off
unsigned long bt_tmp;		// ������ ������
unsigned long C_regulate_timer;		//������ �� ������
unsigned long C_speed_time_meas;	//����� ��������� ��������
boolean C_speed_meas;		// ��������� ��������� ��������
unsigned long C_d_time_long_speed;		//��������������� ����� ��������� ��������
unsigned long C_d_time_long_acc;		//��������������� ����� ��������� ���������
unsigned long C_d_time_long_acc_prev;		//��������������� ����� ��������� ���������
long C_V_current_long;		//������� ������� ��������
int C_d_time;		//����� ���������
int C_V_current;	//������� ��������
int C_V_prev;		//���������� ��������
int C_V_target;		//������� ��������
int C_dV;			//��������� ��������
int C_acc;			//���������
long C_acc_long;			//��������� long
boolean C_acc_dir;	//����������� ���������
boolean C_dV_dir;	//��������� ��������� ��������
boolean C_rot_direction; //����������� �������
long C_dX_acc;		//
long step_regulate;	//���������� ����
//boolean  C_fresh_speed_meas;		//������������� ��������� ��������
boolean Cruise_first_start;			//����� �������� ������

int bt_read; // ������ �������������
int bt_refrense; // ������������ ������
int ai_read; //������ ai
int i; // ������� i ��� ������
boolean switch_on_off; //��������� ������������ false - off true-on

boolean cruise; //��������� ������
boolean C_first_start; //������ ������ ������: ����� ���������� ������ ��������
int current_step; //������� ������� �����
unsigned long speed_count; //������� ��������� ��������

//drl
boolean drl_state;
boolean drl_first_start; //������ ������ ���
boolean drl_lights_on; //��������� ���� drl

//climate
boolean T_data_sent;		//�������� ������ setup= false;
unsigned long T_data_get;	//���������� ������� �������� ������
unsigned long T_time_regulate;	//���������� ������� ���������� ����� �������
byte T_transfer[2];		// ����� ��� �������� �����������
int T_current;			//���������� �����������
int T_target;			//������� �����������
int T_target_previous;	//������� ����������� ����������
int T_dT;				//���������� �����������
int T_sigma;			//���� ������� ����� �������
int T_target_read;		//��������� �����������

//������ �������
const char MenuName_0[] PROGMEM = "    /           ";
const char* const MenuNames[] PROGMEM = { MenuName_0};
char menubuffer[17]; // ����� ��� ��������� ������������ ������

int servo_data;

void Draw () {
	lcd.setCursor(0, 0);
	strcpy_P(menubuffer, (char*)pgm_read_word(&(MenuNames[0])));
	lcd.print(menubuffer);
	lcd.setCursor(9, 1);
	strcpy_P(menubuffer, (char*)pgm_read_word(&(MenuNames[0])));
	lcd.print(menubuffer);
	//������� ��������
	lcd.setCursor(0, 0);
	lcd.print(C_V_current);
	lcd.setCursor(5, 0);
	lcd.print(C_V_target);
	lcd.setCursor(11, 0);
	lcd.print(C_acc);
	lcd.setCursor(10, 1);
	lcd.print(T_current);
	lcd.setCursor(14, 1);
	lcd.print(T_target);
}


//����������� ������� ������
byte getPressedButton() { //��������� ����������� ������� ������
						  /*
						  ai_cruise_pin_1
						  resume 202
						  on 500
						  cancel 68
						  off 493
						  speed+  804
						  speed-  938
						  ai_cruise_pin_2		set
						  off 0
						  on 493
						  ai_cruise_pin_4		on/off
						  off 493
						  on 0
						  */
	bt_refrense = (analogRead(ai_5v_refrense) - button_refrense);
	//Serial.println(bt_refrense);
	if (abs(bt_refrense) < 5)
	{
		bt_read = analogRead(ai_cruise_pin_4);
		if (bt_read < button_on_up)
		{
			if (!switch_on_off)
			{
				bt_tmp = millis() - bt_timer;
				if (bt_tmp < button_delay_const)
				{
					return button_off;
				}
			}
			switch_on_off = true;
			switch_timer = millis();
			if (bt_prev[1] != 0)
			{
				bt_prev[1] = 0;
				return button_on;
			}
			if (analogRead(ai_cruise_pin_2) > button_set_down)
			{
				if (bt_prev[0] != 1)
				{
					bt_timer = millis();
					bt_prev[0] = 1;
					return button_on;
				}
				else
				{
					bt_tmp = millis() - bt_timer;
					if (bt_tmp >= button_delay_const)
					{
						bt_prev[0] = 0;
						////Serial.println("set");
						return buttton_set;
					}
					else { return button_on; }
				}
			}
			bt_read = analogRead(ai_cruise_pin_1);
			if (bt_read > button_cancel_down && bt_read < button_cancel_up)
			{
				bt_prev[1] = 1;
				bt_tmp = millis() - bt_timer;
				if (bt_tmp >= button_delay_const)
				{
					bt_timer = millis();
					//Serial.println("cancel");
					return button_cancel;
				}
				else
				{
					return button_on;
				}
			}
			if (bt_read > button_resume_down && bt_read < button_resume_up)
			{
				bt_prev[1] = 1;
				bt_tmp = millis() - bt_timer;
				if (bt_tmp >= button_delay_const)
				{
					bt_timer = millis();
					//Serial.println("resume");
					return button_resume;
				}
				else
				{
					return button_on;
				}
			}
			if (bt_read > button_SpeedUP_down && bt_read < button_SpeedUP_up)
			{
				bt_prev[1] = 1;
				bt_tmp = millis() - bt_timer;
				if (bt_tmp >= button_delay_const)
				{
					bt_timer = millis();
					//Serial.println("Spd++");
					return button_speed_up;
				}
				else
				{
					return button_on;
				}
			}
			if (bt_read > button_SpeedDOWN_down && bt_read < button_SpeedDOWN_up)
			{
				bt_prev[1] = 1;
				bt_tmp = millis() - bt_timer;
				if (bt_tmp >= button_delay_const)
				{
					bt_timer = millis();
					//Serial.println("Spd--");
					return button_speed_down;
				}
				else
				{
					return button_on;
				}
			}
			return button_on;
		}
		else if (bt_read > button_on_up && bt_read < button_off_down)
		{
			if (switch_on_off)
			{
				return button_on;
			}
			else
			{
				return button_off;
			}
		}
		else
		{
			if (switch_on_off)
			{
				bt_tmp = millis() - bt_timer;
				if (bt_tmp < button_delay_const)
				{
					return button_on;
				}
			}
			switch_on_off = false;
			switch_timer = millis();
			////Serial.println("off");
			// ��������� drl= off+ 5 ���. set
			if (analogRead(ai_cruise_pin_2) > button_set_down)
			{
				if (bt_prev[0] != 1)
				{
					bt_timer = millis();
					bt_prev[0] = 1;
					////Serial.println("off");
					return button_off;
				}
				else
				{
					bt_tmp = millis() - bt_timer;
					if (bt_tmp >= button_delay_const)
					{
						bt_prev[0] = 0;
						return button_drl;
					}
					else {
						////Serial.println("off"); 
						return button_off;
					}
				}
			}
			else
			{
				////Serial.println("off");
				return button_off;
			}
		}
	}
}

//��������� ������� �����
void stepeers(boolean dir_drive, byte current_steps)
{
	digitalWrite(drive_dir, dir_drive ? LOW : HIGH); //�������� �����������
	//�� � ������ ������ ����� �����
	for (i = 0; i < current_steps; i++)
	{
		digitalWrite(drive_step, HIGH);
		delayMicroseconds(wait_step);
		digitalWrite(drive_step, LOW);
		delayMicroseconds(wait_step);
	}
	if (dir_drive) { current_step += current_steps; }
	else { current_step -= current_steps; }
	//�������� ��������� ��
	C_regulate_timer = millis() + C_dead_time_regulate;
}

//��������� DRL �� ������
void switch_drl() {
	drl_state = !drl_state;
	if (drl_state)
	{
		if (digitalRead(di_lamp))  //�������� ���������
		{
			analogWrite(drl_lamp, drl_power);
			drl_lights_on = true;
		}
		else
		{
			digitalWrite(drl_lamp, LOW);
			drl_lights_on = false;
		}
	}
	else
	{
		digitalWrite(drl_lamp, LOW);
		drl_lights_on = false;
	}
}

//Interrupt ��������� ����������� drl
void drl() {
	if (drl_state)
	{
		if (digitalRead(di_lamp))  //�������� ���������
		{
			analogWrite(drl_lamp, drl_power);
			drl_lights_on = true;
		}
		else
		{
			digitalWrite(drl_lamp, LOW);
			drl_lights_on = false;
		}
	}
}

//Interrupt ��������� ����������� stop
void cruise_break() {
	if (digitalRead(drive_sleep))
	{
		digitalWrite(drive_sleep, LOW);
	}
	if (!digitalRead(drive_enable))
	{
		digitalWrite(drive_enable, HIGH);
	}
	//�������� ��������� �� ������ ������
	if (cruise)
	{
		cruise = false;
	}
}

//Interrupt ��������� �������� � ������ ����
void Speed_interrupt() {
	speed_count++;
	if (cruise)
	{
		if (!digitalRead(di_stop))
		{
			cruise_break();
		}
	}
}

//��������� ����������
void temperature_meas_regulate() {
	if (!T_data_sent)
	{
		ds.reset(); // �������� �������������� �� ������ ���� ���������� ������ � ����������
		ds.write(0xCC); // ���� ������� DS18b20 ������� ���������� ����� �� ������. � ����� ������ ������ ���� ����������
		ds.write(0x44); // ���� ������� DS18b20 ������� �������� �����������. ���� �������� ����������� �� ��� �� �������� - ������ ��� ������� �� ���������� ������
		T_data_get = millis() + T_dead_time_meas;
		T_data_sent = true;
	}
	if (T_data_sent && (millis()>T_data_get))
	{
		T_transfer[0] = ds.read(); // ������ ������� ���� �������� �����������
		T_transfer[1] = ds.read(); // � ������ �������
		// ��������� �������� ��������:
		//    - ������ "���������" ��������,
		//    - ����� �������� ��� �� �����������, �������������� ����������� ����������� (��� 12 ��� �� ��������� - ��� 0,0625)
		// ����������� �������� �� 10(!)
		T_current = ((T_transfer[1] << 8) | T_transfer[0]);
		//Serial.println(T_current);
		T_dT = T_current - T_target;
		if (abs(T_dT) > T_dead_temp)
		{
			if (abs(T_dT) <= T_zone_1) {
				T_sigma = T_sigma + T_dT * T_Ktemp;
			}
			else if (abs(T_dT) <= T_zone_2)
			{
				T_sigma = T_sigma + T_dT * 2 * T_Ktemp;
			}
			else
			{
				T_sigma = T_sigma + T_dT * 3 * T_Ktemp;
			}
			if (T_sigma >= T_sigma_max)
			{
				T_sigma = T_sigma_max;
				digitalWrite(led_red, HIGH);
			}
			else if (T_sigma <= T_sigma_min)
			{
				T_sigma = T_sigma_min;
				digitalWrite(led_green, HIGH);
			}
			else if (digitalRead(led_green) || digitalRead(led_red))
			{
				digitalWrite(led_green, LOW);
				digitalWrite(led_red, LOW);
			}
			analogWrite(do_servo, T_sigma);
			T_time_regulate = millis() + T_dead_time_regulate;
		}
		T_data_sent = false;
	}
}

//���������� ��������� ����� �������������
int get_temperature_target() {
	//bt_refrense = (analogRead(ai_5v_refrense) - button_refrense) * 100 / button_refrense;
	T_target_read = analogRead(ai_temp_R);// *bt_refrense / 100;
	if (T_target_read <= T_R_max)
		{  	 return T_target_max; }
	else if (T_target_read <= T_R_30)
		{		  return T_target_30; }
	else if (T_target_read <= T_R_29)
		{		  return T_target_29; }
	else if (T_target_read <= T_R_28)
		{		  return T_target_28; }
	else if (T_target_read <= T_R_27)
		{		  return T_target_27; }
	else if (T_target_read <= T_R_26)
		{		  return T_target_26; }
	else if (T_target_read <= T_R_25)
		{		  return T_target_25; }
	else if (T_target_read <= T_R_24)
		{		  return T_target_24; }
	else if (T_target_read <= T_R_23)
		{		  return T_target_23; }
	else if (T_target_read <= T_R_22)
		{		  return T_target_22; }
	else if (T_target_read <= T_R_21)
		{		  return T_target_21; }
	else if (T_target_read <= T_R_20)
		{		  return T_target_20; }
	else if (T_target_read <= T_R_19)
		{		  return T_target_19; }
	else if (T_target_read <= T_R_18)
		{		  return T_target_18; }
	else if (T_target_read <= T_R_17)
		{		  return T_target_17; }
	else if (T_target_read <= T_R_16)
		{		  return T_target_16; }
	if (T_target_read > T_R_16)
		{  	 return T_target_min; }
}
// ���� ������ ��������
void climate_control() {
	T_target_previous = T_target;
	T_target = get_temperature_target();
	//Serial.println(T_target);
	if (T_target == T_target_max || T_target == T_target_min)
	{
		if (T_sigma != T_sigma_max || T_sigma != T_sigma_min)
		{
			if (T_target == T_target_max)
			{
				T_sigma = T_sigma_min;
			}
			if (T_target == T_target_min)
			{
				T_sigma = T_sigma_max;
			}
			analogWrite(do_servo, T_sigma);
		}
		if (digitalRead(led_green) || digitalRead(led_red))
		{
			digitalWrite(led_green, LOW);
			digitalWrite(led_red, LOW);
		}
	}
	else
	{
		if (T_target == T_target_previous)
		{
			if (millis()>T_time_regulate)
			{
				temperature_meas_regulate();
			}
		}
		else
		{
			temperature_meas_regulate();
		}
	}
}

void climate_control_manual() {
	T_target_previous = T_target;
	T_target = get_temperature_target();
	if (T_target != T_target_previous)
	{
		if (T_target == T_target_min)
			{
				T_sigma = T_sigma_min;
			}
		if (T_target == T_target_max)
		{
			T_sigma = T_sigma_max;
		}
		if (T_target == T_target_16)
		{
			T_sigma = T_sigma_16;
		}
		if (T_target == T_target_17)
		{
			T_sigma = T_sigma_17;
		}
		if (T_target == T_target_18)
		{
			T_sigma = T_sigma_18;
		}
		if (T_target == T_target_19)
		{
			T_sigma = T_sigma_19;
		}
		if (T_target == T_target_20)
		{
			T_sigma = T_sigma_20;
		}
		if (T_target == T_target_21)
		{
			T_sigma = T_sigma_21;
		}
		if (T_target == T_target_22)
		{
			T_sigma = T_sigma_22;
		}
		if (T_target == T_target_23)
		{
			T_sigma = T_sigma_23;
		}
		if (T_target == T_target_24)
		{
			T_sigma = T_sigma_24;
		}
		if (T_target == T_target_25)
		{
			T_sigma = T_sigma_25;
		}
		if (T_target == T_target_26)
		{
			T_sigma = T_sigma_26;
		}
		if (T_target == T_target_27)
		{
			T_sigma = T_sigma_27;
		}
		if (T_target == T_target_28)
		{
			T_sigma = T_sigma_28;
		}
		if (T_target == T_target_29)
		{
			T_sigma = T_sigma_29;
		}
		if (T_target == T_target_30)
		{
			T_sigma = T_sigma_30;
		}
		//analogWrite(do_servo, T_sigma);
	}
}

void Speed_measument() {
	if (millis() > C_speed_time_meas)
	{
		if (!C_speed_meas)
		{
			speed_count = 0;
			C_speed_time_meas = millis() + C_time_speed_meas;
			C_speed_meas = true;
		}
		else
		{
			C_d_time_long_acc = millis();
			C_d_time_long_speed = C_d_time_long_acc - C_speed_time_meas + C_time_speed_meas;
			C_V_current_long = speed_count * C_translate_speed / C_d_time_long_speed;
			C_V_current = C_V_current_long;
			C_dV = C_V_current - C_V_target; //���������� �� ��������
			if (C_V_current >= C_V_target)
			{
				C_dV_dir = true;
			}
			else
			{
				C_dV_dir = false;
			}
			//������ ���������
			C_dX_acc = (C_V_current - C_V_prev);
			if (C_dX_acc >= 0)
			{
				C_acc_dir = true;
			}
			else
			{
				C_acc_dir = false;
			}
			C_acc_long = abs(C_dX_acc) * 1000/ (C_d_time_long_acc - C_d_time_long_acc_prev);
			C_acc = C_acc_long;

			C_d_time_long_acc_prev = C_d_time_long_acc;
			//C_dX_acc = (C_V_current - C_V_prev);
			C_V_prev = C_V_current;
			C_V_current /= 10; //�������� ����� �� 10 ���� ��������� ����
			C_speed_meas = false;
			Draw();
		}
	}
}

void cruise_control() {
	if (millis() > C_regulate_timer)
	{
		C_acc = abs(C_acc);
		C_dV = abs(C_dV);
		step_regulate = 0;
		if (C_dV < C_dead_band_spd && C_acc > C_dead_band_acc)
		{
			if ((C_dV_dir && C_acc_dir) || (!C_dV_dir && !C_acc_dir))
			{
				if (C_dV_dir > 0)
				{
					step_regulate = C_acc / C_k1_1;
					C_rot_direction = false;
					//!!!!!!!!!!!!!!!!!!!!!
					lcd.setCursor(0, 1);
					lcd.print("k1_1");
					Serial.println("r1 k1_1");
					Serial.print("regulate= ");
					Serial.println(step_regulate);

				}
				else
				{
					step_regulate = C_acc / C_k1_2;
					C_rot_direction = true;
					//!!!!!!!!!!!!!!!!!!!!!
					lcd.setCursor(0, 1);
					lcd.print("k1_2");
					Serial.println("r2 k1_2");
					Serial.print("regulate= ");
					Serial.println(step_regulate);
				}
			}
			else
			{
				if (C_dV_dir)
				{
					step_regulate = C_acc / C_k1_1;
					C_rot_direction = false;
					//!!!!!!!!!!!!!!!!!!!!!
					lcd.setCursor(0, 1);
					lcd.print("k1_1");
					Serial.print("r3 k1_1");
					Serial.print("regulate= ");
					Serial.println(step_regulate);
				}
				else
				{
					step_regulate = C_acc / C_k1_2;
					C_rot_direction = true;
					//!!!!!!!!!!!!!!!!!!!!!
					lcd.setCursor(0, 1);
					lcd.print("k1_2");
					Serial.print("r4 k1_2");
					Serial.print("regulate= ");
					Serial.println(step_regulate);
				}
			}
		}
		else if (C_dV > C_dead_band_spd && C_acc > C_dead_band_acc)
		{
			if ((C_dV_dir && C_acc_dir) || (!C_dV_dir && !C_acc_dir))
			{
				if (C_dV_dir)
				{
					step_regulate = C_dV / C_k2_1;
					C_rot_direction = false;
					//!!!!!!!!!!!!!!!!!!!!!
					lcd.setCursor(0, 1);
					lcd.print("k2_1");
					Serial.print("r5 k2_1");
					Serial.print("regulate= ");
					Serial.println(step_regulate);
				}
				else
				{
					step_regulate = C_dV / C_k2_2;
					C_rot_direction = true;
					//!!!!!!!!!!!!!!!!!!!!!
					lcd.setCursor(0, 1);
					lcd.print("k2_2");
					Serial.println("r6 k2_3");
					Serial.print("regulate= ");
					Serial.println(step_regulate);
				}
			}
			else
			{
				if (C_acc < C_acc_min)
				{
					if (C_dV_dir)
					{
						step_regulate = C_dV / C_k2_3;
						C_rot_direction = false;
						//!!!!!!!!!!!!!!!!!!!!!
						lcd.setCursor(0, 1);
						lcd.print("k2_3");
						Serial.print("r7 k2_3");
						Serial.print("regulate= ");
						Serial.println(step_regulate);
					}
					else
					{
						step_regulate = C_dV / C_k2_4;
						C_rot_direction = true;
						//!!!!!!!!!!!!!!!!!!!!!
						lcd.setCursor(0, 1);
						lcd.print("k2_4");
						Serial.print("r8 k2_4");
						Serial.print("regulate= ");
						Serial.println(step_regulate);
					}
				}
				else if (C_acc > C_acc_max)
				{
					if (C_dV_dir)
					{
						step_regulate = C_dV / C_k2_3;
						C_rot_direction = true;
						//!!!!!!!!!!!!!!!!!!!!!
						lcd.setCursor(0, 1);
						lcd.print("k2_3");
						Serial.print("r9 k2_3");
						Serial.print("regulate= ");
						Serial.println(step_regulate);
					}
					else
					{
						step_regulate = C_dV / C_k2_4;
						C_rot_direction = false;
						//!!!!!!!!!!!!!!!!!!!!!
						lcd.setCursor(0, 1);
						lcd.print("k2_4");
						Serial.print("r10 k2_4");
						Serial.print("regulate= ");
						Serial.println(step_regulate);
					}
				}
			}
		}
		else if (C_dV > C_dead_band_spd && C_acc < C_dead_band_acc)
		{
			if (C_dV_dir)
			{
				step_regulate = C_dV / C_k3_1;
				C_rot_direction = false;
				//!!!!!!!!!!!!!!!!!!!!!
				lcd.setCursor(0, 1);
				lcd.print("k3_1");
				Serial.print("r11 k3_1");
				Serial.print("regulate= ");
				Serial.println(step_regulate);
			}
			else
			{
				step_regulate = C_dV / C_k3_2;
				C_rot_direction = true;
				//!!!!!!!!!!!!!!!!!!!!!
				lcd.setCursor(0, 1);
				lcd.print("k3_2");
				Serial.print("r12 k3_2");
				Serial.print("regulate= ");
				Serial.println(step_regulate);
			}
		}
		if (step_regulate != 0)
		{
			if (C_rot_direction)
			{
				if ((current_step + step_regulate) > drive_step_max)
				{
					step_regulate = drive_step_max - current_step;
				}
				stepeers(true, step_regulate);
			}
			else
			{
				if ((current_step - step_regulate) < drive_step_min)
				{
					step_regulate = current_step - drive_step_min;
				}
				stepeers(false, step_regulate);
			}
		}
	}

}

void drl_control() {
	if (drl_first_start)
	{
		if (speed_count > drl_first_start_zone)
		{
			switch_drl();
			drl_first_start = false;
		}
		
	}
	if (drl_state && !drl_first_start)
	{
		if (digitalRead(di_lamp))  //�������� ���������
		{
			if (!drl_lights_on)
			{
				analogWrite(drl_lamp, drl_power);
				drl_lights_on = true;
			}			
		}
		else
		{
			if (drl_lights_on)
			{
				digitalWrite(drl_lamp, LOW);
				drl_lights_on = false;
			}
		}
	}
}

void setup() {
	//drl
	//������� Timer1 �� 60 ��
	//TCCR1A = TCCR1A & 0xe0 | 1; //����������� ���� 1 �������(������ 9,10)
	//TCCR1B = TCCR1B & 0xe0 | 0x0d; //������� ������� 1 ������� (������ 9,10) nano, 11,12 mega
	//
	TCCR1A = TCCR1A & 0xe0 | 3;
	TCCR1B = TCCR1B & 0xe0 | 0x0c;

	drl_state = false;

	attachInterrupt(digitalPinToInterrupt(di_lamp), drl, CHANGE);
	attachInterrupt(digitalPinToInterrupt(di_speed), Speed_interrupt, FALLING); // ��� FALLING
	//������
	pinMode(led_red, OUTPUT);
	pinMode(led_green, OUTPUT);
	pinMode(drive_dir, OUTPUT);
	pinMode(drive_enable, OUTPUT);
	pinMode(drive_step, OUTPUT);
	pinMode(drive_sleep, OUTPUT);
	pinMode(drl_lamp, OUTPUT);
	pinMode(do_servo, OUTPUT);


	// ������� ������������� ����������
	
	current_step = drive_step_start;
	cruise = false;
	switch_on_off = false;
	C_speed_meas = false;
	speed_count = 0;
	C_regulate_timer = 0;
	C_speed_time_meas = 0;
	//������������� ������� ������ 100 ��/�
	C_V_target = 1000;
	Cruise_first_start = true;
	C_d_time_long_acc_prev = 0;
		
	//������������� ���������� �����������
	T_target = T_target_min;
	//�������� ����� � ���������� �����������
	T_sigma = T_sigma_max;
	analogWrite(do_servo, T_sigma);
	T_time_regulate = 0;
	//Drl
	drl_first_start = true;
	drl_state = false;
	drl_lights_on = false;
	digitalWrite(drl_lamp, LOW);

	//�������
	Serial.begin(9600);
	lcd.begin();
	lcd.setCursor(0, 0);
	lcd.print("hello");
}

void loop() {
	
	switch (getPressedButton())
	{
	case (button_off):
		//�������� ����������� ��������� ��
		if (!digitalRead(drive_enable))
		{
			digitalWrite(drive_enable, HIGH);
		}
		if (digitalRead(drive_sleep))
		{
			digitalWrite(drive_sleep, LOW);
		}
		//�������� ��������� �� ������ ������
		if (cruise)
		{
			cruise = false;
		}
		break;
	case (button_on):
		if (cruise)
		{
			cruise_control();
		}
		break;
	case(button_speed_up):
		C_V_target += C_button_delta_spd;
		C_regulate_timer = millis();
		break;
	case (button_speed_down):
		C_V_target -= C_button_delta_spd;
		C_regulate_timer = millis();
		break;
	case (button_resume):
		//��� ������� ����� � �������
		if (!cruise)
		{
			cruise = true;
			if (Cruise_first_start)
			{
				C_V_target = C_V_current;
				Cruise_first_start = false;
			}
			if (!digitalRead(drive_sleep))
			{
				digitalWrite(drive_sleep, HIGH);
			}
			if (digitalRead(drive_enable))
			{
				digitalWrite(drive_enable, LOW);
			}
			stepeers(true, current_step);
			current_step = current_step / 2;
			C_regulate_timer = millis();
		}
		break;
	case (button_cancel):
		cruise = false;
		digitalWrite(drive_enable, HIGH);
		break;
	case (button_drl):
		if (drl_first_start)
		{
			drl_first_start = false;
		}
		switch_drl();
		break;
	case (buttton_set):
		C_V_target = C_V_current;
		Cruise_first_start = false;
		break;
	}
	
	//�������� ��������  "����"
	Speed_measument();
	//drl
	drl_control();
	
	delay(10);
	
}

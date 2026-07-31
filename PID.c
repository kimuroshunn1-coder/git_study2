void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (&htim6 == htim) {
		float32_t TxData_f32[3] = {};
		TxData_f32[0] = (float32_t)state;
		TxData_f32[1] = (float32_t)sub_state;
		CAN(STATE_CANID_UP, TxData_f32);

		uint8_t set_flag;
		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(SET_FLAG_GPIO_Port, SET_FLAG_Pin)) set_flag = 1;
		else set_flag = 0;
		float32_t TxData_flag[3] = {};
		TxData_flag[0] = (float)set_flag;
		CAN(SOTEN_SWITCH, TxData_flag);
	}
	else if (&htim7 == htim) {
		float k_p = 7, k_i = 0.5, k_d = 0.01;
		for (int i = 0; i < 4; i++){
			robomas[i].hensa = robomas[i].trgVel - robomas[i].actVel;
			if (robomas[i].hensa >= 1000) robomas[i].hensa = 1000;
			else if (robomas[i].hensa <= -1000) robomas[i].hensa = -1000;
			float d = (robomas[i].actVel - robomas[i].p_actVel) / 0.001;
			robomas[i].ind += robomas[i].hensa*0.1;
			if (d >= 30000) d = 30000;
			else if (d <= -30000) d = -30000;
			if (robomas[i].ind >= 10000) robomas[i].ind = 10000;
			else if (robomas[i].ind <= -10000) robomas[i].ind = -10000;


			float t = k_p*robomas[i].hensa;
			if (t>=10000) t = 10000;
			else if (t<=-10000) t = -10000;
			robomas[i].cu = (int16_t)(t+k_i*robomas[i].ind+k_d*d);
			if (robomas[i].cu <= -10000) robomas[i].cu = -10000;
			else if (robomas[i].cu >= 10000) robomas[i].cu = 10000;

			robomas[i].p_actVel = robomas[i].actVel;

			//robomas[i].cu = 0;
		}
		CAN_robomas(robomas);
	}
	else if (&htim16 == htim) {
	    for (int i = 0; i < 2; i++) {
	    	if (state == MOVE_STATE) {
				robomas[i].motor_spd = (float)robomas[i].actVel / 60.0;

				robomas[i].diff_pro=(float)(robomas[i].actangle - robomas[i].p_actangle);

				float diff;
				if(robomas[i].actVel >=10000){
					diff = (robomas[i].diff_pro < 0 ? robomas[i].diff_pro + 8192 : robomas[i].diff_pro);
				}else if(robomas[i].actVel <= -10000){
					diff = (robomas[i].diff_pro > 0 ? robomas[i].diff_pro - 8192 : robomas[i].diff_pro);
				}else if( robomas[i].diff_pro < 5000 && robomas[i].diff_pro > -5000){
					diff = robomas[i].diff_pro;
				}else if (robomas[i].diff_pro > 0){
					diff = robomas[i].diff_pro - 8192;
				}else
					diff = robomas[i].diff_pro + 8192;

				robomas[i].p_actangle = robomas[i].actangle;

				float dt = 0.001;
				robomas[i].motor_pos += diff/8192.0;
				robomas[i].motor_spd = diff / 8192.0 / dt;

				float p_gain = (robomas[i].motor_pos_ref - robomas[i].motor_pos) * kp;
				robomas[i].pos_err = robomas[i].motor_pos_ref - robomas[i].motor_pos;
				if(robomas[i].pos_err>10||robomas[i].pos_err<-10){
					robomas[i].pos_err = 0;
				}
				robomas[i].sum_pos_err += robomas[i].pos_err * dt;
				robomas[i].last_pos_err = robomas[i].pos_err;
				if(robomas[i].sum_pos_err >= max_sum_pos_err){
					robomas[i].sum_pos_err = max_sum_pos_err;
				}
				else if(robomas[i].sum_pos_err <= -max_sum_pos_err){
					robomas[i].sum_pos_err = -max_sum_pos_err;
				}
				float i_gain = robomas[i].sum_pos_err * ki;

				float d_gain = robomas[i].motor_spd * kd;

				robomas[i].trgVel = (p_gain + i_gain + d_gain);/*
				if (robomas[i].trgVel > 100*36) robomas[i].trgVel = 100*36;
				if (robomas[i].trgVel > -100*36) robomas[i].trgVel = -100*36;*/
	    	}
	    }
	}
	else if (&htim17 == htim) {
		if (MOVE_STATE == state && ACTIVE_STATE == sub_state) {
			sub_state = SUSPEND_STATE;
		}
		else if (MOVE_STATE == state && SUSPEND_STATE == sub_state) {
			state = ERROR_STATE;
			sub_state = ERROR_STATE;
		}
	}
	else {
		__NOP();
	}
}

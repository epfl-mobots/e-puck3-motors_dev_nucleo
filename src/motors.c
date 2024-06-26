/**
 * @file	motors.c
 * @brief  	Library to control motors
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.07.2019
 */
 
#include "ch.h"
#include "hal.h"
#include "motors.h"
#include "main.h"

#define ADC3_BUFFER_DEPTH				2		//2 sequences of MAX_NB_OF_BRUSHLESS_MOTOR samples
#define ADC1_BUFFER_DEPTH				2		//2 sequences of MAX_NB_OF_BRUSHLESS_MOTOR samples
#define ADC1_NB_ELEMENT_SEQ				3		/* with 52KHz, we can do approx 24 measurements by PWM cycle 
												so we need to do 2 sequences of 12 elements (3 * 4 motors)*/
#define ADC3_OFF_SAMPLE_TIME			0.20f	//we sample the OFF time at 20% of the PWM cycle
#define ADC3_ON_SAMPLE_TIME				0.75f	//we sample the ON time at 75% of the PWM cycle
#define ZC_DETECT_METHOD_THESHOLD		40		//we use the ZC_DETECT_ON method above 40% duty cycle

#define NB_SAMPLE_OFFSET_CALIBRATION	1000

#define DEGAUSS_TICKS_ZC_OFF			1
#define HALF_BUS_VOLTAGE				962

#define PERIOD_PWM_52_KHZ_APB2  		4154	// STM32_TIMCLK2/52000 rounded to an even number to be divisible by 2
#define PERIOD_PWM_52_KHZ_APB1 			PERIOD_PWM_52_KHZ_APB2/2

#define RAMP_STEPS_DUTY_CYCLE 			0.001f

#define LP_FILTER_COEFF_CURRENT			0.0001f
#define LP_FILTER_COEFF_RPM 			0.1f
#define LP_FILTER_COEFF_ZC				0.4f

#define TICKS_52_KHZ_TO_100HZ 			520		//number of tick at 52kHz to achieve 100Hz
#define RP10MS_TO_RPM 					6000	//rounds per 10 milisecond to rounds per minute


/**
 * NB of steps for a 6-steps commutation scheme 
 */
#define NB_STEPS_BRUSHLESS				6

/**
 * Possible phases for a brushless motor
 */
typedef enum{
    PHASE1 = 0,
    PHASE2,
    PHASE3,
    NB_BRUSHLESS_PHASES
}brushless_phases_t;

/**
 * Possible lines states
 */
typedef enum{
	OUT_PWM = 0,
	OUT_LOW,
	OUT_HIGH,
}timer_output_states_t;

/**
 * Possible bemf slopes
 */
typedef enum{
	BEMF_NEGATIVE = 0,
	BEMF_POSITIVE
}bemf_slope_list_t;

/**
 * Zero crossing detection methods
 */
typedef enum{
	ZC_DETECT_OFF = 0,	/* Function to detect the zero crossing with the PWM OFF voltage */
	ZC_DETECT_ON,		/* Function to detect the zero crossing with the PWM ON voltage */
	ZC_CALIBRATE_OFF, 	/* Function to calibrate the ADC offset for the PWM OFF voltage */
	NB_ZC_METHODS
}zc_det_methods_t;

/**
 * Half Bridges list
 */
typedef enum{
    HALF_BRIDGE_1 = 0,
    HALF_BRIDGE_2,
    HALF_BRIDGE_3,
    HALF_BRIDGE_4,
    HALF_BRIDGE_5,
    HALF_BRIDGE_6,
    HALF_BRIDGE_7,
    HALF_BRIDGE_8,
    HALF_BRIDGE_9,
    HALF_BRIDGE_10,
    HALF_BRIDGE_11,
    HALF_BRIDGE_12,
}half_bridges_names_t;

/**
 * Motor states
 */
typedef enum{
	RUNNING = 0,		/* In this mode, the motor uses the commutation table configured in motors_conf.h */
	FREE_WHELLING,		/* In this mode, each lines of the motor are kept in floating state */
	TIED_TO_GROUND,		/* In this mode, each lines of the motor are tied to ground */
	NB_MOTOR_STATES
}motor_states_t;

/**
 *  ADC external triggers list for STM32F746
 */
typedef enum{
	TRG_TIMER1_CH1 = 0,
	TRG_TIMER1_CH2,
	TRG_TIMER1_CH3,
	TRG_TIMER2_CH2,
	TRG_TIMER5_TRGO,
	TRG_TIMER4_CH4,
	TRG_TIMER3_CH4,
	TRG_TIMER8_TRGO,
	TRG_TIMER8_TRGO2,
	TRG_TIMER1_TRGO,
	TRG_TIMER1_TRGO2,
	TRG_TIMER2_TRGO,
	TRG_TIMER4_TRGO,
	TRG_TIMER6_TRGO,
	TRG_RESERVED,
	TRG_EXTI_LINE11,
}adc_triggers_list_t;

/**
 *  Timer triggers list for STM32F746
 */
typedef enum{
	TS_ITR0 = 0,
	TS_ITR1,
	TS_ITR2,
	TS_ITR3,
	TS_T1F_ED,
	TS_TI1FP1,
	TS_TI2FP2,
	TS_ETRF
}tim_trigger_selection_list_t;

/**
 *  Timer salve mode list for STM32F746
 */
typedef enum{
	SMS_DISABLED = 0,
	SMS_ENC_MODE_1,
	SMS_ENC_MODE_2,
	SMS_ENC_MODE_3,
	SMS_RESET_MODE,
	SMS_GATED_MODE,
	SMS_TRIGGER_MODE,
	SMS_EXTERNAL_CLOCK_MODE,
	SMS_COMBINED_RESET_TRIGGER

}tim_slave_mode_selection_list_t;

/**
 *  Timer master mode 1 list for STM32F746
 */
typedef enum{
	MMS_RESET = 0,
	MMS_ENABLE,
	MMS_UPDATE,
	MMS_COMPARE_PULSE,
	MMS_COMPARE_OC1REF,
	MMS_COMPARE_OC2REF,
	MMS_COMPARE_OC3REF,
	MMS_COMPARE_OC4REF
}tim_mms_modes_t;

/**
 *  Timer master mode 2 list for STM32F746
 */
typedef enum{
	MMS2_RESET = 0,
	MMS2_ENABLE,
	MMS2_UPDATE,
	MMS2_COMPARE_PULSE,
	MMS2_COMPARE_OC1REF,
	MMS2_COMPARE_OC2REF,
	MMS2_COMPARE_OC3REF,
	MMS2_COMPARE_OC4REF,
	MMS2_COMPARE_OC5REF,
	MMS2_COMPARE_OC6REF,
	MMS2_COMPARE_PULSE_OC4REF_RISING_FALLING,
	MMS2_COMPARE_PULSE_OC6REF_RISING_FALLING,
	MMS2_COMPARE_PULSE_OC4REF_RISING_OC6REF_RISING,
	MMS2_COMPARE_PULSE_OC4REF_RISING_OC6REF_FALLING,
	MMS2_COMPARE_PULSE_OC5REF_RISING_OC6REF_RISING,
	MMS2_COMPARE_PULSE_OC5REF_RISING_OC6REF_FALLING
}tim_mms2_modes_t;

/**
 *  Timer mode list for STM32F746
 */
typedef enum{
	OC_FROZEN = 0,
	OC_FORCED_MATCH_HIGH,
	OC_FORCED_MATCH_LOW,
	OC_TOGGLE,
	OC_FORCE_REF_HIGH,
	OC_FORCE_REF_LOW,
	OC_PWM_MODE_1,
	OC_PWM_MODE_2,
	OC_RETR_OPM_1,
	OC_RETR_OPM_2,
	OC_RESERVED_1,
	OC_RESERVED_2,
	OC_COMB_PWM_1,
	OC_COMB_PWM_2,
	OC_ASYM_PWM_1,
	OC_ASYM_PWM_2
}tim_oc_modes_t;

/**
 * Structure representing a commutation scheme row
 */
typedef struct {
	timer_output_states_t		phase1_p;
	timer_output_states_t		phase1_n;
	timer_output_states_t		phase2_p;
	timer_output_states_t		phase2_n;
	timer_output_states_t		phase3_p;
	timer_output_states_t		phase3_n;
	brushless_phases_t			floating_phase;
	brushless_phases_t			low_side_conducting_phase;
	bemf_slope_list_t			bemf_slope;
} brushless_commutation_scheme_line_t;

/**
 * Coefficients for a linear approximation function
 * y = ax + b
 */
typedef struct {
  float   a;
  float   b;
} current_approx_coeff_t;


/**
 * Zero Crossing variables
 */
typedef struct{
    bool 				flag;						/* if set, indicates a zero-crossing event is pending */
    uint32_t			time;						/* time counter counting the PWM cycles */
    uint32_t 			detection_time;				/* zero-crossing event detection time */
    uint32_t			previous_detection_time;	/* previous zero-crossing event detection time */
    uint32_t			period;						/* time between the two last zero-crossing events */
    uint32_t			period_filtered;			/* filtered time between the two last zero-crossing events */
    uint32_t			advance_timing;				/* variable telling which advance time to use */
    uint32_t			next_commutation_time;		/* time at which the next commutation should occur */
    uint32_t			ticks_since_last_comm;		/* nb of PWM cycles since the last commutation */
    uint32_t 			nb_commutations;			/* total nb of commutations */
    zc_det_methods_t 	zc_method;					/* used to set which zc_method to call */
    uint16_t 			dataOn;						/* ADC value of the PWM ON time */
    uint16_t			dataOff;					/* ADC value of the PWM OFF time */
    uint16_t 			previous_dataOn;			/* previous ADC value of the PWM ON time */
    uint16_t 			previous_dataOff;			/* previous ADC value of the PWM OFF time */
}zero_crossing_t;

/**
 * RPM counter variables
 */
typedef struct{
	uint32_t nb_comms;								/* total nb of commutations */
	uint32_t previous_nb_comms;						/* previous total nb of commutations */
	uint32_t count;									/* nb of PWM cycles between nb_comms and previous_nb_comms */
	uint32_t rpm;									/* the filtered actual RPM */
}rpm_counter_t;

/**
 * current meter variables
 */
typedef struct{
	float low_pass_1;								/* variable to store the first low pass filter pass */
	float current;									/* actual filtered current */
}current_meter_t;

/**
 * Structure representing a physical half_bridge.
 */
typedef struct {
	ioline_t 		p_control_line;					
	ioline_t 		n_control_line;
	PWMDriver*		pwmp;
	tim_channel_t	PWM_p_channel;
	tim_channel_t	PWM_n_channel;
	uint8_t			ADC3FloatingMeasureChannel;
	uint8_t			ADC1ConductingMeasureChannel;
} half_bridge_t;

/**
 * Structure representing a brushless motor
 */
typedef struct {
	const half_bridge_t*		phases[NB_BRUSHLESS_PHASES];
	const commutation_schemes_t	commutation_scheme;
	/* represents the number of step necessary to do one complete turn. Specific to each motor */
	uint8_t						nb_of_poles;	
	rpm_counter_t 				rpm_counter;
	current_meter_t 			current_meter;
	int8_t 						step_iterator;		/* 6 steps iterator */
	float						duty_cycle_now;		/* actual duty cycle */
	float 						duty_cycle_goal;	/* desired duty cycle */
	float 						ramp_steps;			/* increment used for the duty cycle */
	rotation_dir_t				direction;			/* direction of rotation of the motor */
	motor_states_t				state;				/* motor's state */
	zero_crossing_t				zero_crossing;		
	/* array containing the ADC offset for the PWM OFF time */
	uint32_t					ADC_offset_off[NB_BRUSHLESS_PHASES];
	uint16_t					nb_offset_sample;	/* nb of samples taken to compute the offset */
} brushless_motor_t;

/**
 * Function pointer for zero crossing functions
 */
typedef bool (*zc_function_t)(brushless_motor_t *motor);

/********************         PRIVATE FUCNTION DECLARATIONS         ********************/
void _zero_crossing_reset(brushless_motor_t *motor);
bool _zero_crossing_detect_off(brushless_motor_t *motor);
bool _zero_crossing_detect_on(brushless_motor_t *motor);
void _zero_crossing_cb(brushless_motor_t *motor);
bool _zero_crossing_calibration_off(brushless_motor_t *motor);
void _compute_next_commutation(zero_crossing_t *zc);
void _update_brushless_line(timer_output_states_t state, ioline_t line);
void _update_brushless_phases(brushless_motor_t *motor);
void _do_brushless_calibration(brushless_motor_t *motor);
void _do_brushless_commutation(brushless_motor_t *motor);
void _set_running(brushless_motor_t *motor);
void _set_free_wheeling(brushless_motor_t *motor);
void _set_tied_to_ground(brushless_motor_t *motor);
void _rpm_counter_update(brushless_motor_t *motor);
void _adc1_current_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
void _adc3_voltage_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n);
void _update_duty_cycle(brushless_motor_t *motor);
void _set_duty_cycle(brushless_motor_t *motor, float duty_cycle);
void _put_all_half_bridges_floating(void);
void _adcStart(void);
void _timersStart(void);
void _motorsInit(void);

/********************               INTERNAL VARIABLES              ********************/

/**
 * Commutation tables
 */
static const brushless_commutation_scheme_line_t pwm_commutation_schemes[NB_OF_COMMUTATION_SCHEME][NB_STEPS_BRUSHLESS] = {
	/*	Phase1 P	Phase1 N	Phase2 P	Phase2 N	Phase3 P	Phase3 N	Floating 	Low side	BEMF slope
																				phase 		conducting
																						 	phase 	*/
	/**
	 * Double PWM
	 */
	{
		{OUT_PWM,	OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_LOW,	OUT_HIGH,	PHASE2,		PHASE3, 	BEMF_NEGATIVE},
		{OUT_PWM, 	OUT_PWM,	OUT_LOW,	OUT_HIGH,	OUT_LOW,	OUT_LOW,	PHASE3, 	PHASE2, 	BEMF_POSITIVE},
		{OUT_LOW, 	OUT_LOW,	OUT_LOW,	OUT_HIGH,	OUT_PWM,	OUT_PWM, 	PHASE1, 	PHASE2, 	BEMF_NEGATIVE},
		{OUT_LOW, 	OUT_HIGH,	OUT_LOW,	OUT_LOW,	OUT_PWM,	OUT_PWM,	PHASE2, 	PHASE1, 	BEMF_POSITIVE},
		{OUT_LOW, 	OUT_HIGH,	OUT_PWM,	OUT_PWM,	OUT_LOW,	OUT_LOW,	PHASE3,		PHASE1, 	BEMF_NEGATIVE},
		{OUT_LOW, 	OUT_LOW,	OUT_PWM,	OUT_PWM,	OUT_LOW,	OUT_HIGH,	PHASE1,		PHASE3, 	BEMF_POSITIVE}
	},
	/**
	 * Simple PWM
	 */
	{
		{OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_LOW,	OUT_LOW,	OUT_HIGH,	PHASE2,		PHASE3, 	BEMF_NEGATIVE},
		{OUT_PWM, 	OUT_LOW,	OUT_LOW,	OUT_HIGH,	OUT_LOW,	OUT_LOW,	PHASE3, 	PHASE2, 	BEMF_POSITIVE},
		{OUT_LOW, 	OUT_LOW,	OUT_LOW,	OUT_HIGH,	OUT_PWM,	OUT_LOW, 	PHASE1, 	PHASE2, 	BEMF_NEGATIVE},
		{OUT_LOW, 	OUT_HIGH,	OUT_LOW,	OUT_LOW,	OUT_PWM,	OUT_LOW,	PHASE2, 	PHASE1, 	BEMF_POSITIVE},
		{OUT_LOW, 	OUT_HIGH,	OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_LOW,	PHASE3,		PHASE1, 	BEMF_NEGATIVE},
		{OUT_LOW, 	OUT_LOW,	OUT_PWM,	OUT_LOW,	OUT_LOW,	OUT_HIGH,	PHASE1,		PHASE3, 	BEMF_POSITIVE}
	}
};

static const brushless_commutation_scheme_line_t pwm_free_wheeling = 
{ 
	OUT_LOW, OUT_LOW, OUT_LOW, OUT_LOW, OUT_LOW, OUT_LOW, 0, 0, 0
};

static const brushless_commutation_scheme_line_t pwm_ground = 
{ 
	OUT_LOW, OUT_HIGH, OUT_LOW, OUT_HIGH, OUT_LOW, OUT_HIGH, 0, 0, 0
};

/**
 * zc_function_t functions. See zc_det_methods_t for explanations
 */
static const zc_function_t brushless_zc_functions[NB_ZC_METHODS] = {
	_zero_crossing_detect_off,
	_zero_crossing_detect_on,
	_zero_crossing_calibration_off
};

/**
 * Coeffs of linear current aproximation functions based on collected data.
 * The linear approximation is different for each duty cycle.
 */
static const current_approx_coeff_t current_approx_coeffs[22] = {
  {0, 0},				//	0%
  {0, 0},				//	5%
  {-0.0009, 1.869},		//	15%
  {-0.0014, 3.0555},	//	20%
  {-0.0020, 4.2347},	//	25%
  {-0.0024, 5.2374},	//	30%
  {-0.0029, 6.3813},	//	35%
  {-0.0033, 7.2652},	//	40%
  {-0.0038, 8.2135},	//	45%
  {-0.0042, 9.0953},	//	50%
  {-0.0047, 10.233},	//	55%
  {-0.0052, 11.271},	//	60%
  {-0.0058, 12.513},	//	65%
  {-0.0061, 13.166},	//	70%
  {-0.0069, 14.778},	//	75%
  {-0.0068, 14.558},	//	80%
  {-0.0073, 15.33},		//	85%
  {-0.0081, 16.825},	//	90%
  {-0.0084, 17.445},	//	95%
  {-0.0094, 19.379},	//	100%
  {-0.0096, 19.72},		//	-%
  {0, 0}	//dummy 0 to spare an if statement and avoid the segmentation fault in the motorsGetCurrent function
};

/**
 *	Half bridges structure
 */
#if (NB_OF_HALF_BRIDGES > 0)
static const half_bridge_t half_bridges[NB_OF_HALF_BRIDGES] = {
	{
		.p_control_line					= P_CONTROL_LINE_1,
		.n_control_line					= N_CONTROL_LINE_1,
		.pwmp 							= PWM_DRIVER_1,
		.PWM_p_channel					= PWM_P_CHANNEL_1,
		.PWM_n_channel					= PWM_N_CHANNEL_1,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_1,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_1
	},
#if (NB_OF_HALF_BRIDGES > 1)
	{
		.p_control_line					= P_CONTROL_LINE_2,
		.n_control_line					= N_CONTROL_LINE_2,
		.pwmp 							= PWM_DRIVER_2,
		.PWM_p_channel					= PWM_P_CHANNEL_2,
		.PWM_n_channel					= PWM_N_CHANNEL_2,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_2,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_2
	},
#endif /* (NB_OF_HALF_BRIDGES > 1) */
#if (NB_OF_HALF_BRIDGES > 2)
	{
		.p_control_line					= P_CONTROL_LINE_3,
		.n_control_line					= N_CONTROL_LINE_3,
		.pwmp 							= PWM_DRIVER_3,
		.PWM_p_channel					= PWM_P_CHANNEL_3,
		.PWM_n_channel					= PWM_N_CHANNEL_3,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_3,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_3
	},
#endif /* (NB_OF_HALF_BRIDGES > 2) */
#if (NB_OF_HALF_BRIDGES > 3)
	{
		.p_control_line					= P_CONTROL_LINE_4,
		.n_control_line					= N_CONTROL_LINE_4,
		.pwmp 							= PWM_DRIVER_4,
		.PWM_p_channel					= PWM_P_CHANNEL_4,
		.PWM_n_channel					= PWM_N_CHANNEL_4,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_4,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_4
	},
#endif /* (NB_OF_HALF_BRIDGES > 3) */
#if (NB_OF_HALF_BRIDGES > 4)
	{
		.p_control_line					= P_CONTROL_LINE_5,
		.n_control_line					= N_CONTROL_LINE_5,
		.pwmp 							= PWM_DRIVER_5,
		.PWM_p_channel					= PWM_P_CHANNEL_5,
		.PWM_n_channel					= PWM_N_CHANNEL_5,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_5,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_5
	},
#endif /* (NB_OF_HALF_BRIDGES > 4) */
#if (NB_OF_HALF_BRIDGES > 5)
	{
		.p_control_line					= P_CONTROL_LINE_6,
		.n_control_line					= N_CONTROL_LINE_6,
		.pwmp 							= PWM_DRIVER_6,
		.PWM_p_channel					= PWM_P_CHANNEL_6,
		.PWM_n_channel					= PWM_N_CHANNEL_6,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_6,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_6
	},
#endif /* (NB_OF_HALF_BRIDGES > 5) */
#if (NB_OF_HALF_BRIDGES > 6)
	{
		.p_control_line					= P_CONTROL_LINE_7,
		.n_control_line					= N_CONTROL_LINE_7,
		.pwmp 							= PWM_DRIVER_7,
		.PWM_p_channel					= PWM_P_CHANNEL_7,
		.PWM_n_channel					= PWM_N_CHANNEL_7,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_7,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_7
	},
#endif /* (NB_OF_HALF_BRIDGES > 6) */
#if (NB_OF_HALF_BRIDGES > 7)
	{
		.p_control_line					= P_CONTROL_LINE_8,
		.n_control_line					= N_CONTROL_LINE_8,
		.pwmp 							= PWM_DRIVER_8,
		.PWM_p_channel					= PWM_P_CHANNEL_8,
		.PWM_n_channel					= PWM_N_CHANNEL_8,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_8,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_8
	},
#endif /* (NB_OF_HALF_BRIDGES > 7) */
#if (NB_OF_HALF_BRIDGES > 8)
	{
		.p_control_line					= P_CONTROL_LINE_9,
		.n_control_line					= N_CONTROL_LINE_9,
		.pwmp 							= PWM_DRIVER_9,
		.PWM_p_channel					= PWM_P_CHANNEL_9,
		.PWM_n_channel					= PWM_N_CHANNEL_9,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_9,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_9
	},
#endif /* (NB_OF_HALF_BRIDGES > 8) */
#if (NB_OF_HALF_BRIDGES > 9)
	{
		.p_control_line					= P_CONTROL_LINE_10,
		.n_control_line					= N_CONTROL_LINE_10,
		.pwmp 							= PWM_DRIVER_10,
		.PWM_p_channel					= PWM_P_CHANNEL_10,
		.PWM_n_channel					= PWM_N_CHANNEL_10,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_10,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_10
	},
#endif /* (NB_OF_HALF_BRIDGES > 9) */
#if (NB_OF_HALF_BRIDGES > 10)
	{
		.p_control_line					= P_CONTROL_LINE_11,
		.n_control_line					= N_CONTROL_LINE_11,
		.pwmp 							= PWM_DRIVER_11,
		.PWM_p_channel					= PWM_P_CHANNEL_11,
		.PWM_n_channel					= PWM_N_CHANNEL_11,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_11,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_11
	},
#endif /* (NB_OF_HALF_BRIDGES > 10) */
#if (NB_OF_HALF_BRIDGES > 11)
	{
		.p_control_line					= P_CONTROL_LINE_12,
		.n_control_line					= N_CONTROL_LINE_12,
		.pwmp 							= PWM_DRIVER_12,
		.PWM_p_channel					= PWM_P_CHANNEL_12,
		.PWM_n_channel					= PWM_N_CHANNEL_12,
		.ADC3FloatingMeasureChannel		= ADC3_VOLTAGE_CHANNEL_12,
		.ADC1ConductingMeasureChannel	= ADC1_CURRENT_CHANNEL_12
	}
#endif /* (NB_OF_HALF_BRIDGES > 11) */
};
#endif /* (NB_OF_HALF_BRIDGES > 0) */


/**
 *	Brushless motors structure
 */
#if (NB_OF_BRUSHLESS_MOTOR > 0)
#if (NB_OF_HALF_BRIDGES < 3)
#error "we need at least 3 half bridges to drive one brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 3) */
static brushless_motor_t brushless_motors[NB_OF_BRUSHLESS_MOTOR] = {
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_1_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_1_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_1_DIRECTION,
		.ramp_steps 		= RAMP_STEPS_DUTY_CYCLE,
		.nb_of_poles 		= 14
	},
#if (NB_OF_BRUSHLESS_MOTOR > 1)
#if (NB_OF_HALF_BRIDGES < 6)
#error "we need at least 6 half bridges to drive two brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 6) */
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_2_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_2_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_2_DIRECTION,
		.ramp_steps 		= RAMP_STEPS_DUTY_CYCLE,
		.nb_of_poles 		= 14
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
#if (NB_OF_HALF_BRIDGES < 9)
#error "we need at least 9 half bridges to drive three brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 9) */
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_3_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_3_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_3_DIRECTION,
		.ramp_steps 		= RAMP_STEPS_DUTY_CYCLE,
		.nb_of_poles 		= 14
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
#if (NB_OF_HALF_BRIDGES < 12)
#error "we need at least 12 half bridges to drive four brushless motors !"
#endif /* (NB_OF_HALF_BRIDGES < 12) */
	{
		.phases[PHASE1] 	= &half_bridges[BRUSHLESS_MOTOR_4_PHASE1],
		.phases[PHASE2] 	= &half_bridges[BRUSHLESS_MOTOR_4_PHASE2],
		.phases[PHASE3] 	= &half_bridges[BRUSHLESS_MOTOR_4_PHASE3],
		.commutation_scheme = BRUSHLESS_MOTOR_4_COMMUTATION,
		.direction 			= BRUSHLESS_MOTOR_4_DIRECTION,
		.ramp_steps 		= RAMP_STEPS_DUTY_CYCLE,
		.nb_of_poles 		= 14
	},
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
};
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */


/**
 *	ADCs config
 */

/* circular buffer */
static adcsample_t adc1_buffer[ADC1_NB_ELEMENT_SEQ * MAX_NB_OF_BRUSHLESS_MOTOR * ADC1_BUFFER_DEPTH] = {0};

/* ADC 1 Configuration */
static const ADCConversionGroup ADC1Config = {
    .circular = true,
    .num_channels = ADC1_NB_ELEMENT_SEQ * MAX_NB_OF_BRUSHLESS_MOTOR,
    .end_cb = _adc1_current_cb,
    .error_cb = NULL,
    .cr1 = 0,   /* No OVR int, 12 bit resolution, no AWDG/JAWDG*/
    .cr2 = 0,  	/* Manual start of regular channels, no OVR detect */                     
    .htr = 0,
    .ltr = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN4(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN5(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN6(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN7(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN8(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN9(ADC_SAMPLE_3),
    .smpr1 = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_3)|
             ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
    .sqr3 =  0,
    .sqr2 =  0,
    .sqr1 =  ADC_SQR1_NUM_CH(ADC1_NB_ELEMENT_SEQ * MAX_NB_OF_BRUSHLESS_MOTOR),
};

/* circular buffer */
static adcsample_t adc3_buffer[MAX_NB_OF_BRUSHLESS_MOTOR * ADC3_BUFFER_DEPTH] = {0};
/* ADC 3 Configuration */
static ADCConversionGroup ADC3Config = {
	.circular = true,
    .num_channels = MAX_NB_OF_BRUSHLESS_MOTOR,
    .end_cb = _adc3_voltage_cb,
    .error_cb = NULL,
    .cr1 = 0, 	/* No OVR int, 12 bit resolution, no AWDG/JAWDG */
    .cr2 = 		/* Manual start of regular channels, no OVR detect */
           ADC_CR2_EXTEN_BOTH    |	/* We need both as OCxREF don't behave as expected. See Errata STM32F7 */
           ADC_CR2_EXTSEL_SRC(TRG_TIMER1_TRGO2),  /* External trigger is from Timer 1 TRGO 2*/                
    .htr = 0,
    .ltr = 0,
    .smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN1(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN2(ADC_SAMPLE_3)|
             ADC_SMPR2_SMP_AN3(ADC_SAMPLE_3),
    .smpr1 = 0,
    .sqr3 =  0,
    .sqr2 = 0,
    .sqr1 =  ADC_SQR1_NUM_CH(MAX_NB_OF_BRUSHLESS_MOTOR),
};

/**
 *	Timers config
 */

static PWMConfig tim_1_cfg = {
  .frequency = STM32_TIMCLK2,                	/* PWM clock frequency.   */
  .period    = PERIOD_PWM_52_KHZ_APB2,			/* PWM period in ticks    */
  .callback  = NULL,                            /* Callback called when UIF is set*/
  /* PWM Channels configuration */
  {
   {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  /* Master Mode Selection 1 : Enable, Master Mode Selection 2 : OC4REF */
  .cr2  = STM32_TIM_CR2_MMS(MMS_ENABLE) | STM32_TIM_CR2_MMS2(MMS2_COMPARE_OC4REF), 
  .bdtr = STM32_TIM_BDTR_OSSR, /* OSSR = 1 */
  .dier = 0
};

static PWMConfig tim_8_cfg = {
  .frequency = STM32_TIMCLK2,                	/* PWM clock frequency.   */
  .period    = PERIOD_PWM_52_KHZ_APB2,			/* PWM period in ticks    */
  .callback  = NULL,                            /* Callback called when UIF is set*/
  /* PWM Channels configuration */
  {
   {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  .cr2  = 0,
  .bdtr = STM32_TIM_BDTR_OSSR, /* OSSR = 1 */
  .dier = 0
};

static PWMConfig tim_234_cfg = {
  .frequency = STM32_TIMCLK1,                	/* PWM clock frequency.   */
  .period    = PERIOD_PWM_52_KHZ_APB1,			/* PWM period in ticks    */
  .callback  = NULL,                            /* Callback called when UIF is set*/
  /* PWM Channels configuration */
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL},	//active low to act as the complementary output of channel 1
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_LOW, NULL}	//active low to act as the complementary output of channel 3
  },
  .cr2  = 0,
  .bdtr = 0,
  .dier = 0
};

/********************          PRIVATE MACROS AND FUNCTIONS         ********************/

/**
 * Changes the value of the timer channel linked to the ADC3 trigger
 * 
 * @param time Percentage [0-1] representing at which moment of the PWM cycle we want to have the trigger
 */
#define UPDATE_ADC3_TRIGGER(time) (PWMD1.tim->CCR[TIM_CHANNEL_4] = time * PWMD1.tim->ARR)
/**
 * Changes the sequence of the ADC3
 * 
 * @param reg	Values to put to the SQR3 register of ADC3	
 */
#define UPDATE_ADC3_SEQUENCE(reg) (ADCD3.adc->SQR3 = reg)
/**
 * Changes the sequence of the ADC1
 * 
 * @param reg1	Values to put to the SQR3 register of ADC1	
 * @param reg2	Values to put to the SQR2 register of ADC1	
 */
#define UPDATE_ADC1_SEQUENCE(reg1, reg2) {\
	ADCD1.adc->SQR3 = reg1; \
	ADCD1.adc->SQR2 = reg2; \
}
/**
 * Starts the ADC1 for one sequence
 */
#define DO_ONE_ADC1_SEQUENCE()	(ADCD1.adc->CR2 |= ADC_CR2_SWSTART)

/**
 * Gives the floating phase 
 * 
 * @param motor	Pointer to the brushless_motor_t structure we want to use
 */
#define GET_FLOATING_PHASE(motor) (pwm_commutation_schemes[(motor)->commutation_scheme][(motor)->step_iterator].floating_phase)
/**
 * Gives the ADC3 channel to measure given the motor 
 * 
 * @param motor	Pointer to the brushless_motor_t structure we want to use
 */
#define GET_FLOATING_PHASE_CHANNEL(motor) ((motor)->phases[pwm_commutation_schemes[(motor)->commutation_scheme][(motor)->step_iterator].floating_phase]->ADC3FloatingMeasureChannel)
/**
 * Gives the ADC1 channel to measure given the motor
 * 
 * @param motor	Pointer to the brushless_motor_t structure we want to use
 */
#define GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(motor) ((motor)->phases[pwm_commutation_schemes[(motor)->commutation_scheme][(motor)->step_iterator].low_side_conducting_phase]->ADC1ConductingMeasureChannel)

/**
 * Sets the GPIO mode of the given line to Alternate
 * The alternate number should already have been set because it only changes the mode
 * 
 * @param line	Line we want to change
 */
#define SET_GPIO_ALTERNATE_MODE(line){\
	PAL_PORT(line)->MODER &= ~(3 << (2 * PAL_PAD(line))); \
	PAL_PORT(line)->MODER |= (2 << (2 * PAL_PAD(line))); \
}
/**
 * Sets the GPIO mode of the given line to Output
 * 
 * @param line	Line we want to change
 */
#define SET_GPIO_OUTPUT_MODE(line){\
	PAL_PORT(line)->MODER &= ~(3 << (2 * PAL_PAD(line))); \
	PAL_PORT(line)->MODER |= (1 << (2 * PAL_PAD(line))); \
}

/**
 * Adds the new zero crossing data to the correct fields of the zero_crossing
 * structure given. Only for ADC data of the PWM ON time
 * 
 * @param zc		Pointer to the zero_crossing_t structure we want to update
 * @param new_data	New zero crossing data to add
 */
#define ADD_NEW_ZC_DATAON(zc, new_data) {\
  	(zc)->previous_dataOn = (zc)->dataOn;\
  	(zc)->dataOn = new_data;\
}

/**
 * Adds the new zero crossing data to the correct fields of the zero_crossing
 * structure given. Only for ADC data of the PWM OFF time
 * 
 * @param zc		Pointer to the zero_crossing_t structure we want to update
 * @param new_data	New zero crossing data to add
 */
#define ADD_NEW_ZC_DATAOFF(zc, new_data) {\
  	(zc)->previous_dataOff = (zc)->dataOff;\
  	(zc)->dataOff = new_data;\
}

/**
 * Calls the zero crossing function corresponding to the one set is the zc_method array
 * 
 * @param motor		Pointer to the brushless_motor_t structure we want to use
 */
#define CALL_ZC_FUNCTION(motor) (brushless_zc_functions[(motor)->zero_crossing.zc_method](motor))

/**
 * Sets the zc flag of the given zero_crossing_t structure
 * 
 * @param zc Pointer to the zero_crossing_t structure we want to use
 */
#define SET_ZC_FLAG(zc)	((zc)->flag = true)

/**
 * Resets the zc flag of the given zero_crossing_t structure
 * 
 * @param zc Pointer to the zero_crossing_t structure we want to use
 */
#define RESET_ZC_FLAG(zc)((zc)->flag = false)

/**
 * Returns true if the zc flag of the given zero_crossing_t structure is set, false otherwise
 * 
 * @param zc Pointer to the zero_crossing_t structure we want to use
 */
#define IS_ZC_FLAG(zc) ((zc)->flag)

/**
 * Returns true if it's the time to commute, false otherwise
 * 
 * @param zc Pointer to the zero_crossing_t structure we want to use
 */
#define TIME_TO_COMMUTE(zc)	((((zc)->time >= (zc)->next_commutation_time) && (zc)->flag))

/**
 * Returns true if the slope of the actual step of the given motor is positive, false otherwise
 * 
 * @param motor Pointer to the brushless_motor_t structure we want to use
 */
#define IS_BEMF_SLOPE_POSITIVE(motor) (((motor)->direction > 0) ? pwm_commutation_schemes[(motor)->commutation_scheme][(motor)->step_iterator].bemf_slope : !pwm_commutation_schemes[(motor)->commutation_scheme][(motor)->step_iterator].bemf_slope)

/**
 * Condensed form of a discrete first order low pass filter y = (1-a)y + ax, 
 * x beeing the new value, y the filtered one and a being the filter coefficient
 * 
 * @param actual		Filtered value to update
 * @param new_value		New value to add to the filtered one
 * @coeff coeff			Filter coefficient of the low_pass filter
 */
#define LOW_PASS_FILTER(actual, new_value, coeff) (actual -= coeff * (actual - new_value))

/**
 * @brief 			Resets the zero crossing structure of the given motor.
 * 
 * @param motor 	Motor to reset the zero crossing structure.
 */
void _zero_crossing_reset(brushless_motor_t *motor){
	static zero_crossing_t *zc = NULL;

	zc = &motor->zero_crossing;

	zc->flag 					= false;
	zc->time 					= 0;
	zc->detection_time 			= 0;
	zc->previous_detection_time	= 0;
	zc->period 					= 0;
	zc->period_filtered 		= 0;
	zc->next_commutation_time 	= 0;
	zc->ticks_since_last_comm 	= 0;

}

/**
 * @brief 			Detect if a zero crossing occurs for the PWM OFF voltage.
 * 
 * @param motor 	Motor to detect the zero crossing. See brushless_motor_t
 * @return 			True if a zero crossing has been detected, False otherwise.
 */
bool _zero_crossing_detect_off(brushless_motor_t *motor){
	static zero_crossing_t *zc = NULL;
	static bool zc_found = false;

	zc_found = false;

	zc = &motor->zero_crossing;

	if(!IS_ZC_FLAG(zc)){
		if(zc->ticks_since_last_comm > DEGAUSS_TICKS_ZC_OFF){
			if(IS_BEMF_SLOPE_POSITIVE(motor)){
				zc_found = (zc->dataOff > motor->ADC_offset_off[GET_FLOATING_PHASE(motor)]);
			}else{
				zc_found = (zc->dataOff <= motor->ADC_offset_off[GET_FLOATING_PHASE(motor)]);
			}
		}else{
			zc->ticks_since_last_comm++;
		}

		if(zc_found){
			_compute_next_commutation(zc);
			zc->ticks_since_last_comm = 0;
			SET_ZC_FLAG(zc);
		}
	}

	return zc_found;
}

/**
 * @brief 			Detect if a zero crossing occurs for the PWM ON voltage.
 * 
 * @param motor 	Motor to detect the zero crossing. See brushless_motor_t
 * @return 			True if a zero crossing has been detected, False otherwise.
 */
bool _zero_crossing_detect_on(brushless_motor_t *motor){
	static zero_crossing_t *zc = NULL;
	static bool zc_found = false;

	zc_found = false;

	zc = &motor->zero_crossing;

	if(!IS_ZC_FLAG(zc)){
		//True if sign has changed
		zc_found = ((((int32_t)zc->dataOn - HALF_BUS_VOLTAGE) ^ ((int32_t)zc->previous_dataOn - HALF_BUS_VOLTAGE)) < 0);

		if(zc_found){
			_compute_next_commutation(zc);
			zc->ticks_since_last_comm = 0;
			SET_ZC_FLAG(zc);
		}
	}

	return zc_found;
}

/**
 * @brief 			Function used to call the good zero crossing method.
 * 					The duty cycle is updated here.
 * 
 * @param motor 	Motor to use. See brushless_motor_t
 */
void _zero_crossing_cb(brushless_motor_t *motor){
	static zero_crossing_t *zc = NULL;

	zc = &motor->zero_crossing;

	if(zc->zc_method != ZC_CALIBRATE_OFF){
		if(motor->duty_cycle_now > ZC_DETECT_METHOD_THESHOLD){
			zc->zc_method = ZC_DETECT_ON;
		}else{
			zc->zc_method = ZC_DETECT_OFF;
		}

		_update_duty_cycle(motor);
	}

	CALL_ZC_FUNCTION(motor);

	zc->time++;

	if(TIME_TO_COMMUTE(zc)){
		RESET_ZC_FLAG(zc);
		_do_brushless_commutation(motor);
	}

	// if(zc->time > 150){
	// 	RESET_ZC_FLAG(zc);
	// 	_do_brushless_commutation(motor);
	// 	zc->time = 0;
	// }
	
}

/**
 * @brief 			It gathers the ADC OFF samples, performs an average and store them
 * 					into the ADC_offset_off field of the motor.
 * 					
 * 					At the end, the motor is restored into its normal state
 * 					with a duty cycle of 0.
 * 
 * @param motor 	Motor to calibrate. See brushless_motor_t
 * @return 			Not used. Always returns 0
 */
bool _zero_crossing_calibration_off(brushless_motor_t *motor){

	// We accumulate the ADC offset at the OFF time
	motor->ADC_offset_off[GET_FLOATING_PHASE(motor)] += motor->zero_crossing.dataOff;

	// Increment the counter each time we do a 6-step cycle
	if(motor->step_iterator >= (NB_STEPS_BRUSHLESS - 1)){
		motor->nb_offset_sample++;

		// We have enough data. Now get the average and go to free wheeling mode
		if(motor->nb_offset_sample >= NB_SAMPLE_OFFSET_CALIBRATION){
			//each phase is sampled 2 times per 6-steps cycle
			motor->nb_offset_sample *= 2;

			motor->ADC_offset_off[PHASE1] /= motor->nb_offset_sample;
			motor->ADC_offset_off[PHASE2] /= motor->nb_offset_sample;
			motor->ADC_offset_off[PHASE3] /= motor->nb_offset_sample;
			motor->nb_offset_sample = 0;

			// When we set a duty cycle of 0, we go to free wheeling mode
			_set_duty_cycle(motor, 0);

			// Restores the zc method to a detect one
			motor->zero_crossing.zc_method = ZC_DETECT_OFF;
		}
	}

	/* We force the commutations at a arbitrary number of steps
	 * to gather adc sample of each phase.
	 */
	if((motor->zero_crossing.time % 50) == 0){
		_do_brushless_commutation(motor);
	}

	return 0;
}

/**
 * @brief 		Computes the next commutation time for the given zero crossing structure
 * 
 * @param zc 	Zero crossing structure to update. See zero_crossing_t
 */
void _compute_next_commutation(zero_crossing_t *zc)
{
	zc->previous_detection_time = zc->detection_time;
	zc->detection_time    		= zc->time;
	zc->period 					= zc->detection_time - zc->previous_detection_time;
	LOW_PASS_FILTER(zc->period_filtered, (float)zc->period, LP_FILTER_COEFF_ZC);
	zc->next_commutation_time 	= zc->time + zc->period_filtered/2 - zc->advance_timing;
}

/**
 * @brief 			Updates the given line to the given state
 * 
 * @param state 	see timer_output_states_t
 * @param line 		line to update
 */
void _update_brushless_line(timer_output_states_t state, ioline_t line){
	switch(state){
		case OUT_LOW:
		{	
			palClearLine(line);
			SET_GPIO_OUTPUT_MODE(line);
			break;
		}
		case OUT_HIGH:
		{	
			palSetLine(line);
			SET_GPIO_OUTPUT_MODE(line);
			break;
		}
		case OUT_PWM:
		{	
			SET_GPIO_ALTERNATE_MODE(line);
			break;
		}
	}
}

/**
 * @brief 			Updates the phases lines accordingly to the commutation state and the commutation table
 * 
 * @param motor 	Motor to update
 */
void _update_brushless_phases(brushless_motor_t *motor){

	static const brushless_commutation_scheme_line_t* comm_line = NULL;

	switch(motor->state){
		case RUNNING:
		{
			comm_line = &pwm_commutation_schemes[motor->commutation_scheme][motor->step_iterator];
			break;
		}
		case FREE_WHELLING:
		{
			comm_line = &pwm_free_wheeling;
			break;
		}
		case TIED_TO_GROUND:
		{
			comm_line = &pwm_ground;
			break;
		}
		default:
		return;
	}
	
	_update_brushless_line(comm_line->phase1_p, motor->phases[PHASE1]->p_control_line);
	_update_brushless_line(comm_line->phase1_n, motor->phases[PHASE1]->n_control_line);
	_update_brushless_line(comm_line->phase2_p, motor->phases[PHASE2]->p_control_line);
	_update_brushless_line(comm_line->phase2_n, motor->phases[PHASE2]->n_control_line);
	_update_brushless_line(comm_line->phase3_p, motor->phases[PHASE3]->p_control_line);
	_update_brushless_line(comm_line->phase3_n, motor->phases[PHASE3]->n_control_line);
}

/**
 * @brief 			Configures the motor mode and zero crossing callback 
 * 					to perform a ADC offset calibration for the OFF time
 * 					
 * 					At the end, the motor is restored into its normal state
 * 					with a duty cycle of 0.
 * 
 * @param motor 	Motor to calibrate the ADC offsets. See brushless_motor_t
 */	
void _do_brushless_calibration(brushless_motor_t *motor){
	motor->zero_crossing.zc_method = ZC_CALIBRATE_OFF;
	motor->step_iterator = 0;
	_set_tied_to_ground(motor);
}

/**
 * @brief 			Does one commutation step for the given brushless motor
 * 
 * @param motor 	Motor to commute. See brushless_motor_t
 */
void _do_brushless_commutation(brushless_motor_t *motor){
	motor->step_iterator += motor->direction;

	if(motor->step_iterator >= NB_STEPS_BRUSHLESS){
		motor->step_iterator = 0;
	}else if(motor->step_iterator < 0){
		motor->step_iterator = (NB_STEPS_BRUSHLESS-1);
	}

	motor->zero_crossing.nb_commutations++;
	_update_brushless_phases(motor);

}

/**
 * @brief 			Puts the motor into RUNNING mode. 
 * 					See motor_states_t for mode explanations
 * 
 * @param motor 	Motor to put into RUNNING mode. See brushless_motor_t
 */
void _set_running(brushless_motor_t *motor){
	_zero_crossing_reset(motor);
	motor->state = RUNNING;
	_update_brushless_phases(motor);
}

/**
 * @brief 			Puts the motor into FREE_WHEELING mode.
 * 					See motor_states_t for mode explanations
 * 
 * @param motor 	Motor to put into FREE_WHEELING mode. See brushless_motor_t
 */
void _set_free_wheeling(brushless_motor_t *motor){
	motor->state = FREE_WHELLING;
	_update_brushless_phases(motor);
}

/**
 * @brief 			Puts the motor into TIED_TO_GROUND mode.
 * 					See motor_states_t for mode explanations
 * 
 * @param motor 	Motor to put into TIED_TO_GROUND mode. See brushless_motor_t
 */
void _set_tied_to_ground(brushless_motor_t *motor){
	motor->state = TIED_TO_GROUND;
	_update_brushless_phases(motor);
}

/**
 * @brief 			Updates the actual current meter of the given motor
 * 					Intended to be called at each adc current callback
 * 				
 * @param motor 	Motor to update the current. It is a pointer. See brushless_motor_t
 * @param buffer 	buffer containing the ADC raw values. Be carefull. The address
 * 					of the first element should be given.
 */

#define CURRENT_METER_UPDATE(motor, buffer) {\
	LOW_PASS_FILTER((motor)->current_meter.low_pass_1, (buffer)[0 * MAX_NB_OF_BRUSHLESS_MOTOR], LP_FILTER_COEFF_CURRENT);\
	LOW_PASS_FILTER((motor)->current_meter.low_pass_1, (buffer)[1 * MAX_NB_OF_BRUSHLESS_MOTOR], LP_FILTER_COEFF_CURRENT);\
	LOW_PASS_FILTER((motor)->current_meter.low_pass_1, (buffer)[2 * MAX_NB_OF_BRUSHLESS_MOTOR], LP_FILTER_COEFF_CURRENT);\
	LOW_PASS_FILTER((motor)->current_meter.current, (motor)->current_meter.low_pass_1, LP_FILTER_COEFF_CURRENT);\
}

/**
 * @brief 		Updates the RPM counter of the given motor
 * 				It is intended to be called at a frequency of 52kHz. Otherwise it needs to be modified
 * @param motor Motor to update the RPM counter. See brushless_motor_t
 */
void _rpm_counter_update(brushless_motor_t *motor){

	static rpm_counter_t *rpm = NULL;

	rpm = &motor->rpm_counter;

	rpm->count++;
	if(rpm->count >= TICKS_52_KHZ_TO_100HZ){
		rpm->previous_nb_comms = rpm->nb_comms;
		rpm->nb_comms = motor->zero_crossing.nb_commutations;
		LOW_PASS_FILTER(rpm->rpm, ((float)(rpm->nb_comms - rpm->previous_nb_comms) * RP10MS_TO_RPM) / motor->nb_of_poles, LP_FILTER_COEFF_RPM);
		rpm->count = 0;
	}
}

/**
 * @brief 			ADC1 callback. Used to gather the currents from the four motors
 * @param adcp 		Not used
 * @param buffer 	Buffer containing the new data
 * @param n 		Not used
 */
void _adc1_current_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n){
	(void) adcp;
	(void) n;

#if (NB_OF_BRUSHLESS_MOTOR > 0)
	UPDATE_ADC1_SEQUENCE(
		ADC_SQR3_SQ1_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_1]))|
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		ADC_SQR3_SQ2_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_2]))|
#else
		ADC_SQR3_SQ2_N (ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		ADC_SQR3_SQ3_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_3]))|
#else
		ADC_SQR3_SQ3_N (ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		ADC_SQR3_SQ4_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_4]))|
#else
		ADC_SQR3_SQ4_N (ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
		ADC_SQR3_SQ5_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_1]))|
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		ADC_SQR3_SQ6_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_2]))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */		
		0,
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		ADC_SQR2_SQ7_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_3]))|
#else
		ADC_SQR2_SQ7_N (ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */	
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		ADC_SQR2_SQ8_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_4]))|
#else
		ADC_SQR2_SQ8_N (ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */	
		ADC_SQR2_SQ9_N (GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_1]))|
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		ADC_SQR2_SQ10_N(GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_2]))|
#else
		ADC_SQR2_SQ10_N(ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */	
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		ADC_SQR2_SQ11_N(GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_3]))|
#else
		ADC_SQR2_SQ11_N(ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */	
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		ADC_SQR2_SQ12_N(GET_LOW_SIDE_CONDUCTING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_4]))|
#else
		ADC_SQR2_SQ12_N(ADC_CHANNEL_VREFINT)|	//dummy sampling
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */	
	0);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
	
	DO_ONE_ADC1_SEQUENCE();

#if (NB_OF_BRUSHLESS_MOTOR > 0)
	CURRENT_METER_UPDATE(&brushless_motors[BRUSHLESS_MOTOR_1], &buffer[BRUSHLESS_MOTOR_1]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */	
#if (NB_OF_BRUSHLESS_MOTOR > 1)
	CURRENT_METER_UPDATE(&brushless_motors[BRUSHLESS_MOTOR_2], &buffer[BRUSHLESS_MOTOR_2]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
	CURRENT_METER_UPDATE(&brushless_motors[BRUSHLESS_MOTOR_3], &buffer[BRUSHLESS_MOTOR_3]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
	CURRENT_METER_UPDATE(&brushless_motors[BRUSHLESS_MOTOR_4], &buffer[BRUSHLESS_MOTOR_4]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
}

/**
 * @brief 			ADC3 callback. Used to gather the voltages from the four motors.
 * 					Also runs the brushless zero crossing algorithms and commutations
 * @param adcp 		Not used
 * @param buffer 	Buffer containing the new data
 * @param n 		Not used
 */
void _adc3_voltage_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n){
	(void) adcp;
	(void) n;

	static bool state = true;
	if(state){
		//we sampled OFF PWM
		UPDATE_ADC3_TRIGGER(ADC3_ON_SAMPLE_TIME);

#if (NB_OF_BRUSHLESS_MOTOR > 0)
		ADD_NEW_ZC_DATAOFF(&(brushless_motors[BRUSHLESS_MOTOR_1].zero_crossing), buffer[BRUSHLESS_MOTOR_1]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		ADD_NEW_ZC_DATAOFF(&(brushless_motors[BRUSHLESS_MOTOR_2].zero_crossing), buffer[BRUSHLESS_MOTOR_2]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		ADD_NEW_ZC_DATAOFF(&(brushless_motors[BRUSHLESS_MOTOR_3].zero_crossing), buffer[BRUSHLESS_MOTOR_3]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		ADD_NEW_ZC_DATAOFF(&(brushless_motors[BRUSHLESS_MOTOR_4].zero_crossing), buffer[BRUSHLESS_MOTOR_4]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */


#if (NB_OF_BRUSHLESS_MOTOR > 0)
		_rpm_counter_update(&(brushless_motors[BRUSHLESS_MOTOR_1]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		_rpm_counter_update(&(brushless_motors[BRUSHLESS_MOTOR_2]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		_rpm_counter_update(&(brushless_motors[BRUSHLESS_MOTOR_3]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		_rpm_counter_update(&(brushless_motors[BRUSHLESS_MOTOR_4]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
	}else{
		//we sampled ON PWM
		UPDATE_ADC3_TRIGGER(ADC3_OFF_SAMPLE_TIME);
#if (NB_OF_BRUSHLESS_MOTOR > 0)
		UPDATE_ADC3_SEQUENCE(
			ADC_SQR3_SQ1_N(GET_FLOATING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_1]))|
#if (NB_OF_BRUSHLESS_MOTOR > 1)
    		ADC_SQR3_SQ2_N(GET_FLOATING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_2]))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
    		ADC_SQR3_SQ3_N(GET_FLOATING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_3]))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
    		ADC_SQR3_SQ4_N(GET_FLOATING_PHASE_CHANNEL(&brushless_motors[BRUSHLESS_MOTOR_4]))|
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
    	0);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */


#if (NB_OF_BRUSHLESS_MOTOR > 0)
		ADD_NEW_ZC_DATAON(&(brushless_motors[BRUSHLESS_MOTOR_1].zero_crossing), buffer[BRUSHLESS_MOTOR_1]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		ADD_NEW_ZC_DATAON(&(brushless_motors[BRUSHLESS_MOTOR_2].zero_crossing), buffer[BRUSHLESS_MOTOR_2]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		ADD_NEW_ZC_DATAON(&(brushless_motors[BRUSHLESS_MOTOR_3].zero_crossing), buffer[BRUSHLESS_MOTOR_3]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		ADD_NEW_ZC_DATAON(&(brushless_motors[BRUSHLESS_MOTOR_4].zero_crossing), buffer[BRUSHLESS_MOTOR_4]);
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */


#if (NB_OF_BRUSHLESS_MOTOR > 0)
		_zero_crossing_cb(&(brushless_motors[BRUSHLESS_MOTOR_1]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
#if (NB_OF_BRUSHLESS_MOTOR > 1)
		_zero_crossing_cb(&(brushless_motors[BRUSHLESS_MOTOR_2]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
		_zero_crossing_cb(&(brushless_motors[BRUSHLESS_MOTOR_3]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
		_zero_crossing_cb(&(brushless_motors[BRUSHLESS_MOTOR_4]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */

	}
	//switches the state
	state = !state;

}

/**
 * @brief 			Updated the duty cycle of the given motor.
 * 					This function is called at each PWM cycle
 * 					and updates the duty cycle with a ramp steps
 * 					to smooth the change in the duty cycle.
 * 					
 * 					It changes duty_cycle_now to get close to duty_cycle_goal
 * 					
 * @param motor 	Motor to update the duty_cycle. See brushless_motor_t
 */
void _update_duty_cycle(brushless_motor_t *motor){

	static float duty_cycle = 0;

	if(motor->duty_cycle_goal > motor->duty_cycle_now){
		if((motor->duty_cycle_goal - motor->duty_cycle_now) < motor->ramp_steps){
			duty_cycle = motor->duty_cycle_goal;
		}else{
			duty_cycle = motor->duty_cycle_now + motor->ramp_steps;
		}
	}else{
		if((motor->duty_cycle_now - motor->duty_cycle_goal) < motor->ramp_steps){
			duty_cycle = motor->duty_cycle_goal;
		}else{
			duty_cycle = motor->duty_cycle_now - motor->ramp_steps;
		}
	}

	if(duty_cycle > 100){
		duty_cycle = 100;
	}else if(duty_cycle < 0){
		duty_cycle = 0;
	}
	if(duty_cycle != motor->duty_cycle_now){
		_set_duty_cycle(motor, duty_cycle);
	}
}

/**
 * @brief  				Sets the given duty cycle to the given motor.
 * 						The duty cycle given is immediatly applied without check.
 * 						!! A huge change can lead in really big currents and destroy the motor
 * 						or the H-bridges !!
 * 						
 * 						When the duty cycle is 0, puts the motor into FREE_WHEELING mode.
 * 
 * @param motor 		Motor to set the duty cycle. See brushless_motor_t
 * @param duty_cycle 	duty cycle to set. Between 0 and 100. No check is made 
 * 						in this function !
 */	
void _set_duty_cycle(brushless_motor_t *motor, float duty_cycle){
	static const half_bridge_t* phase1 = NULL;
	static const half_bridge_t* phase2 = NULL;
	static const half_bridge_t* phase3 = NULL;
	static float dc = 0;
	static uint16_t new_ccr = 0;

	phase1 = motor->phases[PHASE1];
	phase2 = motor->phases[PHASE2];
	phase3 = motor->phases[PHASE3];

	/* stores the duty cycle */ 
	motor->duty_cycle_now = duty_cycle;

	if(duty_cycle <= 0){
		_set_free_wheeling(motor);
	}else if (motor->state != RUNNING){
		_set_running(motor);
	}

	/* 
	 * We invert the value because we use a PWM_mode 2 in the timers
	 * so when CCR = 0, we have a duty cycle of 100%									
	 */

	dc = (100 - duty_cycle)/100;

	if(phase1->pwmp == &PWMD1 || phase1->pwmp == &PWMD8){
		new_ccr = dc * PERIOD_PWM_52_KHZ_APB2;
		phase1->pwmp->tim->CCR[phase1->PWM_p_channel] = new_ccr;
		phase2->pwmp->tim->CCR[phase2->PWM_p_channel] = new_ccr;
		phase3->pwmp->tim->CCR[phase3->PWM_p_channel] = new_ccr;
	}else{
		new_ccr = dc * PERIOD_PWM_52_KHZ_APB1;
		phase1->pwmp->tim->CCR[phase1->PWM_p_channel] = new_ccr;
		phase1->pwmp->tim->CCR[phase1->PWM_n_channel] = new_ccr;
		phase2->pwmp->tim->CCR[phase2->PWM_p_channel] = new_ccr;
		phase2->pwmp->tim->CCR[phase2->PWM_n_channel] = new_ccr;
		phase3->pwmp->tim->CCR[phase3->PWM_p_channel] = new_ccr;
		phase3->pwmp->tim->CCR[phase3->PWM_n_channel] = new_ccr;
	}
}

void _put_all_half_bridges_floating(void){

	for(uint8_t i = 0 ; i < NB_OF_HALF_BRIDGES ; i++){
		palClearLine(half_bridges[i].p_control_line);
		SET_GPIO_OUTPUT_MODE(half_bridges[i].p_control_line);
		palClearLine(half_bridges[i].n_control_line);
		SET_GPIO_OUTPUT_MODE(half_bridges[i].n_control_line);
	}
}

/**
 * @brief 	Configures and starts the ADCs used by the motors control (ADC1 and ADC3).
 */
void _adcStart(void){

	adcStart(&ADCD1, NULL);
	adcStart(&ADCD3, NULL);

	/* We have one interrupt per sequence and we use a double buffer */
	adcStartConversion(&ADCD1, &ADC1Config, adc1_buffer, ADC1_BUFFER_DEPTH);
	adcStartConversion(&ADCD3, &ADC3Config, adc3_buffer, ADC3_BUFFER_DEPTH);
	DO_ONE_ADC1_SEQUENCE();

}

/**
 * @brief 	Configures and starts the timers used by the motors control 
 * 			(timers 1, 2, 3, 4, 8).
 */
void _timersStart(void){

	pwmStart(&PWMD1, &tim_1_cfg);
	/* additionnal config */
	PWMD1.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD1.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD1.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD1.tim->CCMR2 	&= ~STM32_TIM_CCMR2_OC4PE;	// Disables the preload for channel 4
	PWMD1.tim->CNT 		 = 0;							// Resets the counter to zero

	pwmStart(&PWMD8, &tim_8_cfg);
	/* additionnal config */
	PWMD8.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD8.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD8.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD8.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2);  // Sets channels 3 to PWM mode 2
	PWMD8.tim->CNT 		 = 0;							// Resets the counter to zero

	pwmStart(&PWMD2, &tim_234_cfg);
	PWMD2.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD2.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD2.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD2.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD2.tim->CNT 		 = 0;

	pwmStart(&PWMD3, &tim_234_cfg);
	PWMD3.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD3.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD3.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD3.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD3.tim->CNT 		 = 0;

	pwmStart(&PWMD4, &tim_234_cfg);
	PWMD4.tim->CR1 		&= ~STM32_TIM_CR1_CEN;     	// Disables the counter until correct configuration
	PWMD4.tim->SMCR   	 = STM32_TIM_SMCR_SMS(SMS_TRIGGER_MODE) | STM32_TIM_SMCR_TS(TS_ITR0); //external trigger mode and TIM1 master
	PWMD4.tim->CCMR1 	|= STM32_TIM_CCMR1_OC1M(OC_PWM_MODE_2) | STM32_TIM_CCMR1_OC2M(OC_PWM_MODE_2);	// Sets channels 1 and 2 to PWM mode 2
	PWMD4.tim->CCMR2 	|= STM32_TIM_CCMR2_OC3M(OC_PWM_MODE_2) | STM32_TIM_CCMR2_OC4M(OC_PWM_MODE_2);  // Sets channels 3 and 4 to PWM mode 2
	PWMD4.tim->CNT 		 = 0;

	/* Enables the timers */ 
	PWMD1.tim->CR1 |= STM32_TIM_CR1_CEN;
}

/**
 * @brief 	Motors inits
 */
void _motorsInit(void){

	_put_all_half_bridges_floating();

#if (NB_OF_BRUSHLESS_MOTOR > 0)
	_do_brushless_calibration(&(brushless_motors[BRUSHLESS_MOTOR_1]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 0) */
#if (NB_OF_BRUSHLESS_MOTOR > 1)
	_do_brushless_calibration(&(brushless_motors[BRUSHLESS_MOTOR_2]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 1) */
#if (NB_OF_BRUSHLESS_MOTOR > 2)
	_do_brushless_calibration(&(brushless_motors[BRUSHLESS_MOTOR_3]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 2) */
#if (NB_OF_BRUSHLESS_MOTOR > 3)
	_do_brushless_calibration(&(brushless_motors[BRUSHLESS_MOTOR_4]));
#endif /* (NB_OF_BRUSHLESS_MOTOR > 3) */
	
}

/********************               PUBLIC FUNCTIONS                ********************/
void motorsStart(void){
	_motorsInit();
	_adcStart();
	_timersStart();
}

void motorSetDutyCycle(brushless_motors_names_t motor_name, uint8_t duty_cycle){
	if(motor_name >= NB_OF_BRUSHLESS_MOTOR){
		return;
	}

	if(duty_cycle > 100){
		duty_cycle = 100;
	}

	brushless_motors[motor_name].duty_cycle_goal = duty_cycle;
}

float motorGetDutyCycle(brushless_motors_names_t motor_name){
	if(motor_name >= NB_OF_BRUSHLESS_MOTOR){
		return 0;
	}

	return brushless_motors[motor_name].duty_cycle_now;
}

float motorsGetCurrent(brushless_motors_names_t motor_name){
	if(motor_name >= NB_OF_BRUSHLESS_MOTOR){
		return 0;
	}

	static float float_index = 0;
	static uint8_t int_index = 0;

	static float a_part = 0;
	static float b_part = 0;

	static float a_coeff = 0;
	static float b_coeff = 0;
	
	//we get a float index of which approximation coefficients to use
	float_index = brushless_motors[motor_name].duty_cycle_now / 5;
	//So now we get the usable index
    int_index = (uint8_t)float_index;

    /*
    *	We do an interpolation of the coeffs of the two approximation functions
    *	arround the duty cycle we have becasue we have an approximation function only
    *	for each 5% step.
    */

    //this is the percentage of the int_index + 1 coeff we will use
    b_part = float_index - int_index;
    //this is the percentage of the index coeff we will use
    a_part = 1 - b_part;

    //computes the coeff a by taking a percentage of the two coeffs arround the duty_cycle we have
    a_coeff = current_approx_coeffs[int_index].a * a_part + current_approx_coeffs[int_index+1].a * b_part;
    //same with coeff b
    b_coeff = current_approx_coeffs[int_index].b * a_part + current_approx_coeffs[int_index+1].b * b_part;

    //finally computes the approximated current with the a and b coeffs found
	return (a_coeff * brushless_motors[motor_name].current_meter.current + b_coeff);
}

float motorsGetRPM(brushless_motors_names_t motor_name){
	if(motor_name >= NB_OF_BRUSHLESS_MOTOR){
		return 0;
	}
	return brushless_motors[motor_name].rpm_counter.rpm;
}

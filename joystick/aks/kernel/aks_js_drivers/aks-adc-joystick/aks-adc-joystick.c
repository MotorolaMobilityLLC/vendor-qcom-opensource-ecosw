// SPDX-License-Identifier: GPL-2.0
/*
 * Input driver for Aksys.co.kr all in one input device driver
 * Copyright (c) 2022 Daniel <daniel@aksys.co.kr>
 */
#include <linux/ctype.h>
#include <linux/math.h>
#include <linux/input.h>
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/kernel.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <asm/unaligned.h>

//GPIO inlcudes below
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/spinlock.h>
#include <dt-bindings/input/gpio-keys.h>

//cdev
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#define AKS_GAMEPAD_ANALOG_POLL (true)

#define AKS_GAMEPAD_ANALOG_POLL_INTERVAL (10)

#define AKS_GAMEPAD_ENLARGE_SHIFT 		(16)
#define AKS_GAMEPAD_DRAG_SPEED_SHIFT	(3)
#define AKS_GAMEPAD_MT_MAX_HEIGHT	 (1201)
#define AKS_GAMEPAD_MT_MAX_WIDTH	 (1201)

#define AKS_GAMEPAD_WORK_MODE_TOUCH 1
#define AKS_GAMEPAD_WORK_MODE_DIRECT 2

#define AKS_IOC_MAGIC  'k'

#define AKS_IOC_INIT    	_IO(AKS_IOC_MAGIC, 1)
#define AKS_IOC_GET_DATA 	_IOR(AKS_IOC_MAGIC, 2, int)
#define AKS_IOC_SET_DATA 	_IOW(AKS_IOC_MAGIC, 3, int)
#define AKS_IOC_SET_MODE 	_IOW(AKS_IOC_MAGIC, 4, int)
#define AKS_IOC_GET_MODE 	_IOR(AKS_IOC_MAGIC, 5, int)
#define AKS_IOC_SET_MAPPING _IOR(AKS_IOC_MAGIC, 6, char*)
#define AKS_IOC_GET_MAPPING _IOR(AKS_IOC_MAGIC, 7, char*)
#define AKS_IOC_SET_FN 		_IOR(AKS_IOC_MAGIC, 8, char*)
#define AKS_IOC_GET_FN 		_IOR(AKS_IOC_MAGIC, 9, char*)
#define AKS_IOC_SET_CALI 	_IOR(AKS_IOC_MAGIC, 10, char*)
#define AKS_IOC_SET_MACRO 	_IOR(AKS_IOC_MAGIC, 11, char*)
#define AKS_IOC_GET_MACRO 	_IOR(AKS_IOC_MAGIC, 12, char*)
#define AKS_IOC_UPDTAE_MACRO 	_IOR(AKS_IOC_MAGIC, 13, char*)

#define AKS_IOC_MAXNR 13

#define POS_ZERO	(0)
#define POS_MIN		(1)
#define POS_MAX		(2)

#define AKS_GAMEPAD_MT_MAX_CONTACTS  (MT_ID_MAX)

#define AKS_GAMEPAD_BINDABLE_KEYS 21
#define AKS_GAMEPAD_KEYCODE_BITS_MAX 16 //It MUST larger or equals the numbers of enum aks_input_key_bit_offset

#define AKS_CDEV_MAJOR 190
#define AKS_CDEV_NR_DEVS 2
#define AKS_CDEV_SIZE 4096

#define AKS_MAPPING_DATA_MAX_LENGTH 1024

#define AKS_FUNC_KEY_BINDING_F1		KEY_FN	//BTN_A
#define AKS_FUNC_KEY_BINDING_F2		BTN_MODE

#define AKS_INPUT_CDEV_CLASS_MODE ((umode_t)(S_IRUGO |S_IWUGO))

enum aks_input_key_bit_offset {
	KEYCODE_BIT_OFFSET_A     = 0,
	KEYCODE_BIT_OFFSET_B 	 = 1,
	KEYCODE_BIT_OFFSET_X 	 = 2,
	KEYCODE_BIT_OFFSET_Y 	 = 3,
	KEYCODE_BIT_OFFSET_L1 	 = 4,
	KEYCODE_BIT_OFFSET_R1 	 = 5,
	KEYCODE_BIT_OFFSET_L2 	 = 6,
	KEYCODE_BIT_OFFSET_R2 	 = 7,
	KEYCODE_BIT_OFFSET_DU 	 = 8,
	KEYCODE_BIT_OFFSET_DD 	 = 9,
	KEYCODE_BIT_OFFSET_DL 	 = 10,
	KEYCODE_BIT_OFFSET_DR 	 = 11,
	KEYCODE_BIT_OFFSET_L3 	 = 12,
	KEYCODE_BIT_OFFSET_R3 	 = 13,
};

enum aks_cali_option_type {
	CALI_OPTION_LS_CENTER    = 0,
	CALI_OPTION_LS_OFFSET 	 = 1,
	CALI_OPTION_RS_CENTER 	 = 2,
	CALI_OPTION_RS_OFFSET 	 = 3,
	CALI_OPTION_LT_ZERO 	 = 4,
	CALI_OPTION_LT_MAX 	 	 = 5,
	CALI_OPTION_RT_ZERO 	 = 6,
	CALI_OPTION_RT_MAX 	     = 7,
};

enum aks_func_option_type {
	FUNC_OPTION_NONE    	= 0,
	FUNC_OPTION_VOL_UP 	 	= 1,
	FUNC_OPTION_SCR_SHOT	= 2,
	FUNC_OPTION_TURBO		= 3,
	FUNC_OPTION_MOUSE		= 4,
	FUNC_OPTION_SNIPER		= 5,
	FUNC_OPTION_HOME		= 6,
	FUNC_OPTION_CALL_MEDIA	= 7,
	FUNC_OPTION_DEPRECATED	= 8,	//DEPRECATED
	FUNC_OPTION_TV			= 9,	//DEPRECATED
	FUNC_OPTION_VOL_DOWN	= 10,
	FUNC_OPTION_MODE_CHANGE = 11,
};

enum aks_func_key_binding {
	FUNC_KEY_F2    	= 0,
	FUNC_KEY_F1		= 1,
};


enum aks_cali_status_type {
	CALI_STATE_START    = 1,
	CALI_STATE_STOP 	= 0,
	CALI_STATE_CANCEL 	= 3,
	CALI_STATE_SAVE 	= 2,
};

enum aks_input_mt_ids {
	MT_ID_A = 1,
	MT_ID_L1,
	MT_ID_R1,
	MT_ID_L2,
	MT_ID_R2,
	MT_ID_DU,
	MT_ID_DD,
	MT_ID_DL,
	MT_ID_DR,
	MT_ID_L3,
	MT_ID_R3,
	MT_ID_LS,
	MT_ID_RS,
	MT_ID_SELECT,
	MT_ID_START,
	MT_ID_FUNC,
	MT_ID_SYSRQ,
	MT_ID_MODE,
	MT_ID_B,
	MT_ID_X,
	MT_ID_Y,
	MT_ID_MAX,
};
	
struct aks_cdev_data
{
  char *data;
  unsigned long size;
};

struct pair {
	unsigned int keycode;
	int x;
	int y;
	int id;
};

struct kv_pair {
	char key[16];
	char value[32];
};


struct coordinate {
	int x;
	int y;
};

struct keycode_keybit_map {
	unsigned int code;
	enum aks_input_key_bit_offset bit_shift;
	enum aks_input_mt_ids touch_id;
	struct coordinate coord;
};

struct touch_move_limit {
	int w;
	int h;
	int half_w;
	int half_h;
};

struct button_mapping_props {
	int slot;
	int rotation;
	int ls_type;
	int rs_type;
	int ls_size;
	int rs_size;
	unsigned long l_bind_key[1];
	unsigned long r_bind_key[1];
	struct pair ls;
	struct pair rs;
};

struct aks_gpio_button_data {
	struct aks_input_device* aks_dev;
	const struct gpio_keys_button *button;
	struct input_dev *input;
	struct gpio_desc *gpiod;

	unsigned short *code;

	struct hrtimer release_timer;
	unsigned int release_delay;	/* in msecs, for IRQ-only buttons */

	struct delayed_work work;
	struct hrtimer debounce_timer;
	unsigned int software_debounce;	/* in msecs, for GPIO-driven buttons */

	unsigned int irq;
	unsigned int wakeup_trigger_type;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
	bool suspended;
	bool debounce_use_hrtimer;
};

struct aks_gpio_keys_drvdata {
	struct aks_input_device* aks_dev;
	const struct gpio_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex key_mutex;
	struct mutex disable_lock;
	unsigned short *keymap;
	struct aks_gpio_button_data data[];
};

struct aks_analog_cali_data {
    s32 physic_zero_pos;
	s32 logic_zero_pos;
	s32 logic_half_distance;
    s32 rang_min;
    s32 rang_max;
	s32 rang_min_calied;
	s32 rang_max_calied;
    s32 ratio_neg;
    s32 ratio_pos;
	s32 dist_min;
	s32 dist_max;
	s32 enlarge_ratio_min;
	s32 enlarge_ratio_max;
};

struct aks_analog_key_axis {
	u32 code;
	s32 range[2];
	s32 fuzz;
	s32 flat;
    struct aks_analog_cali_data cali;
};

struct aks_analog_key_axis_pos {
	int x;
	int y;
	int z;
	int rz;
};

struct aks_fun_key_info_t {
	bool activated;
	int index;
	int option;	//sniper only
	int data;
	int ls_rate;
	int rs_rate;
	int f1_binding;
	int f2_binding;
	int f1_pressed;
	int f2_pressed;
};


struct aks_analog_key_data {
	struct input_dev *input;
	struct iio_cb_buffer *buffer;
	struct aks_analog_key_axis *axes;
	struct iio_channel *chans;
	struct aks_analog_key_axis_pos z_coord;
	int num_chans;
	bool polled;
};


struct aks_multitouch_data {
	struct aks_input_device *aks_dev;
	struct input_dev *mt_input;
	struct pair mapping_coords[AKS_GAMEPAD_BINDABLE_KEYS];
	struct button_mapping_props  mapping_props;
	int mapping_coords_index;
	int max_touch_num;
	unsigned char raw[AKS_MAPPING_DATA_MAX_LENGTH];
};

struct aks_js_mt_status_recoder {
	int last_x;
	int last_y;
	int last_z;
	int last_rz;
	int last_l2;
	int last_r2;
	int ls_touching;
	int rs_touching;
	int last_touch_x;
	int last_touch_y;
	int last_touch_z;
	int last_touch_rz;
	bool last_binding_ls;
	bool last_binding_rs;
};

struct aks_input_device {
	struct device *dev;
	struct input_dev *input;
	struct input_dev *mt_input;
	struct aks_gpio_button_data *btn_data;
	struct aks_gpio_keys_drvdata *btn_drv_data;
	struct aks_analog_key_data *analog_data;
	struct aks_multitouch_data *mt_data;
	struct aks_js_mt_status_recoder js_record;
	struct mutex mutex;
	struct mutex ipc_mutex;
	int work_mode;
	bool js_polling;
};


struct kv_pair kvs[29];

struct aks_input_device *g_aks_dev;

struct aks_fun_key_info_t aks_fun_key_info;

static int CURRENT_WORK_MODE = AKS_GAMEPAD_WORK_MODE_DIRECT;

static struct touch_move_limit move_limits[] = 
	{
		{76, 150, 38, 76}, \
		{150, 300, 76, 150}, \
		{226, 450, 114, 226}, \
		{300, 600, 150, 300}  \
	};

static struct keycode_keybit_map aks_gamepad_keycode_bit_map[] =
{
	{BTN_A, KEYCODE_BIT_OFFSET_A, MT_ID_A, {0, 0}}, \
	{BTN_B, KEYCODE_BIT_OFFSET_B, MT_ID_B, {0, 0}}, \
	{BTN_X, KEYCODE_BIT_OFFSET_X, MT_ID_X, {0, 0}}, \
	{BTN_Y, KEYCODE_BIT_OFFSET_Y, MT_ID_Y, {0, 0}}, \
	{BTN_TL, KEYCODE_BIT_OFFSET_L1, MT_ID_L1, {0, 0}}, \
	{BTN_TR, KEYCODE_BIT_OFFSET_R1, MT_ID_R1, {0, 0}}, \
	{BTN_DPAD_LEFT, KEYCODE_BIT_OFFSET_DL, MT_ID_DL, {0, 0}}, \
	{BTN_DPAD_RIGHT, KEYCODE_BIT_OFFSET_DR, MT_ID_DR, {0, 0}}, \
	{BTN_DPAD_UP, KEYCODE_BIT_OFFSET_DU, MT_ID_DU, {0, 0}}, \
	{BTN_DPAD_DOWN, KEYCODE_BIT_OFFSET_DD, MT_ID_DD, {0, 0}}, \
	{BTN_THUMBL, KEYCODE_BIT_OFFSET_L3, MT_ID_L3, {0, 0}}, \
	{BTN_THUMBR, KEYCODE_BIT_OFFSET_R3, MT_ID_R3, {0, 0}}, \
	{ABS_BRAKE, KEYCODE_BIT_OFFSET_L2, MT_ID_L2, {0, 0}}, \
	{ABS_GAS, KEYCODE_BIT_OFFSET_R2, MT_ID_R2, {0, 0}}, \
};

static struct aks_analog_key_axis user_calied_axis_lt;
static struct aks_analog_key_axis user_calied_axis_rt;
//static struct aks_analog_key_axis user_calied_axis_x;
//static struct aks_analog_key_axis user_calied_axis_y;
//static struct aks_analog_key_axis user_calied_axis_z;
//static struct aks_analog_key_axis user_calied_axis_rz;


#define AKS_GAMEPAD_KEYCODE_BITS_COUNT (sizeof(aks_gamepad_keycode_bit_map)/sizeof(aks_gamepad_keycode_bit_map[0]))

static DECLARE_BITMAP(aks_gamepad_button_status, AKS_GAMEPAD_KEYCODE_BITS_MAX);

static DECLARE_BITMAP(aks_gamepad_touch_number, AKS_GAMEPAD_KEYCODE_BITS_MAX);

//static unsigned long aks_gamepad_button_status[1];

static inline int js_l_touched(struct aks_analog_key_data *adata, int x, int y) {
	if((abs(x - adata->z_coord.x) < 65) && (abs(y - adata->z_coord.y) < 65)) {
		return 0;
	} else {
		return 1;
	}
}

static inline int js_r_touched(struct aks_analog_key_data *adata, int x, int y) {
	if((abs(x - adata->z_coord.z) < 65) && (abs(y - adata->z_coord.rz) < 65)) {
		return 0;
	} else {
		return 1;
	}
}

static inline int js_l_draging(struct aks_analog_key_data *adata, int x, int y) {
	if((abs(x - adata->z_coord.x) < 75) && (abs(y - adata->z_coord.y) < 75)) {
		return 0;
	} else {
		return 1;
	}
}

static inline int js_r_draging(struct aks_analog_key_data *adata, int x, int y) {
	if((abs(x - adata->z_coord.z) < 75) && (abs(y - adata->z_coord.rz) < 75)) {
		return 0;
	} else {
		return 1;
	}
}


int count_bit_one(unsigned long n)
{
    int count = 0;
    while (n)
    {
        n = n & (n - 1);
        count++;
    }
    return count;
}

static inline int calc_sniper_move_speed(int original, unsigned int x) {

	switch(x) {
		case 0:
			return original; // 100%
			break;
		case 1:
			return ((original*3)>>2); //75%
			break;
		case 2:
			return (original>>1);	//50%
			break;
		case 3:
			return (original>>2);	//25%
			break;
		default:
			dev_err(g_aks_dev->dev, "Unsupported sniper rate: %d", x);
			return x;
			break;
	}
}

static inline int moved(int old_x, int x) {
	return abs(old_x - x) > 5;
}

static inline bool str_starts_with(const char *a, const char *b)
{
	if(strncmp(a, b, strlen(b)) == 0) 
		return 1;
	return 0;
}

static inline bool get_touch_info(struct aks_input_device* adev, unsigned int code, int *x, int *y, int *id)
{
	struct aks_multitouch_data *mt_data = adev->mt_data;
	int i=0;
	if(!mt_data->mt_input)
		return false;

	for(i=0; i<AKS_GAMEPAD_BINDABLE_KEYS; i++) {
		if(mt_data->mapping_coords[i].keycode == code) {
			*id = mt_data->mapping_coords[i].id;
			*x= mt_data->mapping_coords[i].x;
			*y= mt_data->mapping_coords[i].y;
			return true;
		}
	}

	return false;
}

static int touched_num = 0;

static void aks_gamepad_mt_report_touch_event(struct aks_input_device *adev, int x, int y, int id, int state)
{

	if(adev->mt_input != NULL && id != 0) {
		if(adev->mt_data->mapping_props.rotation == 0) {
			swap(x, y);
		}

		if(state) {
			input_mt_slot(adev->mt_input, id);
			input_mt_report_slot_state(adev->mt_input, MT_TOOL_FINGER, true);
			input_report_abs(adev->mt_input, ABS_MT_POSITION_X, x);
			input_report_abs(adev->mt_input, ABS_MT_POSITION_Y, y);
			input_report_abs(adev->mt_input, ABS_MT_TOUCH_MAJOR, 0x6000);
		} else {
			input_mt_slot(adev->mt_input, id);
			input_mt_report_slot_state(adev->mt_input, MT_TOOL_FINGER, false);
		}
		state ? set_bit(id-1, aks_gamepad_touch_number) : clear_bit(id-1, aks_gamepad_touch_number);
	} else {
		dev_err(adev->dev, "[%d]Warning, Ignored data adev->mt_input=%p id=%d)\n", __LINE__, adev->mt_input, id);
	}
}

static int find_keycode_shift_bit_pos(unsigned int code) {
	int i=0;
	for(i=0; i<AKS_GAMEPAD_KEYCODE_BITS_COUNT; i++) {
		if(code == aks_gamepad_keycode_bit_map[i].code) {
			return aks_gamepad_keycode_bit_map[i].bit_shift;
		}
	}
	return -1;
}

static int find_keycode_bit_map_index_by_code(unsigned int code) {
	int i=0;
	for(i=0; i<AKS_GAMEPAD_KEYCODE_BITS_COUNT; i++) {
		if(code == aks_gamepad_keycode_bit_map[i].code) {
			return i;
		}
	}
	return -1;
}


static int set_keycode_shift_bit_map(unsigned int code, int state) {
	int bit_pos = find_keycode_shift_bit_pos(code);
	if(bit_pos != -1) {
		state ? set_bit(bit_pos, aks_gamepad_button_status) : clear_bit(bit_pos, aks_gamepad_button_status);
		return 0;
	}
	return -1;
}

static int get_active_bind_key_coord(int val, struct pair *coord) {
	int i=0;
	for(i=0; i<AKS_GAMEPAD_KEYCODE_BITS_COUNT; i++) {
		if(val == (1<<aks_gamepad_keycode_bit_map[i].bit_shift)) {
			coord->x=aks_gamepad_keycode_bit_map[i].coord.x;
			coord->y=aks_gamepad_keycode_bit_map[i].coord.y;
			coord->id=aks_gamepad_keycode_bit_map[i].touch_id;
			return 0;
		}
	}
	return -1;
}

static void aks_gamepad_handle_button_to_touch(struct aks_input_device *adev, unsigned int code, int state)
{
	int x, y, id;
	if(get_touch_info(adev, code, &x, &y, &id)) {
		set_keycode_shift_bit_map(code, state);
		aks_gamepad_mt_report_touch_event(adev, x, y, id, state);
	} else {
		dev_err(adev->dev, "Analog Button %d mapping error", code);
	}
}

static int calculate_target_coordinate(bool invert, int js_pos, int js_center, int last_touch_pos, int ratio, int ui_center, int ui_hf_limite, int keycode)
{
	int rratio, target;
	if(invert) {
		rratio = (js_pos - js_center) * ratio;
	} else {
		rratio = (js_center - js_pos) * ratio;
	}
	target  = ui_center + ((rratio * ui_hf_limite) >> AKS_GAMEPAD_ENLARGE_SHIFT);
	return target;
}

static int calculate_drag_position(bool invert, int js_pos, int js_center, int last_touch_pos, int ui_center, int speed, int ratio, struct aks_input_device *adev,  int keycode, void(*should_reset)(struct aks_input_device *, bool, int))
{
	int rratio, target, scale;
	if(invert) {
		rratio = (js_pos - js_center) * ratio;
	} else {
		rratio = (js_center - js_pos) * ratio;
	}

	if(aks_fun_key_info.f1_pressed) {
		if(keycode == ABS_X || keycode ==ABS_Y) {
			rratio = calc_sniper_move_speed(rratio, aks_fun_key_info.ls_rate);
		}
		if(keycode == ABS_Z || keycode ==ABS_RZ) {
			rratio = calc_sniper_move_speed(rratio, aks_fun_key_info.rs_rate);
		}
	}

	if(keycode == ABS_X || keycode ==ABS_Z) {
		scale = AKS_GAMEPAD_DRAG_SPEED_SHIFT-1;
	} else {
		scale = AKS_GAMEPAD_DRAG_SPEED_SHIFT;
	}
	target  = last_touch_pos + ((rratio * ((speed+1)<<scale)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
	if(target < 0 ||  target > AKS_GAMEPAD_MT_MAX_HEIGHT) {
		target = ui_center;
		should_reset(adev, true, keycode);
	}
	return target;
}


static inline bool js_enabled(int a, int b) {
	return (a!=0 && b!=0);
}

struct pair coord_ls, coord_rs;

static void reset_drag_touch_postion( struct aks_input_device *adev, bool reset, int key_code) {
	switch(key_code) {
		case ABS_X:
		case ABS_Y:
			aks_gamepad_mt_report_touch_event(adev, adev->js_record.last_touch_x, adev->js_record.last_touch_x, MT_ID_LS, false);
			break;
		case ABS_Z:
		case ABS_RZ:
			aks_gamepad_mt_report_touch_event(adev, adev->js_record.last_touch_z, adev->js_record.last_touch_rz, MT_ID_RS, false);
			break;
		default:
			break;
	}
}

static int aks_gamepad_handle_js_to_touch(struct aks_input_device *adev) {
	struct aks_analog_key_axis *axis;
	struct aks_js_mt_status_recoder *record = &adev->js_record;
	struct button_mapping_props* mapping_props = &adev->mt_data->mapping_props;

	int trigger_state, ls_state;
	s32 i, val, ratio;
	bool ls_enabled = false,  rs_enabled = false;
	int center_x, center_y, id_l;
	int act_bind_ls ,id_r, center_z, center_rz, act_bind_rs, rs_state;

	ls_enabled = (js_enabled(mapping_props->ls.x, mapping_props->ls.y) || mapping_props->l_bind_key[0] != 0);
	rs_enabled = (js_enabled(mapping_props->rs.x, mapping_props->rs.y) || mapping_props->r_bind_key[0] != 0);

	mutex_lock(&adev->mutex);

	for (i = 0; i < adev->analog_data->num_chans; i++) {
		axis = &adev->analog_data->axes[i];
		iio_read_channel_raw(&adev->analog_data->chans[i], &val);

		switch(axis->code) {
			case ABS_X:
				if(!ls_enabled) {
					continue;
				}
				center_x = mapping_props->ls.x;
				id_l = MT_ID_LS;

				if(mapping_props->ls_type == 2) {
					record->last_x = val;
					if(mapping_props->l_bind_key[0] != 0) {
						act_bind_ls = (mapping_props->l_bind_key[0] & aks_gamepad_button_status[0]);
						if(act_bind_ls) {
							if(get_active_bind_key_coord(act_bind_ls, &coord_ls) == 0) {
								id_l = coord_ls.id;
								center_x = coord_ls.x;
								if(!record->last_binding_ls) {
									aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, MT_ID_LS, false);
									record->last_touch_x = coord_ls.x;
									record->last_touch_y = coord_ls.y;
									record->last_binding_ls = true;
								}
							}
						} else {
							if(record->last_binding_ls) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, coord_ls.id, false);
								record->last_touch_x = mapping_props->ls.x;
								record->last_touch_y = mapping_props->ls.y;
								record->last_binding_ls = false;
							}
						}
					}

					if(record->last_binding_ls || js_l_draging(adev->analog_data, record->last_x, record->last_y)) {
						if(record->ls_touching == 0) {
							record->ls_touching = 1;
						}
						ls_state = 1;
						ratio = (axis->cali.physic_zero_pos - val > 0) ? (axis->cali.enlarge_ratio_max) : axis->cali.enlarge_ratio_min;
						record->last_touch_x = calculate_drag_position(false, val, axis->cali.physic_zero_pos,\
							record->last_touch_x, center_x, mapping_props->ls_size, ratio, adev, ABS_X, reset_drag_touch_postion);
					} else {
						if(record->ls_touching == 1) {
							record->ls_touching = 0;
							record->last_touch_x = mapping_props->ls.x;
							record->last_touch_y = mapping_props->ls.y;
						}
						ls_state = 0;
					}
				}
				else if(mapping_props->ls_type == 0) {
					if(mapping_props->l_bind_key[0] != 0) {
						act_bind_ls = (mapping_props->l_bind_key[0] & aks_gamepad_button_status[0]);
						if(act_bind_ls) {
							if(get_active_bind_key_coord(act_bind_ls, &coord_ls) == 0) {
								id_l = coord_ls.id;
								center_x = coord_ls.x;
								if(!record->last_binding_ls) {
									aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, MT_ID_LS, false);
									record->last_touch_x = coord_ls.x;
									record->last_touch_y = coord_ls.y;
									record->last_binding_ls = true;
								}
							}
						} else {
							if(record->last_binding_ls) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, coord_ls.id, false);
								record->last_touch_x = mapping_props->ls.x;
								record->last_touch_y = mapping_props->ls.y;
								record->last_binding_ls = false;
							}
						}
					}

					if(moved(record->last_x, val)) {
						record->last_x = val;
						if(record->last_binding_ls || js_l_touched(adev->analog_data, record->last_x, record->last_y)) {
							if(record->ls_touching == 0) {
								record->ls_touching = 1;
							}
							ls_state = 1;
							ratio = (axis->cali.physic_zero_pos - val > 0) ? (axis->cali.enlarge_ratio_max) : axis->cali.enlarge_ratio_min;
							record->last_touch_x = calculate_target_coordinate(false, val, axis->cali.physic_zero_pos, record->last_touch_x, ratio, center_x, move_limits[mapping_props->ls_size].half_w, ABS_X);
						} else {
							if(record->ls_touching == 1) {
								record->ls_touching = 0;
								record->last_touch_x = mapping_props->ls.x;
								record->last_touch_y = mapping_props->ls.y;
							}
							ls_state = 0;
						}
						aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, id_l, ls_state);
					}
				}
				break;
			case ABS_Y:
				if(!ls_enabled)
					continue;
				center_y = mapping_props->ls.y;
				id_l = MT_ID_LS;

				if(mapping_props->ls_type == 2) {
					record->last_y = val;
					if(mapping_props->l_bind_key[0] != 0) {
						act_bind_ls = (mapping_props->l_bind_key[0] & aks_gamepad_button_status[0]);
						if(act_bind_ls) {
							if(get_active_bind_key_coord(act_bind_ls, &coord_ls) == 0) {
								id_l = coord_ls.id;
								center_y = coord_ls.y;
								if(!record->last_binding_ls) {
									aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, MT_ID_LS, false);
									record->last_touch_x = coord_ls.x;
									record->last_touch_y = coord_ls.y;
									record->last_binding_ls = true;
								}
							}
						} else {
							if(record->last_binding_ls) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, coord_ls.id, false);
								record->last_touch_x = mapping_props->ls.x;
								record->last_touch_y = mapping_props->ls.y;
								record->last_binding_ls = false;
							}
						}
					}

					if(record->last_binding_ls || js_l_draging(adev->analog_data, record->last_x, record->last_y)) {
						if(record->ls_touching == 0) {
							record->ls_touching = 1;
						}
						ls_state = 1;
						ratio = (axis->cali.physic_zero_pos - val > 0) ? (axis->cali.enlarge_ratio_max) : axis->cali.enlarge_ratio_min;
						record->last_touch_y = calculate_drag_position(false, val, axis->cali.physic_zero_pos, \
							record->last_touch_y, center_y, mapping_props->ls_size, ratio, adev, ABS_Y, reset_drag_touch_postion);
					} else {
						if(record->ls_touching == 1) {
							record->ls_touching = 0;
							record->last_touch_x = mapping_props->ls.x;
							record->last_touch_y = mapping_props->ls.y;
						}
						ls_state = 0;
					}
				}
				else if(mapping_props->ls_type == 0) {
					if(mapping_props->l_bind_key[0] != 0) {
						act_bind_ls = (mapping_props->l_bind_key[0] & aks_gamepad_button_status[0]);
						if(act_bind_ls) {
							if(get_active_bind_key_coord(act_bind_ls, &coord_ls) == 0) {
								id_l = coord_ls.id;
								center_y = coord_ls.y;
								if(!record->last_binding_ls) {
									aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, MT_ID_LS, false);
									record->last_touch_x = coord_ls.x;
									record->last_touch_y = coord_ls.y;
									record->last_binding_ls = true;
								}
							}
						} else {
							if(record->last_binding_ls) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, coord_ls.id, false);
								record->last_touch_x = mapping_props->ls.x;
								record->last_touch_y = mapping_props->ls.y;
								record->last_binding_ls = false;
							}
						}
					}
					if(moved(record->last_y, val)) {
						record->last_y = val;
						if(record->last_binding_ls || js_l_touched(adev->analog_data, record->last_x, record->last_y)) {
							if(record->ls_touching == 0) {
								record->ls_touching = 1;
								dev_err(adev->dev, "[%d]Touch(id=%d) statue 0 ->  (%d)\n", __LINE__, id_l, record->ls_touching);
							}
							ls_state = 1;
							ratio = (axis->cali.physic_zero_pos - val > 0) ? axis->cali.enlarge_ratio_max : (axis->cali.enlarge_ratio_min);
							record->last_touch_y = calculate_target_coordinate(false, val, axis->cali.physic_zero_pos, record->last_touch_y, ratio, center_y, move_limits[mapping_props->ls_size].half_h, ABS_Y);
						} else {
							if(record->ls_touching == 1) {
								record->ls_touching = 0;
								record->last_touch_x = mapping_props->ls.x;
								record->last_touch_y = mapping_props->ls.y;
								dev_err(adev->dev, "[%d]Touch(id=%d) statue 1 ->  (%d)\n", __LINE__, id_l, record->ls_touching);
							}
							ls_state = 0;
						}
						aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, id_l, ls_state);
					}
				}

				break;

			case ABS_Z:
				if(!rs_enabled)
					continue;

				center_z = mapping_props->rs.x;
				id_r = MT_ID_RS;

				if(mapping_props->rs_type == 2) {
					record->last_z = val;
					act_bind_rs = (mapping_props->r_bind_key[0] & aks_gamepad_button_status[0]);
					if(act_bind_rs) {
						if(get_active_bind_key_coord(act_bind_rs, &coord_rs) == 0) {
							id_r = coord_rs.id;
							center_z = coord_rs.x;
							if(!record->last_binding_rs) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, MT_ID_RS, false);
								record->last_touch_z = coord_rs.x;
								record->last_touch_rz = coord_rs.y;
								record->last_binding_rs = true;
							}
						}
					} else {
						if(record->last_binding_rs) {
							aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, coord_rs.id, false);
							record->last_touch_z = mapping_props->rs.x;
							record->last_touch_rz = mapping_props->rs.y;
							record->last_binding_rs = false;
						}
					}

					if(record->last_binding_rs || js_r_draging(adev->analog_data, record->last_z, record->last_rz)) {
						if(record->rs_touching == 0) {
							record->rs_touching = 1;
						}
						rs_state = 1;
						ratio = (axis->cali.physic_zero_pos - val > 0) ? (axis->cali.enlarge_ratio_max) : axis->cali.enlarge_ratio_min;
						record->last_touch_z = calculate_drag_position(true, val, axis->cali.physic_zero_pos,\
							record->last_touch_z, center_z, mapping_props->rs_size, ratio, adev, ABS_Z, reset_drag_touch_postion);
					} else {
						if(record->rs_touching == 1) {
							record->rs_touching = 0;
							record->last_touch_z = mapping_props->rs.x;
							record->last_touch_rz = mapping_props->rs.y;
						}
						rs_state = 0;
					}
				} else if(mapping_props->rs_type == 0) {
					if(mapping_props->r_bind_key[0] != 0) {
						act_bind_rs = (mapping_props->r_bind_key[0] & aks_gamepad_button_status[0]);
						if(act_bind_rs) {
							if(get_active_bind_key_coord(act_bind_rs, &coord_rs) == 0) {
								id_r = coord_rs.id;
								center_z = coord_rs.x;
								if(!record->last_binding_rs) {
									aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, MT_ID_RS, false);
									record->last_touch_z = coord_rs.x;
									record->last_touch_rz = coord_rs.y;
									record->last_binding_rs = true;
								}
							}
						} else {
							if(record->last_binding_rs) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, coord_rs.id, false);
								record->last_touch_z = mapping_props->rs.x;
								record->last_touch_rz = mapping_props->rs.y;
								record->last_binding_rs = false;
							}
						}
					}

					if(moved(record->last_z, val)) {
						record->last_z = val;
						if(record->last_binding_rs || js_r_touched(adev->analog_data, record->last_z, record->last_rz) )  {
							if(record->rs_touching == 0) {
								record->rs_touching = 1;
							}
							rs_state = 1;
							ratio = (axis->cali.physic_zero_pos - val > 0) ? axis->cali.enlarge_ratio_max : axis->cali.enlarge_ratio_min;
							record->last_touch_z = calculate_target_coordinate(true, val, axis->cali.physic_zero_pos, record->last_touch_z, ratio, center_z, move_limits[mapping_props->rs_size].half_w, ABS_Z);
						} else {
							if(record->rs_touching == 1) {
								record->rs_touching = 0;
								record->last_touch_z = mapping_props->rs.x;
								record->last_touch_rz = mapping_props->rs.y;
							}
							rs_state = 0;
						}
						aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, id_r, rs_state);
					}
				}
				break;
			case ABS_RZ:
				if(!rs_enabled)
					continue;
				center_rz = mapping_props->rs.y;
				id_r = MT_ID_RS;
				if(mapping_props->rs_type == 2) {
					record->last_rz = val;
					act_bind_rs = (mapping_props->r_bind_key[0] & aks_gamepad_button_status[0]);
					if(act_bind_rs) {
						if(get_active_bind_key_coord(act_bind_rs, &coord_rs) == 0) {
							id_r = coord_rs.id;
							center_rz = coord_rs.y;
							if(!record->last_binding_rs) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, MT_ID_RS, false);
								record->last_touch_z = coord_rs.x;
								record->last_touch_rz = coord_rs.y;
								record->last_binding_rs = true;
							}
						}
					} else {
						if(record->last_binding_rs) {
							aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, coord_rs.id, false);
							record->last_touch_z = mapping_props->rs.x;
							record->last_touch_rz = mapping_props->rs.y;
							record->last_binding_rs = false;
						}
					}
					if(record->last_binding_rs || js_r_draging(adev->analog_data, record->last_z, record->last_rz)) {
						if(record->rs_touching == 0) {
							record->rs_touching = 1;
						}
						rs_state = 1;
						ratio = (axis->cali.physic_zero_pos - val > 0) ? (axis->cali.enlarge_ratio_max) : axis->cali.enlarge_ratio_min;
						record->last_touch_rz = calculate_drag_position(true, val, axis->cali.physic_zero_pos, \
							record->last_touch_rz, center_rz, mapping_props->rs_size, ratio, adev, ABS_RZ, reset_drag_touch_postion);
					} else {
						if(record->rs_touching == 1) {
							record->rs_touching = 0;
							record->last_touch_z = mapping_props->rs.x;
							record->last_touch_rz = mapping_props->rs.y;
						}
						rs_state = 0;
					}
				} else if(mapping_props->rs_type == 0) {
					if(mapping_props->r_bind_key[0] != 0) {
						act_bind_rs = (mapping_props->r_bind_key[0] & aks_gamepad_button_status[0]);
						if(act_bind_rs) {
							if(get_active_bind_key_coord(act_bind_rs, &coord_rs) == 0) {
								id_r = coord_rs.id;
								center_rz = coord_rs.y;
								if(!record->last_binding_rs) {
									aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, MT_ID_RS, false);
									record->last_touch_z = coord_rs.x;
									record->last_touch_rz = coord_rs.y;
									record->last_binding_rs = true;
								}
							}
						} else {
							if(record->last_binding_rs) {
								aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, coord_rs.id, false);
								record->last_touch_z = mapping_props->rs.x;
								record->last_touch_rz = mapping_props->rs.y;
								record->last_binding_rs = false;
							}
						}
					}

					if(moved(record->last_rz, val)) {
						record->last_rz = val;
						if(record->last_binding_rs || js_r_touched(adev->analog_data, record->last_z, record->last_rz)) {
							if(record->rs_touching == 0) {
								record->rs_touching = 1;
								dev_err(adev->dev, "[%d]Touch(id=%d) statue 0 ->  (%d)\n", __LINE__, id_r, record->rs_touching);
							}
							rs_state = 1;
							ratio = (axis->cali.physic_zero_pos - val > 0) ? axis->cali.enlarge_ratio_max : axis->cali.enlarge_ratio_min;
							record->last_touch_rz = calculate_target_coordinate(true, val, axis->cali.physic_zero_pos, record->last_touch_rz, ratio, center_rz, move_limits[mapping_props->rs_size].half_h, ABS_RZ);
						} else {
							if(record->rs_touching == 1) {
								record->rs_touching = 0;
								record->last_touch_z = mapping_props->rs.x;
								record->last_touch_rz = mapping_props->rs.y;
								dev_err(adev->dev, "[%d]Touch(id=%d) statue 1 -> (%d)\n", __LINE__, id_r, record->rs_touching);
							}
							rs_state = 0;
						}
						aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, id_r, rs_state);
					}
				}
				break;
			case ABS_BRAKE:
				trigger_state = (abs(val - axis->cali.physic_zero_pos)  > 20);
				if(record->last_l2 != trigger_state) {
					record->last_l2 = trigger_state;
					aks_gamepad_handle_button_to_touch(adev, ABS_BRAKE, trigger_state);
				}
				break;
			case ABS_GAS:
				trigger_state = (abs(val - axis->cali.physic_zero_pos)  > 20);
				if(record->last_r2 != trigger_state) {
					record->last_r2 = trigger_state;
					aks_gamepad_handle_button_to_touch(adev, ABS_GAS, trigger_state);
				}
				break;
			default:
				break;
		}
	}

	if(mapping_props->ls_type == 2) {
		aks_gamepad_mt_report_touch_event(adev, record->last_touch_x, record->last_touch_y, id_l, ls_state);
	}

	if(mapping_props->rs_type == 2) {
		aks_gamepad_mt_report_touch_event(adev, record->last_touch_z, record->last_touch_rz, id_r, rs_state);
	}

	input_report_key(adev->mt_input, BTN_TOUCH, aks_gamepad_touch_number[0] ? 1 : 0);
	input_sync(adev->mt_input);

	mutex_unlock(&adev->mutex);

	return 0;
}

static void aks_gamepad_gpio_keys_quiesce_key(void *data)
{
	struct aks_gpio_button_data *bdata = data;

	if (!bdata->gpiod)
		hrtimer_cancel(&bdata->release_timer);
	if (bdata->debounce_use_hrtimer)
		hrtimer_cancel(&bdata->debounce_timer);
	else
		cancel_delayed_work_sync(&bdata->work);
}

static bool only_key_mode(unsigned int code)
{
	if(code == BTN_MODE || code == KEY_SYSRQ || code == KEY_HOME || code == AKS_FUNC_KEY_BINDING_F1) {
		return true;
	} else {
		return false;
	}
}

static void aks_gamepad_gpio_keys_gpio_report_event(struct aks_gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct aks_input_device* aks_dev = bdata->aks_dev;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state;

	state = bdata->debounce_use_hrtimer ?
			gpiod_get_value(bdata->gpiod) :
			gpiod_get_value_cansleep(bdata->gpiod);
	if (state < 0) {
		dev_err(input->dev.parent, "Error when gpiod get value, return state=%d\n", state);
		return;
	}

	if(aks_dev->work_mode == AKS_GAMEPAD_WORK_MODE_TOUCH && !(only_key_mode(button->code))) {
		if(aks_dev->mt_input && type != EV_ABS) { //Ignore the abs events for gpio
			mutex_lock(&aks_dev->mt_input->mutex);
			aks_gamepad_handle_button_to_touch(aks_dev, button->code, state);
			input_report_key(aks_dev->mt_input, BTN_TOUCH, aks_gamepad_touch_number[0] ? 1 : 0);
			input_sync(aks_dev->mt_input);
			mutex_unlock(&aks_dev->mt_input->mutex);
		} else {
			dev_err(input->dev.parent, "Touch device not ready, ignore events\n");
		}
	} else {
		if (type == EV_ABS) {
			if (state)
				input_event(input, type, button->code, button->value);
		} else {
			//dev_err(input->dev.parent, "Code=%d, state=%d\n", *bdata->code, state);

			if(state) {
				switch(button->code) {
					case BTN_DPAD_UP:
						input_report_abs(input, ABS_HAT0Y, 99);
						input_sync(input);
						break;
					case BTN_DPAD_DOWN:
						input_report_abs(input, ABS_HAT0Y, 1);
						input_sync(input);
						break;
					case BTN_DPAD_LEFT:
						input_report_abs(input, ABS_HAT0X, 1);
						input_sync(input);
						break;
					case BTN_DPAD_RIGHT:
						input_report_abs(input, ABS_HAT0X, 99);
						input_sync(input);
						break;
					case AKS_FUNC_KEY_BINDING_F1:
						if(aks_fun_key_info.activated) {
							aks_fun_key_info.f1_pressed = true;
						}
						break;
					default:
						break;
				}
			}else {
				switch(button->code) {
					case BTN_DPAD_UP:
					case BTN_DPAD_DOWN:
						input_report_abs(input, ABS_HAT0Y, 50);
						input_sync(input);
						break;
					case BTN_DPAD_LEFT:
					case BTN_DPAD_RIGHT:
						input_report_abs(input, ABS_HAT0X, 50);
						input_sync(input);
						break;
					case AKS_FUNC_KEY_BINDING_F1:
						if(aks_fun_key_info.activated) {
							aks_fun_key_info.f1_pressed = false;
						}
						break;
					default:
						break;
				}
			}
			input_event(input, type, *bdata->code, state);
		}
	}
}

static void aks_gamepad_gpio_keys_debounce_event(struct aks_gpio_button_data *bdata)
{
	aks_gamepad_gpio_keys_gpio_report_event(bdata);
	input_sync(bdata->input);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static void aks_gamepad_gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct aks_gpio_button_data *bdata =
		container_of(work, struct aks_gpio_button_data, work.work);

	aks_gamepad_gpio_keys_debounce_event(bdata);
}

static enum hrtimer_restart aks_gamepad_gpio_keys_debounce_timer(struct hrtimer *t)
{
	struct aks_gpio_button_data *bdata =
		container_of(t, struct aks_gpio_button_data, debounce_timer);

	aks_gamepad_gpio_keys_debounce_event(bdata);

	return HRTIMER_NORESTART;
}

static irqreturn_t aks_gamepad_gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct aks_gpio_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup) {
		const struct gpio_keys_button *button = bdata->button;

		pm_stay_awake(bdata->input->dev.parent);
		if (bdata->suspended  &&
		    (button->type == 0 || button->type == EV_KEY)) {
			/*
			 * Simulate wakeup key press in case the key has
			 * already released by the time we got interrupt
			 * handler to run.
			 */
			input_report_key(bdata->input, button->code, 1);
		}
	}

	if (bdata->debounce_use_hrtimer) {
		hrtimer_start(&bdata->debounce_timer,
			      ms_to_ktime(bdata->software_debounce),
			      HRTIMER_MODE_REL);
	} else {
		mod_delayed_work(system_wq,
				 &bdata->work,
				 msecs_to_jiffies(bdata->software_debounce));
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart aks_gamepad_gpio_keys_irq_timer(struct hrtimer *t)
{
	struct aks_gpio_button_data *bdata = container_of(t,
						      struct aks_gpio_button_data,
						      release_timer);
	struct input_dev *input = bdata->input;

	if (bdata->key_pressed) {
		input_event(input, EV_KEY, *bdata->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}

	return HRTIMER_NORESTART;
}

static irqreturn_t aks_gamepad_gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct aks_gpio_button_data *bdata = dev_id;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		if (bdata->button->wakeup)
			pm_wakeup_event(bdata->input->dev.parent, 0);

		input_event(input, EV_KEY, *bdata->code, 1);
		input_sync(input);

		if (!bdata->release_delay) {
			input_event(input, EV_KEY, *bdata->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->release_delay)
		hrtimer_start(&bdata->release_timer,
			      ms_to_ktime(bdata->release_delay),
			      HRTIMER_MODE_REL_HARD);
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}


static int aks_gamepad_gpio_keys_setup_key(struct device *dev,
				struct input_dev *input,
				struct aks_gpio_keys_drvdata *ddata,
				const struct gpio_keys_button *button,
				int idx,
				struct fwnode_handle *child)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct aks_gpio_button_data *bdata = &ddata->data[idx];
	irq_handler_t isr;
	unsigned long irqflags;
	int irq;
	int error;
	unsigned flags;

	bdata->input = input;
	bdata->button = button;
	bdata->aks_dev = ddata->aks_dev;
	spin_lock_init(&bdata->lock);
	//dev_err(dev, "aks_gamepad_gpio_keys_setup_key, %s\n", desc);

	if (child) {
		bdata->gpiod = devm_fwnode_gpiod_get(dev, child, NULL, GPIOD_IN, desc);
		//bdata->gpiod = devm_fwnode_get_gpiod_from_child(dev, NULL, child, GPIOD_IN, desc);
		if (IS_ERR(bdata->gpiod)) {
			error = PTR_ERR(bdata->gpiod);
			if (error == -ENOENT) {
				/*
				 * GPIO is optional, we may be dealing with
				 * purely interrupt-driven setup.
				 */
				bdata->gpiod = NULL;
			} else {
				if (error != -EPROBE_DEFER)
					dev_err(dev, "failed to get gpio: %d\n", error);
				return error;
			}
		}
	} else if (gpio_is_valid(button->gpio)) {
		/*
		 * Legacy GPIO number, so request the GPIO here and
		 * convert it to descriptor.
		 */
		 
		dev_err(dev, "button->gpio = %d\n", button->gpio);
		flags = GPIOF_IN;

		if (button->active_low)
			flags |= GPIOF_ACTIVE_LOW;

		error = devm_gpio_request_one(dev, button->gpio, flags, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		bdata->gpiod = gpio_to_desc(button->gpio);
		if (!bdata->gpiod)
			return -EINVAL;
	}

	if (bdata->gpiod) {
		bool active_low = gpiod_is_active_low(bdata->gpiod);
		//dev_err(g_aks_dev->dev, "%s------(%d) active low: %d\n",__FUNCTION__, __LINE__, active_low);
		if (button->debounce_interval) {
			error = gpiod_set_debounce(bdata->gpiod, button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->software_debounce = button->debounce_interval;

			/*
			 * If reading the GPIO won't sleep, we can use a
			 * hrtimer instead of a standard timer for the software
			 * debounce, to reduce the latency as much as possible.
			 */
			bdata->debounce_use_hrtimer = !gpiod_cansleep(bdata->gpiod);
		}

		if (button->irq) {
			bdata->irq = button->irq;
		} else {
			irq = gpiod_to_irq(bdata->gpiod);
			if (irq < 0) {
				error = irq;
				dev_err(dev,
					"Unable to get irq number for GPIO %d, error %d\n",
					button->gpio, error);
				return error;
			}
			bdata->irq = irq;
		}

		INIT_DELAYED_WORK(&bdata->work, aks_gamepad_gpio_keys_gpio_work_func);

		hrtimer_init(&bdata->debounce_timer,
			     CLOCK_REALTIME, HRTIMER_MODE_REL);
		bdata->debounce_timer.function = aks_gamepad_gpio_keys_debounce_timer;

		isr = aks_gamepad_gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

		switch (button->wakeup_event_action) {
		case EV_ACT_ASSERTED:
			bdata->wakeup_trigger_type = active_low ?
				IRQ_TYPE_EDGE_FALLING : IRQ_TYPE_EDGE_RISING;
			break;
		case EV_ACT_DEASSERTED:
			bdata->wakeup_trigger_type = active_low ?
				IRQ_TYPE_EDGE_RISING : IRQ_TYPE_EDGE_FALLING;
			break;
		case EV_ACT_ANY:
		default:
			/*
			 * For other cases, we are OK letting suspend/resume
			 * not reconfigure the trigger type.
			 */
			break;
		}
	} else {
		if (!button->irq) {
			dev_err(dev, "Found button without gpio or irq\n");
			return -EINVAL;
		}

		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->release_delay = button->debounce_interval;
		hrtimer_init(&bdata->release_timer,
			     CLOCK_REALTIME, HRTIMER_MODE_REL_HARD);
		bdata->release_timer.function = aks_gamepad_gpio_keys_irq_timer;

		isr = aks_gamepad_gpio_keys_irq_isr;
		irqflags = 0;

		/*
		 * For IRQ buttons, there is no interrupt for release.
		 * So we don't need to reconfigure the trigger type for wakeup.
		 */
	}

	bdata->code = &ddata->keymap[idx];
	*bdata->code = button->code;
	input_set_capability(input, button->type ?: EV_KEY, *bdata->code);

	/*
	 * Install custom action to cancel release timer and
	 * workqueue item.
	 */
	error = devm_add_action(dev, aks_gamepad_gpio_keys_quiesce_key, bdata);
	if (error) {
		dev_err(dev, "failed to register quiesce action, error: %d\n",
			error);
		return error;
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = devm_request_any_context_irq(dev, bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static inline bool valid_gpio_key_node(const char* node_name) {
	if(str_starts_with(node_name, "btn") || str_starts_with(node_name, "key")) {
		return true;
	}
	return false;
}

static struct gpio_keys_platform_data *
aks_gamepad_gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct gpio_keys_platform_data *pdata;
	struct gpio_keys_button *button;
	struct fwnode_handle *child;
	const char* node_name;
	int nbuttons = 0;
	int idx= 0;

	device_for_each_child_node(dev, child) {
		fwnode_property_read_string(child, "name", &node_name);
		
		//dev_err(dev, "node_name = %s\n", node_name);
		if(valid_gpio_key_node(node_name)) {
			nbuttons++;
		}
	}

	if (nbuttons == 0) {
		return ERR_PTR(-ENODEV);
		dev_err(dev, "got sub node count NULL\n");
	}

	//dev_err(dev, "got sub node count = %d\n", nbuttons);

	pdata = devm_kzalloc(dev, sizeof(*pdata) + nbuttons * sizeof(*button), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	button = (struct gpio_keys_button *)(pdata + 1);

	pdata->buttons = button;
	pdata->nbuttons = nbuttons;

	pdata->rep = device_property_read_bool(dev, "autorepeat");

	device_property_read_string(dev, "label", &pdata->name);

	//dev_err(dev, "--label=%s, rep=%d\n", pdata->name, pdata->rep);
	device_for_each_child_node(dev, child) {
		fwnode_property_read_string(child, "name", &node_name);
		if(!valid_gpio_key_node(node_name)) {
			//dev_err(dev, "Ignore node: %s\n", node_name);
			continue;
		}

		if (is_of_node(child)) {
			button->irq = irq_of_parse_and_map(to_of_node(child), 0);
		}

		if (fwnode_property_read_u32(child, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		fwnode_property_read_string(child, "label", &button->desc);

		if (fwnode_property_read_u32(child, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = 
			fwnode_property_read_bool(child, "wakeup-source") ||
			/* legacy name */
			fwnode_property_read_bool(child, "gpio-key,wakeup");

		fwnode_property_read_u32(child, "wakeup-event-action", &button->wakeup_event_action);

		button->can_disable = fwnode_property_read_bool(child, "linux,can-disable");

		if (fwnode_property_read_u32(child, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
		//dev_err(dev, "--[%d] irq=%d, keycode=%d, label=%s, type=%d\n", idx, button->irq, button->code, button->desc, button->type);
		button++;
		idx++;
	}
	return pdata;
}

static int aks_gamepad_config_gpio_keys(struct aks_input_device* aks_dev) {
	struct gpio_keys_platform_data *pdata = NULL;
	struct device* dev = aks_dev->dev;

	struct fwnode_handle *child = NULL;
	struct aks_gpio_keys_drvdata *ddata;
	struct input_dev *input = aks_dev->input;
	const char* node_name;
	int i, error;
	int wakeup = 0;

	pdata = aks_gamepad_gpio_keys_get_devtree_pdata(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	ddata = devm_kzalloc(dev, struct_size(ddata, data, pdata->nbuttons), GFP_KERNEL); 

	if (!ddata) {
		dev_err(dev, "failed to allocate button drive data\n");
		return -ENOMEM;
	}

	ddata->keymap = devm_kcalloc(dev, pdata->nbuttons, sizeof(ddata->keymap[0]), GFP_KERNEL);
	if (!ddata->keymap)
		return -ENOMEM;

	ddata->pdata = pdata;
	ddata->input = input;
	ddata->aks_dev = aks_dev;
	mutex_init(&ddata->disable_lock);

	input->keycode = ddata->keymap;
	input->keycodesize = sizeof(ddata->keymap[0]);
	input->keycodemax = pdata->nbuttons;
	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	dev_err(g_aks_dev->dev, "%s------(%d)\n",__FUNCTION__, __LINE__);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];

		child = device_get_next_child_node(dev, child);
		if (!child) {
			dev_err(dev, "missing child device node for entry %d\n", i);
			return -EINVAL;
		}

		fwnode_property_read_string(child, "name", &node_name);
		if(!valid_gpio_key_node(node_name)) {
			dev_err(dev, "Ignore node: %s\n", node_name);
			continue;
		}

		error = aks_gamepad_gpio_keys_setup_key(dev, input, ddata, button, i, child);
		if (error) {
			fwnode_handle_put(child);
			dev_err(g_aks_dev->dev, "error: %d for setup %s, but continue...\n", error, node_name);
			//success = false;
			//return error;
		}

		if (button->wakeup)
			wakeup = 1;
	}

	fwnode_handle_put(child);


	device_init_wakeup(dev, wakeup);
	aks_dev->btn_drv_data = ddata;

	return 0;
}

static void ask_gamepad_gpio_keys_report_state(struct aks_gpio_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct aks_gpio_button_data *bdata = &ddata->data[i];
		if (bdata->gpiod)
			aks_gamepad_gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}


static int ask_gamepad_gpio_keys_open(struct aks_input_device *aks_dev)
{
	struct aks_gpio_keys_drvdata *ddata = aks_dev->btn_drv_data;
	struct input_dev *input = aks_dev->input;
	const struct gpio_keys_platform_data *pdata = ddata->pdata;
	
	int error;
	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error) {
			dev_err(g_aks_dev->dev, "---> %s(%d) error: %d\n",__FUNCTION__, __LINE__, error);
			return error;
		}
	}

	/* Report current state of buttons that are connected to GPIOs */
	ask_gamepad_gpio_keys_report_state(ddata);

	return 0;
}

static void ask_gamepad_gpio_keys_close(struct aks_input_device *aks_dev)
{
	struct input_dev *input = aks_dev->input;
	struct aks_gpio_keys_drvdata *ddata = aks_dev->btn_drv_data;
	const struct gpio_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}


static int aks_gamepad_analog_keys_handle(const void *data, void *private)
{
    struct aks_analog_key_data *joy = private;
    enum iio_endian endianness;
    int bytes, msb, val, idx, i;

    //const u16 *data_u16;
    bool sign;

    typedef struct {
        s16 chans[3];
        s64 timestamp __aligned(8);
    } Scan;

    Scan *scan = (Scan*)data;
    bytes = joy->chans[0].channel->scan_type.storagebits >> 3;

    for (i = 0; i < joy->num_chans; ++i) {
        idx = joy->chans[i].channel->scan_index;
        endianness = joy->chans[i].channel->scan_type.endianness;
        msb = joy->chans[i].channel->scan_type.realbits - 1;
        sign = tolower(joy->chans[i].channel->scan_type.sign) == 's';
        val = scan->chans[i];
        val >>= joy->chans[i].channel->scan_type.shift;
        if (sign)
            val = sign_extend32(val, msb);
        else
            val &= GENMASK(msb, 0);
        input_report_abs(joy->input, joy->axes[i].code, val);
    }
    input_sync(joy->input);
    return 0;
}


static void aks_gamepad_analog_keys_cleanup(void *data)
{
    iio_channel_release_all_cb(data);
}

static bool enable_js_mt=true;
static void aks_gamepad_analog_keys_poll(struct input_dev *input)
{
    struct aks_input_device *joy = input_get_drvdata(input);
	struct aks_analog_key_axis *axis;
    s32 i, val, offset_val;
	//int _x, _y, tx, ty, id;
	//bool process = false;

	if(joy->js_polling) {
		if(joy->work_mode == AKS_GAMEPAD_WORK_MODE_DIRECT) {
		    for (i = 0; i < joy->analog_data->num_chans; i++) {
				axis = &joy->analog_data->axes[i];
		        iio_read_channel_raw(&joy->analog_data->chans[i], &val);
		        //dev_err(g_aks_dev->dev, "code=%d, channel[%d]=%d\n",joy->analog_data->axes[i].code, joy->analog_data->chans[i].channel->address, val);

				if(axis->code == ABS_X || axis->code == ABS_Y || axis->code == ABS_Z || axis->code == ABS_RZ) {
					val = (val < axis->cali.rang_min_calied) ? axis->cali.rang_min_calied : val;
					val = (val > axis->cali.rang_max_calied) ? axis->cali.rang_max_calied : val;
					offset_val = val;
					if(val >= axis->cali.physic_zero_pos) {
						offset_val = axis->cali.logic_zero_pos + ((axis->cali.ratio_pos * (val - axis->cali.physic_zero_pos)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
					} else {
						offset_val = axis->cali.logic_zero_pos - ((axis->cali.ratio_neg * (axis->cali.physic_zero_pos - val)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
					}

				} else if (axis->code == ABS_GAS || axis->code == ABS_BRAKE) {
					val = (val < axis->cali.physic_zero_pos) ? axis->cali.physic_zero_pos : val;
					offset_val = axis->cali.logic_zero_pos + ((axis->cali.ratio_pos * (val - axis->cali.physic_zero_pos)) >> AKS_GAMEPAD_ENLARGE_SHIFT);
				}
				input_report_abs(input, axis->code, offset_val);
		    }
			input_sync(input);
		}else if (joy->work_mode == AKS_GAMEPAD_WORK_MODE_TOUCH) {
			if(enable_js_mt)
				aks_gamepad_handle_js_to_touch(joy);
		}
	}else {
		dev_err(g_aks_dev->dev, "Suspended, Pause the polling...\n");
	}
}

static void aks_gamepad_analog_init_zero_coordinate(struct aks_analog_key_data *adata, int code, int val) {
	switch(code) {
		case ABS_X:
			adata->z_coord.x = val;
			break;
		case ABS_Y:
			adata->z_coord.y = val;
			break;
		case ABS_Z:
			adata->z_coord.z = val;
			break;
		case ABS_RZ:
			adata->z_coord.rz = val;
			break;
		default:
			break;
	}
}

static int aks_gamepad_analog_init_cali_data(struct aks_analog_key_data *adata)
{
    int ret=0, i, val;

	if (!adata) {
		dev_err(g_aks_dev->dev, "%s(%d) error: adata NULL!\n",__FUNCTION__, __LINE__);
		return -EINVAL;
	}

    for (i = 0; i < adata->num_chans; i++) {
        ret = iio_read_channel_raw(&adata->chans[i], &val);
	    if (ret != IIO_VAL_INT) {
        	dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, ret);
        	return ret;
    	} else {
			adata->axes[i].cali.physic_zero_pos = val;
			aks_gamepad_analog_init_zero_coordinate(adata, adata->axes[i].code, val);
			//dev_err(g_aks_dev->dev, "code=%d, channel[%d], zero=%d\n", adata->axes[i].code, adata->chans[i].channel->address, adata->axes[i].cali.zero_pos);
		}
    }

    return 0;
}

static void aks_gamepad_analog_calibrate(struct aks_analog_key_data *adata)
{
	struct aks_analog_key_axis* axis;
	int i=0; 
	//int diff_neg, diff_pos;

	for(i=0; i < adata->num_chans; i++) {
		axis = &adata->axes[i];

		if(axis->code == ABS_X || axis->code == ABS_Y || axis->code == ABS_Z|| axis->code == ABS_RZ) {
			axis->cali.rang_min_calied = axis->cali.rang_min;
			axis->cali.rang_max_calied = axis->cali.rang_max;

			//The logic center is in the middle of range
			axis->cali.logic_zero_pos = (axis->cali.rang_min_calied + axis->cali.rang_max_calied) >> 1;
			axis->cali.dist_max = abs(axis->cali.rang_max - axis->cali.physic_zero_pos);
			axis->cali.dist_min = abs(axis->cali.physic_zero_pos - axis->cali.rang_min);
			axis->cali.logic_half_distance = ((axis->cali.rang_max_calied - axis->cali.rang_min_calied) >> 1);
			axis->cali.ratio_pos = (( axis->cali.logic_half_distance << AKS_GAMEPAD_ENLARGE_SHIFT) / axis->cali.dist_max);
			axis->cali.ratio_neg = (( axis->cali.logic_half_distance << AKS_GAMEPAD_ENLARGE_SHIFT) / axis->cali.dist_min);
			axis->cali.enlarge_ratio_max = ((1 << AKS_GAMEPAD_ENLARGE_SHIFT) / axis->cali.dist_min);
			axis->cali.enlarge_ratio_min = ((1 << AKS_GAMEPAD_ENLARGE_SHIFT) / axis->cali.dist_max);
		} else if(axis->code == ABS_BRAKE || axis->code == ABS_GAS) {
			#if 0
			axis->cali.rang_min_calied = axis->cali.physic_zero_pos;
			axis->cali.rang_max_calied = axis->cali.rang_max;
			axis->cali.ratio_neg = axis->cali.ratio_neg = 1;
			#else
			axis->cali.dist_max = axis->cali.rang_max - axis->cali.physic_zero_pos;
			axis->cali.ratio_pos = ((500 << AKS_GAMEPAD_ENLARGE_SHIFT) / axis->cali.dist_max);
			axis->cali.rang_min_calied = 1;
			axis->cali.rang_max_calied = 501;
			#endif
		}
		/*
		dev_err(g_aks_dev->dev, "code=%d, min=%d, max=%d, cali_min=%d, cali_max=%d, ratio_pos=%d, ratio_neg=%d, pc=%d, lc=%d\n", \
			axis->code, axis->cali.rang_min, axis->cali.rang_max, \
			axis->cali.rang_min_calied, axis->cali.rang_max_calied, \
			axis->cali.ratio_pos, axis->cali.ratio_neg, \
			axis->cali.physic_zero_pos, axis->cali.logic_zero_pos);
		*/
	}
}

static int aks_gamepad_analog_update_paras(struct aks_analog_key_data *adata) {
	
	int i=0;
	if(!adata) {
		return -EINVAL;
	}

	//update range
	for(i=0; i < adata->num_chans; i++) {
		if(adata->axes[i].code == ABS_X || adata->axes[i].code == ABS_RZ) {
			input_set_abs_params(adata->input, adata->axes[i].code, \
				adata->axes[i].cali.rang_max_calied, adata->axes[i].cali.rang_min_calied, \
				adata->axes[i].fuzz, adata->axes[i].flat);

		} else {
			input_set_abs_params(adata->input, adata->axes[i].code, \
				adata->axes[i].cali.rang_min_calied, adata->axes[i].cali.rang_max_calied, \
				adata->axes[i].fuzz, adata->axes[i].flat);
		}
/*
		dev_err(g_aks_dev->dev, "code=%d, cali_min=%d, cali_max=%d\n", \
			adata->axes[i].code, adata->axes[i].cali.rang_min_calied, adata->axes[i].cali.rang_max_calied);
*/
	}
	return 0;
}

static int aks_gamepad_analog_setup_extra(struct aks_analog_key_data *adata) {
	if(!adata) {
		return -EINVAL;
	}

	//simulate DPAD to analog data, Only -1, 0, and 1
	input_set_abs_params(adata->input, ABS_HAT0X, 1, 99, 3, 3);
	input_set_abs_params(adata->input, ABS_HAT0Y, 99, 1, 3, 3);
	return 0;
}


static int aks_gamepad_analog_keys_set_axes(struct device *dev, struct aks_analog_key_data *joy)
{
    struct aks_analog_key_axis *axes;
    struct fwnode_handle *child;

    int num_axes=0, error, i;
	const char* node_name;

	bool forbid = false;

	device_for_each_child_node(dev, child) {
		fwnode_property_read_string(child, "name", &node_name);
		if(str_starts_with(node_name, "axis")) {
			num_axes++;
		}
	}

	dev_err(dev, "Got %d child nodes for %d channels\n",num_axes, joy->num_chans);

    if (!num_axes) {
        dev_err(dev, "Unable to find child nodes\n");
        return -EINVAL;
    }

    if (num_axes != joy->num_chans) {
        dev_err(dev, "Got %d child nodes for %d channels\n",
                num_axes, joy->num_chans);
        return -EINVAL;
    }

    axes = devm_kmalloc_array(dev, num_axes, sizeof(*axes), GFP_KERNEL);
    if (!axes) {
        return -ENOMEM;
    }

    device_for_each_child_node(dev, child) {
        error = fwnode_property_read_u32(child, "index", &i);
        if (error) {
			continue; // ignore the gpio buttons
            //goto err_fwnode_put;
        }

        if (i >= num_axes) {
            error = -EINVAL;
            dev_err(dev, "No matching axis for reg %d\n", i);
            goto err_fwnode_put;
        }

        error = fwnode_property_read_u32(child, "linux,code", &axes[i].code);
        if (error) {
            dev_err(dev, "linux,code invalid or missing\n");
            goto err_fwnode_put;
        }

        error = fwnode_property_read_u32_array(child, "abs-range", axes[i].range, 2);

        if (error) {
            dev_err(dev, "abs-range invalid or missing\n");
            goto err_fwnode_put;
        }

        axes[i].cali.rang_min = min(axes[i].range[0], axes[i].range[1]);
        axes[i].cali.rang_max = max(axes[i].range[0], axes[i].range[1]);

        fwnode_property_read_u32(child, "abs-fuzz", &axes[i].fuzz);
        fwnode_property_read_u32(child, "abs-flat", &axes[i].flat);
		/*
        dev_err(dev, "%s(%d) linux,code=<%d>, abs-range= %d ~ %d, abs-fuzz= %d, abs-flat=%d!\n",\
			__FUNCTION__, __LINE__, \
            axes[i].code, axes[i].range[0], axes[i].range[1], axes[i].fuzz, axes[i].flat);
		*/
        input_set_abs_params(joy->input, axes[i].code, axes[i].range[0], axes[i].range[1], axes[i].fuzz, axes[i].flat);
        input_set_capability(joy->input, EV_ABS, axes[i].code);
    }

    input_set_capability(joy->input, EV_KEY, BTN_JOYSTICK); //Mark as joystick device in Android Event Hub.

	joy->axes = axes;

	error = aks_gamepad_analog_init_cali_data(joy);
	if(error) {
		return -EINVAL;
	}
	if(!forbid) {
		aks_gamepad_analog_calibrate(joy);
		aks_gamepad_analog_update_paras(joy);
	}
	aks_gamepad_analog_setup_extra(joy);
    return 0;

err_fwnode_put:
    fwnode_handle_put(child);
    return error;
}

static int aks_gamepad_config_analog_keys(struct aks_input_device* aks_dev) {
	struct aks_analog_key_data *adata;
	struct device* dev = aks_dev->dev;
	int error;
	int bits;
	int i;

	adata = devm_kzalloc(dev, sizeof(*adata), GFP_KERNEL);
	if (!adata) {
		dev_err(dev, "failed to allocate anolog data\n");
		return -ENOMEM;
	}

	adata->input = aks_dev->input;
	aks_dev->analog_data = adata;

	adata->polled = AKS_GAMEPAD_ANALOG_POLL;

	adata->chans = devm_iio_channel_get_all(dev);
	if (IS_ERR(adata->chans)) {
		error = PTR_ERR(adata->chans);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Unable to get IIO channels");
		return error;
	}

	for (i = 0; adata->chans[i].indio_dev; i++) {
		if (adata->polled)
			continue;
		bits = adata->chans[i].channel->scan_type.storagebits;
		if (!bits || bits > 16) {
			dev_err(dev, "Unsupported channel storage size\n");
			return -EINVAL;
		}
		if (bits != adata->chans[0].channel->scan_type.storagebits) {
			dev_err(dev, "Channels must have equal storage size\n");
			return -EINVAL;
		}
	}
	adata->num_chans = i;

	error = aks_gamepad_analog_keys_set_axes(dev, adata);
	if (error)
		return error;

	if (adata->polled) {
		aks_dev->js_polling = true;
		input_setup_polling(aks_dev->input, aks_gamepad_analog_keys_poll);
		input_set_poll_interval(aks_dev->input, AKS_GAMEPAD_ANALOG_POLL_INTERVAL);
	} else {
		adata->buffer = iio_channel_get_all_cb(dev, aks_gamepad_analog_keys_handle, adata);
		if (IS_ERR(adata->buffer)) {
			dev_err(dev, "Unable to allocate callback buffer\n");
			return PTR_ERR(adata->buffer);
		}

		error = devm_add_action_or_reset(dev, aks_gamepad_analog_keys_cleanup, adata->buffer);
		if (error)  {
			dev_err(dev, "Unable to add action\n");
			return error;
		}
	}

	return 0;
}

static int aks_gamepad_configure_mt_dev(struct aks_input_device *aks_dev)
{
	struct aks_multitouch_data *mdata;
	struct device* dev = aks_dev->dev;
	int error = 0;
	mdata = devm_kzalloc(dev, sizeof(*mdata), GFP_KERNEL);
	if (!mdata) {
		dev_err(dev, "failed to allocate moutch touch data\n");
		return -ENOMEM;
	}

	//mdata->input = aks_dev->input;
	aks_dev->mt_data = mdata;

	mdata->max_touch_num = AKS_GAMEPAD_MT_MAX_CONTACTS;

	mdata->mt_input = devm_input_allocate_device(dev);
	if (!mdata->mt_input) {
		dev_err(dev, "Failed to allocate moutch touch input device.");
		return -ENOMEM;
	}

	mdata->mt_input->name = "AKS TouchScreen";
	mdata->mt_input->phys = "aks/input1";
	mdata->mt_input->id.bustype = BUS_HOST;
	mdata->mt_input->id.vendor = 0x2212;
	mdata->mt_input->id.product = 0x0011;
	mdata->mt_input->id.version = 0x0016;

	dev_err(dev, "%s------(%d)\n",__FUNCTION__, __LINE__);

	set_bit(EV_SYN, mdata->mt_input->evbit);
	set_bit(EV_KEY, mdata->mt_input->evbit);
	set_bit(EV_ABS, mdata->mt_input->evbit);

	set_bit(BTN_TOUCH, mdata->mt_input->keybit);
	set_bit(BTN_TOOL_FINGER, mdata->mt_input->keybit);
	set_bit(INPUT_PROP_DIRECT, mdata->mt_input->propbit);
	/* set input parameters */
	input_set_abs_params(mdata->mt_input, ABS_MT_POSITION_X, 0, AKS_GAMEPAD_MT_MAX_WIDTH - 1, 0, 0);
	input_set_abs_params(mdata->mt_input, ABS_MT_POSITION_Y, 0, AKS_GAMEPAD_MT_MAX_HEIGHT - 1, 0, 0);
	input_set_abs_params(mdata->mt_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_mt_init_slots(mdata->mt_input, AKS_GAMEPAD_MT_MAX_CONTACTS, INPUT_MT_DIRECT);

	aks_dev->mt_input = mdata->mt_input;
	mdata->aks_dev = aks_dev;
	error = input_register_device(aks_dev->mt_input);
	if (error) {
		dev_err(dev, "Failed to register moutch touch input device: %d", error);
		return error;
	}

	return 0;
}


static int aks_gamepad_input_open(struct input_dev *input)
{
	struct aks_input_device *aks_dev = input_get_drvdata(input);
	dev_err(g_aks_dev->dev, "---> input_open (%d)\n", __LINE__);

	ask_gamepad_gpio_keys_open(aks_dev);

	return 0;
}

static void aks_gamepad_input_close(struct input_dev *input)
{
	struct aks_input_device *aks_dev = input_get_drvdata(input);

	dev_err(g_aks_dev->dev, "---> input_close(%d)\n", __LINE__);
	ask_gamepad_gpio_keys_close(aks_dev);
}

static int aks_gamepad_config_input_dev(struct platform_device *pdev) {
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int error;

	struct aks_input_device *aks_dev = platform_get_drvdata(pdev);

	if(!aks_dev) {
		dev_err(dev, "No aksys dev platform data\n");
		return -EFAULT;
	}
	
	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	
	input_set_drvdata(input, aks_dev);

	input->name = "AKS Gamepad";
	input->phys = "aks/input0";
	input->dev.parent = dev;
	input->open = aks_gamepad_input_open;
	input->close = aks_gamepad_input_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x2212;
	input->id.product = 0x0010;
	input->id.version = 0x0016;

	aks_dev->input = input;

	error = aks_gamepad_config_gpio_keys(aks_dev);
	if (error) {
		dev_err(dev, "Unable to config gpio keys error: %d\n", error);
		return error;
	}

	error = aks_gamepad_config_analog_keys(aks_dev);
	if (error) {
		dev_err(dev, "Unable to config analog keys error: %d\n", error);
		return error;
	}

	error = aks_gamepad_configure_mt_dev(aks_dev);
	if (error) {
		dev_err(dev, "Unable to config multi touch error: %d\n", error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n", error);
		return error;
	}

	//update_thread_init(aks_dev);

	return 0;

}
//endif

static int aks_cdev_major = AKS_CDEV_MAJOR;
 
module_param(aks_cdev_major, int, S_IRUGO);
 
struct aks_cdev_data *aks_cdev_data_p;
struct cdev aks_cdev;

struct class *aks_cdev_cls;
struct device *aks_cdev_device;
char* aks_cdev_name = "aks_input";
dev_t aks_cdev_no;

int aks_input_cdev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = g_aks_dev;
	return 0;
}

int aks_input_cdev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

long aks_input_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int ioarg = 0;
	struct aks_input_device *devp = filp->private_data;
	if(devp == NULL) {
		dev_err(g_aks_dev->dev, "---> %s(%d) private data is NULL\n",__FUNCTION__, __LINE__);
	}

    if (_IOC_TYPE(cmd) != AKS_IOC_MAGIC) 
        return -EINVAL;
    if (_IOC_NR(cmd) > AKS_IOC_MAXNR) 
        return -EINVAL;
#if 0
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
    if (err) 
        return -EFAULT;
#endif

    switch(cmd) {

		case AKS_IOC_INIT:
			dev_err(devp->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
			break;

		case AKS_IOC_GET_DATA:
			ioarg = 1101;
			ret = __put_user(ioarg, (int *)arg);
			break;

		case AKS_IOC_SET_DATA:
			ret = __get_user(ioarg, (int *)arg);
			dev_err(devp->dev,"<--- In Kernel MEMDEV_IOCSETDATA ioarg = %d --->\n\n",ioarg);
			break;

		case AKS_IOC_GET_MODE:
			ioarg = devp->work_mode;
			ret = __put_user(ioarg, (int *)arg);
			break;

		case AKS_IOC_SET_MODE:
			ret = __get_user(ioarg, (int *)arg);
			if(ioarg == AKS_GAMEPAD_WORK_MODE_DIRECT || ioarg == AKS_GAMEPAD_WORK_MODE_TOUCH) {
				dev_err(devp->dev, "---> %s(%d) change work mode to -> %d\n",__FUNCTION__, __LINE__, ioarg);
				devp->work_mode = ioarg;
				touched_num = 0;
			} else {
				dev_err(devp->dev, "---> %s(%d) Invalid work mode -> %d\n",__FUNCTION__, __LINE__, ioarg);
			}
			break;

		case AKS_IOC_SET_MAPPING:
			break;
		case AKS_IOC_GET_MAPPING:
			break;
		case AKS_IOC_SET_FN:
			break;
		case AKS_IOC_GET_FN:
			break;	
		case AKS_IOC_SET_CALI:
			break;
		case AKS_IOC_SET_MACRO:
			break;
		case AKS_IOC_GET_MACRO:
			break;
		case AKS_IOC_UPDTAE_MACRO:
			break;
		default:  
			return -EINVAL;
    }
    return ret;

}



static ssize_t aks_input_cdev_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
#if 0
	unsigned long p = *ppos;
	unsigned int count = size;
	int ret = 0;
	struct VirtualDisk *devp = filp->private_data;
 
	if (p >= VIRTUALDISK_SIZE)
	{
		return count ? -ENXIO : 0;
	}
	if (count > VIRTUALDISK_SIZE - p)
	{
		count = VIRTUALDISK_SIZE - p;
	}
 
	if (copy_to_user(buf, (void *)(devp->mem + p), count))
	{
		ret = -EFAULT;
	}
	else
	{
		*ppos += count;
		ret = count;
		printk(KERN_INFO "read %d bytes from %lx\n", count, p);
	}
	return ret;
#else
	return 0;
#endif 
}

static void dump_mapping_data(struct aks_multitouch_data *mt_data)
{
	int i=0;
	for(i=0; i<AKS_GAMEPAD_BINDABLE_KEYS; i++) {
		dev_err(g_aks_dev->dev, " keycode=%d:  x=%d, y=%d", mt_data->mapping_coords[i].keycode , mt_data->mapping_coords[i].x, mt_data->mapping_coords[i].y);
	}

	dev_err(g_aks_dev->dev, "======== Properties: ");

	dev_err(g_aks_dev->dev, "       Slot = %d ", mt_data->mapping_props.slot);
	dev_err(g_aks_dev->dev, "   rotation = %d ", mt_data->mapping_props.rotation);
	dev_err(g_aks_dev->dev, "    ls_type = %d ", mt_data->mapping_props.ls_type);
	dev_err(g_aks_dev->dev, "    rs_type = %d ", mt_data->mapping_props.rs_type);
	dev_err(g_aks_dev->dev, "    ls_size = %d ", mt_data->mapping_props.ls_size);
	dev_err(g_aks_dev->dev, "    rs_size = %d ", mt_data->mapping_props.rs_size);
	dev_err(g_aks_dev->dev, " l_bind_key = %d ", mt_data->mapping_props.l_bind_key[0]);
	dev_err(g_aks_dev->dev, " r_bind_key = %d ", mt_data->mapping_props.r_bind_key[0]);
	dev_err(g_aks_dev->dev, "   ls x=%d, y=%d", mt_data->mapping_props.ls.x, mt_data->mapping_props.ls.y);
	dev_err(g_aks_dev->dev, "   rs x=%d, y=%d", mt_data->mapping_props.rs.x, mt_data->mapping_props.rs.y);
}

static inline int need_parse_pair(char* key)
{
	return (!strcmp(key,"A")) || \
		(!strcmp(key,"B")) || \
		(!strcmp(key,"X")) || \
		(!strcmp(key,"Y")) || \
		(!strcmp(key,"L1")) || \
		(!strcmp(key,"R1")) || \
		(!strcmp(key,"L2")) || \
		(!strcmp(key,"R2")) || \
		(!strcmp(key,"Up")) || \
		(!strcmp(key,"Down")) || \
		(!strcmp(key,"Left")) || \
		(!strcmp(key,"Right")) || \
		(!strcmp(key,"L3")) || \
		(!strcmp(key,"R3")) || \
		(!strcmp(key,"LStick")) || \
		(!strcmp(key,"RStick")) || \
		(!strcmp(key,"Select")) || \
		(!strcmp(key,"Start")) || \
		(!strcmp(key,"Mode")) || \
		(!strcmp(key,"Fn")) || \
		(!strcmp(key,"Sysrq"));
}

static int fill_up_keycode_bit_map(unsigned int keycode, int x, int y) {
	int pos = find_keycode_bit_map_index_by_code(keycode);
	if(pos != -1) {
		aks_gamepad_keycode_bit_map[pos].coord.x = x;
		aks_gamepad_keycode_bit_map[pos].coord.y = y;
		return 0;
	} else {
		dev_err(g_aks_dev->dev,"W: Not found target keycode %d in mapping data. Ignore it\n", keycode);
		return -1;
	}
}

static void setup_mapping_data(struct aks_multitouch_data *mt_data, unsigned int keycode, int x, int y, int id) {
	mt_data->mapping_coords[mt_data->mapping_coords_index].keycode = keycode;
	mt_data->mapping_coords[mt_data->mapping_coords_index].x = x;
	mt_data->mapping_coords[mt_data->mapping_coords_index].y = y;
	mt_data->mapping_coords[mt_data->mapping_coords_index].id = id;
	fill_up_keycode_bit_map(keycode, x, y);
	mt_data->mapping_coords_index++;
}

static int pares_pair(struct aks_multitouch_data *mt_data, struct kv_pair pair, const char* delim) {
	struct aks_js_mt_status_recoder* record = &g_aks_dev->js_record;
	char *token, *cur;
	int x=0, y=0;
	int idx = 0;
	char data[64];	

	memset(data, '\0', sizeof(data));
	strcpy(data, pair.value);

	cur = data;
	while ((token = strsep(&cur, delim))) {
		if(idx == 0) {
			x = simple_strtoul(token, NULL, 10);//atoi(token);
		} else if(idx == 1) {
			y = simple_strtoul(token, NULL, 10);
		}
		if(idx > 0) {
			if(strcmp(pair.key, "A") == 0) {
				setup_mapping_data(mt_data, BTN_A, x, y, MT_ID_A);
			} else if (strcmp(pair.key, "B") == 0) {
				setup_mapping_data(mt_data, BTN_B, x, y, MT_ID_B);
			} else if (strcmp(pair.key, "X") == 0) {
				setup_mapping_data(mt_data, BTN_X, x, y, MT_ID_X);
			} else if (strcmp(pair.key, "Y") == 0) {
				setup_mapping_data(mt_data, BTN_Y, x, y, MT_ID_Y);
			} else if (strcmp(pair.key, "L1") == 0) {
				setup_mapping_data(mt_data, BTN_TL, x, y, MT_ID_L1);
			} else if (strcmp(pair.key, "R1") == 0) {
				setup_mapping_data(mt_data, BTN_TR, x, y, MT_ID_R1);
			} else if (strcmp(pair.key, "L2") == 0) {
				setup_mapping_data(mt_data, ABS_BRAKE, x, y, MT_ID_L2);
			} else if (strcmp(pair.key, "R2") == 0) {
				setup_mapping_data(mt_data, ABS_GAS, x, y, MT_ID_R2);
			} else if (strcmp(pair.key, "Up") == 0) {
				setup_mapping_data(mt_data, BTN_DPAD_UP, x, y, MT_ID_DU);
			} else if (strcmp(pair.key, "Down") == 0) {
				setup_mapping_data(mt_data, BTN_DPAD_DOWN, x, y, MT_ID_DD);
			} else if (strcmp(pair.key, "Left") == 0) {
				setup_mapping_data(mt_data, BTN_DPAD_LEFT, x, y, MT_ID_DL);
			} else if (strcmp(pair.key, "Right") == 0) {
				setup_mapping_data(mt_data, BTN_DPAD_RIGHT, x, y, MT_ID_DR);
			} else if (strcmp(pair.key, "L3") == 0) {
				setup_mapping_data(mt_data, BTN_THUMBL, x, y, MT_ID_L3);
			} else if (strcmp(pair.key, "R3") == 0) {
				setup_mapping_data(mt_data, BTN_THUMBR, x, y, MT_ID_R3);
			} else if (strcmp(pair.key, "LStick") == 0) {
				record->last_touch_x = x;
				record->last_touch_y = y;
				mt_data->mapping_props.ls.x = x;
				mt_data->mapping_props.ls.y = y;
				setup_mapping_data(mt_data, BTN_JOYSTICK, x, y, MT_ID_LS);
			} else if (strcmp(pair.key, "RStick") == 0) {
				record->last_touch_z = x;
				record->last_touch_rz = y;
				mt_data->mapping_props.rs.x = x;
				mt_data->mapping_props.rs.y = y;
				//record->last_binding_rs = true;
				setup_mapping_data(mt_data, BTN_JOYSTICK, x, y, MT_ID_RS);
			} else if (strcmp(pair.key, "Select") == 0) {
				setup_mapping_data(mt_data, BTN_SELECT, x, y, MT_ID_SELECT);
			} else if (strcmp(pair.key, "Start") == 0) {
				setup_mapping_data(mt_data, BTN_START, x, y, MT_ID_START);
			} else if (strcmp(pair.key, "Mode") == 0) {
				setup_mapping_data(mt_data, BTN_MODE, x, y, MT_ID_MODE);
			} else if (strcmp(pair.key, "Fn") == 0) {
				setup_mapping_data(mt_data, KEY_FN, x, y, MT_ID_FUNC);
			} else if (strcmp(pair.key, "Sysrq") == 0) {
				setup_mapping_data(mt_data, KEY_SYSRQ, x, y, MT_ID_SYSRQ);
			} else {
				dev_err(g_aks_dev->dev,"Error, Button not found: %s!\n", pair.key);
			}
		}
		idx++;
	}
	return 0;
}

static int pares_member(struct aks_multitouch_data *mt_data,  char* str, const char* delim, int index) {
	char *token, *cur;
	char* const delim_next= ",";
	int tmp=0, idx=0;
	char data[64];

	memset(kvs, '\0', sizeof(kvs));
	memset(data, '\0', sizeof(data));
	strcpy(data, str);
	cur = data;
	while ((token = strsep(&cur, delim)) != NULL) {
		if(idx == 0) {
			strcpy(kvs[index].key, token);
		} else if(idx == 1) {
			strcpy(kvs[index].value, token);
		}
		if(idx > 0) {
			if(need_parse_pair(kvs[index].key)) {
				pares_pair(mt_data, kvs[index], delim_next);
			} else {
				//dev_err(g_aks_dev->dev,"[%s]value-> %d\n",kvs[index].key, simple_strtoul(kvs[index].value, NULL, 10));
				if(strcmp(kvs[index].key, "Slot") == 0) {
					mt_data->mapping_props.slot = simple_strtoul(kvs[index].value, NULL, 10);
				} else if (strcmp(kvs[index].key, "Rotation") == 0) {
					tmp = simple_strtoul(kvs[index].value, NULL, 10);
					if(tmp > 3 || tmp < 0) {
						tmp = 3;
					}
					mt_data->mapping_props.rotation = tmp;
				} else if (strcmp(kvs[index].key, "LsType") == 0) {
					mt_data->mapping_props.ls_type = simple_strtoul(kvs[index].value, NULL, 10);
				} else if (strcmp(kvs[index].key, "RsType") == 0) {
					mt_data->mapping_props.rs_type = simple_strtoul(kvs[index].value, NULL, 10);
				} else if (strcmp(kvs[index].key, "LsSize") == 0) {
					tmp = simple_strtoul(kvs[index].value, NULL, 10);
					if(tmp > 3 || tmp < 0) {
						tmp = 3;
					}
					mt_data->mapping_props.ls_size = tmp;
				} else if (strcmp(kvs[index].key, "RsSize") == 0) {
					tmp = simple_strtoul(kvs[index].value, NULL, 10);
					if(tmp > 3 || tmp < 0) {
						tmp = 3;
					}
					mt_data->mapping_props.rs_size = tmp;
				} else if (strcmp(kvs[index].key, "LBind") == 0) {
					mt_data->mapping_props.l_bind_key[0] = simple_strtoul(kvs[index].value, NULL, 10);
				} else if (strcmp(kvs[index].key, "RBind") == 0) {
					mt_data->mapping_props.r_bind_key[0] = simple_strtoul(kvs[index].value, NULL, 10);
				} else {
					dev_err(g_aks_dev->dev,"Error, Mapping key not found: %s", kvs[index].key);
				}
			}
		}
		idx++;
	}
	return 0;
}

static int aks_gamepad_parse_mt_mapping_data(char* config) {
	struct aks_input_device *devp = g_aks_dev;

	char* const delim= ";";
	char* const delim_next= ":";

	char *token, *cur;
	int index = 0;

	char data[AKS_MAPPING_DATA_MAX_LENGTH];
	memset(data, '\0', sizeof(data));
	strcpy(data, config);
	cur = data;

	while ((token = strsep(&cur, delim))) {
		pares_member(devp->mt_data, token, delim_next, index);
		index++;
	}

	dump_mapping_data(devp->mt_data);

	return 0;
}

static int aks_gamepad_cali_read_raw(int keycode)
{
	struct aks_analog_key_data *adata = g_aks_dev->analog_data;

    int ret=0, i, val;

	if (!adata) {
		dev_err(g_aks_dev->dev, "%s(%d) error: adata NULL!\n",__FUNCTION__, __LINE__);
		return -1;
	}

	msleep(3000); //waiting for the user action
    for (i = 0; i < adata->num_chans; i++) {
		if(adata->axes[i].code == keycode) {
			ret = iio_read_channel_raw(&adata->chans[i], &val);
			if (ret != IIO_VAL_INT) {
				dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, ret);
				return -1;
			} else {
				//dev_err(g_aks_dev->dev, "code=%d, channel[%d], zero=%d, HW val=%d\n", adata->axes[i].code, adata->chans[i].channel->address, adata->axes[i].cali.physic_zero_pos, val);
				return val;
			}
		}
    }

	return -1;
}

static void aks_gamepad_cali_cache(struct aks_analog_key_axis* axes, int keycode, int pos, int value)
{
	switch(pos) {
		case POS_ZERO:
			axes->code = keycode;
			axes->cali.physic_zero_pos = value;
			break;
		case POS_MIN:
			axes->code = keycode;
			axes->cali.rang_min = value;
			break;
		case POS_MAX:
			axes->code = keycode;
			axes->cali.rang_max = value;
			break;
		default:
			dev_err(g_aks_dev->dev, "%s(%d) Warning: Unknown pos=%d !\n",__FUNCTION__, __LINE__, pos);
			break;
	}
}

static int aks_gamepad_cali_save(struct aks_analog_key_axis* axes)
{
	struct aks_analog_key_data *adata = g_aks_dev->analog_data;

    int i;

	if (!adata) {
		dev_err(g_aks_dev->dev, "%s(%d) error: adata NULL!\n",__FUNCTION__, __LINE__);
		return -EINVAL;
	}

    for (i = 0; i < adata->num_chans; i++) {
		if(adata->axes[i].code == axes->code) {
			if(axes->cali.physic_zero_pos)
				adata->axes[i].cali.physic_zero_pos = axes->cali.physic_zero_pos;
			if(axes->cali.rang_min)
				adata->axes[i].cali.rang_min = axes->cali.rang_min;
			if(axes->cali.rang_max)
				adata->axes[i].cali.rang_max = axes->cali.rang_max;

			dev_err(g_aks_dev->dev, "%s(%d) Process key(%d), zero=%d, min=%d, max=%d\n", \
				__FUNCTION__, __LINE__, \
				adata->axes[i].code, \
				adata->axes[i].cali.logic_zero_pos, \
				adata->axes[i].cali.rang_min, \
				adata->axes[i].cali.rang_max);

			aks_gamepad_analog_calibrate(adata);
			aks_gamepad_analog_update_paras(adata);
			return 0;
		}
    }

	dev_err(g_aks_dev->dev, "%s(%d) error: adata NULL!\n",__FUNCTION__, __LINE__);

    return -1;
}


static int aks_gamepad_process_cali_start(int option) {
	int hw_value = -1;
	switch(option) {
		case CALI_OPTION_LT_ZERO:
			hw_value = aks_gamepad_cali_read_raw(ABS_BRAKE);
			if(hw_value == -1) {
				dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, hw_value);
				return -1;
			} else {
				aks_gamepad_cali_cache(&user_calied_axis_lt, ABS_BRAKE, POS_ZERO, hw_value);
			}
			break;
		case CALI_OPTION_LT_MAX:
			hw_value = aks_gamepad_cali_read_raw(ABS_BRAKE);
			if(hw_value == -1) {
				dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, hw_value);
				return -1;
			} else {
				aks_gamepad_cali_cache(&user_calied_axis_lt, ABS_BRAKE, POS_MAX, hw_value);
			}
			break;
		case CALI_OPTION_RT_ZERO:
			hw_value = aks_gamepad_cali_read_raw(ABS_GAS);
			if(hw_value == -1) {
				dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, hw_value);
				return -1;
			} else {
				aks_gamepad_cali_cache(&user_calied_axis_rt, ABS_GAS, POS_ZERO, hw_value);
			}
			break;
		case CALI_OPTION_RT_MAX:
			hw_value = aks_gamepad_cali_read_raw(ABS_GAS);
			if(hw_value == -1) {
				dev_err(g_aks_dev->dev, "%s(%d) error: %d !\n",__FUNCTION__, __LINE__, hw_value);
				return -1;
			} else {
				aks_gamepad_cali_cache(&user_calied_axis_rt, ABS_GAS, POS_MAX, hw_value);
			}
			break;
		default:
			dev_err(g_aks_dev->dev, "%s(%d) ................. \n",__FUNCTION__, __LINE__);
			break;
	}
	return 0;
}

static int aks_gamepad_process_cali_save(int option) {
	switch(option) {
		case CALI_OPTION_LT_MAX:
			return aks_gamepad_cali_save(&user_calied_axis_lt);
			break;
		case CALI_OPTION_RT_MAX:
			return aks_gamepad_cali_save(&user_calied_axis_rt);
			break;
		case CALI_OPTION_LS_OFFSET:
		case CALI_OPTION_RS_OFFSET:
			dev_err(g_aks_dev->dev, "%s(%d) ................. \n",__FUNCTION__, __LINE__);
			break;
	}

	return 0;
}


static int aks_gamepad_generate_cali_data(int option, int status) {
	switch (status) {
		case CALI_STATE_START:
			dev_err(g_aks_dev->dev, "calibrate state: start\n");
			aks_gamepad_process_cali_start(option);
			break;
		case CALI_STATE_STOP:
			dev_err(g_aks_dev->dev, "calibrate state: stop\n");
			break;
		case CALI_STATE_SAVE:
			dev_err(g_aks_dev->dev, "calibrate state: save\n");
			aks_gamepad_process_cali_save(option);
			break;
		case CALI_STATE_CANCEL:
			dev_err(g_aks_dev->dev, "calibrate state: cancel\n");
			break;
		default:
			dev_err(g_aks_dev->dev, "Unknown calibrate state: %d\n", status);
			break;
	}
	return 0;
}

static int aks_gamepad_parse_user_calibrate(char* config) {
	char* const delim= ";";
	char *token, *cur;
	char data[32] = {'\0'};
	int option;
	int status;

	strcpy(data, config);
	cur = data;

	token = strsep(&cur, delim);
	option = simple_strtol(token, NULL, 10);
	token = strsep(&cur, delim);
	status = simple_strtol(token, NULL, 10);
	dev_err(g_aks_dev->dev, "option=%d, status=%d\n", option, status);
	aks_gamepad_generate_cali_data(option, status);

	return 0;
}

static void aks_gamepad_reset_user_fn_key_option()
{
	aks_fun_key_info.activated = false;
	aks_fun_key_info.index = -1;
	aks_fun_key_info.option = -1;
	aks_fun_key_info.data = -1;
	aks_fun_key_info.ls_rate = -1;
	aks_fun_key_info.rs_rate = -1;
	aks_fun_key_info.f1_binding = AKS_FUNC_KEY_BINDING_F1;
	aks_fun_key_info.f2_binding = AKS_FUNC_KEY_BINDING_F2;
	aks_fun_key_info.f1_pressed = false;
	aks_fun_key_info.f2_pressed = false;
}

static int aks_gamepad_parse_user_function(char* config) {
	char* const delim= ";";
	char *token, *cur;
	char data[32] = {'\0'};
	int index;
	int option; //sniper only
	int full;
	int ls_rate, rs_rate;

	strcpy(data, config);
	cur = data;

	token = strsep(&cur, delim);
	index = simple_strtol(token, NULL, 10);
	token = strsep(&cur, delim);
	option = simple_strtol(token, NULL, 10);
	if(FUNC_OPTION_NONE == option) {
		aks_gamepad_reset_user_fn_key_option();
		return 0;
	}
	if(FUNC_OPTION_SNIPER != option) {
		dev_err(g_aks_dev->dev, "Deprecated options\n");
		return -1;
	}

	token = strsep(&cur, delim);
	full = simple_strtol(token, NULL, 10);

	if(full >= 0 && full <= 33) {
		if(full < 10) {
			ls_rate = 0;
		} else {
			ls_rate = full/10;
		}
		rs_rate = full%10;
		aks_fun_key_info.activated = true;
		aks_fun_key_info.data = full;
		aks_fun_key_info.index = index;
		aks_fun_key_info.option = option;
		aks_fun_key_info.ls_rate = ls_rate;
		aks_fun_key_info.rs_rate = rs_rate;
		dev_err(g_aks_dev->dev, "activated=%d, index=%d, option=%d, full=%d, ls_rate=%d, rs_rate=%d, f1=%d, f2=%d\n", \
					aks_fun_key_info.activated, \
					aks_fun_key_info.index, \
					aks_fun_key_info.option, \
					aks_fun_key_info.data, \
					aks_fun_key_info.ls_rate, \
					aks_fun_key_info.rs_rate, \
					aks_fun_key_info.f1_binding, \
					aks_fun_key_info.f2_binding);
	} else {
		dev_err(g_aks_dev->dev, "Invalide agruments\n");
		return -1;
	}
	return 0;
}


static ssize_t aks_input_cdev_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int count = size;
	int ret = 0;
	struct aks_input_device *devp = filp->private_data;
	char user_data[AKS_MAPPING_DATA_MAX_LENGTH] = {'\0'};
	char cali_data[32] = {'\0'};
	char fn_data[32] = {'\0'};

	mutex_lock(&devp->ipc_mutex);

	ret = copy_from_user((void *)user_data, buf, count);

	if (ret) {
		dev_err(g_aks_dev->dev, "---> %s(%d) ->error=%d\n",__FUNCTION__, __LINE__, ret);
	} else {
		//dev_err(g_aks_dev->dev, "--->(%d) Source(len=%d) -> %s\n", __LINE__,count, user_data);
		switch(user_data[0]) {
			case 'M':
				//MUST reset the raw data buffer every time
				memset(devp->mt_data->raw, '\0', AKS_MAPPING_DATA_MAX_LENGTH);
				strcpy(devp->mt_data->raw, user_data+2); //ignore the first two chars
				devp->mt_data->mapping_coords_index = 0;
				dev_err(g_aks_dev->dev, "--->(%d) Target -> %s\n", __LINE__, devp->mt_data->raw);
				aks_gamepad_parse_mt_mapping_data(devp->mt_data->raw);
				break;
			case 'C':
				strcpy(cali_data, user_data+2); //ignore the first two chars
				dev_err(g_aks_dev->dev, "--->(%d) Cali data -> %s\n", __LINE__, cali_data);
				aks_gamepad_parse_user_calibrate(cali_data);
				break;
			case 'F':
				strcpy(fn_data, user_data+2); //ignore the first two chars
				dev_err(g_aks_dev->dev, "--->(%d) func data -> %s\n", __LINE__, fn_data);
				aks_gamepad_parse_user_function(fn_data);
				break;
		}
	}
	mutex_unlock(&devp->ipc_mutex);
	return ret;
}


static const struct file_operations aks_input_cdev_fops =
{
  .owner = THIS_MODULE,
  .open = aks_input_cdev_open,
  .release = aks_input_cdev_release,
  .unlocked_ioctl = aks_input_cdev_ioctl,
  .read = aks_input_cdev_read,
  .write = aks_input_cdev_write,
};

static char *aks_input_class_devnode(struct device *dev, umode_t *mode)
{

    if (mode != NULL) {
        *mode = AKS_INPUT_CDEV_CLASS_MODE;
		dev_err(g_aks_dev->dev, "---> %s(%d) %s->mode=%o\n",__FUNCTION__, __LINE__, dev_name(dev), *mode);
    }
    return NULL;
}

static int aks_gamepad_cdev_init_ioctl() {
	int result;
	int i;

	aks_cdev_no = MKDEV(aks_cdev_major, 0);

	if (aks_cdev_major) {
		result = register_chrdev_region(aks_cdev_no, 2, "aks_cdev");
	} else {
		result = alloc_chrdev_region(&aks_cdev_no, 0, 2, "aks_cdev");
		aks_cdev_major = MAJOR(aks_cdev_no);
	}

	if (result < 0) {
		dev_err(g_aks_dev->dev, "---> %s(%d): error: %d\n",__FUNCTION__, __LINE__,result);
		return result;
	}


	cdev_init(&aks_cdev, &aks_input_cdev_fops);
	aks_cdev.owner = THIS_MODULE;
	aks_cdev.ops = &aks_input_cdev_fops;

	cdev_add(&aks_cdev, aks_cdev_no, AKS_CDEV_NR_DEVS);

	aks_cdev_data_p = kmalloc(AKS_CDEV_NR_DEVS * sizeof(struct aks_cdev_data), GFP_KERNEL);
	if (!aks_cdev_data_p) {
		dev_err(g_aks_dev->dev, "---> %s(%d) error\n",__FUNCTION__, __LINE__);
		result =  - ENOMEM;
		goto fail_malloc;
	}

	memset(aks_cdev_data_p, 0, sizeof(struct aks_cdev_data));

	for (i=0; i < AKS_CDEV_NR_DEVS; i++) 
	{
		aks_cdev_data_p[i].size = AKS_CDEV_SIZE;
		aks_cdev_data_p[i].data = kmalloc(AKS_CDEV_SIZE, GFP_KERNEL);
		memset(aks_cdev_data_p[i].data, 0, AKS_CDEV_SIZE);
	}

	aks_cdev_cls = class_create(THIS_MODULE, aks_cdev_name);
	if(IS_ERR(aks_cdev_cls)) {
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to create class\n",__FUNCTION__, __LINE__);
		result = -EBUSY;
		goto fail_malloc;
	}
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

	aks_cdev_cls->devnode = aks_input_class_devnode;

	aks_cdev_device = device_create(aks_cdev_cls, NULL, aks_cdev_no, NULL, aks_cdev_name);//mknod /dev/hello
	if(IS_ERR(aks_cdev_device)) {
		dev_err(g_aks_dev->dev, "---> %s(%d), Unable to create device\n",__FUNCTION__, __LINE__);
		class_destroy(aks_cdev_cls);
		result = -EBUSY;
		goto fail_malloc;
	}

	return 0;

	fail_malloc: 
	unregister_chrdev_region(aks_cdev_no, 1);

	return result;
}


static void aks_gamepad_reset_user_cali_cache()
{
	memset(&user_calied_axis_lt, 0, sizeof(struct aks_analog_key_axis));
	memset(&user_calied_axis_rt, 0, sizeof(struct aks_analog_key_axis));
}

static int aks_gamepad_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
	int error = 0;

    g_aks_dev = devm_kzalloc(dev, sizeof(*g_aks_dev), GFP_KERNEL);
	g_aks_dev->dev = dev;
    if (!g_aks_dev) {
        dev_err(dev, "%s(%d) failed allocate aks input dev!\n",__FUNCTION__, __LINE__);
        return -ENOMEM;
    } else {
		dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);
	}

	g_aks_dev->dev = dev;
	g_aks_dev->work_mode = CURRENT_WORK_MODE;
	dev_set_drvdata(dev, g_aks_dev);
	platform_set_drvdata(pdev, g_aks_dev);

	bitmap_zero(aks_gamepad_button_status, AKS_GAMEPAD_KEYCODE_BITS_MAX);
	bitmap_zero(aks_gamepad_touch_number, AKS_GAMEPAD_BINDABLE_KEYS);

	error = aks_gamepad_config_input_dev(pdev);

	if(error) {
		dev_err(g_aks_dev->dev, "---> %s(%d) error %d\n",__FUNCTION__, __LINE__, error);
		return error;
	}

	error = aks_gamepad_cdev_init_ioctl();
	if(error) {
		dev_err(g_aks_dev->dev, "---> %s(%d) error %d\n",__FUNCTION__, __LINE__, error);
		return error;
	}

	mutex_init(&g_aks_dev->mutex);
	mutex_init(&g_aks_dev->ipc_mutex);

	aks_gamepad_reset_user_cali_cache();
	aks_gamepad_reset_user_fn_key_option();
	dev_err(g_aks_dev->dev, "------> AKS driver preobe done <------\n");
    return 0;
}

static int aks_gamepad_remove(struct platform_device *pdev)
{
	dev_err(g_aks_dev->dev, "---> %s(%d)\n",__FUNCTION__, __LINE__);

	device_destroy(aks_cdev_cls, aks_cdev_no);
	class_destroy(aks_cdev_cls);	

	cdev_del(&aks_cdev);
 	kfree(aks_cdev_data_p);
  	unregister_chrdev_region(MKDEV(aks_cdev_major, 0), 2);

    return 0;
}


//#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int aks_js_pm_suspend(struct device *dev)
{
	dev_err(g_aks_dev->dev, "------> aks_js_pm_suspend <------\n");
	g_aks_dev->js_polling = false;
	return 0;
}

static int aks_js_pm_resume(struct device *dev)
{
	dev_err(g_aks_dev->dev, "------> aks_js_pm_resume <------\n");
	g_aks_dev->js_polling = true;
	return 0;
}
//#endif


//#if defined(CONFIG_PM) &&  !defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops dev_pm_ops = {
	.suspend = aks_js_pm_suspend,
	.resume = aks_js_pm_resume,
};
//#endif
static const struct of_device_id adc_joystick_of_match[] = {
    { .compatible = "aks,joystick_mt", },
    { }
};
MODULE_DEVICE_TABLE(of, adc_joystick_of_match);

static struct platform_driver adc_joystick_driver = {
    .driver = {
        .name = "aks,joystick_mt",
        .of_match_table = adc_joystick_of_match,
        .pm = &dev_pm_ops,
    },
    .probe = aks_gamepad_probe,
    .remove = aks_gamepad_remove,
};

module_platform_driver(adc_joystick_driver);

MODULE_DESCRIPTION("Input driver for Aksys.co.kr All in one device.");
MODULE_AUTHOR("Daniel <daniel@aksys.co.kr>");
MODULE_LICENSE("GPL");

#define uchar unsigned char 
struct Angle_Pitch {
	Angle_Pitch() :angle_pitch(0.0) {}
    double angle_pitch;
};


struct ArmorParam
{
	uchar min_light_gray;
	uchar min_light_height;	

	uchar avg_contrast_threshold;

	uchar light_slope_offset;

    int  max_light_delta_h;
	uchar min_light_delta_h;
	uchar max_light_delta_v;

	uchar max_light_delta_angle;

	uchar avg_board_gray_threshold;
	uchar avg_board_grad_threshold;

    uchar grad_threshold;

	uchar br_threshold;

    uchar enemy_color;

    ArmorParam(){
        min_light_gray = 210;
		min_light_height = 8;
        avg_contrast_threshold = 110;
		light_slope_offset = 30;
        max_light_delta_h = 450;
		min_light_delta_h = 12;
		max_light_delta_v = 50;
		max_light_delta_angle = 30;
        avg_board_gray_threshold = 80;
        avg_board_grad_threshold = 25;
        grad_threshold = 25;
		br_threshold = 30;
        enemy_color = 0;
	}
};

struct RuneParam
{
    int sudoku_cell_width; 
    int sudoku_cell_height;
    int shoot_time_gap; 
    int shoot_filter_size;
    RuneParam(){
        sudoku_cell_width = 143;
        sudoku_cell_height = 81;
        shoot_time_gap = 100;
        shoot_filter_size = 5;
    }
};
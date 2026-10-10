#include "controls.h"

uint8_t In_position=0;     //是否在目标位置
// 全局参数（示例：可在业务函数中修改）
float global_speed_scale;  // 速度缩放系数
uint8_t need_skip_next_step = 0;  // 是否跳过下一步
uint8_t time_flag=0;     //计时标志位
uint32_t  timekeeping=0;
uint32_t  now_time=0;    //距离计时到现在的时间


// // 5. 定义步骤数组（按实际运动流程初始化，顺序就是执行顺序）
// Motor_control_MotorStep motion_steps[] = {
//         /*步骤 0 ：沿0度前进，无额外操作*/
//     {	
//         .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {199500, 1000, 0},	// 目标距离，目标速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 1 ：向左差速走，无额外操作*/
//     {		
//         .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//         .params.single_speed = {1000,1000,100,100, 165000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//         .need_wait = 1,																					// 1=需要等待完成，0=不需要
//         .action = NULL, 																				// 无额外业务
// 				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 2 ：原地转到-90度，无额外操作*/
// 	{
//         .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
//         .params.diff_turn = {1000,-1000,-90},	// 左电机速度，右电机速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// //        /*步骤 3 ：沿着-90倒着走，无额外操作*/
// //    {
// //		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
// //        .params.fixed_dist = {-50000, 1000,-90},	// 目标距离，目标速度，目标角度
// //        .need_wait = 1,													// 1=需要等待完成，0=不需要
// //        .action = NULL, 												// 无额外业务
// //				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// //				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
// //    },
// 		/*步骤 4 ：不动，等待识别到二维码*/
//     {
//         .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist =	NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					//任务2:等待识别到二维码 
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   									// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 	{
// 		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {-310500,1000,-90},	// 目标距离，目标速度，目标角度(改318000)
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
// 	},
// 		/*步骤 6 ：向右前方差速走，无额外操作*/
//     {
//        .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//         .params.single_speed = {100,100,1000,1000,-160000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//         .need_wait = 1,																					// 1=需要等待完成，0=不需要
//         .action = NULL, 																				// 无额外业务
// 				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 7 ：原地旋转到0度，无额外操作*/
//     {
// 		.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
//         .params.diff_turn = {1000,-1000,180},	// 左电机速度，右电机速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 8 ：沿0度前进，无额外操作,过减速带，任务：过完减速带后后退矫正*/
//     {
// 		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {-290000, 500, 180},	// 目标距离，目标速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 过完减速带后后退矫正
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
//     },
// 	{
// 		 .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {-5000, 1000, 180},	// 目标距离，目标速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
// 	},
// 		/*步骤 9 ：向右差速走，无额外操作*/
//     {
// 		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//         .params.single_speed = {100,100,1000,1000,-160000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//         .need_wait = 1,																					// 1=需要等待完成，0=不需要
//         .action = NULL, 																				// 无额外业务
// 				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//     },
// 		/*步骤 10 ：原地旋转到90，无额外操作*/
//     {
// 		.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
//         .params.diff_turn = {-1000, 1000, 90},		// 左电机速度，右电机速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 11 ：沿90度直走，无额外操作*/
//     {
//         .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {385000, 1000, 90},	// 目标距离，目标速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//     },
// 		/*步骤 12 ：向左差速走，无额外操作*/
//     {
// 		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//         .params.single_speed = {1000, 1000, 100, 100, 165000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//         .need_wait = 1,																					// 1=需要等待完成，0=不需要
//         .action = NULL, 																				// 无额外业务
// 				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },	
// 		/*步骤 13 ：原地旋转到0度，无额外操作*/
//     {
//          .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
//         .params.diff_turn = {1000, -1000, 0},	// 左电机速度，右电机速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 14 ：沿0度直走，无额外操作*/
//     {
// 		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {262500, 1000, 0},	// 目标距离，目标速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 15 ：向左差速走，无额外操作*/
//     {
// 		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//         .params.single_speed = {1000, 1000, 100, 100, 165000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//         .need_wait = 1,																					// 1=需要等待完成，0=不需要
//         .action = NULL, 																				// 无额外业务
// 				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//     },
// 		/*步骤 16 ：原地旋转到-90，云台旋转到识别球的位置*/
//     {
//         .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
//         .params.diff_turn = {1000, -1000, -90},	// 左电机速度，右电机速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 							// 舵机云台和夹爪转到识别位置
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 17 ：沿-90度直走，发送串口识别球*/
//     {
// 		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = {170000, 1000, -90},	// 目标距离，目标速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL,					
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//     },
// 		/*步骤 18 ：判断是否识别到，任务5：识别第一个位置，到就跟踪，完成抓取，没识别到继续走*/
//     {
//         .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 19 ：判断是否识别到，任务6：识别第二个位置，到就跟踪，完成抓取，没识别到继续走*/
//     {
//        .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },
// 		/*步骤 20 ：判断是否识别到，任务7：识别第三个位置，到就跟踪，完成抓取，没识别到继续走*/
//     {
// 		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//     },
// 		/*步骤 21 ：判断是否识别到，任务8：到达标靶识别位置,串口发送识别信号*/
// 	{
// 		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//     },
// 		/*步骤 22 ：判断是否识别到，任务6：追踪射靶子*/
// 	{
// 		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist =	NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					//任务2:等待识别到二维码 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   									// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
// //        .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
// //        .params.single_speed = {1400, 1400, 200, 200, 175000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
// //        .need_wait = 1,																					// 1=需要等待完成，0=不需要
// //        .action = NULL, 																				// 无额外业务
// //				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// //				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 23 */
// 	{
// 		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },	
// 		/*步骤 24 ：判断是否识别到，任务8：识别第一个位置，到就跟踪，完成击靶，没识别到继续走*/
//     {
// 		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//         .params.single_speed = {1000, 1000, 100, 100, 165000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//         .need_wait = 1,																					// 1=需要等待完成，0=不需要
//         .action = NULL, 																				// 无额外业务
// 				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// //		
// 		/*步骤 25 ：判断是否识别到，任务9：识别第二个位置，到就跟踪，完成击靶，没识别到继续走*/
// 	{
	
// 		.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
//         .params.diff_turn = {1000, -1000, 180},	// 左电机速度，右电机速度，目标角度
//         .need_wait = 1,													// 1=需要等待完成，0=不需要
//         .action = NULL, 												// 无额外业务
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
//     },
// 		/*步骤 26 ：判断是否识别到，任务10：识别第三个位置，到就跟踪，完成击靶*/
// 	{
//         .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },
// 		/*步骤 27 ：判断是否识别到，任务11：根据射靶子的位置，判断走多少距离*/
// 	{
//          .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },
// 		/*步骤 28 ：向右差速走，无额外操作*/
//     {
//          .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },
// 		/*步骤 29 ：原地旋转到180度，无额外操作*/
//     {
//          .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL, 					// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },
// 		/*步骤 30 ：沿180度前进，任务12：矫正云台，发送串口识别人质*/
//     {
//         .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL,				
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
//     },
// 		/*步骤 31 ：判断是否识别到，任务4：识别第一个位置，到就跟踪，完成抓取，没识别到继续走*/
//     {
//         .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL,				// 
// 				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// 		/*步骤 32 ：判断是否识别到，任务5：识别第二个位置，到就跟踪，完成抓取，没识别到继续走*/
// 	{
//         .type = STEP_NONE,								//整体走直线，固定距离，固定速度
//         .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
//         .need_wait = 0,													// 1=需要等待完成，0=不需要
//         .action = NULL,					// 
// 				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
// 				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
//     },
// };

// // 计算总步骤数（自动推导，避免硬编码）
// const uint8_t TOTAL_STEPS = sizeof(motion_steps) / sizeof(Motor_control_MotorStep);

/* ========== H题 比赛路径定义 ========== */

/*
 * 路径1：顺时针一圈（要求2/5/6 共用底盘路径）
 * 场地：AB=CD=1.5m 直道，BC/DA 为 R=0.5m 半圆弧
 * 行驶方向：A→B→C→D→A（顺时针）
 * 
 * TODO: 以下脉冲值需实地标定
 *   - 直道脉冲对应实际1.5m
 *   - 弧线内外轮速度比对应 40:60（内R=40cm, 外R=60cm）
 */
Motor_control_MotorStep path_circle[] = {
    // 步骤0：A→B 直走（1.5m）
    {
        .type = STEP_FIXED_DIST,
        .params.fixed_dist = {690300,1000,0},       // 目标脉冲(待标定), 速度, 角度
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤1：B→C 右转弯弧（半圆弧 R=0.5m，弧长≈1.57m）
    {
        .type = STEP_SINGLE_SPEED,
        .params.single_speed = {1000, 690, 695000},  // 左轮(内)慢, 右轮(外)快
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤2：C→D 直走（1.5m）
    {
        .type = STEP_FIXED_DIST,
        .params.fixed_dist = {837564,1000,180},       // 目标脉冲(待标定), 速度, 角度
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤3：D→A 右转弯弧（半圆弧 R=0.5m）
    {
        .type = STEP_SINGLE_SPEED,
        .params.single_speed = {1000, 690, 695000},  // 左轮(内)慢, 右轮(外)快
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
};
#define PATH_CIRCLE_LEN (sizeof(path_circle) / sizeof(Motor_control_MotorStep))

/*
 * 路径2：A→B 直线段（要求4）
 */
Motor_control_MotorStep path_AB[] = {
    // 步骤0：A→B 直走（1.5m）
    {
        .type = STEP_FIXED_DIST,
        .params.fixed_dist = {0, 1000, 0},       // 目标脉冲(待标定)
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤1：到达B点
    {
        .type = STEP_NONE,
        .params.fixed_dist = {0, 0, 0},
        .need_wait = 0,
        .action = NULL,                        // TODO: 球控制业务回调
        .skip_motor = 1,
        .action_repeat = -1
    },
};
#define PATH_AB_LEN (sizeof(path_AB) / sizeof(Motor_control_MotorStep))

/*
 * 路径3：慢速顺时针一圈（配合球稳定控制）
 * 速度降低约40%，给球控制预留收敛时间
 */
Motor_control_MotorStep path_circle_slow[] = {
    // 步骤0：A→B 直走（1.5m）
    {
        .type = STEP_FIXED_DIST,
        .params.fixed_dist = {0, 600, 0},       // 慢速
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤1：B→C 右转弯弧
    {
        .type = STEP_SINGLE_SPEED,
        .params.single_speed = {400, 600, 0},
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤2：C→D 直走（1.5m）
    {
        .type = STEP_FIXED_DIST,
        .params.fixed_dist = {0, 600, 0},
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤3：D→A 右转弯弧
    {
        .type = STEP_SINGLE_SPEED,
        .params.single_speed = {400, 600, 0},
        .need_wait = 1,
        .action = NULL,
        .skip_motor = 0,
        .action_repeat = 0
    },
    // 步骤4：停A点
    {
        .type = STEP_NONE,
        .params.fixed_dist = {0, 0, 0},
        .need_wait = 0,
        .action = NULL,
        .skip_motor = 1,
        .action_repeat = -1
    },
};
#define PATH_CIRCLE_SLOW_LEN (sizeof(path_circle_slow) / sizeof(Motor_control_MotorStep))

/* ========== 路径注册表（mode → 路径映射） ========== */
MotionPath motion_paths[MOTION_PATH_COUNT] = {
    [MODE_IDLE]        = {NULL,             0},
    [MODE_CIRCLE]      = {path_circle,      PATH_CIRCLE_LEN},
    [MODE_AB]          = {path_AB,          PATH_AB_LEN},
    [MODE_CIRCLE_BALL] = {path_circle_slow, PATH_CIRCLE_SLOW_LEN},  // 慢速
};




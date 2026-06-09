#pragma once
#include <cstdint>

class lib_ladrc2
{
private:

	/**
	 * @brief 跟踪微分器结构体
	 * @note  二阶TD，用于安排过渡过程和提取微分信号
	 *        采用参数固化模式，采样周期在初始化时确定
	 */
	typedef struct td_s {
		float x1;       // 跟踪信号（位置）- 平滑后的目标值输出
		float x2;       // 微分信号（速度）- 目标值的变化率
		float r;        // 快速跟踪因子（相当于最大加速度）
		float h;        // 积分步长（采样周期）- 【固化参数】RTOS任务周期
		float h0;       // 滤波因子（用于输入滤波，h0 = N * h）
		float max_x2;   // 最大速度限制（0表示不限制）
	} td_t;

	/**
	 * @brief 二阶LADRC结构体
	 * @note  适用于位置/角度控制等二阶系统
	 *        采用组合模式，内嵌TD模块
	 */
	typedef struct first_order_ladrc_s {
		// ESO增益（二阶观测器）
		float beta1;        // ESO增益beta1 - 位置观测带宽
		float beta2;        // ESO增益beta2 - 速度观测带宽
		float beta3;        // ESO增益beta3 - 扰动观测带宽

		// 控制器增益
		float kp;           // 比例增益 - 控制器刚度
		float kd;           // 微分增益（阻尼）- 注意：LADRC中通常设为0，由b0处理，除非需要额外的PD

		// 系统参数
		float b;            // 控制增益(b0) - 决定控制量的缩放比例
		float dt;           // 采样周期(秒) - 【固化参数】RTOS固定周期

		// 状态估计值
		float x1;           // 估计的位置（跟踪测量值）
		float x2;           // 估计的速度
		float x3;           // 估计的总扰动（包含内部动态和外部扰动）

		//误差
		float error;
		float measure;

		// 输出限制
		float max_output;   // 输出限幅值（如PWM最大值）

		// 抗积分饱和
		float k_aw;         // 抗积分饱和增益（0表示不使用，建议值1.0~3.0）
		float pre_out;      // 上一时刻输出（用于计算饱和程度）

		// 输出
		float out;          // 当前输出

		// TD组合模式（v3.1新增）
		td_t td;            // 内嵌TD结构体
		bool use_td;        // 是否使用TD（在初始化时根据td_r参数决定）
	} first_order_ladrc_t;

	void td_init(td_t *td, float r, float dt, float n, float max_x2);
	float td_fhan(float x1, float x2, float r, float h, float h0);
	float td_update(td_t *td, float target);
	void td_reset(td_t *td, float init_value);

public:
	lib_ladrc2(/* args */);
	~lib_ladrc2();
	void init(float max_output,
		  float beta1,
		  float beta2,
		  float beta3,
		  float kp,
		  float kd,
		  float b,
		  float dt,
		  float k_aw,
		  float td_r,
		  float td_n,
		  float td_max_x2);

	void reset(float x1,
		   float x2,
		   float x3,
		   float td_r,
		   float td_n,
		   float td_max_x2);

	float calc(float target, float measure, float dt);
	void param_update(float max_output, float beta1, float beta2, float beta3, float kp, float kd, float b, float dt, float k_aw);
	first_order_ladrc_t ctl_param;
};

lib_ladrc2::lib_ladrc2(/* args */)
{
}

lib_ladrc2::~lib_ladrc2()
{
}

void lib_ladrc2::param_update(float max_output, float beta1, float beta2, float beta3, float kp, float kd, float b, float dt, float k_aw)
{
	ctl_param.max_output = max_output;

	/* 初始化ESO参数 */
	ctl_param.beta1 = beta1;
	ctl_param.beta2 = beta2;
	ctl_param.beta3 = beta3;

	/* 初始化控制器参数 */
	ctl_param.kp = kp;
	ctl_param.kd = kd;
	ctl_param.b = b;

	ctl_param.dt = dt;
	ctl_param.k_aw = k_aw;

	ctl_param.td.h = dt;
}


/**
 * @brief 一阶LADRC初始化 - 参数固化+TD组合模式
 *
 * @param max_output 控制量输出限幅（例如PWM最大值）
 * @param beta1      ESO状态观测器增益1（位置观测带宽）
 * @param beta2      ESO状态观测器增益2（扰动观测带宽）
 * @param kp         控制器比例增益（刚度）
 * @param b          系统增益估计值(b0) - 决定控制量的缩放比例
 * @param dt         RTOS固定采样周期(秒) - 必须与实际任务频率一致
 * @param k_aw       抗积分饱和增益（建议值1.0~3.0，0表示不启用）
 * @param td_r       TD快速跟踪因子(r) - 决定目标值响应速度
 *                   0表示禁用TD（直接透传目标值）
 * @param td_n       TD滤波因子(无量纲，建议值1~5) - 决定噪声过滤能力
 *                   N=1: 滤波最弱响应最快；N=3~5: 典型推荐值；N>10: 强滤波但滞后明显
 * @param td_max_x2  TD最大速度限制(0表示不限制) - 防止设定值跳变过大导致系统冲击
 */
void lib_ladrc2::init(float max_output,
		     float beta1,
		     float beta2,
		     float beta3,
		     float kp,
		     float kd,
		     float b,
		     float dt,
		     float k_aw,
		     float td_r,
		     float td_n,
		     float td_max_x2)
{

	/* 初始化ESO参数 */
	ctl_param.beta1 = beta1;
	ctl_param.beta2 = beta2;
	ctl_param.beta3 = beta3;

	/* 初始化控制器参数 */
	ctl_param.kp = kp;
	ctl_param.kd = kd;
	ctl_param.b = b;

	/* 初始化状态估计值 */
	ctl_param.x1 = 0.0f;                              // 位置量估计清零
	ctl_param.x2 = 0.0f;                              // 速度量估计清零
	ctl_param.x3 = 0.0f;				  // 扰动量估计清零

	/* 初始化输出限制 */
	ctl_param.max_output = max_output;

	/* 初始化抗积分饱和参数 */
	ctl_param.k_aw = k_aw;
	ctl_param.pre_out = 0.0f;                         // 上一时刻输出清零

	/* 初始化采样周期 - 固化到结构体中 */
	ctl_param.dt = dt;                                // 【关键】后续计算都使用这个固化的dt

	/* 初始化输出 */
	ctl_param.out = 0.0f;

	ctl_param.error = 0.0f;
	ctl_param.measure = 0.0f;

	/* 初始化TD（组合模式）- 使用参数固化模式 */
	if (td_r > 0.0f) {
		td_init(&ctl_param.td, td_r, dt, td_n, td_max_x2);
		// 调用TD初始化，传入滤波因子N
		ctl_param.use_td = true;                      // 启用TD

	} else {
		ctl_param.use_td = false;                     // 禁用TD，目标值直接透传
		// 为了安全，将 TD 状态清零
        	td_reset(&ctl_param.td, 0.0f);
	}
}

/**
 * @brief 一阶LADRC参数重置 - 支持热更新
 *
 * @note 在系统运行过程中动态调整参数，同时重置ESO状态避免瞬态问题
 *       常用于自适应控制、参数调度等场景
 */
void lib_ladrc2::reset(float x1,
		      float x2,
		      float x3,
		      float td_r,
		      float td_n,
		      float td_max_x2)
{
	/* 更新控制参数 */
	// ctl_param.beta1 = beta1;
	// ctl_param.beta2 = beta2;
	// ctl_param.beta3 = beta3;
	// ctl_param.kp = kp;
	// ctl_param.kd = kd;
	// ctl_param.b = b;
	// ctl_param.dt = dt;
	// ctl_param.k_aw = k_aw;

	/* 重置 ESO 状态，避免参数切换时的瞬态问题 */
	ctl_param.x1 = x1;
	ctl_param.x2 = x2;
	ctl_param.x3 = x3;
	ctl_param.pre_out = 0.0f;

	/* 重新配置TD参数 - 使用参数固化模式 */
	if (td_r > 0.0f) {
		/* 安全性检查：滤波因子N不能小于1.0 */
		float n = td_n;

		if (n < 1.0f) {
			n = 1.0f;
		}

		ctl_param.td.r = td_r;
		// ctl_param.td.h = dt;          // 固化采样周期
		ctl_param.td.h0 = n * ctl_param.td.h;     // 计算内部滤波参数 h0 = N * dt
		ctl_param.td.max_x2 = td_max_x2;
		ctl_param.use_td = true;

	} else {
		ctl_param.use_td = false;
	}
}

/**
 * @brief 二阶LADRC计算 - 核心控制算法
 *
 * @param ladrc    二阶LADRC结构体指针
 * @param target   目标值
 * @param measure  系统实际测量值（如编码器读数）
 * @return         控制量输出
 *
 * @note 控制流程：
 *       1. TD平滑目标值（如果启用）
 *       2. 三阶ESO估计系统状态（位置、速度）和总扰动
 *       3. PD控制器计算虚拟控制量u0
 *       4. 扰动补偿得到实际控制量
 *       5. 输出限幅和抗积分饱和处理
 */
float lib_ladrc2::calc(float target, float measure, float dt)
{
	/*
	* 一阶LADRC原理：
	* 被控对象：ẋ = f(x, w, t) + b*u  (一阶系统)
	* 其中f(x,w,t)为总扰动，包含模型不确定性和外部扰动
	*/

	/* 步骤0: TD（跟踪微分器）处理目标值 */
	float td_target = target;
	if (ctl_param.use_td) {
		/* 参数固化模式：不再传入dt，使用结构体中固化的td->h */
		td_target = td_update(&ctl_param.td, target);
							// 获取平滑后的目标值
							// 同时fladrc->td.x2为目标变化率
	}

	/* 步骤1: 执行三阶扩张状态观测器(ESO) */
	/*
	* 二阶LADRC被控对象: ẍ = f + b*u
	* 三阶ESO公式:
	* dx1 = x2 + β1*(y - x1)         <- ẋ1 = x2 (速度)
	* dx2 = x3 + b*u + β2*(y - x1)   <- ẋ2 = x3 + b*u (加速度=扰动+控制)
	* dx3 = β3*(y - x1)              <- ẋ3 = df/dt (扰动变化率，假设变化缓慢)
	*
	* 状态含义:
	* x1 = y (位置估计)
	* x2 = ẏ = v (速度估计)
	* x3 = f(x,ẋ,d) (总扰动估计，包含模型不确定性和外部扰动)
	*
	* 关键设计：使用上一时刻的实际输出(限幅后的pre_out)进行ESO更新，
	*          防止积分饱和导致观测器发散
	*/

	/* 计算ESO微分方程 - 优化：只计算一次误差 */
	float error = measure - ctl_param.x1;              // 观测误差 = 测量值 - 估计值
	ctl_param.error = error;
	ctl_param.measure = measure;
	float dx1 = ctl_param.x2 + ctl_param.beta1 * error;   // 位置估计的微分 = 速度估计 + 修正项
	float dx2 = ctl_param.x3 + ctl_param.b * ctl_param.pre_out + ctl_param.beta2 * error;
							// 速度估计的微分 = 扰动估计 + b*控制量 + 修正项
	float dx3 = ctl_param.beta3 * error;               // 扰动估计的微分（假设扰动变化缓慢）

	/* 更新状态估计值(欧拉积分，乘以dt) */
	if(ctl_param.dt > 0.0001f){				//如果大于0.0001f，为有效值。那么使用设定的dt，否则使用动态的dt
		ctl_param.x1 += dx1 * ctl_param.dt;
		ctl_param.x2 += dx2 * ctl_param.dt;
		ctl_param.x3 += dx3 * ctl_param.dt;
	}else{
		ctl_param.x1 += dx1 * dt;
		ctl_param.x2 += dx2 * dt;
		ctl_param.x3 += dx3 * dt;
	}


	 /* 步骤2: 计算控制量u0 */
	/*
	* 控制律公式:
	* u0 = kp * (r - x1) - kd * x2   // 名义控制：PD控制器
	* u = (u0 - x3) / b              // 扰动补偿：用估计的扰动x3进行前馈补偿
	*
	* 物理意义：通过ESO估计出总扰动x3，在控制量中将其抵消，
	*          使系统变为纯粹的二重积分器 ẍ = b*u0
	*/

	/* 计算名义控制量u0 */
	float u0 = ctl_param.kp * (td_target - ctl_param.x1) - ctl_param.kd * ctl_param.x2;
                                                    // PD控制器：u0 = kp*误差 - kd*速度
                                                    // 注意：LADRC中kd通常设为0，由b0处理阻尼

	/* 抗积分饱和处理 */
	/*
	* 当控制量达到限幅时，ESO中的扰动估计可能会持续累积（积分饱和），
	* 导致系统退出饱和时出现大的超调。
	* 抗积分饱和通过检测饱和误差，调整u0使其退出饱和状态。
	*/
	if (ctl_param.k_aw > 0.0f && fabsf(ctl_param.pre_out) >= ctl_param.max_output * 0.99f) {
		float u_ideal = (u0 - ctl_param.x3) / ctl_param.b;
							// 理论上的理想控制量（无限幅时）
		float saturation_error = ctl_param.pre_out - u_ideal;
							// 饱和误差 = 实际输出 - 理想输出
		u0 -= ctl_param.k_aw * ctl_param.b * saturation_error;
							/* 调整名义控制量u0，使其趋向于退出饱和
							* k_aw越大，退出饱和越快，但可能影响稳态精度 */
	}

	/* 计算理论控制输出u */
	float out_temp;
	if (fabsf(ctl_param.b) < 0.0001f) {               // 安全检查：防止除零
		out_temp = 0.0f;
	} else {
		out_temp = (u0 - ctl_param.x3) / ctl_param.b;   // 扰动补偿：u = (u0 - x2) / b
							// 将估计的扰动x2从控制量中抵消
	}

	/* 输出限幅 */
	if (out_temp > ctl_param.max_output) {
		out_temp = ctl_param.max_output;
	} else if (out_temp < -ctl_param.max_output) {
		out_temp = -ctl_param.max_output;
	}

	/* 保存输出用于下一次ESO计算 */
	ctl_param.out = out_temp;
	ctl_param.pre_out = out_temp;                   // 保存限幅后的输出，用于下一轮ESO更新
							// 【关键】这防止了ESO看到"想要的"控制量，
							// 而是看到"实际的"控制量，避免积分饱和

	return ctl_param.out;
}

/**
 * @brief TD 初始化 - 参数固化模式
 *
 * @param td      TD结构体指针
 * @param r       快速跟踪因子，决定跟踪速度
 * @param dt      固定采样周期(秒) - RTOS任务周期，将固化到结构体中
 * @param n       滤波因子(无量纲) - 建议值1~5，值越大滤波越强但延迟越大
 * @param max_x2  最大速度限制（0表示不限制）
 *
 * @note 参数固化模式的核心逻辑：
 *       1. 固化采样周期：td->h = dt
 *       2. 自动计算内部滤波参数：td->h0 = n * dt
 *       3. 安全检查：如果 n < 1.0，强制设为 1.0，防止数学模型崩溃
 */
void lib_ladrc2::td_init(td_t *td, float r, float dt, float n, float max_x2)
{
	/* 安全性检查：滤波因子N不能小于1.0，否则数学模型会崩溃 */
	if (n < 1.0f) {
		n = 1.0f;
	}

	td->r = r;
	td->h = dt;         /* 固化采样周期到结构体中 */
	td->h0 = n * dt;    /* 自动计算内部滤波参数 h0 = N * dt */
	td->max_x2 = max_x2;
	td->x1 = 0.0f;
	td->x2 = 0.0f;
}

/**
 * @brief fhan 最速控制综合函数（梯形加速度曲线）
 *
 * 这是韩京清教授提出的最速控制综合函数，用于实现时间最优控制。
 * 产生的加速度曲线是梯形的（加速-匀速-减速）。
 *
 * @param x1 位置误差
 * @param x2 速度
 * @param r 快速跟踪因子（相当于最大加速度）
 * @param h 积分步长（采样周期）
 * @param h0 滤波因子（用于输入滤波，典型值为h的5-10倍）
 * @return 加速度输出
 */
float lib_ladrc2::td_fhan(float x1, float x2, float r, float h, float h0)
{
	float d = r * h;                                // 单步速度变化量 (delta v)
	// d = r·h 是离散系统能感知的最小速度单位

	float d0 = h0 * d;                              // 线性区宽度
	// 当误差|y| <= d0时，系统进入线性区平滑处理

	float y = x1 + h * x2;                          /* 预测：当前位置 + h×速度 = 不加控制时的未来位置
                                                     * 产生"超前意识"，抵消离散系统的相位滞后，是不超调的第一道防线 */

	/* 安全检查：限制 y 的范围防止 sqrtf 溢出 */
	const float MAX_Y = 1e15f;

	if (fabsf(y) > MAX_Y) {
		y = (y > 0.0f) ? MAX_Y : -MAX_Y;
	}

	float sqrt_arg = d * d + 8.0f * r * fabsf(y);   // 离散刹车曲线方程的核心
	// 源于等差数列求和公式，8 = 4×2 是离散求和系数

	/* 安全检查：确保 sqrtf 参数非负且有限 */
	if (sqrt_arg < 0.0f || !std::isfinite(sqrt_arg)) {
		sqrt_arg = 0.0f;
	}

	float a0 = sqrtf(sqrt_arg);                     /* 状态解算指标
                                                     * 代表在当前位置误差y下，系统若想最快停下，
                                                     * 理想中应该具备的"速度量级" */

	float a;                                        // 综合切换指标

	if (fabsf(y) <= d0) {                           // 线性区：系统非常靠近目标
		a = x2 + y / h;                             /* 预测下一时刻刚好归零的逻辑
                                                     * 此时fhan退化为PD控制器结构
                                                     * P项: -y/h², D项: -x2/h */

	} else {                                        // 非线性区：系统离目标较远（赶路）
		a = x2 + 0.5f * (a0 - d) * ((y > 0) ? 1.0f : -1.0f);
		/* 0.5*(a0-d) 是基于当前剩余距离y，
		 * 计算出的"当前时刻应该具有的临界速度"
		 * 系统始终保持在最速控制切换曲线上 */
	}

	float fhan_out;                                 // 加速度输出

	if (fabsf(a) <= d) {                            // 线性插值区（接近停稳）
		fhan_out = -r * a / d;                      /* 比例缩放：加速度随靠近目标而逐渐减小
                                                     * 如果不做这个线性处理，最后一步可能输出过大加速度导致跨过原点
                                                     * 通过a/d比例缩放，最终在目标点刚好减为0 */

	} else {                                        // 饱和区（满速加速/刹车）- Bang-Bang控制
		fhan_out = -r * ((a > 0) ? 1.0f : -1.0f);   /* 只有+r或-r两种状态
                                                     * 保证最快响应，油门踩到底或刹车踩到底 */
	}

	return fhan_out;
}

/**
 * @brief TD 更新计算 - 参数固化模式
 *
 * @param td      TD结构体指针
 * @param target  目标值
 * @return        跟踪输出x1（平滑后的目标值）
 *
 * @note 参数固化模式：此函数使用结构体中固化的采样周期td->h，
 *       不再接受外部传入的dt。这适用于RTOS固定频率调用的场景，
 *       可以消除时间抖动对积分的影响。
 */
float lib_ladrc2::td_update(td_t *td, float target) {
    float x1_error = td->x1 - target;               // 计算当前跟踪误差

    /* 使用固化的采样周期td->h进行计算，不再依赖外部传入的dt */
    float fh = td_fhan(x1_error, td->x2, td->r, td->h, td->h0);
                                                    // 调用fhan计算最优加速度

    /* 使用固化的td->h进行积分，确保时间一致性 */
    float new_x2 = td->x2 + td->h * fh;             // 速度积分：v = v0 + a·dt

    /* 速度限制检查 */
    if (td->max_x2 > 0.0f) {
        if (new_x2 > td->max_x2) {
            new_x2 = td->max_x2;
        } else if (new_x2 < -td->max_x2) {
            new_x2 = -td->max_x2;
        }
    }

    td->x2 = new_x2;                                // 更新速度状态
    /* 使用固化的td->h进行位置积分 */
    td->x1 = td->x1 + td->h * td->x2;               // 位置积分：x = x0 + v·dt

    return td->x1;                                  // 返回平滑后的目标值
}

/**
 * @brief TD 重置状态
 *
 * @param td         TD结构体指针
 * @param init_value 初始值
 *
 * @note 在系统复位、模式切换或故障恢复时调用，
 *       将TD状态重置为指定值，避免历史状态影响新的控制过程
 */
void lib_ladrc2::td_reset(td_t *td, float init_value) {
    td->x1 = init_value;                            // 位置重置为初始值
    td->x2 = 0.0f;                                  // 速度重置为0
}

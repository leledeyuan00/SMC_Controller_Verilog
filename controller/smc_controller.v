module smc_controller(
	input signed [31:0]u_t,
	input signed [31:0]theta_dot,
	input signed [31:0]theta_e,
	input signed [31:0]theta_dote,
	output signed [15:0]u,
	output signed [15:0]u_out
);

	//debug
	//reg limit1,limit2,limit3,limit4,limit5,limit6,limit7,limit8,limit9,limit10,limit11,limit12,limit13;
	//assign limit = {limit13,limit12,limit11,limit10,limit9,limit8,limit7,limit6,limit5,limit4,limit3,limit2,limit1};
	//SMC controller variable
	reg signed [31:0] theta_e_r;
	reg signed [31:0] theta_dote_r;
	parameter [3:0]k = 4'd15;
	parameter [3:0]dm = 4'd0;
	parameter [4:0]lambda = 5'd20;
	//parameter [3:0]ba = 4'd2; //?*ba --> modified "?<<1";
	parameter [12:0]ja = 13'b0001100110011; //0.1 usfix13_En12
	parameter [15:0]divided_eff = 16'b1011011011011011; // sfix16_En14/0.35 --> *2.85714 => 2.85711669921875
	//parameter [15:0] portional = 16'b0000010101110101; 
	reg signed [31:0]s_smc;
	reg signed [31:0]s_smc1;
	reg signed [31:0]h_smc;
	reg signed [31:0]h_smc1;
	reg signed [31:0]h_smc2;
	reg signed [31:0]h_smc2_2;
	reg signed [31:0]h_smc2_2_abs;
	reg signed [31:0]h_smc2_2_1;
	reg signed [31:0]h_smc2_2_2;
	reg signed [31:0]h_smc3;
	reg signed [31:0]h_smc3_abs;
	reg signed [31:0]u_smc;
	reg signed [31:0]u_smc1;
	reg signed [31:0]u_smc2;
	reg signed [31:0]u_smc3;
	//wire signed [15:0]u_out;

	//SMC_Controller
	always@(*) begin //TODO: pipeline
	//s=en_dot+lambda*en

		{s_smc1} = theta_e * lambda;
		// negative sambol over 
		{s_smc} = s_smc1 + theta_dote;
		//h=dm+0.5*ja*abs(u/0.35-lambda*theta_dot)+ba*abs(theta_dot)

		{h_smc2_2_1} = ((u_t>>>10) *divided_eff)>>>14;

		{h_smc2_2_2} = theta_dot * lambda;

		{h_smc2_2} = h_smc2_2_1 - h_smc2_2_2;

		if(!h_smc2_2[31])
			{h_smc2_2_abs} = h_smc2_2; //sfix
		else
			{h_smc2_2_abs} = -h_smc2_2;

		{h_smc2} = (h_smc2_2_abs * ja) >>> 13; //>>12 + 1

		{h_smc3} = theta_dot <<<1;

		if(!h_smc3[31])
			{h_smc3_abs} = h_smc3; //sifx
		else
			{h_smc3_abs} = -h_smc3;
			//h_smc3 = h_smc3_1_abs >>>1; //sfix
		{h_smc} = h_smc2 + h_smc3_abs +(dm <<< 10 );
		//y=-K*s-h*sign(s)+ja*(u/0.35-lambda*theta_dot)+ba*(theta_dot)

		{u_smc1} = k * (s_smc);

		if(!s_smc[31])
			u_smc2 = h_smc;
		else
			u_smc2 = -h_smc;
		{u_smc3} = (ja * h_smc2_2)>>>12;
		{u_smc} = h_smc3 + u_smc3 - u_smc2 -u_smc1;
	end
	assign u_out = u_smc[31:16]; //{{16'b0,u_smc} * portional}>>16;
	assign u = u_out >>>4;
endmodule
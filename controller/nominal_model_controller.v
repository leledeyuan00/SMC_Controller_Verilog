

module nominal_model_controller(
	input signed [31:0]e, //sfix32_En10
	input signed [31:0]e_dot, //sfix32_En10
	input signed [31:0]theta_d_a, //int
	output signed[31:0]u_t //sfix32_En10
);

	//Nominal module Controller variable
	reg signed [31:0] e1;
	reg signed [31:0] e_dot1;
	reg signed [31:0] u1;
	reg signed [31:0] u2;
	parameter [19:0]h1 = 20'd10000;
	parameter [14:0]h2 = 15'd200;

	//Nominal module controller
	/*
	 * y = 0.5*(-h1*e-h2*e_dot+desired_theta_dot2)
	*/
	always@(*) begin
		e1 = h1 * e;
		e_dot1 = h2 * e_dot;
		u1 = (theta_d_a <<< 10)- e_dot1;
		u2 = u1 - e1;
	end
	assign u_t = u2>>>1;
endmodule
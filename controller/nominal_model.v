//Nominal model
/*
	 * theta_dot2n = 0.500*u
	 * theta_dot1n=theta_dot1n+sampletime*theta_dot2n
	 * thetan=thetan+sampletime*theta_dot1n
	 */

`include "controller_defines.v"

module nominal_model(
	input clk,
	input rst_n,

	input signed[31:0] u_t, //sfix32_En10
	output signed[31:0] model_u, //sfix32_En10
	output signed[31:0] model_udot //sfix32_En10
);

	//Nominal model variable
	//sfix32_En10
	localparam sampletime = 12'b0000000000000110100011; //fix22En22
	reg signed[31:0] model_udot_r;
	reg signed[31:0] theta_dot1n_curr;
	reg signed[31:0]theta_dot1n_next;
	reg signed[31:0] thetan_curr;
	reg signed[31:0] thetan_next;
	reg signed[63:0] theta_dot1n_t;
	reg signed[31:0] theta_dot2n;
	reg signed[63:0] theta_dot2n_t;

	always@(*) begin //TODO: pipeline 
		theta_dot2n = u_t>>>1;
		theta_dot2n_t = theta_dot2n * sampletime ;// sfix54_En32
		theta_dot1n_next = theta_dot1n_curr + (theta_dot2n_t>>>22);
		theta_dot1n_t = theta_dot1n_next * sampletime;
		thetan_next = (theta_dot1n_t>>>22) + thetan_curr;
	end

	always@(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			theta_dot1n_curr <= 0;
		end
		else if(clk) begin
			theta_dot1n_curr <= theta_dot1n_next;
		end
	end
	always@(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			thetan_curr <= 0;
		end
		else if(clk) begin
			thetan_curr <= thetan_next;
		end
	end
	
	assign model_u = thetan_next;
	assign model_udot = theta_dot1n_next;
endmodule
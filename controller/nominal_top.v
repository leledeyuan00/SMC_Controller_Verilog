/* 
 * --------------------
 * Company					: Shenzhen MileBot Robotics Tech Co., Ltd.
 * --------------------
 * Project Name				: SEA
 * model Name				: Drive_nominal
 * Description				:
 * --------------------
 * Tool Versions			: Quartus II 13.1
 * Target Device			: Cyclone IV E  EP4CE15F
 * --------------------
 * Engineer					: Dayuan
 * Revision					: V1.0
 * Created Date				: 2018-11-04
 * --------------------
 * Engineer					:
 * Revision					:
 * Modified Date			:
 * --------------------
 * Additional Comments	: PLUSE_DESIRE
 * 
 * --------------------
 */
//-------------------------Timescale----------------------------//
`include "controller_defines.v"
//------------------------Macro_Define--------------------------//
/*  
	 * ---fix-point sfix16_En13  
	 * ---3.1415(d)(pi)  = 011.0010010000111(b) >= 3.141357421875
	 * ---6.2831(d)(2pi) = 110.0100100001110(b) >= 6.282958984375
	 * ---error = 0.000244140625
	 */
`define PI 		 (16'b0110010010000111)
`define PI_2     (16'b1100100100001110)
//--------------------model_PLUSE---------------------//
module nominal
(
	input cs,
	input rst_n,
	input rd,
	input wr,
	input clk,

	input rdempty,
	input [7:0] usedfifo,
	input [15:0] rddata,
	input [3:0] addr,
	input [15:0] indata,
	input [15:0] backdata,

	output cs_code,
	output rd_code,
	output cs_rate,
	output wr_rate,
	output pul_enable,
	output [3:0] outaddr,
	output [15:0] rate,
	output [15:0] outdata,

	output rdreq,
	output rdclk,
	output fifoclr
);
	/*---------Add user variable here---------*/
	wire rdn,wrn;

	wire clk_h,clk_l;
	reg clk_c;

	//fifo
	reg fifo_no_empty;
	reg rd_req;
	reg rd_empty;
	reg rd_clk;
	reg fifo_clr;

	//encoder
	localparam delay_vel = 100<<4;
	reg [(delay_vel)-1:0] delay_theta_r;
	reg cs_code_out;
	reg rd_code_out;
	reg[3:0] outaddr_out;
	reg signed[15:0] tama_temp,tama_temp_r;
	reg signed[15:0] highdata;
	reg signed[15:0] lowdata;
	reg[15:0] insignal;
	reg [15:0] backsignal;
	wire signed[31:0] backsignal32;

	//driver
	reg cs_rate_out;
	reg wr_rate_out;
	reg pul_enable_out;
	reg[15:0] rateout;
	reg[15:0] u;

	//algorithm
	wire signed[15:0] theta_delay;
	wire signed[31:0] theta_delay_t1, theta_dot; //int
	wire signed[31:0] theta_e, theta_dote; //sfix32_En10
	wire signed[31:0] theta_d_a; // int
	wire signed[31:0] e,e_dot; //sifx32_En10
	reg signed [31:0] theta_d_r1, theta_d_r2;
	reg signed [31:0]theta_d_v1, theta_d_v2;
	wire signed[31:0] model_u, model_udot; //sfix32_En10
	reg signed[31:0] model_u_r, model_udot_r; //sfix32_En10

	wire[31:0] u_t; //sfix32_En10

	reg signed[31:0] theta_dot_r; //int
	reg[31:0] theta_e_r, theta_dote_r; //sfix32_En10
	wire[15:0] controller_u;

	reg start;
	reg rdfifobegin;
	reg[7:0] used_info;
	reg[7:0] status;
	reg[15:0] db_fifo;
	reg[15:0] dataout;
	reg[15:0] databuf;
	reg[15:0] insignalbuf;

	reg[15:0] theta_d;
	wire signed[31:0] theta_d32;

	reg[31:0] debug;
	reg[15:0] debugbuf;
	reg[15:0] error_cnt;
	wire[31:0] u_out;
	/*---------Add user logic here -------------*/
	assign rdn = cs|rd;
	assign wrn = cs|wr;


	/* --------- timer -----------------*/
	control_timer inst1 (clk, rst_n, clk_h, clk_l);


	/*---------  model inst----------- */
	nominal_model_controller inst2 (e, e_dot, theta_d_a, u_t);

	nominal_model inst3 (clk_c, rst_n, u_t, model_u, model_udot);

	smc_controller inst4 (u_t, theta_dot_r, theta_e_r, theta_dote_r, controller_u,u_out);

	//Encoder position and velocity calculated logic
	assign backsignal32 = backsignal[15]?{16'hffff,backsignal}:{16'h0,backsignal};
	always@(posedge clk_c or negedge rst_n)begin
		if(!rst_n)
			delay_theta_r <= 0;
		else begin
			delay_theta_r[15:0] <= backsignal;
			delay_theta_r[(delay_vel)-1:16] <= delay_theta_r[(delay_vel)-17:0];
		end
	end

	assign theta_delay = delay_theta_r[(delay_vel)-1:(delay_vel)-16];
	assign theta_delay_t1 = (theta_delay[15]?{16'hffff,theta_delay}:{16'h0,theta_delay}) - backsignal32;
	assign theta_dot = theta_delay_t1 * 7'd100;


	//Desired position and velocity and accelerate
	assign theta_d32 = (theta_d[15]?{16'hffff,theta_d}:{16'h0,theta_d});
	always@(posedge clk_c or negedge rst_n)begin
		if(!rst_n)begin
			theta_d_r1 <= 0;
			theta_d_r2 <= 0;
		end
		else begin
			theta_d_r1 <= theta_d32;
			theta_d_r2 <= theta_d_r1;
		end
	end
	always@(*)begin
		theta_d_v1 = theta_d32 - theta_d_r1;
		theta_d_v2 = theta_d_r2 - theta_d_r1;
	end
	assign theta_d_a = theta_d_v2 - theta_d_v1;

	always @(posedge clk_c or negedge rst_n) begin
		if(!rst_n)
			model_u_r <= 0;
		else
			model_u_r <= model_u;
	end
	always @(posedge clk_c or negedge rst_n) begin
		if(!rst_n)
			model_udot_r <= 0;
		else
			model_udot_r <= model_udot;
	end
	assign e = model_u_r - (theta_d32 <<< 10);
	assign e_dot = model_udot_r - (theta_d_v1<<<10);

	//SMC_Controller VARIABLE
	assign theta_dote = theta_dot<<<10 - model_udot;
	assign theta_e = (backsignal32<<<10) - model_u;
	always@(posedge clk_c or negedge rst_n)begin
		if(!rst_n)
			theta_dot_r <= 32'd0;
		else
			theta_dot_r <= theta_dot;
	end
	always @(posedge clk_c or negedge rst_n) begin
		if(!rst_n)
			theta_dote_r <= 32'd0;
		else
			theta_dote_r <= theta_dote>>>10;
	end
	always @(posedge clk_c or negedge rst_n) begin
		if(!rst_n)
			theta_e_r <= 32'd0;
		else
			theta_e_r <= theta_e>>>10;
	end

	/*------------- read ----------- */
	always @(negedge rdn or negedge rst_n)
	begin
		if(!rst_n)
			begin

			end
		else
			begin
				if(!rdn)
					begin
						case (addr)
							4'd1:
							begin
								dataout <= {backsignal32[15:0]};
								databuf <= {backsignal32[31:16]};
							end
							4'd2:
							begin
								dataout <= databuf;
							end
							4'd3:
							begin
								dataout <= {start_flag,start_flag_1,start_flag_2};
							end
							4'd4: dataout <= u;

							4'd5: dataout <= {pul_enable_out,rdfifobegin,start};
							4'd6: dataout <= rateout;
							4'd7: dataout <= theta_d;
							4'd8: dataout <= controller_u;
							4'd9:
							begin
								dataout <= u_out[15:0];
								debugbuf <= u_out[31:16];
							end
							4'd10: dataout <= debugbuf;
							4'd11:
							begin
								dataout <= theta_delay_t1[15:0];
								debugbuf <= theta_delay_t1[31:16];
							end
							4'd12: dataout <= debugbuf;
							4'd13:
							begin
								dataout <= theta_dote_r[15:0];
								debugbuf <= theta_dote_r[31:16];
							end
							4'd14: dataout <= debugbuf;
							default:;
						endcase
					end
			end
	end

	/*------------- write ------------- */
	/*
	always@(negedge wrn or negedge rst_n)begin
		if(!rst_n) begin
			start <= 1'd0;
			rdfifobegin <= 1'd0;
			pul_enable_out <= 1'd0;
		end
		else if(!wrn)begin
			case(addr)
				4'd1:{insignal[15:0]} <= indata;
				4'd2:;
				4'd3:;
				4'd4:;
				4'd5:
				begin
					case(indata)
						16'd0:
						begin
							start <= 1'd0;
							rdfifobegin <= 1'd0;
							pul_enable_out <= 1'd0;
						end
						16'd1:
						begin
							start <=1'd1;
							rdfifobegin <= 1'd1;
							pul_enable_out <=1'd1;
						end
						16'd2:
						begin
							rdfifobegin <= 1'd0;
						end
						16'd3:
						begin
							rdfifobegin <= 1'd1;
						end
						default:;
					endcase
				end
				default:;
			endcase
		end
	end
	
	*/


	/*------------- for debug --------- */
	`ifdef DEBUG
	always@(*)
	begin
		debug[15:0] = rataout;
			wire signed [31:0]debug_variable1 ;
	wire signed [31:0]debug_variable2;
	assign debug_variable1 = 32'h8ffffff0 * 2'd2;
	assign debug_variable2 = (debug_variable1 >>>3);
	end
	`endif

	/*------------start logic ------ */
	wire start_flag;
	reg start_flag_1,start_flag_2;
	assign start_flag = start_flag_1 ^ start_flag_2;
	always@(posedge clk_l,negedge rst_n) begin
		if(!rst_n)begin
			start_flag_1<=0;
		end
		else
			if(status == 0)
				start_flag_1 <= !start_flag_1;
			else begin
				start_flag_1 <= start_flag_1;
				debug <= debug + 1'd1;
			end
	end
	/*-------------- main -----------*/
	always@(posedge clk_h or negedge rst_n or negedge wrn)begin
		if(!rst_n)begin
			status<=  8'd0;
			rateout<= 16'd5000;
			u <= 16'd0;
			error_cnt <= 16'd0;
			start_flag_2 <= 1'd0;

			tama_temp <= 16'd0;
			tama_temp_r <= 16'd0;

			rdfifobegin <= 1'd0;
			fifo_no_empty <= 1'd0;
			fifo_clr <= 1'd0;
			start  <= 1'd0;
			rd_code_out <= 1'd1;
			wr_rate_out <= 1'd1;
			cs_code_out <= 1'd1;
			cs_rate_out <= 1'd1;
			pul_enable_out <= 1'd0;
			rd_clk <= 1'd0;
			theta_d <= 16'd0;
		end
		else begin
			if(!wrn)begin
				case (addr)
					4'd1:
					begin
						theta_d <= indata;
					end
					4'd2:
					begin
						//{backsignal[31:16]} <= indata;
					end
					4'd3:
					begin
						//{insignal[15:0]} <= indata;
					end
					4'd4:
					begin
						// {insignal[31:16]} <= indata;
					end
					4'd5:
					begin
						case (indata)
							16'd0:
							begin
								start <= 1'd0;
								rdfifobegin <= 1'd0;
								pul_enable_out <= 1'd0;
							end
							16'd1:
							begin
								start <= 1'd1;
								rdfifobegin <= 1'd1;
								pul_enable_out <= 1'd1;
							end
							16'd2:
							begin
								rdfifobegin <= 1'd0;
							end
							16'd3:
							begin
								rdfifobegin <= 1'd1;
							end
							default:;
						endcase
					end
					default:;
				endcase
			end
			else begin
				if(clk_h)begin
					case(status)
						8'd0:
						if(start_flag)
							status <= status + 1'd1;
						8'd1:
						begin
							if(rdfifobegin)
								begin
									rd_clk <= 1'd1;
									status <= status +1'd1;
								end
							else status <= 8'd9;
						end
						8'd2:
						begin
							db_fifo <= rddata;
							used_info <= usedfifo;
							rd_empty <= rdempty;
							status <= status + 1'd1;
						end
						8'd3:
						begin
							if(fifo_no_empty)begin
								//	theta_d[15:0] <= db_fifo;
								error_cnt <= error_cnt +1'd1;
							end
							rd_clk <= 1'd0;
							status <= status + 1'd1;
						end
						8'd4:
						begin
							if(rd_empty == 1'd0)
								fifo_no_empty <= 1'd1;
							else
								fifo_no_empty <= 1'd0;
							status <= status + 1'd1;
						end
						8'd5:
						begin
							outaddr_out <= 4'd1;
							cs_code_out <= 1'd0;
							status <= status + 1'd1;
						end
						8'd6:
						begin
							rd_code_out <= 1'd0;
							status <= status +1'd1;
						end
						8'd7: status <= status + 1'd1;
						8'd8:
						begin
							cs_code_out <= 1'd1;
							rd_code_out <= 1'd1;
							status <= status +1'd1;
						end
						8'd9:
						begin
							outaddr_out <= 4'd2;
							cs_code_out <= 1'd0;
							status <= status +1'd1;
						end
						8'd10:
						begin
							rd_code_out <= 1'd0;
							status <= status + 1'd1;
						end
						8'd11:
						begin
							lowdata <= backdata;
							status <= status +1'd1;
						end
						8'd12:
						begin
							rd_code_out <= 1'd1;
							cs_code_out <= 1'd1;
							status <= status +1'd1;
						end
						8'd13:
						begin
							outaddr_out <= 4'd3;
							cs_code_out <= 1'd0;
							status <= status +1'd1;
						end
						8'd14:
						begin
							rd_code_out <= 1'd0;
							status <= status + 1'd1;
						end
						8'd15:
						begin
							highdata <= (backdata& 4'h1);
							status <= status + 1'd1;
						end
						8'd16:
						begin
							tama_temp[14:0] <= {highdata[0],lowdata[15:2]};
							status <= status + 1'd1;
						end
						8'd17:
						begin
							if(((tama_temp<15'd100 && tama_temp_r > 15'd32567)||(tama_temp_r<15'd100 && tama_temp > 15'd32567)) && backsignal[15] == 0)
								backsignal[15] <= 1;
							else if(((tama_temp<15'd100 && tama_temp_r > 15'd32567)||(tama_temp_r<15'd100 && tama_temp > 15'd32567)) && backsignal[15] == 1)
								backsignal[15] <= 0;
							status <= status + 1'd1;
						end
						8'd18:
						begin
							tama_temp_r <= tama_temp;
							rd_code_out <= 1'd1;
							cs_code_out <= 1'd1;
							backsignal[14:0] <= {highdata[0],lowdata[15:2]};
							if(start)
								status <= status +1'd1;
							else
								status <= 1'd0;
						end
						8'd19:
						begin
							clk_c <= 1'd1;
							status <= status + 1'd1;
						end
						8'd20,8'd21,8'd22,8'd23,8'd24,8'd25,8'd26,8'd27,8'd28,8'd29,8'd30:
						status <= status + 1'd1;
						8'd31:
						begin
							if(controller_u >= 16'd2000 && controller_u < 16'h8000)
								u <= 16'd2000;
							else if (controller_u >= 16'h8000 && controller_u <= 16'hF830)
								u<= 16'hF830;
							else
								u<= controller_u;
							status <= status +1'd1;
						end
						8'd32:
						begin
							rateout <= 16'd5000 + {u[15:0]};
							status <= status + 1'd1;
						end
						8'd33:
						begin
							outaddr_out <= 4'd1;
							status <= status + 1'd1;
						end
						8'd34:
						begin
							cs_rate_out <= 1'd0;
							status <= status + 1'd1;
						end
						8'd35:
						begin
							wr_rate_out <= 1'd0;
							status <= status + 1'd1;
						end
						8'd36:
						begin
							wr_rate_out <= 1'd1;
							status <= status + 1'd1;
						end
						8'd37:
						begin
							cs_rate_out <= 1'd1;
							status <= 8'd40;
						end

						8'd40:
						begin
							clk_c <= 1'd0;
							status <= 8'd0;
							start_flag_2 <= !start_flag_2;
						end

						default status<=8'd0;
					endcase
				end
			end
		end
	end

	assign rd_code = rd_code_out;
	assign outaddr = outaddr_out;
	assign wr_rate = wr_rate_out;
	assign cs_code = cs_code_out;
	assign cs_rate = cs_rate_out;
	assign pul_enable = pul_enable_out;
	assign rate = rateout;

	assign rdreq = rd_req;
	assign rdclk = rd_clk;
	assign fifoclr = fifo_clr;
	assign outdata = !rdn ? dataout : 16'hzzzz;
endmodule

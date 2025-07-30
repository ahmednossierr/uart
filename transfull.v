
///                                                                  بسم الله الرحمن الرحيم    

/////                                               topmodule
module uart_whole(
    input wire  start, rx_in, clk, rst , input wire [7:0] send_data,
    output wire busy, tx_out, rx_done, rx_fail, output wire [7:0] rx_data);
    wire baud_sig;
    wire samp_sig;

    baud baud_inst(.clk(clk),.rst(rst),.baud_pulse(baud_sig),.sample_pulse(samp_sig));

    trans trans_inst(.clk(clk),.rst(rst),.tick(baud_sig),.data_in(send_data),.go(start),.serial_out(tx_out),.is_busy(busy) );

    reciv reciv_inst(.clk(clk),.reset(rst),.sample_tick(samp_sig),.rx_in(rx_in),.data_out(rx_data),.ready(rx_done),.err(rx_fail));

endmodule
///                     baud
module baud(
    input wire clk, rst,
    output reg baud_pulse, sample_pulse
);

    reg [15:0] baud_count;
    reg [15:0] sample_count;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
             sample_count <= 0;
            baud_pulse <= 0;
            sample_pulse <= 0;
            baud_count <= 0;
           
        end else begin
            if (baud_count == 5207) begin
                baud_count <= 0;
                baud_pulse <= 1;
            end else begin
                baud_count <= baud_count + 1;
                baud_pulse <= 0;
            end
            if (sample_count == 324) begin
                sample_count <= 0;
                sample_pulse <= 1;
            end else begin
                sample_count <= sample_count + 1;
                sample_pulse <= 0;
            end
        end
    end

endmodule

//////                  transmimission

module trans
    ( input wire clk, rst, tick, go ,input wire [7:0] data_in,        
    output reg serial_out, is_busy);
    reg [3:0] state;
    reg [7:0] shift;
    reg [3:0] bit_pos;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= 0;
            serial_out <= 1;
            is_busy <= 0;
            shift <= 0;
            bit_pos <= 0;
        end else begin
            case (state)
                0: if (go) begin
                    shift <= data_in;
                    state <= 1;
                    is_busy <= 1;
                end
                1: if (tick) begin
                    serial_out <= 0;
                    state <= 2;
                    bit_pos <= 0;
                end
                2: if (tick) begin
                    serial_out <= shift[0];
                    shift <= shift >> 1;
                    bit_pos <= bit_pos + 1;
                    if (bit_pos == 7) state <= 3;
                end
                3: if (tick) begin
                    serial_out <= 1;
                    state <= 0;
                end
            endcase
        end
    end

endmodule

module reciv(
    input wire clk,reset, sample_tick,rx_in,
    output reg [7:0] data_out, output reg ready, err);
    reg [3:0] rx_phase;
    reg [3:0] samples;
    reg [3:0] bits;
    reg [7:0] buffer;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rx_phase <= 0;
            data_out <= 0;
            ready <= 0;
            err <= 0;
            samples <= 0;
            bits <= 0;
            buffer <= 0;
        end else if (sample_tick) begin
            case (rx_phase)
                0: if (rx_in == 0) begin
                    rx_phase <= 1;
                    samples <= 0;
                end
                1: begin
                    samples <= samples + 1;
                    if (samples == 7) begin
                        if (rx_in == 0) begin
                            rx_phase <= 2;
                            samples <= 0;
                            bits <= 0;
                        end else begin
                            rx_phase <= 0;
                        end
                    end
                end
                2: begin
                    samples <= samples + 1;
                    if (samples == 15) begin
                        buffer <= {rx_in, buffer[7:1]};
                        bits <= bits + 1;
                        samples <= 0;
                        if (bits == 7) rx_phase <= 3;
                    end
                end
                3: begin
                    samples <= samples + 1;
                    if (samples == 15) begin
                        if (rx_in == 1) begin
                            data_out <= buffer;
                            ready <= 1;
                            rx_phase <= 0;
                        end else begin
                            err <= 1;
                            rx_phase <= 0;
                        end
                    end
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (ready) ready <= 0;
        if (err) err <= 0;
    end

endmodule

module uart_tb;

    reg clk;
    reg rst;
    reg [7:0] tx_input;
    reg tx_trigger;
    wire tx_status;
    wire tx_wire;
    wire rx_wire;
    wire [7:0] rx_output;
    wire rx_complete;
    wire rx_error;

    assign rx_wire = tx_wire;

    uart_whole device(
        .clk(clk),
        .rst(rst),
        .send_data(tx_input),
        .start(tx_trigger),
        .busy(tx_status),
        .tx_out(tx_wire),
        .rx_in(rx_wire),
        .rx_data(rx_output),
        .rx_done(rx_complete),
        .rx_fail(rx_error)
    );

    initial begin
        clk = 0;
        forever #10 clk = ~clk;
    end

    initial begin
        rst = 1;
        tx_trigger = 0;
        tx_input = 8'h00;
        #100;
        rst = 0;
        #100;

        tx_input = 8'hAA;
        tx_trigger = 1;
        #20;
        tx_trigger = 0;
        wait(tx_status == 0);
        wait(rx_complete);
        if (rx_output == 8'hAA && !rx_error) $display("AA worked fine");
        else $display("AA messed up");

        #100;

        tx_input = 8'h55;
        tx_trigger = 1;
        #20;
        tx_trigger = 0;
        wait(tx_status == 0);
        wait(rx_complete);
        if (rx_output == 8'h55 && !rx_error) $display("55 worked fine");
        else $display("55 messed up");

        #100;
        $finish;
    end

endmodule


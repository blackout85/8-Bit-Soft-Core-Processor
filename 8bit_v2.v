// 8-Bit CPU with UART Terminal (Verilog Version - Standard Compatible)
// Targeted for Lattice iCEstick

module top (
    input  clk,      // 12MHz onboard clock
    input  uart_rx,  // From PC (iCEstick Pin 9)
    output uart_tx,  // To PC (iCEstick Pin 8)
    output [4:0] leds // Onboard LEDs
);

    // --- CPU STATE SIGNALS ---
    reg [7:0] pc = 0;
    reg [7:0] reg_a = 0;
    reg [4:0] led_reg = 0;
    reg [1:0] state = 0;
    
    localparam ST_FETCH   = 2'd0;
    localparam ST_DECODE  = 2'd1;
    localparam ST_EXECUTE = 2'd2;
    localparam ST_WAIT    = 2'd3;

    wire [7:0] instr;

    // --- UART SIGNALS ---
    reg [7:0] tx_data;
    reg tx_start = 0;
    wire tx_busy;
    wire rx_ready;
    wire [7:0] rx_data;
    reg rx_ack = 0;

    // --- ROM PROGRAM ---
    reg [7:0] rom [0:15];
    integer i; // Declaration outside the loop for Verilog-2001 compatibility

    initial begin
        // The Program:
        rom[0] = 8'h40; // IN A (Wait for character from Serial terminal)
        rom[1] = 8'h50; // OUT A (Send that character back to terminal)
        rom[2] = 8'hF0; // STORE A (Show the binary value on the LEDs)
        rom[3] = 8'h60; // JUMP 0 (Repeat forever)
        
        // Fill the rest of the 16-byte ROM with NOP (0x00)
        for (i = 4; i < 16; i = i + 1) begin
            rom[i] = 8'h00;
        end
    end

    // Instruction is always pointed to by the PC
    assign instr = rom[pc[3:0]];

    // --- CPU LOGIC ---
    always @(posedge clk) begin
        case (state)
            ST_FETCH: begin
                rx_ack <= 0;
                tx_start <= 0;
                state <= ST_DECODE;
            end

            ST_DECODE: begin
                state <= ST_EXECUTE;
            end

            ST_EXECUTE: begin
                case (instr[7:4])
                    4'h4: begin // IN A: Wait for UART RX
                        if (rx_ready) begin
                            reg_a <= rx_data;
                            rx_ack <= 1; // Tell UART we read the data
                            pc <= pc + 1;
                            state <= ST_FETCH;
                        end else begin
                            state <= ST_WAIT; // Not ready yet, wait
                        end
                    end
                    4'h5: begin // OUT A: Send to UART TX
                        tx_data <= reg_a;
                        tx_start <= 1;
                        state <= ST_WAIT;
                    end
                    4'h6: begin // JUMP: Jump to address in lower 4 bits
                        pc <= instr[3:0];
                        state <= ST_FETCH;
                    end
                    4'hF: begin // STORE LEDs: Copy lower 5 bits of A to LEDs
                        led_reg <= reg_a[4:0];
                        pc <= pc + 1;
                        state <= ST_FETCH;
                    end
                    default: begin // NOP or Unknown
                        pc <= pc + 1;
                        state <= ST_FETCH;
                    end
                endcase
            end

            ST_WAIT: begin
                tx_start <= 0;
                // Keep waiting for RX if we are in an 'IN' instruction
                if (instr[7:4] == 4'h4 && rx_ready) 
                    state <= ST_EXECUTE;
                // Wait for TX to finish before moving to next instruction
                else if (instr[7:4] == 4'h5 && !tx_busy) begin
                    pc <= pc + 1;
                    state <= ST_FETCH;
                end
            end
        endcase
    end

    assign leds = led_reg;

    // --- UART TRANSMITTER (9600 Baud @ 12MHz) ---
    uart_tx_module transmitter (
        .clk(clk),
        .data(tx_data),
        .start(tx_start),
        .busy(tx_busy),
        .tx_pin(uart_tx)
    );

    // --- UART RECEIVER (9600 Baud @ 12MHz) ---
    uart_rx_module receiver (
        .clk(clk),
        .rx_pin(uart_rx),
        .ready(rx_ready),
        .ack(rx_ack),
        .data(rx_data)
    );

endmodule

// --- Sub-Module: UART Transmitter ---
module uart_tx_module (
    input clk,
    input [7:0] data,
    input start,
    output reg busy = 0,
    output reg tx_pin = 1
);
    parameter BIT_PERIOD = 1250; // 12MHz / 9600
    reg [11:0] clk_count = 0;
    reg [3:0] bit_idx = 0;
    reg [9:0] shift_reg;

    always @(posedge clk) begin
        if (!busy) begin
            if (start) begin
                shift_reg <= {1'b1, data, 1'b0}; // Stop bit, data, start bit
                busy <= 1;
                bit_idx <= 0;
                clk_count <= 0;
            end
        end else begin
            if (clk_count < BIT_PERIOD) begin
                clk_count <= clk_count + 1;
            end else begin
                clk_count <= 0;
                tx_pin <= shift_reg[bit_idx];
                if (bit_idx < 9) bit_idx <= bit_idx + 1;
                else busy <= 0;
            end
        end
    end
endmodule

// --- Sub-Module: UART Receiver ---
module uart_rx_module (
    input clk,
    input rx_pin,
    input ack,
    output reg ready = 0,
    output reg [7:0] data
);
    parameter BIT_PERIOD = 1250;
    reg [11:0] clk_count = 0;
    reg [3:0] bit_idx = 0;
    reg [1:0] rx_state = 0;

    always @(posedge clk) begin
        if (ack) ready <= 0;
        case (rx_state)
            0: begin // IDLE: Wait for start bit (low)
                if (rx_pin == 0) begin
                    clk_count <= 0;
                    rx_state <= 1;
                end
            end
            1: begin // START BIT: Wait for middle of bit
                if (clk_count < BIT_PERIOD / 2) clk_count <= clk_count + 1;
                else begin
                    clk_count <= 0;
                    bit_idx <= 0;
                    rx_state <= 2;
                end
            end
            2: begin // DATA: Sample 8 bits
                if (clk_count < BIT_PERIOD) clk_count <= clk_count + 1;
                else begin
                    clk_count <= 0;
                    data[bit_idx] <= rx_pin;
                    if (bit_idx < 7) bit_idx <= bit_idx + 1;
                    else rx_state <= 3;
                end
            end
            3: begin // STOP BIT
                if (clk_count < BIT_PERIOD) clk_count <= clk_count + 1;
                else begin
                    ready <= 1;
                    rx_state <= 0;
                end
            end
        endcase
    end
endmodule

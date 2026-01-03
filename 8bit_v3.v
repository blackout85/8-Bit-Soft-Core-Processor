// 8-Bit CPU with UART Terminal - Menu System & String Support
// Targeted for Lattice iCEstick

module top (
    input  clk,      // 12MHz
    input  uart_rx,
    output uart_tx,
    output [4:0] leds
);

    // --- CPU REGISTER & STATE ---
    reg [7:0] pc = 0;
    reg [7:0] reg_a = 0;
    reg [4:0] led_reg = 0;
    reg [2:0] state = 0;
    reg [7:0] ptr = 0; // Pointer for string printing
    
    localparam ST_PRINT_MSG = 3'd0;
    localparam ST_FETCH     = 3'd1;
    localparam ST_DECODE    = 3'd2;
    localparam ST_EXECUTE   = 3'd3;
    localparam ST_WAIT      = 3'd4;

    // --- UART INTERFACE ---
    reg [7:0] tx_data;
    reg tx_start = 0;
    wire tx_busy, rx_ready;
    wire [7:0] rx_data;
    reg rx_ack = 0;

    // --- STRING ROM (The "Hello" message) ---
    reg [7:0] string_rom [0:63];
    initial begin
        string_rom[0]  = "H"; string_rom[1]  = "e"; string_rom[2]  = "l"; string_rom[3]  = "l";
        string_rom[4]  = "o"; string_rom[5]  = ","; string_rom[6]  = " "; string_rom[7]  = "h";
        string_rom[8]  = "o"; string_rom[9]  = "w"; string_rom[10] = " "; string_rom[11] = "c";
        string_rom[12] = "a"; string_rom[13] = "n"; string_rom[14] = " "; string_rom[15] = "I";
        string_rom[16] = " "; string_rom[17] = "h"; string_rom[18] = "e"; string_rom[19] = "l";
        string_rom[20] = "p"; string_rom[21] = "?"; string_rom[22] = "\r";string_rom[23] = "\n";
        string_rom[24] = "1"; string_rom[25] = ":"; string_rom[26] = "F"; string_rom[27] = "i";
        string_rom[28] = "l"; string_rom[29] = "e"; string_rom[30] = " "; string_rom[31] = "2";
        string_rom[32] = ":"; string_rom[33] = "C"; string_rom[34] = "a"; string_rom[35] = "l";
        string_rom[36] = "c"; string_rom[37] = ">"; string_rom[38] = " "; string_rom[39] = 0; // Null terminator
    end

    // --- INSTRUCTION ROM ---
    reg [7:0] rom [0:31];
    initial begin
        // Program Logic:
        rom[0] = 8'h40; // 0: IN A (Wait for menu selection)
        rom[1] = 8'h50; // 1: OUT A (Echo input)
        rom[2] = 8'h81; // 2: CPI '1' (Compare A with ASCII '1')
        rom[3] = 8'h76; // 3: JZ 6 (If '1', jump to File Mode)
        rom[4] = 8'h82; // 4: CPI '2' (Compare A with ASCII '2')
        rom[5] = 8'h7A; // 5: JZ 10 (If '2', jump to Calc Mode)
        
        // File Mode (Echo Loop)
        rom[6] = 8'h40; // IN A
        rom[7] = 8'h50; // OUT A
        rom[8] = 8'hF1; // Set LED pattern 1
        rom[9] = 8'h66; // JMP 6
        
        // Calc Mode (Add 1 Loop)
        rom[10] = 8'h40; // IN A
        rom[11] = 8'h21; // ADD 1
        rom[12] = 8'h50; // OUT A
        rom[13] = 8'hF2; // Set LED pattern 2
        rom[14] = 8'h6A; // JMP 10
    end

    // --- CPU CORE ---
    always @(posedge clk) begin
        case (state)
            ST_PRINT_MSG: begin
                if (string_rom[ptr] != 0) begin
                    if (!tx_busy && !tx_start) begin
                        tx_data <= string_rom[ptr];
                        tx_start <= 1;
                    end else if (tx_start) begin
                        tx_start <= 0;
                        ptr <= ptr + 1;
                    end
                end else begin
                    state <= ST_FETCH;
                end
            end

            ST_FETCH: begin
                rx_ack <= 0;
                tx_start <= 0;
                state <= ST_DECODE;
            end

            ST_DECODE: state <= ST_EXECUTE;

            ST_EXECUTE: begin
                case (rom[pc][7:4])
                    4'h2: begin reg_a <= reg_a + rom[pc][3:0]; pc <= pc + 1; state <= ST_FETCH; end // ADD
                    4'h4: if (rx_ready) begin reg_a <= rx_data; rx_ack <= 1; pc <= pc + 1; state <= ST_FETCH; end else state <= ST_WAIT;
                    4'h5: begin tx_data <= reg_a; tx_start <= 1; state <= ST_WAIT; end
                    4'h6: begin pc <= {4'b0, rom[pc][3:0]}; state <= ST_FETCH; end
                    4'h7: begin if (reg_a == 8'h00) pc <= {4'b0, rom[pc][3:0]}; else pc <= pc + 1; state <= ST_FETCH; end // JZ (Jump if Zero)
                    4'h8: begin reg_a <= (reg_a == (rom[pc][3:0] + 8'h30)) ? 8'h00 : 8'h01; pc <= pc + 1; state <= ST_FETCH; end // CPI
                    4'hF: begin led_reg <= rom[pc][4:0]; pc <= pc + 1; state <= ST_FETCH; end
                    default: begin pc <= pc + 1; state <= ST_FETCH; end
                endcase
            end

            ST_WAIT: begin
                tx_start <= 0;
                if (rom[pc][7:4] == 4'h4 && rx_ready) state <= ST_EXECUTE;
                else if (rom[pc][7:4] == 4'h5 && !tx_busy) begin pc <= pc + 1; state <= ST_FETCH; end
            end
        endcase
    end

    assign leds = led_reg;

    // --- UART MODULES ---
    uart_tx_module tx_unit (.clk(clk), .data(tx_data), .start(tx_start), .busy(tx_busy), .tx_pin(uart_tx));
    uart_rx_module rx_unit (.clk(clk), .rx_pin(uart_rx), .ready(rx_ready), .ack(rx_ack), .data(rx_data));

endmodule

module uart_tx_module (
    input clk, input [7:0] data, input start, output reg busy = 0, output reg tx_pin = 1
);
    parameter BIT_PERIOD = 1250;
    reg [11:0] clk_count = 0;
    reg [3:0] bit_idx = 0;
    reg [9:0] shift_reg;
    always @(posedge clk) begin
        if (!busy) begin
            if (start) begin shift_reg <= {1'b1, data, 1'b0}; busy <= 1; bit_idx <= 0; clk_count <= 0; end
        end else begin
            if (clk_count < BIT_PERIOD) clk_count <= clk_count + 1;
            else begin
                clk_count <= 0;
                tx_pin <= shift_reg[bit_idx];
                if (bit_idx < 9) bit_idx <= bit_idx + 1; else busy <= 0;
            end
        end
    end
endmodule

module uart_rx_module (
    input clk, input rx_pin, input ack, output reg ready = 0, output reg [7:0] data
);
    parameter BIT_PERIOD = 1250;
    reg [11:0] clk_count = 0;
    reg [3:0] bit_idx = 0;
    reg [1:0] rx_state = 0;
    always @(posedge clk) begin
        if (ack) ready <= 0;
        case (rx_state)
            0: if (rx_pin == 0) begin clk_count <= 0; rx_state <= 1; end
            1: if (clk_count < BIT_PERIOD / 2) clk_count <= clk_count + 1; else begin clk_count <= 0; bit_idx <= 0; rx_state <= 2; end
            2: if (clk_count < BIT_PERIOD) clk_count <= clk_count + 1; else begin
                clk_count <= 0; data[bit_idx] <= rx_pin;
                if (bit_idx < 7) bit_idx <= bit_idx + 1; else rx_state <= 3; end
            3: if (clk_count < BIT_PERIOD) clk_count <= clk_count + 1; else begin ready <= 1; rx_state <= 0; end
        endcase
    end
endmodule

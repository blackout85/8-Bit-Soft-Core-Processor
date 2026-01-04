// 8-Bit CPU with UART Terminal - RAM File Storage System
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
    reg [2:0] state = 0;
    reg [7:0] ptr = 0; 
    reg [23:0] boot_delay = 0;
    
    // File System Registers
    reg [7:0] file_ptr = 0;    // Current position in file
    reg [7:0] file_size = 0;   // Total length of saved data
    reg [7:0] read_ptr = 0;    // Pointer for reading back
    
    localparam ST_PRINT_MSG = 3'd0;
    localparam ST_FETCH     = 3'd1;
    localparam ST_DECODE    = 3'd2;
    localparam ST_EXECUTE   = 3'd3;
    localparam ST_WAIT      = 3'd4;
    localparam ST_READ_FILE = 3'd5;

    // --- UART INTERFACE ---
    reg [7:0] tx_data;
    reg tx_start = 0;
    wire tx_busy, rx_ready;
    wire [7:0] rx_data;
    reg rx_ack = 0;

    // --- RAM STORAGE (Optimized Block RAM Inference) ---
    // Added ram_style attribute to force EBR usage and fix "no BELs remaining"
    (* ram_style = "block" *) reg [7:0] file_ram [0:255];
    reg [7:0] ram_read_data;
    reg ram_we = 0;
    
    // Logic to select address outside the block for cleaner inference
    wire [7:0] ram_addr = (state == ST_READ_FILE) ? read_ptr : file_ptr;

    always @(posedge clk) begin
        if (ram_we)
            file_ram[file_ptr] <= reg_a;
        ram_read_data <= file_ram[ram_addr];
    end

    // --- STRING ROM ---
    reg [7:0] string_rom [0:63];
    initial begin
        string_rom[0]  = "\r"; string_rom[1]  = "\n";
        string_rom[2]  = "S"; string_rom[3]  = "y"; string_rom[4]  = "s"; string_rom[5]  = "t";
        string_rom[6]  = "e"; string_rom[7]  = "m"; string_rom[8]  = " "; string_rom[9]  = "R";
        string_rom[10] = "e"; string_rom[11] = "a"; string_rom[12] = "d"; string_rom[13] = "y";
        string_rom[14] = "."; string_rom[15] = "\r"; string_rom[16] = "\n";
        string_rom[17] = "P"; string_rom[18] = "r"; string_rom[19] = "e"; string_rom[20] = "s";
        string_rom[21] = "s"; string_rom[22] = " "; string_rom[23] = "1"; string_rom[24] = " ";
        string_rom[25] = "t"; string_rom[26] = "o"; string_rom[27] = " "; string_rom[28] = "I";
        string_rom[29] = "n"; string_rom[30] = "i"; string_rom[31] = "t"; string_rom[32] = " ";
        string_rom[33] = "F"; string_rom[34] = "i"; string_rom[35] = "l"; string_rom[36] = "e";
        string_rom[37] = ">"; string_rom[38] = " "; string_rom[39] = 0;
    end

    // --- INSTRUCTION ROM ---
    reg [7:0] rom [0:31];
    initial begin
        // Main Loop
        rom[0] = 8'h40; // IN A (Wait for '1')
        rom[1] = 8'h81; // CPI '1'
        rom[2] = 8'h74; // JZ 4 (Go to File Mode)
        rom[3] = 8'h60; // JMP 0
        
        // File Mode
        rom[4] = 8'h40; // IN A
        rom[5] = 8'h8D; // CPI Carriage Return (Enter)
        rom[6] = 8'h7C; // JZ 12 (Go to Reset/Exit)
        rom[7] = 8'h80; // CPI Space
        rom[8] = 8'h7B; // JZ 11 (Go to Read Trigger)
        rom[9] = 8'hA0; // STORE_RAM A
        rom[10] = 8'h50; // OUT A (Echo)
        rom[11] = 8'h64; // JMP 4
        
        // Command Handlers
        rom[12] = 8'hB0; // Trigger READ_FILE state
        rom[13] = 8'h64; // JMP 4 (Return to typing after read)
        rom[14] = 8'h60; // JMP 0 (Exit to Menu)
    end

    // --- CPU CORE ---
    always @(posedge clk) begin
        ram_we <= 0;
        
        case (state)
            ST_PRINT_MSG: begin
                if (boot_delay < 24'd2_000_000) boot_delay <= boot_delay + 1;
                else if (string_rom[ptr] != 0) begin
                    if (!tx_busy && !tx_start) begin 
                        tx_data <= string_rom[ptr]; 
                        tx_start <= 1; 
                    end
                    else if (tx_start) begin 
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
                    4'h4: if (rx_ready) begin reg_a <= rx_data; rx_ack <= 1; pc <= pc + 1; state <= ST_FETCH; end else state <= ST_WAIT;
                    4'h5: begin tx_data <= reg_a; tx_start <= 1; state <= ST_WAIT; end
                    4'h6: begin pc <= {4'b0, rom[pc][3:0]}; state <= ST_FETCH; end
                    4'h7: begin if (reg_a == 8'h00) pc <= {4'b0, rom[pc][3:0]}; else pc <= pc + 1; state <= ST_FETCH; end
                    4'h8: begin 
                            if (rom[pc][3:0] == 4'hD) reg_a <= (reg_a == 8'h0D) ? 8'h00 : 8'h01;
                            else if (rom[pc][3:0] == 4'h0) reg_a <= (reg_a == 8'h20) ? 8'h00 : 8'h01;
                            else reg_a <= (reg_a == (rom[pc][3:0] + 8'h30)) ? 8'h00 : 8'h01; 
                            pc <= pc + 1; state <= ST_FETCH; 
                          end
                    4'hA: begin // STORE_RAM
                            ram_we <= 1;
                            if (file_ptr < 255) begin
                                file_ptr <= file_ptr + 1;
                                file_size <= file_ptr + 1;
                            end
                            pc <= pc + 1; state <= ST_FETCH; end
                    4'hB: begin // READ_FILE Trigger
                            read_ptr <= 0; state <= ST_READ_FILE; pc <= pc + 1; end
                    default: begin pc <= pc + 1; state <= ST_FETCH; end
                endcase
            end

            ST_READ_FILE: begin
                if (read_ptr < file_size) begin
                    if (!tx_busy && !tx_start) begin
                        tx_data <= ram_read_data;
                        tx_start <= 1;
                    end else if (tx_start) begin
                        tx_start <= 0;
                        read_ptr <= read_ptr + 1;
                    end
                end else state <= ST_FETCH;
            end

            ST_WAIT: begin
                tx_start <= 0;
                if (rom[pc][7:4] == 4'h4 && rx_ready) state <= ST_EXECUTE;
                else if (rom[pc][7:4] == 4'h5 && !tx_busy) begin pc <= pc + 1; state <= ST_FETCH; end
                else if (rom[pc][7:4] != 4'h4 && rom[pc][7:4] != 4'h5) state <= ST_FETCH; 
            end
            default: state <= ST_PRINT_MSG;
        endcase
    end

    assign leds = file_ptr[4:0];

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
            tx_pin <= 1;
            if (start) begin shift_reg <= {1'b1, data, 1'b0}; busy <= 1; bit_idx <= 0; clk_count <= 0; end
        end else begin
            if (clk_count < BIT_PERIOD) clk_count <= clk_count + 1;
            else begin
                clk_count <= 0; tx_pin <= shift_reg[bit_idx];
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

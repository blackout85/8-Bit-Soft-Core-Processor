// This Verilog wrapper allows Apio to "see" your VHDL entity 
// as the top-level module of the project.

module ice_cpu_uart_top (
    input clk,
    input uart_rx,
    output uart_tx,
    output [4:0] leds
);

    // This line tells the tools to look for the VHDL entity 
    // named "ice_cpu_uart_top" and connect its pins to these physical pins.
    ice_cpu_uart_top vhdl_cpu (
        .clk(clk),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .leds(leds)
    );

endmodule

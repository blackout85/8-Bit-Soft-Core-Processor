library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ice_cpu_uart_top is
    Port (
        clk      : in  std_logic; -- 12MHz
        uart_tx  : out std_logic; -- Pin 8 on iCEstick (TX)
        uart_rx  : in  std_logic; -- Pin 9 on iCEstick (RX)
        leds     : out std_logic_vector(4 downto 0)
    );
end ice_cpu_uart_top;

architecture Behavioral of ice_cpu_uart_top is
    -- CPU Signals
    type state_type is (ST_FETCH, ST_DECODE, ST_EXECUTE, ST_UART_TX_WAIT, ST_UART_RX_WAIT);
    signal state : state_type := ST_FETCH;
    signal pc : unsigned(7 downto 0) := (others => '0');
    signal reg_a : unsigned(7 downto 0) := (others => '0');
    signal instr : std_logic_vector(7 downto 0);
    signal zero_flag : std_logic := '0';

    -- UART Constants (9600 Baud @ 12MHz)
    constant BIT_PERIOD : integer := 1250;
    
    -- UART TX Signals
    signal uart_tx_data  : std_logic_vector(7 downto 0);
    signal uart_tx_start : std_logic := '0';
    signal uart_tx_busy  : std_logic := '0';
    
    -- UART RX Signals
    signal uart_rx_data  : std_logic_vector(7 downto 0);
    signal uart_rx_ready : std_logic := '0';
    signal uart_rx_ack   : std_logic := '0'; -- CPU signal to clear RX ready

    -- ROM: A program that waits for a keypress and echoes it back + shows on LEDs
    type mem_type is array (0 to 15) of std_logic_vector(7 downto 0);
    constant ROM : mem_type := (
        0 => "0100" & "0000", -- IN A (Wait for UART RX)
        1 => "0101" & "0000", -- OUT A (Send A to UART TX)
        2 => "1111" & "0000", -- STORE A to LEDs (Magic address 0)
        3 => "0110" & "0000", -- JUMP to 0
        others => "00000000"
    );

begin

    -- CPU FSM
    process(clk)
    begin
        if rising_edge(clk) then
            case state is
                when ST_FETCH =>
                    instr <= ROM(to_integer(pc));
                    uart_rx_ack <= '0';
                    state <= ST_DECODE;
                
                when ST_DECODE =>
                    state <= ST_EXECUTE;
                
                when ST_EXECUTE =>
                    case instr(7 downto 4) is
                        when "0001" => -- LOAD Imm
                            reg_a <= unsigned("0000" & instr(3 downto 0));
                            pc <= pc + 1;
                            state <= ST_FETCH;

                        when "0100" => -- IN A (Read UART)
                            if uart_rx_ready = '1' then
                                reg_a <= unsigned(uart_rx_data);
                                uart_rx_ack <= '1'; -- Tell RX logic we got it
                                pc <= pc + 1;
                                state <= ST_FETCH;
                            else
                                state <= ST_UART_RX_WAIT; -- Loop here until ready
                            end if;
                        
                        when "0101" => -- OUT A (Write UART)
                            uart_tx_data <= std_logic_vector(reg_a);
                            uart_tx_start <= '1';
                            state <= ST_UART_TX_WAIT;
                        
                        when "1111" => -- Custom: Write to LEDs
                            leds <= std_logic_vector(reg_a(4 downto 0));
                            pc <= pc + 1;
                            state <= ST_FETCH;

                        when "0110" => -- JUMP
                            pc <= unsigned("0000" & instr(3 downto 0));
                            state <= ST_FETCH;
                            
                        when others => 
                            pc <= pc + 1;
                            state <= ST_FETCH;
                    end case;

                when ST_UART_TX_WAIT =>
                    uart_tx_start <= '0';
                    if uart_tx_busy = '0' then
                        pc <= pc + 1;
                        state <= ST_FETCH;
                    end if;

                when ST_UART_RX_WAIT =>
                    if uart_rx_ready = '1' then
                        state <= ST_EXECUTE; -- Go back and perform the LOAD
                    end if;
            end case;
        end if;
    end process;

    -- UART TRANSMITTER
    process(clk)
        variable count : integer range 0 to BIT_PERIOD := 0;
        variable bit_idx : integer range 0 to 10 := 0;
        variable shift_reg : std_logic_vector(9 downto 0);
    begin
        if rising_edge(clk) then
            if uart_tx_busy = '0' then
                if uart_tx_start = '1' then
                    shift_reg := '1' & uart_tx_data & '0';
                    uart_tx_busy <= '1';
                    bit_idx := 0;
                    count := 0;
                end if;
                uart_tx <= '1';
            else
                if count < BIT_PERIOD then
                    count := count + 1;
                else
                    count := 0;
                    uart_tx <= shift_reg(bit_idx);
                    if bit_idx < 9 then
                        bit_idx := bit_idx + 1;
                    else
                        uart_tx_busy <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- UART RECEIVER
    process(clk)
        variable count : integer range 0 to BIT_PERIOD := 0;
        variable bit_idx : integer range 0 to 8 := 0;
        type rx_state_type is (RX_IDLE, RX_START, RX_DATA, RX_STOP);
        variable rx_state : rx_state_type := RX_IDLE;
    begin
        if rising_edge(clk) then
            if uart_rx_ack = '1' then
                uart_rx_ready <= '0';
            end if;

            case rx_state is
                when RX_IDLE =>
                    if uart_rx = '0' then -- Start bit detected
                        count := 0;
                        rx_state := RX_START;
                    end if;

                when RX_START =>
                    if count < BIT_PERIOD / 2 then -- Wait to sample in the middle of bit
                        count := count + 1;
                    else
                        count := 0;
                        rx_state := RX_DATA;
                        bit_idx := 0;
                    end if;

                when RX_DATA =>
                    if count < BIT_PERIOD then
                        count := count + 1;
                    else
                        count := 0;
                        uart_rx_data(bit_idx) <= uart_rx;
                        if bit_idx < 7 then
                            bit_idx := bit_idx + 1;
                        else
                            rx_state := RX_STOP;
                        end if;
                    end if;

                when RX_STOP =>
                    if count < BIT_PERIOD then
                        count := count + 1;
                    else
                        uart_rx_ready <= '1';
                        rx_state := RX_IDLE;
                    end if;
            end case;
        end if;
    end process;

end Behavioral;

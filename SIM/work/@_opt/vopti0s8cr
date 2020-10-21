library verilog;
use verilog.vl_types.all;
entity clk_ctrl is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        rtx_ok          : in     vl_logic;
        rx_ok           : in     vl_logic;
        tx_ok           : in     vl_logic;
        divclk          : in     vl_logic_vector(15 downto 0);
        rtx_done        : out    vl_logic;
        rx_done         : out    vl_logic;
        tx_done         : out    vl_logic;
        clk_div         : out    vl_logic
    );
end clk_ctrl;

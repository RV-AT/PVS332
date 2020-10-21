library verilog;
use verilog.vl_types.all;
entity SyncFIFO is
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        wr_en           : in     vl_logic;
        rd_en           : in     vl_logic;
        data_in         : in     vl_logic_vector(7 downto 0);
        data_out        : out    vl_logic_vector(7 downto 0);
        empty           : out    vl_logic;
        full            : out    vl_logic
    );
end SyncFIFO;

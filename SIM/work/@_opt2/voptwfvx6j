library verilog;
use verilog.vl_types.all;
entity plic_cell is
    port(
        int_src         : in     vl_logic;
        rst_n           : in     vl_logic;
        prio            : in     vl_logic_vector(4 downto 0);
        thres           : in     vl_logic_vector(4 downto 0);
        en              : in     vl_logic;
        ack             : in     vl_logic;
        ip              : out    vl_logic;
        iclaim          : out    vl_logic
    );
end plic_cell;

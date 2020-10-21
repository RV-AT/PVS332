library verilog;
use verilog.vl_types.all;
entity ahb_reset_ctrl is
    port(
        HCLK            : in     vl_logic;
        nPOReset        : in     vl_logic;
        HRESETn         : out    vl_logic
    );
end ahb_reset_ctrl;

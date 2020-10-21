library verilog;
use verilog.vl_types.all;
entity plic is
    port(
        addr            : in     vl_logic_vector(7 downto 0);
        wr_n            : in     vl_logic;
        oe              : in     vl_logic;
        dat_i           : in     vl_logic_vector(31 downto 0);
        dat_o           : out    vl_logic_vector(31 downto 0);
        ext_int_o       : out    vl_logic
    );
end plic;

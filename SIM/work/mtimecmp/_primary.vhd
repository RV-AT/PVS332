library verilog;
use verilog.vl_types.all;
entity mtimecmp is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        wrh_n           : in     vl_logic;
        wrl_n           : in     vl_logic;
        mtime           : in     vl_logic_vector(63 downto 0);
        mtimecmp_o      : in     vl_logic_vector(31 downto 0);
        mtimecmph_i     : out    vl_logic_vector(31 downto 0);
        mtimecmpl_i     : out    vl_logic_vector(31 downto 0);
        timer_int       : out    vl_logic
    );
end mtimecmp;

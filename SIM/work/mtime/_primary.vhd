library verilog;
use verilog.vl_types.all;
entity mtime is
    port(
        rst             : in     vl_logic;
        clk             : in     vl_logic;
        wrh_n           : in     vl_logic;
        wrl_n           : in     vl_logic;
        mtimer_o        : in     vl_logic_vector(31 downto 0);
        mtimerh_i       : out    vl_logic_vector(31 downto 0);
        mtimerl_i       : out    vl_logic_vector(31 downto 0)
    );
end mtime;

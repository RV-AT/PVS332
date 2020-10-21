library verilog;
use verilog.vl_types.all;
entity BLKRAM8 is
    port(
        addra           : in     vl_logic_vector(14 downto 0);
        clka            : in     vl_logic;
        dina            : in     vl_logic_vector(7 downto 0);
        ena             : in     vl_logic;
        wea             : in     vl_logic;
        clkb            : in     vl_logic;
        enb             : in     vl_logic;
        addrb           : in     vl_logic_vector(14 downto 0);
        doutb           : out    vl_logic_vector(7 downto 0)
    );
end BLKRAM8;

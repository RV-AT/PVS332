library verilog;
use verilog.vl_types.all;
entity ahb is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        haddr           : out    vl_logic_vector(33 downto 0);
        hwrite          : out    vl_logic;
        hsize           : out    vl_logic_vector(2 downto 0);
        hburst          : out    vl_logic_vector(2 downto 0);
        hprot           : out    vl_logic_vector(3 downto 0);
        htrans          : out    vl_logic_vector(1 downto 0);
        hmastlock       : out    vl_logic;
        hwdata          : out    vl_logic_vector(31 downto 0);
        hready          : in     vl_logic;
        hresp           : in     vl_logic;
        hreset_n        : in     vl_logic;
        hrdata          : in     vl_logic_vector(31 downto 0);
        r32             : in     vl_logic;
        r16             : in     vl_logic;
        r8              : in     vl_logic;
        w32             : in     vl_logic;
        w16             : in     vl_logic;
        w8              : in     vl_logic;
        ahb_acc_fault   : out    vl_logic;
        data_out        : out    vl_logic_vector(31 downto 0);
        rdy_ahb         : out    vl_logic;
        addr_in         : in     vl_logic_vector(33 downto 0);
        data_in         : in     vl_logic_vector(31 downto 0)
    );
end ahb;

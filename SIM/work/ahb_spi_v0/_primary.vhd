library verilog;
use verilog.vl_types.all;
entity ahb_spi_v0 is
    generic(
        nsq             : vl_logic_vector(0 to 1) := (Hi1, Hi0);
        idle            : vl_logic_vector(0 to 1) := (Hi0, Hi0);
        t8              : vl_logic_vector(0 to 2) := (Hi0, Hi0, Hi0);
        t16             : vl_logic_vector(0 to 2) := (Hi0, Hi0, Hi1);
        t32             : vl_logic_vector(0 to 2) := (Hi0, Hi1, Hi0)
    );
    port(
        addr_cfg        : in     vl_logic_vector(31 downto 0);
        hsel            : in     vl_logic;
        haddr           : in     vl_logic_vector(31 downto 0);
        hwrite          : in     vl_logic;
        hsize           : in     vl_logic_vector(2 downto 0);
        hburst          : in     vl_logic_vector(2 downto 0);
        hprot           : in     vl_logic_vector(3 downto 0);
        htrans          : in     vl_logic_vector(1 downto 0);
        hmastlock       : in     vl_logic;
        hwdata          : in     vl_logic_vector(31 downto 0);
        hresetn         : in     vl_logic;
        hclk            : in     vl_logic;
        hreadyout       : out    vl_logic;
        hresp           : out    vl_logic;
        hrdata          : out    vl_logic_vector(31 downto 0);
        ssel            : out    vl_logic_vector(2 downto 0);
        cs              : out    vl_logic;
        cpol            : out    vl_logic;
        cpha            : out    vl_logic;
        sck             : out    vl_logic;
        mosi            : out    vl_logic;
        miso            : in     vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of nsq : constant is 1;
    attribute mti_svvh_generic_type of idle : constant is 1;
    attribute mti_svvh_generic_type of t8 : constant is 1;
    attribute mti_svvh_generic_type of t16 : constant is 1;
    attribute mti_svvh_generic_type of t32 : constant is 1;
end ahb_spi_v0;

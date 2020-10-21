library verilog;
use verilog.vl_types.all;
entity prv332sv0 is
    generic(
        w8              : vl_logic_vector(0 to 2) := (Hi0, Hi0, Hi1);
        w16             : vl_logic_vector(0 to 2) := (Hi0, Hi1, Hi0);
        w32             : vl_logic_vector(0 to 2) := (Hi0, Hi1, Hi1);
        r8              : vl_logic_vector(0 to 2) := (Hi1, Hi0, Hi1);
        r16             : vl_logic_vector(0 to 2) := (Hi1, Hi1, Hi0);
        r32             : vl_logic_vector(0 to 2) := (Hi1, Hi1, Hi1)
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        timer_int       : in     vl_logic;
        soft_int        : in     vl_logic;
        ext_int         : in     vl_logic;
        haddr           : out    vl_logic_vector(33 downto 0);
        hwrite          : out    vl_logic;
        hsize           : out    vl_logic_vector(1 downto 0);
        hburst          : out    vl_logic_vector(2 downto 0);
        hprot           : out    vl_logic_vector(3 downto 0);
        htrans          : out    vl_logic_vector(1 downto 0);
        hmastlock       : out    vl_logic;
        hwdata          : out    vl_logic_vector(31 downto 0);
        hready          : in     vl_logic;
        hresp           : in     vl_logic;
        hreset_n        : in     vl_logic;
        hrdata          : in     vl_logic_vector(31 downto 0)
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of w8 : constant is 1;
    attribute mti_svvh_generic_type of w16 : constant is 1;
    attribute mti_svvh_generic_type of w32 : constant is 1;
    attribute mti_svvh_generic_type of r8 : constant is 1;
    attribute mti_svvh_generic_type of r16 : constant is 1;
    attribute mti_svvh_generic_type of r32 : constant is 1;
end prv332sv0;

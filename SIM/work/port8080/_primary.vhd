library verilog;
use verilog.vl_types.all;
entity port8080 is
    generic(
        Idle            : vl_logic_vector(0 to 3) := (Hi0, Hi0, Hi0, Hi0);
        SC0             : vl_logic_vector(0 to 3) := (Hi0, Hi0, Hi0, Hi1);
        SC1             : vl_logic_vector(0 to 3) := (Hi0, Hi0, Hi1, Hi0);
        SC2             : vl_logic_vector(0 to 3) := (Hi0, Hi0, Hi1, Hi1);
        RB0             : vl_logic_vector(0 to 3) := (Hi0, Hi1, Hi0, Hi0);
        RB1             : vl_logic_vector(0 to 3) := (Hi0, Hi1, Hi0, Hi1);
        RB2             : vl_logic_vector(0 to 3) := (Hi0, Hi1, Hi1, Hi0);
        WB0             : vl_logic_vector(0 to 3) := (Hi0, Hi1, Hi1, Hi1);
        WB1             : vl_logic_vector(0 to 3) := (Hi1, Hi0, Hi0, Hi0);
        WB2             : vl_logic_vector(0 to 3) := (Hi1, Hi0, Hi1, Hi0)
    );
    port(
        data_i          : in     vl_logic_vector(7 downto 0);
        data_o          : out    vl_logic_vector(7 downto 0);
        wr              : out    vl_logic;
        rd              : out    vl_logic;
        rs              : out    vl_logic;
        datain          : in     vl_logic_vector(7 downto 0);
        dataout         : out    vl_logic_vector(7 downto 0);
        cmd             : in     vl_logic_vector(7 downto 0);
        func            : in     vl_logic_vector(2 downto 0);
        start           : in     vl_logic;
        busy            : out    vl_logic;
        done            : out    vl_logic;
        clk             : in     vl_logic;
        rst             : in     vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of Idle : constant is 1;
    attribute mti_svvh_generic_type of SC0 : constant is 1;
    attribute mti_svvh_generic_type of SC1 : constant is 1;
    attribute mti_svvh_generic_type of SC2 : constant is 1;
    attribute mti_svvh_generic_type of RB0 : constant is 1;
    attribute mti_svvh_generic_type of RB1 : constant is 1;
    attribute mti_svvh_generic_type of RB2 : constant is 1;
    attribute mti_svvh_generic_type of WB0 : constant is 1;
    attribute mti_svvh_generic_type of WB1 : constant is 1;
    attribute mti_svvh_generic_type of WB2 : constant is 1;
end port8080;

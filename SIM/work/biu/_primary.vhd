library verilog;
use verilog.vl_types.all;
entity biu is
    generic(
        stb             : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0);
        rdy             : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi1);
        err             : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi0, Hi0, Hi0, Hi1, Hi0);
        ifnp            : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi0, Hi1, Hi0, Hi0, Hi0);
        ifwp0           : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi1, Hi0, Hi0, Hi0, Hi0);
        ifwp1           : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi1, Hi0, Hi0, Hi0, Hi1);
        ifwp2           : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi1, Hi0, Hi0, Hi1, Hi0);
        ifwp3           : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi1, Hi0, Hi0, Hi1, Hi1);
        ifwp4           : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi1, Hi0, Hi1, Hi0, Hi0);
        r32np           : vl_logic_vector(0 to 6) := (Hi0, Hi0, Hi1, Hi1, Hi0, Hi0, Hi0);
        r32wp0          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi0, Hi0, Hi0, Hi0, Hi0);
        r32wp1          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi0, Hi0, Hi0, Hi0, Hi1);
        r32wp2          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi0, Hi0, Hi0, Hi1, Hi0);
        r32wp3          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi0, Hi0, Hi0, Hi1, Hi1);
        r32wp4          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi0, Hi0, Hi1, Hi0, Hi0);
        r16np           : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi0, Hi1, Hi0, Hi0, Hi0);
        r16wp0          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi0, Hi0, Hi0, Hi0);
        r16wp1          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi0, Hi0, Hi0, Hi1);
        r16wp2          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi0, Hi0, Hi1, Hi0);
        r16wp3          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi0, Hi0, Hi1, Hi1);
        r16wp4          : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi0, Hi1, Hi0, Hi0);
        r8np            : vl_logic_vector(0 to 6) := (Hi0, Hi1, Hi1, Hi1, Hi0, Hi0, Hi0);
        r8wp0           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0);
        r8wp1           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi0, Hi0, Hi0, Hi0, Hi1);
        r8wp2           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi0, Hi0, Hi0, Hi1, Hi0);
        r8wp3           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi0, Hi0, Hi0, Hi1, Hi1);
        r8wp4           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi0, Hi0, Hi1, Hi0, Hi0);
        w32np           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi0, Hi1, Hi0, Hi0, Hi0);
        w32wp0          : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi1, Hi0, Hi0, Hi0, Hi0);
        w32wp1          : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi1, Hi0, Hi0, Hi0, Hi1);
        w32wp2          : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi1, Hi0, Hi0, Hi1, Hi0);
        w32wp3          : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi1, Hi0, Hi0, Hi1, Hi1);
        w32wp4          : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi1, Hi0, Hi1, Hi0, Hi0);
        w16np           : vl_logic_vector(0 to 6) := (Hi1, Hi0, Hi1, Hi1, Hi0, Hi0, Hi0);
        w16wp0          : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi0, Hi0, Hi0, Hi0, Hi0);
        w16wp1          : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi0, Hi0, Hi0, Hi0, Hi1);
        w16wp2          : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi0, Hi0, Hi0, Hi1, Hi0);
        w16wp3          : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi0, Hi0, Hi0, Hi1, Hi1);
        w16wp4          : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi0, Hi0, Hi1, Hi0, Hi0);
        w8np            : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi0, Hi1, Hi0, Hi0, Hi0);
        w8wp0           : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi1, Hi0, Hi0, Hi0, Hi0);
        w8wp1           : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi1, Hi0, Hi0, Hi0, Hi1);
        w8wp2           : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi1, Hi0, Hi0, Hi1, Hi0);
        w8wp3           : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi1, Hi0, Hi0, Hi1, Hi1);
        w8wp4           : vl_logic_vector(0 to 6) := (Hi1, Hi1, Hi1, Hi0, Hi1, Hi0, Hi0);
        opw8            : vl_logic_vector(0 to 2) := (Hi0, Hi0, Hi1);
        opw16           : vl_logic_vector(0 to 2) := (Hi0, Hi1, Hi0);
        opw32           : vl_logic_vector(0 to 2) := (Hi0, Hi1, Hi1);
        opr8            : vl_logic_vector(0 to 2) := (Hi1, Hi0, Hi1);
        opr16           : vl_logic_vector(0 to 2) := (Hi1, Hi1, Hi0);
        opr32           : vl_logic_vector(0 to 2) := (Hi1, Hi1, Hi1)
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
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
        hrdata          : in     vl_logic_vector(31 downto 0);
        opc             : in     vl_logic_vector(2 downto 0);
        statu_cpu       : in     vl_logic_vector(3 downto 0);
        addr_out        : out    vl_logic_vector(33 downto 0);
        pmp_chk_fault   : in     vl_logic;
        satp            : in     vl_logic_vector(31 downto 0);
        addr            : in     vl_logic_vector(31 downto 0);
        pc              : in     vl_logic_vector(31 downto 0);
        biu_data_in     : in     vl_logic_vector(31 downto 0);
        biu_data_out    : out    vl_logic_vector(31 downto 0);
        msu             : in     vl_logic_vector(1 downto 0);
        ins             : out    vl_logic_vector(31 downto 0);
        rdy_biu         : out    vl_logic;
        mxr             : in     vl_logic;
        sum             : in     vl_logic;
        ins_addr_mis    : out    vl_logic;
        ins_acc_fault   : out    vl_logic;
        load_addr_mis   : out    vl_logic;
        load_acc_fault  : out    vl_logic;
        st_addr_mis     : out    vl_logic;
        st_acc_fault    : out    vl_logic;
        ins_page_fault  : out    vl_logic;
        ld_page_fault   : out    vl_logic;
        st_page_fault   : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of stb : constant is 1;
    attribute mti_svvh_generic_type of rdy : constant is 1;
    attribute mti_svvh_generic_type of err : constant is 1;
    attribute mti_svvh_generic_type of ifnp : constant is 1;
    attribute mti_svvh_generic_type of ifwp0 : constant is 1;
    attribute mti_svvh_generic_type of ifwp1 : constant is 1;
    attribute mti_svvh_generic_type of ifwp2 : constant is 1;
    attribute mti_svvh_generic_type of ifwp3 : constant is 1;
    attribute mti_svvh_generic_type of ifwp4 : constant is 1;
    attribute mti_svvh_generic_type of r32np : constant is 1;
    attribute mti_svvh_generic_type of r32wp0 : constant is 1;
    attribute mti_svvh_generic_type of r32wp1 : constant is 1;
    attribute mti_svvh_generic_type of r32wp2 : constant is 1;
    attribute mti_svvh_generic_type of r32wp3 : constant is 1;
    attribute mti_svvh_generic_type of r32wp4 : constant is 1;
    attribute mti_svvh_generic_type of r16np : constant is 1;
    attribute mti_svvh_generic_type of r16wp0 : constant is 1;
    attribute mti_svvh_generic_type of r16wp1 : constant is 1;
    attribute mti_svvh_generic_type of r16wp2 : constant is 1;
    attribute mti_svvh_generic_type of r16wp3 : constant is 1;
    attribute mti_svvh_generic_type of r16wp4 : constant is 1;
    attribute mti_svvh_generic_type of r8np : constant is 1;
    attribute mti_svvh_generic_type of r8wp0 : constant is 1;
    attribute mti_svvh_generic_type of r8wp1 : constant is 1;
    attribute mti_svvh_generic_type of r8wp2 : constant is 1;
    attribute mti_svvh_generic_type of r8wp3 : constant is 1;
    attribute mti_svvh_generic_type of r8wp4 : constant is 1;
    attribute mti_svvh_generic_type of w32np : constant is 1;
    attribute mti_svvh_generic_type of w32wp0 : constant is 1;
    attribute mti_svvh_generic_type of w32wp1 : constant is 1;
    attribute mti_svvh_generic_type of w32wp2 : constant is 1;
    attribute mti_svvh_generic_type of w32wp3 : constant is 1;
    attribute mti_svvh_generic_type of w32wp4 : constant is 1;
    attribute mti_svvh_generic_type of w16np : constant is 1;
    attribute mti_svvh_generic_type of w16wp0 : constant is 1;
    attribute mti_svvh_generic_type of w16wp1 : constant is 1;
    attribute mti_svvh_generic_type of w16wp2 : constant is 1;
    attribute mti_svvh_generic_type of w16wp3 : constant is 1;
    attribute mti_svvh_generic_type of w16wp4 : constant is 1;
    attribute mti_svvh_generic_type of w8np : constant is 1;
    attribute mti_svvh_generic_type of w8wp0 : constant is 1;
    attribute mti_svvh_generic_type of w8wp1 : constant is 1;
    attribute mti_svvh_generic_type of w8wp2 : constant is 1;
    attribute mti_svvh_generic_type of w8wp3 : constant is 1;
    attribute mti_svvh_generic_type of w8wp4 : constant is 1;
    attribute mti_svvh_generic_type of opw8 : constant is 1;
    attribute mti_svvh_generic_type of opw16 : constant is 1;
    attribute mti_svvh_generic_type of opw32 : constant is 1;
    attribute mti_svvh_generic_type of opr8 : constant is 1;
    attribute mti_svvh_generic_type of opr16 : constant is 1;
    attribute mti_svvh_generic_type of opr32 : constant is 1;
end biu;

library verilog;
use verilog.vl_types.all;
entity AHB_GPIOC is
    port(
        HSEL            : in     vl_logic;
        HADDR           : in     vl_logic_vector(31 downto 0);
        HWRITE          : in     vl_logic;
        HTRANS          : in     vl_logic_vector(1 downto 0);
        HSIZE           : in     vl_logic_vector(2 downto 0);
        HBURST          : in     vl_logic_vector(2 downto 0);
        HWDATA          : in     vl_logic_vector(31 downto 0);
        HRESETn         : in     vl_logic;
        HCLK            : in     vl_logic;
        HMASTLOCK       : in     vl_logic;
        HREADY          : out    vl_logic;
        HRDATA          : out    vl_logic_vector(31 downto 0);
        GPIO1_DI        : in     vl_logic_vector(31 downto 0);
        GPIO1_DO        : out    vl_logic_vector(31 downto 0);
        GPIO1_AFC       : out    vl_logic_vector(31 downto 0);
        GPIO1_DIR       : out    vl_logic_vector(31 downto 0);
        GPIO1_PM        : out    vl_logic_vector(63 downto 0);
        GPIO1_PS        : out    vl_logic_vector(63 downto 0)
    );
end AHB_GPIOC;

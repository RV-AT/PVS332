library verilog;
use verilog.vl_types.all;
entity PERI_MUX is
    port(
        HRESETn         : in     vl_logic;
        HCLK            : in     vl_logic;
        PERISEL         : in     vl_logic_vector(31 downto 0);
        PERIDATAR       : out    vl_logic_vector(31 downto 0);
        PERIREADY       : out    vl_logic;
        PERIDATA0       : in     vl_logic_vector(31 downto 0);
        PERIDATA1       : in     vl_logic_vector(31 downto 0);
        PERIDATA2       : in     vl_logic_vector(31 downto 0);
        PERIDATA3       : in     vl_logic_vector(31 downto 0);
        PERIDATA4       : in     vl_logic_vector(31 downto 0);
        PERIDATA5       : in     vl_logic_vector(31 downto 0);
        PERIDATA6       : in     vl_logic_vector(31 downto 0);
        PERIDATA7       : in     vl_logic_vector(31 downto 0);
        PERIREADY0      : in     vl_logic;
        PERIREADY1      : in     vl_logic;
        PERIREADY2      : in     vl_logic;
        PERIREADY3      : in     vl_logic;
        PERIREADY4      : in     vl_logic;
        PERIREADY5      : in     vl_logic;
        PERIREADY6      : in     vl_logic;
        PERIREADY7      : in     vl_logic
    );
end PERI_MUX;

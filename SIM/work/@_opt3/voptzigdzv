library verilog;
use verilog.vl_types.all;
entity PRV332_SoC is
    port(
        CLK             : in     vl_logic;
        RST_N           : in     vl_logic;
        AFGPIO          : inout  vl_logic_vector(22 downto 0);
        SPI_CS          : out    vl_logic;
        SPI_MOSI        : out    vl_logic;
        SPI_MISO        : in     vl_logic;
        SPI_SCLK        : out    vl_logic;
        SRAM_Addr       : out    vl_logic_vector(21 downto 0);
        SRAM_Data       : inout  vl_logic_vector(31 downto 0);
        SRAM_BSEL       : out    vl_logic_vector(3 downto 0);
        SRAM_CS         : out    vl_logic;
        SRAM_WR         : out    vl_logic;
        SRAM_OE         : out    vl_logic;
        SRAM_RDY        : in     vl_logic
    );
end PRV332_SoC;

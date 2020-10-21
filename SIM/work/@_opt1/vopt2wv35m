library verilog;
use verilog.vl_types.all;
entity spi_module is
    port(
        I_clk           : in     vl_logic;
        I_rst_n         : in     vl_logic;
        I_rx_en         : in     vl_logic;
        I_tx_en         : in     vl_logic;
        rtx_en          : in     vl_logic;
        I_data_in       : in     vl_logic_vector(7 downto 0);
        O_data_out      : out    vl_logic_vector(7 downto 0);
        O_tx_done       : out    vl_logic;
        O_rx_done       : out    vl_logic;
        I_spi_miso      : in     vl_logic;
        O_spi_sck       : out    vl_logic;
        O_spi_cs        : out    vl_logic;
        O_spi_mosi      : out    vl_logic
    );
end spi_module;

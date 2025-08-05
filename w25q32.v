module W25Q32JV_simplified (
    input  wire        clk_i,     
    input  wire        rst_n,     
    input  wire        spi_clk,   
    input  wire        cs_n,      
    input  wire        mosi,       
    output reg         miso,    
    input  wire        wp_n,      
    input  wire        hold_n  
);

    // W25Q32JV Parameters
    localparam MEMORY_SIZE = 4*1024*1024;  // 4MB
    localparam PAGE_SIZE   = 256;          // 256 bytes per page
    localparam ADDR_WIDTH  = 24;           // 24-bit addressing
    
    // Instruction Opcodes
    localparam CMD_WRITE_ENABLE  = 8'h06;  // Write Enable
    localparam CMD_READ_DATA     = 8'h03;  // Read Data
    
    // Status Register Bits
    localparam BUSY_BIT = 0;  // S0
    localparam WEL_BIT  = 1;  // S1
    
    // State Machine States
    localparam STATE_IDLE         = 3'b000;
    localparam STATE_CMD          = 3'b001;
    localparam STATE_ADDR         = 3'b010;
    localparam STATE_READ_DATA    = 3'b011;
    localparam STATE_DUMMY        = 3'b100;
    
    // Internal Registers
    reg [2:0]  state, next_state;
    reg [7:0]  status_reg;        
    reg [7:0]  command_reg;     
    reg [23:0] address_reg;     
    reg [7:0]  data_buffer;       
    reg [4:0]  bit_counter;      
    reg [4:0]  byte_counter;       
    
    // Memory array 
    reg [7:0] memory [0:MEMORY_SIZE-1];
    
    // SPI clock edge detection
    reg spi_clk_prev;
    wire spi_clk_posedge = spi_clk & ~spi_clk_prev;
    wire spi_clk_negedge = ~spi_clk & spi_clk_prev;
    
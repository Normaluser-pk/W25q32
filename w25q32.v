module W25Q32 (
    input  wire        clk_i,     
    input  wire        rst_n,     
    input  wire        spi_clk,   
    input  wire        cs_n,      
    input  wire        mosi,       
    output reg         miso,    
    input  wire        wp_n,      
    input  wire        hold_n  
);

    // W25Q32 Parameters
    localparam MEMORY_SIZE = 1*1024*1024;  // 1MB
    localparam PAGE_SIZE   = 64;           // 64 bytes per page
    localparam SECTOR_SIZE = 1024;          // 1024 bytes per block
    localparam ADDR_WIDTH  = 24;           // 24-bit addressing
    
    // Instruction Opcodes
    localparam CMD_WRITE_ENABLE  = 8'h06;  // Write Enable
    localparam CMD_READ_DATA     = 8'h03;  // Read Data
    localparam CMD_WRITE_DISABLE = 8'h04;  // Write Disable
    localparam CMD_SECTOR_ERASE  = 8'h20;  // Sector Erase
    localparam CMD_CHIP_ERASE    = 8'hC7;  // Chip Erase
    localparam CMD_CHIP_ERASE2   = 8'h60;  // Chip Erase (does the same)
    
    // Status Register Bits
    localparam BUSY_BIT = 0;  // S0
    localparam WEL_BIT  = 1;  // S1
    
    // SPI Slave interface wires
    wire       spi_rx_dv;
    wire [7:0] spi_rx_byte;
    reg        spi_tx_dv;
    reg  [7:0] spi_tx_byte;
    wire       spi_miso;

    // Tie physical wires
    assign miso = spi_miso;

    // Instantiate SPI_Slave
    SPI_Slave #(
        .SPI_MODE(0)
    ) spi_slave_inst (
        .i_Rst_L   (rst_n),
        .i_Clk     (clk_i),
        .o_RX_DV   (spi_rx_dv),
        .o_RX_Byte (spi_rx_byte),
        .i_TX_DV   (spi_tx_dv),
        .i_TX_Byte (spi_tx_byte),
        .i_SPI_Clk (spi_clk),
        .o_SPI_MISO(spi_miso),
        .i_SPI_MOSI(mosi),
        .i_SPI_CS_n(cs_n)
    );

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
    
    // Initialize memory
    initial begin
        integer i;
        for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
            memory[i] = 8'hFF;  //Default erased state
        end
    end

    
    

        // Main protocol state machine
    always @(posedge clk_i or negedge rst_n) begin
        if (!rst_n) begin
            state       <= STATE_IDLE;
            command_reg <= 8'h00;
            address_reg <= 24'h000000;
            status_reg  <= 8'h00;
            byte_counter<= 5'h00;
            spi_tx_dv   <= 1'b0;
            spi_tx_byte <= 8'h00;
        end else begin
            // Default: Don't transmit unless specifically set below
            spi_tx_dv   <= 1'b0;
            case (state)
                STATE_IDLE: begin
                    if (spi_rx_dv && !cs_n) begin
                        command_reg <= spi_rx_byte;
                        byte_counter<= 0;
                        state <= STATE_CMD;
                    end
                end

                STATE_CMD: begin
                    if (cs_n) begin
                        state <= STATE_IDLE;
                    end else begin
                        // Only READ DATA supported in this simple example
                        if (command_reg == CMD_READ_DATA && spi_rx_dv) begin
                            address_reg[23:16] <= spi_rx_byte;
                            byte_counter <= 1;
                            state <= STATE_ADDR;
                        end
                    end
                end

                STATE_ADDR: begin
                    if (cs_n) begin
                        state <= STATE_IDLE;
                    end else if (spi_rx_dv) begin
                        if (byte_counter == 1) begin
                            address_reg[15:8] <= spi_rx_byte;
                            byte_counter <= 2;
                        end else if (byte_counter == 2) begin
                            address_reg[7:0] <= spi_rx_byte;
                            state <= STATE_READ_DATA;
                        end
                    end
                end

                STATE_READ_DATA: begin
                    if (cs_n) begin
                        state <= STATE_IDLE;
                    end else if (!cs_n) begin
                        // Output memory data, page through as bytes arrive from master
                        if (spi_rx_dv) begin
                            spi_tx_byte <= memory[address_reg];
                            spi_tx_dv <= 1'b1;
                            address_reg <= address_reg + 1;
                        end
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
    

     // Command Execution
    always @(posedge clk_i or negedge rst_n) begin
        if (!rst_n) begin
            status_reg[WEL_BIT] <= 1'b0;
            status_reg[BUSY_BIT] <= 1'b0;
        end else if (cs_n && state == STATE_CMD && spi_clk_prev) begin
            // Execute command when CS goes high after command phase
            case (command_reg)
                CMD_WRITE_ENABLE: begin
                    status_reg[WEL_BIT] <= 1'b1;
                    status_reg[BUSY_BIT] <= 1'b1;
                    $display("W25Q32JV: Write Enable executed - WEL set to 1");
                end
                
                CMD_WRITE_DISABLE: begin
                    status_reg[WEL_BIT] <= 1'b0;
                    status_reg[BUSY_BIT] <= 1'b0;
                    $display("W25Q32JV: Write Disable executed - WEL cleared to 0");
                end
                
                CMD_READ_DATA: begin
                    $display("W25Q32JV: Read Data command initiated");
                end

                CMD_CHIP_ERASE,
                CMD_CHIP_ERASE2: begin
                    if (status_reg[WEL_BIT]) begin
                        status_reg[BUSY_BIT] <= 1'b1;
                        for (integer m=0; m<MEMORY_SIZE; m=m+1)
                            memory[m] = 8'hFF;
                        status_reg[WEL_BIT]  <= 1'b0;
                        status_reg[BUSY_BIT] <= 1'b0;
                        $display("%t W25Q32JV : CHIP ERASE completed",$time);
                    end
                end

                CMD_SECTOR_ERASE: begin
                    if (status_reg[WEL_BIT]) begin
                        status_reg[BUSY_BIT] <= 1'b1;
                        integer saddr, idx;
                        saddr = (address_reg / SECTOR_SIZE) * SECTOR_SIZE;
                        for (idx = 0; idx < SECTOR_SIZE; idx = idx + 1)
                            memory[saddr + idx] = 8'hFF;
                        status_reg[WEL_BIT]  <= 1'b0;
                        status_reg[BUSY_BIT] <= 1'b0;
                        $display("%t W25Q32 : SE at 0x%06h (1 KiB) done",$time,saddr);
                    end
                end

                default: begin
                    $display("W25Q32: Unknown command: %02h", command_reg);
                end
            endcase
        end
    end
    
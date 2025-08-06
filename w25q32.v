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

    // Initialize memory
    initial begin
        integer i;
        for (i = 0; i < MEMORY_SIZE; i = i + 1) begin
            memory[i] = 8'hFF;  //Default erased state
        end
    end

    
    







    // Main State Machine
    always @(posedge clk_i or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            status_reg <= 8'h00;
            command_reg <= 8'h00;
            address_reg <= 24'h000000;
            data_buffer <= 8'h00;
            bit_counter <= 5'h00;
            byte_counter <= 5'h00;
            miso <= 1'b0;
        end else begin
            state <= next_state;
            
            // Handle SPI communication on SPI clock edges
            if (!cs_n && hold_n) begin  // Active when CS is low and HOLD is high
                if (spi_clk_posedge) begin
                    // Shift in data from MOSI on rising edge
                    case (state)
                        STATE_CMD: begin
                            data_buffer <= {data_buffer[6:0], mosi};
                            if (bit_counter == 7) begin
                                command_reg <= {data_buffer[6:0], mosi};
                            end
                        end
                        
                        STATE_ADDR: begin
                            data_buffer <= {data_buffer[6:0], mosi};
                            if (bit_counter == 7) begin
                                case (byte_counter)
                                    0: address_reg[23:16] <= {data_buffer[6:0], mosi};
                                    1: address_reg[15:8]  <= {data_buffer[6:0], mosi};
                                    2: address_reg[7:0]   <= {data_buffer[6:0], mosi};
                                endcase
                            end
                        end
                    endcase
                end
                
                if (spi_clk_negedge) begin
                    // Shift out data to MISO on falling edge
                    case (state)
                        STATE_READ_DATA: begin
                            if (bit_counter == 0) begin
                                // Load new byte from memory
                                data_buffer <= memory[address_reg];
                                miso <= memory[address_reg][7];
                            end else begin
                                // Shift out current byte
                                miso <= data_buffer[7-bit_counter];
                            end
                        end
                        
                        default: begin
                            miso <= 1'b0;
                        end
                    endcase
                end
            end
        end
    end
    
    // Next State Logic and Control
    always @(*) begin
        next_state = state;
        
        case (state)
            STATE_IDLE: begin
                if (!cs_n) begin
                    next_state = STATE_CMD;
                end
            end
            
            STATE_CMD: begin
                if (cs_n) begin
                    next_state = STATE_IDLE;
                end else if (spi_clk_posedge && bit_counter == 7) begin
                    // Command received, determine next state
                    case ({data_buffer[6:0], mosi})
                        CMD_READ_DATA: next_state = STATE_ADDR;
                        default: next_state = STATE_IDLE;  // Commands without address
                    endcase
                end
            end
            
            STATE_ADDR: begin
                if (cs_n) begin
                    next_state = STATE_IDLE;
                end else if (spi_clk_posedge && bit_counter == 7 && byte_counter == 2) begin
                    // All 3 address bytes received
                    case (command_reg)
                        CMD_READ_DATA: next_state = STATE_READ_DATA;
                        default: next_state = STATE_IDLE;
                    endcase
                end
            end
            
            STATE_READ_DATA: begin
                if (cs_n) begin
                    next_state = STATE_IDLE;
                end
                // Stay in read state until CS goes high
            end
        endcase
    end
    
    // Bit and Byte Counter Management
    always @(posedge clk_i or negedge rst_n) begin
        if (!rst_n) begin
            bit_counter <= 5'h00;
            byte_counter <= 5'h00;
        end else if (cs_n) begin
            bit_counter <= 5'h00;
            byte_counter <= 5'h00;
        end else if (spi_clk_posedge) begin
            if (bit_counter == 7) begin
                bit_counter <= 5'h00;
                if (state == STATE_ADDR) begin
                    byte_counter <= byte_counter + 1;
                end
            end else begin
                bit_counter <= bit_counter + 1;
            end
        end
    end
    
    // Address Auto-increment for Read Data
    always @(posedge clk_i or negedge rst_n) begin
        if (!rst_n) begin
            // Address reset handled above
        end else if (state == STATE_READ_DATA && spi_clk_negedge && bit_counter == 7) begin
            address_reg <= address_reg + 1;
        end
    end
    
















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
    